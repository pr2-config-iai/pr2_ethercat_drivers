/*
 * Realtime controller manager node for the PR2.
 *
 * Same job as ros2_control_node, but our own executable: it is the one process that gets the
 * capabilities the EtherCAT loop needs (raw socket, mlockall, SCHED_FIFO), granted at start by
 * pr2-rt (tools/pr2-rt-exec.c, decision D11) rather than to a system binary.
 *
 * Like ROS 1 pr2_ethercat, the control thread runs read/update/write at SCHED_FIFO maximum
 * priority by default (parameter thread_priority on the controller manager), timed on
 * CLOCK_MONOTONIC, and publishes the "Realtime Control Loop" diagnostics of ROS 1 (loop, EtherCAT
 * and controller times, jitter, overruns, loop frequency) once a second.
 */
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <controller_manager/controller_manager.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace
{
constexpr long NSEC_PER_SEC = 1000000000L;
constexpr double USEC_PER_SEC = 1e6;

void addNs(timespec &t, long ns)
{
  t.tv_nsec += ns;
  while (t.tv_nsec >= NSEC_PER_SEC)
  {
    t.tv_nsec -= NSEC_PER_SEC;
    ++t.tv_sec;
  }
}

bool before(const timespec &a, const timespec &b)
{
  return a.tv_sec < b.tv_sec || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec);
}

double seconds(const timespec &t)
{
  return double(t.tv_sec) + double(t.tv_nsec) / NSEC_PER_SEC;
}

double monotonicNow()
{
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return seconds(t);
}

// Mean and max of the samples since the last reset (boost accumulators in ROS 1).
struct Accumulator
{
  double sum = 0.0;
  double max = 0.0;
  unsigned long count = 0;

  void operator()(double v)
  {
    sum += v;
    max = count ? std::max(max, v) : v;
    ++count;
  }
  double mean() const { return count ? sum / double(count) : 0.0; }
};

// ROS 1 pr2_ethercat's g_stats and RTLoopHistory, used only by the control thread.
struct LoopStats
{
  Accumulator ec_acc, cm_acc, loop_acc, jitter_acc;  // reset at every publish
  double max_ec = 0, max_cm = 0, max_loop = 0, max_jitter = 0;  // since start
  int overruns = 0;
  int recent_overruns = 0;
  int last_overrun = 1000;         // publishes (seconds) since the last overrun
  int last_severe_overrun = 1000;  // ... since a second with more than 10 overruns
  double overrun_loop_sec = 0, overrun_ec = 0, overrun_cm = 0;

  // Loop frequency: cycles counted over 200 ms, averaged over the last 3 counts.
  static constexpr double monitor_period = 0.6 / 3;
  double history[3] = {1000.0, 1000.0, 1000.0};
  unsigned history_index = 0;
  unsigned cycle_count = 0;
  double last_monitor_time = 0;
  double rt_loop_frequency = 1000.0;
};

void fillDiagnostics(LoopStats &s, double min_acceptable_frequency, diagnostic_msgs::msg::DiagnosticArray &msg,
                     const rclcpp::Time &stamp)
{
  s.max_ec = std::max(s.max_ec, s.ec_acc.max);
  s.max_cm = std::max(s.max_cm, s.cm_acc.max);
  s.max_loop = std::max(s.max_loop, s.loop_acc.max);
  s.max_jitter = std::max(s.max_jitter, s.jitter_acc.max);

  diagnostic_updater::DiagnosticStatusWrapper status;
  status.addf("Max EtherCAT roundtrip (us)", "%.2f", s.max_ec * USEC_PER_SEC);
  status.addf("Avg EtherCAT roundtrip (us)", "%.2f", s.ec_acc.mean() * USEC_PER_SEC);
  status.addf("Max Controller Manager roundtrip (us)", "%.2f", s.max_cm * USEC_PER_SEC);
  status.addf("Avg Controller Manager roundtrip (us)", "%.2f", s.cm_acc.mean() * USEC_PER_SEC);
  status.addf("Max Total Loop roundtrip (us)", "%.2f", s.max_loop * USEC_PER_SEC);
  status.addf("Avg Total Loop roundtrip (us)", "%.2f", s.loop_acc.mean() * USEC_PER_SEC);
  status.addf("Max Loop Jitter (us)", "%.2f", s.max_jitter * USEC_PER_SEC);
  status.addf("Avg Loop Jitter (us)", "%.2f", s.jitter_acc.mean() * USEC_PER_SEC);
  status.addf("Control Loop Overruns", "%d", s.overruns);
  status.addf("Recent Control Loop Overruns", "%d", s.recent_overruns);
  status.addf("Last Control Loop Overrun Cause", "ec: %.2fus, cm: %.2fus", s.overrun_ec * USEC_PER_SEC,
              s.overrun_cm * USEC_PER_SEC);
  status.addf("Last Overrun Loop Time (us)", "%.2f", s.overrun_loop_sec * USEC_PER_SEC);
  status.addf("Realtime Loop Frequency", "%.4f", s.rt_loop_frequency);

  status.name = "Realtime Control Loop";
  status.hardware_id = "";
  if (s.overruns > 0 && s.last_overrun < 30)
  {
    status.level = s.last_severe_overrun < 30 ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                              : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Realtime loop used too much time in the last 30 seconds.";
  }
  else
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  }
  // ROS 1 halts the motors here and keeps this error until the next motor reset. That needs the
  // hardware component (it does the halting); until commanding is added the motors are halted on
  // every cycle anyway, so this only reports.
  if (s.rt_loop_frequency < min_acceptable_frequency)
  {
    status.mergeSummaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Realtime loop only ran at %.4f Hz",
                         s.rt_loop_frequency);
  }

  s.recent_overruns = 0;
  ++s.last_overrun;
  ++s.last_severe_overrun;
  s.ec_acc = s.cm_acc = s.loop_acc = s.jitter_acc = Accumulator();

  msg.header.stamp = stamp;
  msg.status.assign(1, status);
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");
  const auto logger = cm->get_logger();

  const int thread_priority = cm->get_parameter_or<int>("thread_priority", sched_get_priority_max(SCHED_FIFO));
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", true);
  const double min_acceptable_rt_loop_frequency =
    cm->declare_parameter<double>("min_acceptable_rt_loop_frequency", 750.0);
  if (lock_memory)
  {
    const auto [ok, message] = realtime_tools::lock_memory();
    if (!ok)
    {
      RCLCPP_WARN(logger, "Unable to lock memory: %s", message.c_str());
    }
  }

  auto diagnostics_publisher = std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
    cm->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 2));

  std::thread control_thread([cm, thread_priority, logger, diagnostics_publisher, min_acceptable_rt_loop_frequency]() {
    if (!realtime_tools::configure_sched_fifo(thread_priority))
    {
      RCLCPP_WARN(logger, "Could not get SCHED_FIFO priority %d (was the node started through pr2-rt?). "
                          "Running without realtime priority.", thread_priority);
    }
    else
    {
      RCLCPP_INFO(logger, "Control thread running at SCHED_FIFO priority %d.", thread_priority);
    }

    const long period_ns = NSEC_PER_SEC / static_cast<long>(cm->get_update_rate());
    const rclcpp::Duration nominal_period(0, static_cast<uint32_t>(period_ns));
    RCLCPP_INFO(logger, "Update rate: %u Hz.", cm->get_update_rate());

    LoopStats stats;
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    // The first cycle gets the nominal period: a measured one would be the ~1 us since this line,
    // which the controller manager's periodicity statistics keep for the whole run.
    rclcpp::Time previous_time = cm->get_trigger_clock()->now() - nominal_period;
    double last_loop_start = monotonicNow() - double(period_ns) / NSEC_PER_SEC;
    double last_published = monotonicNow();
    stats.last_monitor_time = last_published;

    while (rclcpp::ok())
    {
      const double start = monotonicNow();
      stats.loop_acc(start - last_loop_start);
      last_loop_start = start;

      const rclcpp::Time current_time = cm->get_trigger_clock()->now();
      const rclcpp::Duration measured_period = current_time - previous_time;
      previous_time = current_time;

      cm->read(current_time, measured_period);
      const double after_ec = monotonicNow();
      cm->update(current_time, measured_period);
      cm->write(current_time, measured_period);
      const double end = monotonicNow();
      stats.ec_acc(after_ec - start);
      stats.cm_acc(end - after_ec);

      if (end - last_published > 1.0 && diagnostics_publisher->trylock())
      {
        fillDiagnostics(stats, min_acceptable_rt_loop_frequency, diagnostics_publisher->msg_,
                        cm->get_trigger_clock()->now());
        diagnostics_publisher->unlockAndPublish();
        last_published = end;
      }

      ++stats.cycle_count;
      if (start - stats.last_monitor_time > LoopStats::monitor_period)
      {
        stats.history_index = (stats.history_index + 1) % 3;
        stats.history[stats.history_index] = double(stats.cycle_count) / LoopStats::monitor_period;
        stats.rt_loop_frequency = (stats.history[0] + stats.history[1] + stats.history[2]) / 3.0;
        stats.cycle_count = 0;
        stats.last_monitor_time = start;
      }

      addNs(next, period_ns);
      timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (before(next, now))
      {
        stats.overrun_loop_sec = seconds(now) - seconds(next);
        if (stats.recent_overruns > 10)
        {
          stats.last_severe_overrun = 0;
        }
        stats.last_overrun = 0;
        ++stats.overruns;
        ++stats.recent_overruns;
        stats.overrun_ec = after_ec - start;
        stats.overrun_cm = end - after_ec;
        next = now;  // overrun: don't try to catch up with a burst of cycles
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);

      timespec after;
      clock_gettime(CLOCK_MONOTONIC, &after);
      stats.jitter_acc(seconds(after) - seconds(next));
    }
  });

  executor->add_node(cm);
  executor->spin();
  control_thread.join();
  rclcpp::shutdown();
  return 0;
}
