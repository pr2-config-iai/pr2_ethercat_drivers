/*
 * Realtime controller manager node for the PR2.
 *
 * Same job as ros2_control_node, but as an executable of our own so it can be given the file
 * capabilities the EtherCAT loop needs (raw socket, mlockall, SCHED_FIFO) and an absolute RPATH:
 * a binary with file capabilities runs in secure mode and ignores LD_LIBRARY_PATH.
 *
 * Like ROS 1 pr2_ethercat, the control thread runs read/update/write at SCHED_FIFO maximum
 * priority by default (parameter thread_priority on the controller manager), timed on
 * CLOCK_MONOTONIC.
 */
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_helpers.hpp>

namespace
{
constexpr long NSEC_PER_SEC = 1000000000L;

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
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, "controller_manager");
  const auto logger = cm->get_logger();

  const int thread_priority = cm->get_parameter_or<int>("thread_priority", sched_get_priority_max(SCHED_FIFO));
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", true);
  if (lock_memory)
  {
    const auto [ok, message] = realtime_tools::lock_memory();
    if (!ok)
    {
      RCLCPP_WARN(logger, "Unable to lock memory: %s", message.c_str());
    }
  }

  std::thread control_thread([cm, thread_priority, logger]() {
    if (!realtime_tools::configure_sched_fifo(thread_priority))
    {
      RCLCPP_WARN(logger, "Could not get SCHED_FIFO priority %d (are the capabilities set on this binary?). "
                          "Running without realtime priority.", thread_priority);
    }
    else
    {
      RCLCPP_INFO(logger, "Control thread running at SCHED_FIFO priority %d.", thread_priority);
    }

    const long period_ns = NSEC_PER_SEC / static_cast<long>(cm->get_update_rate());
    RCLCPP_INFO(logger, "Update rate: %u Hz.", cm->get_update_rate());
    timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    rclcpp::Time previous_time = cm->get_trigger_clock()->now();

    while (rclcpp::ok())
    {
      const rclcpp::Time current_time = cm->get_trigger_clock()->now();
      const rclcpp::Duration measured_period = current_time - previous_time;
      previous_time = current_time;

      cm->read(current_time, measured_period);
      cm->update(current_time, measured_period);
      cm->write(current_time, measured_period);

      addNs(next, period_ns);
      timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (before(next, now))
      {
        next = now;  // overrun: don't try to catch up with a burst of cycles
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
  });

  executor->add_node(cm);
  executor->spin();
  control_thread.join();
  rclcpp::shutdown();
  return 0;
}
