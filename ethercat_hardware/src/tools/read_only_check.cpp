/*
 * Read-only bring-up check for the PR2 EtherCAT chain under ROS 2.
 *
 * Initializes all EtherCAT devices exactly like the realtime loop does, then runs the normal
 * update cycle at 1 kHz with the motors permanently halted (update(reset=false, halt=true)),
 * and prints what the boards report: actuator names, encoder counts, positions, calibration
 * switch state, motor voltage.
 *
 * What it deliberately does NOT do:
 *   - enable motors: reset is never requested, halt is requested every cycle, and the loop
 *     aborts if any actuator ever reports being enabled;
 *   - write EEPROM (that only happens through motorconf, which is not built);
 *   - run the diagnostics collector (the only place a calibration offset is written to a
 *     board's RAM).
 * The only device configuration written is the volatile gripper sensor-enable flag, which the
 * ROS 1 realtime loop also sets on every start.
 *
 * Holds the same interface lock as ROS 1 pr2_ethercat (/var/tmp/run/EtherCAT_<if>.pid, fcntl
 * write lock), using the host's copy when running inside distrobox, so it refuses to start
 * while the ROS 1 robot is running and vice versa.
 */

#include <fcntl.h>
#include <getopt.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ethercat_hardware/ethercat_hardware.h"

namespace
{

std::atomic<bool> g_quit{false};
void onSignal(int) { g_quit = true; }

constexpr long NSEC_PER_SEC = 1000000000L;
constexpr long PERIOD_NS = 1000000L;  // 1 kHz, like pr2_ethercat

void timespecAdd(timespec &t, long ns)
{
  t.tv_nsec += ns;
  while (t.tv_nsec >= NSEC_PER_SEC)
  {
    t.tv_nsec -= NSEC_PER_SEC;
    ++t.tv_sec;
  }
}

double secondsBetween(const timespec &a, const timespec &b)
{
  return double(b.tv_sec - a.tv_sec) + double(b.tv_nsec - a.tv_nsec) / NSEC_PER_SEC;
}

/// Take the same lock as ROS 1 pr2_ethercat. Returns the fd (keep it open), -1 on error,
/// -2 if another EtherCAT master holds the interface.
int lockInterface(const std::string &interface, std::string &path)
{
  // Inside distrobox the host's /var/tmp is at /run/host/var/tmp; the ROS 1 robot locks there.
  const std::string dir = (access("/run/host/var/tmp", W_OK) == 0) ? "/run/host/var/tmp/run/" : "/var/tmp/run/";
  mkdir(dir.c_str(), 0777);
  path = dir + "EtherCAT_" + interface + ".pid";

  int fd = open(path.c_str(), O_RDWR | O_CREAT, 0666);
  if (fd < 0)
  {
    return -1;
  }
  struct flock lock;
  std::memset(&lock, 0, sizeof(lock));
  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  if (fcntl(fd, F_SETLK, &lock) == -1)
  {
    close(fd);
    return -2;
  }
  if (ftruncate(fd, 0) == 0)
  {
    dprintf(fd, "%d\n", getpid());
  }
  return fd;
}

void usage(const char *prog)
{
  std::fprintf(stderr,
    "Usage: %s [options] [--ros-args ...]\n"
    "  -i, --interface <if>     EtherCAT interface (default: ecat0)\n"
    "  -d, --duration <sec>     How long to run the halted update loop (default: 5)\n"
    "  -p, --print-rate <hz>    How often to print actuator states (default: 1)\n"
    "  -u, --allow_unprogrammed Allow unprogrammed devices\n"
    "  -h, --help\n", prog);
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> cargs;
  for (const auto &a : args)
  {
    cargs.push_back(const_cast<char *>(a.c_str()));
  }

  std::string interface = "ecat0";
  double duration = 5.0;
  double print_rate = 1.0;
  bool allow_unprogrammed = false;

  static struct option long_options[] = {
    {"interface", required_argument, nullptr, 'i'},
    {"duration", required_argument, nullptr, 'd'},
    {"print-rate", required_argument, nullptr, 'p'},
    {"allow_unprogrammed", no_argument, nullptr, 'u'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, 0, nullptr, 0}};
  int c;
  while ((c = getopt_long(int(cargs.size()), cargs.data(), "i:d:p:uh", long_options, nullptr)) != -1)
  {
    switch (c)
    {
      case 'i': interface = optarg; break;
      case 'd': duration = std::atof(optarg); break;
      case 'p': print_rate = std::max(0.1, std::atof(optarg)); break;
      case 'u': allow_unprogrammed = true; break;
      default: usage(cargs[0]); return c == 'h' ? 0 : 1;
    }
  }

  std::string lock_path;
  const int lock_fd = lockInterface(interface, lock_path);
  if (lock_fd == -2)
  {
    std::fprintf(stderr, "Another EtherCAT master holds %s (lock %s). Is the ROS 1 robot running?\n",
                 interface.c_str(), lock_path.c_str());
    return 1;
  }
  if (lock_fd < 0)
  {
    std::fprintf(stderr, "Cannot create lock file %s: %s\n", lock_path.c_str(), strerror(errno));
    return 1;
  }

  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);

  if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
  {
    std::fprintf(stderr, "WARNING: mlockall failed (%s); timing may be poor\n", strerror(errno));
  }

  // Same node name as the ROS 1 realtime loop, so device parameters keep their place
  // (e.g. realtime_loop: l_gripper_motor.enable_ft_sensor).
  auto node = std::make_shared<rclcpp::Node>("realtime_loop");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 0;
  {
    EthercatHardware ec(node);
    std::printf("Initializing EtherCAT devices on %s ...\n", interface.c_str());
    std::vector<char> iface(interface.begin(), interface.end());
    iface.push_back('\0');
    ec.init(iface.data(), allow_unprogrammed);  // exits the process on failure, like ROS 1

    std::printf("Initialized. %zu actuators:\n", ec.hw_->actuators_.size());
    for (const auto &kv : ec.hw_->actuators_)
    {
      std::printf("  %s\n", kv.first.c_str());
    }

    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    const bool realtime = (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0);
    if (!realtime)
    {
      std::fprintf(stderr, "WARNING: could not get SCHED_FIFO (run setcap on this binary); "
                           "running without realtime priority\n");
    }

    timespec tick, start, now;
    clock_gettime(CLOCK_MONOTONIC, &tick);
    start = tick;
    const long cycles = long(duration * 1000.0);
    const long print_every = std::max(1L, long(1000.0 / print_rate));
    long overruns = 0;
    double max_update_s = 0.0;
    long cycle = 0;

    for (; cycle < cycles && !g_quit; ++cycle)
    {
      timespec before;
      clock_gettime(CLOCK_MONOTONIC, &before);
      ec.update(false /*reset*/, true /*halt*/);
      clock_gettime(CLOCK_MONOTONIC, &now);
      max_update_s = std::max(max_update_s, secondsBetween(before, now));

      for (const auto &kv : ec.hw_->actuators_)
      {
        if (kv.second->state_.is_enabled_)
        {
          std::fprintf(stderr, "FATAL: actuator %s reports enabled in read-only mode, aborting\n", kv.first.c_str());
          exit_code = 2;
          g_quit = true;
        }
      }

      if (cycle % print_every == 0)
      {
        std::printf("\n--- t=%.1fs ---------------------------------------------------------------\n",
                    secondsBetween(start, now));
        std::printf("%-28s %12s %12s %6s %8s %7s\n", "actuator", "encoder", "position", "calib", "V_motor", "halted");
        for (const auto &kv : ec.hw_->actuators_)
        {
          const auto &s = kv.second->state_;
          std::printf("%-28s %12d %12.5f %6s %8.2f %7s\n", kv.first.c_str(), s.encoder_count_, s.position_,
                      s.calibration_reading_ ? "on" : "off", s.motor_voltage_, s.halted_ ? "yes" : "no");
        }
      }

      timespecAdd(tick, PERIOD_NS);
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (secondsBetween(tick, now) > 0)
      {
        ++overruns;
        tick = now;  // snap to now after an overrun
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tick, nullptr);
    }

    ec.update(false, true);
    std::printf("\n=== summary ===\n");
    std::printf("cycles: %ld, overruns: %ld, max update time: %.3f ms, realtime priority: %s\n",
                cycle, overruns, max_update_s * 1e3, realtime ? "yes" : "no");
    ec.printCounters(std::cout);
  }  // ~EthercatHardware puts devices back into PREOP and closes the socket

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  close(lock_fd);
  return exit_code;
}
