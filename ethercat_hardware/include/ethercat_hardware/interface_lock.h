/*
 * Exclusive lock on an EtherCAT interface, compatible with ROS 1 pr2_ethercat.
 *
 * ROS 1 pr2_ethercat writes its pid to /var/tmp/run/EtherCAT_<interface>.pid and holds an fcntl
 * write lock on it while running. This class takes the same lock, so a ROS 1 and a ROS 2 master can
 * never drive the same chain at the same time. Inside distrobox the host's /var/tmp is at
 * /run/host/var/tmp, which is where the ROS 1 robot running on the host locks.
 *
 * Like ROS 1, the pid file is world-writable and removed on release, so whoever starts the robot
 * next (possibly another user) can always open and replace it. A crashed holder loses its fcntl
 * lock automatically; its leftover file is then stale and gets taken over.
 */
#ifndef ETHERCAT_HARDWARE__INTERFACE_LOCK_H
#define ETHERCAT_HARDWARE__INTERFACE_LOCK_H

#include <string>

namespace ethercat_hardware
{

class InterfaceLock
{
public:
  enum class Result { ACQUIRED, HELD_BY_OTHER, ERROR };

  InterfaceLock() = default;
  ~InterfaceLock() { release(); }
  InterfaceLock(const InterfaceLock &) = delete;
  InterfaceLock &operator=(const InterfaceLock &) = delete;

  /// Try to lock \p interface. On failure, error() describes why.
  Result acquire(const std::string &interface);
  /// Remove the pid file and drop the lock (no-op if not held).
  void release();

  bool held() const { return fd_ >= 0; }
  const std::string &path() const { return path_; }
  const std::string &error() const { return error_; }

private:
  int fd_ = -1;
  std::string path_;
  std::string error_;
};

}  // namespace ethercat_hardware

#endif  // ETHERCAT_HARDWARE__INTERFACE_LOCK_H
