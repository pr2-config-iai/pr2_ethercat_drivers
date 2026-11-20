#include "ethercat_hardware/interface_lock.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ethercat_hardware
{

InterfaceLock::Result InterfaceLock::acquire(const std::string &interface)
{
  release();
  const std::string dir = (access("/run/host/var/tmp", W_OK) == 0) ? "/run/host/var/tmp/run/" : "/var/tmp/run/";
  mkdir(dir.c_str(), 0777);
  path_ = dir + "EtherCAT_" + interface + ".pid";

  int fd = open(path_.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
  if (fd < 0)
  {
    error_ = "cannot open " + path_ + ": " + strerror(errno);
    return Result::ERROR;
  }
  struct flock lock;
  std::memset(&lock, 0, sizeof(lock));
  lock.l_type = F_WRLCK;
  lock.l_whence = SEEK_SET;
  if (fcntl(fd, F_SETLK, &lock) == -1)
  {
    close(fd);
    error_ = "another EtherCAT master holds " + interface + " (lock " + path_ + "). Is the ROS 1 robot running?";
    return Result::HELD_BY_OTHER;
  }
  fchmod(fd, 0666);
  if (ftruncate(fd, 0) == 0)
  {
    dprintf(fd, "%d\n", getpid());
  }
  fd_ = fd;
  error_.clear();
  return Result::ACQUIRED;
}

void InterfaceLock::release()
{
  if (fd_ >= 0)
  {
    unlink(path_.c_str());
    close(fd_);
    fd_ = -1;
  }
}

}  // namespace ethercat_hardware
