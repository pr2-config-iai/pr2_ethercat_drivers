#include "pr2_ros2_control/motor_release_gate.hpp"

namespace pr2_ros2_control
{

MotorReleaseGate &MotorReleaseGate::instance()
{
  static MotorReleaseGate gate;
  return gate;
}

void MotorReleaseGate::attach() { attached_ = true; }

void MotorReleaseGate::detach()
{
  attached_ = false;
  // A component still waiting would keep the motors halted forever; let it see a failure.
  int expected = static_cast<int>(State::REQUESTED);
  state_.compare_exchange_strong(expected, static_cast<int>(State::FAILED));
}

bool MotorReleaseGate::waitForRequest(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return cv_.wait_for(lock, timeout, [this] { return state_.load() == static_cast<int>(State::REQUESTED); });
}

void MotorReleaseGate::reportRestart(bool success)
{
  int expected = static_cast<int>(State::REQUESTED);
  state_.compare_exchange_strong(expected, static_cast<int>(success ? State::RESTARTED : State::FAILED));
}

bool MotorReleaseGate::requestRelease()
{
  if (!attached_)
  {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = static_cast<int>(State::REQUESTED);
  }
  cv_.notify_all();
  return true;
}

MotorReleaseGate::State MotorReleaseGate::takeResult()
{
  int s = state_.load();
  if (s == static_cast<int>(State::RESTARTED) || s == static_cast<int>(State::FAILED))
  {
    // Only consume what was read: a new request in between stays pending.
    state_.compare_exchange_strong(s, static_cast<int>(State::IDLE));
  }
  return static_cast<State>(s);
}

void MotorReleaseGate::cancel()
{
  state_ = static_cast<int>(State::IDLE);
}

}  // namespace pr2_ros2_control
