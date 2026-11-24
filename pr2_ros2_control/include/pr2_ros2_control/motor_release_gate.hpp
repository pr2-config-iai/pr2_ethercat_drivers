/*
 * Restart the controllers before halted motors are released (decision D26).
 *
 * ROS 1 pr2_controller_manager restarted all running controllers in the realtime loop when the
 * motors came back from a halt, so controllers started fresh (integrals and setpoints reset) with
 * the motors. ros2_control's controller manager has no such hook, and a restart is a controller
 * switch that it performs in its loop on request. So:
 *
 *   - the hardware component, asked to reset halted motors, requests a release here instead of
 *     resetting right away;
 *   - pr2_control_node (attached here) restarts the active controllers that claim command
 *     interfaces and reports back;
 *   - the component then resets the motors, or keeps them halted if the restart failed.
 *
 * One process-wide instance in its own library, linked by both the component plugin and the node.
 * The component reads it in its realtime read() with atomics only.
 */
#ifndef PR2_ROS2_CONTROL__MOTOR_RELEASE_GATE_HPP_
#define PR2_ROS2_CONTROL__MOTOR_RELEASE_GATE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

namespace pr2_ros2_control
{

class MotorReleaseGate
{
public:
  enum class State : int
  {
    IDLE,       // nothing pending
    REQUESTED,  // the component waits for the controller restart
    RESTARTED,  // controllers restarted: the component may reset the motors
    FAILED,     // restart failed: the motors stay halted
  };

  static MotorReleaseGate &instance();

  // Controller side (pr2_control_node).
  void attach();
  void detach();
  // Waits up to timeout for a request; true if one is pending (state REQUESTED).
  bool waitForRequest(std::chrono::milliseconds timeout);
  void reportRestart(bool success);

  // Component side.
  bool attached() const { return attached_.load(); }
  // Non-realtime. False (and nothing requested) when no controller side is attached.
  bool requestRelease();
  // Realtime: the current state; RESTARTED and FAILED are consumed (back to IDLE).
  State takeResult();
  // Realtime: drop a pending request (e.g. the component is deactivated).
  void cancel();

  MotorReleaseGate(const MotorReleaseGate &) = delete;
  MotorReleaseGate &operator=(const MotorReleaseGate &) = delete;

private:
  MotorReleaseGate() = default;

  std::atomic<bool> attached_{false};
  std::atomic<int> state_{static_cast<int>(State::IDLE)};
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__MOTOR_RELEASE_GATE_HPP_
