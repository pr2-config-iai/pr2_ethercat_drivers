/*
 * ros2_control SystemInterface for the PR2's EtherCAT motor chain.
 *
 * Wraps the ROS 1 drivers the same way ROS 1 pr2_ethercat + pr2_controller_manager did:
 *   - ethercat_hardware talks to the motor boards (one EtherCAT exchange per read()),
 *   - pr2_mechanism_model's PR2 transmissions turn actuator positions into joint positions,
 *     including the wrist differential and the gripper linkage (which publishes the passive
 *     finger joints too).
 *
 * Read-only by default: the motors are halted on every cycle and the component reports an error if
 * any actuator ever reports being enabled. With commanding_enabled, joints in commanded_joints take
 * effort commands, processed as ROS 1 pr2_controller_manager did (zero all efforts, apply the
 * commands, RobotState::enforceSafety, propagate through the transmissions); every other joint gets
 * zero effort and the actuators of other transmissions stay disabled (decision D20). Motors are
 * halted, as ROS 1 pr2_ethercat did, on ~/halt_motors and while the loop runs below
 * min_acceptable_rt_loop_frequency, until ~/reset_motors; they also start halted until the first
 * reset.
 *
 * Velocity: the actuator velocity is the encoder difference over the last velocity_window samples
 * (default 10), timed with the motor boards' sample timestamps, instead of ROS 1's single sample whose
 * steps are 0.04 to 0.16 rad/s at the joints (decision D22); velocity_window 1 gives ROS 1's value.
 *
 * Calibration: the zero offsets that ROS 1's calibration controllers stored on the motor boards
 * are read back at startup (like ROS 1 does), and each calibrated joint's reference position is
 * set from its URDF <calibration> edge, as ROS 1's calibration controllers do when they start.
 * Joints whose transmission needs calibration but has no stored offset are reported when the
 * component is activated.
 */
#ifndef PR2_ROS2_CONTROL__PR2_ETHERCAT_SYSTEM_HPP_
#define PR2_ROS2_CONTROL__PR2_ETHERCAT_SYSTEM_HPP_

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ethercat_hardware/interface_lock.h"
#include "pr2_ros2_control/velocity_window.hpp"

class EthercatHardware;

namespace pr2_hardware_interface
{
class Actuator;
}  // namespace pr2_hardware_interface

namespace pr2_mechanism_model
{
class Robot;
class RobotState;
class JointState;
}  // namespace pr2_mechanism_model

namespace pr2_ros2_control
{

class Pr2EthercatSystem : public hardware_interface::SystemInterface
{
public:
  Pr2EthercatSystem();
  ~Pr2EthercatSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  // Controllers may only claim the effort interfaces of commanded joints; released interfaces are
  // no longer applied (a stopped controller leaves its last value in the interface).
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                              const std::vector<std::string> &stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &start_interfaces,
                                                              const std::vector<std::string> &stop_interfaces) override;

private:
  struct JointHandles
  {
    pr2_mechanism_model::JointState *joint = nullptr;
    hardware_interface::StateInterface::SharedPtr position, velocity, effort;
    hardware_interface::CommandInterface::SharedPtr effort_command;
    bool commanded = false;          // listed in commanded_joints
    bool claimed = false;            // its effort interface is claimed by a controller
    bool needs_calibration = false;  // its transmission is calibrated by ROS 1 (see reportCalibration)
  };

  bool fetchRobotDescription(std::string &urdf) const;
  bool configureCommanding();
  bool loopTooSlow();
  void shutdownHardware();
  void reportCalibration();

  // parameters
  std::string interface_ = "ecat0";
  bool allow_unprogrammed_ = false;
  std::string robot_description_topic_ = "/robot_description";
  double robot_description_timeout_ = 10.0;
  bool commanding_enabled_ = false;
  std::set<std::string> commanded_joints_;
  double max_effort_fraction_ = 0.1;
  double min_acceptable_rt_loop_frequency_ = 750.0;
  int velocity_window_ = 10;  // encoder samples the actuator velocity is taken over (1 = ROS 1)

  std::string urdf_;
  ethercat_hardware::InterfaceLock lock_;
  std::unique_ptr<EthercatHardware> ec_;
  std::unique_ptr<pr2_mechanism_model::Robot> robot_;
  std::unique_ptr<pr2_mechanism_model::RobotState> state_;
  std::vector<JointHandles> handles_;  // in the order of info_.joints
  std::map<std::string, size_t> effort_command_index_;  // "<joint>/effort" -> handles_ index
  bool ethercat_initialized_once_ = false;
  // Non-realtime thread that calls EthercatHardware::collectDiagnostics() once a second while
  // active, like ROS 1 pr2_ethercat's diagnosticLoop (motor board mailbox diagnostics, device count).
  void startDiagnosticsThread();
  void stopDiagnosticsThread();
  std::thread diagnostics_thread_;
  std::mutex diagnostics_mutex_;
  std::condition_variable diagnostics_cv_;
  bool diagnostics_stop_ = false;

  bool fault_reported_ = false;
  bool reset_on_next_read_ = false;  // one motor reset after EtherCAT comes up (see on_configure)
  // ~/reset_motors and ~/halt_motors (std_srvs/Empty, as ROS 1 pr2_ethercat; the dashboard calls
  // them). Still read-only: a reset clears the boards' latched faults, halt stays set.
  std::atomic<bool> reset_requested_{false};
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_motors_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr halt_motors_service_;
  std::atomic<bool> halt_requested_{false};
  bool released_by_reset_ = false;  // commanding: motors stay halted until the first reset request

  // Actuators of the transmissions that drive a commanded joint; all others must stay disabled.
  std::set<const pr2_hardware_interface::Actuator *> commanded_actuators_;
  std::vector<char> joint_state_commanded_;  // per RobotState::joint_states_ entry

  // Actuator velocity over velocity_window_ encoder samples, before the transmissions (D22).
  std::vector<pr2_hardware_interface::Actuator *> windowed_actuators_;
  std::vector<VelocityWindow> velocity_windows_;

  // ROS 1 pr2_ethercat's loop frequency monitor: cycles counted over 200 ms, averaged over the last
  // three counts; below the minimum the motors are halted (until a reset).
  bool monitor_started_ = false;
  double monitor_last_time_ = 0.0;
  unsigned monitor_cycles_ = 0;
  double monitor_history_[3] = {1000.0, 1000.0, 1000.0};
  unsigned monitor_index_ = 0;
  bool loop_too_slow_ = false;
  std::set<std::string> gripper_transmissions_;  // names, from the URDF
};

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__PR2_ETHERCAT_SYSTEM_HPP_
