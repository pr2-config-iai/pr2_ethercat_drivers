/*
 * ros2_control SystemInterface for the PR2's EtherCAT motor chain.
 *
 * Wraps the ROS 1 drivers the same way ROS 1 pr2_ethercat + pr2_controller_manager did:
 *   - ethercat_hardware talks to the motor boards (one EtherCAT exchange per read()),
 *   - pr2_mechanism_model's PR2 transmissions turn actuator positions into joint positions,
 *     including the wrist differential and the gripper linkage (which publishes the passive
 *     finger joints too).
 *
 * Milestone 1 (this version) is read-only: the motors are halted on every cycle, only state
 * interfaces (position, velocity, effort) are exported, and the component reports an error if any
 * actuator ever reports being enabled.
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

class EthercatHardware;

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

private:
  struct JointHandles
  {
    pr2_mechanism_model::JointState *joint = nullptr;
    hardware_interface::StateInterface::SharedPtr position, velocity, effort;
  };

  bool fetchRobotDescription(std::string &urdf) const;
  void shutdownHardware();
  void reportCalibration();

  // parameters
  std::string interface_ = "ecat0";
  bool allow_unprogrammed_ = false;
  std::string robot_description_topic_ = "/robot_description";
  double robot_description_timeout_ = 10.0;

  std::string urdf_;
  ethercat_hardware::InterfaceLock lock_;
  std::unique_ptr<EthercatHardware> ec_;
  std::unique_ptr<pr2_mechanism_model::Robot> robot_;
  std::unique_ptr<pr2_mechanism_model::RobotState> state_;
  std::vector<JointHandles> handles_;  // in the order of info_.joints
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
  std::set<std::string> gripper_transmissions_;  // names, from the URDF
};

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__PR2_ETHERCAT_SYSTEM_HPP_
