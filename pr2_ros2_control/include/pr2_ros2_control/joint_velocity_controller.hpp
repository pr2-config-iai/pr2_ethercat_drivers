// Velocity PI(D) for a group of joints, with the effort command interface: in place of
// pid_controller for the arms, torso and head (D40).
//
// Same interface as the pid_controller configuration it replaces:
//  - parameters dof_names, command_interface (effort), reference_and_state_interfaces ([velocity])
//    and gains.<dof>.{p, i, d, u_clamp_*, i_clamp_*, antiwindup_strategy, ...} (control_toolbox's
//    PidROS, so the same PID and the same runtime gain changes);
//  - references (control_msgs/MultiDOFCommand) on ~/reference, NaN keeps a joint's reference;
//  - control_msgs/MultiDOFStateStamped on ~/controller_state.
//
// Differences:
//  - On activation (also the restart before a motor release, D26) every reference is 0 and the
//    integrals are cleared; references received before the activation are dropped.
//    pid_controller took the measured velocity as the first reference.
//  - command_timeout (s, 0 = off): when no reference has arrived for that long, the references go
//    to 0; the PID keeps holding the joints at zero velocity. pid_controller kept the last one.
//  - On deactivation the efforts are set to 0.
#ifndef PR2_ROS2_CONTROL__JOINT_VELOCITY_CONTROLLER_HPP_
#define PR2_ROS2_CONTROL__JOINT_VELOCITY_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/multi_dof_command.hpp>
#include <control_msgs/msg/multi_dof_state_stamped.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace pr2_ros2_control
{

class JointVelocityController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Reference = control_msgs::msg::MultiDOFCommand;
  using State = control_msgs::msg::MultiDOFStateStamped;

  // A reference in dof_names order, with a sequence number so the realtime loop sees each once.
  struct Input
  {
    std::vector<double> values;
    std::uint64_t seq = 0;
  };

  void referenceCallback(const Reference::SharedPtr msg);

  std::vector<std::string> dof_names_;
  double command_timeout_ = 0.2;
  std::vector<std::shared_ptr<control_toolbox::PidROS>> pids_;

  rclcpp::Subscription<Reference>::SharedPtr reference_sub_;
  realtime_tools::RealtimeBuffer<Input> input_;
  std::uint64_t received_seq_ = 0;  // subscription thread only

  // Realtime loop only.
  std::vector<double> reference_;
  std::uint64_t applied_seq_ = 0;
  bool reference_live_ = false;  // a reference arrived since activation and has not timed out
  rclcpp::Time last_reference_time_;

  rclcpp::Publisher<State>::SharedPtr state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<State>> state_publisher_;
  State state_msg_;
};

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__JOINT_VELOCITY_CONTROLLER_HPP_
