/*
 * The effort part of ROS 1 pr2_controller_manager::update, with the commanding gates of decision
 * D20. Separate from the hardware component so it can be tested without EtherCAT.
 */
#ifndef PR2_ROS2_CONTROL__EFFORT_CHAIN_HPP_
#define PR2_ROS2_CONTROL__EFFORT_CHAIN_HPP_

#include <set>
#include <vector>

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>

namespace pr2_ros2_control
{

/*
 * Called after the efforts were zeroed (RobotState::zeroCommands) and the commands of the
 * commanded joints were set. Like ROS 1: enforceSafety(), then the transmissions. In addition:
 * commanded joints are clamped to max_effort_fraction of their URDF effort limit, every other joint
 * gets exactly zero (also where enforceSafety asked for an effort), and every actuator outside
 * commanded_actuators is disabled with zero effort.
 *
 * joint_commanded has one entry per state.joint_states_ element.
 */
void finishEffortCommands(pr2_mechanism_model::RobotState &state, const std::vector<char> &joint_commanded,
                          double max_effort_fraction,
                          const std::set<const pr2_hardware_interface::Actuator *> &commanded_actuators,
                          const pr2_hardware_interface::ActuatorMap &actuators);

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__EFFORT_CHAIN_HPP_
