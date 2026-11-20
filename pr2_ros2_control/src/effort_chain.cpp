#include "pr2_ros2_control/effort_chain.hpp"

#include <algorithm>

namespace pr2_ros2_control
{

void finishEffortCommands(pr2_mechanism_model::RobotState &state, const std::vector<char> &joint_commanded,
                          double max_effort_fraction,
                          const std::set<const pr2_hardware_interface::Actuator *> &commanded_actuators,
                          const pr2_hardware_interface::ActuatorMap &actuators)
{
  state.enforceSafety();
  for (size_t j = 0; j < state.joint_states_.size(); ++j)
  {
    auto &js = state.joint_states_[j];
    if (j < joint_commanded.size() && joint_commanded[j])
    {
      const double limit = (js.joint_ && js.joint_->limits) ? js.joint_->limits->effort : 0.0;
      const double max = max_effort_fraction * limit;
      js.commanded_effort_ = std::clamp(js.commanded_effort_, -max, max);
    }
    else
    {
      js.commanded_effort_ = 0.0;
    }
  }
  state.propagateJointEffortToActuatorEffort();
  for (const auto &kv : actuators)
  {
    if (!commanded_actuators.count(kv.second))
    {
      kv.second->command_.enable_ = false;
      kv.second->command_.effort_ = 0.0;
    }
  }
}

}  // namespace pr2_ros2_control
