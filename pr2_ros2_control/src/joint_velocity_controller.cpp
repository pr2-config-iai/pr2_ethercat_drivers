#include "pr2_ros2_control/joint_velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pluginlib/class_list_macros.hpp>

namespace pr2_ros2_control
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

CallbackReturn JointVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("dof_names", {});
    auto_declare<std::string>("command_interface", "effort");
    auto_declare<std::vector<std::string>>("reference_and_state_interfaces", {"velocity"});
    auto_declare<double>("command_timeout", 0.2);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception while declaring parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration JointVelocityController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & dof : dof_names_) {
    config.names.push_back(dof + "/effort");
  }
  return config;
}

InterfaceConfiguration JointVelocityController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & dof : dof_names_) {
    config.names.push_back(dof + "/velocity");
  }
  return config;
}

CallbackReturn JointVelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  const auto logger = node->get_logger();

  dof_names_ = node->get_parameter("dof_names").as_string_array();
  const std::string command_interface = node->get_parameter("command_interface").as_string();
  const auto state_interfaces = node->get_parameter("reference_and_state_interfaces").as_string_array();
  command_timeout_ = node->get_parameter("command_timeout").as_double();

  if (dof_names_.empty()) {
    RCLCPP_ERROR(logger, "dof_names is empty");
    return CallbackReturn::ERROR;
  }
  if (command_interface != "effort") {
    RCLCPP_ERROR(logger, "command_interface must be 'effort', not '%s'", command_interface.c_str());
    return CallbackReturn::ERROR;
  }
  if (state_interfaces != std::vector<std::string>{"velocity"}) {
    RCLCPP_ERROR(logger, "reference_and_state_interfaces must be [velocity]");
    return CallbackReturn::ERROR;
  }
  if (!(command_timeout_ >= 0.0)) {
    RCLCPP_ERROR(logger, "command_timeout must be >= 0 (0: no timeout), not %f", command_timeout_);
    return CallbackReturn::ERROR;
  }

  pids_.clear();
  for (const auto & dof : dof_names_) {
    // As pid_controller: parameters gains.<dof>.*, no per-joint PID state topic.
    auto pid = std::make_shared<control_toolbox::PidROS>(node, "gains." + dof, "~/" + dof, false);
    if (!pid->initialize_from_ros_parameters()) {
      RCLCPP_ERROR(logger, "No valid gains for %s (gains.%s.*)", dof.c_str(), dof.c_str());
      return CallbackReturn::ERROR;
    }
    pids_.push_back(pid);
  }

  reference_.assign(dof_names_.size(), 0.0);
  received_seq_ = 0;
  input_.writeFromNonRT(Input{std::vector<double>(dof_names_.size(), 0.0), 0});

  // As pid_controller: keep last 1, best effort (reliable publishers are compatible).
  auto qos = rclcpp::SystemDefaultsQoS();
  qos.keep_last(1);
  qos.best_effort();
  reference_sub_ = node->create_subscription<Reference>(
    "~/reference", qos, [this](const Reference::SharedPtr msg) { referenceCallback(msg); });

  state_pub_ = node->create_publisher<State>("~/controller_state", rclcpp::SystemDefaultsQoS());
  state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<State>>(state_pub_);
  state_msg_.dof_states.resize(dof_names_.size());
  for (size_t i = 0; i < dof_names_.size(); ++i) {
    state_msg_.dof_states[i].name = dof_names_[i];
  }

  RCLCPP_INFO(logger, "%zu joints, command timeout %.3f s%s", dof_names_.size(), command_timeout_,
              command_timeout_ > 0.0 ? "" : " (off)");
  return CallbackReturn::SUCCESS;
}

void JointVelocityController::referenceCallback(const Reference::SharedPtr msg)
{
  auto node = get_node();
  const size_t n = dof_names_.size();
  Input input;
  input.values.assign(n, std::numeric_limits<double>::quiet_NaN());

  if (msg->values.size() != n || !(msg->dof_names.empty() || msg->dof_names.size() == n)) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                         "Reference ignored: %zu names and %zu values for %zu joints",
                         msg->dof_names.size(), msg->values.size(), n);
    return;
  }
  for (size_t k = 0; k < n; ++k) {
    size_t i = k;  // without names: in dof_names order, as pid_controller
    if (!msg->dof_names.empty()) {
      const auto it = std::find(dof_names_.begin(), dof_names_.end(), msg->dof_names[k]);
      if (it == dof_names_.end()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                             "Reference ignored: '%s' is not one of this controller's joints",
                             msg->dof_names[k].c_str());
        return;
      }
      i = static_cast<size_t>(it - dof_names_.begin());
    }
    const double v = msg->values[k];
    if (std::isinf(v)) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                           "Reference ignored: infinite value for %s", dof_names_[i].c_str());
      return;
    }
    input.values[i] = v;  // NaN: keep this joint's reference
  }
  input.seq = ++received_seq_;
  input_.writeFromNonRT(input);
}

CallbackReturn JointVelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  // Start at rest: references 0, integrals cleared, anything received before now dropped.
  std::fill(reference_.begin(), reference_.end(), 0.0);
  const Input * input = input_.readFromRT();
  applied_seq_ = input ? input->seq : 0;
  reference_live_ = false;
  for (auto & pid : pids_) {
    pid->reset(false);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & command : command_interfaces_) {
    (void)command.set_value(0.0);
  }
  for (auto & pid : pids_) {
    pid->reset(false);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointVelocityController::update(const rclcpp::Time & time,
                                                                  const rclcpp::Duration & period)
{
  const size_t n = dof_names_.size();

  const Input * input = input_.readFromRT();
  if (input && input->seq != applied_seq_) {
    applied_seq_ = input->seq;
    for (size_t i = 0; i < n; ++i) {
      if (!std::isnan(input->values[i])) {
        reference_[i] = input->values[i];
      }
    }
    last_reference_time_ = time;
    reference_live_ = true;
  }

  if (reference_live_ && command_timeout_ > 0.0 &&
      (time - last_reference_time_).seconds() > command_timeout_) {
    std::fill(reference_.begin(), reference_.end(), 0.0);
    reference_live_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "No reference for %.3f s: holding at zero velocity",
                command_timeout_);
  }

  for (size_t i = 0; i < n; ++i) {
    const double velocity =
      state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
    auto & s = state_msg_.dof_states[i];
    s.reference = reference_[i];
    s.feedback = velocity;
    s.time_step = period.seconds();
    if (!std::isfinite(velocity)) {
      // As pid_controller: no command without a measurement.
      s.error = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    const double error = reference_[i] - velocity;
    const double effort = pids_[i]->compute_command(error, period);
    (void)command_interfaces_[i].set_value(effort);
    s.error = error;
    s.output = effort;
  }

  if (state_publisher_) {
    state_msg_.header.stamp = time;
    state_publisher_->try_publish(state_msg_);
  }
  return controller_interface::return_type::OK;
}

}  // namespace pr2_ros2_control

PLUGINLIB_EXPORT_CLASS(pr2_ros2_control::JointVelocityController, controller_interface::ControllerInterface)
