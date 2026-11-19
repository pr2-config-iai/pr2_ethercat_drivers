/*
 * Small helpers that keep the ROS 2 port of ethercat_hardware close to the ROS 1 code.
 *
 * - logger(): one shared logger for the whole package (ROS 1 used the global rosconsole).
 * - now():    one clock for every timestamp in the package. rclcpp::Time objects with
 *             different clock types throw when subtracted or compared, which must never
 *             happen inside the realtime loop, so everything uses RCL_SYSTEM_TIME (the
 *             default clock type of a default-constructed rclcpp::Time).
 * - getParam(): ROS 1 NodeHandle::getParam semantics ("false if not set") that work no
 *             matter which NodeOptions the node was created with. ROS 1 hierarchical names
 *             map to dotted ROS 2 names, e.g. "~/l_gripper_motor/enable_ft_sensor" becomes
 *             "l_gripper_motor.enable_ft_sensor".
 */
#ifndef ETHERCAT_HARDWARE__ROS_HELPERS_H
#define ETHERCAT_HARDWARE__ROS_HELPERS_H

#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ethercat_hardware
{

inline rclcpp::Logger logger()
{
  static rclcpp::Logger logger = rclcpp::get_logger("ethercat_hardware");
  return logger;
}

inline rclcpp::Time now()
{
  static rclcpp::Clock clock(RCL_SYSTEM_TIME);
  return clock.now();
}

/// Get a parameter, declaring it on first use. Returns false if it is not set or has the wrong type.
template <typename T>
bool getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, T &value)
{
  if (!node)
  {
    return false;
  }
  if (!node->has_parameter(name))
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    try
    {
      node->declare_parameter(name, rclcpp::ParameterValue{}, descriptor);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
    {
    }
  }

  rclcpp::Parameter param;
  if (!node->get_parameter(name, param) || param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return false;
  }

  try
  {
    if constexpr (std::is_same_v<T, double>)
    {
      // ROS 1 getParam accepted integers for doubles (e.g. "timeout: 5"); keep that.
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        value = static_cast<double>(param.as_int());
        return true;
      }
    }
    if constexpr (std::is_same_v<T, std::vector<double>>)
    {
      // YAML turns "[0, 0, 0]" into an integer array; accept it like ROS 1 did.
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
      {
        const auto ints = param.as_integer_array();
        value.assign(ints.begin(), ints.end());
        return true;
      }
    }
    value = param.get_value<T>();
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException &e)
  {
    RCLCPP_ERROR(logger(), "Parameter '%s' has the wrong type: %s", name.c_str(), e.what());
    return false;
  }
  return true;
}

}  // namespace ethercat_hardware

#endif  // ETHERCAT_HARDWARE__ROS_HELPERS_H
