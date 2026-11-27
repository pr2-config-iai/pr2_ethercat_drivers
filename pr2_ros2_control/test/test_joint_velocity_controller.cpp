// JointVelocityController (D40) with two joints, p only (effort = p * (reference - velocity)).
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/msg/multi_dof_command.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pr2_ros2_control/joint_velocity_controller.hpp"

using control_msgs::msg::MultiDOFCommand;
using lifecycle_msgs::msg::State;

namespace
{
constexpr double P = 2.0;
const std::vector<std::string> DOFS = {"j1", "j2"};

hardware_interface::InterfaceDescription description(const std::string & joint, const std::string & name)
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.initial_value = "0.0";
  return hardware_interface::InterfaceDescription(joint, info);
}

// rclcpp for every test suite of this file.
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
const auto * const rclcpp_environment = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
}  // namespace

class JointVelocityControllerTest : public ::testing::Test
{
protected:
  void SetUp() override { make(0.2); }

  void make(double timeout)
  {
    controller_ = std::make_shared<pr2_ros2_control::JointVelocityController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "test_velocity_controller";
    params.update_rate = 1000;
    params.controller_manager_update_rate = 1000;
    std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("dof_names", DOFS),
      rclcpp::Parameter("command_interface", "effort"),
      rclcpp::Parameter("reference_and_state_interfaces", std::vector<std::string>{"velocity"}),
      rclcpp::Parameter("command_timeout", timeout)};
    for (const auto & d : DOFS) {
      overrides.emplace_back("gains." + d + ".p", P);
      overrides.emplace_back("gains." + d + ".i", 0.0);
      overrides.emplace_back("gains." + d + ".d", 0.0);
    }
    params.node_options = rclcpp::NodeOptions().parameter_overrides(overrides);
    ASSERT_EQ(controller_->init(params), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

    states_.clear();
    commands_.clear();
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (const auto & d : DOFS) {
      states_.push_back(std::make_shared<hardware_interface::StateInterface>(description(d, "velocity")));
      commands_.push_back(std::make_shared<hardware_interface::CommandInterface>(description(d, "effort")));
      loaned_states.emplace_back(states_.back());
      loaned_commands.emplace_back(commands_.back(), nullptr);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    pub_node_ = std::make_shared<rclcpp::Node>("test_reference_publisher");
    pub_ = pub_node_->create_publisher<MultiDOFCommand>("/test_velocity_controller/reference", 10);
    executor_->add_node(pub_node_);
    // Let discovery match the publisher and the subscription.
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < end) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(pub_->get_subscription_count(), 0u);
  }

  void TearDown() override
  {
    executor_->remove_node(pub_node_);
    executor_->remove_node(controller_->get_node()->get_node_base_interface());
  }

  void activate() { ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE); }

  void send(const std::vector<std::string> & names, const std::vector<double> & values)
  {
    MultiDOFCommand m;
    m.dof_names = names;
    m.values = values;
    pub_->publish(m);
    // Spin until the controller has it (or a short while for messages it ignores).
    for (int k = 0; k < 20; ++k) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void velocities(double v1, double v2)
  {
    ASSERT_TRUE(states_[0]->set_value(v1));
    ASSERT_TRUE(states_[1]->set_value(v2));
  }

  void update(double t)
  {
    ASSERT_EQ(controller_->update(rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME),
                                  rclcpp::Duration::from_seconds(0.001)),
              controller_interface::return_type::OK);
  }

  double effort(size_t i) { return commands_[i]->get_optional().value_or(std::nan("")); }

  std::shared_ptr<pr2_ros2_control::JointVelocityController> controller_;
  std::vector<hardware_interface::StateInterface::SharedPtr> states_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Publisher<MultiDOFCommand>::SharedPtr pub_;
};

TEST_F(JointVelocityControllerTest, StartsAtZeroVelocityAndDropsEarlierReferences)
{
  send(DOFS, {1.0, -1.0});  // before activation: dropped
  activate();
  velocities(0.5, -0.25);
  update(100.0);
  EXPECT_DOUBLE_EQ(effort(0), P * (0.0 - 0.5));
  EXPECT_DOUBLE_EQ(effort(1), P * (0.0 + 0.25));
}

TEST_F(JointVelocityControllerTest, FollowsReferencesByNameAndInOrder)
{
  activate();
  velocities(0.0, 0.0);
  send({"j2", "j1"}, {0.3, 0.1});
  update(100.0);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.1);
  EXPECT_DOUBLE_EQ(effort(1), P * 0.3);
  send({}, {0.4, 0.2});  // no names: dof_names order
  update(100.001);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.4);
  EXPECT_DOUBLE_EQ(effort(1), P * 0.2);
  send(DOFS, {std::numeric_limits<double>::quiet_NaN(), 0.5});  // NaN keeps j1's reference
  update(100.002);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.4);
  EXPECT_DOUBLE_EQ(effort(1), P * 0.5);
  send({"j1", "x"}, {9.0, 9.0});  // unknown joint: ignored
  send(DOFS, {9.0});              // wrong size: ignored
  update(100.003);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.4);
  EXPECT_DOUBLE_EQ(effort(1), P * 0.5);
}

TEST_F(JointVelocityControllerTest, TimesOutToZeroVelocity)
{
  activate();
  velocities(0.1, 0.1);
  send(DOFS, {0.5, 0.5});
  update(100.0);
  EXPECT_DOUBLE_EQ(effort(0), P * (0.5 - 0.1));
  update(100.2);  // exactly the timeout: still the reference
  EXPECT_DOUBLE_EQ(effort(0), P * (0.5 - 0.1));
  update(100.201);
  EXPECT_DOUBLE_EQ(effort(0), P * (0.0 - 0.1));
  EXPECT_DOUBLE_EQ(effort(1), P * (0.0 - 0.1));
  send(DOFS, {0.3, 0.3});  // a new reference counts again
  update(105.0);
  EXPECT_DOUBLE_EQ(effort(0), P * (0.3 - 0.1));
}

TEST_F(JointVelocityControllerTest, NoTimeoutWhenZero)
{
  make(0.0);
  activate();
  velocities(0.0, 0.0);
  send(DOFS, {0.5, 0.5});
  update(100.0);
  update(200.0);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.5);
}

TEST_F(JointVelocityControllerTest, RestartStartsAtZeroAndDeactivationZeroesEfforts)
{
  activate();
  velocities(0.0, 0.0);
  send(DOFS, {0.5, 0.5});
  update(100.0);
  EXPECT_DOUBLE_EQ(effort(0), P * 0.5);
  ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(effort(0), 0.0);
  EXPECT_DOUBLE_EQ(effort(1), 0.0);
  activate();  // the restart before a motor release (D26)
  update(100.001);
  EXPECT_DOUBLE_EQ(effort(0), 0.0);
  EXPECT_DOUBLE_EQ(effort(1), 0.0);
}

// The robot's configuration (config/controllers.yaml) configures this controller as it stands.
TEST(JointVelocityControllerConfig, RobotConfigurationLoads)
{
  for (const std::string name : {"r_arm_velocity_controller", "l_arm_velocity_controller",
                                 "torso_velocity_controller", "head_velocity_controller",
                                 "r_wrist_velocity_controller"}) {
    auto controller = std::make_shared<pr2_ros2_control::JointVelocityController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = name;
    params.update_rate = 1000;
    params.controller_manager_update_rate = 1000;
    params.node_options = rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", CONTROLLERS_YAML});
    ASSERT_EQ(controller->init(params), controller_interface::return_type::OK) << name;
    EXPECT_EQ(controller->configure().id(), State::PRIMARY_STATE_INACTIVE) << name;
    EXPECT_FALSE(controller->command_interface_configuration().names.empty()) << name;
  }
}
