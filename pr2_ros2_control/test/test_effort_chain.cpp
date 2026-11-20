// The effort chain (decision D20) on the real PR2 model from the URDF, with simulated actuators.
#include <gtest/gtest.h>
#include <tinyxml.h>

#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_mechanism_model/transmission.h>

#include "pr2_ros2_control/effort_chain.hpp"

using pr2_hardware_interface::Actuator;
using pr2_mechanism_model::JointState;

namespace
{
void collectActuators(TiXmlElement *e, std::set<std::string> &names)
{
  for (TiXmlElement *c = e->FirstChildElement(); c; c = c->NextSiblingElement())
  {
    const std::string tag = c->Value();
    if ((tag == "actuator" || tag == "rightActuator" || tag == "leftActuator" || tag == "gap_actuator") &&
        c->Attribute("name"))
    {
      names.insert(c->Attribute("name"));
    }
    collectActuators(c, names);
  }
}
}  // namespace

class EffortChain : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::ifstream f(TEST_URDF);
    std::stringstream ss;
    ss << f.rdbuf();
    urdf_ = ss.str();
    doc_.Parse(urdf_.c_str());
    ASSERT_NE(doc_.RootElement(), nullptr);
    std::set<std::string> names;
    for (TiXmlElement *t = doc_.RootElement()->FirstChildElement("transmission"); t;
         t = t->NextSiblingElement("transmission"))
    {
      collectActuators(t, names);
    }
    for (const auto &n : names)
    {
      hw_.actuators_[n] = new Actuator(n);
    }
    robot_ = std::make_unique<pr2_mechanism_model::Robot>(&hw_);
    ASSERT_TRUE(robot_->initXml(doc_.RootElement()));
    state_ = std::make_unique<pr2_mechanism_model::RobotState>(robot_.get());

    roll_ = state_->getJointState("r_wrist_roll_joint");
    flex_ = state_->getJointState("r_wrist_flex_joint");
    ASSERT_NE(roll_, nullptr);
    ASSERT_NE(flex_, nullptr);
    for (auto &js : state_->joint_states_) js.calibrated_ = true;
    flex_->position_ = -0.454;  // where the robot's wrist was

    commanded_.assign(state_->joint_states_.size(), 0);
    for (size_t j = 0; j < state_->joint_states_.size(); ++j)
    {
      commanded_[j] = state_->joint_states_[j].joint_->name == "r_wrist_roll_joint";
    }
    for (size_t i = 0; i < robot_->transmissions_.size(); ++i)
    {
      if (robot_->transmissions_[i]->name_ == "r_wrist_trans") wrist_ = i;
    }
    ASSERT_NE(wrist_, size_t(-1));
    for (auto *a : state_->transmissions_in_[wrist_]) allowed_.insert(a);
    ASSERT_EQ(allowed_.size(), 2u);

    // Every actuator starts enabled with an effort, to see that the chain clears the others.
    for (auto &kv : hw_.actuators_)
    {
      kv.second->command_.enable_ = true;
      kv.second->command_.effort_ = 3.0;
    }
  }

  void TearDown() override
  {
    state_.reset();
    robot_.reset();
    for (auto &kv : hw_.actuators_) delete kv.second;
  }

  // Apply a wrist roll command and run the chain.
  void run(double roll_effort)
  {
    state_->zeroCommands();
    roll_->commanded_effort_ = roll_effort;
    pr2_ros2_control::finishEffortCommands(*state_, commanded_, 0.1, allowed_, hw_.actuators_);
  }

  // Joint efforts the actuator commands amount to (inverse of the transmission).
  void jointEffortsFromActuators(double &roll, double &flex)
  {
    state_->zeroCommands();
    robot_->transmissions_[wrist_]->propagateEffortBackwards(state_->transmissions_in_[wrist_],
                                                             state_->transmissions_out_[wrist_]);
    roll = roll_->commanded_effort_;
    flex = flex_->commanded_effort_;
  }

  std::string urdf_;
  TiXmlDocument doc_;
  pr2_hardware_interface::HardwareInterface hw_;
  std::unique_ptr<pr2_mechanism_model::Robot> robot_;
  std::unique_ptr<pr2_mechanism_model::RobotState> state_;
  JointState *roll_ = nullptr, *flex_ = nullptr;
  std::vector<char> commanded_;
  std::set<const Actuator *> allowed_;
  size_t wrist_ = size_t(-1);
};

TEST_F(EffortChain, WristRollReachesBothMotorsFlexGetsZero)
{
  run(0.5);
  for (const auto *a : allowed_)
  {
    EXPECT_TRUE(a->command_.enable_) << a->name_;
    EXPECT_NE(a->command_.effort_, 0.0) << a->name_;
  }
  double roll, flex;
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, 0.5, 1e-9);
  EXPECT_NEAR(flex, 0.0, 1e-9);
}

TEST_F(EffortChain, OtherActuatorsDisabled)
{
  run(0.5);
  size_t others = 0;
  for (const auto &kv : hw_.actuators_)
  {
    if (allowed_.count(kv.second)) continue;
    ++others;
    EXPECT_FALSE(kv.second->command_.enable_) << kv.first;
    EXPECT_EQ(kv.second->command_.effort_, 0.0) << kv.first;
  }
  EXPECT_GT(others, 20u);
}

TEST_F(EffortChain, ClampedToEffortFraction)
{
  run(5.0);  // URDF limit 10 Nm, fraction 0.1
  double roll, flex;
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, 1.0, 1e-9);
  run(-5.0);
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, -1.0, 1e-9);
}

TEST_F(EffortChain, VelocityLimitBrakes)
{
  // URDF: velocity 3.6 rad/s, k_velocity 2. At 4 rad/s ROS 1 allows at most -2 * (4 - 3.6) = -0.8.
  roll_->velocity_ = 4.0;
  run(0.5);
  double roll, flex;
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, -0.8, 1e-9);
}

TEST_F(EffortChain, UncommandedJointOutsideSoftLimitGetsZero)
{
  // ROS 1's enforceSafety alone would push the flex back below its soft limit (-0.1) with -8 Nm.
  flex_->position_ = 0.0;
  run(0.5);
  double roll, flex;
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, 0.5, 1e-9);
  EXPECT_NEAR(flex, 0.0, 1e-9);
}

TEST_F(EffortChain, NothingCommandedNothingEnabledButTheWrist)
{
  run(0.0);
  double roll, flex;
  jointEffortsFromActuators(roll, flex);
  EXPECT_NEAR(roll, 0.0, 1e-9);
  EXPECT_NEAR(flex, 0.0, 1e-9);
}
