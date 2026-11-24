// MotorReleaseGate (decision D26): the hand-over between the hardware component and the node.
#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "pr2_ros2_control/motor_release_gate.hpp"

using pr2_ros2_control::MotorReleaseGate;
using State = MotorReleaseGate::State;
using namespace std::chrono_literals;

class Gate : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gate().detach();
    gate().cancel();
  }
  void TearDown() override
  {
    gate().detach();
    gate().cancel();
  }
  static MotorReleaseGate &gate() { return MotorReleaseGate::instance(); }
};

TEST_F(Gate, NoRequestWithoutAttachedNode)
{
  EXPECT_FALSE(gate().requestRelease());
  EXPECT_EQ(gate().takeResult(), State::IDLE);
}

TEST_F(Gate, RestartThenReleaseOnce)
{
  gate().attach();
  EXPECT_FALSE(gate().waitForRequest(1ms));
  ASSERT_TRUE(gate().requestRelease());
  EXPECT_EQ(gate().takeResult(), State::REQUESTED);  // still waiting: no reset yet
  EXPECT_TRUE(gate().waitForRequest(1ms));
  gate().reportRestart(true);
  EXPECT_EQ(gate().takeResult(), State::RESTARTED);
  EXPECT_EQ(gate().takeResult(), State::IDLE);  // consumed: exactly one reset
}

TEST_F(Gate, FailedRestartKeepsMotorsHalted)
{
  gate().attach();
  ASSERT_TRUE(gate().requestRelease());
  gate().reportRestart(false);
  EXPECT_EQ(gate().takeResult(), State::FAILED);
  EXPECT_EQ(gate().takeResult(), State::IDLE);
}

TEST_F(Gate, NodeGoingAwayFailsAPendingRequest)
{
  gate().attach();
  ASSERT_TRUE(gate().requestRelease());
  gate().detach();
  EXPECT_EQ(gate().takeResult(), State::FAILED);
}

TEST_F(Gate, ReportWithoutRequestIsIgnored)
{
  gate().attach();
  gate().reportRestart(true);
  EXPECT_EQ(gate().takeResult(), State::IDLE);
}

TEST_F(Gate, WakesTheWaitingThread)
{
  gate().attach();
  std::thread node([] {
    if (gate().waitForRequest(2000ms)) gate().reportRestart(true);
  });
  std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(gate().requestRelease());
  node.join();
  EXPECT_EQ(gate().takeResult(), State::RESTARTED);
}
