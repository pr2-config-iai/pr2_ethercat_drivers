// VelocityWindow (decision D22) on encoder-like data: 1200 counts per revolution sampled at 1 kHz.
#include <gtest/gtest.h>

#include <cmath>

#include "pr2_ros2_control/velocity_window.hpp"

using pr2_ros2_control::VelocityWindow;

namespace
{
constexpr double kCount = 2 * M_PI / 1200;  // one encoder count in rad
double quantized(double position) { return std::floor(position / kCount) * kCount; }
}  // namespace

TEST(VelocityWindow, ZeroUntilTwoSamples)
{
  VelocityWindow w(10);
  EXPECT_EQ(w.update(1.0, 0.0), 0.0);
  EXPECT_NEAR(w.update(1.0 + 0.002, 0.001), 2.0, 1e-9);
}

TEST(VelocityWindow, OneSampleIsTheSingleDifference)
{
  VelocityWindow w(1);
  w.update(0.0, 0.0);
  w.update(0.003, 0.001);
  EXPECT_NEAR(w.update(0.004, 0.002), 1.0, 1e-9);  // only the last interval
}

TEST(VelocityWindow, ExactForConstantVelocityWithJitteredTimestamps)
{
  VelocityWindow w(10);
  double v = 0;
  for (int k = 0; k < 50; ++k)
  {
    const double t = k * 0.001 + ((k * 37) % 11 - 5) * 20e-6;  // +-100 us jitter
    v = w.update(10.5 * t, t);
  }
  EXPECT_NEAR(v, 10.5, 1e-9);
}

TEST(VelocityWindow, QuantizationStepsTenTimesFiner)
{
  // A motor at 10.5 rad/s is 2.005 counts per ms: the single difference jumps between 2 and 3 counts
  // (10.47 and 15.71 rad/s); over 10 samples it stays within one count per 10 ms (1.05 rad/s).
  VelocityWindow one(1), ten(10);
  double max_err_one = 0, max_err_ten = 0;
  for (int k = 0; k < 2000; ++k)
  {
    const double t = k * 0.001;
    const double p = quantized(10.5 * t);
    const double v1 = one.update(p, t), v10 = ten.update(p, t);
    if (k > 20)
    {
      max_err_one = std::max(max_err_one, std::abs(v1 - 10.5));
      max_err_ten = std::max(max_err_ten, std::abs(v10 - 10.5));
    }
  }
  EXPECT_GT(max_err_one, 0.9 * kCount / 0.001);
  EXPECT_LT(max_err_ten, 1.01 * kCount / 0.010);
}

TEST(VelocityWindow, RepeatedSampleFromDroppedPacket)
{
  VelocityWindow w(10);
  double v = 0;
  for (int k = 0; k < 30; ++k)
  {
    const double t = k * 0.001;
    v = w.update(2.0 * t, t);
    if (k == 20) v = w.update(2.0 * t, t);  // same sample again
  }
  EXPECT_NEAR(v, 2.0, 1e-9);

  VelocityWindow single(1);
  single.update(0.0, 0.0);
  single.update(0.002, 0.001);
  EXPECT_NEAR(single.update(0.002, 0.001), 2.0, 1e-9);  // no time passed: keeps the last value
}
