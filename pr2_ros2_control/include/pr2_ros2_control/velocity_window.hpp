/*
 * Actuator velocity over a window of encoder samples (decision D22).
 *
 * The motor boards report the encoder count and the time it was sampled. ethercat_hardware (as ROS 1)
 * differentiates over one sample: with the 1200 count encoders at 1 kHz, one count per cycle is
 * 0.04 to 0.16 rad/s at the joints, so the velocity jumps in steps that size. Over the last N samples,
 * timed with the board timestamps, the steps are N times finer for a delay of about N/2 cycles. Exact
 * at constant velocity, and a dropped packet (a repeated sample) does not disturb it.
 */
#ifndef PR2_ROS2_CONTROL__VELOCITY_WINDOW_HPP_
#define PR2_ROS2_CONTROL__VELOCITY_WINDOW_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace pr2_ros2_control
{

class VelocityWindow
{
public:
  // window: number of sample intervals (1 = the single-sample difference of ethercat_hardware).
  explicit VelocityWindow(size_t window = 1) { configure(window); }

  void configure(size_t window)
  {
    window_ = std::max<size_t>(window, 1);
    positions_.assign(window_ + 1, 0.0);
    times_.assign(window_ + 1, 0.0);
    head_ = 0;
    size_ = 0;
    velocity_ = 0.0;
  }

  // position: encoder position (rad, without offsets that can change); time: board sample time (s).
  // Allocation free. Returns the velocity over the samples held so far (0 before the second sample;
  // the last value while no time has passed).
  double update(double position, double time)
  {
    head_ = (head_ + 1) % positions_.size();
    positions_[head_] = position;
    times_[head_] = time;
    size_ = std::min(size_ + 1, positions_.size());
    if (size_ >= 2)
    {
      const size_t oldest = (head_ + positions_.size() - (size_ - 1)) % positions_.size();
      const double dt = times_[head_] - times_[oldest];
      if (dt > 0.0)
      {
        velocity_ = (positions_[head_] - positions_[oldest]) / dt;
      }
    }
    return velocity_;
  }

  size_t window() const { return window_; }

private:
  size_t window_ = 1;
  std::vector<double> positions_, times_;
  size_t head_ = 0, size_ = 0;
  double velocity_ = 0.0;
};

}  // namespace pr2_ros2_control

#endif  // PR2_ROS2_CONTROL__VELOCITY_WINDOW_HPP_
