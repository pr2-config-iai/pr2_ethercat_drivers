^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* New package: ros2_control ``SystemInterface`` ``pr2_ros2_control/Pr2EthercatSystem`` wrapping
  ethercat_hardware and the PR2 transmissions from pr2_mechanism_model, the realtime controller
  manager node ``pr2_control_node``, and ``launch/pr2_control.launch.py``.
* Milestone 1, read-only: state interfaces only, motors halted on every cycle. On the IAI PR2 its
  ``/joint_states`` match ROS 1's for all 45 joints.
* Uses the zero offsets ROS 1's calibration stored on the motor boards and sets each joint's
  calibration reference position like ROS 1's calibration controllers.
