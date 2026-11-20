^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* Collect the motor boards' diagnostics (``EthercatHardware::collectDiagnostics()``) once a second in
  a normal-priority thread while active, like ROS 1 ``pr2_ethercat``'s ``diagnosticLoop``. Without it
  every board reported "Have not yet collected diagnostics" and the EtherCAT Master "Number of
  EtherCAT devices changed".
* Realtime privileges come from ``pr2-rt`` (``tools/``): a setuid helper, installed once, that
  grants CAP_NET_RAW, CAP_IPC_LOCK and CAP_SYS_NICE as ambient capabilities and drops root. The
  launch file uses it as prefix. No more ``setcap`` after builds and no RPATH workaround; the node
  no longer links the plugin library.
* Grippers are recognised by their URDF transmission type instead of a ``dynamic_cast``, so the
  plugin does not link the transmission plugin library.
* New package: ros2_control ``SystemInterface`` ``pr2_ros2_control/Pr2EthercatSystem`` wrapping
  ethercat_hardware and the PR2 transmissions from pr2_mechanism_model, the realtime controller
  manager node ``pr2_control_node``, and ``launch/pr2_control.launch.py``.
* Milestone 1, read-only: state interfaces only, motors halted on every cycle. On the IAI PR2 its
  ``/joint_states`` match ROS 1's for all 45 joints.
* Uses the zero offsets ROS 1's calibration stored on the motor boards and sets each joint's
  calibration reference position like ROS 1's calibration controllers.
