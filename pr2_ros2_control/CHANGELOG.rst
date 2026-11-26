^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* As ROS 1's controller manager, controllers start fresh when halted motors are released
  (decision D26): with commanding enabled, a reset of halted motors first has ``pr2_control_node``
  restart the active controllers that claim command interfaces (one switch, deactivate and
  activate), and the motors are reset after that; if the restart fails they stay halted. The
  component and the node meet in ``MotorReleaseGate`` (library ``pr2_motor_release_gate``).
  ``r_wrist_velocity_controller`` sets ``save_i_term: false`` so that the restart clears the integral.
* ``l_arm_velocity_controller``: the right arm's gains and clamps on the left arm, verified with
  the same steps and sines (decision D29).
* ``r_arm_velocity_controller`` with WG's arm trajectory gains in velocity form (velocity p =
  position d, velocity i = position p), clamps for ``max_effort_fraction:=0.3`` (decision D28).
* ``velocity_step_test.py --sine``: velocity sines with amplitude ratio and lag (D28).
* ``velocity_window`` default back to 1 (ROS 1's single-sample velocity): with 10 samples the elbow's
  velocity loop (p 6) oscillated from the 5 ms of delay; with 1 sample it was stable (decision D27).
* ``r_arm_velocity_controller`` (provisional, not started): all 7 right arm joints, WG's calibration
  velocity gains, clamped to the effort limit (D27).
* ``tools/velocity_step_test.py``: velocity steps on one joint of a velocity controller while the
  others are held, with a watchdog that halts the motors (D27).
* Actuator velocity over the last ``velocity_window`` encoder samples (default 10), timed with the
  motor boards' sample timestamps, instead of ROS 1's single-sample difference, whose steps are one
  encoder count per cycle (0.04 to 0.16 rad/s at the joints); 10 times finer for about 5 ms of delay
  (decision D22). ``velocity_window:=1`` gives the ROS 1 value.
* Commanding (decision D20), off by default: effort command interfaces on all actuated joints;
  parameters ``commanding_enabled``, ``commanded_joints`` and ``max_effort_fraction`` (launch and
  xacro arguments). Efforts of the listed joints go through ROS 1's ``enforceSafety()`` and the
  transmissions; all other joints get zero effort and the actuators of other transmissions stay
  disabled. ``~/halt_motors`` halts until ``~/reset_motors``; motors start halted until the first
  reset; ROS 1's loop frequency monitor (below 750 Hz over 600 ms) halts them too.
* ``pr2_control_node`` publishes ROS 1's "Realtime Control Loop" diagnostics (EtherCAT, controller
  and loop times, jitter, overruns, loop frequency over 600 ms; same keys and 30 s overrun warning)
  once a second. The low-frequency motor halt of ROS 1 (below ``min_acceptable_rt_loop_frequency``,
  750 Hz) is only reported until commanding is added.
* The first cycle gets the nominal period instead of the ~1 us measured since the loop started:
  the controller manager keeps periodicity statistics for the whole run, and that one sample made
  it report "bad periodicity" (about 1.8e6/N Hz too high after N cycles, plus a huge deviation).
* ``~/reset_motors`` and ``~/halt_motors`` (``std_srvs/Empty``, ``/pr2_ethercat/...`` as in ROS 1).
  While read-only, a reset runs the same sequence as the startup reset (motors stay halted) and
  halt is a no-op.
* One motor reset with the first EtherCAT exchange, as ROS 1 ``pr2_ethercat`` does in its first cycle,
  with halt still set: clears the boards' latched safety faults and error flags (every board
  reported "Timestamp jumped" for its first packet forever).
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
