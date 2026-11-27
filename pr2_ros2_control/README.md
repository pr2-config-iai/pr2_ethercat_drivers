# pr2_ros2_control

ros2_control for the PR2's EtherCAT motor chain (ROS 2 Jazzy).

- `pr2_ros2_control/Pr2EthercatSystem`: a `SystemInterface` that wraps `ethercat_hardware` (the
  motor boards) and the PR2 transmissions from `pr2_mechanism_model` (including the wrist
  differential and the gripper linkage with its passive finger joints).
- `pr2_control_node`: the realtime controller manager (1 kHz, SCHED_FIFO maximum priority like
  ROS 1 `pr2_ethercat`).
- `launch/pr2_control.launch.py`: robot_state_publisher + `pr2_control_node` +
  joint_state_broadcaster.

**Read-only by default.** The motors are halted on every cycle and the component reports an error
if any actuator ever reports being enabled. Like ROS 1, it sends one motor reset when EtherCAT comes
up (with halt still set), which clears the driver's error flags (decision D16).
`/pr2_ethercat/reset_motors` (`std_srvs/Empty`, as ROS 1) runs the same reset on request, e.g. from
`pr2_run_stop_auto_restart` after the e-stop is released (decision D18).

**Commanding (decision D20)** is enabled per joint with launch arguments:

```bash
ros2 launch pr2_ros2_control pr2_control.launch.py commanding_enabled:=true \
  commanded_joints:=r_wrist_roll_joint max_effort_fraction:=0.1
```

- All actuated joints have an `effort` command interface; only the `commanded_joints` (comma
  separated) use it. Their efforts go through ROS 1's chain: `enforceSafety()` (URDF effort and
  velocity limits, position soft limits on calibrated joints), then at most `max_effort_fraction`
  of the URDF effort limit, then the transmissions.
- Every other joint gets zero effort, also when it shares motors with a commanded joint (the wrist
  drives roll and flex with the same two motors). Actuators of other transmissions stay disabled;
  if one ever reports enabled, the component fails.
- Joints that need calibration and have no offset stored on their board get no effort.
- The motors start halted and are released by the first `/pr2_ethercat/reset_motors` (dashboard
  "Reset motors", or the e-stop release through `pr2_run_stop_auto_restart`).
- `/pr2_ethercat/halt_motors` halts until the next reset. As in ROS 1, the motors are also halted
  while the loop frequency over 600 ms is below 750 Hz (`min_acceptable_rt_loop_frequency`).
- **Controller restart:** with commanding enabled, a reset of halted motors (dashboard, or the
  e-stop auto restart) first restarts the active controllers that claim command interfaces, then
  releases the motors, as ROS 1's controller manager did (D26). PID controllers need
  `save_i_term: false` for the restart to clear their integral.
- **Velocity:** by default as ROS 1, one encoder count per cycle (steps of 0.04 to 0.16 rad/s at the
  joints). `velocity_window:=N` takes it over the last N samples with the boards' sample timestamps:
  N times finer steps for about N/2 ms of delay, which made the elbow's velocity loop oscillate at
  N = 10 and the upper arm roll's at N = 3 (D22, D27, D28). Useful for plots, not for control.
- **Velocity controller tests:** `ros2 run pr2_ros2_control velocity_step_test.py --controller
  r_arm_velocity_controller --joint r_elbow_flex_joint` holds the controller's other joints at
  0 rad/s, steps one joint up and back, and halts the motors at the end or as soon as a joint
  moves too fast, too far or out of its soft limits (D27). `--joint none` only holds; `--sine
  0.25,0.5,1,2` sends velocity sines and prints amplitude ratio and lag per frequency (D28);
  `--keep-running` leaves the motors on at 0 rad/s when it ends normally, for joints whose integral
  holds a weight (head tilt, D31).
- **Sine analysis:** `ros2 run pr2_ros2_control velocity_sine_analysis.py --joint J run.csv:P:I ...
  --fit` reads those CSVs and prints per frequency the response, the arm's apparent inertia and
  damping, and the time at the clamp; `--fit` fits a rigid and a two-mass arm and compares their
  closed loop with the measurement, `--predict P:I` adds other gains with and without
  feedforward (D30).
- `config/controllers.yaml` defines `r_wrist_roll_effort_controller`
  (`effort_controllers/JointGroupEffortController`) and `r_wrist_velocity_controller`
  (both wrist joints, provisional gains from D23), and
  `r_arm_velocity_controller` and `l_arm_velocity_controller` (all seven joints of one arm, WG's
  arm trajectory gains in velocity form, clamps for `max_effort_fraction:=0.3`; D28, D29), and
  `torso_velocity_controller` and `head_velocity_controller` (WG's torso and head gains, clamps for
  `max_effort_fraction:=0.6`, each in a launch of its own; D31). None is started by the launch file;
  the two arm controllers can run together. All velocity controllers are
  `pr2_ros2_control/JointVelocityController` (D40): `pid_controller`'s parameters and topics, but
  they start at zero velocity with cleared integrals and return to zero velocity after
  `command_timeout` (0.2 s) without a reference.

## Running

Once per machine or container, install the realtime launcher (needs `sudo`):

```bash
src/pr2_ethercat_drivers/pr2_ros2_control/tools/install_pr2_rt.sh
```

Then:

```bash
colcon build --packages-up-to pr2_ros2_control
source install/setup.bash
ros2 launch pr2_ros2_control pr2_control.launch.py   # ethercat_interface:=ecat0
```

### Realtime privileges (`tools/`, decision D11)

The control loop needs a raw socket (`CAP_NET_RAW`), locked memory (`CAP_IPC_LOCK`) and SCHED_FIFO
(`CAP_SYS_NICE`).

- `pr2-rt-exec`: a small setuid-root helper in `/usr/local/bin`, runnable only by one group. It
  grants exactly these three capabilities as *ambient* capabilities, drops root completely and runs
  the program. It also limits the capability bounding set to those three, so nothing started from
  the program can gain more.
- `pr2-rt`: a shell wrapper around it that resolves the program through `PATH` and carries
  `LD_LIBRARY_PATH` across, which glibc strips from setuid programs.
- The launch file starts `pr2_control_node` with `prefix='pr2-rt'`. For other tools:
  `ros2 run --prefix pr2-rt ethercat_hardware read_only_check -i ecat0`.
- Nothing is attached to the binaries, so rebuilding needs no `setcap`. The programs also don't
  run in the loader's secure mode, so no RPATH tricks are needed.

The node takes the same interface lock as ROS 1 (`/var/tmp/run/EtherCAT_<if>.pid`, or the host's
copy under `/run/host` inside distrobox), so it refuses to start while the ROS 1 robot runs and
vice versa. If the EtherCAT chain cannot be brought up, the controller manager exits.

## Calibration

There is no calibration routine yet. At startup the component uses the zero offsets that ROS 1's
calibration controllers stored on the motor boards, which survive as long as the boards keep power.
It sets each joint's reference position from its URDF `<calibration>` edge, as the ROS 1 controllers
did. Joints without a stored offset are listed as "Not calibrated" when the component is
configured, and get no effort. Each joint also has the state interface `calibrated`: 1 when calibrated or
not in need of calibration, else 0 (ROS 1's `JointState::calibrated_`). The base controller and
odometry refuse to activate with an uncalibrated caster, as in ROS 1.
After a full power cycle, run the ROS 1 robot once to calibrate.

## Parameters (`<hardware>` in the `<ros2_control>` block, see `urdf/pr2_ros2_control.xacro`)

| name | default | |
|---|---|---|
| `interface` | `ecat0` | EtherCAT network interface |
| `allow_unprogrammed` | `false` | allow boards without actuator info in their EEPROM |
| `robot_description_topic` | `/robot_description` | where the full URDF (with the PR2 transmissions) is read from |
| `robot_description_timeout` | `10` | seconds to wait for it |
