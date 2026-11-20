# pr2_ros2_control

ros2_control for the PR2's EtherCAT motor chain (ROS 2 Jazzy).

- `pr2_ros2_control/Pr2EthercatSystem`: a `SystemInterface` that wraps `ethercat_hardware` (the
  motor boards) and the PR2 transmissions from `pr2_mechanism_model` (including the wrist
  differential and the gripper linkage with its passive finger joints).
- `pr2_control_node`: the realtime controller manager (1 kHz, SCHED_FIFO maximum priority like
  ROS 1 `pr2_ethercat`).
- `launch/pr2_control.launch.py`: robot_state_publisher + `pr2_control_node` +
  joint_state_broadcaster.

**Current state: read-only.** Only state interfaces are exported and the motors are halted on
every cycle. The component reports an error if any actuator ever reports being enabled.

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
did. Joints without a stored offset are listed as "Not calibrated" when the component activates.
After a full power cycle, run the ROS 1 robot once to calibrate.

## Parameters (`<hardware>` in the `<ros2_control>` block, see `urdf/pr2_ros2_control.xacro`)

| name | default | |
|---|---|---|
| `interface` | `ecat0` | EtherCAT network interface |
| `allow_unprogrammed` | `false` | allow boards without actuator info in their EEPROM |
| `robot_description_topic` | `/robot_description` | where the full URDF (with the PR2 transmissions) is read from |
| `robot_description_timeout` | `10` | seconds to wait for it |
