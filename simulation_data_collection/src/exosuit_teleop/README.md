# ExoSuit teleoperation for Isaac Sim (ROS1 Noetic)

This package owns the Joy-Cons and the three command publishers. Stop the legacy
arm, grip and Isaac recording nodes before launching it. Camera topics and the
ROS1-to-ROS2 bridge use their existing configuration.

## Build

Use a Noetic underlay with the existing `dynamic_biped` message package. If it
is unavailable, build it from the reference source in a separate workspace.
The example uses a clean Catkin build directory because the repository contains
generated files from another machine.

```bash
cd /home/Synthetic_data_collection
repo="$PWD"
source /opt/ros/noetic/setup.bash

# Omit this block if an existing Noetic workspace already supplies dynamic_biped.
mkdir -p "$HOME/exosuit-msgs-ws/src"
ln -s "$repo/real_robot_data_collection/src/dynamic_biped" "$HOME/exosuit-msgs-ws/src/"
(cd "$HOME/exosuit-msgs-ws" && catkin_make -j2)
source "$HOME/exosuit-msgs-ws/devel/setup.bash"

mkdir -p "$HOME/exosuit-sim-ws/src"
ln -s "$repo/simulation_data_collection/src/biped_s40" "$HOME/exosuit-sim-ws/src/"
ln -s "$repo/simulation_data_collection/src/exosuit_teleop" "$HOME/exosuit-sim-ws/src/"
(cd "$HOME/exosuit-sim-ws" && catkin_make -j2 && cd build && ctest -V -R exosuit_teleop)
source "$HOME/exosuit-sim-ws/devel/setup.bash"
```

Running the node also requires the Feetech `scservo_sdk` API (for example,
`feetech-servo-sdk==1.0.0` and its pyserial dependency), Python evdev and
`moveit_msgs` on the node, plus an operational MoveIt planning scene and
`GetStateValidity` service reachable over ROS1. Use Linux `hid-nintendo`
with the two individual Joy-Con button event devices. Select stable udev paths
where available and verify their `evtest` mapping before connecting outputs.
The Joy-Cons are opened once each with an exclusive evdev grab. Stop old
hidraw-based Joy-Con readers as well, since an evdev grab cannot exclude
programs opening a different device interface. Use the serial paths that
correspond to physical left IDs 1–7 and right IDs 8–14.

## Wire contract

| Topic | Message | Notes |
| --- | --- | --- |
| `/kuavo_arm_traj` | `sensor_msgs/JointState` | Right joint names 1–7, then left 1–7. Positions in **degrees**. The reference starts with physical left encoders, including its first-two-channel swaps and inversions. |
| `/control_robot_hand_position` | `dynamic_biped/robotHandPosition` | Reference six values per hand. |
| `/robot_arm_q_v_tau` | `dynamic_biped/robotQVTau` or `sensor_msgs/JointState` | Configure the observed type, wire order if unnamed, and angle/velocity units. |
| `/robot_hand_position` | `dynamic_biped/robotHandPosition` | Stamped feedback with six values per hand. |
| `/data_collection/recording` | `std_msgs/Bool` | Latched recording request. The topic carries no acknowledgment. |

Before arming, check actual types, feedback values and update rates with
`rostopic type`, `rostopic info`, `rostopic echo -n 1` and `rostopic hz`.
The legacy `robotQVTau` message carries no timestamp. Use its opt-in only
after confirming that the sole publisher is a live, non-replaying source,
that it stops publishing stale values after an upstream disconnect, and that
its transport latency is bounded. The node can measure receipt age for this
type. Stamped feedback also requires compatible ROS clocks and checks the
source timestamp. Both types require finite position and velocity feedback.

## Validate the deployment before enabling output

Copy `config/safety_limits.yaml` outside the checkout. Its shipped profile
blocks launch before the node opens devices or advertises commands. Fill the
deployment profile using measured controller and bridge behavior, including
per-joint velocity/acceleration, permissible following error, settling/freshness
bounds and plausible encoder speed. URDF position and velocity limits are
enforced in addition to the configured bounds. The planned setpoint stream is
approximately 10 Hz. Verify that the receiver respects that timing and motion
profile, rejects stale commands and safely stops and holds when commands stop,
including on pause, interruption, competing publishers, link loss and process
exit. The node invokes that watchdog by ceasing arm publication. It does not
publish an estimated stop pose or a ZERO command as a substitute for a watchdog.

Load the matching robot and table geometry, transforms, other relevant robot
joints and world objects in the MoveIt planning scene. Review the collision
matrix and verify that the service rejects known collisions. Use a static scene
for each planned movement, and cover tracking error, braking, command
interpolation and every gripper shape this workflow can select with the
collision margin. The service is queried for the full robot at both endpoints
and intermediate poses with a maximum per-axis joint sampling interval.
Sampling is not a continuous collision proof. Set the spacing against measured
clearance and margins, or keep the profile disabled. Verify that Isaac sees
the same model/world. Confirm that the recorder honors OFF before movement
and does not start recording from another source. ROS1 graph checks reject
competing publishers and missing subscribers at their polling cadence.
They cannot establish downstream ROS2 ownership or delivery by themselves.

Then launch with the reviewed profile and correct physical device paths:

```bash
roslaunch exosuit_teleop teleop.launch \
  safety_file:=/absolute/path/to/verified_safety.yaml \
  left_serial:=/dev/ttyACM0 right_serial:=/dev/ttyACM1 \
  left_joycon:=/dev/input/by-id/LEFT_EVENT \
  right_joycon:=/dev/input/by-id/RIGHT_EVENT
```

### Model poses

`trajectories.yaml` contains model-based candidates in degrees. For the
bundled `biped_s40.urdf`, joint 4 is elbow flexion around Y, joint 3 is shoulder
yaw around Z, and joint 1 is shoulder pitch around Y. The mirrored shoulder
yaw rotates the flexed forearms outward in ELBOWS_OUT. A 30-degree forward
shoulder pitch raises their ends in ARMS_UP. Returning yaw to zero turns the
flexed forearms forward in READY. Forward kinematics at the two URDF end
effectors gives the following approximate coordinates in the `base_link` frame.

| Stage | Forward X (m) | Lateral Y (m) | Up Z (m) |
| --- | ---: | ---: | ---: |
| ZERO | -0.018 | ±0.229 | -0.201 |
| ELBOWS_OUT | 0.013 | ±0.620 | 0.160 |
| ARMS_UP | 0.091 | ±0.620 | 0.197 |
| READY | 0.377 | ±0.229 | 0.362 |

Review these poses with the actual table height, base transform and robot.
Select a deployment-specific file with `trajectories_file:=...` if required.
Waypoint arrival is confirmed using fresh, stationary feedback before advancing.
On return, the planner tries nearby points on the reference path and checks
each connector and its remaining path. It keeps HOLD if no checked route
exists. This algorithm searches straight connectors in joint space and can
decline a return even if a different planner could find a curved path. During
live control, the latest relative encoder target is approached through checked,
bounded segments. Collision-service latency and verified limits affect response.

### Controls

- **A** starts at a confirmed stationary ZERO, pauses an active move or live
  teleoperation, or requests a collision-checked return from the stopped pose.
  A fresh encoder sample taken after confirmed READY becomes the relative
  baseline. The first teleoperation setpoint equals READY.
- **Y** enables or disables gripper changes. **ZL/ZR** toggle the corresponding
  gripper while enabled. Disabling Y keeps the last gripper command.
- **X** toggles recording in TELEOP. Leaving TELEOP sends OFF before any further
  arm commands. Startup and rejoin do not open episodes.
- **FAULT** stops the arm stream and recording. Inspect the logged fault and
  resolve the cause before explicitly restarting the node.

Run the ROS adapter smoke test from the sourced overlay against a fresh
`roscore` with no production devices or bridge:

```bash
python3 -B "$repo/simulation_data_collection/src/exosuit_teleop/tests/ros_smoke.py"
```
