# ExoSuit teleoperation for Isaac Sim (ROS1 Noetic)

The simulation launches three independent ROS processes. The arm publisher
reads the Feetech encoders and controls the simulated arm lifecycle. The grip
publisher retains the reference Y, ZL, and ZR behavior. The Isaac recording
trigger retains the reference X toggle and latched Bool commands. The bag
recorder is the reference real-robot alternative. Start only the Isaac trigger
for this simulation workflow.

## Build

Use a Noetic underlay with the existing `dynamic_biped` message package. If it
is unavailable, build it from the reference source in a separate workspace.
Clean Catkin workspaces avoid the repository's generated files from another
machine.

```bash
cd /home/Synthetic_data_collection
repo="$PWD"
source /opt/ros/noetic/setup.bash

# Omit this block if an existing Noetic workspace already supplies dynamic_biped.
mkdir -p "$HOME/exosuit-msgs-ws/src"
ln -sfn "$repo/real_robot_data_collection/src/dynamic_biped" "$HOME/exosuit-msgs-ws/src/"
(cd "$HOME/exosuit-msgs-ws" && catkin_make -j2)
source "$HOME/exosuit-msgs-ws/devel/setup.bash"

mkdir -p "$HOME/exosuit-sim-ws/src"
ln -sfn "$repo/simulation_data_collection/src/biped_s40" "$HOME/exosuit-sim-ws/src/"
ln -sfn "$repo/simulation_data_collection/src/exosuit_teleop" "$HOME/exosuit-sim-ws/src/"
(cd "$HOME/exosuit-sim-ws" && catkin_make -j2 && cd build && ctest -V -R exosuit_teleop)
source "$HOME/exosuit-sim-ws/devel/setup.bash"
```

Use the Python 3 environment that runs ROS on the Radxa. Reuse its working
JoyconRobotics and Feetech installation if it matches the APIs below. For a
fresh Ubuntu 20.04 container, install `libhidapi-hidraw0` and `python3-pip`
and install these Python packages as the user who runs ROS. The JoyconRobotics
revision comes from the repository's `external_proj/joycon-robotics` Gitlink.

```bash
python3 -m pip install --user feetech-servo-sdk==1.0.0 \
  'joycon-robotics @ git+https://github.com/box2ai-robotics/joycon-robotics.git@c2cf0508642c1ebcd7e5c4e9661100f532c3b063'
python3 -c 'from joyconrobotics import JoyconRobotics; from scservo_sdk import GroupSyncRead'
```

The pinned library discovers individual Joy-Cons through HIDAPI and opens HID
handles by Nintendo product ID. Pair one supported left controller and one
supported right controller, and expose their hidraw devices with access for
the container user. Stop previous teleoperation nodes and verify the pairing
mode, device ownership, and permissions if acquisition fails. The ROS nodes
reuse the right controller across processes as in the reference, with setup
serialized per controller side. An `/dev/input/eventN` path is irrelevant to
this HID backend. Match physical left servo IDs 1–7 and right IDs 8–14 to
their serial paths. The defaults are `/dev/ttyACM0` and `/dev/ttyACM1`,
respectively. Use stable paths when available.

```bash
python3 - <<'PYTHON'
import hid

for device in hid.enumerate(0x057e, 0):
    print(hex(device["product_id"]), device["path"])
PYTHON
ls -l /dev/hidraw* /dev/serial/by-id/
```

## Isolate the simulator before startup

Keep the ExoSuit processes, ROS1 master, and Isaac bridge on a simulation-only
network with no route to a physical robot. Use an isolated container network
for co-located processes or a private network restricted to the simulator
and Radxa for separate hosts. Outside loopback, Noetic binds ROS listeners
on all interfaces. Restrict those listeners to the simulation network using
a network namespace or firewall. Keep the bridge's ROS2 domain and network
separate from the physical robot, and check its forwarding rules and command
subscribers before using the controls. ROS1 does not authenticate node
identities. A separate port and the `simulation_only` argument are guardrails,
while network and bridge isolation prevent delivery to physical controllers.

The tmux launcher starts a fresh master and uses separate panes for the master,
arms, grippers, and recording. It refuses to reuse a master on the selected
port. For a single host or shared network namespace, the master defaults to
`http://127.0.0.1:11321`. When the simulator is on a separate host, set
`ISAAC_ROS_MASTER_URI=http://<radxa-simulation-ip>:11321` and
`ISAAC_ROS_IP=<radxa-simulation-ip>` on the Radxa, then point the simulator's
ROS1 bridge to that URI. The publishers check that ROS selected the specified
master on a local IPv4 address and a dedicated port. Set unique simulator
addresses and keep the physical ROS graph off that network.

## Wire contract and arm setup

| ROS1 topic | Message | Contract |
| --- | --- | --- |
| `/kuavo_arm_traj` | `sensor_msgs/JointState` | Command positions in degrees at about 10 Hz. Robot right joints come first, driven by physical left encoders, then robot left joints. |
| `/control_robot_hand_position` | `dynamic_biped/robotHandPosition` | Six values per hand at 50 Hz when enabled; republishes the last message when disabled. |
| `/data_collection/recording` | `std_msgs/Bool` | Latched request, OFF on startup and shutdown. The topic carries no acknowledgment. |
| `/robot_arm_q_v_tau` | `dynamic_biped/robotQVTau` or `sensor_msgs/JointState` | Confirm the type, order, position and velocity units, and update rate before arming. |
| `/robot_hand_position` | `dynamic_biped/robotHandPosition` | Reference feedback has a header and six values per hand. Grip control runs independently of it. |

The reference position mapping swaps the first two channels on each bus and
inverts encoder IDs 1, 8, 9, and 13. Each bus is read using a Feetech
`GroupSyncRead` of register 56. The pinned SDK checks transport and response
data, but its group reader discards per-device status bytes. The controller
rejects missing data, invalid ticks, and implausible wraparound intervals.

Copy `config/safety_limits.yaml` to a file outside the checkout and fill it
using the simulator's measured behavior. The shipped profile disables arm
startup. Configure the joint limits, freshness thresholds, and feedback
order and units. Verify that the simulator stops and holds on loss of the arm
stream, which is how HOLD and FAULT request a stop. Preset and return motion
use position, velocity, and acceleration bounds. Live tracking updates the
latest relative target with position, following-error, and per-tick velocity
bounds. Live acceleration shaping, braking, and setpoint interpolation are
responsibilities of the verified simulator controller.

Only use the unstamped `robotQVTau` option after verifying a single live
publisher, bounded transport latency, and a bridge that stops publishing on
upstream disconnect. Its position (`q`) and velocity (`v`) arrays must
match the configured 14-joint order. Torque is unused. Stamped
`JointState` requires unique names, matching arrays, and a fresh timestamp.
From a terminal sourced and configured for the isolated simulation master:

```bash
rostopic type /robot_arm_q_v_tau
rostopic info /robot_arm_q_v_tau
rostopic echo -n 1 /robot_arm_q_v_tau
rostopic hz /robot_arm_q_v_tau
rostopic info /kuavo_arm_traj
rostopic info /control_robot_hand_position
rostopic info /data_collection/recording
```

MoveIt collision sampling is optional. Without a configured service there is
no collision checking, and resume follows a straight joint-space return from
the measured stop pose to READY. Validate clearances in the simulated model
before running. To enable the checks, install `moveit_msgs`, configure a
`GetStateValidity` service with the approved scene, and verify its collision
matrix and rejection of a known collision. The node samples each planned
segment and each proposed live step for the full robot. Sampling cannot
guarantee swept clearance or account for controller interpolation. Review
the scene, sampling spacing, controller behavior, and gripper geometry.

```bash
cd "$repo/simulation_data_collection"
ISAAC_CATKIN_SETUP="$HOME/exosuit-sim-ws/devel/setup.bash" \
  ./isaac_datacollection_start.sh simulation_only:=true \
  safety_file:=/absolute/path/to/verified_safety.yaml \
  left_serial:=/dev/ttyACM0 right_serial:=/dev/ttyACM1
```

For direct `roslaunch exosuit_teleop teleop.launch`, export `ROS_MASTER_URI`
and `ROS_IP` for the dedicated master, unset `ROS_HOSTNAME`, and pass
`simulation_only:=true`. Pass `sim_master_uri:=...` when using a nondefault
URI. Start the master and use the same isolation as the tmux workflow.
Stop the grip and recording panes with Ctrl+C, then stop arms and the master.
Keep the recording subscriber connected until it receives OFF.

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

Review these poses using the actual simulated table height, base transform,
and joint model. Select a deployment-specific file with
`trajectories_file:=...` if needed.

### Controls

- **A** initializes from a stationary ZERO. Each waypoint needs fresh,
  stationary feedback before advance. At READY a new encoder sample establishes
  the relative baseline. **A** pauses the stream, then requests a return from
  the measured stop pose. A fresh encoder baseline is taken again at READY.
  A latched arm fault requires a restart after addressing its cause.
- **Y** enables or disables grip changes. **ZL/ZR** toggle the respective grip
  through the reference JoyconRobotics `get_control()` state. Disabled mode
  republishes the last grip command if one exists.
- **X** toggles recording independently of arm state. Press X to end a
  recording before pausing arms if the episode should exclude the pause.

### Hardware-free checks

From the sourced overlay, run the ROS adapter smoke tests against a fresh
loopback `roscore` with no production devices or bridge. The test supplies
fake button inputs, arm feedback, and encoder samples.

```bash
python3 -B "$repo/simulation_data_collection/src/exosuit_teleop/tests/ros_smoke.py"
python3 -B "$repo/simulation_data_collection/src/exosuit_teleop/tests/ros_smoke.py" --legacy
```
