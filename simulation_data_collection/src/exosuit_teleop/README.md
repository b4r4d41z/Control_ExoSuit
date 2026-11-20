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

## Start

Point ROS networking at the simulation graph. Keep physical robot controllers
off that graph because the command topics use their existing names. The
launcher defaults to the loopback master `http://127.0.0.1:11321`, reuses a
reachable ROS master at the configured URI, and starts a local master when
needed. Isaac Sim can be started separately.

```bash
cd "$repo/simulation_data_collection"
export ISAAC_CATKIN_SETUP="$HOME/exosuit-sim-ws/devel/setup.bash"
./isaac_datacollection_start.sh
```

The Catkin setup path defaults to `simulation_data_collection/devel/setup.bash`.
The launcher honors `ROS_MASTER_URI`, `ROS_HOSTNAME`, and `ROS_IP`.
`ISAAC_ROS_MASTER_URI` and `ISAAC_ROS_IP` take precedence when set.
The four panes show the master, arms, grippers, and recording. Stop each node
with Ctrl+C in its pane. Keep the recording subscriber connected through
recording shutdown so it can receive OFF.

The optional `transition_duration_s:=SECONDS` argument changes the duration
of each initialization leg and the return. The default is 3 seconds per leg.
Use `left_serial:=...`, `right_serial:=...` or `trajectories_file:=...` to
override their respective defaults.

## Wire contract

| ROS1 topic | Message | Contract |
| --- | --- | --- |
| `/kuavo_arm_traj` | `sensor_msgs/JointState` | Command positions in degrees at about 10 Hz. Robot right joints come first, driven by physical left encoders, then robot left joints. |
| `/control_robot_hand_position` | `dynamic_biped/robotHandPosition` | Six values per hand at 50 Hz when enabled. Republishes the last message when disabled. |
| `/data_collection/recording` | `std_msgs/Bool` | Latched request, OFF on startup and shutdown. |

The reference position mapping swaps the first two channels on each bus and
inverts encoder IDs 1, 8, 9, and 13. Each bus is read using a Feetech
`GroupSyncRead` of register 56.

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

- **A** starts the sequence from an assumed ZERO pose. Each quintic segment
  completes on local elapsed time. The first A press establishes the
  reference encoder origin before initialization. After publishing READY,
  the arm process switches to the original accumulated encoder angles.
- **A** during teleoperation immediately switches to a timed return from the
  last published arm command. During return and while paused at READY, the
  arm process stops sampling encoders. When the return finishes, press
  **A** again to resume with the original encoder origin. Presses during
  initialization or return are ignored. The joint command can jump on
  entering teleoperation. Wraparound relies on the shortest signed encoder
  delta, so movement beyond half a revolution between reads is ambiguous.
- **Y** enables or disables grip changes. **ZL/ZR** toggle the respective grip
  through the reference JoyconRobotics `get_control()` state. Disabled mode
  republishes the last grip command if one exists.
- **X** toggles recording independently of arm state. Press X to end a
  recording before pausing arms if the episode should exclude the pause.

### Hardware-free checks

From the sourced overlay, run the ROS smoke test against a fresh loopback
`roscore` with no production devices or bridge. It supplies fake button and
encoder input and checks initialization before attaching an arm subscriber.

```bash
python3 -B "$repo/simulation_data_collection/src/exosuit_teleop/tests/ros_smoke.py"
```

The arm process sends commands open loop. Waypoint completion describes the
published sequence. Isaac Sim can receive commands while it is connected,
and the controller does not measure physical arrival or collisions.
