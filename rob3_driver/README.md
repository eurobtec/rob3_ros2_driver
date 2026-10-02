# ROB3 ROS 2 driver

A ROS 2 **Lyrical** driver for the **Eurobtec ROB 3** 6-axis robot, talking to
the robot's Intel 8031 controller over its **RS-232** serial link. It is
implemented in Python and provides a protocol client, ROS 2 driver node, robot
description, launch configuration, and controller configuration.

The wire protocol is the ROB3 low-level protocol, reverse-engineered and
verified against the ROM/simulator in the firmware repo (see that repo's
`hardware/host/command.md` and `firmware/src/annotated/rs232.asm`). The driver
speaks the same bytes to either **real hardware** (`/dev/ttyUSB0`) or the
**ucSim simulator** (via its `-S` UART socket), so it can be developed without
the robot.

## Layout

```
rob3_driver/
├── package.xml
├── setup.py / setup.cfg
├── resource/rob3_driver
├── rob3_driver/
│   ├── protocol.py           # pure ROB3 wire-protocol codec (no ROS deps)
│   ├── transport.py          # serial transport (real HW and ucSim pty)
│   ├── calibration.py        # joint <-> 0..255 count mapping (per-axis)
│   ├── rob3_interface.py     # protocol + transport = high-level robot client
│   └── rob3_driver_node.py   # ROS 2 node: JointState, trajectory action, services
├── launch/rob3.launch.py
├── config/rob3_controllers.yaml
├── urdf/rob3.urdf.xacro
└── test/                     # pytest unit tests (protocol codec, calibration)
```

## Architecture

| Component | Responsibility |
| :-------- | :-------------- |
| `protocol.py` | Encode ROB3 commands and decode controller replies |
| `transport.py` | Communicate over serial (the real robot, or a ucSim pty) |
| `rob3_interface.py` | Provide a high-level client for the ROB3 protocol |
| `rob3_driver_node.py` | Publish joint states and provide trajectory/action services |
| `launch/`, `config/`, `urdf/` | Describe and configure the ROB3 ROS 2 system |

## Build with Docker

A [`Dockerfile`](../Dockerfile) (at the repository root) bakes a coherent ROS 2
Lyrical environment and pre-builds this package, so you don't re-`apt` on every
run. Build it **once** from the repository root (the directory that contains the
`rob3_driver/` package):

```bash
docker build -t rob3-ros2:lyrical .
```

> Why a baked image: the stock `osrf/ros:lyrical-desktop` ships a *frozen*
> package set, but `control_msgs` from apt is newer and needs
> `service_msgs`/`builtin_interfaces` upgraded in lockstep — otherwise you hit
> `undefined symbol: has_buffer_fields_*` at runtime. The Dockerfile does one
> coherent `apt upgrade` + installs `control_msgs`, `xacro`,
> `joint_state_publisher`, `pyserial`, and the noVNC stack.

The package is pre-built into `/opt/rob3_ws` and sourced automatically in every
shell, so `ros2 run rob3_driver ...` and `ros2 launch rob3_driver ...` work out
of the box.

## Run RViz in Docker (noVNC)

Start a container from the baked image with a virtual X display and a
browser-accessible VNC connection. This example skips the transport driver and
just views the URDF (`driver:=false`):

```bash
docker run --rm -it --name rob3-rviz -p 6080:6080 rob3-ros2:lyrical bash -lc '
  x11vnc_disp=:99
  Xvfb $x11vnc_disp -screen 0 1400x900x24 >/tmp/xvfb.log 2>&1 &
  export DISPLAY=$x11vnc_disp
  x11vnc -display $DISPLAY -forever -shared -nopw \
    -rfbport 5900 -listen 0.0.0.0 >/tmp/x11vnc.log 2>&1 &
  websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/ws.log 2>&1 &
  exec ros2 launch rob3_driver rob3.launch.py driver:=false rviz:=true
'
```

Open port `6080` in a browser → `/vnc.html?autoconnect=1&resize=remote`. To
connect the driver to the robot/ucSim instead, drop `driver:=false` and pass the
`device:=` argument (see Teleop below). Exit with `exit` / Ctrl+C.

## Quickstart

```bash
# real robot: point the driver at the USB-serial adapter
ros2 launch rob3_driver rob3.launch.py device:=/dev/ttyUSB0
```

Against the **ucSim simulator** the driver uses the *same* serial transport —
ucSim is attached to a pty and the driver opens the other end. The sim needs an
auto-baud bring-up first (load the cl_hw modules, lock the baud with `rxd`); the
step-by-step is in [docs/SIMULATION.md](docs/SIMULATION.md):

```bash
# after bringing ucSim up on a pty (see docs/SIMULATION.md), e.g. /dev/pts/7:
ros2 launch rob3_driver rob3.launch.py device:=/dev/pts/7
```

> A TCP transport to ucSim's `-S port=` socket was tried and removed: that
> socket feeds ucSim's single-byte RX buffer asynchronously and drops frames.
> The paced serial pty/file path round-trips cleanly and matches real hardware.

See `rob3_driver/protocol.py` for the exact wire encoding; every command there
is annotated with its ROM provenance.

## Teleop (jog the arm) + RViz

The driver subscribes to **`/joint_jog`** (`control_msgs/JointJog`): each
message nudges the named joints and the driver sends the resulting position
setpoints to the robot/sim. RViz shows the motion because the driver publishes
**`/joint_states`**, which `robot_state_publisher` turns into TF for the URDF.

> Note: this is a 6-axis arm, so teleop means **jogging joints**, not
> `cmd_vel`/`teleop_twist_keyboard` (those are for mobile bases). A keyboard jog
> node is included: `rob3_jog_keyboard`.

### One-command self-contained demo (no hardware, no ucSim)

The fastest way to *try* teleop. It starts a **fake ROB3 controller** (answers
the protocol on a pty), the real driver, RViz over noVNC, and the keyboard jog
node — so you press keys and watch the arm move, with nothing but Docker:

```bash
docker run --rm -it -p 6080:6080 rob3-ros2:lyrical \
  ros2 run rob3_driver teleop_demo.sh
```

Then open **http://localhost:6080/vnc.html?autoconnect=1&resize=remote** for
RViz, and in the same terminal use the jog keys: `1`..`6` select the axis,
`+`/`-` jog, `[`/`]` change the step, `q` quits.

### Against the real robot or ucSim (RViz over noVNC)

In the baked image (Xvfb + x11vnc + novnc on port 6080), two panes:

```bash
# pane 1 — driver + robot_state_publisher + RViz, talking to the robot
#   real robot:  device:=/dev/ttyUSB0   (pass --device /dev/ttyUSB0 to docker run)
#   ucSim:       first run scripts/sim_bringup.py, then device:=/dev/pts/N
ros2 launch rob3_driver rob3.launch.py rviz:=true device:=/dev/ttyUSB0

# pane 2 — keyboard teleop (publishes /joint_jog)
ros2 run rob3_driver rob3_jog_keyboard
```

Keyboard jog keys: `1`..`6` select the axis (base, shoulder, elbow,
wrist_pitch, wrist_roll, gripper); `+`/`-` jog it; `[`/`]` change the step size;
`q` quits. Watch the arm move in RViz (open port `6080` →
`/vnc.html?autoconnect=1&resize=remote`).

### Jog from the command line (no keyboard node)

```bash
# nudge the base joint by +0.1 rad
ros2 topic pub --once /joint_jog control_msgs/msg/JointJog \
  '{joint_names: [base], displacements: [0.1]}'
```

### Scripted motion via the trajectory action

```bash
ros2 action send_goal /follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [base, shoulder, elbow, wrist_pitch, wrist_roll, gripper],
     points: [{positions: [0.2, 0.0, -0.3, 0.0, 0.0, 0.0],
               time_from_start: {sec: 2}}]}}'
```

Services: `enable_motors`, `disable_motors`, `estop`, `read_serial_number`
(`std_srvs/Trigger`), e.g. `ros2 service call /estop std_srvs/srv/Trigger`.

## Status / scope

- Protocol codec + transports + driver node + URDF/launch/config: implemented.
- Position control (single + all-axis) and readback: implemented per the
  verified protocol.
- Speed/time-factor moves (`0x70`–`0x7F`) and the stored-program upload are
  wired in the codec but the trajectory controller uses simple position
  setpoints by default.
- Joint↔count calibration uses the per-axis bench values from the ROB3 firmware
  repo (`hardware/motors/`) where available; unmeasured axes use a linear
  placeholder clearly marked in `calibration.py`.
