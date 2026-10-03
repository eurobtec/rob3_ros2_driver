# rob3_ros2_driver

A **ROS 2** (Lyrical) driver for the **Eurobtec ROB 3** 6-axis robot, speaking
the robot's reverse-engineered RS-232 low-level protocol to its Intel 8031
controller. Pure Python.

The actual ament package lives in [`rob3_driver/`](rob3_driver/) — see its
[README](rob3_driver/README.md) for architecture, build (Docker/colcon), launch,
and usage against real hardware or the ucSim simulator.

```
rob3_ros2_driver/
├── LICENSE
└── rob3_driver/          # ament_python package (clone/symlink into your ROS 2 workspace src/)
```

## Quick start

Drop the package into a ROS 2 workspace and build:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s /path/to/rob3_ros2_driver/rob3_driver .
# the driver depends on the standalone rob3 library:
pip install "rob3[serial] @ git+https://github.com/eurobtec/rob3_py.git"
cd ~/ros2_ws && colcon build --packages-select rob3_driver
```

The protocol codec, transports, and calibration are **ROS-independent** and now
live in the standalone [`rob3`](https://github.com/eurobtec/rob3_py) library.
Install it (the driver depends on it):

```bash
pip install "rob3[serial] @ git+https://github.com/eurobtec/rob3_py.git"
```

Its codec/calibration/client can be used and tested with plain `pytest` (no ROS
install needed) — see that repo.

## Docker

The easiest way to run the driver (with RViz + teleop) is the baked image. It is
**not** published to a registry, so you build it locally **first** from the
[`Dockerfile`](Dockerfile) at the repo root:

```bash
docker build -t rob3-ros2:lyrical .     # build once; stays in your local image store
docker images rob3-ros2                 # confirm the rob3-ros2:lyrical tag exists
```

All the `docker run ... rob3-ros2:lyrical` examples (see
[`rob3_driver/README.md`](rob3_driver/README.md)) use this locally-built image.

## Relationship to the ROB3 firmware repo

The driver's bytes are the reverse-engineered ROB3 low-level protocol,
cross-checked against the **real ROM** running in ucSim. That verification
(the codec-vs-ROM check and the ucSim round-trip) lives with the protocol code
in the [`rob3`](https://github.com/eurobtec/rob3_py) library repo, which needs
the ROB3 firmware repo (for the ROM image) and a ucSim binary.

The protocol/firmware documentation (`hardware/host/command.md`,
`firmware/src/annotated/`) lives in the firmware repo
[eurobtec/rob3](https://github.com/eurobtec/rob3).

## License

MIT (see [LICENSE](LICENSE)).
