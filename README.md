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
cd ~/ros2_ws && colcon build --packages-select rob3_driver
```

The protocol codec, transports, and calibration are **ROS-independent** and can
be used / tested with plain `pytest` (no ROS install needed):

```bash
cd rob3_driver && python3 -m pytest test/test_protocol.py test/test_calibration.py test/test_client_fake.py
```

## Relationship to the ROB3 firmware repo

This driver is standalone, but one integration test cross-checks the driver's
encoded bytes against the **real ROM** running in ucSim:
`rob3_driver/test/test_driver_protocol_vs_rom.sh`. That test needs the ROB3
firmware repo (for the ROM image) and a ucSim binary; it takes the ROM via the
`SAFEHEX` environment variable, e.g.:

```bash
SAFEHEX=/path/to/rob3/simulator/build/rob3.hex SIM=ucsim_51 \
  rob3_driver/test/test_driver_protocol_vs_rom.sh
```

The protocol/firmware documentation (`hardware/host/command.md`,
`firmware/src/annotated/`) also lives in that firmware repo.

## License

MIT (see [LICENSE](LICENSE)).
