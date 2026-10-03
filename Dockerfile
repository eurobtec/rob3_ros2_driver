# ROB3 ROS 2 driver — reproducible build/run image.
#
# Bakes a coherent ROS 2 Lyrical environment so you don't re-apt on every run:
#   - full apt upgrade of the base image's ros-lyrical-* to one coherent build
#     set (the stock osrf/ros:lyrical-desktop ships a frozen set; control_msgs
#     from apt is newer and needs service_msgs/builtin_interfaces upgraded too,
#     else you hit `undefined symbol: has_buffer_fields_*`),
#   - control_msgs (JointJog + FollowJointTrajectory), xacro,
#     joint_state_publisher, and pyserial (real-HW transport),
#   - the rob3_driver package pre-built into /opt/rob3_ws.
#
# Build (from the repo root, the dir containing rob3_driver/):
#   docker build -t rob3-ros2:lyrical .
#
# Run RViz (noVNC on :6080) — see README "Teleop (jog the arm) + RViz".
FROM osrf/ros:lyrical-desktop

SHELL ["/bin/bash", "-c"]

# One coherent upgrade + the extra ROS packages this driver needs.
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get upgrade -y \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-lyrical-control-msgs \
      ros-lyrical-xacro \
      ros-lyrical-robot-state-publisher \
      ros-lyrical-joint-state-publisher \
      python3-pip \
      python3-serial \
      git \
      xvfb x11vnc novnc websockify \
 && rm -rf /var/lib/apt/lists/*

# Build the driver into a workspace layer.
WORKDIR /opt/rob3_ws/src/rob3_driver
COPY rob3_driver/ ./

# The ROS-independent protocol/transport/calibration/client core lives in the
# standalone `rob3` library (https://github.com/eurobtec/rob3_py). Install it
# (with pyserial for the real-HW transport) before building the ament package.
RUN python3 -m pip install --no-cache-dir --break-system-packages \
      "rob3[serial] @ git+https://github.com/eurobtec/rob3_py.git"

WORKDIR /opt/rob3_ws
RUN source /opt/ros/lyrical/setup.bash \
 && colcon build

# Source ROS + the workspace for every shell (login and non-login).
RUN printf 'source /opt/ros/lyrical/setup.bash\nsource /opt/rob3_ws/install/setup.bash\n' \
      > /etc/profile.d/rob3.sh \
 && cat /etc/profile.d/rob3.sh >> /etc/bash.bashrc

ENV LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
WORKDIR /opt/rob3_ws
CMD ["bash"]
