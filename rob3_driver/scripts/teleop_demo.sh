#!/usr/bin/env bash
# Self-contained ROB3 teleop demo — NO hardware, NO ucSim.
#
# Starts a fake ROB3 controller on a pty, the real driver pointed at it, RViz
# over noVNC, and the keyboard jog node. Press number keys to pick an axis and
# +/- to jog; watch the arm move in RViz at http://localhost:6080/vnc.html.
#
# Meant to run INSIDE the baked image, as a single command:
#   docker run --rm -it -p 6080:6080 rob3-ros2:lyrical \
#     ros2 run rob3_driver teleop_demo.sh
set -e

TTY_FILE=/tmp/rob3_fake_tty
rm -f "$TTY_FILE"

# 1) virtual display + noVNC
Xvfb :99 -screen 0 1400x900x24 >/tmp/xvfb.log 2>&1 &
export DISPLAY=:99
x11vnc -display :99 -forever -shared -nopw -rfbport 5900 -listen 0.0.0.0 \
  >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/ws.log 2>&1 &

# 2) fake robot on a pty
ros2 run rob3_driver rob3_fake_robot --tty-file "$TTY_FILE" >/tmp/fake.log 2>&1 &
for _ in $(seq 1 50); do [ -s "$TTY_FILE" ] && break; sleep 0.1; done
DEV="$(cat "$TTY_FILE")"
echo "[demo] fake robot at $DEV"

# 3) driver + robot_state_publisher + RViz
ros2 launch rob3_driver rob3.launch.py rviz:=true device:="$DEV" \
  >/tmp/launch.log 2>&1 &
sleep 8
grep -iE "handshake|up \(" /tmp/launch.log | tail -2 || true

echo
echo "========================================================================"
echo " ROB3 teleop demo is up."
echo "  * Open RViz:  http://localhost:6080/vnc.html?autoconnect=1&resize=remote"
echo "  * Jog keys :  1..6 select axis, +/- jog, [ ] step size, q quit"
echo "========================================================================"
echo

# 4) keyboard jog in the foreground (so stdin works)
exec ros2 run rob3_driver rob3_jog_keyboard
