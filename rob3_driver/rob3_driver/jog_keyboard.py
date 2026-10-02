"""Keyboard teleop for the ROB3 arm — publishes control_msgs/JointJog.

Jog one joint at a time from the terminal; the driver converts the delta to a
position setpoint and the arm (or ucSim) moves, with RViz reflecting it via
/joint_states.

Keys:
  1..6     select axis (base, shoulder, elbow, wrist_pitch, wrist_roll, gripper)
  + / =    jog selected joint by +step
  - / _    jog selected joint by -step
  [ / ]    decrease / increase the step size
  0        print current selection
  q        quit

Revolute steps are in radians; the gripper (axis 6) step is in metres.
"""
from __future__ import annotations

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog

JOINTS = ["base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"]
HELP = __doc__


class JogKeyboard(Node):
    def __init__(self):
        super().__init__("rob3_jog_keyboard")
        self.declare_parameter("joint_prefix", "")
        self.prefix = self.get_parameter("joint_prefix").value or ""
        self.pub = self.create_publisher(JointJog, "joint_jog", 10)
        self.axis = 0
        self.step = 0.05           # rad (revolute) — gripper uses step/10 as m
        self.get_logger().info(HELP)
        self._announce()

    def _announce(self):
        name = JOINTS[self.axis]
        unit = "m" if self.axis == 5 else "rad"
        step = self.step / 10.0 if self.axis == 5 else self.step
        self.get_logger().info(
            f"axis {self.axis + 1} = {name}  step={step:.4f} {unit}")

    def jog(self, sign: float):
        name = self.prefix + JOINTS[self.axis]
        step = self.step / 10.0 if self.axis == 5 else self.step
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [name]
        msg.displacements = [sign * step]
        msg.duration = 0.0
        self.pub.publish(msg)


def _getkey() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = JogKeyboard()
    try:
        while rclpy.ok():
            k = _getkey()
            if k in ("q", "\x03"):          # q or Ctrl-C
                break
            elif k in "123456":
                node.axis = int(k) - 1
                node._announce()
            elif k in ("+", "="):
                node.jog(+1.0)
            elif k in ("-", "_"):
                node.jog(-1.0)
            elif k == "]":
                node.step *= 1.5
                node._announce()
            elif k == "[":
                node.step /= 1.5
                node._announce()
            elif k == "0":
                node._announce()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
