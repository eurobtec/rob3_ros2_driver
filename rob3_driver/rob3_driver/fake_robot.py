"""A byte-level fake ROB3 controller on a PTY — for the self-contained teleop demo.

Lets you drive the real driver + RViz with no hardware and no ucSim. It opens a
pseudo-terminal, writes the slave device path to a file (default
/tmp/rob3_fake_tty), and answers the ROB3 wire protocol:

  * 0x20 (startup)                 -> 0x15 (init OK)
  * query class (0x4x, incl. 0x4F) -> keyword + 6 stored counts + ETX
  * set-all position (0x07/0x0F)   -> store the 6 counts, reply 0xF3 + ETX
  * anything else framed by ETX    -> 0xF3 + ETX (generic ACK)

It also integrates a tiny "plant": stored counts are the position the driver
reads back, so jogging moves the (virtual) joints and RViz reflects it.

Not a ROS node — a plain process. The demo launch starts it, waits for the tty
path, and points the driver at it.
"""
from __future__ import annotations

import argparse
import os
import pty
import time

from . import protocol as P


def run(tty_path_file: str, initial=None):
    counts = list(initial) if initial else [128] * P.NUM_AXES
    master, slave = pty.openpty()
    dev = os.ttyname(slave)
    with open(tty_path_file, "w") as f:
        f.write(dev)
    os.set_blocking(master, False)
    print(f"[fake_robot] ROB3 fake on {dev} (counts={counts})", flush=True)

    buf = bytearray()

    def reply(data: bytes):
        try:
            os.write(master, data)
        except OSError:
            pass

    def handle(frame: bytes):
        header = frame[0]
        if (header & 0x80) == 0 and (header & 0x40):
            # query class (single 0x40|a, or all 0x4F): echo header + counts
            axis = header & 0x07
            if axis == P.ALL_AXES:
                reply(bytes([header]) + bytes(counts) + bytes([P.ETX]))
            else:
                reply(bytes([header, counts[axis], P.ETX]))
        elif (header & 0x80) == 0 and (header & 0x40) == 0 \
                and (header & 0x07) == P.ALL_AXES:
            # set-all position (0x07 / 0x0F): operands are the 6 counts
            for i in range(P.NUM_AXES):
                if 1 + i < len(frame) - 1:
                    counts[i] = frame[1 + i]
            reply(bytes([P.STATUS_ACK, P.ETX]))
        elif (header & 0x80) == 0 and (header & 0x40) == 0 \
                and (header & 0x07) < P.NUM_AXES and len(frame) >= 3:
            # set single-axis position: header(+0x08 ack) axis in low 3 bits
            axis = header & 0x07
            counts[axis] = frame[1]
            reply(bytes([P.STATUS_ACK, P.ETX]))
        else:
            reply(bytes([P.STATUS_ACK, P.ETX]))

    while True:
        try:
            data = os.read(master, 64)
        except (BlockingIOError, OSError):
            data = b""
        for b in data:
            if b == P.SPACE and not buf:
                reply(bytes([P.REPLY_INIT_OK]))
                continue
            buf.append(b)
            if b == P.ETX:
                handle(bytes(buf))
                buf.clear()
        time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser(description="Fake ROB3 controller on a PTY")
    ap.add_argument("--tty-file", default="/tmp/rob3_fake_tty",
                    help="file to write the slave pty device path to")
    args = ap.parse_args()
    try:
        run(args.tty_file)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
