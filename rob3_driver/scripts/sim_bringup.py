#!/usr/bin/env python3
"""Bring up the ROB3 firmware in ucSim on a serial PTY for the ROS 2 driver.

This automates the simulator counterpart of "plug in the robot":

  1. create a PTY pair,
  2. launch ucSim with its UART attached to the PTY (-S in=<pty>,out=<pty>,raw),
  3. load the adc + rxd cl_hw plugins,
  4. lock the firmware's software auto-baud by shifting the training byte 0x20
     on P3.0 via the rxd module,
  5. free-run the ROM,
  6. print the PTY device path and hold the session open.

Then point the driver at the printed device:

    ros2 launch rob3_driver rob3.launch.py device:=/dev/pts/N

Why a PTY (vs ucSim's `-S port=` TCP socket): either works once ucSim's
`check_often` flag is on; this helper uses a pty so the driver opens it as a
plain serial device, exactly like the real robot.

NOTE (ROB3 firmware repo simulator/issues/004): live serial over a pty/socket
used to lose frames because ucSim polled the host fd too rarely during a free
`run` (longer than the firmware's RX timeout). This helper now enables ucSim's
`check_often` flag after lock (`set hardware uart check_often 1`), which drains
the fd every serial tick and makes live multi-byte round-trips reliable. It was
never a real-hardware issue (the firmware timeout is ~50x the on-wire byte gap).
A fully deterministic alternative remains the pre-staged `-S in=<file>` path
used by test/test_sim_roundtrip.py.

Env:
  UCSIM_51   path to a loader-enabled ucsim_51 (default: the local build)
  ROB3_ROM   ROB3 ROM image (default: the firmware repo path)
  ROB3_MODS  dir holding the cl_hw .so plugins (default: firmware repo simulator/)
"""
import os
import pty
import re
import select
import shutil
import signal
import sys
import time

UCSIM = os.environ.get(
    "UCSIM_51",
    os.path.expanduser("~/github/razr/ucsim/src/sims/s51.src/ucsim_51"))
ROM_SRC = os.environ.get(
    "ROB3_ROM",
    os.path.expanduser("~/github/razr/rob3/firmware/hex/M2764A@DIP28.HEX"))
MODS = os.environ.get(
    "ROB3_MODS",
    os.path.expanduser("~/github/razr/rob3/simulator/ucsim-modules"))

PROMPT = "ROB3SIM>"
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _read(fd, token, timeout):
    buf = ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        r, _, _ = select.select([fd], [], [], 0.1)
        if not r:
            continue
        try:
            data = os.read(fd, 4096)
        except OSError:
            break
        if not data:
            break
        buf += data.decode("latin-1", "replace")
        if token in buf:
            break
    return ANSI.sub("", buf)


def _cmd(fd, line, timeout=5.0):
    os.write(fd, (line + "\n").encode())
    return _read(fd, PROMPT, timeout)


def main():
    if not (os.path.exists(UCSIM) and os.access(UCSIM, os.X_OK)):
        sys.exit(f"ucsim_51 not found/executable: {UCSIM} (set UCSIM_51)")
    if not os.path.exists(ROM_SRC):
        sys.exit(f"ROB3 ROM not found: {ROM_SRC} (set ROB3_ROM)")

    rom = "/tmp/rob3_sim.hex"
    shutil.copyfile(ROM_SRC, rom)  # @-free name for ucSim

    ser_master, ser_slave = pty.openpty()
    dev = os.ttyname(ser_slave)

    pid, cfd = pty.fork()
    if pid == 0:  # child -> ucSim
        os.execv(UCSIM, [
            UCSIM, "-t", "51", "-X", "11.0592M", "-p", PROMPT,
            "-S", f"in={dev},out={dev},raw", rom])
        os._exit(127)

    _read(cfd, PROMPT, 5.0)
    _cmd(cfd, f'loadhw "{MODS}/adc/adc.so"')
    _cmd(cfd, f'loadhw "{MODS}/rxd/rxd.so"')
    _cmd(cfd, "reset")
    _cmd(cfd, "break 0x06bf")
    _cmd(cfd, "run", timeout=5.0)
    _cmd(cfd, "clear")
    _cmd(cfd, "break 0x073c")
    _cmd(cfd, "set hardware rxd 0x20 128")
    out = _cmd(cfd, "step 60000", timeout=15.0)
    locked = "0x00073c" in out
    _cmd(cfd, "clear")
    # Enable live-serial fd draining (issue-004 fix). Harmless if the ucsim_51
    # build predates it (the command is simply rejected).
    _cmd(cfd, "set hardware uart check_often 1")
    os.write(cfd, b"run\n")  # free-run; UART now live

    print(f"ucSim up (pid {pid}); auto-baud {'LOCKED' if locked else 'NOT locked'}")
    print(f"serial device: {dev}")
    print("point the driver at it, e.g.:")
    print(f"  ros2 launch rob3_driver rob3.launch.py device:={dev}")
    print("Ctrl-C to stop.")

    def _stop(*_):
        try:
            os.write(cfd, b"quit\n")
            time.sleep(0.1)
            os.close(cfd)
        except OSError:
            pass
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
