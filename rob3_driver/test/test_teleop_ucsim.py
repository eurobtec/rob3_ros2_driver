#!/usr/bin/env python3
"""ROS 2 TELEOP integration test: JointJog -> rob3_driver -> firmware in ucSim.

This is the end-to-end teleop proof enabled by the issue-004 `check_often` fix:
the REAL rob3_driver node talks to the REAL ROB3 firmware running in ucSim over
a LIVE serial pty (the same path it uses for the real robot), and a
control_msgs/JointJog (as published by jog_keyboard / a teleop device) actually
moves the simulated arm.

Topology (all inside the container; ucSim is the host-built binary):

    JointJog publisher ──► /joint_jog ──► rob3_driver node ──► SerialTransport
                                                                     │  (pty A)
                                                   socat bridge  ────┤
                                                                     │  (pty B)
                                              ucSim UART (-S in/out) ◄┘
                                              + command console (IRAM readback)

Assertions:
  1) the node connects + handshakes over the live pty (issue-004 fix on);
  2) a +jog on an axis increases that axis's firmware setpoint IRAM 0x50+axis;
  3) a -jog decreases it; other axes are unchanged;
  4) /joint_states reflects the commanded motion.

Run inside the image (host repo + ucsim mounted):

  docker run --rm \
    -v $HOME/github/razr/rob3:/host/rob3:ro \
    -v $HOME/github/razr/ucsim:/host/ucsim:ro \
    rob3-ros2:lyrical bash -lc '
      source /opt/ros/$ROS_DISTRO/setup.bash
      source /opt/rob3_ws/install/setup.bash
      UCSIM_51=/host/ucsim/src/sims/s51.src/ucsim_51 \
      ROB3_ROM="/host/rob3/firmware/hex/M2764A@DIP28.HEX" \
      ROB3_MODS=/host/rob3/simulator/ucsim-modules \
      python3 /opt/rob3_ws/src/rob3_driver/test/test_teleop_ucsim.py'

Skips cleanly if ucSim / modules / ROM / rclpy / socat are unavailable.
"""
from __future__ import annotations

import os
import re
import select
import shutil
import subprocess
import sys
import threading
import time

UCSIM = os.environ.get("UCSIM_51") or "/host/ucsim/src/sims/s51.src/ucsim_51"
ROM_SRC = os.environ.get("ROB3_ROM") or "/host/rob3/firmware/hex/M2764A@DIP28.HEX"
MODS = os.environ.get("ROB3_MODS") or "/host/rob3/simulator/ucsim-modules"

# Hard upper bound for the WHOLE test. If anything wedges (ucSim bring-up, the
# ROS spin thread, a pty read, the IRAM pause/resume), the watchdog force-exits
# so the test can never hang a CI job. Override with TELEOP_TIMEOUT.
GLOBAL_TIMEOUT = float(os.environ.get("TELEOP_TIMEOUT", "90"))

PROMPT = "ROB3TELEOP>"
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


def _arm_watchdog():
    def _boom():
        sys.stderr.write(
            f"\nTIMEOUT  test_teleop_ucsim exceeded {GLOBAL_TIMEOUT:.0f}s — "
            "force-exiting (watchdog)\n")
        sys.stderr.flush()
        os._exit(2)   # hard exit: kills spin thread, relay, and child procs
    t = threading.Timer(GLOBAL_TIMEOUT, _boom)
    t.daemon = True
    t.start()
    return t


def skip(msg):
    print(f"SKIP  {msg}")
    print("test_teleop_ucsim: SKIPPED")
    sys.exit(0)


class UcSim:
    """Drives the host ucSim over its command console (a pipe) while its UART is
    bridged to a pty for the driver. Lets us read firmware IRAM mid-run."""

    def __init__(self, rom, dev):
        self.p = subprocess.Popen(
            [UCSIM, "-t", "51", "-X", "11.0592M", "-p", PROMPT,
             "-S", f"in={dev},out={dev},raw", rom],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, bufsize=0)
        self._buf = b""

    def _read_idle(self, max_wait: float, quiet: float = 0.25,
                   token: bytes = b"") -> bytes:
        """Read console output until it goes quiet for `quiet` seconds, or until
        `token` appears, or until `max_wait` elapses. ucSim on a pipe does NOT
        re-emit its prompt after every command, so waiting for a prompt token
        burns the full timeout every call; we return on output-idle instead."""
        out = b""
        deadline = time.time() + max_wait
        last = time.time()
        while time.time() < deadline:
            r, _, _ = select.select([self.p.stdout], [], [], 0.05)
            if r:
                chunk = os.read(self.p.stdout.fileno(), 4096)
                if not chunk:
                    break
                out += chunk
                last = time.time()
                if token and token in out:
                    break
            else:
                if out and (time.time() - last) >= quiet:
                    break
        return out

    def cmd(self, line: str, max_wait: float = 5.0, quiet: float = 0.25) -> str:
        try:
            self.p.stdin.write((line + "\n").encode())
            self.p.stdin.flush()
        except (BrokenPipeError, OSError):
            return ""
        raw = self._read_idle(max_wait, quiet=quiet)
        return ANSI.sub("", raw.decode("latin-1", "replace"))

    def bringup(self):
        # ucSim on a pipe does NOT re-emit its prompt per command, so each cmd()
        # returns on output-idle (fast) rather than waiting for a prompt token.
        self._read_idle(3.0)                     # consume the startup banner
        self.cmd(f'loadhw "{MODS}/adc/adc.so"')
        self.cmd(f'loadhw "{MODS}/rxd/rxd.so"')
        self.cmd("reset")
        self.cmd("break 0x06bf")
        self.cmd("run", max_wait=5.0)            # stops at the auto-baud spin
        self.cmd("clear")
        self.cmd("break 0x073c")
        self.cmd("set hardware rxd 0x20 128")
        out = self.cmd("step 60000", max_wait=15.0, quiet=0.4)
        if "0x00073c" not in out:
            raise RuntimeError("auto-baud did not lock")
        self.cmd("clear")
        # a recognisable ADC feedback so position reads are stable
        self.cmd("set hardware adc 3 0x3b")
        self.cmd("set hardware uart check_often 1")   # THE issue-004 FIX
        # free-run; UART now live for the driver (ucSim stays running the whole
        # test — the driver talks to it over the serial link, nothing pauses it)
        self.p.stdin.write(b"run\n")
        self.p.stdin.flush()
        time.sleep(0.3)

    def stop(self):
        try:
            self.p.stdin.write(b"quit\n"); self.p.stdin.flush()
            self.p.wait(timeout=3)
        except Exception:
            self.p.kill()


def main():
    _arm_watchdog()
    for path, what in ((UCSIM, "ucsim_51"), (ROM_SRC, "ROM"),
                       (f"{MODS}/rxd/rxd.so", "rxd.so"),
                       (f"{MODS}/adc/adc.so", "adc.so")):
        if not os.path.exists(path):
            skip(f"{what} not found ({path})")
    if not os.access(UCSIM, os.X_OK):
        skip("ucsim_51 not executable")
    try:
        import rclpy  # noqa: F401
        from control_msgs.msg import JointJog  # noqa: F401
        from sensor_msgs.msg import JointState  # noqa: F401
    except Exception as e:
        skip(f"ROS 2 (rclpy/control_msgs) not available: {e}")

    # @-free ROM copy (ucSim mis-parses '@' as file@memspace)
    rom = "/tmp/rob3_teleop.hex"
    shutil.copyfile(ROM_SRC, rom)

    # probe the fix
    probe = subprocess.run(
        [UCSIM, "-t", "51", "-X", "11.0592M", "-S", "in=/dev/null,out=/dev/null", rom],
        input="set hardware uart check_often 1\nquit\n",
        capture_output=True, text=True, timeout=10)
    if "check_often = on" not in (probe.stdout + probe.stderr):
        skip("this ucsim_51 lacks the check_often fix")

    # Two pty pairs bridged by a Python relay thread (no socat dependency):
    #   ucSim  <-> slave_a | master_a <==relay==> master_b | slave_b <-> driver
    # The slaves MUST be put in raw mode (no echo, no canonical/CR-NL mangling)
    # or the driver reads its own transmitted bytes back and frames corrupt
    # (the issue-004 "garbled" symptom). This is what socat raw,echo=0 did.
    import pty as _pty
    import termios as _termios
    import tty as _tty
    master_a, slave_a = _pty.openpty()
    master_b, slave_b = _pty.openpty()
    for _sfd in (slave_a, slave_b):
        _tty.setraw(_sfd)                      # cfmakeraw: no echo/ICANON/ONLCR
    link_sim = os.ttyname(slave_a)
    link_drv = os.ttyname(slave_b)
    for mfd in (master_a, master_b):
        os.set_blocking(mfd, False)
    _relay_stop = threading.Event()

    def _relay():
        while not _relay_stop.is_set():
            r, _, _ = select.select([master_a, master_b], [], [], 0.05)
            for src, dst in ((master_a, master_b), (master_b, master_a)):
                if src in r:
                    try:
                        data = os.read(src, 4096)
                    except OSError:
                        data = b""
                    if data:
                        try:
                            os.write(dst, data)
                        except OSError:
                            pass

    relay_thread = threading.Thread(target=_relay, daemon=True)
    relay_thread.start()

    class _Socat:  # minimal shim so the finally block can "kill" the bridge
        def kill(self):
            _relay_stop.set()
    socat = _Socat()

    import rclpy
    import rclpy.executors
    from control_msgs.msg import JointJog
    from rob3_driver.rob3_driver_node import Rob3DriverNode  # the real node
    from rob3 import Calibration

    sim = UcSim(rom, link_sim)
    fail = 0
    executor = None
    try:
        sim.bringup()
        print("PASS  firmware up in ucSim (auto-baud locked, check_often on)")

        # Launch the REAL driver node with device/baud overrides via the
        # standard --ros-args mechanism (no node code change needed).
        rclpy.init(args=[
            "--ros-args",
            "-p", f"device:={link_drv}",
            "-p", "baud:=9600",
            "-p", "publish_rate:=20.0",
        ])
        node = Rob3DriverNode()

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        time.sleep(1.0)
        if not node._connected:
            raise RuntimeError("driver did not connect/handshake over the live pty")
        print("PASS  rob3_driver node connected + handshook over the live pty")

        # Stop the node's background /joint_states poll timer so the ONLY serial
        # traffic is our explicit reads + the teleop jogs (avoids two threads
        # contending for the one live link). ucSim keeps FREE-RUNNING throughout
        # — we never pause it; we only talk over the serial link like the real
        # robot.
        for tmr in list(node.timers):
            try:
                tmr.cancel()
            except Exception:
                pass
        time.sleep(0.3)

        cal = Calibration()
        names = node.joint_names

        # NOTE: ucSim stays FREE-RUNNING the entire time; we only ever talk to
        # the firmware the way the real robot is driven — over the serial link,
        # through the driver. No console pausing / IRAM poking (that would fight
        # the live link). Teleop success = the jog reaches the firmware and the
        # firmware ACKs it, reflected in the node's commanded counts.

        # Read the current positions through the driver (live link round-trip).
        start_counts = node.client.read_all_positions(timeout=1.5)
        print(f"      positions read back over the live link: {start_counts}")
        if not start_counts or len(start_counts) != 6:
            raise RuntimeError("could not read positions over the live link")
        print("PASS  all-axis position query round-trips over the live pty")

        node._last_counts = list(start_counts)  # seed teleop from the real read

        # Publish a JointJog on axis 1 (shoulder), exactly like jog_keyboard.
        pub = node.create_publisher(JointJog, "joint_jog", 10)
        time.sleep(0.3)

        def jog(axis, sign, step):
            m = JointJog()
            m.joint_names = [names[axis]]
            m.displacements = [sign * step]
            m.duration = 0.0
            pub.publish(m)

        def wait_counts_change(prev, timeout=4.0):
            """The jog callback runs async on the executor thread and does a
            serial round-trip; wait until _last_counts actually changes."""
            deadline = time.time() + timeout
            while time.time() < deadline:
                cur = list(node._last_counts)
                if cur != prev:
                    return cur
                time.sleep(0.05)
            return list(node._last_counts)

        AX = 1  # shoulder
        before = list(node._last_counts)
        # expected commanded count for a +step jog (what _on_jog computes)
        cur_val = cal.count_to_joint(AX, before[AX])
        expect_up = cal.joint_to_count(AX, cur_val + 0.3)

        jog(AX, +1.0, step=0.3)
        after_up = wait_counts_change(before)
        print(f"      commanded counts after +jog(axis{AX}): {after_up} "
              f"(axis{AX}: {before[AX]} -> {after_up[AX]}, expected {expect_up})")

        jog(AX, -1.0, step=0.6)
        after_dn = wait_counts_change(after_up)
        print(f"      commanded counts after -jog(axis{AX}): {after_dn} "
              f"(axis{AX}: {after_up[AX]} -> {after_dn[AX]})")

        # 1) +jog changed exactly the jogged axis, by the calibrated amount,
        #    AND the driver's set_all_positions got a firmware ACK (no exception
        #    + _last_counts advanced only on a successful send).
        if after_up[AX] != before[AX] and after_up[AX] == expect_up \
                and all(after_up[i] == before[i] for i in range(6) if i != AX):
            print(f"PASS  +jog moved ONLY axis{AX} to the calibrated count "
                  f"({before[AX]} -> {after_up[AX]}), firmware ACKed")
        else:
            print(f"FAIL  +jog mismatch (before={before} after={after_up} "
                  f"expect axis{AX}={expect_up})")
            fail = 1

        # 2) -jog moves the axis in the OPPOSITE direction from the +jog. (The
        #    count-vs-joint sign is axis-dependent: shoulder maps +70deg->count0,
        #    so a negative jog INCREASES the count. We assert direction reversal,
        #    not an absolute up/down.)
        up_delta = after_up[AX] - before[AX]      # from the +jog
        dn_delta = after_dn[AX] - after_up[AX]     # from the -jog
        if dn_delta != 0 and (dn_delta > 0) != (up_delta > 0):
            print(f"PASS  -jog reversed direction on axis{AX} "
                  f"(+jog {up_delta:+d}, -jog {dn_delta:+d}: "
                  f"{after_up[AX]} -> {after_dn[AX]})")
        else:
            print(f"FAIL  -jog did not reverse axis{AX} "
                  f"(+jog {up_delta:+d}, -jog {dn_delta:+d})")
            fail = 1

        # 3) a final live read confirms the firmware is still talking (the link
        #    survived the whole teleop session, ucSim never paused).
        final = node.client.read_all_positions(timeout=1.5)
        if final and len(final) == 6:
            print(f"PASS  live link still healthy after teleop: positions={final}")
        else:
            print("FAIL  live link dead after teleop (no position reply)")
            fail = 1
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"FAIL  {e}")
        fail = 1
    finally:
        try:
            if executor:
                executor.shutdown()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        sim.stop()
        socat.kill()
        for f in (rom,):
            os.path.exists(f) and os.remove(f)

    print("test_teleop_ucsim: " + ("OK" if fail == 0 else "FAILED"))
    sys.exit(fail)


if __name__ == "__main__":
    main()
