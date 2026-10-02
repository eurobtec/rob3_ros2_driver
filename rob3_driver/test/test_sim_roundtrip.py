"""ucSim integration test: the driver's protocol over the real ROM serial path.

Brings the ROB3 ROM up in ucSim to the serial auto-baud lock (via the `rxd`
cl_hw module), then exchanges the driver's own encoded command frames over
ucSim's **pre-staged file** serial path (`-S in=<file>,out=<file>`) — the
reliable, baud-paced path (see simulator/issues/004 for why the socket/pty
paths are not used). Asserts the firmware dispatches the frame and replies with
a well-formed frame that the driver's codec parses.

Skips cleanly without a loader-enabled ucsim_51 + the adc/rxd modules + the ROM.
No ROS required; drives the pure-Python codec against the real ROM in ucSim.
"""
import os
import pty
import re
import select
import shutil
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)  # .../rob3_driver (holds the rob3_driver/ package)
sys.path.insert(0, PKG)

import rob3_driver.protocol as P  # noqa: E402

UCSIM = os.environ.get(
    "UCSIM_51",
    os.path.expanduser("~/github/razr/ucsim/src/sims/s51.src/ucsim_51"))
MODS = os.environ.get(
    "ROB3_MODS",
    os.path.expanduser("~/github/razr/rob3/simulator/ucsim-modules"))
ROM_SRC = os.environ.get(
    "ROB3_ROM",
    os.path.expanduser("~/github/razr/rob3/firmware/hex/M2764A@DIP28.HEX"))

PROMPT = "ITEST>"
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")

_have = (
    (shutil.which(UCSIM) or os.path.exists(UCSIM))
    and os.path.exists(f"{MODS}/rxd/rxd.so")
    and os.path.exists(f"{MODS}/adc/adc.so")
    and os.path.exists(ROM_SRC)
)
requires_sim = pytest.mark.skipif(
    not _have, reason="loader-enabled ucsim_51 + adc/rxd modules + ROM required")


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


@requires_sim
def test_query_roundtrip_over_rom_serial():
    rom = "/tmp/rob3_itest.hex"
    shutil.copyfile(ROM_SRC, rom)
    IN = "/tmp/rob3_itest_in"
    OUT = "/tmp/rob3_itest_out"
    # Pre-stage ONLY the query frame; the auto-baud training byte is supplied by
    # the rxd module (so the host must not also send 0x20).
    with open(IN, "wb") as f:
        f.write(P.query_all_positions())      # 0x4F 0x03
    open(OUT, "wb").close()

    pid, cfd = pty.fork()
    if pid == 0:
        os.execv(UCSIM, [
            UCSIM, "-t", "51", "-X", "11.0592M", "-p", PROMPT,
            "-S", f"in={IN},out={OUT}", rom])
        os._exit(127)
    try:
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
        assert "0x00073c" in out, "auto-baud did not lock"
        _cmd(cfd, "clear")
        _cmd(cfd, "set hardware adc 3 0x3b")   # a recognisable feedback value
        _cmd(cfd, "step 2000000", timeout=30.0)
        _cmd(cfd, "step 2000000", timeout=30.0)
    finally:
        os.write(cfd, b"quit\n")
        time.sleep(0.1)
        os.close(cfd)
        os.waitpid(pid, os.WNOHANG)

    data = open(OUT, "rb").read()
    os.remove(IN)
    os.remove(OUT)
    assert data, "no serial output from the ROM"

    # Parse from the echoed query keyword (0x4F); a leading 0x15 init-ack may
    # precede it.
    start = data.find(0x4F)
    assert start >= 0, f"no 0x4F reply keyword in {data.hex()}"
    reply = P.parse_reply(data[start:])
    assert reply.ok, f"reply not ETX-terminated: {data.hex()}"
    assert (reply.keyword & 0xFF) == 0x4F
    positions = P.parse_positions(reply)
    assert len(positions) == P.NUM_AXES
    assert all(0 <= v <= 255 for v in positions)
