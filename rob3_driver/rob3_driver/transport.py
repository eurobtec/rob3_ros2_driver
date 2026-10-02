"""Serial transport for the ROB3 link.

The ROB3 controller uses RS-232. The driver speaks the same bytes to the real
robot (`/dev/ttyUSB0`) and to the ucSim simulator — in the simulator case ucSim
is attached to a pty/file with `-S in=<dev>,out=<dev>` and the driver opens the
other end as a serial device, so the code path is identical.

Interface:
    open() / close() / write(bytes) / read(n, timeout) / read_until(term, timeout)

No ROS dependencies here so the transport is unit-testable and reusable.

Note on ucSim: a TCP-socket transport (`-S port=`) was tried and removed — that
socket feeds ucSim's single-byte RX buffer asynchronously and drops frames. The
paced serial file/pty path (`-S in=/out=`) is the one that round-trips cleanly;
see docs/SIMULATION.md for the bring-up.
"""
from __future__ import annotations

import time
from typing import Protocol


class Transport(Protocol):
    def open(self) -> None: ...
    def close(self) -> None: ...
    def write(self, data: bytes) -> None: ...
    def read(self, n: int, timeout: float = 1.0) -> bytes: ...
    def read_until(self, term: int, timeout: float = 1.0, limit: int = 256) -> bytes: ...


class SerialTransport:
    """RS-232 via pyserial. 9600 8N1, no flow control (the ROB3 firmware
    auto-bauds off the first 0x20). Works against the real robot
    (`/dev/ttyUSB0`) and against a ucSim-attached pty. pyserial is imported
    lazily so the package can be built/tested without it."""

    def __init__(self, device: str = "/dev/ttyUSB0", baud: int = 9600):
        self.device = device
        self.baud = baud
        self._ser = None

    def open(self) -> None:
        import serial  # lazy import; only needed to actually talk to a device

        self._ser = serial.Serial(
            port=self.device,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.0,          # non-blocking; we poll with our own timeouts
            rtscts=False,
            xonxoff=False,
        )

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def write(self, data: bytes) -> None:
        assert self._ser is not None, "transport not open"
        self._ser.write(data)
        self._ser.flush()

    def read(self, n: int, timeout: float = 1.0) -> bytes:
        assert self._ser is not None, "transport not open"
        out = bytearray()
        deadline = time.monotonic() + timeout
        while len(out) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(out))
            if chunk:
                out.extend(chunk)
            else:
                time.sleep(0.002)
        return bytes(out)

    def read_until(self, term: int, timeout: float = 1.0, limit: int = 256) -> bytes:
        assert self._ser is not None, "transport not open"
        out = bytearray()
        deadline = time.monotonic() + timeout
        while len(out) < limit and time.monotonic() < deadline:
            chunk = self._ser.read(1)
            if chunk:
                out.extend(chunk)
                if chunk[0] == term:
                    break
            else:
                time.sleep(0.002)
        return bytes(out)


def make_transport(kind: str = "serial", **kw) -> Transport:
    """Factory. Only 'serial' is supported (the real link and the ucSim pty)."""
    kind = kind.lower()
    if kind == "serial":
        return SerialTransport(device=kw.get("device", "/dev/ttyUSB0"),
                               baud=int(kw.get("baud", 9600)))
    raise ValueError(
        f"unknown transport kind: {kind!r} (only 'serial' is supported)")

