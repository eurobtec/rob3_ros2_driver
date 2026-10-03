# Running the driver against the ucSim simulator

The ROB3 ROS 2 driver talks RS-232 to the robot. You can run it against the
**ucSim** simulation of the 8031 firmware instead of real hardware — it uses the
*same* `SerialTransport`, so there is no sim-specific code path in the driver.

This needs the **ROB3 firmware repo** (for the ROM and the compiled `cl_hw`
peripheral modules) and a **loader-enabled `ucsim_51`** (one that supports
`loadhw`).

## TL;DR

```bash
# 1) bring the ROM up on a serial pty (loads adc + rxd modules, locks auto-baud)
UCSIM_51=/path/to/ucsim_51 \
  python3 rob3_driver/scripts/sim_bringup.py
# -> prints:  serial device: /dev/pts/N

# 2) point the driver at that device (another terminal)
ros2 launch rob3_driver rob3.launch.py device:=/dev/pts/N
```

## Live serial works now (the `check_often` ucSim fix)

Driving ucSim **live** over a serial pty/socket used to be unreliable: during a
free `run`, ucSim polled the host serial fd only ~every 1.84M machine-cycles,
which is **longer** than the firmware's ~1.47M-cycle RX timeout — so the two
bytes of a command frame arrived on different polls and the firmware timed out
between them, idle-replying `0xF1`. (Root cause + analysis: ROB3 firmware repo
`simulator/issues/004-async-serial-rx-dropped-or-garbled/`.)

This is **fixed** by a one-line ucSim addition that exposes its existing
per-UART `check_often` flag as a runtime command:

```
set hardware uart check_often 1
```

With it, ucSim drains the serial fd on **every** serial tick, so queued bytes
are picked up well inside the firmware's timeout and live multi-byte frames
dispatch. The patch lives in the firmware repo
(`simulator/issues/004-*/fix-check_often-set_cmd.patch`); rebuild `ucsim_51`
with it. `sim_bringup.py` enables the flag automatically.

> This was never a real-hardware problem: at the locked baud two bytes are ~ms
> apart on the wire, 50x inside the firmware's generous timeout (which exists to
> recover from a host that stops mid-frame). It is purely a ucSim run-loop
> polling artifact that only shows up on a live, async, multi-byte round-trip
> under a free `run` — i.e. exactly what a hardware-style driver does.

## Why serial (the driver uses one transport for robot + sim)

The driver uses `SerialTransport` for both the real robot (`/dev/ttyUSB0`) and
the simulator (a pty), so there is no sim-specific code path. ucSim can also
expose its UART as a TCP socket (`-S port=`); with the `check_often` fix the
socket round-trips too, but the pty path is what `sim_bringup.py` sets up because
it lets the driver open it as a plain serial device.

## The auto-baud bring-up (why a plain launch isn't enough)

The ROB3 firmware does **software auto-baud**: it polls the raw RXD (P3.0) pin
and times the first byte's bit edges before enabling the UART. ucSim's core UART
never drives that pin (`simulator/issues/003`), so the sim needs the **`rxd`**
`cl_hw` module to shift the training byte `0x20` on P3.0 and lock the baud, plus
the **`adc`** module so the servo ISR lets init complete. `sim_bringup.py` does
this choreography for you:

1. create a pty pair; attach ucSim's UART to it (`-S in=<pty>,out=<pty>,raw`);
2. `loadhw adc.so`, `loadhw rxd.so`;
3. run to the auto-baud pin-poll (`0x06BF`), `set hardware rxd 0x20 128` to lock;
4. `set hardware uart check_often 1` (the live-serial fix);
5. free-run the ROM;
6. print the pty device path.

## Verification

Two integration tests cover the sim path:

```bash
# (a) deterministic protocol round-trip over the pre-staged file path
#     (works with stock ucSim; no check_often needed)
UCSIM_51=/path/to/ucsim_51 \
  python3 -m pytest rob3_driver/test/test_sim_roundtrip.py

# (b) LIVE teleop: control_msgs/JointJog -> driver -> firmware in free-running
#     ucSim, over a live pty (needs the check_often-enabled ucsim_51)
#     Run inside the rob3-ros2 image; see test/test_teleop_ucsim.py header.
python3 rob3_driver/test/test_teleop_ucsim.py
```

Test (b) brings the ROM up, connects the real driver node, publishes a JointJog
on an axis, and asserts the jog reaches the firmware (calibrated count + ACK)
over the live link — the end-to-end teleop proof enabled by the fix.

## Environment variables

| Var | Meaning | Default |
| :-- | :------ | :------ |
| `UCSIM_51` | loader-enabled `ucsim_51` | local build path |
| `ROB3_ROM` | ROB3 ROM image | firmware repo `firmware/hex/M2764A@DIP28.HEX` |
| `ROB3_MODS` | dir with the `cl_hw` `.so` plugins | firmware repo `simulator/ucsim-modules` |
