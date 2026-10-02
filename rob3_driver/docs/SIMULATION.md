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

## Why serial, not TCP

ucSim can expose its UART as a TCP socket (`-S port=`). We tried that and
**removed** the TCP transport: that socket feeds ucSim's single-byte RX buffer
asynchronously and **drops multi-byte frames** (a query's ETX is lost, so the
firmware never dispatches and just idle-replies `0xF1`). See the ROB3 firmware
repo `simulator/issues/004-async-serial-rx-dropped-or-garbled/`.

The reliable path is ucSim's **serial file/pty** input, which is clocked at the
modeled baud. The driver therefore uses `SerialTransport` for both the real
robot (`/dev/ttyUSB0`) and the simulator (a pty).

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
4. free-run the ROM;
5. print the pty device path.

## Reliability caveat

Live *interactive* RX over the pty is **not fully reliable** in ucSim 0.9.9:
asynchronously-arriving bytes can misalign with the UART bit clock and corrupt a
frame (`simulator/issues/004`). For **deterministic verification**, use the
driver's integration test, which drives the exact same protocol bytes through
ucSim's pre-staged `-S in=<file>` path (baud-paced, reliable):

```bash
UCSIM_51=/path/to/ucsim_51 \
  python3 -m pytest rob3_driver/test/test_sim_roundtrip.py
```

That test brings the ROM up, sends the all-axis position query, and asserts the
firmware dispatches and returns a well-formed reply frame that the driver's
codec parses.

## Environment variables

| Var | Meaning | Default |
| :-- | :------ | :------ |
| `UCSIM_51` | loader-enabled `ucsim_51` | local build path |
| `ROB3_ROM` | ROB3 ROM image | firmware repo `firmware/hex/M2764A@DIP28.HEX` |
| `ROB3_MODS` | dir with the `cl_hw` `.so` plugins | firmware repo `simulator/ucsim-modules` |
