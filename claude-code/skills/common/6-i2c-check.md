# Common 6: I2C check (back-to-back transfers) and retry workaround

Run it after the drivers probe (common/5 §Probe check) and before configuring the pipeline
(common/7), and whenever streaming fails with `-ETIMEDOUT` (`-110`) or `-EIO` (`-5`) at a
different step each time.

**A clean result does not rule out I2C problems:** failures that happen only during probe and
stream start (last row of §Results) do not show in idle tests.

Why back to back: kernel drivers send register accesses back to back, microseconds apart; single
`i2ctransfer` calls are milliseconds apart and hide timing problems. `i2cburst.c`
(`<skill dir>/common/i2cburst.c`) alternates same-value writes and reads of one register, back to
back, and counts errors; it only writes the value it just read.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save each `i2cburst` output as `<run_dir>/board/i2cburst_<n>.txt`, and the retry counters if the workaround is used (§Verbose mode §What to save).

## Run

Compile `i2cburst` on the board if it has `gcc`, and run it on the deserializer, the serializer
(through the link) and the sensor:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "mkdir -p ~/Workspace"
scp -o ControlPath=/tmp/gmsl_ssh_sock <skill dir>/common/i2cburst.c <user>@<ip>:Workspace/
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace && gcc -O2 -o i2cburst i2cburst.c && \
  ./i2cburst <i2c_bus> <des_addr> <des_reg> 2000; ./i2cburst <ser_bus> <ser_addr> <ser_reg> 2000"
```

**No `gcc` on the board** (common on vendor images, e.g. NXP's Yocto images): cross-compile it on
the host as a static binary (no dependency on the board's C library) and copy only the binary:
```bash
mkdir -p /tmp/gmsl_i2cburst
aarch64-linux-gnu-gcc -static -O2 -o /tmp/gmsl_i2cburst/i2cburst <skill dir>/common/i2cburst.c
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "mkdir -p ~/Workspace"
scp -o ControlPath=/tmp/gmsl_ssh_sock /tmp/gmsl_i2cburst/i2cburst <user>@<ip>:Workspace/
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace && chmod +x i2cburst && \
  ./i2cburst <i2c_bus> <des_addr> <des_reg> 2000; ./i2cburst <ser_bus> <ser_addr> <ser_reg> 2000"
```
(`aarch64-linux-gnu-gcc` is in the host tools of common/0 §Host tools, package
`gcc-aarch64-linux-gnu`; for a non-arm64 board use that board's cross-compiler prefix, e.g. the
`toolchain_prefix` in `<state>`.)

Arguments: `i2cburst <bus> <addr> <reg16> <count> [gap_us]`.
- `<i2c_bus>`: the number of the camera I2C bus (`i2cdetect -l`); `<ser_bus>`/`<sensor bus>`: the
  bus the serializer or sensor appears on once the drivers are bound (the deserializer's I2C ATR
  creates one adapter per link: find it with `i2cdetect -l` and
  `ls /sys/bus/i2c/devices/`), with the address the device has there.
- Registers without side effects: deserializer `0x0050`, serializer `0x0383` (MAX96717), sensor
  chip ID `0x0000`.
- The sensor is often powered off between uses (runtime PM): power it first with
  `echo on > /sys/bus/i2c/devices/<sensor dev>/power/control` (with sudo), run the test, and set
  it back to `auto` after:
  ```bash
  ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S sh -c 'echo on > /sys/bus/i2c/devices/<sensor dev>/power/control'; cd ~/Workspace && ./i2cburst <sensor_bus> <sensor_addr> 0x0000 2000; echo '<pw>' | sudo -S sh -c 'echo auto > /sys/bus/i2c/devices/<sensor dev>/power/control'"
  ```
- `i2cburst` uses `I2C_RDWR`, which works while the drivers are bound (unlike `i2ctransfer`
  without `-f`).
- The optional fifth argument pauses between transfers (µs): the **gap argument**. A failure
  rate that drops to zero with a gap of 50–200 µs shows that the pass-through needs time between
  transfers, which the drivers do not give it.

`i2cburst` and its source stay in `~/Workspace` (the board is left deployed); delete the host's
`/tmp/gmsl_i2cburst` in the orchestrator's §Cleanup.

## Results

| Result | Cause and fix |
|--------|---------------|
| All OK | Continue |
| Failures on the serializer or the sensor (tens of %), bus faster than 100 kHz | The GMSL I2C pass-through needs time between transfers. Confirm with `./i2cburst ... 400 <gap_us>` (gaps of 50, 100, 200 µs: failures stop with a gap), then set the bus to 100 kHz as in common/4 §Bus speed and reboot |
| Failures on the deserializer itself, or still failures at 100 kHz | Measure with gaps as above. If the deserializer EV kit's µC is connected (common/3 §EV kit checks), a test with its I2C jumpers removed shows whether it is involved (ask first; the user does it, power off first; put them back if it makes no difference). Otherwise check the cable, the adapter switches and the power order |
| Clean idle tests, but failures during probe and stream start (one `-110`, then a few `-5`, mostly on sensor PLL/clock registers and right after serdes reconfiguration) | Not reproducible from user space; seen at 100 kHz with and without the EV kit's µC. A logic analyzer on SCL/SDA shows the cause. As a workaround, and only after the user agrees, retry failed transfers once in the drivers: §Retry workaround |

## Retry workaround

Only for the last row of §Results, and only after the user agrees (AskUserQuestion, header:
"I2C retry"). Tell the user before asking that it **hides the problem**: the transfers still
fail, the drivers only repeat them; the cause (seen with a logic analyzer on SCL/SDA) still needs
fixing in hardware or timing. Record the answer, so the final message lists it as a workaround.

`<skill dir>/common/i2c_retry_regmap.h` is a reference implementation: a regmap without a bus,
whose `reg_read`/`reg_write` retry a transfer that times out or is not acknowledged
(`I2C_RETRY_COUNT`, default 1 retry, 1–2 ms apart), for devices with 16-bit registers and 8-bit
values (the ADI serializers/deserializers and most CCI sensors). It needs the driver sources, so it
is applied where the drivers are built (common/9 §Retry patch); a kernel whose GMSL drivers are
built in cannot get it without a kernel rebuild (RPi: rpi/1 and rpi/2).

In each driver's git repository (common/9 §ADI drivers, §Sensor driver):
1. Copy the header next to the driver sources and include it: `#include "i2c_retry_regmap.h"`.
2. Replace the regmap setup:
   - deserializer driver (`max9296a.c` or `max96724.c`) and serializer driver (`max96717.c`):
     `devm_regmap_init_i2c(client, &cfg)` → `i2c_retry_devm_regmap_init(client, &cfg)`;
   - sensor driver (a patched copy built out of tree, as for `.get_frame_desc`, common/9
     §Sensor driver): `devm_cci_regmap_init_i2c(client, 16)` →
     `i2c_retry_devm_regmap_init(client, &cfg)` with
     `{ .reg_bits = 16, .val_bits = 8, .disable_locking = true }` (what
     `devm_cci_regmap_init_i2c()` uses).
3. Replace raw `i2c_transfer()` calls (`max_ser_read_reg`/`max_ser_write_reg` in `max_ser.c`)
   with `i2c_retry_transfer(adapter, msgs, num)`; it returns 0 or a negative errno (not the
   number of messages), so adapt the return check.
4. `i2c_retry_devm_regmap_init()` refuses other register widths (`-EINVAL`): check the driver's
   `regmap_config` first. Raw regmap I/O (`regmap_raw_read/write`) is not supported; grep for it.
5. **One commit per driver**, with a message marking it as a workaround
   (`WORKAROUND: retry failed I2C transfers once (hides an I2C problem)`).
6. Build, install and reboot as in common/9, then run the stream start again.

Recoveries are logged: every failure and every recovery prints an `i2c-retry:` line in `dmesg`
(adapter, address, register, and the running count of recovered transfers per driver). Report how
many per boot and per stream start:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "dmesg | grep -c 'i2c-retry:.*OK after'; dmesg | grep 'i2c-retry:' | tail -5"
```
If transfers still fail after the retry (a `failed` line without a following `OK after`), the
workaround is not enough: say so and stop at the frame check's result. Undo: install the drivers
built without the patch (or remove the commits and rebuild).
