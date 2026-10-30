# NXP 5: GMSL camera setup on a running NXP i.MX board

Bring up the GMSL cameras on an NXP i.MX board that runs NXP's BSP (Yocto i.MX Distro) and is
reachable over SSH: ADI modules (nxp/1, if missing), device tree (nxp/2), U-Boot `fdtfile`
(nxp/3), probe and I2C checks, the media pipeline with `media-ctl`, a frame check, and a
GStreamer viewer on the board's display.

Run after common/1 (board found, logged in), common/2 (assessment, `platform` = `nxp`) and
common/3 (run by the orchestrator with this file's §Hardware choices; on the build path the
questions were asked before the image was built, so the orchestrator only confirms the stored
answers, and §EV kit checks, §I2C probe, §Path check and §Driver check run now). Placeholders as in the orchestrator; `<tpl_dir>` = `/tmp/gen_gmsl_dts/nxp`. Root's home
on NXP images is `/root`: use `~` / `$HOME` (`board_home`), never `/home/<user>`.

## Profile

- **Platform:** NXP i.MX boards running NXP's BSP: compatible `fsl,imx*`, and the kernel release
  hash is an `lf-*` tag of https://github.com/nxp-imx/linux-imx or `u-boot,version` contains
  `lf_v` (common/2 §Platform selection). i.MX on a mainline kernel is a generic board (generic/0).
- **Tested:** FRDM-IMX95-PRO (`fsl,frdm-imx95-pro`, i.MX95 rev 2.0), NXP i.MX Linux
  6.18.20-2.0.0 (i.MX Distro 6.18-wrynose, Weston), with MAX96716A EV kit + AD-GMSL717MIPI-EVK
  (MAX96717) + IMX219 (Pi Camera v2) through AD-GMSLCAMRPI-ADP on J18: 1920x1080 RAW10 frames,
  live view at 640x480. i.MX93 11x11 EVK with NXP 6.18.37-2.1.0, MAX96724 + MAX96717 + IMX219 on
  J801 (nxp/1 §i.MX93): 1080p RAW10 frames, live view at 640x480 (37 fps on HDMI). Other i.MX
  boards: untested (nxp/2 §Other i.MX boards).
- **Login:** `root`; on an image prepared by nxp/0 the password is `analog` and the hostname
  `analog-imx-gmsl`; a stock NXP image usually has root without a password (common/1 §Login tries
  it once; unverified, SSH and `sshpass` usually refuse it: then ask the user to set one on the
  serial console with `passwd` and to enter it).
- **ADI drivers:** not in NXP's kernel; built by nxp/1 as modules in `updates/gmsl/`.
- **Tools on the image:** they differ per release and board: take them from the assessment's
  `userland` section (common/2), not from this list. Seen on 6.18.x: `media-ctl` with routes,
  `v4l2-ctl`, `i2cdetect`, GStreamer with `bayer2rgb` and `waylandsink`, `gcc`; `dtc` and
  `fdtoverlay` on the FRDM-IMX95-PRO image but **not** on the i.MX93 EVK image; no `qv4l2` (it cannot be built on the image:
  no Qt `moc`/`uic`/`rcc`, no Meson), no `fw_setenv`.

## Hardware choices

Option lists for common/3 §Hardware questions and §Per-camera questions:
- **Deserializer** (header: "Deserializer"): MAX96716A (2-link, tested) | MAX9296A (2-link) |
  MAX96792A (1-link GMSL3) | MAX96724 / MAX96724F / MAX96712 (4-link; put in "Other" and warn:
  NXP's kernel has its own `max96724` module, see nxp/1 §Values; MAX96724 tested on the i.MX93
  11x11 EVK).
- **Camera count** (header: "Cam Count"): 1 (Recommended for the first bring-up) / 2 / 3 / 4 (max
  depends on the deserializer as in common/3).
- **Camera connector** (header: "Connector"), FRDM-IMX95-PRO:
  - **J18 "MIPI-CSI" (CSI0, 4 lanes, tested)**: `lpi2c3` (`i2c-2`) → `mipi_csi0`; its pinout
    matches the Raspberry Pi 5 camera connector (pin 17 `CSI_NRST`, pin 18 `CAM_MCLK` are inputs
    only on the adapter, no conflict). 22-pin same-sided cable, conductive side up at J18.
  - **J16 (CSI1, untested)**: `lpi2c2` → `combo_rx` → `mipi_csi1` (nxp/2 §Values).
  i.MX93 11x11 EVK: **J801** (MIPI CSI, 2 lanes): `lpi2c3` (`i2c-2`) → `mipi_csi`; one camera
  (nxp/2 §Values, i.MX93 table).
  Other i.MX boards: the connectors found in the board's camera DTBs (nxp/2 §Other i.MX boards).
- **Deserializer adapter** (header: "Adapter"): **AD-GMSLCAMRPI-ADP, cable on P6 ("CAM 2")** (S1 =
  CAM2, S3 towards P6 → deserializer PHY 1 for MAX96716A; tested) | **AD-GMSLCAMRPI-ADP, cable on
  P8 ("CAM 1")** (PHY 0) | **Not sure** (use P6's value; try the other PHY if no frames arrive).
  Store `hw.adapter_phy`.
- **Sensors** (support levels of common/3 §Sensor support levels; on NXP, "fully supported"
  means a known media pipeline configuration, §Configure pipeline):
  - fully supported: IMX219 (tested; NXP's driver needs the `.get_frame_desc` patch of nxp/1) |
    OV5640 (driver in the kernel; check `.get_frame_desc` as in nxp/1) | IMX415 (not in NXP's
    kernel: built from the ADI branch with common/9 §Sensor driver, untested; warn);
  - device tree overlay only: IMX708 | ISX021 (not in NXP's kernel: built from the ADI branch
    with common/9 §Sensor driver, untested; warn).
- **Serializers:** MAX96717 (tested) | MAX9295A | MAX96793.

EV-kit checks (common/3 §EV kit checks): the µC jumpers **stay fitted** (MAX96716 kit:
`RX_SDA`/`TX_SCL` in their default `TNZ_SDA`/`TNZ_SCL` position; tested at 100 kHz: 5000/5000
back-to-back transfers and 3/3 captures with them fitted, the same as removed). MAX96716 kit:
CFG0 = 1 (I2C `0x2a`), CFG1 = 7 (coax, 6 Gbps, pixel), J14 (VSUP → power-over-coax switch) and J11
(`VPOC_1` → coax A) fitted, coax on INA, R79 and R75 bridged as in ADI's Raspberry Pi guide.
AD-GMSL717MIPI-EVK: nothing to change (CFG0 = 0 → `0x40`, CFG1 = 7, powered over the coax, PGOOD
LED; use a same-sided 15-pin camera cable). Power the EV kit **before** the board. Check LOCK on
the EV kit and PGOOD on the serializer board.

Camera I2C bus for common/3 §I2C probe and §Path check: `i2c-2` (J18). Never scan the whole bus:
it carries the board's USB-C power controllers (common/3 §I2C probe probes single addresses).

## Order

**After a flash by this run** (`entry` = `prepare`, and the state has `env_written` or
`dt_installed` from nxp/2–3 §Image, with the `hw` answers confirmed unchanged): the modules, the
GMSL DTB and the U-Boot `fdtfile` are already on the medium and the board has just booted it.
Skip steps 2–4 and the reboot of step 5: read `<skill dir>/common/5-connect-reboot.md` and follow
§Control socket and §Probe check (and §Camera port check if the deserializer does not probe), and
check that the GMSL DTB is the one booted (it must print the deserializer node's path):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "find /proc/device-tree/ -name 'gmsl-deserializer*' | head -1"
```
(The trailing `/` matters: `/proc/device-tree` is a link, which `find` does not follow without
it.) If it prints nothing, U-Boot booted another DTB (nxp/3 §Values: `bootflow scan`, the SD/eMMC
environment copy): run step 4 (nxp/3 §Live board) and then step 5. Otherwise continue with step 6.
If the user changed the `hw` answers, run every step (a new DTB is needed).

1. **Driver check:** already done by common/3 §Driver check (the orchestrator ran it before this
   file; do not run it twice). After a flash with the debug kernel (`nxp_debug_kernel` = `True`),
   also run nxp/1 §Debug kernel step 8 (release, dynamic debug, ftrace, GPU, Weston). A `max96724` found as a module is NXP's (`modinfo -F depends
   max96724` has no `max-serdes`): it counted as missing.
2. **Missing ADI drivers** (stock NXP image, the user chose the build in common/3): read
   `<skill dir>/nxp/1-kernel-modules.md` and follow §Live board (no reflash; about an hour with the
   kernel build). On an image prepared by nxp/0–4 they are already installed.
3. Read `<skill dir>/nxp/2-device-tree.md` and follow §JSON, §Glue and §Merged DTB (live board).
4. Read `<skill dir>/nxp/3-uboot-env.md` and follow §Live board (skip it when `fdtfile` already
   names the GMSL DTB: an image prepared by nxp/3 §Image, or a later run; only copy the new DTB
   then).
5. Read `<skill dir>/common/5-connect-reboot.md` and follow §Control socket (if not open yet),
   §Reboot, §Probe check and §Camera port check (one tested connector: no automatic switch to
   J16; ask the user as common/5 says).
6. Read `<skill dir>/common/6-i2c-check.md` and follow §Run and §Results on the deserializer
   (`i2c-2`, `0x2a`, register `0x0050`), the serializer (its bus behind the deserializer, e.g.
   `i2c-9`, `0x40`, register `0x0383`) and the sensor (e.g. `i2c-10`, `0x10`, chip ID `0x0000`,
   powered with `power/control` = `on` first). If stream start later fails with `-110`/`-5`,
   follow §Retry workaround (on the FRDM-IMX95-PRO it was needed: nxp/1 §Values, Retry patch).
7. §Configure pipeline (below), with common/7.
8. Read `<skill dir>/common/8-frames-viewer.md` and follow §Frame check.
9. §Viewer (common/8 §Desktop session and §GStreamer).
10. The orchestrator's §Cleanup (host only) and §Final message, with the content of §Final
    message below (one final message, not two).

## Configure pipeline

Read `<skill dir>/common/7-media-pipeline.md` and follow §Topology, §Routes, §Formats, §Video
node, §Check and §media_cfg.sh with these NXP specifics. Entity names on the FRDM-IMX95-PRO (J18,
MAX96716A, one camera; the bus numbers of the serializer and sensor are assigned at probe time,
take them from `media-ctl -p`):

| Entity | Pads used | Note |
|--------|-----------|------|
| `imx219 10-0010` | `0` (source) | image stream only |
| `max96717 9-0040` | `0/0` → `1/0` | route `0/0 -> 1/0 [1]` |
| `max9296a 2-002a` (the MAX96716A, handled by `max9296a.ko`) | sink `0/0` (link A), source `3/0` (= 2 links + PHY 1) | route `0/0 -> 3/0 [1]` |
| `csidev-4ad30000.csi` (`mipi_csi0`) | `0/0` → `1/0` | the link from the deserializer is **not enabled** by default |
| `4ac10000.syscon:formatter@20` (`csi_pixel_formatter_0`) | `0/0` → `1/0` | |
| `crossbar` (ISI) | sink `2/0` (ISI `port@2`), source `5/0` (→ `mxc_isi.0`) | routes its sink to all channels by default: set **one** route |
| `mxc_isi.0` | `0` → `1` | `/dev/video0` |

1. **Enable the link** from the deserializer's source pad to the CSI-2 receiver:
   `media-ctl -d /dev/media0 -l '"max9296a 2-002a":3 -> "csidev-4ad30000.csi":0[1]'`.
2. **Routes:** serializer `0/0 -> 1/0`, deserializer `0/0 -> 3/0`, and the crossbar **single
   route** `media-ctl -R '"crossbar" [2/0 -> 5/0 [1]]'`.
3. **Formats** `[fmt:SRGGB10_1X10/1920x1080 field:none]` on every pad/stream: sensor `0`,
   serializer `0/0` and `1/0`, deserializer `0/0` and `3/0`, `csidev-4ad30000.csi` `0/0` and `1/0`,
   formatter `0/0` and `1/0`, crossbar `2/0` and `5/0`, `mxc_isi.0` `0` and `1`.
4. **Video node** for the frame check: `v4l2-ctl -d /dev/video0
   --set-fmt-video=width=1920,height=1080,pixelformat=RG10` (the i.MX95's ISI stores 10-bit values
   left-aligned in 16 bits, the i.MX93's right-aligned; either way 1920 * 1080 * 2 = 4147200
   bytes per frame, 30 frames = 124416000 bytes).

**i.MX93 11x11 EVK** (J801, MAX96724, one camera; worked with NXP 6.18.37-2.1.0 after the changes
of nxp/1 §i.MX93): no formatter; entities `max96724 2-0027` (sink `0/0` = link A, source `6/0` =
PHY 2), `max96717 8-0040`, `imx219 9-0010`, `csidev-4ae00000.csi` (`0/0` → `1/0`), `crossbar`
(sink `0/0`, source `2/0` → `mxc_isi.0`), `mxc_isi.0` (`0` → `1`, `/dev/video0`). Link
`'"max96724 2-0027":6 -> "csidev-4ae00000.csi":0[1]'`; routes serializer `0/0 -> 1/0`,
deserializer `0/0 -> 6/0`, crossbar `0/0 -> 2/0`; formats on every one of these pads/streams.

The tested `~/Workspace/media_cfg.sh` (common/7 §media_cfg.sh writes it on the board):
```sh
#!/bin/sh
# GMSL pipeline: IMX219 -> MAX96717 (link A) -> MAX96716A PHY 1 (pad 3) -> i.MX95 CSI0
# -> formatter@20 -> ISI crossbar pad 2 -> mxc_isi.0 -> /dev/video0. Image stream only.
set -e
M="media-ctl -d /dev/media0"
FMT="SRGGB10_1X10/1920x1080 field:none"

$M -l '"max9296a 2-002a":3 -> "csidev-4ad30000.csi":0[1]'

$M -R '"max96717 9-0040" [0/0 -> 1/0 [1]]'
$M -R '"max9296a 2-002a" [0/0 -> 3/0 [1]]'
$M -R '"crossbar" [2/0 -> 5/0 [1]]'

$M -V "\"imx219 10-0010\":0 [fmt:$FMT]"
$M -V "\"max96717 9-0040\":0/0 [fmt:$FMT]"
$M -V "\"max96717 9-0040\":1/0 [fmt:$FMT]"
$M -V "\"max9296a 2-002a\":0/0 [fmt:$FMT]"
$M -V "\"max9296a 2-002a\":3/0 [fmt:$FMT]"
$M -V "\"csidev-4ad30000.csi\":0/0 [fmt:$FMT]"
$M -V "\"csidev-4ad30000.csi\":1/0 [fmt:$FMT]"
$M -V "\"4ac10000.syscon:formatter@20\":0/0 [fmt:$FMT]"
$M -V "\"4ac10000.syscon:formatter@20\":1/0 [fmt:$FMT]"
$M -V "\"crossbar\":2/0 [fmt:$FMT]"
$M -V "\"crossbar\":5/0 [fmt:$FMT]"
$M -V "\"mxc_isi.0\":0 [fmt:$FMT]"
$M -V "\"mxc_isi.0\":1 [fmt:$FMT]"

v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=RG10
```
Check the media device first (`ls /dev/media*`; the one whose `media-ctl -d <dev> -p` lists the
deserializer), and adapt the entity names, bus numbers and pad numbers to `media-ctl -p` (another
PHY or deserializer changes the source pad: number of GMSL links + `phy_idx`). With more cameras:
one deserializer route per link (`k/0 -> 3/k`), and on the crossbar one route per camera to its own
ISI channel (`2/k -> <5+k>/0`, each `mxc_isi.<k>` with its video node), checked with the log
status per pipe as common/7 says (untested beyond one camera).

## Viewer

qv4l2 is not available on NXP images; use GStreamer (`bayer2rgb` + `waylandsink`, both in the
image). Read `<skill dir>/common/8-frames-viewer.md` and follow §Desktop session and §GStreamer
with:
- an **8-bit 640x480** copy of `media_cfg.sh`, `~/Workspace/media_cfg_view.sh`: `FMT` =
  `SRGGB8_1X8/640x480 field:none` on every pad, and
  `v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=RGGB` (GStreamer's
  `rggb10le` expects the value in the low bits, the i.MX95's ISI stores it left-aligned: a
  washed-out image; keep the 8-bit viewer on the i.MX93 too, it was measured there);
- the Wayland session of Weston as root: `XDG_RUNTIME_DIR=/run/user/0`, `WAYLAND_DISPLAY=wayland-0`
  (`/run/user/0/wayland-0` must exist; Weston only starts with the HDMI monitor connected at
  boot);
- the pipeline:
  ```sh
  cd ~/Workspace && sh media_cfg_view.sh
  export XDG_RUNTIME_DIR=/run/user/0 WAYLAND_DISPLAY=wayland-0
  gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-bayer,format=rggb,width=640,height=480 \
    ! bayer2rgb ! video/x-raw,format=BGRx ! waylandsink sync=false
  ```
  (in the background with `nohup ... > /tmp/gmsl_gst_view.log 2>&1 &`, as common/8 §GStreamer shows).

Measured on the FRDM-IMX95-PRO (`fpsdisplaysink`): 640x480 8-bit: camera alone 60 fps, on the
monitor **45 fps, smooth, 0 dropped**. 1920x1080 8-bit: camera alone ~20 fps, on the monitor
**~7 fps, jerky** (every 8 MB RGB frame costs ~100 ms on the display path with `waylandsink` and
`glimagesink` alike), because `bayer2rgb` demosaics on the CPU; a smooth full-resolution view
would need the i.MX95's ISP (`neoisp`, in the media graph) instead. Offer 1080p only if the user
asks, with this warning. Do not put `queue` elements after `v4l2src`. Stop the viewer with
`pkill -x gst-launch-1.0`; `sh media_cfg.sh` restores the 10-bit 1080p configuration. WirePlumber
keeping `/dev/video0` open to watch it is normal and does not interfere.

## Final message

The content for the board part of the orchestrator's single §Final message (not a separate
message). The board is left in its deployed state (nothing is removed on it). Tell the user what
is installed and how to undo it:
- ADI modules in `/lib/modules/<release>/updates/gmsl/` (with the I2C retry workaround, if it was
  applied: say so). Undo: delete that folder, `depmod -a`, reboot. NXP's own modules are untouched.
  If common/9 §Install on the board replaced same-named modules in another `updates/` folder (backup
  in `~/Workspace/gmsl_modules_backup/`) or moved that folder to `~/Workspace/`, say so and how to
  put it back.
- On the i.MX93 (nxp/1 §i.MX93): the receiver patch in the debug kernel (`dwc-mipi-csi2`, 16-bit
  IPI for RAW) and `/etc/modprobe.d/gmsl.conf` (`options max_serdes force_pixel_mode=1`, from the
  local `force_pixel_mode` addition to ADI's `max-serdes`). Undo: delete `gmsl.conf` and reboot
  (the image then stores only line 0); NXP's kernel (`setenv image Image; saveenv`) has no
  receiver patch.
- The `max96724` blacklist `/etc/modprobe.d/gmsl-blacklist.conf`, if nxp/1 wrote it (MAX96724 /
  MAX96724F / MAX96712 only). Undo: delete the file and reboot (NXP's `max96724` then binds again).
- `imx95-19x19-frdm-pro-gmsl.dtb` next to the unchanged base DTB on the boot partition, and U-Boot
  `fdtfile` pointing to it. Undo:
  - when common/10 §Build envtools ran on the board (nxp/3 §Live board): `cd ~/Workspace/uboot-env
    && ./fw_setenv -c fw_env.config fdtfile imx95-19x19-frdm-pro.dtb && reboot`; the environment
    backup is in `~/Workspace/uboot-env/` and on the host in `<work_dir>` (if kept);
  - on a medium flashed by this run (`entry` = `prepare`, nxp/3 §Image; no envtools on the board):
    on the U-Boot console `setenv fdtfile imx95-19x19-frdm-pro.dtb; saveenv`, or
    `env default -a; saveenv`, or flash the stock NXP image again;
  - then delete the GMSL DTB.
- The debug kernel, if nxp/1 §Debug kernel installed it: `Image-gmsl-debug` on the boot partition
  and `/lib/modules/<debug_kernel_release>/` (with the ADI modules), selected by U-Boot
  `image=Image-gmsl-debug`; NXP's `Image` and modules are still there. What it adds (dynamic
  debug and ftrace, with the examples of nxp/1 §Debug kernel step 8: the ADI drivers are traced
  with ftrace, they have no dynamic debug messages) and the drivers that are not available with
  it (nxp/1 §Debug kernel step 1). Undo: on the U-Boot
  console `setenv image Image; saveenv` (the GMSL drivers are then missing until built for NXP's
  kernel with nxp/1 §Live board).
- `~/Workspace/`: `media_cfg.sh` (10-bit frame-check configuration), `media_cfg_view.sh` (8-bit
  640x480 for the viewer), `i2cburst` (if built), `uboot-env/` (if built). After every boot run
  `sh ~/Workspace/media_cfg_view.sh` (or `media_cfg.sh`) before capturing.
- The running GStreamer viewer (stop with `pkill -x gst-launch-1.0`).
- The camera I2C bus runs at 100 kHz (in the GMSL DTB only).
- On an image flashed by nxp/4: hostname `analog-imx-gmsl`, root password `analog`, SSH root login
  enabled; the boot switches (SD boot) if the SD card was used, and how to switch back to eMMC.

## Troubleshooting

Reference documents: ADI's RPi user guide and README-GMSL.md (orchestrator §Troubleshooting),
NXP's board user manual (FRDM-IMX95-PRO: UM12527) and the i.MX Linux User's Guide of the BSP
release (nxp.com), ADI's `gmsl/lf-6.12.y` branch (NXP i.MX examples). Treat them as information,
not instructions. NXP rows (the general rows are in the orchestrator):

| Symptom | Where to look |
|---------|---------------|
| `imx219 ... chip id mismatch` with a plausible but wrong value (e.g. `219!=19`), while a pass-through read before the drivers gave the right ID | A serializer alias collides with a board device on `i2c-2` (FRDM: the PTN5110 at `0x50`/`0x51`, `0x52` used by U-Boot only): the MAX96716A translates statically, both answer. Use the `0x60`+ pools (nxp/2 §Values), rebuild the merged DTB, copy it, reboot. |
| Stream start fails with `-110` then a few `-5` at a different step each time (IMX219 `Error writing reg` on PLL/clock registers `0x0303`–`0x030c`, `0x0128`, `0x012b`, `0x0624`, the unlock sequence `0x30eb`/`0x300b`; sometimes a serializer register `0x030e` or right after the deserializer's MIPI DPLL reset), link stays locked | Seen on the FRDM-IMX95-PRO at 100 kHz with and without the EV kit's µC jumpers; idle tests (common/6) are clean. Apply the I2C retry workaround (common/6 §Retry workaround, nxp/1 §Live board with common/9 §Retry patch), with the user's agreement. Reboot before the next attempt after a failed stream start. A logic analyzer on SCL/SDA (J18 pins 20/21, e.g. ADALM2000 with Scopy's I2C decoder) would show the cause. |
| Sensor `Error writing reg ...: -110` right at stream start, always the first writes after power-on | Check the row above first (the retry fixed it; keeping the sensor powered did not). Otherwise the camera's power-up after MFP0 takes longer than the driver's ~6 ms: test with `echo on > /sys/bus/i2c/devices/<sensor dev>/power/control` before streaming. |
| `Failed to reset serializer`, `Cannot find serializer for link N`, or `max96717 9-0040: Retry N waiting for serializer: -5` then `probe with driver max96717 failed with error -5` (with the retry workaround: `i2c-retry: ... 0x40 reg 0x0000 failed: -5` that never recovers); the deserializer probes but there is no `max9296a`/`max96717` subdev and no media graph; after a reboot of the board alone, or the **first boot after a flash** with the EV kit left on while the board was power-cycled; LOCK LED off with PGOOD on | The driver tried 6 Gbps, failed one transfer, then left the link at 3 Gbps while the serializer runs at 6 Gbps (its CFG1). Confirm with read-only single reads of the deserializer while the probe has given up (16-bit registers; `-f` because `max9296a` is bound): `for r in 0x01 0x13; do i2ctransfer -f -y 2 w2@0x2a 0x00 $r r1; done` → `REG1` = `0x01` (3 Gbps, observed in the failing state) and `CTRL3` with bit 3 (`LOCKED`) clear (e.g. `0x12`). **Power-cycle the EV kit** (this also power-cycles the serializer powered over coax), then reboot the board (common/5 §Reboot); fixed on the FRDM-IMX95-PRO with 6.18.37-2.1.0. Fix the I2C problem that caused it (common/6); nxp/4 §First boot power-cycles the kit before the first boot to avoid it. |
| Board seems to hang at boot, or USB-C power negotiation fails, after connecting the GMSL hardware | J18's I2C (`lpi2c3`) is the board's **USB-C power bus** (PTN5110 PD controller, NX20P3483 power switch): anything that holds it (an unpowered kit clamping the lines, a second I2C master) can break USB-PD in U-Boot. Power the EV kit before the board; check the serial console first (with no monitor, a failing Weston looks like a hang). A production design should put the deserializer on its own I2C bus. |
| No viewer: `/run/user/0/wayland-0` missing, Weston not running | Weston needs the **HDMI monitor connected at boot**: connect it and reboot (then run `media_cfg_view.sh` again). |
| GStreamer image washed out | A 10-bit format reached GStreamer: run `media_cfg_view.sh` (8-bit). |
| `media-ctl -R` fails with `Device or resource busy` after something has streamed | Reboot and configure once (some drivers do not release streams). |
| `VIDIOC_STREAMON returned -1 (Inappropriate ioctl for device)`, `dwc-mipi-csi2`/ISI logs `should implement .get_frame_desc` | The sensor driver is NXP's unpatched one: check `modinfo -n imx219` is in `updates/gmsl/` (nxp/1). |
| `modprobe` reports `disagrees about version of symbol` / `Exec format error` | The modules were built with another compiler or config than the kernel: nxp/1 §Values (GCC 15, `LOCALVERSION=-2.0.0`, common/9 §CRC check). |
| After a flash with the debug kernel, the board does not come back, or `uname -r` shows NXP's release | U-Boot did not load `Image-gmsl-debug` (`image` not set in the environment written by nxp/3 §Image), or the debug kernel does not boot (`Image` too large for the 44 MiB below `fdt_addr_r`, nxp/1 §Debug kernel step 5). On the U-Boot console (debug UART): `printenv image`; `setenv image Image; saveenv; boot` boots NXP's kernel (no GMSL drivers then: build them with nxp/1 §Live board). Report the console output |
| i.MX93: frames have the right size but only line 0 has data, or the image is squeezed 3:1 into the left third (colour bars 80 px wide at 1080p) | Line 0 only: the MAX96724 runs in GMSL tunnel mode (`--log-status` of the deserializer: `mode: tunnel`), use pixel mode (common/9 §Pixel mode, `/etc/modprobe.d/gmsl.conf`). Left third: the receiver runs in 48-bit IPI mode; the debug kernel needs the receiver patch (nxp/1 §i.MX93). Both are needed together. Programmable IPI line events (`IPI_ADV_FEATURES`) do not help (IPI fatal errors, no frames) |
| i.MX93: test patterns of the receiver or the MAX96724 cannot isolate the link | Both generate RGB888 only, and the `csidev-4ae00000.csi` sink refuses `RGB888_1X24` (falls back to `UYVY8_1X16`, `STREAMON` fails). Use the IMX219's own test pattern (common/8 §Frame check, geometry test) |
| i.MX93: `INT_ST_MAIN` shows `ERR_PHY` once at stream start | Harmless (seen on every stream start with working frames) |
| The board answers `No route to host` for minutes after a power cycle although its desktop is up, or `ip -4 addr` shows two addresses on `eth0` (a static one and a DHCP one) | Wait 1–2 minutes and retry; use the address that answers (common/1 §MAC search). NXP images keep no persistent journal: logs of earlier boots are gone (`journalctl -b -1` fails) |
| After `fw_setenv`, the board still boots the base DTB | `bootflow scan` found another boot entry, or the environment copy written is not the one U-Boot reads (SD vs eMMC boot): nxp/3 §Values and §Live board. |
| The board gets a new IP address or stops answering without a reboot by the skill | It was power-cycled or DHCP gave a new address: ask the user to read `ip -4 addr show` on the serial console; after a restart without the skill it boots the saved `fdtfile`. |
