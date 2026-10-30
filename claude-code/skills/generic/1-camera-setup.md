# Generic 1: Camera setup on a board without a platform folder

The hardware option lists, the device tree install methods and the tools for a generic board, and
the order in which the common steps run. Run after generic/0 (ratings, best-effort plan agreed).
Placeholders as in the orchestrator; `<tpl_dir>` = `/tmp/gen_gmsl_dts/generic`. Parameters come
from the board's own device tree and the user's answers, never from guesses (orchestrator
§Rules).

## Order

1. Read `<skill dir>/common/5-connect-reboot.md` and follow §Control socket, then §Tools (below):
   common/3 §I2C probe and §Path check need `i2cdetect`, and common/7 needs `media-ctl` with
   routes, so they are checked (and installed, after asking) first.
2. Read `<skill dir>/common/3-camera-questions.md` and follow it (all sections) with the option
   lists of §Hardware choices below. This is the only run of common/3 on a generic board (the
   orchestrator does not run it for `generic`).
3. Read `<skill dir>/common/4-device-tree.md` and follow the **glue variant**: §JSON, §Bus speed,
   §Alias pools, §Templates, §Generate, §Glue, §Compile and §Validate (and §Merged DTB when the
   install method of §Install device tree needs it).
4. Missing ADI drivers (common/3 §Driver check, generic/0 §Best-effort plan): read
   `<skill dir>/common/9-vendor-modules.md` and follow it with the live inputs, up to §Install on
   the board (`<subfolder>` = `gmsl`). Then common/3 §Driver check again.
5. §Install device tree (below), with common/5 §Control socket open.
6. common/5 §Reboot (skipped for a runtime overlay), §Probe check and §Camera port check.
7. Read `<skill dir>/common/6-i2c-check.md` and follow §Run and §Results (and §Retry workaround if
   the results call for it and the user agrees).
8. Read `<skill dir>/common/7-media-pipeline.md` and follow it (receivers differ most here: follow
   their own pad layout in `media-ctl -p`).
9. Read `<skill dir>/common/8-frames-viewer.md` and follow §Frame check, then §Desktop session and
   §GStreamer (the viewer of generic boards).
10. Read `<skill dir>/generic/2-artifacts.md` and follow §Artifacts package (it records
    `artifacts_written`); then the orchestrator's §Cleanup (host only) and §Final message, with
    the content of generic/2 §Final message (one final message, not two).

If a step fails, try the fixes of generic/2 §Troubleshooting and the orchestrator's
§Troubleshooting before giving up on it; only when a task is truly impossible or the user declines,
go to generic/2 with what was reached.

## Hardware choices

The option lists for common/3 §Hardware questions, in one AskUserQuestion call:
- **Deserializer** (header: "Deserializer"): MAX96724 (4-link) | MAX9296A (2-link) | MAX96716A
  (2-link) | MAX96712 (4-link). Also available: MAX96724F (1-link), MAX96714 (1-link), MAX96792A
  (1-link GMSL3) — put these in "Other".
- **Camera count** (header: "Cam Count"): 1 / 2 / 3 / 4 (max depends on deserializer:
  MAX96724/MAX96712=4, MAX9296A/MAX96716A=2, the others 1). Recommend 1 for the first bring-up on a
  new board.
- **Camera connector** (header: "Connector"): the receivers / camera connectors found in the
  assessment, described by what the vendor DT says about them (node path, the sensor or overlay
  that uses it, number of lanes), plus "Not sure". Ask the user which connector the deserializer
  board is plugged into, as labeled on the board. Device trees do not carry board labels (J18,
  "MIPI-CSI"): map the user's answer to a receiver through the vendor's camera DTBs or overlays
  (their names say which connector they are for, e.g. "combo" for a connector shared with DSI; the
  I2C bus and receiver they enable differ per connector: diff each camera DTB against the base DTB,
  common/4 §Glue), or the board's user manual if the user has it. If the mapping is still
  ambiguous, ask the user rather than guess, and say which receiver each choice means.
- **Deserializer adapter** (header: "Adapter"): how the deserializer's CSI-2 output reaches the
  board. Explain in the question that the deserializer has two CSI-2 output ports (PHY A / PHY B)
  and the device tree must name the one that is wired; users do not know adapter names. Options:
  **AD-GMSLCAMRPI-ADP, cable on P6 ("CAM 2")** (PHY B; ADI's Raspberry Pi setup) |
  **AD-GMSLCAMRPI-ADP, cable on P8 ("CAM 1")** (PHY A) | **Other adapter or direct cable** (ask
  which deserializer CSI-2 port is wired) | **Not sure / not connected yet** (use the Raspberry Pi
  value, and say which PHY to try if no frames arrive). Store `hw.adapter_phy`.

Deserializer PHY (`phys[0].phy_idx`, common/4 §JSON):

| Adapter | max9296a | max96712 | max96716a | max96724 | max96724f | max96792a |
|---------|----------|----------|-----------|----------|-----------|-----------|
| AD-GMSLCAMRPI-ADP, P6 ("CAM 2", PHY B) | 1 | 2 | 1 | 2 | 2 | 1 |
| AD-GMSLCAMRPI-ADP, P8 ("CAM 1", PHY A) | 0 | PHY A, from the template | 0 | PHY A, from the template | PHY A, from the template | 0 |
| Other / not sure | ask; if unknown, the P6 value, and the other PHY if no frames arrive | | | | | |

(The P8 values of the 2-PHY chips are the other PHY, as on the NXP FRDM setup, nxp/2 §Values; for
the 4-PHY chips read the index of PHY A from the chip's template, `<tpl_dir>/<chip>.dtsi.in`,
instead of guessing. MAX96714: one PHY, `phy_idx` from `<tpl_dir>/max96714.dtsi.in`.)

Per-camera questions (common/3 §Per-camera questions): serializers MAX96717 (GMSL2) | MAX9295A
(GMSL2) | MAX96793 (GMSL3); sensors: the sensors with a template whose driver the assessment
found, with their level of common/3 §Sensor support levels (fully supported: IMX219, OV5640,
IMX415; device tree overlay only: IMX708, ISX021); warn about the others (their driver is built
with common/9 §Sensor driver, best effort). On a generic board every pipeline is derived from
the topology (common/7), so the levels mean only that an overlay-only sensor has no tested
configuration anywhere.

Camera I2C bus: the bus of the vendor camera nodes for the chosen connector (common/3 §I2C probe
finds the deserializer on it). Bus speed: 100 kHz by default (common/4 §Bus speed).

## Install device tree

Use the first method that works, in this order. Before each method that needs a reboot, tell the
user how to recover (below) and ask them to confirm (header: "Boot change"): **Change the boot
config (Recommended)** | **Artifacts only**. Record `dt_installed` and the method in `<state>`.

1. **Runtime overlay (no reboot)**, if the assessment found `of_overlay_fdt_apply` and
   `dt-overlays-dir=yes`:
   ```bash
   scp -o ControlPath=/tmp/gmsl_ssh_sock -o BatchMode=yes /tmp/gen_gmsl_dts/gmsl.dtbo <user>@<ip>:/tmp/gmsl.dtbo
   ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'mkdir /sys/kernel/config/device-tree/overlays/gmsl && cat /tmp/gmsl.dtbo > /sys/kernel/config/device-tree/overlays/gmsl/dtbo && cat /sys/kernel/config/device-tree/overlays/gmsl/status'; rm -f /tmp/gmsl.dtbo"
   ```
   The status must be `applied`. Load the drivers (`sudo modprobe max9296a` or `max96724`,
   `max96717`, the sensor), then **skip the reboot of common/5** (run only its §Probe check and
   §Camera port check). The overlay is gone at the next reboot (remove it earlier with
   `sudo rmdir /sys/kernel/config/device-tree/overlays/gmsl`). If the receiver driver was already
   bound before the overlay, unbind and bind it again (`/sys/bus/platform/drivers/<driver>/unbind`
   and `bind` with the device name), or use method 2.
2. **Merged DTB with a fallback boot entry**, if the board boots with extlinux
   (`/boot/extlinux/extlinux.conf`), or with U-Boot and a writable environment:
   - Build and copy the merged DTB with common/4 §Merged DTB (`<gmsl_dtb>` = `<base_dtb>` with
     `.dtb` replaced by `-gmsl.dtb`, next to it; never modify `<base_dtb>`).
   - **extlinux:** add a copy of the default label named `gmsl` with `FDT <gmsl_dtb>` (and without
     `FDTDIR`), and set `DEFAULT gmsl`; keep the original label unchanged. Edit a copy in the
     board's `/tmp` (`/tmp/extlinux.conf.new`), show the diff to the user, then install it with
     sudo and delete the copy. Recovery: choose the original label in the boot menu on the serial
     console, or put the original `DEFAULT` back in `extlinux.conf` from the SD card / storage on
     a PC.
   - **U-Boot environment** (`fdtfile`/`fdt_file` names the DTB): with `fw_setenv` on the board
     (`/etc/fw_env.config` present), note the old value, then `fw_setenv fdtfile <gmsl_dtb name>`.
     **Without `fw_setenv`** (common on vendor images): read `<skill dir>/common/10-uboot-env.md`
     and follow §Version, §Environment location, §CRC search, §Build envtools, §Verify and §Backup
     and write (no valid saved environment: §Console fallback once, or a fresh environment as in
     §Offline env image, written to the board's device after the backup, with the user's
     agreement). Recovery: on the U-Boot serial console, `setenv fdtfile <old value>; saveenv;
     boot`, or `env default -a; saveenv`; the backup of common/10 §Backup and write.
   - Show the user the exact change (old → new) in the confirmation question.
3. **U-Boot console (done by the user)**, if U-Boot loads the DTB as a file from a writable boot
   partition but `fw_setenv` cannot be used (method 2's build is not possible: no matching U-Boot
   source, or no valid saved environment yet), and the user has the serial console: common/4
   §Merged DTB for the file, then common/10 §Console fallback. Each later DTB change (for example
   new alias pools) only needs the file copied again and a reboot.
4. **Other boot methods** (`config.txt` without being a Raspberry Pi, GRUB/systemd-boot
   `devicetree`, FIT images, signed or verified boot, read-only boot files): do not change them
   without a fallback. Best effort (generic/0 §Best-effort plan): if the user can boot an
   alternative entry (a second GRUB entry with `devicetree <gmsl_dtb>`, a second FIT
   configuration they can select on the console), propose it with its recovery and ask; signed or
   verified boot without the user's keys is truly impossible: generic/2 (Artifacts) and say why.

## Tools

Needed on the board: `media-ctl` with routing support (`media-ctl --help` lists `--set-routes`,
v4l-utils 1.28+), `v4l2-ctl`, `i2cdetect` (`i2c-tools`), `fdtoverlay` for merging on the board, and
for the viewer GStreamer with `bayer2rgb` (common/8 §GStreamer installs it). Check:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "media-ctl --help 2>&1 | grep -q -- '--set-routes' && echo routes-ok || echo routes-missing; which v4l2-ctl i2cdetect fdtoverlay; which apt-get dnf opkg 2>/dev/null"
```
Install missing ones with the board's package manager (apt `v4l-utils i2c-tools
device-tree-compiler`, dnf, opkg) after asking the user (header: "Install"). Installing needs
Internet access on the board.

If `routes-missing` (the package's v4l-utils is older than 1.28), v4l-utils must be built from
source on the board (several minutes; Internet access needed on the board; needs `git`, `meson`,
`ninja` and a compiler): use AskUserQuestion (header: "v4l-utils"): **Build v4l-utils 1.28.1
(Recommended)** | **Stop**. To build (run it in the background, since it can take longer than the
command timeout, and check the result before continuing):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get update && SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y git build-essential meson ninja-build pkg-config libudev-dev libjpeg-dev"
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "mkdir -p ~/Workspace && cd ~/Workspace && (test -d v4l-utils || git clone https://git.linuxtv.org/v4l-utils.git) && cd v4l-utils && git checkout v4l-utils-1.28.1 && (test -d build || meson setup build) && ninja -C build && SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A ninja -C build install"
```
(The Qt development packages are not installed: the Qt tools such as qv4l2 are not needed, and
meson skips them without Qt. With dnf/opkg, install the same build dependencies under the
distribution's names.) `ninja install` puts the tools in `/usr/local/bin`; check that
`media-ctl` now resolves there (`which media-ctl`, `media-ctl --help | grep set-routes`).

Without a package manager: cross-build static `media-ctl`/`v4l2-ctl` (v4l-utils with
`meson setup --cross-file ... -Ddefault_library=static`) and `i2cdetect` on the host and copy them
to `~/Workspace/bin` on the board (ask first); if that is not possible, put the needed commands in
the artifacts (generic/2).
