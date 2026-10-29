# NXP 1: ADI GMSL modules for the NXP BSP kernel

NXP's i.MX kernels (`linux-imx`) have no ADI GMSL drivers: they ship NXP's own `max96724.ko` and
`max96717_lib.ko`, which do not depend on `max-serdes` and do not work with ADI's device tree.
This file builds ADI's `maxim-serdes` drivers (and a sensor driver with `.get_frame_desc`) as
out-of-tree modules for the exact kernel the board runs or the image contains, with
common/9. The NXP kernel itself is not replaced; the modules go to `updates/gmsl/`.

Two entry points:
- **§Live board**: the board runs an NXP BSP image without the ADI drivers (nxp/5 §Order, or the
  orchestrator's "Drivers missing on a board on the network": no reflash).
- **§Image**: the build path (orchestrator §Flow, "No", NXP), with the image mounted by nxp/0.

Placeholders as in the orchestrator; `<tpl_dir>` = `/tmp/gen_gmsl_dts/nxp`.

## Values

Tested on the FRDM-IMX95-PRO with NXP i.MX Linux 6.18.20-2.0.0. For another release or board,
find each value the same way (common/9 §Inputs) and tell the user what differs from this table.

| Item | FRDM-IMX95-PRO, 6.18.20-2.0.0 | How to find it for another release |
|------|-------------------------------|-----------------------------------|
| Kernel release | `6.18.20-2.0.0-gb096ce610e95` | `uname -r` / `lib/modules` of the image |
| Kernel tree | https://github.com/nxp-imx/linux-imx | the same for every NXP BSP |
| Tag | `lf-6.18.20-2.0.0` (commit `b096ce610e95` = the `-g` hash) | `git ls-remote https://github.com/nxp-imx/linux-imx 'refs/tags/lf-*' \| grep <hash>` |
| `LOCALVERSION` | `-2.0.0` (Yocto passes it on the command line; it is not in the config) | the part between the kernel version and `-g<hash>`; `make ... kernelrelease` must match exactly |
| `ARCH` | `arm64` | |
| Board compiler (`CONFIG_CC_VERSION_TEXT`) | GCC 15.2.0 | the config |
| Host toolchain | the **latest Arm GNU Toolchain release of the board's GCC major version** (`aarch64-none-linux-gnu`, x86_64 host): find the newest `<ver>.rel<n>` whose GCC major equals the board's with the probe of common/9 §Compiler (Arm's downloads page no longer lists the releases); URL pattern `https://developer.arm.com/-/media/Files/downloads/gnu/<ver>.rel<n>/binrel/arm-gnu-toolchain-<ver>.rel<n>-x86_64-aarch64-none-linux-gnu.tar.xz` and its `.sha256asc`; `<prefix>` = `<work_dir>/arm-gnu-toolchain-<ver>.rel<n>-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-`. Tested: 15.2.rel1 for the board's GCC 15.2.0. A newer **major** version would change the symbol CRCs: never use it | same GCC major version as the board |
| `CONFIG_MODVERSIONS` | `y` (so the kernel build and the CRC check are needed) | the config |
| CRC check | 64/64 CRCs match with GCC 15 (checked against the board's `imx219.ko`); Ubuntu's GCC 13 gave 24 of 64 mismatches (`CC_HAS_COUNTED_BY`, `CC_HAS_MULTIDIMENSIONAL_NONSTRING`) | common/9 §CRC check |
| ADI branch | the newest ADI GMSL branch (`adi_gmsl_branch`, common/4 §Templates); tested: `gmsl/rpi-6.13.y` at commit `72bb110`; folder `drivers/media/i2c/maxim-serdes` | common/4 §Templates (newest `gmsl/rpi-*`) |
| Port (common/9 §Port) | 3 changes: `PIN_CONFIG_OUTPUT` → `PIN_CONFIG_LEVEL` (all uses in `max96717.c`); `.set_rv` → `.set` in `max96717.c`'s `gpio_chip`; `V4L2_CID_USER_MAXIM_SERDES_BASE` defined in `max_serdes.h` as `(V4L2_CID_USER_BASE + 0x1250)` inside `#ifndef` (ADI's `0x11e0` is `V4L2_CID_USER_UVC_BASE` in 6.18) | compile errors, one commit each |
| `Kbuild` | `max-serdes-objs := max_serdes.o max_ser.o max_des.o` and `obj-m += max-serdes.o max96717.o max9296a.o` | |
| Left out | **`max96724`**: NXP's kernel ships its own `max96724.ko` (same module name) | `find <modules dir> -name 'max*'` |
| Deserializers | `max9296a.ko` handles MAX9296A, MAX96716A (tested) and MAX96792A. MAX96724/MAX96724F/MAX96712 need ADI's `max96724.ko`: see below | |
| Sensor | NXP's `imx219.c` (6.18) has no `.get_frame_desc`: patched copy (common/9 §Sensor driver), one image stream, VC 0, RAW8/RAW10 | `grep -n get_frame_desc drivers/media/i2c/<sensor>.c` |
| Other sensors | `ov5640` is in the kernel (check it the same way); `imx415`, `imx708`, `isx021` are not built in this BSP: build them from the ADI branch with common/9 §Sensor driver (untested) | `CONFIG_VIDEO_<SENSOR>` in the config |
| `i2c-atr` | `CONFIG_I2C_ATR=m`: NXP's `i2c-atr.ko` is used; `depmod` resolves it | |
| Install folder | `/lib/modules/<release>/updates/gmsl/` (`<subfolder>` = `gmsl`) | |

**MAX96724 / MAX96724F / MAX96712 on NXP** (untested): ADI's `max96724.ko` cannot be installed
next to NXP's module of the same name, and both bind `maxim,max96724`. Tell the user. Best effort,
only if they agree: build ADI's driver under another module name (`Kbuild`:
`adi-max96724-objs := max96724.o`, `obj-m += adi-max96724.o`), and keep NXP's module from binding
first by blacklisting it in `/etc/modprobe.d/gmsl-blacklist.conf` (`blacklist max96724`; on the
image path in `/tmp/gmsl_nxp_rootfs/etc/modprobe.d/`; undo: delete that file and reboot). The
driver check of common/3 then looks for `adi-max96724`. List the file and its undo in the final
message (nxp/5 §Final message).

**Retry patch:** on the FRDM-IMX95-PRO, every stream start failed without the I2C retry
workaround (sporadic `-110`/`-5` during stream start; common/6 §Results, last row). On the live
path, add it only after common/6 shows that row and the user agrees. On the build path (§Image)
the board cannot be tested before flashing: ask **in advance**, together with the build plan
(header: "I2C retry"): **Include the retry workaround (Recommended on FRDM-IMX95-PRO)** |
**Build without it** (it can be added later from the live board with §Live board). Tell the user it
hides the problem rather than fixing it. If agreed, apply common/9 §Retry patch before installing.

## Live board

The control socket of common/5 §Control socket is open, and common/2 saved the board's config as
`<tpl_dir>/board.config`.

Read `<skill dir>/common/9-vendor-modules.md` and follow it with the live inputs of §Inputs and
the values of §Values above: §Inputs, §Kernel source, §Compiler, §Kernel build, §CRC check (on the
board, against a vendor module such as `videodev` or `imx219`), §ADI drivers, §Port, §Sensor
driver, §Retry patch (only if agreed), then **§Install on the board** with `<subfolder>` =
`gmsl`.

Then run common/3 §Driver check again. On NXP, a `max96724` found as a module is NXP's (check
`modinfo -F depends max96724`: ADI modules depend on `max-serdes`). Continue with nxp/2.

Record in `<state>`: `kernel_release`, `localversion` (`-2.0.0`), `toolchain_prefix`,
`kernel_src_path` (`<work_dir>/linux-lf-6.18.20-2.0.0`), `modules_built`, `drivers_ok`.

## Image

The image is mounted by nxp/0 §Mount (`/tmp/gmsl_nxp_rootfs`, `/tmp/gmsl_nxp_boot`), and nxp/0
§Read the image saved the kernel config as `<tpl_dir>/board.config` and found the release and
tag.

Read `<skill dir>/common/9-vendor-modules.md` and follow it with the image inputs of §Inputs and
the values of §Values above: §Inputs, §Kernel source, §Compiler, §Kernel build, §CRC check (on the
host, against a module copied from `/tmp/gmsl_nxp_rootfs/lib/modules/<release>/kernel/`), §ADI
drivers, §Port, §Sensor driver, §Retry patch (only if the user agreed in advance, see §Values),
then **§Install into an image** (`updates/gmsl/`, `depmod -b /tmp/gmsl_nxp_rootfs <release>`).

The modules are test-loaded on the first boot: nxp/5 §Order starts with the driver check, which
then finds them. Record the same `<state>` keys as for §Live board. Continue with nxp/2.
