# NXP 1: ADI GMSL modules for the NXP BSP kernel

NXP's i.MX kernels (`linux-imx`) have no ADI GMSL drivers: they ship NXP's own `max96724.ko` and
`max96717_lib.ko`, which do not depend on `max-serdes` and do not work with ADI's device tree.
This file builds ADI's `maxim-serdes` drivers (and a sensor driver with `.get_frame_desc`) as
out-of-tree modules for the exact kernel the board runs or the image contains, with
common/9. The NXP kernel itself is not replaced; the modules go to `updates/gmsl/`.

Two entry points:
- **§Live board**: the board runs an NXP BSP image without the ADI drivers (nxp/5 §Order, or the
  orchestrator's "Drivers missing on a board on the network": no reflash).
- **§Image**: the build path (orchestrator §Flow, "No", NXP), with the image mounted by nxp/0. By
  default the build path also replaces NXP's kernel with a **debug kernel** built from the same
  source (§Debug kernel); the live board path never does.

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

**MAX96724 / MAX96724F / MAX96712 on NXP** (MAX96724 tested on the i.MX93 11x11 EVK with 6.18.37-2.1.0): ADI's `max96724.ko` cannot be installed
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

## i.MX93

The i.MX93's CSI-2 receiver (NXP's `dwc-mipi-csi2`, `snps,dw-mipi-csi2` at `csi@4ae00000`) feeds
the ISI directly, without the i.MX95's pixel formatter. Two changes are needed for RAW sensors,
both measured on the i.MX93 11x11 EVK with NXP 6.18.37-2.1.0 (MAX96724 + MAX96717 + IMX219):
1. **Receiver in 16-bit mode:** NXP's `dwc_csi_device_ipi_config()` clears `IPI_MODE` bit 8
   (`COLOR_MODE16`), so the IPI runs in 48-bit mode and only one of three RAW pixels reaches the
   ISI: the image is squeezed 3:1 into the left third (colour bars 80 px wide instead of 240 at
   1080p), for RAW8 and RAW10 alike. The driver is built into the kernel
   (`CONFIG_VIDEO_DWC_MIPI_CSIS=y`), so a module cannot fix it: the debug kernel carries the patch
   `<skill dir>/nxp/patches/imx93-dwc-mipi-csi2-color-mode16.patch` (§Debug kernel step 3a). It
   sets `COLOR_MODE16` for RAW6–RAW14 data types only when the board is `fsl,imx93`, so the i.MX95
   is unchanged.
2. **GMSL pixel mode:** in tunnel mode the receiver gets no usable line timing and stores only
   the first line of each frame (also with change 1; the receiver's programmable line events,
   `IPI_ADV_FEATURES`, give only IPI fatal errors). Pixel mode gives complete frames: common/9
   §Pixel mode, with `options max_serdes force_pixel_mode=1` in
   `/etc/modprobe.d/gmsl.conf`. Pixel doubling is not involved.

So on the i.MX93 the debug kernel is required, not only the default: on the build path do not
offer "Keep NXP's kernel" (say why); on the live board path see §Live board. List both changes in
the final message (nxp/5 §Final message).

## Live board

The control socket of common/5 §Control socket is open, and common/2 saved the board's config as
`<tpl_dir>/board.config`.

Read `<skill dir>/common/9-vendor-modules.md` and follow it with the live inputs of §Inputs and
the values of §Values above: §Inputs, §Kernel source, §Compiler, §Kernel build, §CRC check (on the
board, against a vendor module such as `videodev` or `imx219`), §ADI drivers, §Port, §Sensor
driver, §Retry patch (only if agreed), then **§Install on the board** with `<subfolder>` =
`gmsl`.

**i.MX93** (§i.MX93): ADI modules alone are not enough, the receiver fix needs a kernel. Tell the
user and ask (header: "Kernel"): **Install the debug kernel next to NXP's (Recommended)** | **Stop**.
With consent: build it as §Debug kernel steps 1–5 (with step 3a), install it on the running board
the same way as step 6 but over the control socket (the modules to
`/lib/modules/<debug_kernel_release>/` with the ADI modules in its `updates/gmsl/`, `depmod -a
<debug_kernel_release>` on the board, `Image-gmsl-debug` on the boot partition that was booted;
NXP's `Image` and modules stay), and set `image=Image-gmsl-debug` with common/10 (§Build envtools,
backup first; recovery `setenv image Image; saveenv`). Write `/etc/modprobe.d/gmsl.conf`
(common/9 §Pixel mode) on the board. Then reboot (common/5).

Then run common/3 §Driver check again. On NXP, a `max96724` found as a module is NXP's (check
`modinfo -F depends max96724`: ADI modules depend on `max-serdes`). Continue with nxp/2.

Record in `<state>`: `kernel_release`, `localversion` (`-2.0.0`), `toolchain_prefix`,
`kernel_src_path` (`<work_dir>/linux-lf-6.18.20-2.0.0`), `modules_built`, `drivers_ok`.

## Image

The image is mounted by nxp/0 §Mount (`/tmp/gmsl_nxp_rootfs`, `/tmp/gmsl_nxp_boot`), and nxp/0
§Read the image saved the kernel config as `<tpl_dir>/board.config` and found the release and
tag.

Ask first, together with the build plan and the retry question (header: "Kernel"): **Debug kernel
(Recommended, default)** (description: "NXP's kernel source with dynamic debug and ftrace
enabled, installed next to NXP's kernel, which stays on the card; drivers NXP builds outside the
kernel source are not available with it (list them)") | **Keep NXP's kernel** (description: "Only
the ADI modules are added to NXP's kernel"). Before asking, list the drivers that would be missing
(§Debug kernel step 1). On the i.MX93 do not ask: the debug kernel is required (§i.MX93); say so. Store `nxp_debug_kernel` (`True`/`False`). Without an answer (the user
skips the question), use the debug kernel.

Read `<skill dir>/common/9-vendor-modules.md` and follow it with the image inputs of §Inputs and
the values of §Values above: §Inputs, §Kernel source, §Compiler, then:
- **`nxp_debug_kernel` = `True`:** §Debug kernel below (it replaces common/9 §Kernel build and
  §CRC check), then common/9 §ADI drivers, §Port, §Pixel mode (i.MX93 only, §i.MX93), §Sensor
  driver, §Retry patch (only if the user agreed in advance, see §Values), all built against the
  debug tree with its `LOCALVERSION`, then §Debug kernel step 6 (install). On the i.MX93 also
  write `/tmp/gmsl_nxp_rootfs/etc/modprobe.d/gmsl.conf` with `options max_serdes force_pixel_mode=1`
  (root rights).
- **`nxp_debug_kernel` = `False`:** §Kernel build, §CRC check (on the host, against a module
  copied from `/tmp/gmsl_nxp_rootfs/lib/modules/<release>/kernel/`), §ADI drivers, §Port, §Sensor
  driver, §Retry patch (only if the user agreed in advance, see §Values), then **§Install into an
  image** (`updates/gmsl/`, `depmod -b /tmp/gmsl_nxp_rootfs <release>`).

The modules are test-loaded on the first boot: nxp/5 §Order starts with the driver check, which
then finds them. Record the same `<state>` keys as for §Live board. Continue with nxp/2.

## Debug kernel

Build path only (§Image, `nxp_debug_kernel` = `True`; the default). The kernel built from NXP's
source with the image's own config, plus dynamic debug and function tracing, gets **its own
release name**, so it is installed next to NXP's kernel instead of over it: NXP's `Image` and
`/lib/modules/<release>/` stay untouched, and U-Boot's `image` variable selects the kernel.
Tested on the FRDM-IMX95-PRO (end of this section); values checked on it with NXP 6.18.37-2.1.0: U-Boot loads
`${image}` (default `Image`) from the boot partition to `loadaddr` = `0x90400000` and the DTB to
`fdt_addr_r` = `0x93000000` (44 MiB apart); NXP's `Image` is 35.7 MB; the GPU driver
`mali_kbase` (Weston, the viewer) is built inside `linux-imx` (`drivers/gpu/arm/midgard`).

1. **What is lost:** list the modules NXP builds outside the kernel source (they exist only for
   NXP's kernel):
   ```bash
   cd /tmp/gmsl_nxp_rootfs/lib/modules/<release> && find . -name '*.ko*' ! -path './kernel/*' ! -path './updates/gmsl/*'
   ```
   On 6.18.37-2.1.0: Wi-Fi (`moal`, `mlan`), DPDK/UIO (`enetc4_uio`, `igb_uio`, `uiodma`),
   `jailhouse`, `cryptodev`, `user_resv_mem`, `kpage_ncache`. None is needed for the camera
   (the board is reached over Ethernet). Tell the user with the question of §Image.
2. **Release name:** `LOCALVERSION` = the image's value plus `-debug` (for `6.18.37-2.1.0`:
   `-2.1.0-debug`), checked with
   `make ARCH=arm64 CROSS_COMPILE=<prefix> LOCALVERSION=<x>-debug kernelrelease`: it must differ
   from `<release>` (for example `6.18.37-2.1.0-debug-g6e694eda40aa`). Store it as
   `debug_kernel_release` and use `LOCALVERSION=<x>-debug` in **every** `make` of the kernel tree
   and of the out-of-tree modules from here on.
3. **Options:** after common/9 §Compiler (the image's config in `.config`, `olddefconfig` done):
   ```bash
   cd <work_dir>/linux-<tag>
   ./scripts/config --enable DYNAMIC_DEBUG --enable FTRACE --enable FUNCTION_TRACER --enable DYNAMIC_FTRACE
   make ARCH=arm64 CROSS_COMPILE=<prefix> LOCALVERSION=<x>-debug olddefconfig
   grep -E '^CONFIG_(DYNAMIC_DEBUG|FTRACE|FUNCTION_TRACER|DYNAMIC_FTRACE)=' .config
   ```
   All four must show `=y` (`DYNAMIC_FTRACE` needs `FUNCTION_TRACER`). Save the config diff
   (`diff <tpl_dir>/board.config .config`) for the final message: only these options and what
   they select may differ.
3a. **Kernel patches** (i.MX93: `fsl,imx93` in the base DTB's compatible; §i.MX93): apply each as
   its own commit, then check the release name again, because the commit changes the `-g<hash>`
   part:
   ```bash
   cd <work_dir>/linux-<tag>
   git apply --check <skill dir>/nxp/patches/imx93-dwc-mipi-csi2-color-mode16.patch && git apply <skill dir>/nxp/patches/imx93-dwc-mipi-csi2-color-mode16.patch
   git -c user.name=gmsl-skill -c user.email=gmsl@localhost commit -qam "dwc-mipi-csi2: 16-bit IPI for RAW on i.MX93 (local)"
   make ARCH=arm64 CROSS_COMPILE=<prefix> LOCALVERSION=<x>-debug kernelrelease
   ```
   Store the printed value as `debug_kernel_release` (it replaces the one of step 2), and copy the
   patch into `<run_dir>/host/patches/` in a verbose run.
4. **Build** in the background (15–30 minutes, as common/9 §Kernel build):
   ```bash
   cd <work_dir>/linux-<tag> && make -j$(nproc) ARCH=arm64 CROSS_COMPILE=<prefix> LOCALVERSION=<x>-debug Image modules > <work_dir>/kernel-build.log 2>&1; echo "exit $?" >> <work_dir>/kernel-build.log
   ```
   common/9 §CRC check is not needed: the kernel and every module come from this tree. Instead,
   after building the ADI modules, check `modinfo -F vermagic <work_dir>/maxim-serdes/*.ko`
   starts with `debug_kernel_release`.
5. **Size check:** the kernel must fit below the DTB address, or the DTB is overwritten when it is
   loaded or when the kernel starts. Check the kernel's **memory size** from its header
   (`image_size`, bytes 16–23, includes the zero-initialised data after the file), not only the
   file size:
   ```bash
   python3 -I -c "import struct,sys; h=open(sys.argv[1],'rb').read(64); t,s=struct.unpack('<QQ',h[8:24]); print('text_offset',t,'image_size',s,'fits' if t+s < 0x93000000-0x90400000 else 'TOO LARGE')" <work_dir>/linux-<tag>/arch/arm64/boot/Image
   ```
   It must print `fits` (limit `0x93000000 - 0x90400000` = 46137344 bytes on the i.MX95; the
   i.MX93's `0x83000000 - 0x80400000` is the same size: use its addresses in the command). Measured on
   6.18.37-2.1.0: NXP's kernel `image_size` 36503552; the debug kernel 45809664 (file 44968448):
   **only 320 KiB left**, so enable nothing beyond the four options of step 3. Read `loadaddr` and
   `fdt_addr_r` from the image's environment (`u-boot-initial-env`, nxp/3 §Image step 1) on other
   boards. If it does not fit, stop and report (do not move the addresses).
6. **Install** into the mounted image (after the ADI modules are built; root rights as common/0
   §Sudo password). The kernel's modules go through a staging folder in `<work_dir>` (no root
   needed for `make`):
   ```bash
   cd <work_dir>/linux-<tag>
   rm -rf <work_dir>/debug-modules && make ARCH=arm64 CROSS_COMPILE=<prefix> LOCALVERSION=<x>-debug INSTALL_MOD_PATH=<work_dir>/debug-modules INSTALL_MOD_STRIP=1 modules_install
   DREL=<debug_kernel_release>
   rm -f <work_dir>/debug-modules/lib/modules/$DREL/build <work_dir>/debug-modules/lib/modules/$DREL/source   # links to the host's tree
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c "cp -a <work_dir>/debug-modules/lib/modules/$DREL /tmp/gmsl_nxp_rootfs/lib/modules/ \
     && mkdir -p /tmp/gmsl_nxp_rootfs/lib/modules/$DREL/updates/gmsl \
     && cp <work_dir>/maxim-serdes/*.ko [<work_dir>/<sensor>/<sensor>.ko] /tmp/gmsl_nxp_rootfs/lib/modules/$DREL/updates/gmsl/ \
     && depmod -b /tmp/gmsl_nxp_rootfs $DREL \
     && cp arch/arm64/boot/Image /tmp/gmsl_nxp_boot/Image-gmsl-debug"
   ls -l /tmp/gmsl_nxp_boot/Image /tmp/gmsl_nxp_boot/Image-gmsl-debug
   grep -E 'updates/gmsl/(max|<sensor>)|gpu/arm/midgard/mali_kbase' /tmp/gmsl_nxp_rootfs/lib/modules/$DREL/modules.dep
   ```
   The `modules.dep` check must list the ADI modules with their `updates/gmsl/` path and
   `mali_kbase`. Firmware (`/lib/firmware`) is shared and stays as it is. NXP's `Image` and
   `/lib/modules/<release>` are not touched; the ADI modules exist only for the debug kernel.
   Record `kernel_release` = `debug_kernel_release`, `modules_built`, `drivers_ok`.
7. **U-Boot:** nxp/3 §Image also sets `image=Image-gmsl-debug` (step 2 there).
8. **First boot** (nxp/5 §Order, driver check): `uname -r` must print `debug_kernel_release`;
   check the debugging features and the GPU:
   ```bash
   ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "uname -r; mount | grep -q ' /sys/kernel/debug ' || mount -t debugfs none /sys/kernel/debug; test -e /sys/kernel/debug/dynamic_debug/control && echo dyndbg ok; grep -qw function /sys/kernel/tracing/available_tracers && echo ftrace ok; lsmod | grep -q mali_kbase && echo gpu ok; pgrep -x weston >/dev/null && echo weston ok"
   ```
   The ADI drivers (`max-serdes`, `max9296a`, `max96717`) have **no** `dev_dbg`/`pr_debug`
   calls, so dynamic debug has nothing to switch on in them: trace them with ftrace instead (163
   functions on 6.18.37; most calls are in the shared `max_serdes` module, for example
   `max_des_init_state`, `__max_des_set_routing`). Dynamic debug covers the rest of the camera path (`videodev`,
   `v4l2_fwnode`, `i2c_core`, `imx8_isi`, `imx8_isi_capture`). Examples for the final message:
   ```bash
   echo 'module imx8_isi +p' > /sys/kernel/debug/dynamic_debug/control      # messages in dmesg
   cd /sys/kernel/tracing && echo ':mod:max_serdes' > set_ftrace_filter && echo ':mod:max9296a' >> set_ftrace_filter && echo function > current_tracer
   cat trace_pipe            # start streaming in another shell; echo nop > current_tracer to stop
   ```
   If the board does not boot the debug kernel, nxp/5 §Troubleshooting lists the recovery.

**Tested** on the FRDM-IMX95-PRO with NXP 6.18.37-2.1.0 (installed on a running board next to
NXP's kernel, the same files and environment change as above; October 2026): boots in about 35 s;
dynamic debug (9219 call sites) and the `function`/`function_graph` tracers work; `mali_kbase`
and Weston run; the GMSL drivers probe; 30 frames at 1920x1080 RAW10, 30 fps; the GStreamer viewer
at 640x480 is smooth. `image_size` 45809664 (320 KiB below the DTB address, step 5). The SD/eMMC
image path itself (install into the mounted image, offline environment) was not run with the
debug kernel.

Undo (final message): on the U-Boot console `setenv image Image; saveenv` boots NXP's kernel again
(the GMSL drivers are then missing: they were built for the debug kernel; build them for NXP's
kernel with §Live board), and `Image-gmsl-debug` and `/lib/modules/<debug_kernel_release>` can
then be deleted.

