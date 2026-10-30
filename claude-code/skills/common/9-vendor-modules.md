# Common 9: ADI GMSL modules for a vendor kernel (out of tree)

Builds the ADI GMSL drivers (`maxim-serdes`: `max-serdes`, `max96717`, `max9296a`/`max96724`)
and, if needed, a patched sensor driver, as out-of-tree modules for a kernel that is already on
the board or in its image (a vendor kernel, e.g. NXP's `linux-imx`). The vendor kernel itself is
not replaced: the modules go to `updates/`, which takes priority. Used by nxp/1 (values for the
NXP BSP) and generic/0 (best effort on other boards). The Raspberry Pi path builds a whole kernel
with the drivers built in instead (rpi/0–rpi/2).

Everything is built on the host, in `<work_dir>` (default `~/gmsl-camera-work`, state
`work_dir`). Keep the driver sources in a local git repository **per driver** (one commit per
change: the original sources first, then every fix, with the reason in the message), so the
result can be exported as patches (`git format-patch`); **no remote, never push it anywhere**.
Tell the user the plan and the time (the kernel build takes 15–30 minutes) and ask before
starting.

**Permissions:** this step runs code downloaded during the run: a cross toolchain, `make` in the
kernel and U-Boot trees, and ADI's `gen_gmsl_dts.py` (common/4 §Generate). Claude Code's
permission settings can refuse to run downloaded programs even after the user approved the plan.
Tell the user before starting which programs will run (the toolchain's `bin/` folder, `make` in
`<work_dir>/linux-*` and the U-Boot tree, `python3` on the templates), so they can allow them
(permission rules, or the default permission mode). If one is refused, stop that step and report
it; do not rewrite the downloaded program to get around the refusal. A toolchain's `.sha256asc`
comes from the same server as the archive: it detects a damaged download, it is not a signature.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save a copy of the build logs (`<work_dir>/kernel-build.log`, the module builds) in `<run_dir>/host/`, the config diff, the CRC comparison, and `git format-patch` of each driver repository into `<run_dir>/host/patches/` (§Verbose mode §What to save).

## Inputs

The build needs four things from the target kernel. They come from the live board over SSH, or,
without a running board, from its image (mounted by nxp/0):

| Input | Live board (SSH, control socket of common/5) | Image (rootfs mounted at `/tmp/gmsl_nxp_rootfs`, boot partition at `/tmp/gmsl_nxp_boot`) |
|-------|----------------------------------------------|------------------------------------------------|
| Kernel release (`<rel>`) | `uname -r` | `ls /tmp/gmsl_nxp_rootfs/lib/modules/` (one folder per release; if more than one, ask which one the boot partition's kernel is) |
| Kernel config (`<tpl_dir>/board.config` in both cases) | `zcat /proc/config.gz` (already saved as `<tpl_dir>/board.config` by common/2) | Already saved as `<tpl_dir>/board.config` by nxp/0 §Read the image (`scripts/extract-ikconfig` on `/tmp/gmsl_nxp_boot/Image`, or on `/tmp/gmsl_nxp_rootfs/lib/modules/<rel>/kernel/kernel/configs.ko*`, decompressed first; needs `CONFIG_IKCONFIG`). For another image, write it to the same file the same way |
| A vendor module for §CRC check | Any module of the board, e.g. `modinfo -n videodev`; run `modprobe --dump-modversions` on the board (decompress first) | A module from `/tmp/gmsl_nxp_rootfs/lib/modules/<rel>/kernel/...` (e.g. `videodev.ko*`), copied and decompressed on the host; run `modprobe --dump-modversions` on the host (`kmod` package) |
| Module folder (§Install, name collisions in §ADI drivers) | `/lib/modules/$(uname -r)` on the board (`find ... -name 'max*'`, `modules.builtin`) | `/tmp/gmsl_nxp_rootfs/lib/modules/<rel>` |

The release hash (`-g<12 hex>` at the end of `<rel>`, or `kernel_source` in the assessment) and
`CONFIG_CC_VERSION_TEXT` of the config decide the source tag (§Kernel source) and the compiler
(§Compiler). Record `kernel_release`, `localversion`, `toolchain_prefix` and `kernel_src_path`
in `<state>`.

## Kernel source

Shallow-clone the vendor tree at the tag or commit found for the release (the platform file gives
the tree; generic: the assessment's **Kernel source** layer, found with
`git ls-remote <vendor tree> 'refs/tags/*' | grep <hash>`):
```bash
mkdir -p <work_dir> && cd <work_dir>
git clone --depth 1 --branch <tag> <tree> linux-<tag>
```
Copy the board's config (unzipped) to `.config` and run olddefconfig:
```bash
cd <work_dir>/linux-<tag>
cp <tpl_dir>/board.config .config      # live board (common/2) and image (nxp/0) alike
make ARCH=<arch> CROSS_COMPILE=<prefix> olddefconfig
```
Find the `LOCALVERSION` that reproduces the board's release (Yocto builds often pass one on the
command line, for example `-2.0.0`), and check it against `uname -r` (or `<rel>`) **exactly**:
```bash
make ARCH=<arch> CROSS_COMPILE=<prefix> LOCALVERSION=<x> kernelrelease
```
A release that differs in any character means the modules will not load (`vermagic`).

## Compiler

Compare `CONFIG_CC_VERSION_TEXT` of the board with the host cross-compiler:
```bash
grep CONFIG_CC_VERSION_TEXT <tpl_dir>/board.config
<prefix>gcc --version | head -1
```
The **major version must match**: with `CONFIG_MODVERSIONS=y`, symbol CRCs are computed from type
definitions that depend on compiler features (`CONFIG_CC_HAS_*`, for example `COUNTED_BY` and
`MULTIDIMENSIONAL_NONSTRING` with GCC 15). After `olddefconfig`, diff the board's config against
`.config`: only version numbers may differ, not `CC_HAS_*` options:
```bash
diff <(grep -v '^#' <tpl_dir>/board.config | sort) <(grep -v '^#' .config | sort)
```
If the host compiler does not match, ask to download a toolchain of the same major version (Arm
GNU Toolchain for Arm boards, `https://developer.arm.com/-/media/Files/downloads/gnu/<ver>/binrel/`,
for example `arm-gnu-toolchain-<ver>-x86_64-aarch64-none-linux-gnu.tar.xz`). Take the **newest**
release of the board's GCC major version `<M>`. Arm's downloads page no longer lists the releases
(it redirects to gitlab.arm.com, checked October 2026), so find the newest one by probing the
download server (a missing version answers 404):
```bash
M=<board GCC major, e.g. 15>
for minor in 9 8 7 6 5 4 3 2 1 0; do for rel in 3 2 1; do v=$M.$minor.rel$rel
  code=$(curl -sSIL -o /dev/null -m 20 -w '%{http_code}' https://developer.arm.com/-/media/Files/downloads/gnu/$v/binrel/arm-gnu-toolchain-$v-x86_64-aarch64-none-linux-gnu.tar.xz.sha256asc)
  [ "$code" = 200 ] && { echo "$v"; break 2; }; done; done
```
The printed `<ver>` is the newest release; if nothing is printed (no network, or Arm changed the
server), tell the user and ask for the toolchain to use. Verify its published
`.sha256asc` (`sha256sum -c`; a damaged-download check only, see **Permissions**), unpack it into
`<work_dir>`, and use it (`<prefix>` = `<work_dir>/<toolchain>/bin/aarch64-none-linux-gnu-`).
Run `olddefconfig` again with the new compiler and repeat the diff.

## Kernel build

Only needed for `Module.symvers` when `CONFIG_MODVERSIONS=y`; the image is not installed. Run it
in the background (15–30 minutes) and wait for it to finish:
```bash
cd <work_dir>/linux-<tag> && make -j$(nproc) ARCH=<arch> CROSS_COMPILE=<prefix> LOCALVERSION=<x> Image modules > <work_dir>/kernel-build.log 2>&1; echo "exit $?" >> <work_dir>/kernel-build.log
```
(Bash tool with `run_in_background`; check `tail <work_dir>/kernel-build.log` for `exit 0`.)
Without `CONFIG_MODVERSIONS`, `make modules_prepare` is enough. Record `kernel_built` in
`<state>`.

## CRC check

(Not used when the whole kernel is replaced by one built here, as the NXP debug kernel of nxp/1
§Debug kernel: then the kernel and every module come from the same tree.)

**Verify the CRCs** against a module from the board before trusting the tree: dump the CRCs of
a vendor module and compare them with `Module.symvers`. Any mismatch means the modules would be
refused (`disagrees about version of symbol`); fix the compiler or config first.
```bash
# live board: on the board (decompress .ko.xz/.ko.zst first, in /tmp)
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "f=\$(modinfo -n videodev); case \$f in *.xz) xz -dc \$f > /tmp/gmsl_m.ko;; *.zst) zstd -dcq \$f > /tmp/gmsl_m.ko;; *) cp \$f /tmp/gmsl_m.ko;; esac; modprobe --dump-modversions /tmp/gmsl_m.ko; rm -f /tmp/gmsl_m.ko" > /tmp/gmsl_board_crcs.txt
# image: on the host
modprobe --dump-modversions /tmp/gmsl_m.ko > /tmp/gmsl_board_crcs.txt
# compare
awk '{print $2, $1}' /tmp/gmsl_board_crcs.txt | sort > /tmp/gmsl_crc_a.txt
awk '{print $2, $1}' <work_dir>/linux-<tag>/Module.symvers | sort > /tmp/gmsl_crc_b.txt
join /tmp/gmsl_crc_a.txt /tmp/gmsl_crc_b.txt | awk '$2 != $3' | head
```
The last command must print nothing (every symbol the vendor module uses has the same CRC).

## ADI drivers

Copy `drivers/media/i2c/maxim-serdes` from the **newest** ADI GMSL branch (`adi_gmsl_branch`: read
`<skill dir>/common/4-device-tree.md` and follow the **Latest ADI GMSL branch** paragraph of
§Templates, if it is not in the state yet). Newer kernels than that branch's base are handled by
§Port: sparse checkout of that folder and the bindings:
```bash
cd <work_dir> && git clone --depth 1 --filter=blob:none --sparse -b <adi_gmsl_branch> https://github.com/analogdevicesinc/linux.git adi-linux
cd adi-linux && git sparse-checkout set drivers/media/i2c/maxim-serdes Documentation/devicetree/bindings/media/i2c
mkdir -p <work_dir>/maxim-serdes && cp drivers/media/i2c/maxim-serdes/* <work_dir>/maxim-serdes/
cd <work_dir>/maxim-serdes && git init -q && git add -A && git commit -qm "ADI maxim-serdes from <adi_gmsl_branch> (original)"
```
Replace its Kconfig/Makefile with a `Kbuild` (commit it):
```make
max-serdes-objs := max_serdes.o max_ser.o max_des.o
obj-m += max-serdes.o max96717.o max9296a.o   # or max96724.o: the modules needed
```
**Leave out any module whose name the vendor kernel already uses** (for example NXP ships its own
`max96724.ko`; check the module folder of §Inputs with `find <modules dir> -name 'max*'`): two
modules with one name cannot coexist. If the chosen deserializer's module is one of them, tell the
user (it cannot be installed without replacing the vendor module) and ask.

Build:
```bash
cd <work_dir>/maxim-serdes && make -C <work_dir>/linux-<tag> ARCH=<arch> CROSS_COMPILE=<prefix> LOCALVERSION=<x> M=$PWD KCFLAGS=-Werror modules
```
Record `drivers_added` in `<state>`.

## Port

Compile errors from API changes between the ADI branch and the vendor kernel are expected. Fix
them one at a time with the smallest change, each as its own commit with the reason in the
message, and show the user the diff. Changes found between 6.13 and 6.18:
- `PIN_CONFIG_OUTPUT` → `PIN_CONFIG_LEVEL`;
- `gpio_chip.set_rv` → `.set`;
- `V4L2_CID_USER_MAXIM_SERDES_BASE` is defined only in ADI's uapi header: define it in
  `max_serdes.h` on a free `V4L2_CID_USER_BASE + 0x...` range (ADI's `0x11e0` is UVC's in 6.18,
  so read the vendor's `include/uapi/linux/v4l2-controls.h` and pick a range nobody uses).

**Stop condition:** if a fix needs more than renames (the kernel lacks the V4L2 streams API or I2C
ATR, or core structures changed), that is a real backport. On NXP, stop and report it. On generic
boards (best effort, generic/0 §Best-effort plan), tell the user what is missing and ask whether
to try the backport (rpi/1's patch-backport approach as the reference); otherwise stop and
report.

## Pixel mode

ADI's deserializer drivers choose the GMSL mode themselves: **tunnel mode** whenever the
deserializer and every serializer support it and each pipe goes to one PHY
(`max_des_populate_remap_context_mode()` in `max_des.c`), otherwise pixel mode. They have **no**
module parameter or device tree property for it; do not offer one that does not exist. Some
receivers need pixel mode (the platform file says which: nxp/1 §i.MX93). Then add a local module
parameter, as its own commit marked as a local addition:
```bash
cd <work_dir>/maxim-serdes && git apply --check <skill dir>/common/patches/max_des-force-pixel-mode.patch \
  && git apply <skill dir>/common/patches/max_des-force-pixel-mode.patch \
  && git -c user.name=gmsl-skill -c user.email=gmsl@localhost commit -qam "max_des: force_pixel_mode parameter (local addition, not in ADI's driver)"
```
It adds `force_pixel_mode` (bool, `0444`) to `max-serdes` and, when set, restricts
`max_des_get_supported_modes()` to `MAX_SERDES_GMSL_PIXEL_MODE` (every ADI deserializer supports
it). Build as usual, then check `modinfo -p max-serdes.ko` lists `force_pixel_mode`. It is set
with `/etc/modprobe.d/gmsl.conf`: `options max_serdes force_pixel_mode=1` (in the mounted image
on the image path); the platform file's final message lists the file and its undo (delete it,
reboot). Check after the next stream start: `v4l2-ctl -d <deserializer subdev> --log-status`
shows `mode: pixel`.

## Sensor driver

The ADI serializer drivers get the sensor's CSI-2 data type and virtual channel through the
sensor driver's `.get_frame_desc` pad operation. Mainline and vendor sensor drivers often lack it
(the 6.18 `imx219` does); streaming then fails with `-ENOTTY` (`VIDIOC_STREAMON returned -1
(Inappropriate ioctl for device)`) and the CSI-2 receiver logs
`Remote sub-device ... should implement .get_frame_desc`. Check the sensor driver in the kernel
tree:
```bash
grep -n get_frame_desc <work_dir>/linux-<tag>/drivers/media/i2c/<sensor>.c
```
If it is missing, add a **minimal** one in a copy of the driver, in its own git repository
(`<work_dir>/<sensor>`; original first, then the patch): one entry: the active format's code,
stream 0, VC 0, the CSI-2 data type for the format's bit depth (`MIPI_CSI2_DT_RAW8`/`RAW10`), as
in ADI's `imx219.c` without its embedded-data stream; `.type =
V4L2_MBUS_FRAME_DESC_TYPE_CSI2`. Build it out of tree like the serdes drivers (`Kbuild`:
`obj-m += <sensor>.o`), and install it to `updates/` (it then takes priority over the vendor
module; ask first and say so).

## Retry patch

Only when the user agreed to the I2C retry workaround (common/6 §Retry workaround, after the last
row of common/6 §Results): apply `<skill dir>/common/i2c_retry_regmap.h` to the deserializer,
serializer and sensor driver repositories as described there (one commit per driver, marked as a
workaround), rebuild them with the same `make ... M=$PWD` command, and install them again
(§Install on the board, or §Install into an image). Report the `i2c-retry:` recoveries after the
next boot and stream start, and list the workaround in the final message.

## Install on the board

Ask first. Before installing, look for modules with the same names anywhere under `updates/`
(for example from an earlier manual install of the ADI drivers):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "find /lib/modules/\$(uname -r)/updates \( -name 'max*.ko*' -o -name '<sensor>.ko*' \) 2>/dev/null"
```
If modules with the names of the new ones (`max-serdes`/`max_serdes`, `max96717`, `max9296a`,
`max96724`, `<sensor>`) are in another `updates/` subfolder than `updates/<subfolder>/`, tell the
user (the folder, the files, `modinfo -F depends` of each) and ask (header: "Old modules"):
- **Install into that folder (Recommended when it holds only these ADI/sensor modules)**: copy the
  old files to `~/Workspace/gmsl_modules_backup/` first, then use that folder as `<subfolder>`
  (replacing those files) below;
- **Move that folder aside**: move it to `~/Workspace/` (with the user's consent;
  `mv /lib/modules/$(uname -r)/updates/<old> ~/Workspace/updates-<old>.moved` with root rights),
  then install into `updates/<subfolder>/` as below;
- **Stop**.
Never leave two modules with the same name under `updates/` (`depmod` then picks one of them, not
necessarily the new one). Never touch files there that are not one of these modules (vendor
modules, e.g. NXP's). Note what was replaced or moved for the final message (with how to put it
back).

Copy the modules to `/lib/modules/$(uname -r)/updates/<subfolder>/` (the platform file names
`<subfolder>`, e.g. `gmsl`), run `depmod -a`:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "mkdir -p /tmp/gmsl_mods"
scp -o ControlPath=/tmp/gmsl_ssh_sock -o BatchMode=yes <work_dir>/maxim-serdes/*.ko [<work_dir>/<sensor>/<sensor>.ko] <user>@<ip>:/tmp/gmsl_mods/
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'mkdir -p /lib/modules/\$(uname -r)/updates/<subfolder> && cp /tmp/gmsl_mods/*.ko /lib/modules/\$(uname -r)/updates/<subfolder>/ && depmod -a'; rm -rf /tmp/gmsl_mods"
```
Check with `modinfo -n <module>` that the new ones are picked (the path must be in
`updates/<subfolder>/`), and test-load them once before the device tree exists (`modprobe
max9296a` / `max96717`: no "disagrees about version of symbol" in `dmesg`), then unload them:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "for m in max-serdes max96717 <des module> <sensor>; do modinfo -n \$m; done; SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'modprobe <des module> && modprobe max96717; dmesg | tail -5; modprobe -r max96717 <des module> max-serdes'"
```
A module that is part of a live media graph cannot be unloaded (`is in use`); replacing it needs
a reboot (common/5 §Reboot).

**Never touch other files in `updates/`** (vendors ship their own modules there, e.g. NXP): only
`updates/<subfolder>/` belongs to this skill, plus the same-named modules the user agreed to
replace or move above. Undo: delete `updates/<subfolder>` and run `depmod -a` (and put back what
was replaced or moved: the backup in `~/Workspace/gmsl_modules_backup/`, or the folder moved to
`~/Workspace/`). The final message lists all of it. Record `modules_built`/`drivers_ok` in `<state>`, then run the driver check again
(common/3 §Driver check).

## Install into an image

Without a live board, install into the mounted rootfs of the image (nxp/0 §Mount):
```bash
REL=<rel>
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mkdir -p /tmp/gmsl_nxp_rootfs/lib/modules/$REL/updates/gmsl
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp <work_dir>/maxim-serdes/*.ko [<work_dir>/<sensor>/<sensor>.ko] /tmp/gmsl_nxp_rootfs/lib/modules/$REL/updates/gmsl/
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A depmod -b /tmp/gmsl_nxp_rootfs $REL
```
`depmod -b` on the host (package `kmod`) writes the image's `modules.dep` for that release; it
does not touch the host's own modules. Check:
```bash
ls -l /tmp/gmsl_nxp_rootfs/lib/modules/$REL/updates/gmsl/
grep -E 'updates/gmsl/(max|<sensor>)' /tmp/gmsl_nxp_rootfs/lib/modules/$REL/modules.dep
```
Every module must be listed with its `updates/gmsl/` path. The same rule applies: never touch
other files in `updates/`. The test load happens on the first boot (common/3 §Driver check on the
live board).
