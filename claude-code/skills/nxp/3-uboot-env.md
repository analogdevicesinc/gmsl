# NXP 3: U-Boot environment (boot the GMSL DTB)

NXP i.MX values for common/10: make U-Boot load `imx95-19x19-frdm-pro-gmsl.dtb` (nxp/2 §Merged
DTB) instead of the base DTB, by setting its `fdtfile` variable:
- **§Live board**: from Linux with U-Boot's own `fw_setenv`, built from NXP's U-Boot source (NXP
  images have no `fw_setenv` and no `/etc/fw_env.config`); no serial console needed.
- **§Image**: on the build path, a complete environment written into the `.wic` before flashing,
  so the first boot already uses the GMSL DTB.

Placeholders as in the orchestrator; `<tpl_dir>` = `/tmp/gen_gmsl_dts/nxp`.

## Values

FRDM-IMX95-PRO, NXP i.MX Linux 6.18.20-2.0.0 (tested on eMMC boot):

| Item | Value |
|------|-------|
| U-Boot version | `2026.04-lf_v2026.04+g6eeef838dac+p0` (`/proc/device-tree/chosen/u-boot,version`; offline: `strings` of the imx-boot binary, nxp/0 §Read the image) |
| U-Boot tree / tag | https://github.com/nxp-imx/uboot-imx, tag `lf-6.18.20-2.0.0` (the kernel and U-Boot tags of one BSP release have the same name) |
| Defconfig | `imx95_19x19_frdm_pro_defconfig` |
| Environment | `CONFIG_ENV_IS_IN_MMC`, MMC **user area**, offset `0x700000` (7 MiB), size `0x4000` (16 KiB), **no redundant copy**; found by its valid CRC32 |
| `fw_env.config` | `/dev/mmcblk0 0x700000 0x4000` on eMMC boot (eMMC = `mmcblk0`). On SD boot U-Boot uses the boot device, so the environment is on the SD card at the same offset (untested): run common/10 §CRC search over every `mmcblk<N>`, expect the SD card (e.g. `mmcblk1`) |
| `dd` offset | `0x700000 / 16384` = 448 blocks of 16 KiB (for `dd`) |
| Cross compiler for envtools | the host's `aarch64-linux-gnu-` worked (check the glibc as in common/10 §Build envtools; the image's glibc from nxp/0 §Read the image) |
| `bootcmd` | `bootflow scan -l; run bsp_bootcmd`: U-Boot standard boot runs first and boots any extlinux, boot script or EFI entry it finds; only if there is none does `bsp_bootcmd` load `fdtfile` from the FAT boot partition. Check that the boot partition has no `extlinux/extlinux.conf`, `boot.scr` or `EFI/` (otherwise `fdtfile` may be ignored: tell the user) |
| `bootdelay` | 2 s (hold Enter on the serial console while the board resets to reach `=>`) |
| Base / GMSL DTB | `imx95-19x19-frdm-pro.dtb` / `imx95-19x19-frdm-pro-gmsl.dtb` |

i.MX93 11x11 EVK, 6.18.37-2.1.0: defconfig `imx93_11x11_evk_defconfig`; environment in MMC at
`0x700000`, `0x4000`, no redundant copy (the boot device's); `image=Image`, `loadaddr=0x80400000`,
`fdt_addr_r=0x83000000` (the same 44 MiB below the DTB as on the i.MX95, nxp/1 §Debug kernel
step 5); base / GMSL DTB `imx93-11x11-evk.dtb` / `imx93-11x11-evk-gmsl.dtb`.

Other i.MX boards: the same procedure; the defconfig is `configs/<board>_defconfig` matching the
device tree's model (search `configs/` for the board name), and the location comes from
common/10 §Environment location and §CRC search, never from this table.

## Live board

The control socket of common/5 §Control socket is open, and nxp/2 §Merged DTB copied
`imx95-19x19-frdm-pro-gmsl.dtb` next to the base DTB on the boot partition.

Read `<skill dir>/common/10-uboot-env.md` and follow §Version, §Environment location, §CRC search,
§Build envtools, §Verify and §Backup and write with the values of §Values. The login on NXP images
is `root` and the image may have no `sudo`: run common/10's board commands as root, without
`SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c` (the commands below show that form):
- tools in `~/Workspace/uboot-env/` on the board (`fw_printenv`, the `fw_setenv` symlink,
  `fw_env.config`), used with `-c fw_env.config`;
- the backup of the raw environment there and on the host (`<work_dir>/env_backup_<hostname>.bin`,
  sha256 noted);
- the write, after the user agreed (header: "Boot change", showing `fdtfile:
  imx95-19x19-frdm-pro.dtb → imx95-19x19-frdm-pro-gmsl.dtb` and the recovery below):
  ```bash
  ssh -S /tmp/gmsl_ssh_sock root@<ip> -- "cd ~/Workspace/uboot-env && ./fw_setenv -c fw_env.config fdtfile imx95-19x19-frdm-pro-gmsl.dtb && ./fw_printenv -c fw_env.config fdtfile"
  ```
  (a non-root login: wrap the command in `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c '...'`).

If §CRC search finds **no valid copy** (the environment was never saved on this board), either
the user runs `saveenv` once on the U-Boot console (common/10 §Console fallback) or, with the
user's agreement, write a fresh environment as in §Image below, but to the board's device (via
the control socket, after the backup).

Then reboot with common/5 §Reboot (and its §Probe check). After the reboot, check that the GMSL DTB
is the one running (it must print the deserializer node's path):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "find /proc/device-tree/ -name 'gmsl-deserializer*' | head -1"
```
If it is not, U-Boot did not use the written environment (wrong copy, or `bootflow scan` found
another boot entry): see the recovery, and fall back to common/10 §Console fallback.

Recovery (tell the user before the write):
- from Linux (as root): `cd ~/Workspace/uboot-env && ./fw_setenv -c fw_env.config fdtfile imx95-19x19-frdm-pro.dtb && reboot`;
- the backup (as root, in `~/Workspace/uboot-env`): `dd if=env_backup.bin of=/dev/mmcblk0 bs=16384 seek=448 count=1 conv=notrunc`;
- on the U-Boot console: `setenv fdtfile imx95-19x19-frdm-pro.dtb; saveenv; boot`, or
  `env default -a; saveenv` (the built-in defaults boot the base DTB).

Record `env_written` in `<state>`.

## Image

Build path, after nxp/1 §Image and nxp/2 §Merged DTB (the GMSL DTB is already on the image's boot
partition). First read `<skill dir>/nxp/0-base-image.md` and follow §Unmount (the environment is
written into the `.wic` file, outside its partitions, while nothing is mounted).

Read `<skill dir>/common/10-uboot-env.md` and follow §Version (offline: `strings` of
`<imx_boot>`), §Environment location and **§Offline env image** with the values of §Values and
`<image>` = `<wic>` (`<work_dir>/nxp-image/<name>.wic`):
1. `make imx95_19x19_frdm_pro_defconfig && make CROSS_COMPILE=<prefix> u-boot-initial-env` in the
   U-Boot tree of the image's release (the tag of §Values: the same name as the kernel tag, e.g.
   `lf-6.18.37-2.1.0`); check that `fdtfile` = `imx95-19x19-frdm-pro.dtb` and the `bootcmd` of
   §Values are in `u-boot-initial-env` (the same tag as the image's U-Boot, so this is the
   built-in environment). The file writes the value in quotes,
   `fdtfile="imx95-19x19-frdm-pro.dtb"` (checked with `lf-6.18.37-2.1.0`): grep for
   `^fdtfile=`, not for the unquoted line.
2. Set `fdtfile=imx95-19x19-frdm-pro-gmsl.dtb` (the `sed` of common/10 §Offline env image step 3
   replaces the whole line, quotes included; U-Boot accepts the value with or without them). With
   the debug kernel (`nxp_debug_kernel` = `True`, nxp/1 §Debug kernel), also set
   `image=Image-gmsl-debug` the same way (default `image=Image`; check that the line exists).
3. `mkenvimage` (common/10 §Offline env image, step 4): use the host's `mkenvimage` from the
   `u-boot-tools` package (`command -v mkenvimage`; install it with
   `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y u-boot-tools` if missing, and list it in the final
   message as installed on the host). Do not build it with `make tools-only` in NXP's tree: it
   also builds `mkeficapsule`, which needs the GnuTLS headers (`gnutls/gnutls.h: No such file`,
   seen with `lf-6.18.37-2.1.0`), and `make tools/mkenvimage` is not a valid target. Then
   `mkenvimage -s 0x4000 -o /tmp/gmsl_env.bin /tmp/gmsl_env.txt` (no `-r`: no redundant copy).
   The binary format (CRC32 + `name=value\0` pairs) is the same in every U-Boot version.
4. Check the first partition starts after `0x704000` (nxp/0 §Read the image, step 5).
5. Write it into the image:
   ```bash
   dd if=/tmp/gmsl_env.bin of='<wic>' bs=16384 seek=448 conv=notrunc
   ```
6. Verify with the CRC script (common/10 §Offline env image, step 7): `CRC OK` and
   `fdtfile=imx95-19x19-frdm-pro-gmsl.dtb` (and `image=Image-gmsl-debug` with the debug kernel).

`uuu ... emmc_all` (nxp/4 §eMMC with uuu) writes the whole `.wic` to the eMMC user area, and `dd`
of common/11 writes it to the SD card, so the environment at `0x700000` goes with it. nxp/4 checks
it again on the written medium where it can.

After the first boot: the board boots the GMSL DTB (and the debug kernel) without a console.
Recovery: on the U-Boot console `setenv fdtfile imx95-19x19-frdm-pro.dtb; setenv image Image;
saveenv` or `env default -a; saveenv`, or the
stock `.wic` flashed again. The `fw_setenv` way of §Live board applies only if common/10 §Build
envtools is run on the board later (the image has no envtools).
Record `env_written` in `<state>`, delete `/tmp/gmsl_env.txt`, `/tmp/gmsl_env.bin` (the
orchestrator's §Cleanup also removes `/tmp/gmsl_*`), and continue with nxp/4.
