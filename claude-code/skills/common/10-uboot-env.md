# Common 10: U-Boot environment (fdtfile) without a serial console

Makes U-Boot boot the merged GMSL DTB (common/4 §Merged DTB) by changing its `fdtfile` variable:
- **live board** without `fw_setenv` (common on vendor images): build U-Boot's own envtools from
  the matching vendor U-Boot source, find the saved environment by its CRC, and write it from
  Linux (§Version to §Backup and write);
- **fresh medium** (image prepared on the host, nothing booted yet): write a complete environment
  image into the image file or device (§Offline env image);
- **the user on the serial console**, when neither works (§Console fallback).

Used by nxp/3 (values for the NXP BSP) and generic/1 §Install device tree. Before any write,
tell the user how to recover (§Backup and write) and ask (header: "Boot change"); show the exact
change (old → new) in the question. Everything is built on the host in `<work_dir>`; see the
**Permissions** paragraph of common/9 (`make` runs in the downloaded U-Boot tree). Record
`env_written` in `<state>`.

The board commands below use `echo '<pw>' | sudo -S` for a normal login. Logged in as `root`
(NXP images, which may have no `sudo`): run the same commands directly, e.g.
`ssh -S /tmp/gmsl_ssh_sock root@<ip> -- "cd ~/Workspace/uboot-env && ./fw_printenv -c fw_env.config fdtfile"`
(nxp/3 §Live board shows the NXP commands in that form).

## Version

The U-Boot version and commit:
- live board: `tr -d '\0' < /proc/device-tree/chosen/u-boot,version` (assessment
  `device_tree_and_boot`), e.g. `2026.04-lf_v2026.04+g6eeef838dac+p0`;
- offline inputs (image only): from the boot loader binary, `strings` of the imx-boot / flash
  binary (or the first MiB of the image, where it is written):
  ```bash
  strings <imx-boot or image> | grep -m3 -E 'U-Boot 20[0-9]{2}\.[0-9]{2}'
  ```
  (`U-Boot 20xx.xx-<vendor version>+g<hash>...`).

The hash after `g` is the commit. Find the vendor U-Boot tree's tag at that commit (NXP:
`https://github.com/nxp-imx/uboot-imx`, tags `lf-*`; other vendors: the tree the platform file or
the user names) and shallow-clone it:
```bash
git ls-remote <tree> 'refs/tags/*' | grep <hash>
mkdir -p <work_dir> && cd <work_dir> && git clone --depth 1 --branch <tag> <tree> uboot-<tag>
```
No tag at that commit: try the branches (`git ls-remote --heads <tree> | grep <hash>`); if
nothing matches, ask the user for the source, or use §Console fallback.

## Environment location

From the board's defconfig (`configs/<board>_defconfig`, the one matching the device tree's
model; the platform file names it, e.g. nxp/3 §Values):
```bash
cd <work_dir>/uboot-<tag> && grep -E 'CONFIG_ENV_(IS_IN_[A-Z]+|OFFSET|OFFSET_REDUND|SIZE|SECT_SIZE)|CONFIG_SYS_MMC_ENV_(DEV|PART)|CONFIG_ENV_SIZE_REDUND' configs/<board>_defconfig
```
Note `CONFIG_ENV_IS_IN_MMC` (or `_SPI_FLASH`, `_FAT`, ...), `CONFIG_ENV_OFFSET`,
`CONFIG_ENV_SIZE`, `CONFIG_ENV_OFFSET_REDUND` (redundant copy), and the device index
(`CONFIG_SYS_MMC_ENV_DEV`/`_PART`; the board code may pick the boot device at runtime instead, so
the index alone is not proof). Values not in the defconfig come from the Kconfig defaults:
`make <board>_defconfig && grep -E '^CONFIG_ENV_' .config`. This file covers the environment in
raw MMC (user area or boot partitions); other locations (`ENV_IS_IN_FAT`, SPI flash): adapt
the device and offset, or use §Console fallback.

## CRC search

Find the copy with a valid CRC, **read-only**, over every candidate: the `ENV_SIZE` bytes at
`ENV_OFFSET` of each `/dev/mmcblk<N>` user area, `mmcblk<N>boot0`, `boot1` (and the redundant
offset). First check that the offset lies **before the first partition** (`sudo fdisk -l
/dev/mmcblk<N>`: start sector × 512 > `ENV_OFFSET` + `ENV_SIZE`). Read the candidates on the
board into its `/tmp`, copy them to the host, and delete them on the board:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "lsblk -dno NAME,SIZE | grep mmcblk; echo '<pw>' | sudo -S fdisk -l /dev/mmcblk<N> | grep -E '^/dev|Units'"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S sh -c 'for d in /dev/mmcblk[0-9] /dev/mmcblk[0-9]boot[01]; do [ -b \$d ] || continue; n=\$(basename \$d); dd if=\$d of=/tmp/gmsl_env_\$n.bin bs=1024 skip=\$((<ENV_OFFSET> / 1024)) count=\$((<ENV_SIZE> / 1024)) 2>/dev/null; done; chmod 644 /tmp/gmsl_env_*.bin'"
mkdir -p /tmp/gmsl_env && scp -o ControlPath=/tmp/gmsl_ssh_sock '<user>@<ip>:/tmp/gmsl_env_*.bin' /tmp/gmsl_env/
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "rm -f /tmp/gmsl_env_*.bin"
```
(boot partitions are read-only block devices, `dd` reading them is fine; repeat with
`<ENV_OFFSET_REDUND>` if set. `bs=1024` needs offset and size in whole KiB, as usual; otherwise
use `bs=1 skip=$((<ENV_OFFSET>)) count=$((<ENV_SIZE>))`.) Then check each copy on the host: the stored little-endian CRC32
(first 4 bytes) must equal the CRC32 of the rest (a redundant environment has one more flag byte
after the CRC: the data then starts at byte 5). Write the check script (it only reads):
```bash
cat > /tmp/gmsl_envcrc.py <<'EOF'
import sys, struct, zlib
redund = '-r' in sys.argv
for f in [a for a in sys.argv[1:] if a != '-r']:
    b = open(f, 'rb').read()
    data = b[5:] if redund else b[4:]
    ok = struct.unpack('<I', b[:4])[0] == zlib.crc32(data) & 0xffffffff
    fdt = [v.decode(errors='replace') for v in data.split(b'\0') if v.startswith((b'fdtfile=', b'fdt_file='))]
    print(f, 'CRC OK' if ok else 'bad CRC', fdt if ok else '')
EOF
python3 -I /tmp/gmsl_envcrc.py /tmp/gmsl_env/*.bin        # add -r for a redundant environment
```
Exactly one valid copy must contain the `fdtfile` the board boots; that is `<device>` and
`<offset>`. **No valid copy** (U-Boot prints "bad CRC, using default environment"): the
environment was never saved; the user must run `saveenv` once on the console (§Console fallback)
before this method can work — or write a fresh one offline (§Offline env image). More than one
valid copy with different contents: ask the user, or use the console.

## Build envtools

```bash
cd <work_dir>/uboot-<tag> && make <board>_defconfig && make CROSS_COMPILE=<prefix> envtools
```
(host cross-compiler, `<prefix>` e.g. `aarch64-linux-gnu-`, or the toolchain of common/9
§Compiler). The result is linked against the toolchain's C library: check that the board's glibc
is at least the newest version it needs:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "ldd --version | head -1"
<prefix>objdump -T tools/env/fw_printenv | grep -o 'GLIBC_[0-9.]*' | sort -V | tail -1
```
If the board's glibc is older, build with an older toolchain or `LDFLAGS=-static`
(`make CROSS_COMPILE=<prefix> LDFLAGS=-static envtools`).

Copy `tools/env/fw_printenv` to the board into a working folder, `~/Workspace/uboot-env` (not
`/usr/bin`), add `fw_setenv` as a symlink to it, and write a config there: `<device> <offset>
<size>` (plus a second line for a redundant copy). Use `-c <config>` on every call instead of
creating `/etc/fw_env.config`:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "mkdir -p ~/Workspace/uboot-env"
scp -o ControlPath=/tmp/gmsl_ssh_sock <work_dir>/uboot-<tag>/tools/env/fw_printenv <user>@<ip>:Workspace/uboot-env/
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace/uboot-env && ln -sf fw_printenv fw_setenv && printf '%s %s %s\n' <device> <offset> <size> > fw_env.config && cat fw_env.config"
```
(e.g. `/dev/mmcblk0 0x700000 0x4000`.)

## Verify

Read-only:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace/uboot-env && echo '<pw>' | sudo -S ./fw_printenv -c fw_env.config fdtfile"
```
It must print the value found in §CRC search. Also save the whole environment on the host
(`./fw_printenv -c fw_env.config > ...`, as `/tmp/gmsl_env/printenv_before.txt`) for the round
trip below, and check `bootcmd` boots by `fdtfile` (the platform file says what to look for, e.g.
nxp/3: `bootflow scan -l` and the distro/`mmcboot` scripts).

## Backup and write

Before the first write, back up the raw environment (`dd` of the same bytes) on the board and on
the host:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S dd if=<device> of=\$HOME/Workspace/uboot-env/env_backup.bin bs=1024 skip=\$((<offset> / 1024)) count=\$((<size> / 1024)) && echo '<pw>' | sudo -S chown \$(id -u):\$(id -g) ~/Workspace/uboot-env/env_backup.bin"
scp -o ControlPath=/tmp/gmsl_ssh_sock <user>@<ip>:Workspace/uboot-env/env_backup.bin <work_dir>/env_backup_<hostname>.bin
```
Optionally test a round trip with a harmless variable (set, read back, delete, compare the full
`fw_printenv` output with the one before):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace/uboot-env && echo '<pw>' | sudo -S sh -c './fw_setenv -c fw_env.config gmsl_test 1 && ./fw_printenv -c fw_env.config gmsl_test && ./fw_setenv -c fw_env.config gmsl_test && ./fw_printenv -c fw_env.config'" > /tmp/gmsl_env/printenv_after.txt
diff <(grep -v '^gmsl_test=' /tmp/gmsl_env/printenv_after.txt) /tmp/gmsl_env/printenv_before.txt
```
Then, after the user agreed, note the old value and write the GMSL DTB name:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd ~/Workspace/uboot-env && echo '<pw>' | sudo -S sh -c './fw_setenv -c fw_env.config fdtfile <gmsl_dtb name> && ./fw_printenv -c fw_env.config fdtfile'"
```
A reboot (common/5 §Reboot) then shows that U-Boot accepts it: the board boots the `fdtfile`
that was written (check `/sys/firmware/fdt` contains the deserializer node, or the probe check).

**Recovery:** `dd` the backup back (`sudo dd if=env_backup.bin of=<device> bs=1024 seek=$((<offset>
/ 1024)) conv=notrunc`), or `fw_setenv -c fw_env.config fdtfile <old value>`, or on the U-Boot
console `setenv fdtfile <old value>; saveenv; boot`, or `env default -a; saveenv`. A corrupted
environment makes U-Boot fall back to its built-in defaults, which boot the original DTB.

On an NXP FRDM-IMX95-PRO this found the environment in the eMMC user area at `0x700000`
(16 KiB), and `fw_setenv` then switched DTBs from Linux, with no serial console needed (as root:
`cd ~/Workspace/uboot-env && ./fw_setenv -c fw_env.config fdtfile imx95-19x19-frdm-pro-gmsl.dtb`). The
tools and the backup stay in `~/Workspace/uboot-env` (the board is left deployed).

## Offline env image

For a fresh medium (an image prepared on the host, e.g. nxp/0–nxp/4): the medium has no saved
environment yet, so U-Boot would use its built-in default and boot the base DTB. Write a complete
environment with `fdtfile` set to the GMSL DTB at `ENV_OFFSET` of the image (or device), so the
first boot uses it without a console.

1. Version and source as in §Version (offline: `strings` of the imx-boot / flash binary), and the
   location as in §Environment location (`ENV_OFFSET`, `ENV_SIZE`, redundant or not).
2. The default environment text, from the vendor U-Boot tree:
   ```bash
   cd <work_dir>/uboot-<tag> && make <board>_defconfig && make CROSS_COMPILE=<prefix> u-boot-initial-env
   cp u-boot-initial-env /tmp/gmsl_env.txt
   grep -nE '^(fdtfile|fdt_file|bootcmd)=' /tmp/gmsl_env.txt
   ```
   (one `name=value` per line, the environment U-Boot builds in.) Board code can set some
   variables at runtime (e.g. `fdtfile` chosen from the board revision); if `fdtfile` is not in
   the text, add it.
3. Change `fdtfile=` to the GMSL DTB (`<gmsl_dtb name>`, common/4 §Merged DTB, already copied to
   the boot partition of the image):
   ```bash
   sed -i 's/^fdtfile=.*/fdtfile=<gmsl_dtb name>/' /tmp/gmsl_env.txt
   grep -q '^fdtfile=' /tmp/gmsl_env.txt || echo 'fdtfile=<gmsl_dtb name>' >> /tmp/gmsl_env.txt
   ```
4. Build the binary environment with `mkenvimage`. Prefer the host's `u-boot-tools` package
   (`command -v mkenvimage`; `apt-get install -y u-boot-tools` with the host sudo of common/0):
   the format does not depend on the U-Boot version. Building it in the vendor tree
   (`make CROSS_COMPILE=<prefix> tools-only`) also builds tools such as `mkeficapsule`, which need
   extra host headers (GnuTLS) and failed in NXP's tree; use that only when the package is not
   available, then `<work_dir>/uboot-<tag>/tools/mkenvimage`.
   ```bash
   mkenvimage -s <ENV_SIZE> -o /tmp/gmsl_env.bin /tmp/gmsl_env.txt
   ```
   Add `-r` for a redundant environment (`CONFIG_ENV_OFFSET_REDUND` set), and write the same file
   at both offsets.
5. Check the first partition starts after `ENV_OFFSET` + `ENV_SIZE` (the environment must not
   overlap a partition):
   ```bash
   fdisk -l <image or /dev/sdX>      # start sector * 512 must be > ENV_OFFSET + ENV_SIZE
   ```
6. Write it (image file: no sudo needed when it is the user's file; a device: with sudo, after the
   confirmation of common/11 §Confirm):
   ```bash
   dd if=/tmp/gmsl_env.bin of=<image or device> bs=1 seek=$((<ENV_OFFSET>)) conv=notrunc
   ```
   (or faster, `bs=512 seek=$((<ENV_OFFSET> / 512))` when the offset is a multiple of 512).
   `conv=notrunc` keeps an image file at its size.
7. Verify by reading the bytes back and checking the CRC with the same script as §CRC search:
   ```bash
   dd if=<image or device> of=/tmp/gmsl_env/readback.bin bs=512 skip=$((<ENV_OFFSET> / 512)) count=$((<ENV_SIZE> / 512))
   python3 -I /tmp/gmsl_envcrc.py /tmp/gmsl_env/readback.bin      # -r for redundant
   ```
   It must print `CRC OK` and `fdtfile=<gmsl_dtb name>`.

The platform file says where this happens in its flow (nxp/3 §Image: written into the wic before
flashing; the eMMC path checks that it is still there, nxp/4). Recovery is the same as for a
live write: on the console `env default -a; saveenv` boots the base DTB again.

## Console fallback

Done by the user, if U-Boot loads the DTB as a file from a writable boot partition (a FAT
partition often mounted elsewhere than `/boot`, for example `/run/media/boot-mmcblk0p1`; the
DTB's size in the U-Boot log, `<n> bytes read`, identifies the file) but `fw_setenv` cannot be
used (method above not possible: no matching U-Boot source, or no valid saved environment yet),
and the user has the serial console:
- Find the base DTB as in common/4 §Merged DTB (on boards that boot it unchanged, it is
  byte-identical to `/sys/firmware/fdt` except for bootloader fixups such as `memory`, `chosen`,
  MAC addresses). Merge on the host into `<gmsl_dtb>` and copy it next to `<base_dtb>` (never
  modify the base).
- The user stops autoboot and selects it for one boot: `setenv fdtfile <gmsl_dtb name>` then
  `boot`. Tell them that the boot delay may be 0 s (`Hit any key to stop autoboot: 0`): hold Enter
  on the serial terminal while the board resets. Without `saveenv` the next power cycle boots the
  original DTB, which is the recovery.
- After the drivers probe, offer to make it the default (`saveenv` after the same `setenv`), so
  later reboots can be done over SSH; recovery: `setenv fdtfile <base_dtb name>; saveenv`, or
  `env default -a; saveenv`. A "bad CRC, using default environment" line in the U-Boot log means
  the environment was never saved: `saveenv` then stores the defaults plus this one change (after
  which §CRC search finds it, and `fw_setenv` can be used for later changes).
- Each later DTB change (for example new alias pools) only needs the file copied again and a
  reboot.

Wait for the user to confirm each console step; then continue with common/5 (reconnect, probe
check).
