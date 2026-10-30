# NXP 0: Base image (unpack, mount, read, discovery settings)

First step of the NXP i.MX build path (orchestrator §Flow, "No" answer, NXP i.MX): take the NXP
i.MX Linux BSP release the user downloaded, unpack a working copy into `<work_dir>`, mount it,
read what the later steps need (kernel release and config, U-Boot version, DTBs, partition
table, glibc), and set the hostname and root password that the orchestrator uses to find the
board after the first boot.

Run after common/0 (host tools, sudo password) and the flash-medium question (`flash_medium` in
`<state>`). The camera questions of common/3 are asked before or after this file (orchestrator
§Flow): the device tree is built offline from the image's DTBs. All local sudo commands follow
common/0 §Sudo password (`SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c '<cmd>'`). `<tpl_dir>` =
`/tmp/gen_gmsl_dts/nxp`.

Tested: FRDM-IMX95-PRO with NXP i.MX Linux 6.18.20-2.0.0 (i.MX Distro 6.18-wrynose, Yocto
`MACHINE` = `imx95-19x19-lpddr5-frdm-pro`). Other i.MX boards and releases: the same steps, with
the names found in the release (nxp/2 §Other i.MX boards).

## Image file

Only for a board that is **not** on the network (the orchestrator's No path). A board that is
already running an NXP BSP is never reflashed: it gets the drivers built for its running kernel
(nxp/1 §Live board), and this file is not used.

NXP's prebuilt images need an NXP login and a license agreement, so the user downloads the file
in the browser; the skill finds the exact link. Always use the **latest** release:
1. Read NXP's public release page with WebFetch:
   `https://www.nxp.com/design/design-center/software/embedded-software/i-mx-software/embedded-linux-for-i-mx-applications-processors:IMXLINUX`
   (prompt: list the releases marked current/newest and, for the board's SoC (e.g. i.MX 95 for
   the FRDM-IMX95-PRO), the prebuilt-image download link texts and URLs with their `colCode`).
   The page is long: if the answer says it was truncated, the latest releases are at the start,
   so the first part is enough.
2. Take the **newest** release that lists the board's SoC (the one marked "Current"; its row
   names the boards, e.g. "i.MX 95 EVK, FRDM"). The link has the form
   `https://www.nxp.com/webapp/Download?colCode=<colCode>&appType=license`, for example
   `colCode=LF_v6.18.37-2.1.0_MX95` (Linux 6.18.37_2.1.0, the current i.MX 95 release in
   October 2026; 6.18.20_2.0.0 was the one tested on the FRDM-IMX95-PRO). Give the user the full
   URL, the release name and the board list of that row, **written out in the chat** (not only
   "the link above"), for example:
   `Latest NXP release for i.MX 95: Linux 6.18.37_2.1.0 (boards: i.MX 95 EVK, FRDM).`
   `Download: https://www.nxp.com/webapp/Download?colCode=LF_v6.18.37-2.1.0_MX95&appType=license`
   Repeat the full URL in the "NXP image" question and in any message where Claude waits for the
   download. If the page cannot be read or has no
   row for the SoC, tell the user to open that page and pick the newest release for their SoC.
3. Tell the user: the link opens NXP's login (an NXP account is free) and the license agreement;
   after accepting, the download starts (a `.zip` of several GB with the prebuilt images). Then
   they give Claude the path of the downloaded file. A newer release than the tested one is
   fine: nxp/0 §Read the image and nxp/1 derive the kernel tag, `LOCALVERSION`, U-Boot tag and
   compiler from the image itself. A `.wic`, `.wic.zst` or `.wic.bz2` extracted from
   such a zip is accepted too (then the `imx-boot` binary for eMMC flashing must be next to it, or
   given as a second path).

If `nxp_image_path` is already in `<state>` (orchestrator §Flow, Q4), skip the question. Otherwise
ask for the path with AskUserQuestion (header: "NXP image"; the question text contains the full
download URL of step 2): the user types the path in "Other"
(a Windows path on WSL, `C:\Users\...`, is converted with `wslpath -u '<path>'`). Check that the
file exists and is readable (`ls -l '<path>'`), and record it (`nxp_board` normalized as in the
orchestrator's §State: `FRDM-IMX95-PRO`, `IMX93-11X11-EVK`, or `other-imx` with the named board in `nxp_model`;
keep the values the orchestrator's Q2/Q3 already stored):
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['nxp_image_path'] = '<path>'
state['nxp_board'] = '<FRDM-IMX95-PRO|IMX93-11X11-EVK|other-imx>'
state['nxp_model'] = '<the board the user named, other-imx only>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

**Never change the user's file**: every later step works on a copy in `<work_dir>/nxp-image/`.

## Unpack

Space: the zip is several GB, a decompressed `.wic` 10–16 GB (check `df -h <work_dir>` and tell
the user if less than about 25 GB are free).

```bash
mkdir -p <work_dir>/nxp-image
```

**Zip:** list it first, without extracting everything:
```bash
unzip -l '<nxp_image_path>' | grep -iE '\.wic(\.zst|\.bz2)?$|imx-boot|flash'
```
Pick the files for the board, by its Yocto machine name (FRDM-IMX95-PRO:
`imx95-19x19-lpddr5-frdm-pro`; i.MX93 11x11 EVK with the PCA9451A PMIC:
`imx93-11x11-lpddr4x-evk`, image `imx-image-full-imx93evk.wic`; the running board's hostname is the machine name on NXP images):
- the root filesystem image: `*<machine>*.wic.zst` (or `.wic.bz2`, `.wic`); prefer the full
  image with the desktop (`imx-image-full-*`, Weston is needed for the viewer) over
  `core-image-minimal`/`multimedia` if several are listed;
- the bootloader for eMMC flashing with uuu: `imx-boot*<machine>*flash*` (for example
  `imx-boot-<machine>-sd.bin-flash_all`; on i.MX95 the name can also contain the SoC revision).

If no file matches the board, or more than one could be it, show the candidates and ask the user.
Extract only those files:
```bash
unzip -o -j '<nxp_image_path>' '<wic member>' '<imx-boot member>' -d <work_dir>/nxp-image/
```

**Decompress** into the work folder (the source stays as it is):
```bash
cd <work_dir>/nxp-image
zstd -dc '<file>.wic.zst' > '<name>.wic'      # .wic.zst
bzip2 -dc '<file>.wic.bz2' > '<name>.wic'     # .wic.bz2
cp '<nxp_image_path>' '<name>.wic'            # an uncompressed .wic given directly
```
Remove the compressed copy that `unzip` extracted into `<work_dir>/nxp-image/` once the `.wic`
is complete (not the user's file). If the user gave a `.wic*` directly, copy the `imx-boot` binary
they name into `<work_dir>/nxp-image/` too.

Keep `<wic>` = `<work_dir>/nxp-image/<name>.wic` and `<imx_boot>` =
`<work_dir>/nxp-image/<imx-boot file>` for the later files.

**Board bootloader in the `.wic`.** An NXP release can ship one `.wic` for several boards of a
SoC, with another board's bootloader inside. Release 6.18.37-2.1.0 for the i.MX 95 has only
`imx-image-full-imx95evk.wic`, whose bootloader is the 19x19 EVK's; the FRDM-IMX95-PRO's comes
only as `imx-boot-imx95-19x19-lpddr5-frdm-pro-sd.bin-flash_all`. Booting from an SD card uses the
bootloader inside the `.wic`; flashing the eMMC with `uuu ... emmc_all` writes `<imx_boot>`
separately, so only the SD card fails, but fix the copy on both paths. When no `.wic` is named
after the board's machine, compare the two bootloaders' built-in `fdtfile`:
```bash
head -c 8M '<wic>' | strings | grep -m1 -E '^fdtfile='
strings '<imx_boot>' | grep -m1 -E '^fdtfile='
```
If they differ (the `.wic` names another board's DTB), tell the user and ask (header:
"Bootloader"): **Write the board's bootloader into the copy (Recommended)** | **Stop**. Then clear
the old bootloader area of the copy (the other board's can be longer: on 6.18.37-2.1.0 the EVK's
ends at `0x2f5027`, the FRDM-PRO's at `0x2ddc00`; nothing else lies between 32 KiB and the U-Boot
environment at `0x700000`) and write `<imx_boot>` at the SD boot offset, never into the user's
download:
```bash
W='<wic>'; B='<imx_boot>'; OFF=32    # KiB: i.MX 95, 93, 8M Nano, 8M Plus (i.MX 8M Quad/Mini: 33)
sfdisk -d "$W" > /tmp/gmsl_pt_before.txt
dd if=/dev/zero of="$W" bs=1k seek=$OFF count=$(( 0x700000 / 1024 - OFF )) conv=notrunc status=none
dd if="$B" of="$W" bs=1k seek=$OFF conv=notrunc status=none
cmp -n $(stat -c %s "$B") -i $(( OFF * 1024 )):0 "$W" "$B" && echo "bootloader written"
sfdisk -d "$W" | diff /tmp/gmsl_pt_before.txt - && echo "partition table unchanged"; rm -f /tmp/gmsl_pt_before.txt
```
Both checks must print their line. Then the `.wic`'s built-in `fdtfile` is the board's (checked
again in §Read the image step 3). Tested with 6.18.37-2.1.0 on the FRDM-IMX95-PRO (SD boot).

## Mount

```bash
mkdir -p /tmp/gmsl_nxp_boot /tmp/gmsl_nxp_rootfs
LOOP=$(SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A losetup -fP --show '<wic>')
echo "Loop device: $LOOP"
lsblk "$LOOP"
```
Mount the boot partition (first partition, FAT) and the root filesystem (second, ext4).
Specify the filesystem types explicitly — without `-t`, WSL2 may attempt an NFS mount and fail:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mount -t vfat "${LOOP}p1" /tmp/gmsl_nxp_boot
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mount -t ext4 "${LOOP}p2" /tmp/gmsl_nxp_rootfs
ls /tmp/gmsl_nxp_boot/*.dtb /tmp/gmsl_nxp_boot/Image* 2>/dev/null | head
ls /tmp/gmsl_nxp_rootfs/etc/os-release && cat /tmp/gmsl_nxp_rootfs/etc/os-release
```
If the partition layout differs (more partitions, the kernel in the root filesystem's `/boot`),
show `lsblk` and the listings to the user and adapt the paths; stop if no DTB and no kernel are
found.

The image stays mounted through nxp/1 §Image (modules) and nxp/2 §Merged DTB (DTB); nxp/3 §Image
runs §Unmount before it writes the U-Boot environment into the `.wic`. Shell variables do not
survive between commands: find the loop device again with `losetup -j '<wic>'`.

## Read the image

Save what is read in `<tpl_dir>` (create it with `mkdir -p <tpl_dir>`) and in `<state>`.

1. **Kernel release** (the folder name in `lib/modules`):
   ```bash
   ls /tmp/gmsl_nxp_rootfs/lib/modules/
   ```
   e.g. `6.18.20-2.0.0-gb096ce610e95`. Store it as `kernel_release`. The hash after `-g` is the
   kernel commit; find NXP's tag at that commit:
   ```bash
   git ls-remote https://github.com/nxp-imx/linux-imx 'refs/tags/lf-*' | grep -i '<hash>'
   ```
   (annotated tags show the commit on the `^{}` line). `6.18.20-2.0.0-gb096ce610e95` =
   `lf-6.18.20-2.0.0`.
2. **Kernel config** (common/9 needs it as `<tpl_dir>/board.config`, like `/proc/config.gz` on a
   live board): with `CONFIG_IKCONFIG` it is inside the kernel image or in `configs.ko`. Use the
   compressed config between the markers `IKCFG_ST` and `IKCFG_ED` (what the kernel's
   `scripts/extract-ikconfig` reads); extract it directly, with no download:
   ```bash
   python3 -I - /tmp/gmsl_nxp_boot/Image > <tpl_dir>/board.config <<'EOF'
   import gzip, sys
   d = open(sys.argv[1], 'rb').read()
   i = d.find(b'IKCFG_ST'); j = d.find(b'IKCFG_ED', i)
   if i < 0 or j < 0:
       sys.exit('no IKCFG markers (CONFIG_IKCONFIG off, or a compressed file)')
   sys.stdout.write(gzip.decompress(d[i + 8:j]).decode())
   EOF
   grep -E '^CONFIG_(CC_VERSION_TEXT|MODVERSIONS|MODULE_SIG_FORCE|LOCALVERSION|I2C_ATR|VIDEO_IMX219)=' <tpl_dir>/board.config
   ```
   If the `Image` has no markers, run the same script on `configs.ko` from
   `/tmp/gmsl_nxp_rootfs/lib/modules/<kernel_release>/kernel/kernel/configs.ko*` (checked on
   6.18.37-2.1.0: identical to the kernel tree's `.config`).
   (a compressed `configs.ko.zst`/`.xz` must be decompressed into `/tmp/gmsl_configs.ko` first;
   delete it afterwards). If neither has the config, tell the user that the modules cannot be
   built for this image without the release's kernel config, and stop the build path.
   `CONFIG_CC_VERSION_TEXT` gives the compiler (6.18.20-2.0.0: GCC 15.2.0).
3. **U-Boot version** (common/10 §Version needs the commit):
   ```bash
   strings '<imx_boot>' | grep -m3 -E 'U-Boot 20[0-9.]+'
   head -c 8M '<wic>' | strings | grep -m3 -E 'U-Boot 20[0-9.]+'
   ```
   e.g. `U-Boot 2026.04-lf_v2026.04+g6eeef838dac` (tag `lf-6.18.20-2.0.0` of
   https://github.com/nxp-imx/uboot-imx). The two must agree, and so must their built-in board:
   the `fdtfile=` of `head -c 8M '<wic>' | strings` and of `strings '<imx_boot>'` must both be the
   board's base DTB (a release can carry the same U-Boot version for several boards; otherwise
   go back to §Unpack, "Board bootloader in the `.wic`"). Also note the default `fdtfile=`
   and `bootcmd=` of the built-in environment (`strings '<imx_boot>' | grep -E '^(fdtfile|bootcmd|bootdelay)='`).
4. **DTB list** on the boot partition:
   ```bash
   ls -l /tmp/gmsl_nxp_boot/*.dtb
   mkdir -p <tpl_dir>/dtbs && cp /tmp/gmsl_nxp_boot/*.dtb <tpl_dir>/dtbs/
   ```
   The base DTB is the built-in `fdtfile` (FRDM-IMX95-PRO: `imx95-19x19-frdm-pro.dtb`); the camera
   DTBs (`*os08a20*`, `*cam*`, `*<sensor>*`) are the wiring reference for nxp/2. Decompile the base
   for common/4: `dtc -q -I dtb -O dts -o <tpl_dir>/base.dts <tpl_dir>/dtbs/<base_dtb>`.
   Also check that nothing on the boot partition makes U-Boot's `bootflow scan` boot something
   else than `fdtfile` (nxp/3 §Values): `ls /tmp/gmsl_nxp_boot/extlinux /tmp/gmsl_nxp_boot/boot.scr /tmp/gmsl_nxp_boot/EFI 2>/dev/null`.
5. **Partition table:** the U-Boot environment lives before the first partition (FRDM-IMX95-PRO:
   offset `0x700000`, 16 KiB, nxp/3 §Values), so the first partition must start after
   `0x700000 + 0x4000` = 7356416 bytes:
   ```bash
   sfdisk -d '<wic>'
   ```
   (`start=` is in 512-byte sectors: the first partition's start × 512 must be ≥ 7356416). If it
   is not, the environment would overwrite the partition: stop the offline environment (nxp/3
   §Image) and tell the user.
6. **glibc version** of the image (common/10 §Build envtools checks the built `fw_printenv`
   against it):
   ```bash
   ls -l /tmp/gmsl_nxp_rootfs/lib/libc.so.6 /tmp/gmsl_nxp_rootfs/usr/lib/libc.so.6 2>/dev/null
   strings "$(readlink -f /tmp/gmsl_nxp_rootfs/usr/lib/libc.so.6 2>/dev/null || readlink -f /tmp/gmsl_nxp_rootfs/lib/libc.so.6)" | grep -m1 -E 'GNU C Library .* version [0-9.]+'
   ```
   (`readlink -f` resolves inside the host, so if the link is absolute, resolve it under
   `/tmp/gmsl_nxp_rootfs` by hand.)

Write the values to `<state>` (`kernel_release`, and `localversion` once nxp/1 finds it) and show
the user a short summary: release, kernel tag, compiler, U-Boot version and tag, base DTB, camera
DTBs, first partition start, glibc.

## Discovery settings

Make the board easy to find and log in to after the first boot (common/1 tries this hostname and
the login `root` / `analog`).

1. **Hostname** `analog-imx-gmsl`:
   ```bash
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'echo analog-imx-gmsl > /tmp/gmsl_nxp_rootfs/etc/hostname'
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'grep -q "^127\.0\.1\.1" /tmp/gmsl_nxp_rootfs/etc/hosts && sed -i "s/^127\.0\.1\.1.*/127.0.1.1\tanalog-imx-gmsl/" /tmp/gmsl_nxp_rootfs/etc/hosts || printf "127.0.1.1\tanalog-imx-gmsl\n" >> /tmp/gmsl_nxp_rootfs/etc/hosts'
   cat /tmp/gmsl_nxp_rootfs/etc/hostname
   ```
   mDNS needs avahi on the board: check `ls -l /tmp/gmsl_nxp_rootfs/usr/sbin/avahi-daemon` (use `ls -l`,
   not `test -x`, for files in the mounted image: many are links to absolute paths such as
   `/usr/bin/...`, which resolve on the host, not in the image). Without
   it, common/1 finds the board by its MAC address (NXP OUI `00:04:9F`) instead; tell the user.
2. **Root password** `analog` (NXP images log in as `root`, often with an empty password; that may
   work on the console, but SSH and `sshpass` usually refuse it, unverified for every release, see
   common/1 §Login). Replace only the password field of root's line:
   ```bash
   HASH=$(echo 'analog' | openssl passwd -6 -stdin)
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i -E "s#^root:[^:]*:#root:${HASH}:#" /tmp/gmsl_nxp_rootfs/etc/shadow
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A grep -c '^root:\$6\$' /tmp/gmsl_nxp_rootfs/etc/shadow
   ```
   The last command must print `1`. (`#` as the sed separator: the hash contains `/` but never
   `#`.) If the image has no `/etc/shadow` (password in `/etc/passwd`), tell the user and stop:
   do not guess another format.
3. **SSH root login with a password**: find the SSH server of the image:
   ```bash
   ls /tmp/gmsl_nxp_rootfs/usr/sbin/sshd /tmp/gmsl_nxp_rootfs/usr/sbin/dropbear 2>/dev/null
   SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A grep -nE '^[[:space:]]*(PermitRootLogin|PasswordAuthentication|Include)' /tmp/gmsl_nxp_rootfs/etc/ssh/sshd_config 2>/dev/null
   cat /tmp/gmsl_nxp_rootfs/etc/default/dropbear 2>/dev/null
   ```
   - OpenSSH: root needs `PermitRootLogin yes` (the default `prohibit-password` refuses passwords)
     and `PasswordAuthentication` not `no`. If `sshd_config` has an `Include` of
     `/etc/ssh/sshd_config.d/*.conf`, write a drop-in through a temporary file (a heredoc piped
     into `sudo -S tee` ends up empty, because the password pipe consumes stdin):
     ```bash
     printf 'PermitRootLogin yes\nPasswordAuthentication yes\n' > /tmp/gmsl_sshd.conf
     SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'mkdir -p /tmp/gmsl_nxp_rootfs/etc/ssh/sshd_config.d && cp /tmp/gmsl_sshd.conf /tmp/gmsl_nxp_rootfs/etc/ssh/sshd_config.d/10-gmsl.conf'
     rm -f /tmp/gmsl_sshd.conf
     ```
     Otherwise change the existing lines in `sshd_config` with `sudo sed -i` (or append them if
     absent); the first value of a keyword wins in OpenSSH, so remove or change earlier lines.
     Yocto images can also have `sshd_config_readonly`: change it the same way.
   - Dropbear: `DROPBEAR_EXTRA_ARGS` in `/etc/default/dropbear` must not contain `-w` (no root
     login), `-s` (no password logins) or `-g` (no password logins for root); remove those
     flags.
   - Neither present: tell the user the image has no SSH server and stop (the Yes path needs SSH).
4. Record the expected hostname and login:
   ```bash
   python3 -c "
   import json
   state = json.load(open('/tmp/gmsl_camera_state.json'))
   state['expected_hostname'] = 'analog-imx-gmsl'
   state['board_user'] = 'root'
   state['board_pw'] = 'analog'
   json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
   "
   ```
   Root's home on NXP images is `/root` (checked on the FRDM-IMX95-PRO with `getent passwd root`): the later steps use `~` / `$HOME`
   (state `board_home`, read with `echo $HOME` after the first login), never a fixed path.

## Unmount

Run this when nxp/3 §Image asks for it (after nxp/1 §Image and nxp/2 §Merged DTB), and in the
orchestrator's §Cleanup:
```bash
sync
for m in /tmp/gmsl_nxp_boot /tmp/gmsl_nxp_rootfs; do
  mountpoint -q "$m" && SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A umount "$m"
done
for l in $(losetup -nO NAME -j '<wic>'); do SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A losetup -d "$l"; done
rmdir /tmp/gmsl_nxp_boot /tmp/gmsl_nxp_rootfs 2>/dev/null
losetup -j '<wic>'
```
The last command must print nothing. If `umount` reports `target is busy`, a shell or file
manager is inside the mount: `cd` out of it and retry.
