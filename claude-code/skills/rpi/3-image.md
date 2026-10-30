# RPi 3: Prepare the Raspberry Pi OS image and write the SD card

## Purpose
Download the Raspberry Pi OS Desktop (64-bit) image, inject the custom-built kernel
(image, modules, DTBs, overlays), prepare first-boot configuration (user credentials,
SSH, skip setup wizard, hostname), deploy the video pipeline tools, and write the modified
image to the SD card with common/11.

## Prerequisites
Run after rpi/2. Read from `<state>` (`/tmp/gmsl_camera_state.json`): `work_dir`,
`kernel_src_path` (`<work_dir>/rpi-linux`), `rpi_model`, `sudo_pw_file`, and the SD card keys
written by common/11 §Detect (`sd_type`, `sd_device`, `sd_busid`, `sd_disk_number`).

All local sudo commands follow common/0 §Sudo password: `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c '<cmd>'`
(plain `sudo <cmd>` when the password is empty, passwordless sudo).

## Step 1: Download Raspberry Pi OS Desktop (64-bit)

### 1a. Get the latest image URL

Fetch the download page and extract the latest image URL:
```bash
LATEST_DIR=$(curl -sL https://downloads.raspberrypi.com/raspios_arm64/images/ \
  | grep -oP 'raspios_arm64-\d{4}-\d{2}-\d{2}/' | tail -1)
IMG_FILE=$(curl -sL "https://downloads.raspberrypi.com/raspios_arm64/images/${LATEST_DIR}" \
  | grep -oP '[^"]+\.img\.xz(?=")' | head -1)
IMG_URL="https://downloads.raspberrypi.com/raspios_arm64/images/${LATEST_DIR}${IMG_FILE}"
echo "Image URL: $IMG_URL"
```

### 1b. Download and decompress

The image goes into `<work_dir>` (about 3 GB compressed, 6–7 GB decompressed; check the free space
with `df -h <work_dir>` and tell the user if it is not enough):
```bash
mkdir -p <work_dir>
cd <work_dir>
if [ ! -f "<work_dir>/${IMG_FILE}" ]; then
  curl -L -o "<work_dir>/${IMG_FILE}" "$IMG_URL"
fi
```

Decompress (`xz -d` removes the .xz once the image is extracted):
```bash
IMG_NAME="${IMG_FILE%.xz}"
if [ ! -f "<work_dir>/${IMG_NAME}" ]; then
  xz -d "<work_dir>/${IMG_FILE}"
fi
echo "Image file: <work_dir>/${IMG_NAME}"
```

Shell variables do not survive between commands: keep `IMG_FILE`, `IMG_NAME` (and `LOOP` from
Step 2) in mind and write their values into later commands.

## Step 2: Mount image via loop device

```bash
mkdir -p /tmp/gmsl_rpi_boot /tmp/gmsl_rpi_rootfs
LOOP=$(SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A losetup -fP --show "<work_dir>/${IMG_NAME}")
echo "Loop device: $LOOP"
```

Mount boot partition (first partition) and rootfs (second partition).
**Important**: Specify filesystem types explicitly — without `-t`, WSL2 may attempt
NFS mount and fail.
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mount -t vfat "${LOOP}p1" /tmp/gmsl_rpi_boot
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mount -t ext4 "${LOOP}p2" /tmp/gmsl_rpi_rootfs
```

Verify mounts:
```bash
ls /tmp/gmsl_rpi_boot/kernel*.img 2>/dev/null && echo "Boot partition mounted ok"
ls /tmp/gmsl_rpi_rootfs/etc/os-release 2>/dev/null && echo "Rootfs partition mounted ok"
```

## Step 3: Install custom kernel into the image

### 3a. Copy kernel image

The kernel image filename depends on the RPi model:
- RPi 5: `kernel_2712.img`
- RPi 4: `kernel8.img`

```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp <kernel_src_path>/arch/arm64/boot/Image /tmp/gmsl_rpi_boot/<kernel_filename>
```
Leave `config.txt`'s `auto_initramfs=1` and the image's `initramfs_2712`/`initramfs8` as they are:
Raspberry Pi OS Trixie grows the root partition to the card's size from the initramfs on the first
boot (its `resize_early` script; checked on a Pi 5, October 2026). The initramfs was built for the
image's own kernel (6.18.50 there): it still works with the kernel built here because the SD card,
ext4 and the GMSL drivers are built in (`=y`); only modules it would load from its own tree are
missing, which the first boot does not need.

### 3b. Install kernel modules

Even though GMSL drivers are built-in (`=y`), other kernel modules (USB, networking,
filesystems, etc.) still need to be installed:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'make -C <kernel_src_path> ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=/tmp/gmsl_rpi_rootfs modules_install'
```

### 3c. Copy device tree blobs

```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp <kernel_src_path>/arch/arm64/boot/dts/broadcom/*.dtb /tmp/gmsl_rpi_boot/
```

### 3d. Copy device tree overlays

```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp <kernel_src_path>/arch/arm64/boot/dts/overlays/*.dtbo /tmp/gmsl_rpi_boot/overlays/
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp <kernel_src_path>/arch/arm64/boot/dts/overlays/README /tmp/gmsl_rpi_boot/overlays/ 2>/dev/null || true
```

## Step 4: First-boot preparation

These steps configure the OS image so the board is immediately accessible via SSH
after first boot, without requiring a monitor/keyboard for initial setup.

### 4a. Create user credentials directly in rootfs

The login is `analog` / `analog`, as on the ADI GMSL Raspberry Pi images (common/1 §Login tries it
first). The `userconf.txt` boot partition method is unreliable (user may not be created on first
boot). Instead, create the user directly in the rootfs filesystem.

**Important**: Some OS images ship with a pre-created user at UID 1000 (e.g., RPi OS
Trixie pre-creates a `pi` user with shell `/usr/sbin/nologin`). The script must detect
and replace any existing UID 1000 user rather than blindly appending, otherwise both
entries coexist and the system resolves UID 1000 to the first match, breaking
authentication and sudo.

```bash
ROOTFS=/tmp/gmsl_rpi_rootfs

# --- Remove any existing user or group with UID/GID 1000 ---
# Detect existing username(s) at UID 1000
EXISTING_USERS=$(SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A awk -F: '$3 == 1000 {print $1}' "$ROOTFS/etc/passwd" 2>/dev/null)
EXISTING_GROUPS=$(SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A awk -F: '$3 == 1000 {print $1}' "$ROOTFS/etc/group" 2>/dev/null)

# Remove existing UID 1000 entries from passwd, shadow, and group
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i '/^[^:]*:x:1000:/d' "$ROOTFS/etc/passwd"
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c "sed -i '/^[^:]*:x:1000:/d' $ROOTFS/etc/group"
for u in $EXISTING_USERS; do
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i "/^${u}:/d" "$ROOTFS/etc/shadow"
  # Remove from supplementary group memberships: as the only member (sudo:x:27:pi), the
  # first (:pi,vnc), a middle or the last one. The member is matched in full, so pi-foo stays.
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i -E "s/:${u}\$/:/; s/:${u},/:/; s/,${u}(,|\$)/\\1/" "$ROOTFS/etc/group"
done

# Also remove any stale 'analog' entries (from prior runs or partial state)
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i '/^analog:/d' "$ROOTFS/etc/passwd"
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i '/^analog:/d' "$ROOTFS/etc/shadow"
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i '/^analog:x:1000:/d' "$ROOTFS/etc/group"

# --- Create the analog user cleanly ---
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'echo "analog:x:1000:1000::/home/analog:/bin/bash" >> '"$ROOTFS"'/etc/passwd'

ENCRYPTED_PW=$(echo 'analog' | openssl passwd -6 -stdin)
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c "echo 'analog:${ENCRYPTED_PW}:19700:0:99999:7:::' >> $ROOTFS/etc/shadow"

SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'echo "analog:x:1000:" >> '"$ROOTFS"'/etc/group'

# Add to sudo and to the device groups (video: /dev/video* and /dev/media*, i2c: /dev/i2c-*,
# gpio, render, ...), otherwise cam.py and i2cdetect fail with permission errors.
# Groups that do not exist in the image are skipped; the sed handles empty member lists.
for g in sudo video adm dialout audio plugdev users input render netdev gpio i2c spi; do
  grep -q "^$g:" "$ROOTFS/etc/group" && \
    SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i -E "/^$g:/ { /[:,]analog(,|$)/! s/$/,analog/; s/:,analog$/:analog/ }" "$ROOTFS/etc/group"
done

# --- Set up home directory ---
# If an old UID 1000 user's home exists but /home/analog does not, rename it
for u in $EXISTING_USERS; do
  if [ "$u" != "analog" ] && [ -d "$ROOTFS/home/$u" ] && [ ! -d "$ROOTFS/home/analog" ]; then
    SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mv "$ROOTFS/home/$u" "$ROOTFS/home/analog"
  fi
done

SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mkdir -p "$ROOTFS/home/analog"
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp -rn "$ROOTFS/etc/skel/." "$ROOTFS/home/analog/" 2>/dev/null || true
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown -R 1000:1000 "$ROOTFS/home/analog"

# Add sudo access with password
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'echo "analog ALL=(ALL) ALL" > '"$ROOTFS"'/etc/sudoers.d/010_analog'
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chmod 440 "$ROOTFS/etc/sudoers.d/010_analog"

# Clean up old home directories from replaced users
for u in $EXISTING_USERS; do
  if [ "$u" != "analog" ] && [ -d "$ROOTFS/home/$u" ]; then
    SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A rm -rf "$ROOTFS/home/$u"
  fi
done
```

Also create `userconf.txt` on the boot partition as a fallback (some platforms consume
this to set the default user on first boot):
```bash
echo "analog:${ENCRYPTED_PW}" > /tmp/gmsl_userconf.txt
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gmsl_userconf.txt /tmp/gmsl_rpi_boot/userconf.txt
rm -f /tmp/gmsl_userconf.txt
```
(`ENCRYPTED_PW` comes from the block above; run both in the same command, or compute it again.)

### 4b. Enable SSH on first boot

Create an empty `ssh` file on the boot partition to enable the SSH server:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A touch /tmp/gmsl_rpi_boot/ssh
```

### 4c. Skip the first-boot setup wizard (piwiz)

The desktop setup wizard blocks the boot process and requires a monitor. The
`autostart-disabled.list` method is unreliable. Instead, remove the piwiz autostart
entry directly and create the completion flag:
```bash
# Remove piwiz autostart desktop file
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A rm -f /tmp/gmsl_rpi_rootfs/etc/xdg/autostart/piwiz.desktop
# Mark first-boot setup as complete
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'mkdir -p /tmp/gmsl_rpi_rootfs/home/analog/.config && touch /tmp/gmsl_rpi_rootfs/home/analog/.config/setup-done'
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown -R 1000:1000 /tmp/gmsl_rpi_rootfs/home/analog/.config
```

### 4d. Set the hostname

Use the same hostname as the ADI GMSL Raspberry Pi images, so that common/1 can find the
board on the network as `analog-rpi-gmsl.local`:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'echo analog-rpi-gmsl > /tmp/gmsl_rpi_rootfs/etc/hostname'
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'grep -q "^127\.0\.1\.1" /tmp/gmsl_rpi_rootfs/etc/hosts && sed -i "s/^127\.0\.1\.1.*/127.0.1.1\tanalog-rpi-gmsl/" /tmp/gmsl_rpi_rootfs/etc/hosts || printf "127.0.1.1\tanalog-rpi-gmsl\n" >> /tmp/gmsl_rpi_rootfs/etc/hosts'
```

Verify:
```bash
cat /tmp/gmsl_rpi_rootfs/etc/hostname
grep '^127\.0\.1\.1' /tmp/gmsl_rpi_rootfs/etc/hosts
```

Record it for the orchestrator's first-boot search:
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['expected_hostname'] = 'analog-rpi-gmsl'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

## Step 5: Deploy video pipeline tools

The GMSL camera pipeline requires two tools on the target board:

1. **pyv4l2** — Python V4L2/media-ctl library with GMSL-aware camera configurations
2. **video_cfg.sh** — Shell wrapper that invokes cam.py with the right config

**Important**: Do NOT use `curl ... | sudo tee` or `cat <<HEREDOC | SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A tee`
with a piped sudo password — the password pipe consumes stdin and the file ends up empty. Always
write or download to a temporary file `/tmp/gmsl_*` first, then copy it with sudo, then delete the
temporary file.

### 5a. Clone pyv4l2 into the image

```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mkdir -p /tmp/gmsl_rpi_rootfs/home/analog/Workspace
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A git clone --branch gmsl https://github.com/iristeiu06/pyv4l2 /tmp/gmsl_rpi_rootfs/home/analog/Workspace/pyv4l2
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown -R 1000:1000 /tmp/gmsl_rpi_rootfs/home/analog/Workspace/pyv4l2
```

### 5b. Download video_cfg.sh

```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A mkdir -p /tmp/gmsl_rpi_rootfs/home/analog/Workspace
curl -sL https://raw.githubusercontent.com/analogdevicesinc/gmsl/tools/rpi/video_cfg.sh \
  -o /tmp/gmsl_video_cfg.sh
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gmsl_video_cfg.sh /tmp/gmsl_rpi_rootfs/home/analog/Workspace/video_cfg.sh
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chmod +x /tmp/gmsl_rpi_rootfs/home/analog/Workspace/video_cfg.sh
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown 1000:1000 /tmp/gmsl_rpi_rootfs/home/analog/Workspace/video_cfg.sh
rm -f /tmp/gmsl_video_cfg.sh
```

Check that the copy is not empty (`test -s`); an empty file means the download failed or the
password pipe consumed it (see the note above).

### 5c. Fix paths in video_cfg.sh

The default `PYV4L2_PATH` and the cam.py invocation path may need fixing:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i 's|^PYV4L2_PATH=.*|PYV4L2_PATH="/home/analog/Workspace/pyv4l2"|' \
  /tmp/gmsl_rpi_rootfs/home/analog/Workspace/video_cfg.sh

# Fix cam.py path — the script may reference utils/cam.py but the actual path is utils/cam/cam.py.
# The pattern matches only /utils/cam.py, so a script that already calls utils/cam/cam.py is
# left unchanged (a plain 'utils/cam.py' pattern would also match it and give cam/cam/cam.py).
if [ -f /tmp/gmsl_rpi_rootfs/home/analog/Workspace/pyv4l2/utils/cam/cam.py ]; then
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sed -i 's#/utils/cam\.py#/utils/cam/cam.py#' \
    /tmp/gmsl_rpi_rootfs/home/analog/Workspace/video_cfg.sh
fi
```

### 5d. Add first-boot dependency install script

The video pipeline requires qv4l2 (the first viewer, common/8 §qv4l2), the GStreamer tools and
`bayer2rgb` (the fallback viewer, common/8 §GStreamer: `gstreamer1.0-tools`,
`gstreamer1.0-plugins-bad`), and pixutils and pykms for cam.py, which aren't in the base RPi OS
image. Since the image is arm64 and the host is x86, chroot won't work. Instead, add a
systemd oneshot service that installs dependencies on first boot.

The board may boot before its network cable is connected, and `network-online.target` does
not guarantee internet access. The script therefore waits for the package servers, exits
with an error if any install fails, and only removes itself after everything is installed,
so a failed attempt is retried on the next boot. rpi/4 §Tools also installs anything that is
still missing. It also waits for the clock: the image's date is older than the servers'
certificates, so `git clone` over HTTPS fails with a certificate error until NTP has set the
time (seen on the first boot of Raspberry Pi OS Trixie 2026-10-06).

```bash
cat > /tmp/gmsl_first_boot.sh <<'SCRIPT'
#!/bin/bash
set -e
# Wait up to 5 minutes for internet access
for i in $(seq 1 60); do
  getent hosts deb.debian.org >/dev/null 2>&1 && break
  sleep 5
done
# Wait up to 5 minutes for the clock to be synchronized (HTTPS certificates)
for i in $(seq 1 60); do
  [ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" = yes ] && break
  sleep 5
done
apt-get update
apt-get install -y qv4l2 gstreamer1.0-tools gstreamer1.0-plugins-bad
for i in 1 2 3; do
  pip3 install --break-system-packages git+https://github.com/tomba/pixutils.git git+https://github.com/tomba/pykms.git && break
  [ $i = 3 ] && exit 1; sleep 30
done
cd /home/analog/Workspace/pyv4l2 && pip3 install --break-system-packages -e .
python3 -c 'import v4l2, pixutils, kms'
# Only reached when every install succeeded
systemctl disable gmsl-first-boot.service
rm -f /etc/systemd/system/gmsl-first-boot.service /usr/local/bin/gmsl-first-boot.sh
SCRIPT
chmod +x /tmp/gmsl_first_boot.sh
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gmsl_first_boot.sh /tmp/gmsl_rpi_rootfs/usr/local/bin/gmsl-first-boot.sh
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chmod +x /tmp/gmsl_rpi_rootfs/usr/local/bin/gmsl-first-boot.sh
rm -f /tmp/gmsl_first_boot.sh

cat > /tmp/gmsl_first_boot.service <<'UNIT'
[Unit]
Description=GMSL first-boot dependency install
After=network-online.target time-sync.target
Wants=network-online.target
[Service]
Type=oneshot
ExecStart=/usr/local/bin/gmsl-first-boot.sh
[Install]
WantedBy=multi-user.target
UNIT
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gmsl_first_boot.service /tmp/gmsl_rpi_rootfs/etc/systemd/system/gmsl-first-boot.service
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A ln -sf /etc/systemd/system/gmsl-first-boot.service \
  /tmp/gmsl_rpi_rootfs/etc/systemd/system/multi-user.target.wants/gmsl-first-boot.service
rm -f /tmp/gmsl_first_boot.service
```

### 5e. Fix ownership of all deployed files

Files created via sudo end up owned by root. Fix ownership so the `analog` user
(UID 1000) can access everything:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown -R 1000:1000 /tmp/gmsl_rpi_rootfs/home/analog
```

## Step 6: Unmount and detach loop device

```bash
sync
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A umount /tmp/gmsl_rpi_boot
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A umount /tmp/gmsl_rpi_rootfs
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A losetup -d "$LOOP"
rmdir /tmp/gmsl_rpi_boot /tmp/gmsl_rpi_rootfs 2>/dev/null
losetup -j "<work_dir>/${IMG_NAME}"
```
The last command must print nothing (no loop device left on the image).

## Step 7: Write the SD card

Read `<skill dir>/common/11-sd-card.md` and follow §Detect (only if the card was not detected
yet, or is no longer present: `lsblk -dno NAME,SIZE /dev/<sd_device>` prints nothing),
§Confirm, §Write and §Release with `<image>` = `<work_dir>/${IMG_NAME}`.

**Exception — "No, abort"** at §Confirm: run the orchestrator's §Cleanup, but leave out
`<work_dir>/${IMG_NAME}` (the image is already unmounted and detached in Step 6). Then stop and
report that the modified image is available at `<work_dir>/${IMG_NAME}` for manual flashing, and
that they should delete it with `rm <work_dir>/${IMG_NAME}` once they have flashed it.

## Step 8: Clean up

The image is not needed after flashing: rpi/4 builds the overlay from templates it downloads
itself. Remove it and the mount points:
```bash
rm -f "<work_dir>/${IMG_NAME}" "<work_dir>/${IMG_FILE}"
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A rm -rf /tmp/gmsl_rpi_boot /tmp/gmsl_rpi_rootfs
ls -d "<work_dir>/${IMG_NAME}" "<work_dir>/${IMG_FILE}" /tmp/gmsl_rpi_boot /tmp/gmsl_rpi_rootfs 2>/dev/null
```
The last command must print nothing. The temporary files of Steps 4–5 (`/tmp/gmsl_userconf.txt`,
`/tmp/gmsl_video_cfg.sh`, `/tmp/gmsl_first_boot.*`) were already removed in those steps.

The kernel tree `<kernel_src_path>` stays in `<work_dir>`: the orchestrator's §Cleanup asks at the
end whether to keep `<work_dir>`. `sudo make modules_install` may have left root-owned files in
it; give them back to the current user now, while the sudo password is still in `<state>`, so the
tree can be removed later without sudo:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A chown -R "$(id -u):$(id -g)" <kernel_src_path>
```

## Step 9: Update state

```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['image_flashed'] = True
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

Tell the user: "SD card flashed successfully. You can now remove the SD card, insert it
into the Raspberry Pi, connect the Pi to the same network as this PC, and power it on. The first
boot takes a few minutes (the first-boot service installs qv4l2, GStreamer and the cam.py
dependencies when the Pi has Internet access)." Then continue with the orchestrator's first-boot
step (the "Yes" path, trying `expected_hostname` = `analog-rpi-gmsl` first).
