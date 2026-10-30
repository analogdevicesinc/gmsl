# Common 11: SD card (detect, confirm, write, release)

Detects the SD card on the host by comparing the devices before and after insertion, then writes
an image to it with `dd`. Supports USB card readers and built-in laptop SD card slots on WSL2,
and any SD card reader on native Linux. Used by the RPi flash path (rpi/3, image = the modified
Raspberry Pi OS image) and the NXP SD path (nxp/4 §SD card, image = the prepared `.wic`).

Parameter: `<image>`, the full path of the (uncompressed) image file to write. The platform file
gives it.

In this file, `<pw>` is the **host's sudo password** (`/tmp/gmsl_pw_sudo`, from common/0 §Sudo
password; passwordless sudo: use plain `sudo`), not the board's login.

Read from `<state>`: `host` (`wsl` or `linux`), `sudo_pw_file`. Written to `<state>`:
`sd_type` (`usb`, `builtin` or `native`), `sd_device`, `sd_busid`, `sd_disk_number`.

Sudo on the host: if the password is empty, `sudo <cmd>`; otherwise
`SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c '<cmd>'` (common/0 §Sudo password). Here `<pw>` is the **host**
sudo password from `<state>`, not the board's.

On WSL this needs `usbipd` (common/0 §usbipd) for USB readers and `wsl --mount` for built-in
slots. Check `wsl --mount` support once:
```bash
wsl.exe --mount 2>&1 | iconv -f utf-16le -t utf-8 | tr -d '\r' | grep -q 'Invalid command line argument: --mount' && echo "wsl --mount: not supported" || echo "wsl --mount: supported"
```
(`wsl.exe` prints UTF-16; `--mount` without arguments prints the usage text when it exists, and
`Invalid command line argument: --mount` when it does not; `--help` is never accepted.) If it is
supported, built-in SD card slots can be used. If not, only USB readers will work: inform the user.

**Running Windows commands as administrator:** `usbipd.exe bind` and `wsl.exe --mount` /
`--unmount` need administrator rights on Windows. Run them directly first. If they fail with an
access-denied or "administrator" error, run them elevated through PowerShell instead. Windows then
shows a UAC prompt: tell the user to approve it, and wait for the command to finish:
```bash
powershell.exe -NoProfile -Command 'Start-Process <program> -ArgumentList "<arg1>","<arg2>" -Verb RunAs -Wait'
```
Keep this command in single quotes in bash, so the backslashes in `\\.\PHYSICALDRIVE<N>` reach
PowerShell unchanged. If the user declines the UAC prompt, report it and use AskUserQuestion to
ask whether to retry or stop.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the `lsblk` line of the chosen device, the image name and size, and the `dd` output (§Verbose mode §What to save).

## Detect

Can run early (the RPi path detects the card before the kernel build, so the long steps run
unattended); §Confirm re-checks it before writing.

1. **"Before" snapshot (SD card NOT inserted).** Tell the user: "Make sure the SD card is NOT
   inserted yet." Use AskUserQuestion:
   - **SD Card** (header: "SD Card"): "SD card is NOT inserted, ready to continue" | Other

   Then capture the baseline. On both hosts:
   ```bash
   lsblk -dno NAME,SIZE 2>/dev/null > /tmp/gmsl_lsblk_before.txt
   ```
   On WSL only, also capture the Windows USB devices and physical disks:
   ```bash
   usbipd.exe list 2>/dev/null > /tmp/gmsl_usb_before.txt
   powershell.exe -NoProfile -Command 'Get-Disk | Sort-Object Number | ForEach-Object { "{0}|{1}|{2}|{3}" -f $_.Number, $_.FriendlyName, $_.BusType, $_.Size }' 2>/dev/null | tr -d '\r' > /tmp/gmsl_disk_before.txt
   ```
2. **Insert the SD card.** Use AskUserQuestion:
   - **Insert SD** (header: "Insert SD"): "SD card is now inserted, ready to continue" | Other

   Wait 3 seconds for device enumeration: `sleep 3`.
3. **"After" snapshot and diff.**
   - **Native Linux** (`host` is `linux`): the new block device is the SD card. Skip step 4 and go
     straight to step 5.
   - **WSL**: take the Windows snapshots again and compare:
     ```bash
     usbipd.exe list 2>/dev/null > /tmp/gmsl_usb_after.txt
     powershell.exe -NoProfile -Command 'Get-Disk | Sort-Object Number | ForEach-Object { "{0}|{1}|{2}|{3}" -f $_.Number, $_.FriendlyName, $_.BusType, $_.Size }' 2>/dev/null | tr -d '\r' > /tmp/gmsl_disk_after.txt
     diff /tmp/gmsl_usb_before.txt /tmp/gmsl_usb_after.txt
     diff /tmp/gmsl_disk_before.txt /tmp/gmsl_disk_after.txt
     ```
4. **Auto-detect the type and attach to WSL (WSL only).**
   - **If a new USB device appeared** (new line in the usbipd diff): extract the bus ID from the
     new line (format: `<BUSID>  <VID:PID>  <description>  <state>`). If its state is already
     `Attached`, skip bind and attach (the reader is in WSL already). If it is `Shared`, only
     attach. Otherwise bind (needs administrator rights, see above) and attach:
     ```bash
     usbipd.exe bind --busid <busid>
     usbipd.exe attach --wsl --busid <busid>
     ```
     If `bind` warns about USBPcap ("'bind --force' will be required"), tell the user and use
     `usbipd.exe bind --force --busid <busid>` only with their consent (it bypasses the USB filter
     driver for that device). Store `sd_type = "usb"`, `sd_busid = "<busid>"`.
   - **Else if a new physical disk appeared** (new line in the Get-Disk diff): the disk number is
     the first field of the new line (e.g. `2|Generic Mass-Storage|USB|...` → `2`). Mount it to WSL
     (needs administrator rights, see above):
     ```bash
     wsl.exe --mount '\\.\PHYSICALDRIVE<N>' --bare
     ```
     Store `sd_type = "builtin"`, `sd_disk_number = "<N>"`.
   - **Else**: no new device detected. A reader that was already attached to WSL before the
     "before" snapshot (for example after a pause) shows no difference either: list the removable
     USB disks in WSL (`lsblk -dno NAME,SIZE,RM,TRAN,MODEL | awk '$3 == 1 && $2 != "0B"'`); if one
     is there, confirm it is the card by its contents (step 5) before using it. Otherwise use
     AskUserQuestion to ask the user to check the SD card and retry, or stop.
5. **Verify the block device.** Wait 2 seconds, then diff lsblk:
   ```bash
   sleep 2
   lsblk -dno NAME,SIZE > /tmp/gmsl_lsblk_after.txt
   diff /tmp/gmsl_lsblk_before.txt /tmp/gmsl_lsblk_after.txt
   ```
   Extract the new device name (e.g. `sda` or `mmcblk0`) and store it as `sd_device`. Ignore new
   devices of size `0B` (an empty slot of a multi-slot reader). On native Linux, store
   `sd_type = "native"`. Device names and usbipd bus IDs can change between sessions and after a
   WSL restart: never reuse a stored `sd_device` or `sd_busid` from an earlier session without
   detecting the card again; when the card already holds an image, its contents identify it
   (`lsblk -o NAME,SIZE,LABEL,FSTYPE /dev/<dev>`: partition labels `boot`/`root` on NXP images).

   If no new block device appeared, report the error and ask the user to troubleshoot (reseat the
   card or the reader, then repeat from step 1). If more than one new block device appeared, show
   them with their sizes and use AskUserQuestion to let the user pick the SD card.
6. **Update state** and remove the diff files:
   ```bash
   python3 -c "
   import json
   state = json.load(open('/tmp/gmsl_camera_state.json'))
   state['sd_type'] = '<usb, builtin or native>'
   state['sd_device'] = '<device_name>'
   state['sd_busid'] = '<busid_or_empty>'
   state['sd_disk_number'] = '<disk_number_or_empty>'
   json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
   "
   rm -f /tmp/gmsl_usb_before.txt /tmp/gmsl_usb_after.txt /tmp/gmsl_disk_before.txt \
     /tmp/gmsl_disk_after.txt /tmp/gmsl_lsblk_before.txt /tmp/gmsl_lsblk_after.txt
   ```

## Confirm

Right before writing:
1. **Verify the SD card is still present:**
   ```bash
   lsblk -dno NAME,SIZE,RM,TRAN,MODEL "/dev/<sd_device>" 2>/dev/null
   lsblk -nrpo NAME,MOUNTPOINT "/dev/<sd_device>"
   for m in / /boot /boot/efi; do findmnt -no SOURCE "$m" 2>/dev/null; done   # one path per call: two arguments mean source + target
   ls -l <image>
   ```
   If the device is not found, the SD card may have been removed or the usbipd attachment dropped.
   Then run §Detect again (including its removal of the diff files). Check that the card is at
   least as large as `<image>`; if it is smaller, stop and tell the user.
2. **Safety check — refuse, stop and ask** (header: "SD check") instead of flashing when:
   - the device is not removable and not on USB or an SD/MMC bus: `RM` is `0` **and** `TRAN` is
     not `usb` or `mmc` (an internal SATA/NVMe disk, or a virtual disk). Exception: a built-in
     slot mounted with `wsl --mount` (`sd_type` = `builtin`) shows up as a SCSI disk with `RM` =
     `0`; there, check instead that the Windows disk `<sd_disk_number>` has `BusType` `SD`, `MMC` or
     `USB` (`powershell.exe -NoProfile -Command "(Get-Disk -Number <sd_disk_number>).BusType"`);
   - the device or one of its partitions holds a mounted host filesystem: `/`, `/boot`,
     `/boot/efi` (the `findmnt` sources above are on `/dev/<sd_device>`), any other mount point
     outside `/media`, `/run/media` and `/mnt` (the second `lsblk` command), or, on WSL, the WSL system
     disk (the device that holds `/`: `findmnt -no SOURCE /`, for example `/dev/sdd`; it can
     change after a WSL restart, so check it every time).
   Show the `lsblk` line to the user, say why it does not look like the SD card, and offer
   **Run §Detect again** | **Stop**. Never flash such a device, even if the user insists: ask
   them to check the device with `lsblk` themselves.
3. **Ask** with AskUserQuestion, 1 question (the platform file may add its own notice to the
   question, e.g. nxp/4 §Hardware notice for the boot-mode switches). Show the
   `lsblk -dno NAME,SIZE,RM,TRAN,MODEL` line of step 1 in the question:
   - **Flash SD Card** (header: "Flash SD"):
     - "Yes, flash /dev/<sd_device> (<size>, <model>)" (description: "WARNING: This will ERASE ALL
       DATA on this device. This action cannot be undone.")
     - "No, abort" (description: "Stop here without flashing")

   If the user selects "No, abort": run the orchestrator's §Cleanup, but **keep `<image>`** (it is
   already unmounted and detached by the platform file). Then stop, and report that the modified
   image is available at `<image>` for manual flashing, and that they should delete it once they
   have flashed it.

## Write

Desktop Linux usually mounts an inserted SD card's partitions automatically. Unmount any mounted
partition of the SD card first, otherwise `dd` writes under a mounted filesystem:
```bash
for p in $(lsblk -nrpo NAME,MOUNTPOINT /dev/<sd_device> | awk '$2 != "" {print $1}'); do
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A umount "$p"
done
lsblk -nrpo NAME,MOUNTPOINT /dev/<sd_device>
```
The last command must show no mount points. If a partition could not be unmounted, stop and
report it.

Then flash:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A dd if="<image>" of=/dev/<sd_device> bs=4M status=progress conv=fsync
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sync
```
This may take several minutes depending on the image size and SD card speed (run it in the
background for large images and wait for it). Pipe the password into every sudo command after
`dd` as well: the cached sudo credentials can expire while `dd` runs, so a plain `sudo`
afterwards fails with "a terminal is required".

After flashing, verify the SD card has partitions:
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A partprobe /dev/<sd_device> 2>/dev/null; lsblk "/dev/<sd_device>"
```
Record `image_flashed` in `<state>`.

## Release

Release the card so it can be removed safely, depending on `sd_type` in `<state>`:
- `usb`: `usbipd.exe detach --busid <sd_busid>`
- `builtin`: `wsl.exe --unmount '\\.\PHYSICALDRIVE<sd_disk_number>'` (needs administrator
  rights; if it fails with an access-denied error, run it elevated as above). Until it is
  unmounted from WSL, Windows keeps the card offline.
- `native`: nothing to release; `dd` with `conv=fsync` and `sync` already flushed all writes.

Then tell the user they can remove the SD card and insert it into the board (the platform file
gives the boot instructions, e.g. nxp/4 §First boot).
