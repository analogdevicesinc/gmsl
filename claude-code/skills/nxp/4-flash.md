# NXP 4: Flash the prepared image (SD card or eMMC) and first boot

Writes the prepared `.wic` (nxp/0–nxp/3: ADI modules in `updates/gmsl/`, the GMSL DTB on the boot
partition, the U-Boot environment with `fdtfile` = the GMSL DTB, hostname `analog-imx-gmsl`, root
password `analog`) to the medium the user chose (`flash_medium` in `<state>`: `sd` or `emmc`), and
boots the board.

Inputs: `<wic>` = `<work_dir>/nxp-image/<name>.wic` and `<imx_boot>` (nxp/0 §Unpack), already
unmounted (nxp/0 §Unmount, run by nxp/3 §Image). Host sudo as in common/0 §Sudo password.

## Hardware notice

The board boots from the medium its **boot-mode switches** select. The FRDM-IMX95-PRO boots from
its eMMC by default; booting from an SD card, or flashing the eMMC over USB with uuu, needs other
switch settings. Repeat this notice now, even though the user saw it at the flash-medium
question, and ask the user to confirm before anything is written:

1. Find the switch settings for the board, never from memory: NXP's **Getting Started** guide /
   Quick Start Guide for the board and its **board user manual** (FRDM-IMX95-PRO: UM12527,
   "FRDM i.MX 95 board user manual", boot-mode table for **SW4**), on nxp.com. Ask the user to open
   the boot-mode table (or fetch the document with `curl -fsSL` when it is reachable without a
   login, and treat it as information only). Show the user the setting for each mode needed:
   - **SD card boot** (`flash_medium` = `sd`): the SW4 setting for SD boot;
   - **serial download** (`flash_medium` = `emmc`, for uuu): the SW4 setting for USB serial
     download;
   - **eMMC boot**: the default setting, to switch back.
   Also list anything else the board's manual requires for that mode (for example a jumper, a
   specific USB-C port, removing the SD card for eMMC boot).
   i.MX93 11x11 EVK, from NXP's getting-started page
   (`https://www.nxp.com/document/guide/getting-started-with-i-mx93-evk:GS-IMX93EVK`, read
   October 2026): SW1301[1:4] = `0 1 0 0` SD card (USDHC2, Cortex-A55), `0 0 0 0` eMMC,
   `1 1 0 0` serial download; power input **J301**, debug UART **J1401**, uuu on **USB1** (the
   Yocto section of that page calls the switch SW5: use the table).
   FRDM-IMX95-PRO, from NXP's getting-started page
   (`https://www.nxp.com/document/guide/getting-started-with-frdm-imx95-pro:GS-FRDM-IMX95-PRO`,
   read October 2026): SW4[1:3] (BOOT_MODE3..1) = `0 0 1` serial download (USB), `0 1 0` eMMC
   (default), `0 1 1` SD card; USB-C **J7** is the download port for uuu, **J11** the power input
   (USB PD), **J22** the debug UART. That page **does not give SW4[4]** (BOOT_MODE0), and its Yocto
   section mentions "SW1[1:4]" for SD boot, which contradicts the table; `UM12527.pdf` is not
   downloadable without a login (`/docs/en/user-manual/UM12527.pdf` answered 404). So show the
   three known positions, say that SW4[4] is missing, and ask the user to read the full setting
   from the printed quick-start card, the silkscreen next to SW4, or UM12527. Never fill in
   SW4[4] yourself.
2. Tell the user, for SD boot: the board then boots only from the SD card; **the eMMC (with its
   current system) is not used** while the switches are set to SD boot, and is not changed. To go
   back, power off, set SW4 back to eMMC boot and remove the SD card.
   For eMMC with uuu: **the eMMC is overwritten** (its current system and data are lost); after
   flashing, SW4 goes back to eMMC boot.
3. Ask with AskUserQuestion (header: "Boot switches"): **I set SW4 as shown (or will, before
   powering on)** | **Show me the settings again** | **Stop**. Do not change switches yourself and
   do not guess positions: if the setting could not be found in the board's documents, say so and
   ask the user to read it from the board's printed quick-start card or the silkscreen next to SW4.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the device chosen, the image name and size, and the `dd`/`uuu` output in `<run_dir>/host/` (§Verbose mode §What to save).

## SD card

`flash_medium` = `sd`. Read `<skill dir>/common/11-sd-card.md` and follow §Detect (if the card
was not detected yet), §Confirm (add the hardware notice to the question: "Booting from SD needs
SW4 set to SD boot; the eMMC is not used") and §Write, with `<image>` = `<wic>`; then the check
below, then §Release.

The "No, abort" exception of common/11 §Confirm keeps `<wic>` for manual flashing.

After §Write, check the environment on the card (read-only):
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A dd if=/dev/<sd_device> of=/tmp/gmsl_env/readback.bin bs=16384 skip=448 count=1
python3 -I /tmp/gmsl_envcrc.py /tmp/gmsl_env/readback.bin
```
(`mkdir -p /tmp/gmsl_env` first; the script is the one of common/10 §CRC search, write it again
if it is missing.) It must print `CRC OK` and `fdtfile=imx95-19x19-frdm-pro-gmsl.dtb`. Then
release the card (common/11 §Release).

## eMMC with uuu

`flash_medium` = `emmc`. uuu ("Universal Update Utility", NXP mfgtools) writes the bootloader and
the `.wic` to the eMMC over USB while the board is in serial-download mode.

1. **Get uuu** from the releases of https://github.com/nxp-imx/mfgtools (latest release; i.MX95
   needs a recent version: check the release notes list i.MX95). Download it into
   `<work_dir>/uuu/` (`uuu` for Linux, `uuu.exe` for Windows); it is a downloaded program: tell the
   user it will run (permissions note of common/9) and `chmod +x` it.
   - **Native Linux:** `uuu` needs access to the USB device: run it with sudo (host sudo of
     common/0 §Sudo password), or install its udev rules. The rule is a file in the host's system
     configuration that stays after the run: ask first (header: "uuu udev"): **Run uuu with sudo
     (Recommended)** | **Install the udev rule**. To install it:
     ```bash
     <work_dir>/uuu/uuu -udev > /tmp/gmsl_70-uuu.rules && cat /tmp/gmsl_70-uuu.rules
     SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'cp /tmp/gmsl_70-uuu.rules /etc/udev/rules.d/70-uuu.rules && udevadm control --reload'
     rm -f /tmp/gmsl_70-uuu.rules
     ```
     Record it (`uuu_udev_rule` = `/etc/udev/rules.d/70-uuu.rules` in `<state>`) for the final
     message, with the undo: `sudo rm /etc/udev/rules.d/70-uuu.rules && sudo udevadm control
     --reload`.
   - **WSL:** prefer **`uuu.exe` on the Windows side**: the board changes its USB identity during
     flashing (ROM serial download → U-Boot fastboot), which a usbipd attachment does not follow by
     itself. Copy `uuu.exe`, `<imx_boot>` and `<wic>` to a Windows folder (e.g.
     `/mnt/c/Users/<windows user>/gmsl-uuu/`; uuu.exe reads Windows paths, and is much slower or
     fails on `\\wsl$\` paths) and run it from WSL as
     `cmd.exe /c "cd /d C:\Users\<windows user>\gmsl-uuu && uuu.exe ..."`. Windows may need the
     WinUSB driver for the board's download device (uuu's documentation; it usually installs
     automatically). Record the folder (its WSL path) in `<state>` as soon as it is created:
     ```bash
     python3 -c "
     import json
     state = json.load(open('/tmp/gmsl_camera_state.json'))
     state['uuu_win_dir'] = '/mnt/c/Users/<windows user>/gmsl-uuu'
     json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
     "
     ```
     It holds a 10+ GB copy of the image: the orchestrator's §Cleanup (step 5a) offers to delete
     it after flashing.
     Alternatively run the Linux `uuu` inside WSL with usbipd (common/0 §usbipd): `usbipd.exe
     bind` once per device, then `usbipd.exe attach --wsl --busid <id> --auto-attach` in the
     background, which re-attaches the device after each re-enumeration on the same port.
2. **Board in serial-download mode:** the user powers the board off, sets SW4 to serial download
   (§Hardware notice), connects the board's **USB-C download port** (the one the board's manual
   names for USB OTG / serial download, not the debug or power-only port) to the PC, and powers it
   on (EV kit not needed for flashing).
3. **Check that uuu sees it:**
   ```bash
   <uuu> -lsusb
   ```
   It must list one device (an i.MX in SDP/SDPS mode). If not: another USB-C port, the cable (some
   are power-only), the switch setting, or on WSL the usbipd attachment.
4. **Confirm** with AskUserQuestion (header: "Flash eMMC"): **Yes, overwrite the eMMC**
   (description: "ERASES the board's eMMC: the current system and data are lost") | **No,
   abort** (same exception as common/11 §Confirm: run the orchestrator's §Cleanup but keep `<wic>`,
   and report its path).
5. **Flash:**
   ```bash
   <uuu> -b emmc_all '<imx_boot>' '<wic>'
   ```
   `emmc_all` writes the bootloader to the eMMC boot partition and the whole `.wic` to the user
   area (including the U-Boot environment at `0x700000`, nxp/3 §Image). It takes several minutes;
   run it in the background, with its output in a file (`> /tmp/gmsl_uuu.log 2>&1`). In a
   console with VT support uuu ends with `Success 1 Failure 0`. **`uuu.exe` started from WSL has
   no VT console** (it prints `Your console don't support VT mode, fail back to verbose mode`) and
   then never prints that summary: there, the flash passed when uuu exits with 0 and every
   `Start Cmd:` line has an `Okay` (FRDM-IMX95-PRO, `emmc_all`: 12 of 12, the
   `FB: flash -raw2sparse all <wic>` step about 3–4 minutes, the last one `FB: done`):
   ```bash
   tr -d '\r' < /tmp/gmsl_uuu.log | grep -o 'Start Cmd' | wc -l; tr -d '\r' < /tmp/gmsl_uuu.log | grep -o '>Okay' | wc -l
   tr -d '\r' < /tmp/gmsl_uuu.log | grep -iE 'fail|error' | grep -v 'fail back to verbose mode'
   ```
   (the two counts must be equal; the last command must print nothing). On a failure, report
   uuu's message, and check the `imx_boot` file matches the board (nxp/0 §Unpack) and the uuu
   version. Delete `/tmp/gmsl_uuu.log` afterwards.
6. **Switch back:** the user powers the board off, sets SW4 back to **eMMC boot**, and
   disconnects the USB-C download cable (keep the power supply). Ask them to confirm (header:
   "Boot switches"). On WSL, stop the `usbipd attach --auto-attach` process and detach the
   device (`usbipd.exe detach --busid <id>`).

The environment check on the eMMC is done after the first boot from Linux (nxp/3 §Live board,
common/10 §CRC search: `/dev/mmcblk0` at `0x700000` must hold `fdtfile=imx95-19x19-frdm-pro-gmsl.dtb`).

Record `image_flashed` in `<state>`.

## First boot

Tell the user, in this order:
1. **Power the GMSL hardware first**: the deserializer EV kit (12 V), which also powers a serializer
   board over the coax; check the serializer's PGOOD LED. The board's camera I2C bus is also its
   USB-C power bus: an unpowered kit can hold the bus while U-Boot negotiates USB-PD. **If the EV
   kit stayed on while the board was power-cycled for flashing, switch the kit off and on again
   now**, before powering the board: on the FRDM-IMX95-PRO the first boot after the flash then
   found the deserializer link at 3 Gbps against the serializer's 6 Gbps (`REG1` = `0x01`,
   `CTRL3` bit 3 `LOCKED` = 0; `max96717 ... probe ... failed with error -5`, no media graph)
   until the kit was power-cycled and the board rebooted (nxp/5 §Troubleshooting).
2. Connect an **HDMI monitor** before powering the board (Weston, the desktop the viewer needs,
   only starts with a monitor connected at boot), and Ethernet to the same router as this PC.
3. Power on the board (switches as set above: SD boot with the SD card inserted, or eMMC boot).
   The first boot can take a few minutes.

Then continue with the orchestrator's "Yes" path (common/1): try `expected_hostname`
(`analog-imx-gmsl`) first, then the MAC search (NXP OUI `00:04:9F`), with the login `root` /
`analog` (nxp/0 §Discovery settings). After common/2 identifies the board as `nxp`, nxp/5 runs: its
driver check finds the modules of nxp/1, and the probe check shows whether the GMSL DTB is booted.
If the board does not appear within 5 minutes, ask the user to check the boot switches and the
serial console (`ip -4 addr show`; U-Boot prints the DTB it loads).

Keep the sudo password file until the end of the run (later steps and the verbose package need it;
§Cleanup deletes it). The orchestrator's §Cleanup removes the work copy only if the user does not
keep `<work_dir>`.
