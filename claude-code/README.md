# GMSL Camera Skills for Claude Code

A Claude Code skill that takes a GMSL camera setup from one prompt to a live video feed on the
board's display: it finds the board on the network (or prepares one), identifies it, asks for the
GMSL hardware (ADI deserializer, serializers, camera sensors), makes sure the kernel has the ADI
GMSL drivers (building them when they are missing), generates and installs the device tree,
configures the media pipeline, checks that frames arrive and starts a viewer on the board.

With no board ready, it builds a kernel or the ADI drivers and flashes an SD card (Raspberry Pi,
NXP i.MX) or the eMMC (NXP i.MX).

## Install

The skill is the `skills` folder next to this README: `SKILL.md` (the only registered skill,
named `gmsl-camera-pipeline`) and the `common/`, `rpi/`, `nxp/`, `generic/` subfolders that it
reads during the run. Run the commands below from the folder that contains this README.

**Option 1 (Recommended): Run Claude Code from the skill folder (no copying)**

The simplest option: nothing to install or update.
```bash
cd skills
claude
```
The skill is not registered this way: Claude Code reads `SKILL.md` from the current folder. If it
does not pick it up, start with "follow SKILL.md".

**Option 2: User-wide (available in every project)**

Always copy the whole folder.
```bash
mkdir -p ~/.claude/skills
cp -r skills ~/.claude/skills/gmsl-camera-pipeline
```

**Option 3: Per project (only in a specific working directory)**

Always copy the whole folder.
```bash
mkdir -p /path/to/your/project/.claude/skills
cp -r skills /path/to/your/project/.claude/skills/gmsl-camera-pipeline
```

**Verify:** with option 2 or 3, type `/` in Claude Code: `gmsl-camera-pipeline` must be in the
list. To update, delete the copied `gmsl-camera-pipeline` folder and copy again.

## Usage

Start Claude Code (`claude`) and ask, for example:
- "set up my GMSL camera"
- "set up my GMSL camera on 192.168.1.42" (an IP address skips the board search)
- "build a Raspberry Pi 5 SD card with the ADI GMSL drivers"
- "flash my FRDM-IMX95-PRO with GMSL support"
- "GMSL setup only" (nothing is built or flashed; the board must already have the ADI drivers)

By default the **entire pipeline** runs: even when the prompt mentions flashing or building, the
skill first asks whether the board is on the network. **A board on the network is never
flashed**: if its kernel lacks the ADI GMSL drivers, they are patched and built for the kernel it
runs and installed on it. Flashing happens only on the "No" path (no board running yet). Say
"GMSL setup only" to skip building and flashing altogether.

The very first question is how verbose the whole run should be:
- **Non-verbose**: the normal run, progress in the chat only.
- **Verbose (debug log + ZIP)**: everything is recorded in a run folder `gmsl_debug_<date>_<time>`
  in the folder Claude Code was started in: an activity log (`adi_gmsl_agent_log.txt`: every step,
  command with its output and exit code, detected value, answer and decision), `choices.json`
  (the hardware answers), board diagnostics (`dmesg`, `lsmod`, media topology, assessment report,
  device tree), the generated device tree files and scripts, build logs and driver patches, and a
  copy of the Claude Code session transcript. At the end, passwords are replaced by `****` in all
  files, the folder is packed into `gmsl_debug_<date>_<time>.zip` (checked) and the folder is
  deleted. Share the ZIP to report a problem.

Saying "verbose" or "debug" (or "non-verbose", "quiet") in the prompt answers it.

The next question is always whether the board is running and on the network:
- **Yes**: the skill finds the board (mDNS, then a MAC address search on the local subnets, then
  it asks for the IP address and login), runs a read-only assessment on it, selects the platform
  and continues with the camera setup.
- **No**: the skill asks which board to prepare (Raspberry Pi 4 / 5, NXP i.MX), builds and flashes
  the medium, tells you how to boot it, and then continues as for **Yes**.

At the end, host-side temporary files are removed, and the final message lists what is installed on
the board and how to undo it. Nothing is removed from the board: it is left in its deployed state.

## Supported Hardware

| Board | Live board | Build/flash | Viewer |
|-------|-----------|-------------|--------|
| Raspberry Pi 4/5 Model B | ADI GMSL images, or other Raspberry Pi OS images | SD card: Raspberry Pi OS with a kernel built from the Raspberry Pi tree plus the ADI GMSL drivers | qv4l2, GStreamer as the fallback |
| NXP i.MX with the NXP BSP | ADI modules built against NXP's kernel source and installed in `updates/gmsl/`, merged GMSL DTB, U-Boot `fdtfile` setting | Your NXP release image (downloaded from nxp.com) with the same changes, written to an SD card or to the eMMC with `uuu` | GStreamer |
| Other Linux boards | Best effort: assessment, driver build/port against the board's kernel where possible, device tree glue from the board's own device tree; an artifacts package when something cannot be done | — | GStreamer |

Booting an NXP board from an SD card needs hardware changes on the board (boot-mode switches,
for example SW4 on the FRDM-IMX95-PRO, and anything else the board's manual lists); flashing the
eMMC needs the serial-download switch setting and a USB-C cable. The skill tells you when, but you
set the switches yourself.

GMSL chips and cameras:

**Deserializers**
- MAX96724
- MAX96724F
- MAX96712
- MAX9296A
- MAX96716A
- MAX96792A
- MAX96714 (other boards only)

**Serializers**
- MAX96717
- MAX9295A
- MAX96793

**Camera sensors** (all with a device tree template)
- Fully supported (device tree overlay, ready media pipeline configuration, frame check and live
  view):
  - IMX219
  - OV5640
  - IMX415
- Device tree overlay only (the media pipeline, frames and live view are best effort):
  - IMX708
  - ISX021

## Host Requirements

- **WSL2 (Windows) or Linux** for everything. The build/flash paths and
  driver builds need `sudo` on the host (the skill asks for the password once and installs the
  build tools with `apt-get`; Claude Code must be in manual mode for this, see
  [Permissions](#permissions)), about 15 GB free in the work folder, and Internet access.
  On WSL, SD cards and USB devices are passed through with `usbipd-win` (installed by the skill
  with `winget` if missing).
- **Native Windows**: only for setting up a board that is already running with the ADI drivers
  (SSH through Python `paramiko`). Building and flashing are not possible there.
- Minimum tools for a live board (the skill checks them at the start and offers to install the
  missing ones):
  `sudo apt install python3 python3-jinja2 curl openssh-client sshpass device-tree-compiler git`.

## Permissions

**Run Claude Code in manual mode** (the default permission mode, where you approve each command)
for any run that uses `sudo`: the build/flash paths, building drivers for a board's kernel, and
installing missing host tools. The skill passes your sudo password to `sudo -S`, and in auto mode
Claude Code's safety checks block those commands, so the run stops at the first `sudo` step.
Switch modes with Shift+Tab before starting (or when the skill asks for the sudo password). A
board that already has the ADI drivers needs no host `sudo`.

## Folder Overview

```
skills/
│
├── SKILL.md                                        orchestrator: platforms, state, flow, rules, cleanup, final message
│
├── common/
│   │
│   ├── 0-host.md                                   host OS (Windows/WSL/Linux), host tools, sudo password, usbipd
│   │
│   ├── 1-find-board.md                             mDNS + MAC address search, default logins, manual IP/credentials
│   │
│   ├── 2-identify-board.md                         assess.sh, running device tree, kernel config, DTBs, platform choice
│   │
│   ├── 3-camera-questions.md                       deserializer/serializer/sensor, EV kit checks, I2C probe, path check, driver check
│   │
│   ├── 4-device-tree.md                            JSON, templates, gen_gmsl_dts, dtc, glue, validation, merged DTB
│   │
│   ├── 5-connect-reboot.md                         SSH control socket, reboot, probe and camera port checks
│   │
│   ├── 6-i2c-check.md                              i2cburst tests, I2C retry workaround
│   │
│   ├── 7-media-pipeline.md                         media-ctl routes and formats, media_cfg.sh
│   │
│   ├── 8-frames-viewer.md                          frame check, desktop detection, qv4l2, GStreamer
│   │
│   ├── 9-vendor-modules.md                         ADI modules built out of tree against a vendor kernel
│   │
│   ├── 10-uboot-env.md                             U-Boot envtools, environment CRC search, fw_setenv, offline env image
│   │
│   ├── 11-sd-card.md                               SD card detect, confirm, write, release
│   │
│   └── assess.sh, i2cburst.c, i2c_retry_regmap.h   helpers
│
├── rpi/                                            Raspberry Pi: kernel source, ADI drivers, kernel build, image, camera setup
│
├── nxp/                                            NXP i.MX: base image, kernel modules, device tree, U-Boot env, flash, camera setups
│
└── generic/                                        other boards: assessment, camera setup, artifacts
```
