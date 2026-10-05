# GMSL Camera Setup – Claude Code Skill

This folder contains a [Claude Code](https://claude.com/claude-code) skill, `gmsl-camera-setup`,
that configures a GMSL camera pipeline on a Raspberry Pi from a single prompt.

When triggered, Claude asks you a few questions (RPi model, deserializer, serializers, camera
sensors, board IP and credentials) and then:

1. Generates a device tree overlay with `gen_gmsl_dts.py` from the ADI Linux kernel tree
2. Compiles it with `dtc` and copies it to the board's `/boot/overlays/`
3. Adds `dtoverlay=gmsl` to the board's `config.txt` and reboots the board
4. Runs `video_cfg.sh` on the board to configure the video pipeline
5. Launches `qv4l2` on the board's display

You can also ask for a GUI version, case in which Claude generates and launches a Python
tkinter application with the same functionality.

Supported hardware:

| Component    | Options                                                       |
|--------------|---------------------------------------------------------------|
| Platform     | Raspberry Pi 5 (CAM0 / CAM1), Raspberry Pi 4                  |
| Deserializer | MAX96724, MAX96724F, MAX9296A, MAX96716A, MAX96712, MAX96792A |
| Serializer   | MAX96717, MAX9295A, MAX96793                                  |
| Sensor       | IMX219, OV5640 (fully supported); IMX708, IMX415, ISX021 (overlay only) |

**Fully supported vs. overlay only:** for all sensors, the skill generates and installs a valid
device tree overlay, so the drivers probe after reboot. However, `video_cfg.sh` (and the
`cam.py` profiles it uses) only covers IMX219 and OV5640. For the other sensors the video
pipeline configuration step may fail or leave the video nodes unconfigured, so you may need
to configure the media pipeline manually before streaming. Claude warns you when you select
one of these sensors.

**CAM0 / CAM1:** the Raspberry Pi 5 has two 4-lane MIPI CSI-2 camera connectors, labeled
CAM0 and CAM1 on the board. Select the one the deserializer board is plugged into. The
Raspberry Pi 4 has a single 2-lane camera connector, so there is no port choice.

## Prerequisites

### Host machine (where Claude Code runs)

- [Claude Code](https://docs.claude.com/en/docs/claude-code/overview) installed and logged in
- `python3` (and `python3-tk` if you want the GUI)
- `curl`, `ssh`, `scp`
- `sshpass`
- `dtc` (device tree compiler)

On Debian/Ubuntu:

```bash
sudo apt install python3 python3-tk curl openssh-client sshpass device-tree-compiler
```

The host needs Internet access to download the `gen_gmsl_dts` templates from
[analogdevicesinc/linux](https://github.com/analogdevicesinc/linux/tree/gmsl/rpi-6.13.y/arch/arm/boot/dts/overlays/gen_gmsl_dts).

### Target board (Raspberry Pi)

- Raspberry Pi OS image with the ADI GMSL kernel (drivers for the deserializer/serializer/sensor)
- SSH enabled and reachable from the host
- A user with `sudo` rights (default: `analog` / `analog`)
- `video_cfg.sh` at `/home/<user>/Workspace/video_cfg.sh` (see [`rpi/video_cfg.sh`](../rpi/video_cfg.sh)),
  together with the `pyv4l2` utilities it references
- `qv4l2` installed

The skill does not install packages on the board, so all of the above must already be present.

## Installation

The skill is distributed as a `CLAUDE.md` file. Claude Code automatically loads `CLAUDE.md`
files as instructions, so installing the skill means placing this file where Claude Code
will read it. Choose one of the options below.

### Option 1: User-wide (available in every project)

```bash
mkdir -p ~/.claude
cp claude-code/CLAUDE.md ~/.claude/CLAUDE.md
```

If you already have a `~/.claude/CLAUDE.md`, append the skill instead of overwriting it:

```bash
cat claude-code/CLAUDE.md >> ~/.claude/CLAUDE.md
```

### Option 2: Per project (only in a specific working directory)

```bash
cp claude-code/CLAUDE.md /path/to/your/project/CLAUDE.md
```

Again, append with `cat ... >>` if the project already has a `CLAUDE.md`.

### Option 3: Run Claude Code from this folder

No copying needed. Start Claude Code from inside this folder and the skill is picked up:

```bash
cd claude-code
claude
```

### Verify the installation

Inside Claude Code, run `/memory` to list the loaded `CLAUDE.md` files and check that the one
containing `gmsl-camera-setup` is present.

## Usage

Start Claude Code and describe what you want, for example:

```
Set up a GMSL camera on my Raspberry Pi
```

```
Configure a MAX96724 with two IMX219 cameras on RPi 5
```

```
Open the GMSL camera setup GUI
```

Answer the questions Claude asks. When deployment finishes, press the green **Play** button
in `qv4l2` on the board's display to start the video feed.

> **Note:** Claude will run `ssh`, `scp`, `sudo` and `reboot` commands against the board.
> Review the permission prompts as they appear. Board credentials are passed via `sshpass`,
> so use this only on trusted lab networks.

## Updating

Pull the latest version of this repository and repeat the installation step (if you appended
the file, replace the old `gmsl-camera-setup` section rather than appending it twice).

## License

Apache License 2.0 – see [LICENSE](LICENSE).
