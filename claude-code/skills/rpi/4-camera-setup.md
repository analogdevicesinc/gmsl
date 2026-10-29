# RPi 4: GMSL camera setup on a running Raspberry Pi

Configure and deploy the GMSL camera overlay on a Raspberry Pi 4 or 5 that is running and
reachable over SSH, configure the video pipeline with the pyv4l2 tools (`video_cfg.sh` / cam.py),
check that frames arrive, and start a viewer on the board's display (qv4l2 first, GStreamer as the
fallback).

Run this after common/1 (board found, logged in), common/2 (assessment, `platform` = `rpi`) and
common/3 (hardware questions, I2C probe, driver check; the orchestrator runs it with this file's
§Hardware choices and §Drivers, and its driver check is the only driver question). Placeholders (`<ip>`, `<user>`, `<pw>`, `<board_home>`, `<tpl_dir>` =
`/tmp/gen_gmsl_dts/rpi`, `<state>`) are the orchestrator's (SKILL.md §Placeholders). All commands
on the board go through the control socket of common/5 §Control socket:
`ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "<cmd>"`, and sudo on the board is
`echo '<pw>' | sudo -S sh -c '<cmd>'`.

## Order

common/3 §Driver check has already run (the orchestrator, before this file). If the drivers were
missing, the orchestrator built and installed them for the running kernel first ("Drivers
missing on a board on the network"; the board is never reflashed); with **Continue anyway**,
continue below.

1. Read `<skill dir>/common/4-device-tree.md` and follow §JSON (the **`platform_cfg` variant**,
   with the values of §JSON mapping below), §Bus speed (nothing to do on the RPi), §Alias pools
   (keep the defaults), §Templates (both variants' files plus `rpi-4-b.dtsi.in rpi-5-b.dtsi.in`),
   §Generate and §Compile. Skip §Glue, §Validate and §Merged DTB.
2. Read `<skill dir>/common/5-connect-reboot.md` and follow §Control socket.
3. §Install device tree (below).
4. common/5 §Reboot, §Probe check and §Camera port check (the RPi 5 has a second camera port:
   with "Not sure" in §Hardware choices, switch from CAM0 to CAM1 and deploy again from step 1;
   the camera I2C bus for `i2cdetect` is in §Troubleshooting).
5. If the probe check shows I2C errors (`-110`, `-5`) after the deserializer probed, or a later
   stream start fails with them: read `<skill dir>/common/6-i2c-check.md` and follow §Run,
   §Results and, only with the user's agreement, §Retry workaround (on a Raspberry Pi the retry
   patch needs the kernel build path, rpi/0–3). ADI's reference setup does not need it.
6. §Tools.
7. §Configure pipeline (video_cfg.sh; common/7 only as the fallback for sensors without a cam.py
   profile).
8. §Frame check (common/8 §Frame check).
9. §Viewer (common/8 §Desktop session, then §qv4l2, falling back to §GStreamer).
10. The orchestrator's §Cleanup (host only) and §Final message, with the content of §Final
    message below (one final message, not two).

## Profile
- Boards: Raspberry Pi 5 Model B (state `rpi_model` = `5`, two 4-lane camera connectors CAM0 /
  CAM1) and Raspberry Pi 4 Model B (`rpi_model` = `4`, one 2-lane camera connector). The cam.py
  profile prefix is `rpi<rpi_model>`: `rpi5` or `rpi4` (§Configure pipeline). Detect the model
  from the model string (`Raspberry Pi 5 Model` / `Raspberry Pi 4 Model`, common/2 §Platform
  selection). If `<state>` has a different `rpi_model` (for example from the build path), tell
  the user and use the detected model. Other Raspberry Pi models (CM4/CM5, Pi 400/500, Pi 3) are
  `generic` (common/2 §Platform selection) and do not reach this file.
- ADI GMSL image (or an SD card prepared by rpi/3): hostname starting with `analog-rpi-gmsl`, login
  `analog` / `analog`, and `~/Workspace/pyv4l2` present (common/1 §Login prints `pyv4l2`).
  Other Raspberry Pi OS images are accepted too (common/3 §Driver check checks the kernel).
- ADI kernel branch: the newest ADI GMSL branch (`adi_gmsl_branch`, common/4 §Templates; `gmsl/rpi-6.13.y`
  when tested) of https://github.com/analogdevicesinc/linux.
- Tested OS: the ADI GMSL Raspberry Pi images (X11 desktop started with startx) and Raspberry Pi
  OS Desktop (Trixie, labwc + Xwayland) with the kernel of rpi/0–2.

## Hardware choices
The option lists for common/3 §Hardware questions and §Per-camera questions.

Ask the RPi model only if it was not detected:
- **RPi Model** (header: "RPi Model"): Raspberry Pi 5 (4 MIPI lanes) | Raspberry Pi 4 (2 MIPI lanes)

Then, in one AskUserQuestion call:
- **Deserializer** (header: "Deserializer"): MAX96724 (4-link) | MAX9296A (2-link) | MAX96716A (2-link) | MAX96712 (4-link). Also available but less common: MAX96724F (1-link), MAX96792A (1-link GMSL3) — put these in "Other".
- **Camera count** (header: "Cam Count"): 1 / 2 / 3 / 4 (max depends on deserializer: MAX96724/MAX96712=4, MAX9296A/MAX96716A=2, MAX96724F/MAX96792A=1)
- **Camera Port** (header: "Camera Port", RPi 5 only): CAM0 | CAM1 | Not sure — the camera
  connector the deserializer board's cable is plugged into, as labeled on the board (say this in
  the question). For "Not sure", use CAM0 first; the camera port check of common/5 switches to
  CAM1 if nothing answers. Skip for RPi 4 (store `hw.connector` = `CAM0` for it).

The adapter is ADI's AD-GMSLCAMRPI-ADP with the deserializer's PHY B (cable on P6, "CAM 2"), as in
ADI's Raspberry Pi user guide; the PHY values of §JSON mapping assume it, so the adapter question
of common/3 is not asked on a Raspberry Pi. If the user says the cable is on P8 ("CAM 1", PHY A),
tell them that ADI's RPi setup uses P6 and ask them to move it (do not change the mapping).

Sensors (per-camera questions): IMX219 | OV5640 | IMX415 (fully supported) | IMX708 (device tree
overlay only). Also available: ISX021 (device tree overlay only); the levels are in common/3
§Sensor support levels. cam.py has pipeline configurations for the fully supported sensors
(IMX219, OV5640, IMX415). The overlay-only sensors get a valid DTS overlay but no cam-config
profile: warn the user (§Configure pipeline then uses the media-ctl fallback of common/7).

Serializers: MAX96717 (GMSL2) | MAX9295A (GMSL2) | MAX96793 (GMSL3), as in common/3.

## Drivers
The Raspberry Pi values for common/3 §Driver check (which asks the only driver question; this
file does not ask it again):
- Sensor driver: the sensor name in lowercase (`imx219`, `ov5640`, `imx415`, `imx708`, `isx021`).
- What the build option of that question means on a Raspberry Pi (say it in its description):
  the ADI drivers are built on the host for the kernel the board runs and installed on the board
  (the orchestrator's "Drivers missing on a board on the network", Raspberry Pi); the board and
  its SD card are not reflashed. It needs a WSL or Linux host (common/0 §Host OS). If the running
  kernel lacks core changes that modules cannot carry, the orchestrator says so; a GMSL kernel on
  a separate SD card is then a new run with Q1 **No** (rpi/0 → rpi/3).
- **Continue anyway**: the overlay is installed, but the cameras will not probe.
- Only the sensor driver missing: the kernel of rpi/2 builds `imx219` and `ov5640` in.

## JSON mapping
Values for common/4 §JSON, `platform_cfg` variant:

| Board / port | `platform_cfg.name` | `i2c_bus` | `csi_idx` | `num_lanes` | `data_lanes` |
|--------------|---------------------|-----------|-----------|-------------|--------------|
| RPi 4 | `rpi-4-b` | `i2c_csi_dsi` | 1 | 2 | `[1,2]` |
| RPi 5 CAM0 | `rpi-5-b` | `i2c_csi_dsi0` | 0 | 4 | `[1,2,3,4]` |
| RPi 5 CAM1 | `rpi-5-b` | `i2c_csi_dsi1` | 1 | 4 | `[1,2,3,4]` |

- Deserializer `phy_idx` (the same value in `platform_cfg.phy_idx` and `phys[0].phy_idx`):

  | max9296a | max96712 | max96716a | max96724 | max96724f | max96792a |
  |----------|----------|-----------|----------|-----------|-----------|
  | 1 | 2 | 1 | 2 | 2 | 1 |

- `clock_lanes`: `[0]`; `link_frequencies`: `[750000000]`.
- Pool addrs per link: the defaults link0=["0x50","0x51"], link1=["0x52","0x53"],
  link2=["0x54","0x55"], link3=["0x56","0x57"]. They are free on the Raspberry Pi camera bus, so
  §Alias pools of common/4 needs no probing here, and no top-level deserializer `pool_addrs`.
- Sensor name: the sensor name in lowercase (`imx219`, `ov5640`, ...).
- The camera I2C bus already runs at 100 kHz in ADI's RPi templates (common/4 §Bus speed).

Example (RPi 5, CAM0, MAX96716A, one MAX96717 + IMX219):
```json
[{
    "name": "max96716a",
    "i2c_bus": "i2c_csi_dsi0",
    "platform_cfg": { "name": "rpi-5-b", "csi_idx": 0, "phy_idx": 1 },
    "phys": [{ "phy_idx": 1, "num_lanes": 4, "link_frequencies": [750000000],
               "clock_lanes": [0], "data_lanes": [1,2,3,4] }],
    "links": [{ "name": "max96717", "cameras": [{"name": "imx219"}],
                "pool_addrs": ["0x50","0x51"] }]
}]
```

## Install device tree
First check whether the boot files are in `/boot/firmware` or directly in `/boot`:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "test -d /boot/firmware/overlays && echo fw || echo boot"
```
- `fw`: overlays dir is `/boot/firmware/overlays/`, config path is `/boot/firmware/config.txt`
- `boot`: overlays dir is `/boot/overlays/`, config path is `/boot/config.txt`

Copy the overlay to the detected overlays dir:
```bash
scp -o ControlPath=/tmp/gmsl_ssh_sock /tmp/gen_gmsl_dts/gmsl.dtbo <user>@<ip>:<board_home>/gmsl.dtbo
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S cp <board_home>/gmsl.dtbo <overlays_dir>/"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "rm -f <board_home>/gmsl.dtbo"
```

Enable it in the boot config:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S sed -i '/^dtoverlay=gmsl/d' <config_path>"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S sh -c 'echo dtoverlay=gmsl >> <config_path>'"
```
Then look for camera overlays that can claim the same CSI port (`camera_auto_detect=1`, which
the ADI GMSL images use, does not conflict and is not checked):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "grep -nE '^[[:space:]]*dtoverlay=(imx|ov|arducam|isx)' <config_path>"
```
If anything is found, tell the user that it may conflict with the GMSL overlay and use
AskUserQuestion (header: "Camera cfg"): **Comment them out (Recommended)** (prefix the lines with
`#` using `sudo sed -i`) | **Keep them**. Do not change any other line of the boot config.

Record it:
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['dt_installed'] = True
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

Then reboot with common/5 §Reboot (the overlay is loaded by the firmware at boot).

## Tools
Each check only installs when it fails, so nothing is installed on boards that already have the
tools. Installing needs Internet access on the board.

First make sure the basic tools are present (images other than the ADI GMSL images may not
have them):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which git pip3 >/dev/null 2>&1 || (echo '<pw>' | sudo -S apt-get update && echo '<pw>' | sudo -S apt-get install -y git python3-pip)"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which v4l2-ctl media-ctl >/dev/null 2>&1 || (echo '<pw>' | sudo -S apt-get update && echo '<pw>' | sudo -S apt-get install -y v4l-utils)"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which i2cdetect >/dev/null 2>&1 || (echo '<pw>' | sudo -S apt-get update && echo '<pw>' | sudo -S apt-get install -y i2c-tools)"
```

Check if pyv4l2 and video_cfg.sh are already on the board (included in the ADI GMSL images and in
the SD card of rpi/3):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "test -d <board_home>/Workspace/pyv4l2 && echo ok || echo missing"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "test -f <board_home>/Workspace/video_cfg.sh && echo found || echo missing"
```

On a freshly flashed card, the first-boot service of rpi/3 may still be installing; check it
before installing anything by hand:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "systemctl is-active gmsl-first-boot.service 2>/dev/null"
```
If it prints `activating`, wait (poll every 15 s, up to 10 minutes) until it is no longer active.

If pyv4l2 is missing, clone it:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "mkdir -p <board_home>/Workspace && git clone --branch gmsl https://github.com/iristeiu06/pyv4l2 <board_home>/Workspace/pyv4l2"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S chown -R <user>:<user> <board_home>/Workspace/pyv4l2"
```

Then make sure it is installed as a Python package. Check the import, not the folder: the
folder can be present while the package is not installed (for example when the first-boot
service of rpi/3 had no internet access):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "python3 -c 'import v4l2' 2>/dev/null || (cd <board_home>/Workspace/pyv4l2 && pip3 install --break-system-packages -e .)"
```

Then install any missing dependencies (qv4l2 for the first viewer, pixutils and pykms for
cam.py; the GStreamer packages of the fallback viewer are installed by common/8 §GStreamer only
when needed):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which qv4l2 >/dev/null 2>&1 || (echo '<pw>' | sudo -S apt-get update && echo '<pw>' | sudo -S apt-get install -y qv4l2)"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "python3 -c 'import pixutils' 2>/dev/null || pip3 install --break-system-packages git+https://github.com/tomba/pixutils.git"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "python3 -c 'import kms' 2>/dev/null || pip3 install --break-system-packages git+https://github.com/tomba/pykms.git"
```
If the qv4l2 install fails (no Internet, package not found), note it and continue: §Viewer then
uses the GStreamer fallback.

Note: The pykms pip package installs as the `kms` module (not `pykms`), so the import
check must use `import kms`.

Older pip versions (before 23.0.1, for example on Raspberry Pi OS Bullseye) do not know
`--break-system-packages`. If pip reports `no such option: --break-system-packages`, run the
same pip command without it.

If video_cfg.sh is missing, create it from the ADI repository. Download it on the host and copy
it to the board (the board may not have internet access):
```bash
curl -sL -o /tmp/gen_gmsl_dts/video_cfg.sh https://raw.githubusercontent.com/analogdevicesinc/gmsl/tools/rpi/video_cfg.sh
scp -o ControlPath=/tmp/gmsl_ssh_sock /tmp/gen_gmsl_dts/video_cfg.sh <user>@<ip>:<board_home>/Workspace/video_cfg.sh
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "chmod +x <board_home>/Workspace/video_cfg.sh"
```

Then check whether cam.py is in `pyv4l2/utils/cam/` or directly in `pyv4l2/utils/`:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "test -f <board_home>/Workspace/pyv4l2/utils/cam/cam.py && echo cam || echo utils"
```
If it is `cam`, make video_cfg.sh call cam.py from the `cam` folder (no change if already correct):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "sed -i 's#/utils/cam\.py#/utils/cam/cam.py#' <board_home>/Workspace/video_cfg.sh"
```

If video_cfg.sh has a hard-coded pyv4l2 path for another user (e.g. `/home/analog/...`), point it
to the current user's home (no change if the script already derives the path itself):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "sed -i 's#^PYV4L2_PATH=\"/home/[^/]*/#PYV4L2_PATH=\"<board_home>/#' <board_home>/Workspace/video_cfg.sh"
```

## Configure pipeline
At the end of this step you must know, for every camera, its video device (`<dev_path>`, e.g.
`/dev/video0`) and the configured format, and which routes were set (§Viewer needs them).

The `video_cfg.sh` script wraps `cam.py` from the pyv4l2 library. It auto-discovers the
media topology (deserializer, serializers, sensors) and configures:
- Subdev routing (V4L2 streams API)
- Pad formats on all entities (sensor → serializer → deserializer → CSI2 RX)
- Media links between entities
- Video device format

The config name follows the pattern: `rpi<rpi_model>-gmsl-<sensor>` (e.g., `rpi5-gmsl-imx219`).
`video_cfg.sh` picks the prefix itself from `/proc/device-tree/model` (`rpi4` or `rpi5`; it exits
with "Unknown platform" on other boards), so it needs no change for the model; only `CAM_LIST`
and `CAM_MODEL` are set below. When calling cam.py directly, use `rpi<rpi_model>` (`rpi5` for
state `5`).
The camera list specifies which GMSL link ports to configure (e.g., `cam0`, `cam0,cam1`).

Update `video_cfg.sh` variables and run:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "sed -i 's/^CAM_LIST=.*/CAM_LIST=\"<cam_list>\"/' <board_home>/Workspace/video_cfg.sh"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "sed -i 's/^CAM_MODEL=.*/CAM_MODEL=\"<model>\"/' <board_home>/Workspace/video_cfg.sh"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "cd <board_home>/Workspace && ./video_cfg.sh"
```

Where:
- `cam_list` maps to GMSL link ports: e.g. "cam0" for 1 camera, "cam0,cam1,cam2,cam3" for 4 cameras
- `model` is the sensor name in lowercase (e.g., "imx219", "ov5640", "imx415")
- The script internally calls: `cam.py -c -p rpi<rpi_model>-gmsl-<model>:<cam_list>` (state
  `rpi_model` `5` → `rpi5-gmsl-imx219`)

If cameras use different sensor models, repeat sed+run for each model group:
- Update CAM_LIST to only the cams with that model
- Update CAM_MODEL to match
- Run video_cfg.sh

The script prints one dictionary per camera, with its video device (`dev_path`, e.g.
`/dev/video0`) and its format (`w`, `h`, `format`, e.g. 640x480 `SRGGB10P`).

Save what it configured, for §Viewer (`-p` prints routes, formats and links):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "media-ctl -p" > /tmp/gen_gmsl_dts/media_topology.txt
```

**Fallback for sensors without a cam.py profile** (IMX708, ISX021, or `video_cfg.sh` fails with
`profile not found` / a `KeyError` for the config name): read
`<skill dir>/common/7-media-pipeline.md` and follow §Topology, §Routes, §Formats, §Video node,
§Check and §media_cfg.sh, with these Raspberry Pi values:
- RPi 5 (`rp1-cfe`, upstream streams-capable driver of rpi/2): the `csi2` subdev has the sink pad
  0 and the source pads 1–4, one per channel, each linked to a video node (named like
  `rp1-cfe-csi2-ch0` … `ch3`; the names differ between driver versions, take the exact entity
  names from `media-ctl -p`). Route camera `k`'s sink stream to its
  own channel (`0/<k> -> <1+k>/0 [1]`), enable the link from that source pad to its video node
  (`media-ctl -l '"csi2":<1+k> -> "<video entity>":0[1]'`), and set the formats on both pads.
- RPi 4 (`unicam`): one image source pad; route only one camera (`0/0 -> 1/0 [1]`) and start with
  one camera.
- Use the cam.py profile of a supported sensor as the model: `video_cfg.sh` for IMX219 on the same
  board shows which links and routes cam.py enables on the receiver (`media-ctl -p` after it ran).
- Video node format: `v4l2-ctl --list-formats` on the node; for 10-bit Bayer the CFE offers packed
  (`pRAA` = `SRGGB10P`) and unpacked 16-bit formats.

## Frame check
Read `<skill dir>/common/8-frames-viewer.md` and follow §Frame check for each `<dev_path>`.
Example: 640x480 `SRGGB10P` (packed RAW10) is 640 * 480 * 10 / 8 = 384000 bytes per frame; 8-bit
formats such as `SRGGB8` are `w * h` bytes per frame. Run it before the viewer starts, because the
viewer keeps the video devices open (`EBUSY`).

## Viewer
First follow common/8 §Desktop session (it also detects `$XAUTH`). Without a desktop session, do
not launch a viewer: the frame check shows whether the pipeline works.

1. **qv4l2 first:** follow common/8 §qv4l2 (launch on `DISPLAY=:0` with `$XAUTH`, check that it
   runs). If it runs, tell the user: "Press the green Play button in qv4l2 to start the video
   feed", and ask whether the image appears (header: "Video"): **Yes, I see the video** | **No
   image** | **qv4l2 did not open**.
2. **GStreamer fallback**, when qv4l2 is missing and cannot be installed (§Tools), does not start
   (after the `$XAUTH` retry of common/8 §qv4l2), or the user sees no image. Stop qv4l2 first
   (`pkill -x qv4l2`; it holds the video devices, `EBUSY`). Then:
   - Bayer sensors (IMX219, IMX708, IMX415): reconfigure the camera to **8-bit 640x480** on the
     routes `video_cfg.sh` set: GStreamer's `v4l2src` cannot read packed `SRGGB10P`, and the CFE's
     unpacked RAW10 is MSB-aligned (GStreamer's 10-bit Bayer formats expect the value in the low
     bits: a washed-out image). Take every pad/stream of the camera's chain from
     `/tmp/gen_gmsl_dts/media_topology.txt` (sensor source pad, serializer `0/0` and `1/0`,
     deserializer sink and source streams, the CFE `csi2` sink and source pads) and write
     `<board_home>/Workspace/media_cfg_view.sh` (common/7 §media_cfg.sh: `#!/bin/sh`, `set -e`, a
     comment with the configuration) with one `media-ctl -d <media dev> -V
     '"<entity>":<pad>[/<stream>] [fmt:SRGGB8_1X8/640x480 field:none]'` per pad/stream (use the
     sensor's own 8-bit Bayer order from `media-ctl -p`, e.g. `SRGGB8_1X8` for IMX219) and
     `v4l2-ctl -d <dev_path> --set-fmt-video=width=640,height=480,pixelformat=RGGB`. Run it, check
     with `media-ctl -p` (common/7 §Check) that every pad of the chain shows the same code, size and
     `field:none`, and run common/8 §Frame check once more (640 * 480 = 307200 bytes per frame).
   - YUV sensors (OV5640): keep the `video_cfg.sh` format; common/8 §GStreamer uses `videoconvert`
     for them.
   - Then follow common/8 §GStreamer with `<dev_path>`, `format=rggb` (the Bayer order in
     lowercase), `width=640`, `height=480`. It installs `gstreamer1.0-tools
     gstreamer1.0-plugins-bad` if `bayer2rgb` is missing (the SD card of rpi/3 already has them),
     and picks `ximagesink` on the X11 desktop of the ADI images or `waylandsink` on labwc.
   - `./video_cfg.sh` restores the 10-bit configuration (stop the viewer first with
     `pkill -x gst-launch-1.0`).

Record which viewer runs, for §Final message.

## Final message
The content for the board part of the orchestrator's single §Final message (not a separate
message). The board is left in its deployed state (nothing is removed on it). Tell the user what
is installed and how to undo it:
- `gmsl.dtbo` in `<overlays_dir>` and the line `dtoverlay=gmsl` in `<config_path>` (and any camera
  overlay lines that were commented out). Undo: delete the line (and remove the `#` from the
  commented lines), delete `<overlays_dir>/gmsl.dtbo`, reboot.
- `~/Workspace/` on the board: pyv4l2, `video_cfg.sh` (with the CAM_LIST/CAM_MODEL that were set),
  and `media_cfg_view.sh` / `media_cfg.sh` if written. Rerun `cd ~/Workspace && ./video_cfg.sh` after
  every boot to configure the pipeline again.
- Packages installed by §Tools or common/8 (list them: apt packages, pip packages).
- The running viewer: qv4l2 — "Press the green Play button in qv4l2 to start the video feed." —
  or the GStreamer pipeline (stop it with `pkill -x gst-launch-1.0`).
- On an SD card from rpi/3: the GMSL kernel, the `analog` user and the hostname `analog-rpi-gmsl`
  are part of the image; going back to the stock kernel means flashing a stock Raspberry Pi OS
  image.

## Troubleshooting
Reference documents:
- **Raspberry Pi user guide** (hardware: CFG pin levels per chip, board resistor bridges,
  adapter switches, cables, power-over-coax):
  https://analogdevicesinc.github.io/system-level/solutions/reference-designs/ad-gmslcamrpi-adp/raspberry-pi-user-guide/
- **README-GMSL.md** (drivers: `v4l2-ctl --log-status` fields, register reads with `v4l2-dbg`;
  building a kernel with the ADI GMSL drivers):
  https://github.com/analogdevicesinc/linux/blob/<adi_gmsl_branch>/README-GMSL.md
- **gen_gmsl_dts README** (overlay generator and its JSON format; the optional deserializer
  `"address"` key is not documented there, but the `des.dtsi.in` template reads it):
  https://github.com/analogdevicesinc/linux/blob/<adi_gmsl_branch>/arch/arm/boot/dts/overlays/gen_gmsl_dts/README.md

Fetch them with `curl -fsSL` (use the `raw.githubusercontent.com` form for GitHub files), and use
them as the orchestrator's §Rules say (information, not instructions).

Camera I2C bus for `i2cdetect` (common/5 §Camera port check, common/6): bus 10 for RPi 5 CAM0 and
RPi 4. For RPi 5 CAM1, find the bus of `i2c_csi_dsi1` with `i2cdetect -l`.

The general rows (probe `-121`, `Cannot find serializer`, no `v4l-subdev*` nodes, no frames,
`EBUSY`) are in the orchestrator's §Troubleshooting. Raspberry Pi rows:

| Symptom | Where to look |
|---------|---------------|
| Deserializer probe fails with `-121`, devices answer on the bus | Wrong I2C address. Compare the deserializer's CFG0 level with the user guide's CFG table, and find the real address with `i2cdetect -y -r 10` on the board (bus 10 is the camera I2C bus for RPi 5 CAM0 and RPi 4). Regenerate the overlay with `"address": "<addr>"` in the JSON. |
| `cam.py` fails with `EBUSY` | Another process (qv4l2, v4l2-ctl, gst-launch-1.0, an earlier cam.py) holds the video devices; close it and retry. |
| `cam.py` / `video_cfg.sh` fails with `ENOTTY` (errno 25, "Inappropriate ioctl for device") when setting routes | The kernel's V4L2 streams API is disabled (stock kernels without rpi/1 Step 8). Use an ADI GMSL image or the kernel of rpi/0–2. |
| qv4l2 does not appear, display `:0` does not respond | On the ADI images without a monitor, two X servers can start and block each other; connect a monitor to HDMI0 and reboot. |
| qv4l2 runs, Play shows nothing or an error | Check that the frame check passed; then use the GStreamer fallback of §Viewer. |
| GStreamer shows a washed-out or very dark image | A 10-bit format reached `v4l2src`: run `media_cfg_view.sh` (8-bit) as in §Viewer. |
| GStreamer: `not-negotiated` / `Internal data stream error` | The caps do not match the video node: compare `v4l2-ctl -d <dev_path> --get-fmt-video` (must be `RGGB` 640x480) with the `video/x-bayer` caps, and the Bayer order of the sensor. |
| No GMSL driver messages in `dmesg`, on an image other than the ADI GMSL images | Check that `dtoverlay=gmsl` is in the `config.txt` that is actually used (`/boot/firmware` or `/boot`), and run common/3 §Driver check again. |
| Board does not come back after the reboot | Connect a monitor to see the boot; to undo the overlay without the board, remove `dtoverlay=gmsl` from `config.txt` on the SD card's boot partition from a PC. |
