# Skills

## gmsl-camera-setup

TRIGGER when: user mentions GMSL camera setup, camera configuration for Raspberry Pi,
deploying camera overlay to RPi, video feed from GMSL camera, configuring deserializer/serializer
on Raspberry Pi, or asks about MAX96724/MAX96716/MAX9296A/MAX96712/MAX96792A camera setup.

When this skill is triggered, use AskUserQuestion to prompt the user interactively.
If the user asks for the GUI instead, generate the full GUI application as a Python tkinter
script, write it to `/tmp/gmsl_setup.py`, and launch it with `python3 /tmp/gmsl_setup.py`.
The GUI must include all the fields, mappings, deployment logic, and rules described below.
Do NOT reference or launch `~/gmsl_setup.py` — always generate the code fresh.

### Step 1: Prompt for hardware choices (single AskUserQuestion, 3 questions)
- **RPi Model** (header: "RPi Model"): Raspberry Pi 5 (4 MIPI lanes) | Raspberry Pi 4 (2 MIPI lanes)
- **Deserializer** (header: "Deserializer"): MAX96724 (4-link) | MAX9296A (2-link) | MAX96716 (2-link) | MAX96712 (4-link). Also available but less common: MAX96724F (1-link), MAX96792A (1-link GMSL3) — put these in "Other".
- **Camera count** (header: "Cam Count"): 1 / 2 / 3 / 4 (max depends on deserializer)

### Step 2: Prompt for per-camera and port config (single AskUserQuestion, 2–3 questions)
- **Camera Port** (header: "Camera Port", RPi 5 only): CAM0 | CAM1. Skip for RPi 4.
- **Serializer for Camera N** (header: "Serializer"): MAX96717 (GMSL2) | MAX9295A (GMSL2) | MAX96793 (GMSL3)
- **Camera Sensor for Camera N** (header: "Sensor"): IMX219 | OV5640 | IMX708 | IMX415. Also available: ISX021.
  Note: video_cfg.sh only officially supports IMX219 and OV5640 as CAM_MODEL. Other sensors
  will get a valid DTS overlay but the video pipeline configuration (cam.py) may not have
  profiles for them — warn the user if they pick a different sensor.
- If multiple cameras, repeat serializer/sensor questions for each camera (batch up to 4 questions per AskUserQuestion call).

### Step 3: Prompt for connection details (single AskUserQuestion, 2 questions)
- **IP Address** (header: "IP Address"): Offer common subnet examples, user types actual IP in "Other".
- **Credentials** (header: "Credentials"): "analog / analog" (default) | Custom (user types user:pass in "Other").

### Step 4: Deploy directly from the terminal

After collecting all choices, execute deployment as follows:

#### 4a. Generate JSON config
Write the JSON to `/tmp/gen_gmsl_dts/gmsl_config.json`. Structure:
```json
[{
    "name": "<deserializer_name>",
    "i2c_bus": "<i2c_bus>",
    "platform_cfg": { "name": "<rpi_name>", "csi_idx": <idx>, "phy_idx": <idx> },
    "phys": [{ "phy_idx": <idx>, "num_lanes": <2|4>, "link_frequencies": [750000000],
               "clock_lanes": [0], "data_lanes": [1,2] or [1,2,3,4] }],
    "links": [{ "name": "<serializer>", "cameras": [{"name": "<sensor>"}],
                "pool_addrs": ["0x50","0x51"] }, ...]
}]
```

Parameter mapping:
- RPi 4: name="rpi-4-b", i2c_bus="i2c_csi_dsi", csi_idx=1, num_lanes=2, data_lanes=[1,2]
- RPi 5 CAM0: name="rpi-5-b", i2c_bus="i2c_csi_dsi0", csi_idx=0, num_lanes=4, data_lanes=[1,2,3,4]
- RPi 5 CAM1: name="rpi-5-b", i2c_bus="i2c_csi_dsi1", csi_idx=1, num_lanes=4, data_lanes=[1,2,3,4]
- Deserializer phy_idx: max9296a=1, max96712=2, max96716a=1, max96724=2, max96724f=2, max96792a=1
- Pool addrs per link: link0=["0x50","0x51"], link1=["0x52","0x53"], link2=["0x54","0x55"], link3=["0x56","0x57"]

#### 4b. Download gen_gmsl_dts templates (if not cached)
```bash
cd /tmp/gen_gmsl_dts && for f in gen_gmsl_dts.py des.dtsi.in ser.dtsi.in \
  max9296a.dtsi.in max96712.dtsi.in max96714.dtsi.in max96716a.dtsi.in \
  max96724.dtsi.in max96724f.dtsi.in max96792a.dtsi.in max96717.dtsi.in \
  max9295a.dtsi.in max96793.dtsi.in imx219.dtsi.in ov5640.dtsi.in \
  imx415.dtsi.in imx708.dtsi.in isx021.dtsi.in max20087.dtsi.in \
  rpi-4-b.dtsi.in rpi-5-b.dtsi.in; do
  [ ! -f "$f" ] && curl -sLO "https://raw.githubusercontent.com/analogdevicesinc/linux/gmsl/rpi-6.13.y/arch/arm/boot/dts/overlays/gen_gmsl_dts/$f"
done
```

#### 4c. Generate DTS overlay
```bash
cd /tmp/gen_gmsl_dts && python3 gen_gmsl_dts.py --dtbo -o gmsl-overlay.dts gmsl_config.json
```

#### 4d. Compile with dtc
```bash
dtc -@ -I dts -O dtb -o /tmp/gen_gmsl_dts/gmsl.dtbo /tmp/gen_gmsl_dts/gmsl-overlay.dts
```
Warnings from dtc are expected and harmless.

#### 4e. Connect to board via SSH control socket
First close any stale socket and remove the socket file, then open a new one:
```bash
ssh -S /tmp/gmsl_ssh_sock -O exit <user>@<ip> 2>/dev/null
rm -f /tmp/gmsl_ssh_sock
sleep 0.5
sshpass -p '<password>' ssh -M -S /tmp/gmsl_ssh_sock -fN \
  -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 <user>@<ip>
```
If connection fails, use AskUserQuestion to ask user to retry, change IP, or stop.

#### 4f. Copy overlay and install
```bash
scp -o ControlPath=/tmp/gmsl_ssh_sock /tmp/gen_gmsl_dts/gmsl.dtbo <user>@<ip>:/home/<user>/gmsl.dtbo
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<password>' | sudo -S cp /home/<user>/gmsl.dtbo /boot/overlays/"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "rm -f /home/<user>/gmsl.dtbo"
```

#### 4g. Update boot config
Detect config path: `test -f /boot/firmware/config.txt && echo fw || echo boot`
Then:
```bash
ssh ... -- "echo '<pw>' | sudo -S sed -i '/^dtoverlay=gmsl/d' <config_path>"
ssh ... -- "echo '<pw>' | sudo -S sh -c 'echo dtoverlay=gmsl >> <config_path>'"
```

#### 4h. Reboot and wait
```bash
ssh ... -- "echo '<pw>' | sudo -S reboot"
ssh -S /tmp/gmsl_ssh_sock -O exit <user>@<ip>
```
Then sleep 30, poll with `sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 <user>@<ip> echo ok` every 5s until success (max 40 attempts). After online, sleep 5 more, then re-establish the control socket (step 4e).

#### 4i. Run video_cfg.sh
Update variables and run:
```bash
ssh ... -- "sed -i 's/^CAM_LIST=.*/CAM_LIST=\"<cam_list>\"/' /home/<user>/Workspace/video_cfg.sh"
ssh ... -- "sed -i 's/^CAM_MODEL=.*/CAM_MODEL=\"<model>\"/' /home/<user>/Workspace/video_cfg.sh"
ssh ... -- "cd /home/<user>/Workspace && ./video_cfg.sh"
```
Where cam_list is e.g. "cam0" or "cam0,cam1,cam2,cam3" and model is the sensor name.
If cameras use different sensor models, repeat sed+run for each model group (update CAM_LIST to only the cams with that model, update CAM_MODEL, run).

#### 4j. Launch video viewer
```bash
ssh ... -- "DISPLAY=:0 XAUTHORITY=/home/<user>/.Xauthority nohup qv4l2 > /dev/null 2>&1 &"
```

#### 4k. Cleanup
```bash
ssh -S /tmp/gmsl_ssh_sock -O exit <user>@<ip>
```
Tell user: "Press the green Play button in qv4l2 to start the video feed."

### Rules
- Video feed appears only on the board (qv4l2 on DISPLAY=:0)
- Disregard existing board configuration
- Ignore that MAX9296A appears as the deserializer (the max9296a.c driver handles variants)
- Do not create folders or backups on the board
- Do not install packages/libraries on the board
- Use sshpass for SSH; pipe password via echo for sudo commands over ssh -S
- All ssh commands through the control socket use: `ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "<cmd>"`
- All sudo commands use: `echo '<pw>' | sudo -S sh -c '<cmd>'`

### GUI Application
When the user requests the GUI, generate the full Python tkinter application on the spot.
The GUI must contain:
- **Board Connection section**: Username (default "analog"), IP Address, Password (default "analog")
- **Hardware Setup section**: RPi Model combobox, Camera Port combobox (disabled for RPi 4),
  Deserializer combobox, Number of Cameras spinbox (max capped by deserializer max_links)
- **Per-Camera Configuration section**: Dynamically rebuilt when camera count changes.
  Each camera gets a Serializer combobox and Camera Sensor combobox.
- **Deploy and Start Video button**: Runs the full deployment pipeline (steps 4a-4k) in a
  background thread, logging progress to a scrolled text widget.
- On startup, download gen_gmsl_dts templates from GitHub to `/tmp/gen_gmsl_dts/` in a
  background thread (skip files already cached).

The GUI deployment logic must follow steps 4a through 4k exactly, including:
- SSH control socket at `/tmp/gmsl_ssh_sock` established with sshpass
- All sudo commands piped with `echo '<pw>' | sudo -S sh -c '<cmd>'`
- video_cfg.sh modified with sed for CAM_LIST and CAM_MODEL, then run
- qv4l2 launched on DISPLAY=:0

Use the same data tables (DESERIALIZERS, SERIALIZERS, CAMERAS, POOL_ADDRS, RPI_MODELS)
and parameter mapping defined in this skill file.

Write the generated script to `/tmp/gmsl_setup.py` and launch it with:
```bash
python3 /tmp/gmsl_setup.py &
```
