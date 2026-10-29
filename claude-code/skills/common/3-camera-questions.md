# Common 3: Camera questions and checks

The questions about the GMSL hardware, and the checks that can run before any device tree is
installed. The platform file says which sections to run and gives the option lists (its
**Hardware choices** section: rpi/4 §Hardware choices, nxp/5 §Hardware choices,
generic/1 §Hardware choices). Where this file says "the platform file's option list", use that
list; where the platform file has none, use the defaults below.

Two ways to run it:
- **Live board** (Yes path; started by the orchestrator for `rpi` and `nxp`, by generic/1 §Order
  for `generic`, once per deployment): every section, in order. The board is reachable
  (`board_ip`, ...). When `hw` is already in the state, the orchestrator confirms the stored
  answers with one question instead of asking §Hardware questions and §Per-camera questions again.
- **Offline** (NXP "No" path, before the image is built: orchestrator §Flow): only
  §Hardware questions and §Per-camera questions. §EV kit checks, §I2C probe, §Path check and
  §Driver check run later on the booted board (orchestrator Yes path, step 3), which confirms the
  answers already stored (`hw` in the state) instead of asking them again.

The board's physical setup can only be answered by the user, who has it within reach. Never
assume the hardware matches another kit's guide.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save every answer as an `ANSWER` line and `<run_dir>/choices.json` once the hardware answers are complete (§Verbose mode §What to save).

## Hardware questions

Ask in a single AskUserQuestion call (2–4 questions). Skip a question whose answer is already in
the state (`hw`, `rpi_model`) or in the user's prompt.

- **Deserializer** (header: "Deserializer"): the platform file's option list. Default:
  MAX96724 (4-link) | MAX9296A (2-link) | MAX96716A (2-link) | MAX96712 (4-link). Also available
  but less common: MAX96724F (1-link), MAX96714 (1-link, generic only), MAX96792A (1-link GMSL3) —
  put these in "Other".
- **Camera count** (header: "Cam Count"): 1 / 2 / 3 / 4 (max depends on the deserializer:
  MAX96724/MAX96712=4, MAX9296A/MAX96716A=2, MAX96724F/MAX96714/MAX96792A=1). On a board without
  a dedicated platform file (generic), and on an NXP board, recommend 1 for the first bring-up.
- **Camera connector** (header: "Connector" — the RPi platform file names it "Camera Port"):
  the platform file's option list (RPi 5: CAM0 | CAM1 | Not sure; RPi 4: skip; NXP: the board's
  camera connectors from nxp/2 §Values; generic: the connectors found in the assessment, see
  generic/1 §Hardware choices), plus "Not sure". Ask which connector the deserializer board's
  cable is plugged into, as labeled on the board (say this in the question).
- **Deserializer adapter** (header: "Adapter"), when the platform file asks for it (NXP and
  generic; on a Raspberry Pi the adapter is ADI's AD-GMSLCAMRPI-ADP and the platform file's PHY
  table applies): how the deserializer's CSI-2 output reaches the board. Explain in the question
  that the deserializer has two CSI-2 output ports (PHY A / PHY B) and the device tree must name
  the one that is wired; users do not know adapter names. Options:
  **AD-GMSLCAMRPI-ADP, cable on P6 ("CAM 2")** (PHY B; ADI's Raspberry Pi setup) |
  **AD-GMSLCAMRPI-ADP, cable on P8 ("CAM 1")** (PHY A) |
  **Other adapter** (ask which deserializer CSI-2 port is wired to the board) |
  **Not sure / not connected yet** (use the Raspberry Pi value, and say which PHY to try if no
  frames arrive). It decides the deserializer PHY (common/4 §JSON, the platform file's PHY table).

Apply the platform file's warnings (untested or unsupported combinations).

## Per-camera questions

For each camera, 2 questions (batch up to 4 questions per AskUserQuestion call, i.e. two cameras
per call):
- **Serializer for Camera N** (header: "Serializer"): MAX96717 (GMSL2) | MAX9295A (GMSL2) |
  MAX96793 (GMSL3)
- **Camera Sensor for Camera N** (header: "Sensor"): the platform file's option list. Default:
  **IMX219 (fully supported)** | **OV5640 (fully supported)** | **IMX415 (fully supported)** |
  **IMX708 (device tree overlay only)**. Also available: **ISX021 (device tree overlay only)**
  (in "Other"). Put the support level in each option's description (§Sensor support levels). On
  a live board, the platform file also warns about sensors whose driver the assessment did not
  find (generic) or that NXP's kernel lacks (nxp/5).

### Sensor support levels

All five sensors have a gen_gmsl_dts template (common/4). They differ in how far the setup goes:

| Level | Sensors | What the skill does |
|-------|---------|---------------------|
| **Fully supported** | IMX219, OV5640, IMX415 | Device tree overlay, a ready media pipeline configuration (on the Raspberry Pi, ADI's `cam.py` profiles for `video_cfg.sh`), frame check and live viewer |
| **Device tree overlay only** | IMX708, ISX021 | Device tree overlay; the media pipeline is derived from the live topology (common/7) and the frame check and viewer are best effort, with no ready configuration |

When the user picks an overlay-only sensor, tell them once, before continuing: the device tree is
generated and installed as for the others, but the pipeline configuration, frames and live view
are not guaranteed (no ready configuration). Store the level per camera as `hw.links[].support`
(`full` or `overlay`), and list it in the final message next to the sensor.

Store the answers in the state (`hw`; `links` in camera order, Camera 1 = link 0):
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['hw'] = {
  'deserializer': '<max96724|max9296a|max96716a|max96712|max96724f|max96714|max96792a>',
  'cam_count': <n>,
  'connector': '<CAM0|CAM1|connector label|not sure>',
  'adapter_phy': '<phy index, or empty>',
  'links': [{'serializer': '<max96717|max9295a|max96793>', 'sensor': '<imx219|...>', 'support': '<full|overlay>'}]
}
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
Use the chip names in lowercase, as gen_gmsl_dts expects them (common/4 §JSON).

## EV kit checks

Before the first I2C access, check the evaluation kits (tell the user what to look at; never
assume the hardware matches another kit's guide). Ask in one AskUserQuestion call (header:
"EV kit") whether the LEDs below are as expected, with **All as described** | **Something
differs** (the user describes it in "Other"):
- **On-board microcontroller:** deserializer EV kits have a µC (a Teensy on the MAX96716/MAX96792
  kits) wired to the DUT I2C through jumpers: `RX_SDA`/`TX_SCL` (default `TNZ_SDA`/`TNZ_SCL`) on
  MAX96716/MAX96792 kits, J18/J19 on MAX96724 kits. **These jumpers stay fitted in their default
  position**: ADI's Raspberry Pi setup works with them fitted on the MAX96716 kit, and so did an
  NXP i.MX95 board once its camera bus ran at 100 kHz (common/4 §Bus speed). ADI's Raspberry Pi
  guide removes J18/J19 on the MAX96724 kit; follow that guide for that kit on a Raspberry Pi.
  Suggest removing them only if the I2C check (common/6) still fails at 100 kHz and the user
  agrees to test it (power off first): removing them disconnects the µC, so ADI's PC
  configuration software cannot reach the deserializer any more; the CFG digital potentiometers
  keep their values (checked on a MAX96716 kit). Never change jumpers yourself: tell the user what
  to change and why, and continue after they confirm.
- **Serializer power:** compact serializer boards (for example AD-GMSL717MIPI-EVK) are powered
  only over the coax: the deserializer kit must feed power over coax on the input used (MAX96716
  kit: J14 = VSUP to the PoC switch, J11 = coax A, J10 = coax B). The serializer board's PGOOD LED
  shows it; the deserializer kit's LOCK LED shows the link. Read the CFG resistors from the
  serializer board's schematic instead of asking for jumper settings it does not have.
- **Power order:** power the kits before (or together with) the board: an unpowered kit can clamp
  the I2C lines of the bus it is connected to.

## I2C probe

Find the I2C bus of the deserializer (`<i2c_bus>`, as a bus number on the board). On a Raspberry
Pi the platform file gives the bus (rpi/4 §Troubleshooting: bus 10 for RPi 5 CAM0 and RPi 4; for
CAM1 the bus of `i2c_csi_dsi1` from `i2cdetect -l`); on an NXP board nxp/2 §Values gives it for
the FRDM-IMX95-PRO; otherwise the candidates come from the device tree.

Ask the user first (header: "I2C probe"): probing reads one address on each candidate bus, which
is normally harmless, but on unknown hardware it is still bus traffic:
**Probe (Recommended)** | **Skip** (use the bus the platform file or the vendor DT associates with
the chosen connector). On a Raspberry Pi with the ADI image, probing only the platform's camera
bus needs no question.

Candidate buses: the bus of the platform file for the chosen connector first; otherwise the buses
of the vendor camera nodes for that connector (from `running.dts` and the camera DTBs/overlays in
`<tpl_dir>/dtbs`), then every bus in the report's `i2c` section. Never scan a whole bus that is
shared with power-critical devices (USB-PD controllers such as `ptn5110`/`tcpci`, PMICs, power
switches, see the report's `i2c_devices`): probe single addresses only. Probe only the default
address of the chosen deserializer (from `<tpl_dir>/<chip>.dtsi.in`, `default_address`, once the
templates are downloaded: common/4 §Templates): MAX96724/MAX96724F `0x27`, MAX96712 `0x29`,
MAX9296A `0x48`, MAX96716A `0x2a`, MAX96714/MAX96792A `0x4c`:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "i2cdetect -y -r <bus> <addr> <addr>"
```
(use the control socket of common/5 §Control socket if it is open, or plain
`sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- "..."` before it exists;
`i2cdetect` is in `i2c-tools`; if it is missing, the platform file's **Tools** section installs
it, after asking). Root rights: as a normal user, run it with `echo '<pw>' | sudo -S sh -c '...'`.

- A device at the address on exactly one bus: that is `<i2c_bus>`.
- A device at the address on more than one bus: tell the user which buses, and use the one the
  platform file or vendor DT associates with the chosen connector; ask if that is ambiguous.
- `UU` at the address: a driver is already bound to that address (a GMSL driver from an earlier
  deployment, or a vendor driver): note it, and check it in §Driver check and the report's
  `i2c_devices`.
- If nothing answers anywhere, the deserializer may be unpowered, held in power-down by a SoC
  GPIO (the vendor camera nodes show `powerdown-gpios` / `reset-gpios`), behind an I2C mux the
  device tree has not enabled, or at another address (CFG pins), or the cable is on another
  connector: ask the user to check the power and the CFG levels, and use the bus from the platform
  file or the vendor DT. Do not suspect I2C addresses while nothing at all answers on the bus.

## Path check

Read-only, works without the ADI drivers. While no GMSL driver is bound, the deserializer passes
I2C through to the link, so the serializer answers at its own address on `<i2c_bus>`
(`i2cdetect -y -r <bus> 0x40 0x40` for a MAX96717 at CFG0 = 0) when the LOCK LED is on. Run it
for each camera's serializer (they share the default address until the drivers assign aliases:
with more than one camera, one answer only shows that at least one link is up).

The sensor answers only when its enable line (often a serializer MFP) is high: say so instead of
reporting a fault. The ADI sensor templates use `reset_pin` (default 0, i.e. MFP0) for it; on the
AD-GMSL717MIPI-EVK, MFP0 is the Raspberry Pi camera's enable (CAM_GPIO, connector P6 pin 5). For
other serializer boards, read it from their schematic. If the user agrees, that MFP can be raised
for a test with one volatile write (MAX96717: `GPIO_A(n)` = `0x2be + 3n`, clear bit 0
`GPIO_OUT_DIS` with `GPIO_OUT` bit 4 set, as ADI's `max96717.c` does for an output-high), then
read the sensor's chip ID through the same pass-through (for example IMX219 at `0x10`, register
`0x0000`, which reads `0x02 0x19`). This proves the cable, the link and the addresses before any
kernel work. The write is lost at the next power cycle of the serializer.

Skip this check when a GMSL driver is already bound to the deserializer (`UU` in §I2C probe): the
pass-through then belongs to the driver.

## Driver check

The device tree only works if the board's kernel has the ADI GMSL drivers (from
https://github.com/analogdevicesinc/linux, included in the ADI GMSL images; NXP's BSP and most
other images do not have them). Driver for each chip:
- Deserializer: MAX96724, MAX96724F, MAX96712 → `max96724`; MAX9296A, MAX96716A, MAX96792A →
  `max9296a`; MAX96714 → `max9296a` (generic)
- Serializer: MAX96717, MAX9295A, MAX96793 → `max96717`
- All of them also need the common `max-serdes` driver
- Sensor: the sensor name in lowercase (`imx219`, `ov5640`, `imx415`, `imx708`, `isx021`), or what
  the platform file's **Drivers** section says

Check each driver needed for the chosen hardware, as a loadable module or built into the kernel:
```bash
sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 <user>@<ip> -- '
K=/lib/modules/$(uname -r); echo "kernel: $(uname -r)"; . /etc/os-release; echo "os: $PRETTY_NAME"
for m in max-serdes <deserializer_driver> <serializer_driver> <sensor_drivers>; do
  u=$(echo $m | tr - _)
  if find $K \( -name "$m.ko*" -o -name "$u.ko*" \) 2>/dev/null | grep -q .; then echo "$m: module"
  elif grep -qE "/($m|$u)\.ko" $K/modules.builtin 2>/dev/null; then echo "$m: built-in"
  else echo "$m: MISSING"; fi
done'
```
(on Windows, use common/0 §Windows adaptations instead of `sshpass`; through the control socket,
`ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- '...'`).

A module with the right name is not always the ADI driver: vendor kernels can ship their own
`max96724`/`max96717` drivers (NXP: `max96724`, `max96717_lib`), which are not compatible with the
ADI device tree. Check that the found modules belong to the ADI drivers: their `depends`
(`modinfo -F depends <module>`, or the assessment's `gmsl_drivers` section) contains `max-serdes`
(or `max_serdes`), or they are built in together with a built-in `max-serdes`. A found
`max96724` without that dependency counts as MISSING here, and its name must not be reused
(common/9 §ADI drivers).

- **All found:** store `drivers_ok` = `true` and continue with the platform file.
- **`max-serdes`, the deserializer or the serializer driver missing:** the kernel does not have
  the ADI GMSL drivers (it may have other GMSL drivers, which these skills do not support). Tell
  the user, then use AskUserQuestion (header: "Drivers") with the platform's way forward (the
  orchestrator's §Flow, "Drivers missing on a board on the network"):
  - `rpi`: **Build the ADI drivers for this kernel (Recommended)** (the orchestrator's "Drivers
    missing on a board on the network", Raspberry Pi: modules built on the host for the running
    kernel and installed on the board; the board is never reflashed; WSL or Linux on the host) |
    **Continue anyway** (the device tree is installed, but the cameras will not probe) |
    **Stop**. This is the only driver question on a Raspberry Pi (rpi/4 does not ask it again).
    If the modules cannot carry what the kernel lacks, the orchestrator says that a GMSL kernel on
    a separate SD card is needed (a new run with Q1 **No**).
  - `nxp`: **Build the ADI drivers for this kernel (Recommended)** (nxp/1 §Live board: modules
    built on the host against NXP's kernel source and installed to `updates/gmsl/`, no reflash;
    about 30–60 minutes) | **Continue anyway** | **Stop**.
  - `generic`: **Build the ADI drivers for this kernel (Recommended)** (generic/0 §Best-effort
    plan, common/9) | **Continue anyway** | **Stop**.
  Building needs WSL or Linux on the host (common/0 §Host OS).
- **Only the sensor driver missing:** warn the user that the camera will not probe, and ask the
  same question; on `nxp` and `generic` the build option builds the sensor driver too
  (common/9 §Sensor driver), on `rpi` the ADI kernel of the flash path includes it.

After a successful build and install, run this check again: it must now find every driver. If
the user stops, run the orchestrator's §Cleanup. Store the result:
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['drivers_ok'] = <True|False>
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
