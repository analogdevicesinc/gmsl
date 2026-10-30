# Common 4: GMSL device tree (JSON, templates, overlay, glue, merged DTB)

Builds the GMSL device tree overlay on the host: the JSON configuration, ADI's `gen_gmsl_dts`
templates, the generated overlay, and its compiled `.dtbo`; for boards without a platform
template also the receiver glue, its validation against the base device tree, and a merged DTB.
Nothing here changes the board: the platform file installs the result.

There are two variants. The platform file says which one applies and gives its values:
- **With `platform_cfg`** (Raspberry Pi): ADI has a platform template (`rpi-4-b.dtsi.in`,
  `rpi-5-b.dtsi.in`) that connects the deserializer to the receiver. The JSON has a
  `platform_cfg` object, there is no glue, and §Glue and §Validate are skipped. The platform file
  (rpi/4 §JSON mapping) gives the whole mapping.
- **With glue** (NXP i.MX, generic boards): no platform template exists. The JSON has **no
  `platform_cfg`**, and a glue fragment written from the board's own device tree connects the
  deserializer to the receiver (§Glue), validated with `fdtoverlay` (§Validate). The board boots a
  merged DTB (§Merged DTB) or applies the overlay at runtime (generic/1 §Install device tree).

Inputs: the hardware answers of common/3 (`hw` in `<state>`), and for the glue variant the
board's device tree files kept by common/2 in `<tpl_dir>` (`running.dtb` / `running.dts`, the
vendor DTBs in `<tpl_dir>/dtbs/`, `assess_report.txt`) or, without a live board, the DTBs read
from the image by nxp/0.

## JSON

Create the template folder:
```bash
mkdir -p <tpl_dir>
```

Write the JSON to `<tpl_dir>/gmsl_config.json` (gen_gmsl_dts.py loads the templates from the
folder of the JSON file, so it must be next to them). Structure:
```json
[{
    "name": "<deserializer_name>",
    "i2c_bus": "<i2c_bus>",
    "platform_cfg": { "name": "<platform_name>", "csi_idx": <idx>, "phy_idx": <idx> },
    "pool_addrs": ["0x60", "0x61", "0x62", "0x63"],      (optional: see below)
    "phys": [{ "phy_idx": <idx>, "num_lanes": <2|4>, "link_frequencies": [750000000],
               "clock_lanes": [0], "data_lanes": [1,2] or [1,2,3,4] }],
    "links": [{ "name": "<serializer>", "cameras": [{"name": "<sensor>"}],
                "pool_addrs": ["0x50","0x51"] }, ...]
}]
```
- One entry in `links` per camera, in camera order (Camera 1 = link 0).
- `<deserializer_name>` and `<serializer>`: the chip names in lowercase (`max96724`,
  `max96717`, ...).
- `<sensor>`: the lowercase sensor name (`imx219`, `ov5640`, `imx415`, `imx708`, `isx021`;
  mainline bindings, e.g. `sony,imx219`).
- `platform_cfg`: **only in the `platform_cfg` variant** (values from the platform file). Leave
  it out entirely in the glue variant.
- Top-level `pool_addrs` (the deserializer's own alias pool) is **optional**: leave the line out
  unless the template's default `["0x40", "0x41", "0x42", "0x43"]` collides with a device on
  `<i2c_bus>` (§Alias pools); the value above is only an example of a free pool. The same for an
  optional `"address"` (the deserializer's I2C address when it is not the chip's default, read by
  `des.dtsi.in`): only when needed, see the troubleshooting rows of the orchestrator.
- Pool addrs per link (defaults, free on a Raspberry Pi camera bus): link0=["0x50","0x51"],
  link1=["0x52","0x53"], link2=["0x54","0x55"], link3=["0x56","0x57"]. On other boards choose
  them as in §Alias pools.
- Everything else (`i2c_bus`, `platform_cfg`, PHY index, lanes): the platform file's
  **JSON mapping**, or, in the glue variant, the rules below.

Glue-variant mapping (the platform file can fix some of these values, e.g. nxp/2 §JSON for the
FRDM-IMX95-PRO):
- `i2c_bus`: the label of the I2C bus node if it is in `__symbols__` of `running.dts`, otherwise
  its path in braces, e.g. `"i2c_bus": "{/soc/i2c@30a40000}"` (the template writes `&` before it,
  and `&{/path}` targets a node by path, so the base DTB needs no labels). The node must exist
  in the base DTB: vendor camera overlays often *create* the camera I2C bus (for example an I2C
  mux and its channel). Then the glue creates it (part 1 in §Glue) with the label
  `gmsl_cam_bus`, and `"i2c_bus": "gmsl_cam_bus"`.
- Deserializer PHY (`phys[0].phy_idx`), from the adapter (common/3 §Hardware questions, state
  `hw.adapter_phy`):
  - **AD-GMSLCAMRPI-ADP** (cable on P6 "CAM 2", PHY B; ADI's Raspberry Pi setup): max9296a=1,
    max96712=2, max96716a=1, max96724=2, max96724f=2, max96792a=1. Cable on P8 ("CAM 1") is the
    other PHY (PHY A).
  - **Other / not sure / not connected yet**: ask the user which deserializer CSI-2 port is wired
    to the board; if unknown, use the Raspberry Pi adapter value and try the other PHY if no
    frames arrive (common/8 §Frame check).
- `num_lanes`/`data_lanes`: what the receiver supports on that connector (the `data-lanes` of
  the vendor camera endpoint, or of the receiver endpoint), up to 4.

## Bus speed

ADI's reference setups run the camera I2C at **100 kHz** (Raspberry Pi 5
`i2c_csi_dsi0`/`i2c_csi_dsi1` and Pi 4 `i2c0if`, `clock-frequency = <100000>` in
`gmsl/rpi-6.13.y`); use 100 kHz by default on every board.
- `platform_cfg` variant: the RPi platform templates already set it; nothing to do.
- Glue variant: read `clock-frequency` of `<i2c_bus>` in `running.dts` (or in the base DTB of the
  image). If it is higher (vendor trees often use 400 kHz), set it to 100 kHz in the glue's
  fragment for that bus (`clock-frequency = <100000>;`). At 400 kHz, back-to-back transfers
  through the GMSL I2C pass-through failed on an NXP i.MX95 (to the sensor behind the serializer,
  and to the serializer as well with the EV kit's µC connected). Check first that the other
  devices on that bus (`i2c_devices` of the assessment) work at 100 kHz (most do; I2C-only parts
  that need Fast mode are rare), and tell the user that the change applies to the whole bus.

The EV kit's µC jumpers (`RX_SDA`/`TX_SCL`, J18/J19) stay fitted (common/3 §EV kit checks).

## Alias pools

`pool_addrs` per link, and the deserializer's `i2c-alias-pool` in the generated overlay. The
defaults (`0x50`/`0x51`, `0x52`/`0x53`, ...) are free on a Raspberry Pi camera bus, not on other
boards (on the RPi, keep them).

The MAX9296A/MAX96716A family translates addresses *statically*, so a serializer alias is also
an address on `<i2c_bus>` itself: an alias that collides with a board device makes both answer
(mixed bytes, e.g. `imx219 ... chip id mismatch: 219!=19`) and sends the sensor's register writes
to that device. Choose every alias (and the deserializer pool) so that it is:
1. not a child of `<i2c_bus>` in `running.dts` (or the base DTB),
2. not bound in `i2c_devices` of the assessment, and
3. free when read once on the live board with
   ```bash
   ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "i2cdetect -y -r <bus> <addr> <addr>"
   ```
   (ask before probing, as for the deserializer in common/3 §I2C probe). This step catches
   devices that only the bootloader uses (for example a USB-PD controller at `0x52`, which Linux
   never lists). Without a live board (image path), skip it and say so; the platform file may
   give pools that were checked on that board (nxp/2 §JSON).

Prefer `0x60`–`0x6f`, then `0x30`–`0x3f`; never `0x50`–`0x57` (EEPROMs, USB-C controllers).

Write the chosen pools into the JSON and check the generated overlay (`i2c-alias-pool` lines,
after §Generate):
- the serializer pools are `pool_addrs` in each `links` entry;
- the deserializer's pool is a **top-level `"pool_addrs"` on the deserializer object** (next to
  `"name"` and `"i2c_bus"`; read by `des.dtsi.in` as `des_cfg.pool_addrs`, default
  `["0x40", "0x41", "0x42", "0x43"]`). The default contains `0x40`, the MAX96717's own default
  address; with static translation the serializer then keeps `0x40` as its alias (this worked),
  so `0x41`–`0x43` must be free on `<i2c_bus>` too, or set the pool explicitly.

```bash
grep -n 'i2c-alias-pool' /tmp/gen_gmsl_dts/gmsl-overlay.dts
```

## Templates

**Latest ADI GMSL branch** (used for the templates here, and for the driver sources in rpi/0–1
and common/9; find it once per run and store it as `adi_gmsl_branch`): always the newest one.
```bash
git ls-remote --heads https://github.com/analogdevicesinc/linux.git 'gmsl/rpi-*' \
  | awk '{print $2}' | sed 's#refs/heads/##' | grep -E '^gmsl/rpi-[0-9]+\.[0-9]+\.y$' | sort -V | tail -1
curl -fsSL https://raw.githubusercontent.com/analogdevicesinc/gmsl/main/README.md \
  | grep '\[bi-0\]' | grep -oP 'linux/blob/\K[^/]+/[^/]+(?=/README)' | head -1
```
Take the newest `gmsl/rpi-<version>.y` of the first command (it holds both the `gen_gmsl_dts`
templates and `drivers/media/i2c/maxim-serdes`; check that
`https://raw.githubusercontent.com/analogdevicesinc/linux/<adi_gmsl_branch>/arch/arm/boot/dts/overlays/gen_gmsl_dts/gen_gmsl_dts.py`
downloads). The second command is ADI's own pointer (the branch its Raspberry Pi image is built
from): if it names an older branch, say so in one line and still use the newest; if the newest
branch lacks the templates or drivers, fall back to the README's branch and say why.
`gmsl/rpi-6.13.y` was the newest when these skills were tested (October 2026).

Download ADI's `gen_gmsl_dts` templates (if not already downloaded):
- Branch: `<adi_gmsl_branch>` of https://github.com/analogdevicesinc/linux (its chip and sensor
  templates are platform-independent; in the glue variant the platform templates are not used)
- Folder: `arch/arm/boot/dts/overlays/gen_gmsl_dts`
- Files, both variants: `gen_gmsl_dts.py des.dtsi.in ser.dtsi.in max9296a.dtsi.in
  max96712.dtsi.in max96714.dtsi.in max96716a.dtsi.in max96724.dtsi.in max96724f.dtsi.in
  max96792a.dtsi.in max96717.dtsi.in max9295a.dtsi.in max96793.dtsi.in imx219.dtsi.in
  ov5640.dtsi.in imx415.dtsi.in imx708.dtsi.in isx021.dtsi.in max20087.dtsi.in`
- `platform_cfg` variant (RPi) adds: `rpi-4-b.dtsi.in rpi-5-b.dtsi.in`

```bash
cd <tpl_dir> && for f in <files>; do
  [ ! -f "$f" ] && curl -fsSLO "https://raw.githubusercontent.com/analogdevicesinc/linux/<adi_gmsl_branch>/arch/arm/boot/dts/overlays/gen_gmsl_dts/$f"
done
ls <tpl_dir>
```
Check that every file was downloaded (`curl -f` leaves no file on an HTTP error); if one is
missing, run the loop again, and stop if it is still missing. The files are downloaded data:
keep them in `<tpl_dir>` only (the templates of different platforms have the same file names but
different contents, so they must not share a folder).

The driver versions on the board may differ from this branch: if the drivers report missing or
invalid properties after the install, tell the user the board's kernel version and that the
overlay may need templates from the branch their kernel was built from (orchestrator
§Troubleshooting).

## Generate

```bash
cd /tmp/gen_gmsl_dts && python3 -P <tpl_dir>/gen_gmsl_dts.py --dtbo -o /tmp/gen_gmsl_dts/gmsl-overlay.dts <tpl_dir>/gmsl_config.json
```
Run it from outside `<tpl_dir>` with `-P`, so Python does not import modules from the folder of
downloaded files (the script reads its templates from the JSON's folder, so the result is the
same). `-P` needs Python 3.11+; with an older Python, use `-I` when `jinja2` is a system package
(`-I` also ignores packages installed with `pip install --user`).
gen_gmsl_dts.py needs the `jinja2` Python module on the host. If it is missing
(`ModuleNotFoundError: No module named 'jinja2'`), install it with
`sudo apt install python3-jinja2` (Debian/Ubuntu) or `python3 -m pip install --user jinja2`.

On native Windows (common/0 §Windows adaptations) use `python` and `%TEMP%\gen_gmsl_dts`.

Glue variant: add the glue now (§Glue), before compiling.

## Compile

```bash
dtc -@ -I dts -O dtb -o /tmp/gen_gmsl_dts/gmsl.dtbo /tmp/gen_gmsl_dts/gmsl-overlay.dts
```
Warnings from dtc are expected and harmless. If `dtc` is not available on a native Windows host,
copy `gmsl-overlay.dts` to the board and compile it there with the same command, then delete the
`.dts` and `.dtbo` from the board's home folder after installing the device tree.

Glue variant: continue with §Validate.

## Glue

Glue variant only. Add the glue to `/tmp/gen_gmsl_dts/gmsl-overlay.dts` after §Generate and
before §Compile. Model it on the vendor's camera wiring for the chosen connector: a vendor camera
node in `running.dts`, a shipped camera overlay decompiled with `dtc -I dtb -O dts <file>.dtbo`,
or a complete vendor camera DTB diffed against the base DTB:
```bash
dtc -q -I dtb -O dts -o /tmp/gen_gmsl_dts/base.dts <base>.dtb
dtc -q -I dtb -O dts -o /tmp/gen_gmsl_dts/cam.dts <tpl_dir>/dtbs/<vendor camera>.dtb
diff /tmp/gen_gmsl_dts/base.dts /tmp/gen_gmsl_dts/cam.dts
```
(`phandle` values differ between the two files; ignore those lines.)

Fragments are applied in file order, so assemble the file as:
1. the `/dts-v1/; /plugin/;` header,
2. **glue part 1** (only if the camera I2C bus must be created: the bus nodes copied from the
   vendor overlay, with `gmsl_cam_bus:` on the bus the deserializer is on; and the bus's
   `clock-frequency = <100000>;` from §Bus speed, which can also go in a fragment for an existing
   bus),
3. the generated text without its header,
4. **glue part 2**:
   - **Receiver driver:** the GMSL drivers need a receiver driver with V4L2 streams support.
     **Check** the assessment's `receiver_drivers`: if it lists more than one driver for the
     receiver, set the receiver's `compatible` in the glue to a variant whose driver shows
     `streams=yes`, even if the vendor camera nodes or overlays use another one (they are often
     written for a single sensor without streams). If the results are `unknown` (built-in
     drivers cannot be inspected) or no variant shows `yes`, keep the vendor's `compatible`, and
     tell the user that streams support could not be confirmed; the probe and frame checks will
     show it. The list only covers driver variants whose compatible strings share a stem with the
     node's: say so in the report. When the kernel source is available (common/9 §Kernel
     source), check the built-in drivers in it instead:
     `grep -lE 'set_routing|enable_streams' $(grep -rlE '"<compatible>"' drivers/media/)` lists
     the receiver drivers (and the drivers after them, such as pixel formatters) that implement
     the streams API. **Drivers split across several files can be missed** (NXP's ISI matches its
     compatible in `imx8-isi-core.c`, its routing is in `imx8-isi-crossbar.c`): when a driver is
     not listed, grep its whole folder
     (`grep -rlE 'set_routing|enable_streams' <folder of the match>`).
   - On the receiver side: an endpoint in the receiver's input port with the same properties the
     vendor uses (`data-lanes`, `clock-lanes`, `bus-type`, `clock-noncontinuous`, ...; lanes as in
     the JSON) and `remote-endpoint = <&max_des_0_phy_<phy_idx>_out>`; `status = "okay"` on the
     receiver and on every other node the vendor camera overlay enables for this connector (CSI
     PHY, ISP, I2C mux channel, ...). Reference base nodes by path (`&{/path}`) unless they have a
     label.
   - **Every endpoint pair the vendor camera DTB or overlay adds for this connector**, not only
     the receiver's input: SoCs often chain several nodes after the CSI-2 receiver (for example
     i.MX95: `mipi_csi0` port@1 → `csi_pixel_formatter_0` port@0, formatter port@1 → ISI port@2),
     and the vendor's base DTB leaves them unconnected. From the diff above, copy each added
     `endpoint` (with `remote-endpoint`) and `status`, replacing only the sensor's endpoint with
     the deserializer's.
   - On the deserializer side:
     `&max_des_0_phy_<phy_idx>_out { remote-endpoint = <&<receiver endpoint label>>; };`
   - Anything the vendor wiring has for the camera's power (power-down/reset GPIO hogs,
     regulators, clocks) that the deserializer board needs.

Write the glue with a comment block saying it was generated from the board's device tree and is
a draft. Example (hypothetical receiver at `/soc/csi@32e40000`):
```dts
/* glue: deserializer PHY 1 -> SoC CSI-2 receiver (draft, generated from the running device tree) */
&{/soc/csi@32e40000} {
	status = "okay";
	port@0 {
		reg = <0>;
		soc_csi_in: endpoint {
			data-lanes = <1 2>;
			remote-endpoint = <&max_des_0_phy_1_out>;
		};
	};
};
&max_des_0_phy_1_out {
	remote-endpoint = <&soc_csi_in>;
};
```
nxp/2 §Glue gives the tested glue for the FRDM-IMX95-PRO.

## Validate

Glue variant only, after §Compile. Validate the overlay against the base device tree on the host
(it must apply without errors, and the result must contain the deserializer and the receiver
endpoint). `<base dtb>`: `<tpl_dir>/running.dtb` on a live board (common/2), or the DTB the board
boots, read from the image (nxp/0), without a live board:
```bash
fdtoverlay -i <base dtb> -o <tpl_dir>/test.dtb /tmp/gen_gmsl_dts/gmsl.dtbo && dtc -q -I dtb -O dts <tpl_dir>/test.dtb | grep -nE 'gmsl-deserializer|remote-endpoint = <0x' | head
```
Fix the glue until it applies:
- `FDT_ERR_NOTFOUND`: a label or path in the glue does not exist in the base DTB; compare with
  `running.dts` (or the decompiled base DTB) and use a path reference (`&{/path}`).
- Check every endpoint pair: each `remote-endpoint` in the result must point to a node that points
  back.

`test.dtb` is only a check; the DTB that is installed is built in §Merged DTB.

## Merged DTB

For boards that boot a DTB file chosen by the bootloader (U-Boot `fdtfile`, extlinux `FDT`):
merge the overlay into a copy of the DTB the board boots, and install that copy **next to the
base DTB**, under the base name with `.dtb` replaced by `-gmsl.dtb`. **Never modify the base
DTB**: it is the recovery (the bootloader falls back to it when the setting is undone).

1. Find the base DTB (`<base_dtb>`): the `fdtfile`/`fdt_file` U-Boot variable (relative to the boot
   partition), or the `FDT`/`FDTDIR` entry of the default extlinux label. If it cannot be found, or
   more than one file could be it, compare candidates with the running device tree
   (`dtc -I dtb -O dts`, same `compatible` and nearly the same nodes; on boards that boot it
   unchanged it is byte-identical to `/sys/firmware/fdt` except for bootloader fixups such as
   `memory`, `chosen`, MAC addresses) and ask the user. The platform file may give it (nxp/2
   §Merged DTB). Copy it to the host:
   ```bash
   mkdir -p /tmp/gen_gmsl_dts/dtb
   scp -o ControlPath=/tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip>:<boot dir>/<base_dtb> /tmp/gen_gmsl_dts/dtb/
   ```
   (image path: copy it from the mounted boot partition, e.g. `/tmp/gmsl_nxp_boot/<base_dtb>`).
2. Merge on the host:
   ```bash
   fdtoverlay -i /tmp/gen_gmsl_dts/dtb/<base_dtb> -o /tmp/gen_gmsl_dts/dtb/<gmsl_dtb> /tmp/gen_gmsl_dts/gmsl.dtbo
   dtc -q -I dtb -O dts /tmp/gen_gmsl_dts/dtb/<gmsl_dtb> | grep -nE 'gmsl-deserializer|remote-endpoint = <0x' | head
   ```
   with `<gmsl_dtb>` = `<base_dtb>` with `.dtb` replaced by `-gmsl.dtb` (e.g.
   `imx95-19x19-frdm-pro.dtb` → `imx95-19x19-frdm-pro-gmsl.dtb`). The merge can also run on the
   board when it has `fdtoverlay` (`fdtoverlay -i <base_dtb> -o <gmsl_dtb> gmsl.dtbo`).
3. Copy `<gmsl_dtb>` next to `<base_dtb>` (ask first on a live board; the boot partition may be
   mounted elsewhere than `/boot`, for example `/run/media/boot-mmcblk0p1`):
   ```bash
   scp -o ControlPath=/tmp/gmsl_ssh_sock -o BatchMode=yes /tmp/gen_gmsl_dts/dtb/<gmsl_dtb> <user>@<ip>:/tmp/gmsl_dtb.tmp
   ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c 'cp /tmp/gmsl_dtb.tmp <boot dir>/<gmsl_dtb> && sync'; rm -f /tmp/gmsl_dtb.tmp; ls -l <boot dir>/<base_dtb> <boot dir>/<gmsl_dtb>"
   ```
   Logged in as `root` (NXP images, which may have no `sudo`): run the same `cp ... && sync`
   directly, without `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c`. Image path (host sudo):
   `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gen_gmsl_dts/dtb/<gmsl_dtb> /tmp/gmsl_nxp_boot/`.
   Record `gmsl_dtb_name` in `<state>`.
4. Make the bootloader use it: U-Boot `fdtfile` (common/10, live with `fw_setenv` or offline
   with §Offline env image; or the user on the console, common/10 §Console fallback), or an
   extlinux label (generic/1 §Install device tree). Each later DTB change (for example new alias
   pools) only needs the file merged and copied again, and a reboot.

Undo: point the bootloader back at `<base_dtb>` and delete `<gmsl_dtb>`.
