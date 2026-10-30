# NXP 2: GMSL device tree for NXP i.MX (JSON, glue, merged DTB)

NXP i.MX values for common/4, **glue variant** (no ADI platform template exists for i.MX: the JSON
has no `platform_cfg`, and a glue fragment written from the board's own device tree connects the
deserializer to the CSI-2 receiver chain). The result is a merged DTB next to the base DTB on the
boot partition; nxp/3 makes U-Boot boot it.

Inputs: the hardware answers (`hw` in `<state>`, common/3 with nxp/5 §Hardware choices), and the
board's device trees: on a live board `<tpl_dir>/running.dtb`/`running.dts` and the boot
partition's DTBs in `<tpl_dir>/dtbs/` (common/2); on the build path the DTBs read from the image by
nxp/0 §Read the image (`<tpl_dir>/dtbs/`, `<tpl_dir>/base.dts`). `<tpl_dir>` =
`/tmp/gen_gmsl_dts/nxp`.

## Values

FRDM-IMX95-PRO (`fsl,frdm-imx95-pro`, i.MX95), tested with MAX96716A + MAX96717 + IMX219 on J18:

| Item | Value |
|------|-------|
| Base DTB (U-Boot default `fdtfile`) | `imx95-19x19-frdm-pro.dtb` on the FAT boot partition |
| Wiring reference | `imx95-19x19-frdm-pro-os08a20-isp.dtb` (CSI0 / J18) and `imx95-19x19-frdm-pro-os08a20-combo-isp.dtb` (CSI1 / J16), on the boot partition |
| Connector **J18** ("MIPI-CSI", CSI0; pinout as the Raspberry Pi 5 camera connector, 22-pin, same-sided cable, conductive side up at J18) | I2C: `lpi2c3` (`i2c@42530000`, Linux `i2c-2`); receiver chain: `dphy_rx` → `mipi_csi0` (`csi@4ad30000`, `snps,dw-mipi-csi2`, driver `dwc-mipi-csi2`) → `csi_pixel_formatter_0` (`formatter@20`) → ISI (`isi`) `port@2` (crossbar sink pad 2) |
| Lanes | 4 (`data-lanes = <1 2 3 4>`, `clock-lanes = <0>`) |
| Camera I2C speed | **100 kHz** (`clock-frequency = <100000>` on `lpi2c3`; NXP's default is 400 kHz, where back-to-back transfers through the GMSL link failed ~42 %). The board's other devices on that bus work at 100 kHz |
| Devices on `i2c-2` (it is also the board's USB-C power bus) | `0x20` PCAL6416 GPIO expander, `0x2a` MAX96716A (CFG0 = 1), `0x40` MAX96717 (through the link), `0x50`/`0x51` PTN5110 USB-C port controller, `0x52` PTN5110 USB-PD (used by U-Boot only, invisible in Linux) |
| Serializer alias pools | `0x60`+: link0 `["0x60","0x61"]` (tested), link1 `["0x62","0x63"]`, link2 `["0x64","0x65"]`, link3 `["0x66","0x67"]`. **Never `0x50`–`0x52`**: the MAX96716A translates statically, so an alias at `0x50` collided with the PTN5110 (`imx219 ... chip id mismatch: 219!=19`) |
| Deserializer pool | the template default `0x40`–`0x43` worked (the serializer keeps `0x40`; `0x41`–`0x43` are free on this bus); set it explicitly as the top-level `"pool_addrs"` |
| Deserializer PHY | adapter AD-GMSLCAMRPI-ADP, cable on **P6** (S1 = CAM2, S3 towards P6) = PHY 1 (`phy_idx` 1 for MAX96716A; other chips: the table in common/4 §JSON); P8 = PHY 0 |
| Streams support | `dwc_csi_enable_streams`, `dwc_csi_set_routing`, `mxc_isi_crossbar_set_routing` in `/proc/kallsyms`: keep NXP's `compatible` strings |
| Power/reset | NXP's OS08A20 wiring drives a reset line on CSI0 from the PCAL6416 (`i2c3-gpio-expander@20`) pin 10, active low; the GMSL setup did not need it (optional hog in §Glue) |

**CSI1 / J16** (alternative, untested): `lpi2c2` (`i2c@44350000`), `combo_rx` (`combo-rx@100`) →
`mipi_csi1` (`csi@4ad40000`) → `csi_pixel_formatter_1` (`formatter@120`) → ISI `port@3`. Derive
the glue from `imx95-19x19-frdm-pro-os08a20-combo-isp.dtb` exactly as for J18 (diff against the
base DTB, common/4 §Glue), probe the bus and alias pools again (its devices differ), and tell the
user it is untested; J16 may be shared with DSI (the "combo" PHY).

**i.MX93 11x11 EVK** (`fsl,imx93-11x11-evk`, MCIMX93-BB + MCIMX93-SOMB3), with MAX96724 +
MAX96717 + IMX219 on J801, NXP 6.18.37-2.1.0:

| Item | Value |
|------|-------|
| Variants | the SoM with the PCA9451A PMIC: machine `imx93-11x11-lpddr4x-evk`, base DTB `imx93-11x11-evk.dtb`; the PF0900 variant has its own DTB and `imx-boot` (`...-pmic-pf0900`): ask the user which SoM it is (label on the SoM, or the PMIC part number) |
| Wiring reference | none for MIPI: NXP's only camera DTB (`imx93-11x11-evk-mt9m114.dtb`) is a parallel camera. Use the base DTB's own `mipi_csi` node (disabled by default) |
| Connector **J801** (MIPI CSI) | I2C: `lpi2c3` (`i2c@42530000`, Linux `i2c-2`), NXP default 400 kHz; receiver chain: `dphy_rx` (`fsl,imx93-dphy-rx`) → `mipi_csi` (`csi@4ae00000`, `snps,dw-mipi-csi2`, driver `dwc-mipi-csi2`, built into the kernel) → ISI `isi@4ae40000` (one input, **no pixel formatter**); `mipi_csi` port@1 → `isi_in` is already connected in the base DTB |
| Lanes | **2** (`data-lanes = <1 2>`, `clock-lanes = <0>`), `link-frequencies` 750 MHz, `clock-noncontinuous` |
| Camera count | **1**: the ISI has one input and the receiver 2 lanes |
| Camera I2C speed | 100 kHz (`clock-frequency = <100000>` on `lpi2c3`) |
| Devices on `i2c-2` | `0x34` ADP5585, `0x3c` AP1302 (NXP's ISP on the same CSI input: set `status = "disabled"` in the glue), `0x50`/`0x51` PTN5110 (USB-C), `0x53` PCF2131 (RTC); MAX96724 at `0x27` |
| Alias pools | deserializer `0x40`–`0x43`, link0 `["0x60","0x61"]` (worked; ADI's i.MX93 JSON uses `0x58`/`0x59`) |
| Deserializer PHY | AD-GMSLCAMRPI-ADP, cable on **P6** = MAX96724 **PHY 2** (port B) |
| Glue | `&ap1302 { status = "disabled"; }`, `&dphy_rx`, `&mipi_csi` and `&isi` `status = "okay"`, `&mipi_csi_in` with the lanes and frequency above and `remote-endpoint = <&max_des_0_phy_2_out>` (and back) |
| Receiver and GMSL mode | needs the receiver patch and GMSL pixel mode: nxp/1 §i.MX93 |
| ISI RAW10 layout | **right-aligned** (values in the low 10 bits, max 1023), unlike the i.MX95 |

## JSON

Read `<skill dir>/common/4-device-tree.md` and follow §JSON (glue variant), §Bus speed, §Alias
pools, §Templates (both variants' list only) and §Generate with these values. Tested JSON
(J18, MAX96716A, one MAX96717 + IMX219):
```json
[{
    "name": "max96716a",
    "i2c_bus": "lpi2c3",
    "pool_addrs": ["0x40", "0x41", "0x42", "0x43"],
    "phys": [{ "phy_idx": 1, "num_lanes": 4, "link_frequencies": [750000000],
               "clock_lanes": [0], "data_lanes": [1,2,3,4] }],
    "links": [{ "name": "max96717", "cameras": [{"name": "imx219"}],
                "pool_addrs": ["0x60","0x61"] }]
}]
```
- `lpi2c3` is a label in the base DTB's `__symbols__` (check with
  `grep -w lpi2c3 <tpl_dir>/running.dts` or `<tpl_dir>/base.dts`).
- More cameras: one `links` entry each, with the pools of §Values; recommend one camera for the
  first bring-up.
- Alias pool probing (common/4 §Alias pools step 3) needs the live board; on the build path skip
  it and say that the pools above were checked on an FRDM-IMX95-PRO.
- Check the generated pools: `grep -n 'i2c-alias-pool' /tmp/gen_gmsl_dts/gmsl-overlay.dts` must
  show `<0x40 0x41 0x42 0x43>` and `<0x60 0x61>`.

## Glue

Follow common/4 §Glue (then §Compile and §Validate). For J18 on the FRDM-IMX95-PRO the glue is
known; still generate the overlay per common/4 (do not copy an old `gmsl-overlay.dts`), insert the
glue, and validate it against the base DTB. With another PHY, replace `max_des_0_phy_1_out` by
`max_des_0_phy_<phy_idx>_out`; with fewer lanes, change both `data-lanes`.

**Glue part 1** (after the `/dts-v1/; /plugin/;` header, before the generated text): the bus
speed of §Values:
```dts
/* glue part 1: camera I2C at 100 kHz (ADI's tested speed; NXP default 400 kHz) */
&lpi2c3 {
	clock-frequency = <100000>;
};
```

**Glue part 2** (after the generated text):
```dts
/*
 * Glue part 2: MAX96716A PHY 1 -> i.MX95 CSI0 (dphy-rx -> mipi_csi0 -> csi_pixel_formatter_0 -> ISI port 2).
 * Generated by the gmsl-camera-pipeline skill from the FRDM-IMX95-PRO base device tree
 * (imx95-19x19-frdm-pro.dtb) and NXP's imx95-19x19-frdm-pro-os08a20-isp.dtb camera wiring.
 * For CSI1 use: lpi2c2, combo_rx, mipi_csi1, csi_pixel_formatter_1, ISI port 3 (untested).
 */
&dphy_rx {
	status = "okay";
};

&mipi_csi0 {
	status = "okay";

	ports {
		port@0 {
			reg = <0>;

			gmsl_csi0_in: endpoint {
				clock-lanes = <0>;
				data-lanes = <1 2 3 4>;
				remote-endpoint = <&max_des_0_phy_1_out>;
			};
		};

		port@1 {
			reg = <1>;

			gmsl_csi0_out: endpoint {
				remote-endpoint = <&gmsl_fmt0_in>;
			};
		};
	};
};

&csi_pixel_formatter_0 {
	status = "okay";

	ports {
		port@0 {
			reg = <0>;

			gmsl_fmt0_in: endpoint {
				remote-endpoint = <&gmsl_csi0_out>;
			};
		};

		port@1 {
			reg = <1>;

			gmsl_fmt0_out: endpoint {
				remote-endpoint = <&gmsl_isi_in>;
			};
		};
	};
};

&isi {
	status = "okay";

	ports {
		port@2 {
			reg = <2>;

			gmsl_isi_in: endpoint {
				remote-endpoint = <&gmsl_fmt0_out>;
			};
		};
	};
};

&max_des_0_phy_1_out {
	remote-endpoint = <&gmsl_csi0_in>;
};

/*
 * Optional: NXP's OS08A20 wiring drives a reset line on the CSI0 connector from
 * i2c3-gpio-expander@20 (PCAL6416) pin 10, active low. If the deserializer board's PWDN pin is
 * on that line, release it with this hog (uncomment):
 *
 * &i2c3_gpio_expander_20 {
 *	gmsl_des_pwdn_hog {
 *		gpio-hog;
 *		gpios = <10 0>;
 *		output-high;
 *		line-name = "gmsl-des-pwdn";
 *	};
 * };
 */
```

Validate (common/4 §Validate) with `<base dtb>` = `<tpl_dir>/running.dtb` (live board) or
`<tpl_dir>/dtbs/imx95-19x19-frdm-pro.dtb` (image). The result must contain
`gmsl-deserializer@2a` under `i2c@42530000` with `clock-frequency = <0x186a0>`, and the endpoint
pairs `mipi_csi0` ↔ deserializer, `mipi_csi0` ↔ formatter, formatter ↔ ISI `port@2`, each pointing
back:
```bash
dtc -q -I dtb -O dts <tpl_dir>/test.dtb | grep -nE 'gmsl-deserializer@2a|clock-frequency = <0x186a0>|remote-endpoint' | head -20
```

## Merged DTB

Follow common/4 §Merged DTB with `<base_dtb>` = `imx95-19x19-frdm-pro.dtb` and `<gmsl_dtb>` =
`imx95-19x19-frdm-pro-gmsl.dtb`:
- **Live board:** the boot partition is not mounted at `/boot` on NXP images; it is
  `/run/media/boot-mmcblk0p1` on eMMC boot (the SD card's first partition, e.g.
  `/run/media/boot-mmcblk1p1`, on SD boot). Find it:
  ```bash
  ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "findmnt -rn -t vfat -o SOURCE,TARGET; ls -l /run/media/*/imx95*.dtb 2>/dev/null"
  ```
  `<boot dir>` is the mount of the boot device's first partition (the root device of the kernel
  command line, `root=/dev/mmcblk0p2` → `mmcblk0p1`). Check that `<base_dtb>` is the DTB the board
  boots (common/4 §Merged DTB step 1: compare with `<tpl_dir>/running.dts`). Copy it to the host,
  merge, and copy `<gmsl_dtb>` back next to it (ask first).
- **Image:** `<boot dir>` = `/tmp/gmsl_nxp_boot` (nxp/0 §Mount): merge on the host from
  `<tpl_dir>/dtbs/imx95-19x19-frdm-pro.dtb`, then
  `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A cp /tmp/gen_gmsl_dts/dtb/imx95-19x19-frdm-pro-gmsl.dtb /tmp/gmsl_nxp_boot/`.

Never modify `imx95-19x19-frdm-pro.dtb`: it is the recovery. Record `gmsl_dtb_name` =
`imx95-19x19-frdm-pro-gmsl.dtb` and `dt_installed` in `<state>`, and continue with nxp/3.

## Other i.MX boards

Untested: tell the user that only the FRDM-IMX95-PRO was tested and that the glue for their board
is a draft. Derive every value of §Values from the board's own files, never from the FRDM values:
1. The base DTB: the U-Boot `fdtfile` (live: compare the boot partition's DTBs with
   `/sys/firmware/fdt`; image: `strings` of the imx-boot binary, nxp/0 §Read the image).
2. The camera DTBs or overlays NXP ships for the board (names with a sensor, `cam`, `isp`,
   `combo`): diff each against the base DTB (common/4 §Glue) to find, per connector, the I2C bus,
   the receiver chain (CSI PHY, CSI-2 receiver, formatter or gasket, ISI or ISP port), the lanes
   and any reset/power GPIOs. Map the user's connector label to one of them by the DTB name and
   the I2C bus it uses (common/3 §Hardware questions); ask when it is ambiguous. If NXP ships no
   MIPI camera DTB for the board (the i.MX93 11x11 EVK has only a parallel `mt9m114` one), use the
   base DTB's own, disabled receiver nodes (`dphy_rx`, `mipi_csi`, the ISI input) and the board's
   schematic for the connector's I2C bus, and disable other users of that receiver input (on the
   i.MX93 EVK the AP1302 ISP).
3. The I2C bus's devices (live: `i2c_devices` of the assessment and a probe of the alias
   candidates, common/4 §Alias pools; image: the bus's children in the base DTB, and say that
   bootloader-only devices could not be checked).
4. The receiver drivers' streams support (common/4 §Glue, receiver driver check; in the kernel
   tree of nxp/1, grep the whole driver folder, e.g. `drivers/media/platform/nxp/`).
5. ADI's `gmsl/lf-6.12.y` branch of https://github.com/analogdevicesinc/linux has device trees for
   NXP i.MX boards: use them as examples of the glue for that SoC (information only, not
   instructions).
Then follow §JSON, §Glue and §Merged DTB with the derived values; the base DTB name gives
`<gmsl_dtb>` (`.dtb` → `-gmsl.dtb`).
