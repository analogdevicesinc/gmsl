# Common 2: Identify the board

Run after common/1 (the state holds `board_ip`, `board_user`, `board_pw`). Assesses the board
read-only, saves the files the later steps need on the host, selects the platform and reports to
the user. At the end, the state holds `platform` and `board_home`.

`<tpl_dir>` = `/tmp/gen_gmsl_dts/<platform>`. The platform is not known before §Platform
selection: save everything in `/tmp/gen_gmsl_dts/assess` first, and move the folder to
`<tpl_dir>` once the platform is selected (`mkdir -p /tmp/gen_gmsl_dts && mv` — if `<tpl_dir>`
already exists from an earlier deployment in this run, move only the files). All of it is
deleted by the orchestrator's §Cleanup (with `/tmp/gen_gmsl_dts`); none of it is copied
anywhere else, except what generic/2 §Artifacts package lists.

## Run the assessment

Copy `<skill dir>/common/assess.sh` to the board and run it. It only reads; the password is used
for `sudo dmesg`, `fw_printenv` and reading `/sys/firmware/fdt` (as root it needs none). Save the
report on the host:
```bash
mkdir -p /tmp/gen_gmsl_dts/assess
sshpass -p '<pw>' scp -o StrictHostKeyChecking=no <skill dir>/common/assess.sh <user>@<ip>:/tmp/gmsl_assess.sh
sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- "sh /tmp/gmsl_assess.sh '<pw>'; rm -f /tmp/gmsl_assess.sh" > /tmp/gen_gmsl_dts/assess/assess_report.txt 2>&1
```
(on Windows, use common/0 §Windows adaptations; the report then goes to
`%TEMP%\gen_gmsl_dts\assess\assess_report.txt`). The report ends with `=== end`; if it does not,
the script was interrupted (connection lost): run it again once, then report the error.

Also save the running device tree on the host, for the later steps (common/3, common/4):
```bash
sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- "echo '<pw>' | sudo -S -p '' cat /sys/firmware/fdt" > /tmp/gen_gmsl_dts/assess/running.dtb
dtc -q -I dtb -O dts -o /tmp/gen_gmsl_dts/assess/running.dts /tmp/gen_gmsl_dts/assess/running.dtb
```
(as `root`, or on images without `sudo`, run `cat /sys/firmware/fdt` directly). If
`running.dtb` is empty or `dtc` fails, check the `running fdt` line of the report's
`device_tree_and_boot` section and read `/sys/firmware/fdt` as it says.

Also save the board's kernel config (Build mode needs it: common/9, nxp/1) and its DTB files (the
glue is modelled on the vendor's camera DTBs: common/4 §Glue, nxp/2):
```bash
sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- 'zcat /proc/config.gz' > /tmp/gen_gmsl_dts/assess/board.config 2>/dev/null || rm -f /tmp/gen_gmsl_dts/assess/board.config
mkdir -p /tmp/gen_gmsl_dts/assess/dtbs; for f in <DTB files listed under shipped_overlays>; do sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- "cat $f" > /tmp/gen_gmsl_dts/assess/dtbs/$(basename $f); done
```
`<DTB files listed under shipped_overlays>`: the paths of the `dtb: <path> <size>` lines and the
camera `*.dtbo` lines of the report's `shipped_overlays` section. On a Raspberry Pi, skip the DTB
copy (the rpi platform uses ADI's platform templates, not the vendor DTBs). Check that every
copied file has the size the report lists.

If `/proc/config.gz` is missing (`kernel_config` section: `source: none`) and the report says the
`configs` module exists, the config can be read after `modprobe configs`. It loads a module on the
board (harmless, gone at the next reboot): ask the user first (header: "Kernel config"), only when
Build mode or nxp/1 needs it. With `/boot/config-$(uname -r)` instead, copy that file.

If the run has to stop before the end (for example the board goes offline) and the user wants to
resume later in this session, tell them that `/tmp/gen_gmsl_dts` is kept for that; the
orchestrator's §Cleanup deletes it when the run finishes.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the assessment report, `running.dts`, `board.config` and the DTB list into `<run_dir>/board/` (§Verbose mode §What to save).

## Board data

Read from the report (section names in brackets) and from the identify command of common/1
§Login:
- **Identity** (`identity`): `model` (M), `compatible` (C), `os`, `kernel` (release, version,
  machine), `cmdline`, and the `soc0.*` lines.
- **Hostname** and ADI image signs: an ADI GMSL Raspberry Pi image has a hostname starting with
  `analog-rpi-gmsl` and `~/Workspace/pyv4l2` (`pyv4l2` printed by common/1 §Login).
- **Kernel source** (`kernel_source`): `release`, `git hash` (the `-g<hash>` of the release
  string, or `none`), `CONFIG_CC_VERSION_TEXT`, `CONFIG_MODVERSIONS`, `CONFIG_LOCALVERSION`, the
  vendor `max9*`/`max2*` module names.
- **Boot** (`device_tree_and_boot`): `u-boot version` (from
  `/proc/device-tree/chosen/u-boot,version`, for example `2026.04-lf_v2026.04+g6eeef838dac+p0`),
  the boot files found, `fw_printenv`, runtime overlay support.
- **Drivers** (`gmsl_drivers`): the ADI serdes modules and their `depends` (common/3 §Driver
  check confirms them for the chosen hardware).
- **Home folder** of the login, `<board_home>`. Do not assume it: root's home is `/root` on most
  images, including NXP's Yocto images:
  ```bash
  sshpass -p '<pw>' ssh -o StrictHostKeyChecking=no <user>@<ip> -- 'echo $HOME'
  ```
  Use `~` or `$HOME` in remote commands, or the stored `<board_home>` where a full path is
  needed (an `scp` target, a file written into the board's configuration), never
  `/home/<user>`.

For the NXP check of §Platform selection, look up the git hash in NXP's kernel tree (read-only,
needs Internet on the host; only when the compatible contains `fsl,imx`):
```bash
git ls-remote https://github.com/nxp-imx/linux-imx 'refs/tags/lf-*' | grep -i '^<git hash>'
```
A match gives the release tag (for example `lf-6.18.20-2.0.0`), which nxp/1 §Values uses.
Annotated tags are listed twice (`<tag>` and `<tag>^{}`); the `^{}` line carries the commit hash.

Store the home folder:
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['board_home'] = '<board_home>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

## Platform selection

Select the platform from the board data, in this order (first match wins):

| `platform` | Selected when | Platform file to read next | Notes |
|------------|---------------|----------------------------|-------|
| `rpi` | M contains `Raspberry Pi 5 Model` or `Raspberry Pi 4 Model` (e.g. `Raspberry Pi 5 Model B Rev 1.0`; the word `Model` keeps the Pi 400/500 out) | `<skill dir>/rpi/4-camera-setup.md` | `rpi_model` = `5` or `4`. RPi 5: two 4-lane camera connectors CAM0 / CAM1; RPi 4: one 2-lane connector. ADI GMSL image: hostname `analog-rpi-gmsl*` and `pyv4l2`; other Raspberry Pi OS images are accepted too (the driver check decides) |
| `nxp` | C contains `fsl,imx` **and** the NXP BSP is running: the kernel's git hash is an `lf-*` tag of `nxp-imx/linux-imx` (§Board data), or the U-Boot version contains `lf_v` | `<skill dir>/nxp/5-camera-setup.md` | `nxp_board` normalized: `FRDM-IMX95-PRO` (C contains `fsl,frdm-imx95-pro`; tested), otherwise `other-imx` with the model string M in `nxp_model` (best effort, nxp/2 §Other i.MX boards) |
| `generic` | Everything else: other SoCs (including custom ones), other Raspberry Pi models, and i.MX boards that run a mainline or other non-NXP kernel | `<skill dir>/generic/0-assessment.md` | Best effort (generic/0 §Best-effort plan) |

- If the board was prepared by this run (`entry` = `prepare` in the state), the platform must be
  `target_platform`. If it is not (for example an i.MX board whose release hash is not found
  because the host has no Internet access), tell the user what was found; use `target_platform`
  only if the model and the `u-boot version` agree with it, otherwise ask.
- An i.MX board whose hash lookup fails only because the host has no Internet: the U-Boot
  version check (`lf_v`) still applies. If neither can be checked, ask the user (header:
  "Platform"): **NXP BSP image** | **Other image (generic)**.
- Another Raspberry Pi model (Raspberry Pi 3 and older, the Compute Modules CM4/CM5, the
  keyboard models Raspberry Pi 400/500, whose model strings have no `Model` after the number):
  tell the user that only the Raspberry Pi 4 Model B and 5 Model B have a dedicated platform file
  and that the board is handled as `generic` (best effort).

Store the platform (and the board variant):
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['platform'] = '<rpi|nxp|generic>'
state['rpi_model'] = '<5|4, rpi only>'
state['nxp_board'] = '<FRDM-IMX95-PRO|other-imx, nxp only>'
state['nxp_model'] = '<model string M, other-imx only>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
(leave out the keys that do not apply). Then move `/tmp/gen_gmsl_dts/assess` to `<tpl_dir>`
(see the top of this file).

## Report to the user

Tell the user what was found in one line: IP address, board model and platform, kernel version,
OS release, login used, and the image type (ADI GMSL image, NXP BSP release `<lf tag>`, a medium
prepared by this run, or another image). For `generic`, add that the board has no dedicated
platform file and that the run is best effort.

Then return to the orchestrator's §Flow, which reads the platform file of the table above.
