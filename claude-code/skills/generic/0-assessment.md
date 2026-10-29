# Generic 0: Assessment ratings and best-effort plan

For boards without a dedicated platform folder (any SoC, including custom ones, and i.MX boards on
a mainline kernel). Nothing is known about the board in advance: no vendor documentation, BSP or
kernel source. Everything comes from the running board, and from the person who has the board
within reach (the user), who can answer physical questions and recover the board if a boot change
fails.

The goal is always the same as on the known platforms: **captured frames and a live view**. The
assessment shows what is missing; every missing piece becomes a task in the best-effort plan
below, not a reason to stop. Placeholders as in the orchestrator; `<tpl_dir>` =
`/tmp/gen_gmsl_dts/generic`.

The data comes from common/2 §Run the assessment (`assess.sh`, saved as
`<tpl_dir>/assess_report.txt`, with `running.dtb`/`running.dts`, `board.config` and the boot
partition's DTBs in `<tpl_dir>/dtbs/`). Read the report completely before rating.

## Ratings

Rate every layer from the report (green = works, amber = works after a fix the skill makes,
red = needs more work: a task of §Best-effort plan):

| Layer | Green | Amber | Red |
|-------|-------|-------|-----|
| **ADI serdes drivers** (`gmsl_drivers`, `kernel_config`) | `max_serdes` and the deserializer/serializer drivers: modules whose `depends` contains `max-serdes`, or built in together with a built-in `max_serdes` (or `CONFIG_VIDEO_MAXIM_SERDES=y`) | Missing, but the **Kernel source** layer is green: build them (common/9) | Missing, and **Kernel source** is not green. GMSL modules that do not depend on `max-serdes` (for example vendor `max96712`/`max9295`/`max9286` drivers, or NXP's `max96724`/`max96717_lib`) are other drivers, not compatible with the ADI device tree. List their module names: an ADI module with the same name (for example `max96724.ko`) must not be built (common/9 §ADI drivers) |
| **Kernel source** (`kernel_source`) | The release string ends in a git hash (`-g<12 hex>`) and the vendor's public tree has a tag or branch at that commit (find it with `git ls-remote <vendor tree> 'refs/tags/*'`, matching the hash; known trees: NXP i.MX `https://github.com/nxp-imx/linux-imx` (tags `lf-*`), AMD Xilinx `https://github.com/Xilinx/linux-xlnx`, TI `https://git.ti.com/git/ti-linux-kernel/ti-linux-kernel.git`, Raspberry Pi `https://github.com/raspberrypi/linux`; otherwise ask the user for the vendor's BSP source); `/proc/config.gz` exists; `CONFIG_MODULE_SIG_FORCE` is not set; the compiler in `CONFIG_CC_VERSION_TEXT` is available for the host (same major version) | Source found, but no matching compiler packaged on the host: a vendor or Arm GNU toolchain of the same major version can be downloaded (ask) | No public source at that commit, no `/proc/config.gz`, or forced module signing |
| **Kernel prerequisites** (`kernel_symbols`) | `v4l2_subdev_enable_streams`, `v4l2_subdev_set_routing`, `i2c_atr_new`, `media_create_pad_link`, `v4l2_fwnode_endpoint_alloc_parse` found | Some only in a module that is not loaded (fine) | `v4l2_subdev_enable_streams` or `i2c_atr_new` NOT FOUND: the kernel is too old for the ADI drivers (needs the V4L2 streams API and I2C ATR, recent mainline kernels) |
| **Building drivers on the board** (`build_capability`) | — | Kernel headers and `gcc`/`make` present: the ADI `maxim-serdes` drivers could also be built on the board (the skill builds on the host, common/9; mention it) | No headers: drivers come from the host build against the vendor's kernel source |
| **Sensor driver** | The sensor module (mainline name, e.g. `imx219`) present | Present but without `.get_frame_desc` (checked in the kernel source, common/9 §Sensor driver): patched copy | Missing |
| **CSI-2 receiver** (`csi_receivers_and_cameras`, `media`) | A receiver node with graph endpoints and a bound driver, and a media device with V4L2 subdevs | A receiver node that is `disabled` (the glue enables it). Vendor base device trees often list receivers `disabled` with empty `ports` and no endpoints (for example NXP i.MX95 `snps,dw-mipi-csi2`); the camera DTBs or overlays show how they are wired | None found, or the camera pipeline is a vendor stack outside V4L2 (no media device; vendor daemons) |
| **Vendor camera framework** (`gmsl_drivers`, `csi_receivers_and_cameras`, `shipped_overlays`) | Mainline sensor drivers are used with the receiver (the vendor camera nodes use mainline `compatible` strings and properties) | — | The receiver works only with vendor sensor drivers (for example vendor sensor modules depending on a vendor camera module, or vendor camera nodes with mode tables such as `mode0 { ... }` and vendor camera platform nodes). The ADI sensor templates use mainline bindings and will not bind: the sensor nodes must be rewritten in the vendor's format (a platform port) |
| **Wiring reference** (`csi_receivers_and_cameras`, `shipped_overlays`) | Vendor camera nodes, camera overlays (`*.dtbo`) or complete camera DTBs (`*cam*.dtb`, `*<sensor>*.dtb` on the boot partition) that connect a sensor to the receiver: they show endpoints, lanes, I2C bus, clocks and power GPIOs. For a complete DTB, diff it against the base DTB (`dtc -I dtb -O dts`, both) to get the camera wiring | — | None: the glue must be written from the receiver's binding alone (tell the user it is a draft) |
| **Camera I2C bus** (`i2c_devices`) | Only camera devices, or devices whose addresses are known and do not collide with the GMSL addresses and alias pools (common/4 §Alias pools) | Shared with other devices: choose alias pools around them | Shared with power-critical devices (USB-PD controllers such as `ptn5110`/`tcpci`, PMICs, power switches): it works, but anything that holds the bus can break power negotiation (often done by the bootloader, with devices Linux does not list). Tell the user, and never scan the whole bus |
| **Device tree install** (`device_tree_and_boot`) | Runtime overlays: `of_overlay_fdt_apply` found and `dt-overlays-dir=yes` | A boot method the skill can change with a fallback (generic/1 §Install device tree), including U-Boot loading the DTB file from a writable boot partition without `fw_setenv` on the board: `fw_setenv` built from the matching U-Boot source (common/10, needs `u-boot version` in the report and a saved environment), or the user on the serial console (common/10 §Console fallback) | FIT image, verified/signed boot, read-only boot files, or an unknown boot method |
| **Userland** (`userland`) | `media-ctl routes: yes`, `v4l2-ctl`, `i2cdetect`, `dtc`/`fdtoverlay` | Missing, but a package manager exists (generic/1 §Tools) | Missing and no package manager |

The prerequisites only show that the kernel *can* run the drivers. Whether the receiver driver
handles GMSL streams (one virtual channel per camera on one CSI-2 link) is only known once frames
arrive; with more than one camera, recommend starting with one.

Show the user the table of ratings (one line per layer, with the evidence from the report), and
the plan of §Best-effort plan for every amber and red layer. Write the table to
`<tpl_dir>/assessment.md` (it goes into the artifacts, generic/2).

**If the board matches a dedicated platform**, say so and switch (orchestrator §Flow): a
Raspberry Pi 4/5 → rpi/4; an NXP i.MX board running NXP's BSP (compatible `fsl,imx*` and an `lf-*`
kernel tag or `lf_v` in `u-boot,version`) → nxp/5. Update `platform` in `<state>`.

## Best-effort plan

With `scope` = `setup_only` (the user asked for the GMSL setup only), the plan contains no
building, porting or boot changes: tasks that need them are listed as missing, and the user is
offered the full pipeline once (header: "Scope", as in the orchestrator's §Flow).

Every red or amber layer becomes a task with the approach to try. Show the plan as a numbered
list (layer → task → what it changes on the board → time), store it as `generic_plan` in
`<state>`, and ask the user to agree (header: "Plan"): **Go ahead (Recommended)** | **Only build
the artifacts** (generic/2 §Artifacts package, nothing is changed on the board) | **Only the
report**. Each task that changes the board is asked again before it runs (orchestrator §Rules).

| Layer red/amber | Task | Approach |
|-----------------|------|----------|
| ADI serdes drivers missing | Build them against the board's kernel | common/9, from §Inputs to §Install on the board (`updates/<subfolder>`, e.g. `gmsl`). Port compile errors one at a time (common/9 §Port; rpi/1 Steps 5a-fix1–5a-fix3 and 5c show the same API changes handled in a full tree). |
| Kernel source: no matching compiler | Download a toolchain of the same major version | common/9 §Compiler (Arm GNU Toolchain, `.sha256asc` check), then the CRC check (common/9 §CRC check) before trusting it |
| Kernel source: no public tag at the commit | Find the source elsewhere | Ask the user for the vendor's BSP source or SDK (a tarball, a Yocto layer's `SRC_URI`/`SRCREV`, a git URL); try the vendor tree's branches (`git ls-remote --heads <tree> \| grep <hash>`) and the release notes of the OS image |
| Kernel source: no `/proc/config.gz` | Get the exact config | `/boot/config-$(uname -r)` on the board, `configs.ko` (`modprobe configs`), `scripts/extract-ikconfig` on the kernel image, or the vendor's defconfig plus the BSP's fragments (ask the user); then check `kernelrelease` and the CRCs (common/9) — a config that does not reproduce the CRCs cannot be used |
| Kernel prerequisites missing (no streams API, no I2C ATR) | Backport them, with the user's agreement | This replaces the kernel image, not only modules: tell the user the risk and how to boot the old kernel. Use rpi/1's approach (ADI commits `--not` the base branch, applied all-or-nothing with `patch --dry-run`, Steps 1–3b, and the conditional fixes of Steps 4–9) on the vendor tree, build the whole kernel (rpi/2 as the model: config overrides, verify), and install it next to the vendor kernel with a fallback boot entry (generic/1 §Install device tree, method 2 rules). If the backport needs more than the ADI patches (core structures changed), report what is missing |
| Sensor driver missing or without `.get_frame_desc` | Build a patched copy | common/9 §Sensor driver (from the vendor tree, or the ADI branch's driver, ported like the serdes drivers) |
| CSI-2 receiver disabled | Enable it in the glue | common/4 §Glue |
| CSI-2 receiver driver without streams support | Try another driver variant, then one camera | Another `compatible` whose driver shows `streams=yes` (common/4 §Glue, receiver driver check); otherwise a single-camera configuration (one route); report it as a receiver limitation if routing still fails |
| No CSI-2 receiver, or a vendor stack outside V4L2 | — | Truly impossible for this skill: Artifacts/Report |
| Vendor camera framework (vendor-only sensor bindings) | Port the sensor nodes, with the user's agreement | Rewrite the sensor nodes of the generated overlay in the vendor's format, modelled on the vendor camera nodes (mode tables, platform nodes), keeping the deserializer/serializer nodes from the templates; tell the user it is a platform port and a draft. If the vendor sensor driver cannot be fed by the ADI serializer (no `.get_frame_desc`, no V4L2 subdev): Artifacts with this as the main missing item |
| No wiring reference | Write the glue from the receiver's binding | common/4 §Glue, from the mainline binding of the receiver's `compatible` (generic/2 §Troubleshooting, reference documents); mark it as a draft |
| Camera I2C bus shared with power-critical devices | Proceed carefully | Tell the user; probe single addresses only; alias pools around the bus's devices (common/4 §Alias pools); EV kit powered before the board |
| No usable install method | Try every method | generic/1 §Install device tree, methods 1–3, including the U-Boot paths of common/10 (envtools from the vendor U-Boot source, the offline environment, the console). Signed/verified boot without the user's keys: truly impossible: Artifacts |
| Userland missing | Install or build the tools | generic/1 §Tools (package manager; v4l-utils 1.28.1 from source; without a package manager, static binaries cross-built on the host, ask first) |

**Artifacts or Report only** when something is truly impossible — no kernel source at all (the
vendor does not publish it and the user cannot provide it), forced module signing without the
user's signing key, signed/verified boot without the user's keys, or a vendor camera stack outside
V4L2 — or when the user declines the plan or a task. Then continue with generic/2: the artifacts
package (everything up to the validated overlay; the built modules and patches if the build got
that far) or the report, with a clear list of what was tried, what failed and why, and what is
needed to go further.

Otherwise continue with generic/1 (§Order).
