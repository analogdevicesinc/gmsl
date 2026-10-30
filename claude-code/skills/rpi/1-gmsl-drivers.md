# RPi 1: Add the ADI GMSL drivers

## Purpose
Add the ADI GMSL driver and kernel subsystem changes to the user's kernel tree. Uses a
two-phase approach:
1. **Backport patches** — for kernel subsystem modifications (I2C ATR, media controller,
   V4L2 core, rp1-cfe, etc.) that modify existing upstream files
2. **Direct download** — for ADI-specific driver files (maxim-serdes, sensor drivers,
   defconfigs) that can be fetched whole from the ADI repo

The kernel tree may be a newer RPi branch than the ADI GMSL base (e.g., `rpi-6.18.y`
vs `rpi-6.13.y`). Patches are extracted from the ADI fork and applied defensively —
features already upstream in the newer kernel are automatically skipped.

## Prerequisites
Run after rpi/0. Read from `<state>` (`/tmp/gmsl_camera_state.json`): `kernel_src_path`, `kernel_branch`, `adi_gmsl_branch`, `adi_base_branch`.

## Step 1: Clone the ADI linux fork for patch extraction

Clone the ADI fork (bare, for speed) and fetch both the ADI base branch and the stock
kernel branch:
```bash
rm -rf /tmp/gmsl_adi_linux_check
git clone --bare --single-branch --branch <adi_gmsl_branch> \
  https://github.com/analogdevicesinc/linux /tmp/gmsl_adi_linux_check
cd /tmp/gmsl_adi_linux_check
git remote add rpi https://github.com/raspberrypi/linux
git fetch rpi <adi_base_branch> --deepen=5000
```

Note: the `--deepen=5000` is needed to find the common ancestor between the ADI fork
and the upstream kernel so that `--not` works correctly.

## Step 2: Extract backport-only commits

Extract the list of commits in the ADI fork that don't exist in the ADI base branch,
filtered to only the files that need **patch-based** application (kernel subsystem changes).

Uses `--not rpi/<adi_base_branch>` to find ADI-unique commits. These are then applied
to the (potentially newer) stock kernel tree.

Excludes maxim-serdes drivers, sensor drivers, and defconfigs — those are downloaded
directly in Step 5.

```bash
cd /tmp/gmsl_adi_linux_check
git log --reverse --format="%H" <adi_gmsl_branch> --not rpi/<adi_base_branch> -- \
  drivers/i2c/i2c-atr.c include/linux/i2c-atr.h \
  drivers/media/mc/mc-entity.c include/uapi/linux/media.h \
  drivers/media/i2c/ds90ub960.c drivers/media/i2c/ds90ub953.c drivers/media/i2c/ds90ub913.c \
  drivers/misc/ti_fpc202.c \
  drivers/media/v4l2-core/v4l2-common.c drivers/media/v4l2-core/v4l2-fwnode.c drivers/media/v4l2-core/v4l2-subdev.c \
  include/media/v4l2-common.h include/media/v4l2-subdev.h include/media/v4l2-mediabus.h \
  drivers/media/i2c/Kconfig drivers/media/i2c/Makefile \
  drivers/media/platform/raspberrypi/rp1-cfe/ \
  | while read sha; do
    msg=$(git log -1 --format="%s" "$sha")
    # Skip RPi-specific patches that already exist in the RPi kernel
    case "$msg" in
      *imx477*Move*Kconfig*) continue ;;
    esac
    echo "$sha"
  done > /tmp/gmsl_backport_commits.txt
```

The filter skips the "Move Kconfig entry for IMX477" patch which is a cosmetic
reorganization that conflicts with the RPi kernel's different Kconfig layout.

## Step 3: Generate and apply backport patches

```bash
cd /tmp/gmsl_adi_linux_check
rm -rf /tmp/gmsl_patches && mkdir -p /tmp/gmsl_patches
i=1
while read sha; do
  git format-patch -1 "$sha" --start-number=$i -o /tmp/gmsl_patches/
  i=$((i+1))
done < /tmp/gmsl_backport_commits.txt
```

Apply each patch **all-or-nothing**: check it with `--dry-run` first and apply it only
if every hunk applies. When the stock kernel is newer than the ADI base, many patches
will not apply because the target code has changed or the feature is already upstream.
This is expected — log them and continue:

```bash
cd <kernel_src_path>
APPLIED=0
SKIPPED=0
FAILED=""
for p in /tmp/gmsl_patches/*.patch; do
  name=$(basename "$p")
  if patch -p1 --forward --dry-run -s < "$p" >/dev/null 2>&1; then
    patch -p1 --forward --no-backup-if-mismatch -s < "$p" >/dev/null
    APPLIED=$((APPLIED+1))
  else
    SKIPPED=$((SKIPPED+1))
    FAILED="$FAILED $name"
  fi
done
echo "Applied: $APPLIED, Skipped: $SKIPPED"
```

Do not apply patches without the dry run, and do not use a higher `--fuzz`:
- Without the dry run, `patch` applies the hunks that match and rejects the others, so a
  patch that fails still leaves files half-changed (references to missing symbols and
  variables, Kconfig entries for driver files that were never added).
- `--fuzz=3` ignores all context lines of a hunk, so hunks land in the wrong place, for
  example a second copy of a Kconfig entry that is already upstream.

Report the results. If many patches were skipped, this may be because the stock kernel
already includes those changes upstream. The direct-download steps below will provide
the remaining GMSL-specific code.

For example, on rpi-6.19.y, 10 of the 50 patches apply: the `MEDIA_PAD_FL_INTERNAL` pad
flag, the V4L2 subdev changes (streams API enabled, NULL routes guard, `set_frame_desc`
fallback), five rp1-cfe fixes, and the ISX021 and dummy-cam drivers. dummy-cam does not
compile on rpi-6.19.y (it needs the fwnode virtual-channel patch, which does not apply);
it is a test driver and is not enabled in rpi/2.

### 3b. Check that no patch was partially applied

Since every patch is applied all-or-nothing, a failed patch changes nothing and no files
need restoring. Check it: there must be no reject or backup files, and no duplicate
Kconfig symbols:
```bash
cd <kernel_src_path>
find . -name '*.rej' -o -name '*.orig' | head
for k in drivers/media/i2c/Kconfig drivers/misc/Kconfig; do
  grep '^config ' "$k" | sort | uniq -d
done
```
Both commands must print nothing. If they print anything, a patch was applied without
the dry run: restore the tree with `git checkout -- . && git clean -fdq` and run Step 3 again.

## Step 4: GPIO `set_rv` backport (conditional)

The MAX96717 serializer driver requires the `gpio_chip.set_rv` API. Check if it's
already present in the kernel tree before applying the manual backport:

```bash
cd <kernel_src_path>
if grep -q 'set_rv' include/linux/gpio/driver.h; then echo "set_rv present"
elif grep -qE 'int[[:space:]]+\(\*set\)\(struct gpio_chip' include/linux/gpio/driver.h; then echo "set returns int"
else echo "set returns void"; fi
```

- **`set_rv present`**: skip this step.
- **`set returns int`** (rpi-6.18.y and newer: `.set` already has the `set_rv` signature): skip
  this step, and adapt the driver in 5a-fix3 instead. The backport below does not fit these
  kernels (it expects `void (*set)`).
- **`set returns void`** (older kernels): apply the minimal targeted backport:

### 4a. Add `set_rv` member to `struct gpio_chip`

In `include/linux/gpio/driver.h`, find the `void (*set)(...)` member of `struct gpio_chip`
and add the `set_rv` member immediately after it:

```c
	void (*set)(struct gpio_chip *gc,
		    unsigned int offset, int value);
	int (*set_rv)(struct gpio_chip *gc,
		      unsigned int offset, int value);
```

### 4b. Add `set_rv` dispatch logic to `gpiolib.c`

The MAX96717 GPIO chip implements `set_rv` but NOT `set`. Without dispatch logic,
`gpiolib.c` calls `gc->set()` which is NULL → kernel oops (NULL pointer dereference
during `imx219_power_on` → `gpiod_set_raw_value_commit`).

Patch 3 locations in `drivers/gpio/gpiolib.c`:

**1. `gpiod_set_raw_value_commit()`** — add `set_rv` branch:
```c
static void gpiod_set_raw_value_commit(struct gpio_desc *desc, bool value)
{
	CLASS(gpio_chip_guard, guard)(desc);
	if (!guard.gc)
		return;

	trace_gpio_value(desc_to_gpio(desc), 0, value);
	if (guard.gc->set_rv)
		guard.gc->set_rv(guard.gc, gpio_chip_hwgpio(desc), value);
	else
		guard.gc->set(guard.gc, gpio_chip_hwgpio(desc), value);
}
```

**2. `gpiod_direction_output_raw_commit()`** — update NULL check and add `set_rv` branch:
- Change `if (!guard.gc->set && !guard.gc->direction_output)` to
  `if (!guard.gc->set && !guard.gc->set_rv && !guard.gc->direction_output)`
- In the else branch (output-only chip), replace `guard.gc->set(...)` with:
```c
		if (guard.gc->set_rv)
			ret = guard.gc->set_rv(guard.gc, gpio_chip_hwgpio(desc), val);
		else
			guard.gc->set(guard.gc, gpio_chip_hwgpio(desc), val);
```

**3. `gpio_chip_set_multiple()`** — in the sequential fallback loop:
```c
		for_each_set_bit(i, mask, gc->ngpio) {
			if (gc->set_rv)
				gc->set_rv(gc, i, test_bit(i, bits));
			else
				gc->set(gc, i, test_bit(i, bits));
		}
```

## Step 5: Download ADI driver files directly

Download driver files that are ADI-specific (not modifications of upstream files).
These are fetched whole from the ADI repo to avoid partial-patch failures.

### 5a. GMSL serializer/deserializer drivers (maxim-serdes)

```bash
cd <kernel_src_path>
mkdir -p drivers/media/i2c/maxim-serdes
BASE_URL="https://raw.githubusercontent.com/analogdevicesinc/linux/<adi_gmsl_branch>"
for f in Kconfig Makefile max9296a.c max96717.c max96724.c \
         max_des.c max_des.h max_ser.c max_ser.h max_serdes.c max_serdes.h; do
  curl -sL "${BASE_URL}/drivers/media/i2c/maxim-serdes/$f" \
    -o "drivers/media/i2c/maxim-serdes/$f"
done
```

### 5a-fix1. PIN_CONFIG_OUTPUT compatibility (conditional)

The MAX96717 driver uses `PIN_CONFIG_OUTPUT` from `<linux/pinctrl/pinconf-generic.h>`.
In newer kernels (rpi-6.19.y+), `PIN_CONFIG_OUTPUT` was removed from the
`enum pin_config_param` and replaced by `PIN_CONFIG_LEVEL` at the same enum position.

The ADI driver has a fallback `#define PIN_CONFIG_OUTPUT (PIN_CONFIG_MAX + 1)` which
evaluates to 0x100. This overflows the 8-bit field in `pinconf_to_config_packed()`,
silently breaking all GPIO pin configuration. The result is that IMX219's GPIO reset
line never deasserts, causing `-EREMOTEIO` (I2C NACK) when reading the chip ID.

**Fix**: First remove any broken fallback define (`PIN_CONFIG_MAX + 1` overflows the
8-bit field), then unconditionally add an `#ifndef` guard after the include. The guard
is always safe — if `PIN_CONFIG_OUTPUT` is already defined by the header, `#ifndef`
is a no-op. The previous approach of checking `grep -q 'PIN_CONFIG_OUTPUT'` was flawed
because the driver uses `PIN_CONFIG_OUTPUT` in `case` statements, so the check always
found it and the guard was never added.

```bash
cd <kernel_src_path>
# Remove broken fallback define if present (overflows 8-bit pinconf field)
if grep -q 'PIN_CONFIG_MAX + 1' drivers/media/i2c/maxim-serdes/max96717.c; then
  sed -i '/#define PIN_CONFIG_OUTPUT.*PIN_CONFIG_MAX.*/d' \
    drivers/media/i2c/maxim-serdes/max96717.c
  echo "Removed broken PIN_CONFIG_OUTPUT fallback define"
fi
# Always add #ifndef guard after the include (safe even if already defined)
if ! grep -q '#ifndef PIN_CONFIG_OUTPUT' drivers/media/i2c/maxim-serdes/max96717.c; then
  sed -i '/#include <linux\/pinctrl\/pinconf-generic.h>/a \/* PIN_CONFIG_OUTPUT was replaced by PIN_CONFIG_LEVEL in newer kernels *\/\n#ifndef PIN_CONFIG_OUTPUT\n#define PIN_CONFIG_OUTPUT\tPIN_CONFIG_LEVEL\n#endif' \
    drivers/media/i2c/maxim-serdes/max96717.c
  echo "Added PIN_CONFIG_OUTPUT compatibility guard"
fi
```

### 5a-fix2. field_get/field_prep compatibility guards

The `max_serdes.h` header defines its own `field_get()` and `field_prep()` macros without
a guard. Newer kernels (rpi-6.19.y+) also define them in `<linux/bitfield.h>`, which gives
"field_get redefined" warnings. Replace ADI's definitions with guarded ones (same meaning:
mask, then shift by `__ffs(mask)`). The unguarded lines are only removed while there is no
guard yet, so running this twice leaves the file unchanged:

```bash
cd <kernel_src_path>
if ! grep -q '#ifndef field_get' drivers/media/i2c/maxim-serdes/max_serdes.h; then
  # Remove ADI's unguarded definitions
  sed -i '/^#define field_get(mask, val)/d; /^#define field_prep(mask, val)/d' \
    drivers/media/i2c/maxim-serdes/max_serdes.h
  # Add the guarded definitions after the last #include line
  LAST_INCLUDE=$(grep -n '#include' drivers/media/i2c/maxim-serdes/max_serdes.h | tail -1 | cut -d: -f1)
  sed -i "${LAST_INCLUDE}a\\
\\
#ifndef field_get\\
#define field_get(mask, val) (((val) \& (mask)) >> __ffs(mask))\\
#endif\\
#ifndef field_prep\\
#define field_prep(mask, val) (((val) << __ffs(mask)) \& (mask))\\
#endif" drivers/media/i2c/maxim-serdes/max_serdes.h
  echo "Added field_get/field_prep guards"
fi
```

### 5a-fix3. gpio_chip .set_rv → .set adaptation (conditional)

In newer kernels (rpi-6.18.y and later), `gpio_chip.set` already returns `int` (same
signature as `.set_rv`), making `.set_rv` unnecessary. When Step 4 printed `set returns int`,
change max96717.c to use `.set` instead:

```bash
cd <kernel_src_path>
if ! grep -q 'set_rv' include/linux/gpio/driver.h; then
  if grep -q '\.set_rv' drivers/media/i2c/maxim-serdes/max96717.c; then
    sed -i 's/\.set_rv\b/.set/' drivers/media/i2c/maxim-serdes/max96717.c
    echo "Changed .set_rv to .set in max96717.c (kernel has int-returning .set)"
  fi
fi
```

### 5b. ADI sensor drivers

The ADI imx219 is substantially different from the RPi version — it has 3 pads
(SOURCE, IMAGE, EDATA), full V4L2 routing support, and embedded data streaming.
Download it directly:

```bash
curl -sL "${BASE_URL}/drivers/media/i2c/imx219.c" \
  -o "<kernel_src_path>/drivers/media/i2c/imx219.c"
```

Also download ISX021 and dummy-cam if present in the ADI fork:
```bash
curl -sL "${BASE_URL}/drivers/media/i2c/isx021.c" \
  -o "<kernel_src_path>/drivers/media/i2c/isx021.c" 2>/dev/null || true
curl -sL "${BASE_URL}/drivers/media/i2c/dummy-cam.c" \
  -o "<kernel_src_path>/drivers/media/i2c/dummy-cam.c" 2>/dev/null || true
```

### 5c. Missing header defines for ADI imx219 (conditional)

The ADI imx219 uses two defines that may or may not be present in the stock kernel.
Check before adding:

**`V4L2_SUBDEV_ROUTE_FL_IMMUTABLE`** in `include/uapi/linux/v4l2-subdev.h`:
```bash
if ! grep -q 'V4L2_SUBDEV_ROUTE_FL_IMMUTABLE' <kernel_src_path>/include/uapi/linux/v4l2-subdev.h; then
  # Add after V4L2_SUBDEV_ROUTE_FL_ACTIVE
  sed -i '/V4L2_SUBDEV_ROUTE_FL_ACTIVE/a #define V4L2_SUBDEV_ROUTE_FL_IMMUTABLE\t\t(1U << 1)' \
    <kernel_src_path>/include/uapi/linux/v4l2-subdev.h
fi
```

**`MEDIA_BUS_FMT_CCS_EMBEDDED`** in `include/uapi/linux/media-bus-format.h`:
```bash
if ! grep -q 'MEDIA_BUS_FMT_CCS_EMBEDDED' <kernel_src_path>/include/uapi/linux/media-bus-format.h; then
  # Add before the #endif
  sed -i '/#endif.*__LINUX_MEDIA_BUS_FORMAT_H/i \/* MIPI CCS embedded data - next is 0x9002 *\/\n#define MEDIA_BUS_FMT_CCS_EMBEDDED\t\t0x9001\n' \
    <kernel_src_path>/include/uapi/linux/media-bus-format.h
fi
```

**`V4L2_CID_USER_MAXIM_SERDES_BASE`** in `include/uapi/linux/v4l2-controls.h`:
```bash
if ! grep -q 'V4L2_CID_USER_MAXIM_SERDES_BASE' <kernel_src_path>/include/uapi/linux/v4l2-controls.h; then
  # Add after the last V4L2_CID_USER_*_BASE entry
  LAST_BASE=$(grep -n 'V4L2_CID_USER_.*_BASE' <kernel_src_path>/include/uapi/linux/v4l2-controls.h | tail -1 | cut -d: -f1)
  sed -i "${LAST_BASE}a #define V4L2_CID_USER_MAXIM_SERDES_BASE\t\t(V4L2_CID_USER_BASE + 0x2010)" \
    <kernel_src_path>/include/uapi/linux/v4l2-controls.h
  echo "Added V4L2_CID_USER_MAXIM_SERDES_BASE"
fi
```

### 5d. ADI defconfigs (reference only)

Download ADI defconfigs for reference. These are NOT used directly for building
(rpi/2 uses the stock defconfig + GMSL overrides instead), but are useful
for checking which configs ADI enables:

```bash
curl -sL "${BASE_URL}/arch/arm64/configs/bcm2711_adi_gmsl_defconfig" \
  -o "<kernel_src_path>/arch/arm64/configs/bcm2711_adi_gmsl_defconfig"
curl -sL "${BASE_URL}/arch/arm64/configs/bcm2712_adi_gmsl_defconfig" \
  -o "<kernel_src_path>/arch/arm64/configs/bcm2712_adi_gmsl_defconfig"
```

## Step 6: Adapt other sensor drivers for GMSL pipeline (if needed)

The ADI imx219 is downloaded directly (Step 5b) and already has full GMSL support.
For other sensors (e.g., ov5640) that don't have ADI-specific versions, the following
V4L2 pad ops must be added manually:

1. **`get_frame_desc`** — Returns CSI-2 virtual channel and data type
2. **`enable_streams` / `disable_streams`** — Per-stream start/stop callbacks

See the generic patterns below. Only apply these if the sensor is used in the pipeline
AND doesn't already have these ops.

### Generic pattern for `get_frame_desc`

Required include: `#include <media/mipi-csi2.h>`

```c
static int <sensor>_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
                                   struct v4l2_mbus_frame_desc *fd)
{
    struct v4l2_subdev_state *state;
    const struct v4l2_mbus_framefmt *fmt;
    u32 code;

    state = v4l2_subdev_lock_and_get_active_state(sd);
    fmt = v4l2_subdev_state_get_format(state, 0);
    code = fmt->code;
    v4l2_subdev_unlock_state(state);

    fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
    fd->num_entries = 1;
    fd->entry[0].pixelcode = code;
    fd->entry[0].stream = 0;
    fd->entry[0].bus.csi2.vc = 0;

    switch (code) {
    case MEDIA_BUS_FMT_SRGGB8_1X8:
    case MEDIA_BUS_FMT_SGRBG8_1X8:
    case MEDIA_BUS_FMT_SGBRG8_1X8:
    case MEDIA_BUS_FMT_SBGGR8_1X8:
        fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW8;
        break;
    default:
        fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW10;
        break;
    }
    return 0;
}
```

### Generic pattern for `enable_streams` / `disable_streams`

These callbacks receive the active state lock already held by the V4L2 core. The sensor
**must not** call `v4l2_subdev_lock_and_get_active_state()` inside these callbacks.

```c
static int <sensor>_enable_streams(struct v4l2_subdev *sd,
                                   struct v4l2_subdev_state *state, u32 pad,
                                   u64 streams_mask)
{
    struct <sensor>_priv *priv = to_<sensor>(sd);
    return <sensor>_start_streaming(priv, state);
}

static int <sensor>_disable_streams(struct v4l2_subdev *sd,
                                    struct v4l2_subdev_state *state, u32 pad,
                                    u64 streams_mask)
{
    struct <sensor>_priv *priv = to_<sensor>(sd);
    <sensor>_stop_streaming(priv);
    return 0;
}
```

Register in `v4l2_subdev_pad_ops`:
```c
.get_frame_desc = <sensor>_get_frame_desc,
.enable_streams = <sensor>_enable_streams,
.disable_streams = <sensor>_disable_streams,
```

## Step 7: Platform-specific CSI receiver fixes (conditional)

Apply fixes based on the target platform. Each fix is conditional — only applied if the
target code matches the expected pattern.

### RPi-specific fixes (rp1-cfe)

**Fix 1: Compatible string** — The upstream `rp1-cfe` driver may register with compatible
`"raspberrypi,rp1-cfe-upstream"` but the RPi device tree uses `"raspberrypi,rp1-cfe"`.
Only apply if the upstream variant is present:
```bash
cd <kernel_src_path>
if grep -q 'rp1-cfe-upstream' drivers/media/platform/raspberrypi/rp1-cfe/cfe.c; then
  sed -i 's/"raspberrypi,rp1-cfe-upstream"/"raspberrypi,rp1-cfe"/' \
    drivers/media/platform/raspberrypi/rp1-cfe/cfe.c
  echo "Fixed rp1-cfe compatible string"
fi
```

**Fix 2: Link frequency pad API** — Older kernels may call
`v4l2_get_link_freq(cfe->source_sd->ctrl_handler, ...)`. Only apply if the old form is
present (newer kernels already use the pad-based API):
```bash
cd <kernel_src_path>
if grep -q 'v4l2_get_link_freq(cfe->source_sd->ctrl_handler' drivers/media/platform/raspberrypi/rp1-cfe/cfe.c; then
  sed -i 's/v4l2_get_link_freq(cfe->source_sd->ctrl_handler,/v4l2_get_link_freq(\&cfe->source_sd->entity.pads[cfe->source_pad],/' \
    drivers/media/platform/raspberrypi/rp1-cfe/cfe.c
  echo "Fixed rp1-cfe link frequency API"
fi
```

## Step 8: Enable V4L2 Streams API

The V4L2 streams routing API (`VIDIOC_SUBDEV_S_ROUTING`, `VIDIOC_SUBDEV_G_ROUTING`)
is required by the GMSL pipeline for configuring multi-stream routes through the
deserializer → serializer → sensor chain.

In stock RPi kernels, the streams API is disabled by default (experimental flag).
Enable it:

```bash
cd <kernel_src_path>
if grep -q 'v4l2_subdev_enable_streams_api;$' drivers/media/v4l2-core/v4l2-subdev.c; then
  sed -i 's/static bool v4l2_subdev_enable_streams_api;/static bool v4l2_subdev_enable_streams_api = true;/' \
    drivers/media/v4l2-core/v4l2-subdev.c
  echo "Enabled V4L2 streams API"
elif grep -q 'v4l2_subdev_enable_streams_api = true' drivers/media/v4l2-core/v4l2-subdev.c; then
  echo "V4L2 streams API already enabled"
elif ! grep -q 'v4l2_subdev_enable_streams_api' drivers/media/v4l2-core/v4l2-subdev.c; then
  echo "V4L2 streams API always enabled (the experimental flag was removed)"
else
  echo "WARNING: unexpected v4l2_subdev_enable_streams_api definition — check v4l2-subdev.c"
fi
```

The ADI patch "media: uapi: v4l: subdev: Enable streams API" (applied in Step 3 when it
fits) removes the flag entirely, so on such trees there is nothing to change here.

Without this fix, `video_cfg.sh` (cam.py) fails with `ENOTTY` (errno 25, "Inappropriate
ioctl for device") when trying to set routes on sensor subdevices.

## Step 9: Ensure Kconfig/Makefile entries for GMSL drivers

The backport patches may or may not have added the Kconfig and Makefile entries for the
drivers downloaded in Step 5. Add whatever is missing, the same way the ADI tree has them:
- Newer kernels (rpi-6.19.y+) ship upstream `max96714`/`max96717` drivers that use the same
  `CONFIG_VIDEO_MAX96717` symbol as maxim-serdes, so both drivers would be built. ADI removes
  them (patches "remove MAX96717/MAX96714 driver"); do the same.
- The `VIDEO_DUMMY_CAM` and `VIDEO_ISX021` entries go with the camera sensors.
- The maxim-serdes Kconfig is sourced exactly once, at the end of the "Video serializers and
  deserializers" menu. (A `sed '/^endmenu/i ...'` would insert it before every `endmenu` in
  the file.)

Every change is skipped when it is already present, so running this twice is safe:

```bash
cd <kernel_src_path>
python3 -I - <<'EOF'
import re

KCONFIG = 'drivers/media/i2c/Kconfig'
MAKEFILE = 'drivers/media/i2c/Makefile'

def entry(sym, text):
    return re.search(r'^config %s\n(?:(?:\t.*|)\n)*?(?=^\S)' % sym, text, re.M)

k = open(KCONFIG).read()

# The upstream MAX96714/MAX96717 drivers use the same CONFIG_VIDEO_MAX96717 symbol as
# maxim-serdes; remove them, as ADI does (patches "remove MAX96717/MAX96714 driver").
for sym in ('VIDEO_MAX96714', 'VIDEO_MAX96717'):
    m = entry(sym, k)
    if m:
        k = k[:m.start()] + k[m.end():]
        print('Removed upstream', sym)

# ADI sensor drivers downloaded in Step 5b, placed with the camera sensors.
SENSORS = {
    'VIDEO_DUMMY_CAM': '''config VIDEO_DUMMY_CAM
\ttristate "Dummy sensor support"
\tdepends on I2C && VIDEO_DEV
\tselect MEDIA_CONTROLLER
\tselect V4L2_FWNODE
\tselect VIDEO_V4L2_SUBDEV_API
\thelp
\t  This is a Video4Linux2 dummy sensor driver. It does not
\t  perform any I2C bus writes.

\t  To compile this driver as a module, choose M here: the
\t  module will be called dummy-cam.

''',
    'VIDEO_ISX021': '''config VIDEO_ISX021
\ttristate "Sony ISX021 sensor support"
\tdepends on I2C && VIDEO_DEV
\tselect VIDEO_V4L2_SUBDEV_API
\tselect MEDIA_CONTROLLER
\tselect V4L2_FWNODE
\thelp
\t  This is a Video4Linux2 sensor driver for the Sony
\t  ISX021 camera.

\t  To compile this driver as a module, choose M here: the
\t  module will be called isx021.

''',
}
for sym, text in SENSORS.items():
    if not entry(sym, k):
        anchor = k.index('config VIDEO_IMX219\n')
        k = k[:anchor] + text + k[anchor:]
        print('Added', sym)

# maxim-serdes Kconfig: exactly once, at the end of the serializer/deserializer menu
# (the last menu in the file on kernels that do not have it).
SOURCE = 'source "drivers/media/i2c/maxim-serdes/Kconfig"'
if SOURCE not in k:
    menu = k.find('menu "Video serializers and deserializers"')
    end = k.index('\nendmenu', menu) if menu >= 0 else k.rindex('\nendmenu')
    k = k[:end] + '\n' + SOURCE + '\n' + k[end:]
    print('Added maxim-serdes Kconfig entry')
open(KCONFIG, 'w').write(k)

mk = open(MAKEFILE).read().splitlines(keepends=True)
mk = [l for l in mk if l not in ('obj-$(CONFIG_VIDEO_MAX96714) += max96714.o\n',
                                 'obj-$(CONFIG_VIDEO_MAX96717) += max96717.o\n')]
for line in ('obj-$(CONFIG_VIDEO_DUMMY_CAM) += dummy-cam.o\n',
             'obj-$(CONFIG_VIDEO_ISX021) += isx021.o\n',
             'obj-$(CONFIG_VIDEO_MAXIM_SERDES) += maxim-serdes/\n'):
    if line not in mk:
        mk.append(line)
        print('Added', line.strip())
open(MAKEFILE, 'w').write(''.join(mk))
EOF
```

## Step 10: Verify driver files

```bash
ls <kernel_src_path>/drivers/media/i2c/maxim-serdes/
```

Expected maxim-serdes contents: `Kconfig`, `Makefile`, `max9296a.c`, `max96717.c`,
`max96724.c`, `max_des.c`, `max_des.h`, `max_ser.c`, `max_ser.h`, `max_serdes.c`,
`max_serdes.h`

## Step 11: Update state

```bash
python3 -I -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['drivers_added'] = True
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```

## Step 12: Cleanup

```bash
rm -rf /tmp/gmsl_adi_linux_check /tmp/gmsl_patches /tmp/gmsl_backport_commits.txt
```

## What the changes modify

| Area | Method | Files | Purpose |
|------|--------|-------|---------|
| GPIO backport | Conditional manual patch | `drivers/gpio/gpiolib.c`, `include/linux/gpio/driver.h` | `gpio_chip.set_rv` API needed by MAX96717 driver (skipped if already present) |
| I2C ATR | Backport patch | `drivers/i2c/i2c-atr.c`, `include/linux/i2c-atr.h` | Address Translation Router — required for GMSL I2C tunneling |
| Media controller | Backport patch | `drivers/media/mc/mc-entity.c`, `include/uapi/linux/media.h` | `MEDIA_PAD_FL_INTERNAL` flag |
| V4L2 core | Backport patch | `drivers/media/v4l2-core/v4l2-*.c`, `include/media/v4l2-*.h` | Link frequency, mbus config, fwnode VC IDs, streams API |
| ds90ub960 | Backport patch | `drivers/media/i2c/ds90ub9*.c` | Updated to work with new I2C ATR API |
| FPC202 | Backport patch | `drivers/misc/ti_fpc202.c` | TI FPC202 dual-port controller |
| rp1-cfe | Backport patch + conditional sed | `drivers/media/platform/raspberrypi/rp1-cfe/*` | CSI2 receiver fixes for GMSL streams |
| Kconfig/Makefile | Backport patch + conditional add | `drivers/media/i2c/Kconfig`, `Makefile` | Build system entries for maxim-serdes, ISX021 and dummy-cam (each added once) |
| Upstream MAX96714/MAX96717 | Conditional remove | `drivers/media/i2c/Kconfig`, `Makefile` | Same `CONFIG_VIDEO_MAX96717` symbol as maxim-serdes; removed as in the ADI tree |
| GMSL drivers | Direct download | `drivers/media/i2c/maxim-serdes/*` | MAX96717, MAX96724, MAX9296A ser/des drivers |
| Sensor drivers | Direct download | `drivers/media/i2c/imx219.c`, `isx021.c` | ADI versions with routing + embedded data |
| Defconfigs | Direct download (reference) | `arch/arm64/configs/bcm27*_adi_gmsl_defconfig` | Reference configs (not used for building) |
| Header defines | Conditional add | `include/uapi/linux/v4l2-subdev.h`, `media-bus-format.h`, `v4l2-controls.h` | `ROUTE_FL_IMMUTABLE`, `CCS_EMBEDDED`, `MAXIM_SERDES_BASE` (skipped if present) |
| PIN_CONFIG_OUTPUT | Conditional sed | `drivers/media/i2c/maxim-serdes/max96717.c` | Fix 8-bit overflow: `PIN_CONFIG_OUTPUT` → `PIN_CONFIG_LEVEL` in newer kernels |
| field_get/prep | Conditional replace | `drivers/media/i2c/maxim-serdes/max_serdes.h` | Guard ADI's `field_get()`/`field_prep()` so they do not redefine the `<linux/bitfield.h>` ones |
| set_rv → set | Conditional sed | `drivers/media/i2c/maxim-serdes/max96717.c` | Use `.set` instead of `.set_rv` when kernel already has int-returning `.set` |
| Streams API | Conditional sed | `drivers/media/v4l2-core/v4l2-subdev.c` | Enable `v4l2_subdev_enable_streams_api` (disabled by default in stock kernels) |
| Partial patches | Dry-run check | All patched files | Each patch is applied all-or-nothing, so a failed patch leaves no half-changed files |
