# RPi 2: Cross-compile the kernel

## Purpose
Cross-compile the Linux kernel with ADI GMSL drivers built into the kernel image
(not as loadable modules) for the target platform.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save a copy of the kernel build log and the config changes in `<run_dir>/host/` (§Verbose mode §What to save).

## Prerequisites
Read from `<state>` (`/tmp/gmsl_camera_state.json`): `kernel_src_path` (`<work_dir>/rpi-linux`),
`rpi_model`. Run after rpi/1.

## Step 1: Verify cross-compiler is available

```bash
aarch64-linux-gnu-gcc --version
```

If this fails, the cross-compiler was not installed by common/0 §Host tools. Install it (sudo as
in common/0 §Sudo password):
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y gcc-aarch64-linux-gnu
```

## Step 2: Select defconfig

Use the **stock RPi defconfig** (not the ADI GMSL defconfig). GMSL-specific config
options are applied as overrides in Step 4b. This ensures the kernel configuration
matches the stock RPi OS as closely as possible.

Based on the RPi model from state:
- RPi 4: `bcm2711_defconfig`
- RPi 5: `bcm2712_defconfig`

## Step 3: Configure the kernel

```bash
cd <kernel_src_path>
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- <defconfig>
```

Verify `.config` was created:
```bash
test -f <kernel_src_path>/.config && echo "ok" || echo "FAILED"
```

## Step 4: Adjust kernel config for GMSL pipeline

After generating `.config` from defconfig, apply the following adjustments.
These are critical for the GMSL pipeline to work correctly.

**Important**: `olddefconfig` must run FIRST, then config overrides are applied AFTER.
Running `olddefconfig` between the overrides (4b, 4c) and the forcing of 4d reverts some values
(e.g., `CONFIG_VIDEO_MAXIM_SERDES` gets changed from `=y` back to `=m` due to Kconfig
dependency resolution). It runs once more only at the end of 4d, after the forcing, so that the
build's own config sync (Step 5) finds no new options to ask about; the check after it must still
show `=y` for every option.

### 4a. Resolve Kconfig dependencies first

```bash
cd <kernel_src_path>
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig
```

### 4b. Build GMSL and sensor drivers as built-in (=y)

Building drivers directly into the kernel image (`=y`) instead of as loadable modules
(`=m`) eliminates module deployment issues: no `.ko` vs `.ko.xz` priority conflicts,
no `modules_install` ordering problems, and no risk of stale modules on the target
filesystem. The drivers are always present as soon as the kernel boots.

```bash
cd <kernel_src_path>
# Force media subsystem dependencies to =y — required so that downstream
# GMSL/sensor configs can also be =y (a tristate depending on =m can't be =y)
./scripts/config --set-val CONFIG_MEDIA_SUPPORT y
./scripts/config --set-val CONFIG_VIDEO_DEV y
./scripts/config --set-val CONFIG_I2C_MUX y
./scripts/config --set-val CONFIG_VIDEO_V4L2_SUBDEV_API y
./scripts/config --set-val CONFIG_V4L2_FWNODE y
./scripts/config --set-val CONFIG_V4L2_ASYNC y
./scripts/config --set-val CONFIG_MEDIA_CONTROLLER y
./scripts/config --set-val CONFIG_VIDEOBUF2_CORE y
./scripts/config --set-val CONFIG_VIDEOBUF2_V4L2 y
./scripts/config --set-val CONFIG_VIDEOBUF2_MEMOPS y
./scripts/config --set-val CONFIG_VIDEOBUF2_DMA_CONTIG y

# GMSL serializer/deserializer drivers
./scripts/config --set-val CONFIG_VIDEO_MAXIM_SERDES y
./scripts/config --set-val CONFIG_VIDEO_MAX9296A y
./scripts/config --set-val CONFIG_VIDEO_MAX96717 y
./scripts/config --set-val CONFIG_VIDEO_MAX96724 y

# I2C ATR (required by GMSL I2C tunneling)
./scripts/config --set-val CONFIG_I2C_ATR y

# Camera sensor drivers used in the pipeline
./scripts/config --set-val CONFIG_VIDEO_IMX219 y
./scripts/config --set-val CONFIG_VIDEO_OV5640 y

# Platform CSI receiver (upstream, streams-capable version)
./scripts/config --set-val CONFIG_VIDEO_RP1_CFE y

# Kernel debugging: dynamic debug (pr_debug/dev_dbg of the GMSL drivers can be switched on at
# run time through /sys/kernel/debug/dynamic_debug/control) and function tracing (ftrace).
# DYNAMIC_FTRACE depends on FUNCTION_TRACER: without it, the build's own config sync turns
# DYNAMIC_FTRACE off again. bcm2712_defconfig of rpi-6.18.y already has FTRACE, FUNCTION_TRACER
# and DYNAMIC_FTRACE (=y) and IKCONFIG_PROC (=y, with IKCONFIG=m); DYNAMIC_DEBUG is new, and
# IKCONFIG goes from =m to =y. Older branches and bcm2711_defconfig can differ: set them all.
./scripts/config --set-val CONFIG_DYNAMIC_DEBUG y
./scripts/config --set-val CONFIG_FTRACE y
./scripts/config --set-val CONFIG_FUNCTION_TRACER y
./scripts/config --set-val CONFIG_DYNAMIC_FTRACE y

# Kernel config readable on the board (/proc/config.gz), for common/2 and debugging
./scripts/config --set-val CONFIG_IKCONFIG y
./scripts/config --set-val CONFIG_IKCONFIG_PROC y
```

### 4c. Disable conflicting drivers and options

```bash
cd <kernel_src_path>
# Disable downstream CSI receiver — it lacks V4L2_SUBDEV_FL_STREAMS and set_routing,
# causing "Dangling sink streams" errors when the GMSL deserializer validates links.
./scripts/config --disable CONFIG_VIDEO_RP1_CFE_DOWNSTREAM

# Disable MODVERSIONS — the GPIO backport patches change symbol signatures (e.g.,
# gpio_chip.set_rv), causing CRC mismatches at module load time. Since GMSL drivers
# are built-in this mostly affects other modules, but disabling it prevents any
# version mismatch errors.
./scripts/config --disable CONFIG_MODVERSIONS
```

### 4d. Verify and force critical config values

`scripts/config` may not override all values if Kconfig dependencies interfere.
Verify and force with `sed` if needed:

```bash
cd <kernel_src_path>
# Check for any critical configs that are =m instead of =y
CFGS="CONFIG_MEDIA_SUPPORT CONFIG_VIDEO_DEV CONFIG_I2C_MUX
      CONFIG_VIDEO_V4L2_SUBDEV_API CONFIG_V4L2_FWNODE CONFIG_V4L2_ASYNC
      CONFIG_MEDIA_CONTROLLER CONFIG_VIDEOBUF2_CORE CONFIG_VIDEOBUF2_V4L2
      CONFIG_VIDEOBUF2_MEMOPS CONFIG_VIDEOBUF2_DMA_CONTIG
      CONFIG_VIDEO_MAXIM_SERDES CONFIG_VIDEO_MAX9296A CONFIG_VIDEO_MAX96717
      CONFIG_VIDEO_MAX96724 CONFIG_I2C_ATR CONFIG_VIDEO_IMX219
      CONFIG_VIDEO_RP1_CFE CONFIG_DYNAMIC_DEBUG CONFIG_FTRACE
      CONFIG_FUNCTION_TRACER CONFIG_DYNAMIC_FTRACE CONFIG_IKCONFIG CONFIG_IKCONFIG_PROC"
for cfg in $CFGS; do
  val=$(grep "^${cfg}=" .config | cut -d= -f2)
  if [ "$val" = "m" ]; then
    echo "Forcing $cfg from =m to =y"
    sed -i "s/^${cfg}=m/${cfg}=y/" .config
  fi
done

# Give the options that the overrides made visible their defaults now. Without this, the build's
# own config sync asks for them (FW_LOADER_DEBUG, TEST_DYNAMIC_DEBUG, ...), cannot read an answer,
# prints "Error in reading or end of file." and takes the defaults anyway.
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- olddefconfig

for cfg in $CFGS; do echo "$cfg=$(grep "^${cfg}=" .config | cut -d= -f2)"; done
grep -E 'CONFIG_VIDEO_RP1_CFE_DOWNSTREAM|CONFIG_MODVERSIONS' .config
```

Expected output: `=y` for every option of the list (all 24: media core, GMSL/sensor/CSI, the
debugging options and IKCONFIG), and `not set` for `CONFIG_VIDEO_RP1_CFE_DOWNSTREAM` and
`CONFIG_MODVERSIONS`. An empty value (`CONFIG_X=`) means the option does not exist in this
kernel: stop and report it. If an option is back at `=m` after the
`olddefconfig`, stop and report it (its dependency is a module: name the option and what
`grep -n '<option>' .config` shows).

### 4e. Verify V4L2 streams API is enabled

The streams API must be enabled in the source code (done in rpi/1, Step 8).
Verify before building:

```bash
cd <kernel_src_path>
if grep -q 'v4l2_subdev_enable_streams_api = true' drivers/media/v4l2-core/v4l2-subdev.c; then
  echo "Streams API: enabled"
elif ! grep -q 'v4l2_subdev_enable_streams_api' drivers/media/v4l2-core/v4l2-subdev.c; then
  echo "Streams API: always enabled (experimental flag removed)"
else
  echo "WARNING: Streams API NOT enabled — GMSL routing will fail at runtime"
  echo "Run: sed -i 's/static bool v4l2_subdev_enable_streams_api;/static bool v4l2_subdev_enable_streams_api = true;/' drivers/media/v4l2-core/v4l2-subdev.c"
fi
```

## Step 5: Build kernel image, modules, and DTBs

```bash
cd <kernel_src_path>
make -j$(nproc) ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs
```

This will take several minutes depending on the host machine. The `-j$(nproc)` flag
uses all available CPU cores.

If the build fails, check the error output. Common issues:
- Missing `libssl-dev`: install with `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y libssl-dev`
- Missing `flex`/`bison`: install with `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y flex bison`

## Step 6: Verify build output

Check that the key artifacts exist:
```bash
test -f <kernel_src_path>/arch/arm64/boot/Image && echo "Image: ok" || echo "Image: MISSING"
ls <kernel_src_path>/arch/arm64/boot/dts/broadcom/*.dtb >/dev/null 2>&1 && echo "DTBs: ok" || echo "DTBs: MISSING"
ls <kernel_src_path>/arch/arm64/boot/dts/overlays/*.dtbo >/dev/null 2>&1 && echo "Overlays: ok" || echo "Overlays: MISSING"
```

Verify that GMSL drivers are built-in (not modules) and that the debugging options survived the
build's config sync:
```bash
grep -E 'CONFIG_VIDEO_MAXIM_SERDES|CONFIG_VIDEO_MAX96724|CONFIG_VIDEO_IMX219|CONFIG_DYNAMIC_DEBUG=|CONFIG_FTRACE=|CONFIG_FUNCTION_TRACER=|CONFIG_DYNAMIC_FTRACE=|CONFIG_IKCONFIG=|CONFIG_IKCONFIG_PROC=' <kernel_src_path>/.config
```

All should show `=y`. If any GMSL driver shows `=m`, the driver will need `modules_install` and
module file management on the target — go back to Step 4 and fix. If a debugging option is
missing, its dependency was not met on this kernel (for `CONFIG_DYNAMIC_FTRACE`:
`CONFIG_FUNCTION_TRACER`; read its `depends on` lines in `kernel/trace/Kconfig` or
`lib/Kconfig.debug`): tell the user,
it does not affect the camera.

Report the kernel version:
```bash
make -C <kernel_src_path> -s ARCH=arm64 kernelrelease
```

## Step 7: Update state

```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['kernel_built'] = True
state['kernel_release'] = '<the release printed by Step 6, e.g. 6.18.50-v8-16k+>'
state['localversion'] = '<the CONFIG_LOCALVERSION of .config, e.g. -v8-16k>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
