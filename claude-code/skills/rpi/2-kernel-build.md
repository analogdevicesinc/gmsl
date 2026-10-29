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
echo '<pw>' | sudo -S apt-get install -y gcc-aarch64-linux-gnu
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
Running `olddefconfig` after `scripts/config` reverts some values (e.g.,
`CONFIG_VIDEO_MAXIM_SERDES` gets changed from `=y` back to `=m` due to Kconfig
dependency resolution). Do NOT run `olddefconfig` again after setting overrides.

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
./scripts/config --set-val CONFIG_V4L2_SUBDEV_API y
./scripts/config --set-val CONFIG_V4L2_FWNODE y
./scripts/config --set-val CONFIG_V4L2_ASYNC y
./scripts/config --set-val CONFIG_MC y
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
```

### 4c. Disable conflicting drivers and options

```bash
cd <kernel_src_path>
# Disable downstream CSI receiver — it lacks V4L2_SUBDEV_FL_STREAMS and set_routing,
# causing "Dangling sink streams" errors when the GMSL deserializer validates links.
./scripts/config --set-val CONFIG_VIDEO_RP1_CFE_DOWNSTREAM n

# Disable MODVERSIONS — the GPIO backport patches change symbol signatures (e.g.,
# gpio_chip.set_rv), causing CRC mismatches at module load time. Since GMSL drivers
# are built-in this mostly affects other modules, but disabling it prevents any
# version mismatch errors.
./scripts/config --set-val CONFIG_MODVERSIONS n
```

### 4d. Verify and force critical config values

`scripts/config` may not override all values if Kconfig dependencies interfere.
Verify and force with `sed` if needed:

```bash
cd <kernel_src_path>
# Check for any critical configs that are =m instead of =y
for cfg in CONFIG_MEDIA_SUPPORT CONFIG_VIDEO_DEV CONFIG_I2C_MUX \
           CONFIG_V4L2_SUBDEV_API CONFIG_V4L2_FWNODE CONFIG_V4L2_ASYNC \
           CONFIG_MC CONFIG_VIDEOBUF2_CORE CONFIG_VIDEOBUF2_V4L2 \
           CONFIG_VIDEOBUF2_MEMOPS CONFIG_VIDEOBUF2_DMA_CONTIG \
           CONFIG_VIDEO_MAXIM_SERDES CONFIG_VIDEO_MAX9296A CONFIG_VIDEO_MAX96717 \
           CONFIG_VIDEO_MAX96724 CONFIG_I2C_ATR CONFIG_VIDEO_IMX219 \
           CONFIG_VIDEO_RP1_CFE; do
  val=$(grep "^${cfg}=" .config | cut -d= -f2)
  if [ "$val" = "m" ]; then
    echo "Forcing $cfg from =m to =y"
    sed -i "s/^${cfg}=m/${cfg}=y/" .config
  fi
done

grep -E 'CONFIG_VIDEO_MAXIM_SERDES|CONFIG_VIDEO_MAX9296A|CONFIG_VIDEO_MAX96717|CONFIG_VIDEO_MAX96724|CONFIG_I2C_ATR|CONFIG_VIDEO_IMX219|CONFIG_VIDEO_RP1_CFE|CONFIG_VIDEO_RP1_CFE_DOWNSTREAM|CONFIG_MODVERSIONS' .config
```

Expected output should show `=y` for all GMSL/sensor/CSI configs, `not set` for
`CONFIG_VIDEO_RP1_CFE_DOWNSTREAM` and `CONFIG_MODVERSIONS`.

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
- Missing `libssl-dev`: install with `echo '<pw>' | sudo -S apt-get install -y libssl-dev`
- Missing `flex`/`bison`: install with `echo '<pw>' | sudo -S apt-get install -y flex bison`

## Step 6: Verify build output

Check that the key artifacts exist:
```bash
test -f <kernel_src_path>/arch/arm64/boot/Image && echo "Image: ok" || echo "Image: MISSING"
ls <kernel_src_path>/arch/arm64/boot/dts/broadcom/*.dtb >/dev/null 2>&1 && echo "DTBs: ok" || echo "DTBs: MISSING"
ls <kernel_src_path>/arch/arm64/boot/dts/overlays/*.dtbo >/dev/null 2>&1 && echo "Overlays: ok" || echo "Overlays: MISSING"
```

Verify that GMSL drivers are built-in (not modules):
```bash
grep -E 'CONFIG_VIDEO_MAXIM_SERDES|CONFIG_VIDEO_MAX96724|CONFIG_VIDEO_IMX219' <kernel_src_path>/.config
```

All should show `=y`. If any show `=m`, the driver will need `modules_install` and
module file management on the target — go back to Step 4 and fix.

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
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
