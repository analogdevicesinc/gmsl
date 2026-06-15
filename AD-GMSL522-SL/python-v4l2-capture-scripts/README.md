# V4L2 Player

`v4l2_player.py` is a Python script that captures video from a V4L2 (Video4Linux2) device and displays it using OpenCV. It supports various command-line arguments to customize the video capture and display settings.

## Requirements

- Python 3.x
- v4l2-python3
- OpenCV
- NumPy

## Installation

Install the required Python packages:

```sh
pip install opencv-python numpy v4l2-python3
```

## Usage

```sh
python v4l2_player.py [options]
```

### Options

| Option | Short | Default | Description |
|--------|-------|---------|-------------|
| `--device` | `-d` | `/dev/video2` | Video device path |
| `--width` | `-x` | `1920` | Capture width in pixels |
| `--height` | `-y` | `1080` | Capture height in pixels |
| `--bpp` | `-b` | `4` | Bytes per pixel (1, 2, 3, or 4) |
| `--position` | `-p` | `0,0` | Window position as `x,y` |
| `--fullscreen` | `-f` | off | Enable fullscreen mode |
| `--raw` | `-r` | off | Bayer sensor mode (applies debayering) |
| `--capture` | `-c` | off | Save a frame to PNG every 100 frames |
| `--tegra` | `-t` | auto | Force Tegra mode for RAW10/RAW12/RAW14 left-aligned Bayer data |

### Bytes per pixel modes

| `--bpp` | Format | Notes |
|---------|--------|-------|
| 1 | 8-bit Bayer | Debayered with `COLOR_BAYER_BG2BGR` |
| 2 | YUYV (default) or Bayer (`--raw`) | See Bayer section below |
| 3 | BGR | Displayed directly |
| 4 | BGRA | Alpha set to 255, converted to BGR |

### Bayer sensor mode (`--raw`)

Use `--raw` with `--bpp 2` for raw Bayer sensors (e.g., IMX219). The script handles two cases:

- **Tegra platforms** (auto-detected or `--tegra`): The Tegra VI capture interface delivers RAW10, RAW12, or RAW14 Bayer data left-aligned in 16-bit words, with the MSBs replicated in the lower bits. The script reads the buffer as `uint16`, shifts right by 8 to extract the top 8 bits, then applies Bayer-to-BGR debayering. This works for all three bit depths since the data is always left-aligned to bit 15.
- **Non-Tegra platforms**: The buffer is read as a 2-channel `uint8` array and debayered directly.

Tegra mode is auto-detected by checking for `/sys/bus/platform/drivers/tegra-camrtc-capture-vi`. Use `--tegra` to force it on if auto-detection fails.

### Examples

Standard BGRA capture:
```sh
python v4l2_player.py -d /dev/video0 -x 1280 -y 720 -b 4 -f
```

Raw Bayer sensor on Tegra (e.g., GMSL camera with IMX219):
```sh
python v4l2_player.py -d /dev/video0 -x 1920 -y 1080 -b 2 -r
```

## License

This project is licensed under the MIT License.
