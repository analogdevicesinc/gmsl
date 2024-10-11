# V4L2 Player

`v4l2_player.py` is a Python script that captures video from a V4L2 (Video4Linux2) device and displays it using OpenCV. It supports various command-line arguments to customize the video capture and display settings.

## Requirements

- Python 3.x
- v4l2-python3
- OpenCV
- NumPy

## Installation

1. Install the required Python packages:
    ```sh
    pip install opencv-python numpy v4l2-python3
    ```

## Usage

Run the script with the following command-line arguments:

```sh
python 

v4l2_player.py

 [options]
```

### Options

- `--device`, `-d`: Specify the video device (default: `/dev/video2`).
- `--width`, `-x`: Set the width of the video capture (default: `1920`).
- `--height`, `-y`: Set the height of the video capture (default: `1080`).
- `--position`, `-p`: Set the position of the display window in the format `x,y` (default: `0,0`).
- `--fullscreen`, `-f`: Enable fullscreen mode (default: `False`).
- `--capture`, `-c`: Enable frame capture to a PNG file (default: `False`).

### Example

```sh
python 

v4l2_player.py

 --device /dev/video0 --width 1280 --height 720 --fullscreen
```

## License

This project is licensed under the MIT License.