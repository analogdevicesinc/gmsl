# GMSL Configuration Script

This script configures the GMSL (Gigabit Multimedia Serial Link) using the provided configuration files.

## Prerequisites

- Python 3.x
- `smbus2` library
- `gmsl_lib` library (hosted on this repo)

## Installation

1. Install the required Python libraries:
    ```sh
    pip install smbus2
    ```

2. Ensure the `gmsl_lib` module is accessible. Make sure it's in the parent directory or adjust the `sys.path.append('..')` line accordingly.

## Usage

1. Place your configuration files in the `gmsl_scripts_examples` directory.
2. Update the `IN_FILES` list in `example.py` with the paths to your configuration files.
3. Run the script:
    ```sh
    python example.py
    ```

## Script Details

- **I2C_BUS_NR**: The I2C bus number to use.
- **IN_FILES**: A list of file paths for the configuration files.

## Example

```sh
python example.py
