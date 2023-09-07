# Gigabit Multimedia Serial Link (GMSL) technology from Analog Devices, Inc. (ADI)

## Overview
Gigabit Multimedia Serial Link (GMSL™) technology from Analog Devices, Inc. (ADI) is providing reliable transport of high resolution digital video data
over coaxial or twisted pair serial links. This repository is the root starting point for GMSL technology open source software provided by ADI.

## Platforms, User Guides, SD Card Images, and Build Instructions

For faster evaluation check out the **SD Card Image** for your platform.

More details on how to extract a compressed image and write it on the SD card on Linux and Windows can be found [here][sdcard-burn].

[sdcard-burn]: http://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/sdcard_burn.md

For more details on hardware setup and testing check out the **User Guide** for your platform.

For more details on development environment setup and building check out the **Build Instructions** for your platform.

| Platform                       | User Guide         | SD Card Image               | Build Instructions         |
| ------------------------------ | ------------------ | --------------------------- | -------------------------- |
| Raspberry Pi                   | [User Guide][ug-0] | [![Download][dlsh-0]][dl-0] | [Build Instructions][bi-0] |
| AMD KV260                      | [User Guide][ug-1] | [![Download][dlsh-2]][dl-1] | [Build Instructions][bi-1] |
| Nvidia Orin Nano Developer Kit | [User Guide][ug-2] | [![Download][dlsh-3]][dl-2] | [Build Instructions][bi-2] |

[ug-0]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_rpi
[ug-1]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_amd_kria
[ug-2]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_nvidia_jetson_orin_nano

[bi-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/README-GMSL.md
[bi-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/README-GMSL.md
[bi-2]: https://github.com/analogdevicesinc/nvidia/tree/gmsl/main/README.md

[dlsh-0]: https://img.shields.io/badge/release-RPI_Image-blue.svg
[dlsh-1]: https://img.shields.io/badge/release-KV260_Image-blue.svg
[dlsh-2]: https://img.shields.io/badge/release-Nvidia_Orin_Nano_Image-blue.svg

[dl-0]: #none
[dl-1]: #none
[dl-2]: #none
