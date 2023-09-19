# Gigabit Multimedia Serial Link™ (GMSL) technology from Analog Devices Inc.

## Overview
[Gigabit Multimedia Serial Link™ (GMSL)][ref-0] technology from Analog Devices, Inc. (ADI) reliably transports high resolution digital video data 
from cameras to computers, between computers, and from computers to displays, enabling advanced vision and infotainment applications for automotive, 
agriculture, industrial automation, shipping, and more.

This repository is the root starting point for GMSL technology open-source software provided by ADI.

[ref-0]: https://www.analog.com/en/product-category/gigabit-multimedia-serial-link.html

## Supported Development Platforms

Software support si provided for multiple embedded platforms to enable evaluation and prototyping with ADI's GMSL technology on your
development system of choice.

The table below contains references to the documentation and SD card images for all the supported development platforms. 

The **User Guides** provide details on hardware setup and testing, while the **Build Instructions** contain information about 
getting the source code, setting up the development environments and building the code.

For faster evaluation check out the **SD Card Image** for your platform.
More details on how to extract a compressed image and write it on the SD card on Linux and Windows can be found [here][sdcard-burn].

| Platform                               | Documentation                                      | Downloads                   |
| -------------------------------------- | ---------------------------------------------------| --------------------------- |
| [Raspberry Pi 4][pl-0]                 | [User Guide][ug-0] <br> [Build Instructions][bi-0] | [![Download][dlsh-0]][dl-0] <br> [![sha256sum][shash-0]][sha-0] |
| [AMD KV260][pl-1]                      | [User Guide][ug-1] <br> [Build Instructions][bi-1] | [![Download][dlsh-1]][dl-1] <br> [![sha256sum][shash-0]][sha-1] |
| [Nvidia Orin Nano Developer Kit][pl-2] | [User Guide][ug-2] <br> [Build Instructions][bi-2] | [![Download][dlsh-2]][dl-2] <br> [![sha256sum][shash-0]][sha-2] |

[sdcard-burn]: http://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/sdcard_burn.md

[pl-0]: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/
[pl-1]: https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html
[pl-2]: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

[ug-0]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_rpi
[ug-1]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_amd_kria
[ug-2]: https://wiki.analog.com/resources/eval/user-guides/ad-gmslcamrpi-adp/ug_nvidia_jetson_orin_nano

[bi-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/README-GMSL.md
[bi-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/README-GMSL.md
[bi-2]: https://github.com/analogdevicesinc/nvidia/tree/gmsl/main/README.md

[dlsh-0]: https://img.shields.io/badge/release-RPI_SD_Card_Image-blue.svg
[dlsh-1]: https://img.shields.io/badge/release-KV260_SD_Card_Image-blue.svg
[dlsh-2]: https://img.shields.io/badge/release-Nvidia_Orin_Nano_SD_Card_Image-blue.svg

[dl-0]: https://swdownloads.analog.com/cse/gmsl/gmsl-kuiper-rpi-g67e0a5b77cd6.tar.xz
[dl-1]: https://swdownloads.analog.com/cse/gmsl/gmsl-kria-2eb64424cc31.tar.xz
[dl-2]: https://swdownloads.analog.com/cse/aditof/gmsl-nvidia-orin-nano.img.tar.xz

[shash-0]: https://img.shields.io/badge/sha256sum-yellow.svg

[sha-0]: https://swdownloads.analog.com/cse/gmsl/gmsl-kuiper-rpi-g67e0a5b77cd6-sha256sum.txt
[sha-1]: https://swdownloads.analog.com/cse/gmsl/gmsl-kria-2eb64424cc31-sha256sum.txt
[sha-2]: https://swdownloads.analog.com/cse/aditof/gmsl-nvidia-orin-nano-sha256sum.txt
