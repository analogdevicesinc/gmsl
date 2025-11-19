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

| Platform                               | Collateral                                                         | Downloads                   |
| -------------------------------------- | ------------------------------------------------------------------ | --------------------------- |
| [Raspberry Pi 4 or 5][pl-0]            | [User Guide][ug-0] <br> [Source code][bi-0]                        | [![Download][dlsh-0]][dl-0] <br> [![sha256sum][shash-0]][sha-0] |
| [AMD KV260][pl-1]                      | [User Guide][ug-1] <br> [Source code][bi-1]                        | [![Download][dlsh-1]][dl-1] <br> [![sha256sum][shash-0]][sha-1] |
| [Nvidia Orin Nano Developer Kit][pl-2] | [User Guide][ug-2] <br> [Source code][bi-2]                        | [![Download][dlsh-2]][dl-2] <br> [![sha256sum][shash-0]][sha-2] |
| [AD-GMSL522-SL][pl-3]                  | [User Guide][ug-3] <br> [Source code][bi-2] <br> [Tools][tool-3]   | <br> |
| [AD-GMSL2ETH-SL][pl-4]                 | [User Guide][ug-4] <br> [Source code][bi-3]                        | [![Download][dlsh-2]][dl-4] <br> [![sha256sum][shash-0]][sha-4] |

[sdcard-burn]: http://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/sdcard_burn.md

[pl-0]: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/
[pl-1]: https://www.xilinx.com/products/som/kria/kv260-vision-starter-kit.html
[pl-2]: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
[pl-3]: https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/ad-gmsl522-sl.html#eb-overview
[pl-4]: https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/ad-gmsl2eth-sl.html

[ug-0]: https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmslcamrpi-adp/raspberry-pi-user-guide/index.html
[ug-1]: https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmslcamrpi-adp/amd-kria/index.html
[ug-2]: https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmslcamrpi-adp/nvidia-jetson/index.html
[ug-3]: https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmsl522-sl/index.html
[ug-4]: https://analogdevicesinc.github.io/documentation/solutions/reference-designs/ad-gmsl2eth-sl/index.html

[bi-0]: https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.13.y/README-GMSL.md
[bi-1]: https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/README-GMSL.md
[bi-2]: https://github.com/analogdevicesinc/nvidia/tree/gmsl/main/README.md
[bi-3]: https://github.com/analogdevicesinc/linux/tree/gmsl_k26/xilinx_v6.1_LTS/README-GMSL.md

[tool-3]: https://github.com/analogdevicesinc/gmsl/tree/tools/AD-GMSL522-SL

[dlsh-0]: https://img.shields.io/badge/release-RPI_SD_Card_Image-blue.svg
[dlsh-1]: https://img.shields.io/badge/release-KV260_SD_Card_Image-blue.svg
[dlsh-2]: https://img.shields.io/badge/release-Nvidia_Orin_Nano_SD_Card_Image-blue.svg

[dl-0]: https://swdownloads.analog.com/cse/gmsl/gmsl-kuiper-rpi-gdf25c15ded99.tar.xz
[dl-1]: https://swdownloads.analog.com/cse/gmsl/Kria/gmsl-kv260.tar.xz
[dl-2]: https://swdownloads.analog.com/cse/aditof/gmsl-nvidia-orin-nano.img.tar.xz
[dl-4]: https://swdownloads.analog.com/cse/gmsl/10G/gmsl-10g-fsync.tar.xz

[shash-0]: https://img.shields.io/badge/sha256sum-yellow.svg

[sha-0]: https://swdownloads.analog.com/cse/gmsl/gmsl-kuiper-rpi-gdf25c15ded99-sha256sum.txt
[sha-1]: https://swdownloads.analog.com/cse/gmsl/Kria/gmsl-kv260-sha256sum.txt
[sha-2]: https://swdownloads.analog.com/cse/aditof/gmsl-nvidia-orin-nano-sha256sum.txt
[sha-4]: https://swdownloads.analog.com/cse/gmsl/10G/gmsl-10g-fsync-sha256sum.txt
