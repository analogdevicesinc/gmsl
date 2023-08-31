# Gigabit Multimedia Serial Link (GMSL) technology from Analog Devices, Inc. (ADI)

## Overview
Gigabit Multimedia Serial Link (GMSL™) technology from Analog Devices, Inc. (ADI) is providing reliable transport of high resolution digital video data
over coaxial or twisted pair serial links. This repository is the root starting point for GMSL technology open source software provided by ADI.

## Supported host platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below:

| Host Platform | Documentation | Devicetree examples | Driver sources |
| --------- | ----------- | ----------- | ----------- |
| Raspberry Pi | [Build Instructions](https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/README-GMSL.md) | [![gmsl/rpi-6.1.y](https://img.shields.io/badge/release-RPI_overlay-green.svg)](https://github.com/analogdevicesinc/linux/blob/gmsl/rpi-6.1.y/arch/arm/boot/dts/overlays/gmsl-overlay.dts) | [![gmsl/rpi-6.1.y](https://img.shields.io/badge/release-GMSL_RPI_driver_sources-blue.svg)](https://github.com/analogdevicesinc/linux/tree/gmsl/rpi-6.1.y/drivers/media/i2c/maxim-serdes) |
| Xilinx KV260 / ZCU102 | [Build Instructions](https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/README-GMSL.md) | [![gmsl/xilinx_v6.1_LTS](https://img.shields.io/badge/release-Xilinx_device_tree-green.svg)](https://github.com/analogdevicesinc/linux/blob/gmsl/xilinx_v6.1_LTS/arch/arm64/boot/dts/xilinx/gmsl.dts) | [![gmsl/xilinx_v6.1_LTS](https://img.shields.io/badge/release-GMSL_Xilinx_driver_sources-blue.svg)](https://github.com/analogdevicesinc/linux/tree/gmsl/xilinx_v6.1_LTS/drivers/media/i2c/maxim-serdes) |
| Nvidia Orin Nano Developer Kit | [Build Instructions](https://github.com/analogdevicesinc/linux/tree/gmsl/jetson_35.3.1/README-GMSL.md) | [![gmsl/jetson_35.3.1](https://img.shields.io/badge/release-GMSL_Nvidia_device_tree-green.svg)](https://github.com/analogdevicesinc/nvidia/tree/gmsl/jetson_35.3.1/kernel_kernel-5.10/arch/arm64/boot/dts/gmsl) | [![gmsl/jetson_35.3.1](https://img.shields.io/badge/release-GMSL_Nvidia_driver_sources-blue.svg)](https://github.com/analogdevicesinc/nvidia/tree/gmsl/jetson_35.3.1/kernel_kernel-5.10/drivers/media/i2c/maxim-serdes) |

## Prebuilt SD card images

For faster evaluation of GMSL technology ADI is providing a list of prebuilt SD card images for the platforms specified below:

| Host Platform | User Guide | Downloads |
| --------- | ----------- | ----------- |
| Raspberry Pi | [RPI User Guide]() | [![GMSL RPI Image](https://img.shields.io/badge/release-RPI_Image-blue.svg)]() |
| Xilinx KV260 Kria | [Xilinx Kria User Guide]() | [![GMSL Kria Image](https://img.shields.io/badge/release-Kria_Image-blue.svg)]() |
| Xilinx ZCU102 | [Xilinx ZCU102 User Guide](https://wiki.analog.com/playground/gmsl-zcu102-guide) | [![GMSL ZCU102 Image](https://img.shields.io/badge/release-ZCU102_Image-blue.svg)]() |
| Nvidia Orin Nano Developer Kit | [Nvidia Orin User Guide]() | [![Nvidia Orin Nano Image](https://img.shields.io/badge/release-Nvidia_Orin_Nano_Image-blue.svg)]() |

More details on how to extract a compressed image and write it on the SD card on Linux and Windows can be found here: [Writing an image onto the SD card](http://github.com/analogdevicesinc/aditof_sdk/blob/master/doc/sdcard_burn.md)

---
**Known issues**

---
