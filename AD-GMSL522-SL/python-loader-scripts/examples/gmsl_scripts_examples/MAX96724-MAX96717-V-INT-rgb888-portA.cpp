/*
# Name: btogorea
# Date: 2/16/2024
# Version: 6.6.0
#
# I2C Address(0x), Register Address(0x), Register Value(0x), Read Modify Write(0x)
#
# THIS DATA FILE, AND ALL INFORMATION CONTAINED THEREIN,
# IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DATA FILE,
# THE INFORMATION CONTAINED THEREIN, OR ITS USE FOR ANY PURPOSE.
# BEFORE USING THIS DATA FILE IN ANY APPLICATION FOR PRODUCTION OR DEPLOYMENT,
# THE CUSTOMER IS SOLELY RESPONSIBLE FOR TESTING AND VERIFYING
# THE CONTENT OF THIS DATA FILE IN CONNECTION WITH THEIR PRODUCTS AND SYSTEM(S).
# ---------------------------------------------------------------------------------
#
#            _____ _____  
#      /\   |  __ \_   _| 
#     /  \  | |  | || |   
#    / /\ \ | |  | || |   
#   / ____ \| |__| || |_  
#  /_/    \_\_____/_____| 
#
# ---------------------------------------------------------------------------------
*/
/*
# This script is validated on: 
# MAX96717
# MAX96724
# Please refer to the Errata sheet for each device.
# ---------------------------------------------------------------------------------
*/
//  
// CSIConfigurationTool
//  
// GMSL-A / Serializer: MAX96717 (Pixel Mode) / Mode: 1x4 / Device Address: 0x84 / Multiple-VC Case: Single VC / Multiple-VC Pipe Sharing: N/A
// PipeZ:
// Input Stream: VC0 RGB888 PortB (D-PHY)

// Deserializer: MAX96724 / Mode: 2 (1x4) / Device Address: 0x4E
// Pipe0:
// GMSL-A Input Stream: VC0 RGB888 PortB - Output Stream: VC0 RGB888 PortB (D-PHY)

0x04,0x4E,0x04,0x0B,0x00, // BACKTOP : BACKTOP12 | CSI_OUT_EN (CSI_OUT_EN): CSI output disabled
// Link Initialization for Deserializer
0x04,0x4E,0x00,0x06,0xFF, // DEV : REG6 | (Default) LINK_EN_A (LINK_EN_A): Enabled | LINK_EN_B (LINK_EN_B): Enabled | LINK_EN_C (LINK_EN_C): Enabled | LINK_EN_D (LINK_EN_D): Enabled
0x04,0x4E,0x00,0x03,0xFF, // DEV : REG3 | DIS_REM_CC_A (GMSL Link A I2C Port 0): Disabled | DIS_REM_CC_B (GMSL Link B I2C Port 0): Disabled | DIS_REM_CC_C (GMSL Link C I2C Port 0): Disabled | DIS_REM_CC_D (GMSL Link D I2C Port 0): Disabled
0x00,0x01, // Warning: The actual recommended delay is 5 usec.
// Video Transmit Configuration for Serializer(s)
0x04,0x84,0x00,0x02,0x03, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Disabled
//  
// INSTRUCTIONS FOR GMSL-A SERIALIZER MAX96717
//  
// MIPI DPHY Configuration
0x04,0x84,0x03,0x30,0x00, // MIPI_RX : MIPI_RX0 | (Default) RSVD (Port Configuration): 1x4
0x04,0x84,0x03,0x83,0x00, // MIPI_RX_EXT : EXT11 | Tun_Mode (Tunnel Mode): Disabled
0x04,0x84,0x03,0x31,0x30, // MIPI_RX : MIPI_RX1 | (Default) ctrl1_num_lanes (Port B - Lane Count): 4
0x04,0x84,0x03,0x32,0xE0, // MIPI_RX : MIPI_RX2 | (Default) phy1_lane_map (Lane Map - PHY1 D0): Lane 2 | (Default) phy1_lane_map (Lane Map - PHY1 D1): Lane 3
0x04,0x84,0x03,0x33,0x04, // MIPI_RX : MIPI_RX3 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1
0x04,0x84,0x03,0x34,0x00, // MIPI_RX : MIPI_RX4 | (Default) phy1_pol_map (Polarity - PHY1 Lane 0): Normal | (Default) phy1_pol_map (Polarity - PHY1 Lane 1): Normal
0x04,0x84,0x03,0x35,0x00, // MIPI_RX : MIPI_RX5 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
// Controller to Pipe Mapping Configuration
0x04,0x84,0x03,0x08,0x64, // FRONTTOP : FRONTTOP_0 | (Default) RSVD (CLK_SELZ): Port B | (Default) START_PORTB (START_PORTB): Enabled
0x04,0x84,0x03,0x11,0x40, // FRONTTOP : FRONTTOP_9 | (Default) START_PORTBZ (START_PORTBZ): Start Video
0x04,0x84,0x03,0x18,0x64, // FRONTTOP : FRONTTOP_16 | mem_dt1_selz (mem_dt1_selz): 0x64
0x04,0x84,0x03,0x15,0x00, // (Default)  (independent_vs_mode): Disabled
// Pipe Configuration
0x04,0x84,0x00,0x5B,0x00, // CFGV__VIDEO_Z : TX3 | TX_STR_SEL (TX_STR_SEL Pipe Z): 0x0
//  
// INSTRUCTIONS FOR DESERIALIZER MAX96724
//  
// Video Pipes And Routing Configuration
0x04,0x4E,0x00,0xF0,0x60, // VIDEO_PIPE_SEL : VIDEO_PIPE_SEL_0 | (Default) VIDEO_PIPE_SEL_0 (Pipe 0 GMSL2 PHY): A | VIDEO_PIPE_SEL_0 (Pipe 0 Input Pipe): X
0x04,0x4E,0x00,0xF4,0x01, // VIDEO_PIPE_SEL : VIDEO_PIPE_EN | (Default) VIDEO_PIPE_EN (Video Pipe 0): Enabled | VIDEO_PIPE_EN (Video Pipe 1): Disabled | VIDEO_PIPE_EN (Video Pipe 2): Disabled | VIDEO_PIPE_EN (Video Pipe 3): Disabled | STREAM_SEL_ALL (Stream Select All): Disabled
// Pipe to Controller Mapping Configuration
0x04,0x4E,0x09,0x0B,0x07, // MIPI_TX__0 : MIPI_TX11 | MAP_EN_L (MAP_EN_L Pipe 0): 0x7
0x04,0x4E,0x09,0x0C,0x00, // MIPI_TX__0 : MIPI_TX12 | (Default) MAP_EN_H (MAP_EN_H Pipe 0): 0x0
0x04,0x4E,0x09,0x0D,0x24, // MIPI_TX__0 : MIPI_TX13 | MAP_SRC_0 (MAP_SRC_0 Pipe 0 DT): 0x24 | (Default) MAP_SRC_0 (MAP_SRC_0 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x0E,0x24, // MIPI_TX__0 : MIPI_TX14 | MAP_DST_0 (MAP_DST_0 Pipe 0 DT): 0x24 | (Default) MAP_DST_0 (MAP_DST_0 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x0F,0x00, // MIPI_TX__0 : MIPI_TX15 | (Default) MAP_SRC_1 (MAP_SRC_1 Pipe 0 DT): 0x0 | (Default) MAP_SRC_1 (MAP_SRC_1 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x10,0x00, // MIPI_TX__0 : MIPI_TX16 | (Default) MAP_DST_1 (MAP_DST_1 Pipe 0 DT): 0x0 | (Default) MAP_DST_1 (MAP_DST_1 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x11,0x01, // MIPI_TX__0 : MIPI_TX17 | MAP_SRC_2 (MAP_SRC_2 Pipe 0 DT): 0x1 | (Default) MAP_SRC_2 (MAP_SRC_2 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x12,0x01, // MIPI_TX__0 : MIPI_TX18 | MAP_DST_2 (MAP_DST_2 Pipe 0 DT): 0x1 | (Default) MAP_DST_2 (MAP_DST_2 Pipe 0 VC): 0x0
0x04,0x4E,0x09,0x2D,0x2A, // MIPI_TX__0 : MIPI_TX45 | MAP_DPHY_DEST_0 (MAP_DPHY_DST_0 Pipe 0): 0x2 | MAP_DPHY_DEST_1 (MAP_DPHY_DST_1 Pipe 0): 0x2 | MAP_DPHY_DEST_2 (MAP_DPHY_DST_2 Pipe 0): 0x2
// Double Mode Configuration
// MIPI DPHY Configuration
0x04,0x4E,0x08,0xA0,0x04, // MIPI_PHY : MIPI_PHY0 | (Default) phy_4x2 (Port Configuration): 2 (1x4)
0x04,0x4E,0x09,0x8A,0xD0, // MIPI_TX__2 : MIPI_TX10 | (Default) CSI2_LANE_CNT (Port B - Lane Count): 4
0x04,0x4E,0x08,0xA4,0xE4, // MIPI_PHY : MIPI_PHY4 | (Default) phy2_lane_map (Lane Map - PHY2 D0): Lane 0 | (Default) phy2_lane_map (Lane Map - PHY2 D1): Lane 1 | (Default) phy3_lane_map (Lane Map - PHY3 D0): Lane 2 | (Default) phy3_lane_map (Lane Map - PHY3 D1): Lane 3
0x04,0x4E,0x08,0xA6,0x00, // MIPI_PHY : MIPI_PHY6 | (Default) phy2_pol_map (Polarity - PHY2 Lane 0): Normal | (Default) phy2_pol_map (Polarity - PHY2 Lane 1): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 0): Normal | (Default) phy3_pol_map (Polarity - PHY3 Lane 1): Normal | (Default) phy2_pol_map (Polarity - PHY2 Clock Lane): Normal
0x04,0x4E,0x09,0x83,0x07, // MIPI_TX__2 : MIPI_TX3 | DESKEW_INIT (Controller 2 Auto Initial Deskew): Disabled
0x04,0x4E,0x09,0x84,0x01, // MIPI_TX__2 : MIPI_TX4 | DESKEW_PER (Controller 2 Periodic Deskew): Disabled
0x04,0x4E,0x1E,0x00,0xF4, //  (config_soft_rst_n - PHY2): 0x0
// This is to set predefined (coarse) CSI output frequency
// CSI Phy 2 is 1500 Mbps/lane.
0x04,0x4E,0x1E,0x00,0xF4, // (Default) 
0x04,0x4E,0x04,0x1B,0x2F, // (Default) 
0x04,0x4E,0x1E,0x00,0xF5, //  | (Default)  (config_soft_rst_n - PHY2): 0x1
0x04,0x4E,0x08,0xA2,0xC4, // MIPI_PHY : MIPI_PHY2 | phy_Stdby_n (phy_Stdby_0): Put PHY0 in standby mode | phy_Stdby_n (phy_Stdby_1): Put PHY1 in standby mode
0x04,0x4E,0x04,0x0B,0x02, // BACKTOP : BACKTOP12 | CSI_OUT_EN (CSI_OUT_EN): CSI output enabled
// Video Transmit Configuration for Serializer(s)
0x04,0x84,0x00,0x02,0x43, // DEV : REG2 | VID_TX_EN_Z (VID_TX_EN_Z): Enabled
