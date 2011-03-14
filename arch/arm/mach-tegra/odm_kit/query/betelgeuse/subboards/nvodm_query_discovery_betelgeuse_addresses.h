/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file
 * <b>NVIDIA APX ODM Kit::
 *         Implementation of the ODM Peripheral Discovery API</b>
 *
 * @b Description: Specifies the peripheral connectivity database NvOdmIoAddress entries
 *                 for the peripherals on E1162 module.
 */

#include "pmu/tps6586x/nvodm_pmu_tps6586x_supply_info_table.h"
#include "tmon/adt7461/nvodm_tmon_adt7461_channel.h"
#include "nvodm_tmon.h"
#include "../nvodm_query_kbc_gpio_def.h"


// RTC voltage rail
static const NvOdmIoAddress s_RtcAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO2 }  /* VDD_RTC -> LD02 */
};

// Core voltage rail
static const NvOdmIoAddress s_CoreAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_DCD0 }  /* VDD_CORE -> SM0 */
};

// CPU voltage rail
static const NvOdmIoAddress s_ffaCpuAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_DCD1 }  /* VDD_CPU -> SM1 */
};

// PLLA voltage rail
static const NvOdmIoAddress s_PllAAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLM voltage rail
static const NvOdmIoAddress s_PllMAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLP voltage rail
static const NvOdmIoAddress s_PllPAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLC voltage rail
static const NvOdmIoAddress s_PllCAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDDPLLX_1V2 -> LDO1 */
};

// PLLE voltage rail
static const NvOdmIoAddress s_PllEAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS62290PmuSupply_BUCK } /* AVDD_PLLE -> VDD_1V05 */
};

// PLLU voltage rail
static const NvOdmIoAddress s_PllUAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDD_PLLU -> LDO1 */
};

// PLLU1 voltage rail
static const NvOdmIoAddress s_ffaPllU1Addresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDD_PLLU -> LDO1 */
};

// PLLS voltage rail
static const NvOdmIoAddress s_PllSAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* PLL_S -> LDO1 */
};

// PLLHD voltage rail
static const NvOdmIoAddress s_PllHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO8 } /* AVDD_HDMI_PLL -> LDO8 */
};

// OSC voltage rail
static const NvOdmIoAddress s_VddOscAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* AVDD_OSC -> LDO4 */
};

// PLLX voltage rail
static const NvOdmIoAddress s_PllXAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO1 } /* AVDDPLLX -> LDO1 */
};

// PLL_USB voltage rail
static const NvOdmIoAddress s_PllUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 } /* AVDD_USB_PLL -> derived from LDO3 (VDD_3V3) */
};

// SYS IO voltage rail
static const NvOdmIoAddress s_VddSysAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDIO_SYS -> LDO4 */
};

// USB voltage rail
static const NvOdmIoAddress s_VddUsbAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 } /* AVDD_USB -> derived from LDO3 (VDD_3V3) */
};

// HDMI voltage rail
static const NvOdmIoAddress s_VddHdmiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO7 } /* AVDD_HDMI -> LDO7 */
};

// MIPI voltage rail (DSI_CSI)
static const NvOdmIoAddress s_VddMipiAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS72012PmuSupply_LDO  } /* AVDD_DSI_CSI -> VDD_1V2 */
};

// LCD voltage rail
static const NvOdmIoAddress s_VddLcdAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDIO_LCD -> (LDO4PG) */
};

// Audio voltage rail
static const NvOdmIoAddress s_VddAudAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDIO_AUDIO -> (LDO4PG) */
};

// DDR voltage rail
static const NvOdmIoAddress s_VddDdrAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 }  /* VDDIO_DDR -> (LDO4PG) */
};

// DDR_RX voltage rail
static const NvOdmIoAddress s_VddDdrRxAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO9 }  /* VDDIO_RX_DDR(2.7-3.3) -> LDO9 */
};

// NAND voltage rail
static const NvOdmIoAddress s_VddNandAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 }  /* VDDIO_NAND_3V3 -> derived from LDO3 (VDD_3V3) */
};

// UART voltage rail
static const NvOdmIoAddress s_VddUartAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDIO_UART -> (LDO4PG) */
};

// SDIO voltage rail
static const NvOdmIoAddress s_VddSdioAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 } /* VDDIO_SDIO -> derived from LDO3 (VDD_3V3) */
};

// VDAC voltage rail
static const NvOdmIoAddress s_VddVdacAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO6 } /* AVDD_VDAC -> LDO6 */
};

// VI voltage rail
static const NvOdmIoAddress s_VddViAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 } /* VDDIO_VI -> derived from LDO3 (VDD_3V3) */
};

// BB voltage rail
static const NvOdmIoAddress s_VddBbAddresses[] = 
{
    // This is in the AON domain
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDIO_BB -> (LDO4PG) */
};

// Super power voltage rail for the SOC
static const NvOdmIoAddress s_VddSocAddresses[]=
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_SoC } /* VDD SOC */
};

// PEX_CLK voltage rail
static const NvOdmIoAddress s_VddPexClkAddresses[] = 
{
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO0 }, /* VDDIO_PEX_CLK -> LDO0 */
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS62290PmuSupply_BUCK }, /* AVDD_PLLE -> VDD_1V05 */
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS74201PmuSupply_LDO }, /* PMU_GPIO-1 -> VDD_1V5 */
};

// PMU0
static const NvOdmIoAddress s_Pmu0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x68 },
};

static const NvOdmIoAddress s_Vddio_Vid_En[] = {
    { NvOdmIoModule_Gpio, 't'-'a', 2 },
};

static const NvOdmIoAddress s_VSleep_En[] = {
    { NvOdmIoModule_Gpio, 27, 4 },
};

static const NvOdmIoAddress s_Cam_Pwdn[] = {
    { NvOdmIoModule_Gpio, 'v'-'a', 4 },
};

static const NvOdmIoAddress s_Wwan_En[] = {
    { NvOdmIoModule_Gpio, 'a'-'a', 7 },
};

static const NvOdmIoAddress s_Vddio_Sd_En[] = {
    { NvOdmIoModule_Gpio, 't'-'a', 3 },
};

static const NvOdmIoAddress s_Vddio_Sdmmc_En[] = {
    { NvOdmIoModule_Gpio, 'i'-'a', 6 },
};

static const NvOdmIoAddress s_Vddio_Bl_En[] = {
    { NvOdmIoModule_Gpio, 'w'-'a', 0 },
};

static const NvOdmIoAddress s_Vddio_Pnl_En[] = {
    { NvOdmIoModule_Gpio, 'c'-'a', 6 },
};

// P1160 ULPI USB
static const NvOdmIoAddress s_UlpiUsbAddresses[] = 
{
    { NvOdmIoModule_ExternalClock, 1, 0 }, /* ULPI PHY Clock -> DAP_MCLK2 */
};

//  LVDS LCD Display
static const NvOdmIoAddress s_LvdsDisplayAddresses[] = 
{
    { NvOdmIoModule_Display, 0, 0 },
    { NvOdmIoModule_Pwm, 0x00, 0 },
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4},     /* VDDIO_LCD (AON:VDD_1V8) */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 },    /* VDD_LVDS (VDD_3V3) */
};

//  HDMI addresses based on Concorde 2 design
static const NvOdmIoAddress s_HdmiAddresses[] =
{
    { NvOdmIoModule_Hdmi, 0, 0 },

    // Display Data Channel (DDC) for Extended Display Identification
    // Data (EDID)
    { NvOdmIoModule_I2c, 0x01, 0xA0 },

    // HDCP ROM
    { NvOdmIoModule_I2c, 0x01, 0x74 },

    // AVDD_HDMI
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO7 },
    // AVDD_HDMI_PLL
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO8 },

    // Power for DDC (VDDIO_LCD (VDD_1V8))
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 },
    // HDMI +5V for the pull-up for DDC (VDDIO_VID)
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS2051BPmuSupply_VDDIO_VID },
};

//  Power for HDMI Hotplug
static const NvOdmIoAddress s_HdmiHotplug[] =
{
    // Power for Hotplug GPIO
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 },
    // HDMI +5V for hotplug
    { NvOdmIoModule_Vdd, 0x00, Ext_TPS2051BPmuSupply_VDDIO_VID },
};


// Sdio
static const NvOdmIoAddress s_SdioAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x1,  0x0 },                      /* SD Memory on SD Bus */
    { NvOdmIoModule_Sdio, 0x3,  0x0 },                      /* SD Memory on SD Bus */
    { NvOdmIoModule_Vdd,  0x00, Ext_SWITCHPmuSupply_VDDIO_SD },   /* EN_VDDIO_SD */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 } /* VDDIO_SDIO -> derived from LDO3 (VDD_3V3) */
};

static const NvOdmIoAddress s_I2cSmbusAddresses[] = 
{
    { NvOdmIoModule_I2c, 2, 0x8A },
    { NvOdmIoModule_Gpio, 27, 1} //Port BB:01 is used on harmony. 
};

static const NvOdmIoAddress s_UsbMuxAddress[] = 
{
    {NvOdmIoModule_Usb, 1, 0}
};

static const NvOdmIoAddress s_QwertyKeyPad16x8Addresses[] =
{
    // instance = 1 indicates Column info.
    // instance = 0 indicates Row info.
    // address holds KBC pin number used for row/column.

    // All Row info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow0}, // Row 0
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow1}, // Row 1
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow2}, // Row 2
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow3}, // Row 3
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow4}, // Row 4
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow5}, // Row 5
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow6}, // Row 6
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow7}, // Row 7
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow8}, // Row 8
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow9}, // Row 9
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow10}, // Row 10
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow11}, // Row 11
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow12}, // Row 12
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow13}, // Row 13
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow14}, // Row 14
    { NvOdmIoModule_Kbd, 0x00, NvOdmKbcGpioPin_KBRow15}, // Row 15
    
    // All Column info has to be defined contiguously from 0 to max.
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol0}, // Column 0
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol1}, // Column 1
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol2}, // Column 2
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol3}, // Column 3
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol4}, // Column 4
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol5}, // Column 5
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol6}, // Column 6
    { NvOdmIoModule_Kbd, 0x01, NvOdmKbcGpioPin_KBCol7}, // Column 7
};


static const NvOdmIoAddress s_Tmon0Addresses[] = 
{
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x98 },                  /* I2C bus */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO3 },    /* TMON pwer rail -> LDO3 (VDD_3V3) */
    { NvOdmIoModule_Gpio, (NvU32)'n'-'a', 6 },              /* GPIO Port N and Pin 6 */

    /* Temperature zone mapping */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Core, ADT7461ChannelID_Remote },   /* TSENSOR */
    { NvOdmIoModule_Tsense, NvOdmTmonZoneID_Ambient, ADT7461ChannelID_Local }, /* TSENSOR */
};

// Bluetooth
static const NvOdmIoAddress s_p1162BluetoothAddresses[] =
{
    { NvOdmIoModule_Uart, 0x2,  0x0 },                  // FIXME: Is this used?
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0 },          /* BT_RST#: GPIO Port U and Pin 0 */
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_LDO4 } /* VDDHOSTIF_BT -> LDO4 (AON:VDD_1V8) */
};

// Wlan
static const NvOdmIoAddress s_WlanAddresses[] =
{
    { NvOdmIoModule_Sdio, 0x0, 0x0 },                      /* WLAN is on SD Bus */
    { NvOdmIoModule_Gpio, 0xa, 0x5 },                      /* GPIO Port K and Pin 5 - WIFI_PWR*/
    { NvOdmIoModule_Gpio, 0xa, 0x6 },                      /* GPIO Port K and Pin 6 - WIFI_RST */
    { NvOdmIoModule_Vdd,  0x00, TPS6586xPmuSupply_LDO4 },  /* VDDIO_WLAN (AON:VDD_1V8) */
    { NvOdmIoModule_Vdd,  0x00, Ext_TPS72012PmuSupply_LDO } /* VCORE_WIFI (VDD_1V2) */
};

// Audio Codec
static const NvOdmIoAddress s_AudioCodecAddresses[] = 
{
    { NvOdmIoModule_ExternalClock, 0, 0 },       /* Codec MCLK -> APxx DAP_MCLK1 */
    { NvOdmIoModule_I2c, 0x00, 0x34 },       /* Codec I2C ->  APxx PMU I2C, segment 0 */
                                                 /* Codec I2C address is 0x34 */
    { NvOdmIoModule_Gpio, (NvU32)'w'-'a', 0x02 }, /* GPIO Port W and Pin 2 for HP_DET */
    
};

// TouchPanel
static const NvOdmIoAddress s_TouchPanelAddresses[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0x04 }, /* I2C device address is 0x02, PR: use GEN1_I2C, ER: use GEN2_I2C */
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0x04 }, /* GPIO Port U and Pin 4 */
};

//g-sensor
static const NvOdmIoAddress s_AccelerometerAddresses[] =
{
        {NvOdmIoModule_I2c, 0x00, 0x12}, /*I2C device address is 0x09*/
        { NvOdmIoModule_Gpio, (NvU32)'v'-'a', 0x07 }, /* GPIO Port V and Pin 7 for gsensor, ball: ac24 */
};

// E-compas on GEN1_I2C (I2C_1)
static const NvOdmIoAddress s_EcompassAddressesI2C_1[] =
{
	{ NvOdmIoModule_I2c, 0x00, 0x18 },                 /* E compass I2C address is 0x18 */
	{ NvOdmIoModule_Gpio, (NvU32)'v'-'a', 0x01 },      /* GPIO Port V and Pin 1 for E-compass  */
};

// Vibrator
static const NvOdmIoAddress s_VibratorAddresses[] =
{
    { NvOdmIoModule_I2c, 0x00, 0x68 },
    { NvOdmIoModule_Vdd, 0x00, TPS6586xPmuSupply_PWM },
};

// Cap Sensor on GEN1_I2C (I2C_1)
static const NvOdmIoAddress s_CapSensorAddressesI2C_1[] = 
{
    { NvOdmIoModule_I2c, 0x00, 0x43 },            /* GEN1_I2C Cap Sensor I2C address is 0x43     */                                                              
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0x01 }, /* GPIO Port U and Pin 1 for CAP_INT  */

};

static const NvOdmIoAddress s_CapSensorAddressesI2C_PWR[] = 
{
 
    { NvOdmIoModule_I2c_Pmu, 0x00, 0x43 },        /* PWR_I2C Cap Sensor I2C address is 0x43     */                                                            
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0x01 }, /* GPIO Port U and Pin 1 for CAP_INT  */

};
// Carmera GPIO
static const NvOdmIoAddress s_CameraGpio[] =
{
    { NvOdmIoModule_Gpio, (NvU32)'v'-'a', 0x4 },                      /* GPIO Port V and Pin 4 (GPIO_PV4) */
};
/* paul merge smith begin */

// USB GPIO
static const NvOdmIoAddress s_UsbGpio[] =
{
    { NvOdmIoModule_Gpio, (NvU32)'u'-'a', 0x03 },                      /* GPIO Port U and Pin 3 (GPIO_PU3) */
};
/* paul merge smith end */
static const NvOdmIoAddress s_PCB_ID_GPIO[] =
{
    { NvOdmIoModule_Gpio, (NvU32)'x'-'a', 0x2 },                      /* GPIO Port x and Pin 2 (GPIO_PX2) */
    { NvOdmIoModule_Gpio, (NvU32)'x'-'a', 0x0 },                      /* GPIO Port x and Pin 0 (GPIO_PX0) */
};
// USB hub suspend
static const NvOdmIoAddress s_UsbHubSuspendGpio[] =
{
    { NvOdmIoModule_Gpio, (NvU32)'w'-'a', 0x02 },                      /* GPIO Port W and Pin 2 (GPIO_PW2) */
};

