/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

/*  NVIDIA Tegra ODM Kit  Cap Sensor Adaptation of the
 *  Cap Sensor Button Driver
 */

#ifndef INCLUDED_NVODM_CAPSENSOR_PIC16F722_H
#define INCLUDED_NVODM_CAPSENSOR_PIC16F722_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_capsensor.h"

/* PIC16F722 register address */
#define BUTTONS_STATUS_REG      0x00
#define BUTTONS_CHANGE_REG      0x01
#define LED_STATUS_REG          0x02
#define LED_ONOFF_REG           0x03
#define LED_BLINK_REG           0x04
#define WAKEUP_REG              0x05
#define LED_CTRL_REG            0x06
#define CHIP_ID_REG             0x07
#define INT_REG                 0x08
#define PCTRL_REG               0x09
#define FW_REV0_REG             0x0a
#define FW_REV1_REG             0x0b
#define FW_REV2_REG             0x0c
#define LED_MANUAL_MODE_REG     0x0d
#define LED_BLINK_FREQ_REG      0x0e
#define LED_BLINK_DUTY_REG      0x0f
#define LED_SET_T1_REG          0x10
#define LED_SET_T2_REG          0x11
#define LED_SET_T3_REG          0x12
#define BTN0_SENSITIVITY_REG    0x13
#define BTN1_SENSITIVITY_REG    0x14
#define BTN2_SENSITIVITY_REG    0x15
#define BTN3_SENSITIVITY_REG    0x16
#define BTN4_SENSITIVITY_REG    0x17
#define BTN5_SENSITIVITY_REG    0x18
#define BTN6_SENSITIVITY_REG    0x19
#define BTN7_SENSITIVITY_REG    0x1a

/* mode settings */
#define CAPSENSOR_MODE_NORMAL         0
#define CAPSENSOR_MODE_SLEEP          1

#define CAPSENSOR_CHIP_ID      0x01       // RO - device identification

// Timeout for I2C transaction.
enum { I2C_CAPSENSOR_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_CAPSENSOR_PACKET_SIZE = 8};
static NvU8 s_ReadBuffer[I2C_CAPSENSOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_CAPSENSOR_PACKET_SIZE];

#define INT_EVENT_TIMEOUT 100


/*
 * Defines the way to read cap sensor registers.
 */
typedef NvBool
(*CapSensorRegsRead)(
    NvOdmCapHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);
/*
 * Defines the way to write cap sensor registers.
 */
typedef NvBool
(*CapSensorRegsWrite)(
    NvOdmCapHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);
/*
 * Holds register address and value pairs.
 */
typedef struct NvDevCtrlRegRec {
   /// Holds the register offset.
   NvU8 RegAddr;
   /// Holds the value programmed into the upper address.
   NvU8 RegValue;
} NvDevCtrlReg;
/*
 * Max Cap Sensor registers number.
 */
#define CAPSENSOR_CONTROL_REGS_MAX_LENGHT 32
/*
 * Max accelerometer callback functions number.
 */
#define CAPSENSOR_CALLBACK_ARRAY_LENGTH   5

typedef struct NvOdmCapRec
{
  
    // Specifies accelerometer device address, for example, I2C write address.
    NvU8 nDevAddr;
    // Specifies the initial value that make accelerometer work,
    // CAPSENSOR_CONTROL_REGS_MAX_LENGHT is always 32.
    NvDevCtrlReg CtrlRegsList[CAPSENSOR_CONTROL_REGS_MAX_LENGHT];
    // Specifies the initial CtrlRegsList length.
    NvU8 nLength;
    // Specifies cap sensor chip ID.
    NvU8 nChipID;
    
    // Specifies the way to get cap sensor register information.
    CapSensorRegsRead RegsRead;
    // Specifies the way to set cap sensor register information.
    CapSensorRegsWrite RegsWrite;
    // Specifies I2C handle from the system.
    NvOdmServicesI2cHandle  hOdmI2C;        
    
    // Interrupt pin to ap20.
    NvOdmServicesGpioHandle hGpioINT;
    NvOdmGpioPinHandle      hPinINT;
    NvU32 GPIOPortINT;
    NvU32 GPIOPinINT;
    NvOdmOsSemaphoreHandle SemaphoreForINT;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
    NvOdmServicesPmuHandle hPmu;
    NvU32 VddId;
    NvU32 I2CChannelId;
} NvOdmCapSensor;

#if defined(__cplusplus)
}
#endif
#endif
