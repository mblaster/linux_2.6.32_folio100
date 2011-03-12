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

/*  NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */

#ifndef INCLUDED_NVODM_ACCELEROMETER_LSM303DLH_H
#define INCLUDED_NVODM_ACCELEROMETER_LSM303DLH_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_accelerometer.h"

#define LSM303DLH_CHIP_ID      0x12        //device identification

/* LSM303DLH register address */
#define CHIP_ID_REG             0x0f

#define X_AXIS_LSB_REG          0x28
#define X_AXIS_MSB_REG          0x29
#define Y_AXIS_LSB_REG          0x2a
#define Y_AXIS_MSB_REG          0x2b
#define Z_AXIS_LSB_REG          0x2c
#define Z_AXIS_MSB_REG          0x2d
/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1		0x20	/* power control reg */
#define CTRL_REG2		0x21	/* power control reg */
#define CTRL_REG3		0x22	/* power control reg */
#define CTRL_REG4		0x23	/* interrupt control reg */
#define CTRL_REG5		0x24	/* interrupt control reg */
#define INT1_SRC			0x31
/* Accelerometer Sensor Full Scale */
#define LSM303DLH_G_2G 			0x00
#define LSM303DLH_G_4G 			0x10
#define LSM303DLH_G_8G 			0x30

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

/* Accelerometer Sensor Operating Mode */
#define LSM303DLH_ACC_PM_OFF		0x00
#define LSM303DLH_ACC_PM_NORMAL		0x20
#define LSM303DLH_ACC_ENABLE_ALL_AXES	0x07

/* Accelerometer output data rate  */
#define LSM303DLH_ACC_ODRHALF		0x40	/* 0.5Hz output data rate */
#define LSM303DLH_ACC_ODR1		0x60	/* 1Hz output data rate */
#define LSM303DLH_ACC_ODR2		0x80	/* 2Hz output data rate */
#define LSM303DLH_ACC_ODR5		0xA0	/* 5Hz output data rate */
#define LSM303DLH_ACC_ODR10		0xC0	/* 10Hz output data rate */
#define LSM303DLH_ACC_ODR50		0x00	/* 50Hz output data rate */
#define LSM303DLH_ACC_ODR100		0x08	/* 100Hz output data rate */
#define LSM303DLH_ACC_ODR400		0x10	/* 400Hz output data rate */
#define LSM303DLH_ACC_ODR1000		0x18	/* 1000Hz output data rate */

#define AUTO_INCREMENT		0x80

/*
 * Defines the threshold source for the accelerometer.
 */
typedef enum
{
    /// Indicates the accelerometer generated interrupt by exceeding the x threshold.
    NvOdmAccelerometerThresholdSource_X = 0,

    /// Indicates the accelerometer generated interrupt by exceeding the y threshold.
    NvOdmAccelerometerThresholdSource_Y,

    /// Indicates the accelerometer generated interrupt by exceeding the z threshold.
    NvOdmAccelerometerThresholdSource_Z,

    NvOdmAccelerometerThresholdSource_Force32 = 0x7FFFFFFF
} NvOdmAccelerometerThresholdSource;

// Timeout for I2C transaction.
enum { I2C_ACCELRATOR_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_ACCELRATOR_PACKET_SIZE = 8};
static NvU8 s_ReadBuffer[I2C_ACCELRATOR_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ACCELRATOR_PACKET_SIZE];

#define INT_EVENT_TIMEOUT 100
#define NV_ACCELEROMETER_BUS_I2C 0
#define NV_ACCELEROMETER_BUS_SPI_3 1
#define NV_ACCELEROMETER_BUS_SPI_4 2

/*
 * Defines the way to read accelerometer registers.
 */
typedef NvBool
(*AccelerometerRegsRead)(
    NvOdmAccelHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool
(*AccelerometerRegsWrite)(
    NvOdmAccelHandle hDevice,
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
 * Max accelerometer registers number.
 */
#define ACCELEROMETER_CONTROL_REGS_MAX_LENGHT 100
/*
 * Max accelerometer callback functions number.
 */
#define ACCELEROMETER_CALLBACK_ARRAY_LENGTH   5

typedef struct NvOdmAccelRec
{
    // Specifies use I2C or SPI to configure accelerometer registers.
    NvU8 nBusType;
    // Specifies accelerometer device address, for example, I2C write address.
    NvU8 nDevAddr;
    // Specifies the initial value that make accelerometer work,
    // ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
    NvDevCtrlReg CtrlRegsList[ACCELEROMETER_CONTROL_REGS_MAX_LENGHT];
    // Specifies the initial CtrlRegsList length.
    NvU8 nLength;
    // Specifies accelerometer chip ID.
    NvU8 nChipID;
    // Specifies the way to get accelerometer register information.
    AccelerometerRegsRead RegsRead;
    // Specifies the way to set accelerometer register information.
    AccelerometerRegsWrite RegsWrite;
    // Specifies I2C handle from the system.
    NvOdmServicesI2cHandle  hOdmI2C;
    // Interrupt pin to ap15.
    NvOdmServicesGpioHandle hGpioINT;
    NvOdmGpioPinHandle      hPinINT;
    NvU32 GPIOPortINT;
    NvU32 GPIOPinINT;
    NvOdmOsSemaphoreHandle SemaphoreForINT;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
    NvOdmAccelIntType Data;
    NvOdmServicesPmuHandle hPmu;
    NvU32 VddId;
    NvU32 I2CChannelId;
    NvOdmAccelerometerCaps Caption;
    NvOdmAccelPowerType PowerState;
    // In real case, when the board put in frontispiece, the value from z axis
    // should be g, but due to physical connect on different board, the axis
    // should be remapped to the correct one.
    NvOdmAccelAxisType AxisXMapping;
    // If the physical direct is the same with our expection, the value
    // should be set to 1, or else the value should be -1.
    NvS32              AxisXDirection;
    NvOdmAccelAxisType AxisYMapping;
    NvS32              AxisYDirection;
    NvOdmAccelAxisType AxisZMapping;
    NvS32              AxisZDirection;
} NvOdmAccel;

#if defined(__cplusplus)
}
#endif
#endif
