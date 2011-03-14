/*
 * PEGATRON Corp. Confidential and Proprietary
 *
 * The following software source code ("Software") is strictly confidential and
 * is proprietary to PEGATRON Corp. ("PEGATRON").  It may only be read,
 * used, copied, adapted, modified or otherwise dealt with by you if you have
 * entered into a confidentiality agreement with PEGATRON and then subject to the
 * terms of that confidentiality agreement and any other applicable agreement
 * between you and PEGATRON.  If you are in any doubt as to whether you are
 * entitled to access, read, use, copy, adapt, modify or otherwise deal with
 * the Software or whether you are entitled to disclose the Software to any
 * other person you should contact PEGATRON.  If you have not entered into a
 * confidentiality agreement with PEGATRON granting access to this Software you
 * should forthwith return all media, copies and printed listings containing
 * the Software to PEGATRON.
 *
 * PEGATRON reserves the right to take legal action against you should you breach
 * the above provisions.
 */

#ifndef INCLUDED_NVODM_ECOMPASS_AKM8975_H
#define INCLUDED_NVODM_ECOMPASS_AKM8975_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_ecompass.h"

/*! \name AK8975 operation mode
 \anchor AK8975_Mode
 Defines an operation mode of the AK8975.*/
/*! @{*/
#define AK8975_MODE_SNG_MEASURE	0x01
#define	AK8975_MODE_SELF_TEST	0x08
#define	AK8975_MODE_FUSE_ACCESS	0x0F
#define	AK8975_MODE_POWERDOWN	0x00
/*! @}*/

/*! \name AK8975 register address
\anchor AK8975_REG
Defines a register address of the AK8975.*/
/*! @{*/
#define AK8975_REG_WIA		0x00
#define AK8975_REG_INFO		0x01
#define AK8975_REG_ST1		0x02
#define AK8975_REG_HXL		0x03
#define AK8975_REG_HXH		0x04
#define AK8975_REG_HYL		0x05
#define AK8975_REG_HYH		0x06
#define AK8975_REG_HZL		0x07
#define AK8975_REG_HZH		0x08
#define AK8975_REG_ST2		0x09
#define AK8975_REG_CNTL		0x0A
#define AK8975_REG_RSV		0x0B
#define AK8975_REG_ASTC		0x0C
#define AK8975_REG_TS1		0x0D
#define AK8975_REG_TS2		0x0E
#define AK8975_REG_I2CDIS	0x0F
/*! @}*/

// Timeout for I2C transaction.
enum { I2C_ECOMPASS_TRANSACTION_TIMEOUT = 1000 };
// Maximum number of packetsize supported by the I2C controller.
enum { I2C_ECOMPASS_PACKET_SIZE = 16};
static NvU8 s_ReadBuffer[I2C_ECOMPASS_PACKET_SIZE];
static NvU8 s_WriteBuffer[I2C_ECOMPASS_PACKET_SIZE];

#define NV_ECOMPASS_BUS_I2C 0

/*
 * Defines the way to read accelerometer registers.
 */
typedef NvBool
(*EcompassRegsRead)(
    NvOdmEcompassHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);
/*
 * Defines the way to write accelerometer registers.
 */
typedef NvBool
(*EcompassRegsWrite)(
    NvOdmEcompassHandle hDevice,
    NvU8 nRegOffset,
    NvU8* nData,
    NvU32 nLen);

typedef struct NvOdmEcompassRec
{
    // Specifies use I2C or SPI to configure ecompass registers.
    NvU8 nBusType;
    // Specifies ecompass device address, for example, I2C write address.
    NvU8 nDevAddr;
    // Specifies the initial value that make ecompass work,
    // ACCELEROMETER_CONTROL_REGS_MAX_LENGHT is always 100.
		//NvDevCtrlReg CtrlRegsList[ACCELEROMETER_CONTROL_REGS_MAX_LENGHT];
    // Specifies the initial CtrlRegsList length.
		//NvU8 nLength;
    // Specifies ecompass chip ID.
    NvU8 nChipID;
    // Specifies the way to get ecompass register information.
    EcompassRegsRead RegsRead;
    // Specifies the way to set ecompass register information.
    EcompassRegsWrite RegsWrite;
    // Specifies I2C handle from the system.
    NvOdmServicesI2cHandle  hOdmI2C;
    // Interrupt pin to ap15.
    NvOdmServicesGpioHandle hGpioINT;
    NvOdmGpioPinHandle      hPinINT;
    NvU32 GPIOPortINT;
    NvU32 GPIOPinINT;
    NvOdmOsSemaphoreHandle SemaphoreForINT;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;
		//NvOdmAccelIntType Data;
		//NvOdmServicesPmuHandle hPmu;
		//NvU32 VddId;
    NvU32 I2CChannelId;
		//NvOdmAccelerometerCaps Caption;
		//NvOdmAccelPowerType PowerState;
	// In real case, when the board put in frontispiece, the value from z axis
	// should be g, but due to physical connect on different board, the axis
	// should be remapped to the correct one.
		//NvOdmAccelAxisType AxisXMapping;
	// If the physical direct is the same with our expection, the value
	// should be set to 1, or else the value should be -1.
		//NvS32              AxisXDirection;
		//NvOdmAccelAxisType AxisYMapping;
		//NvS32              AxisYDirection;
		//NvOdmAccelAxisType AxisZMapping;
		//NvS32              AxisZDirection;
} NvOdmEcompass;

#endif


