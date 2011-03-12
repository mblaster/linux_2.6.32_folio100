/*
 * Copyright (c) 2006-2009 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvodm_vibrate.h"
#include "nvos.h"
#include "nvassert.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_pmu.h"
#include "../odm_kit/adaptations/pmu/tps6586x/nvodm_pmu_tps6586x_i2c.h"

#define VIBRATE_DEVICE_GUID NV_ODM_GUID('v','i','b','r','a','t','o','r')

/**
 * @brief Used to enable/disable debug print messages.
 */
#define NV_ODM_DEBUG 0

#if NV_ODM_DEBUG
    #define NV_ODM_TRACE NvOdmOsDebugPrintf
#else
    #define NV_ODM_TRACE (void)
#endif

#define BOARD_PRSTR "PR"
#define PR_MODE 0
#define ER_MODE 1

int Vib_mode; /* 0:PR, 1:ER */

typedef struct NvOdmVibDeviceRec
{
    /* The handle to the Pmu device */
    NvOdmServicesPmuHandle hOdmServicePmuDevice;

    /*Pmu Vdd Rail capabilities*/
    NvOdmServicesPmuVddRailCapabilities RailCaps;

    /* Pmu Rail ID*/
    NvU32  VddId;

	/* I2C handle */
    NvOdmServicesI2cHandle hOdmI2c;
	
	NvU32 DeviceAddr;

} NvOdmVibDevice;

static NvBool I2C_Read(NvOdmVibDeviceHandle hOdmVibrate, NvU8 reg, NvU8 *val)
{
    NvU8 ReadBuffer = 0;
    NvOdmI2cStatus Status;    
    NvOdmI2cTransactionInfo TransactionInfo[2];
    
	ReadBuffer = reg & 0xFF;
	
    TransactionInfo[0].Address = hOdmVibrate->DeviceAddr;
    TransactionInfo[0].Buf = &ReadBuffer;
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;    
    TransactionInfo[1].Address = (hOdmVibrate->DeviceAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer;
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 1;

  	Status = NvOdmI2cTransaction(hOdmVibrate->hOdmI2c,
  								&TransactionInfo[0], 
  								2, 
                                TPS6586x_I2C_SPEED_KHZ, 
                                NV_WAIT_INFINITE);
  	
	if (Status != NvOdmI2cStatus_Success)
    {
        NV_ODM_TRACE(("I2C Read Failure = %d (addr=0x%x, reg=0x%x)\n", Status, hOdmVibrate->DeviceAddr, reg));
        return NV_FALSE;
    }
	
    *val = ReadBuffer;
    return NV_TRUE;
}

static NvBool I2C_Write (NvOdmVibDeviceHandle hOdmVibrate, NvU8 reg, NvU8 val)
{
    NvOdmI2cStatus Status;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[2];

    arr[0] = reg;
    arr[1] = val;

    TransactionInfo.Address = hOdmVibrate->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 2;

    Status = NvOdmI2cTransaction(hOdmVibrate->hOdmI2c,
                                &TransactionInfo,
                                1,
                                TPS6586x_I2C_SPEED_KHZ,
                                NV_WAIT_INFINITE);

    if (Status != NvOdmI2cStatus_Success)
    {
        NV_ODM_TRACE(("I2C Write Failure = %d (addr=0x%x, reg=0x%x, val=0x%0x)\n", Status, hOdmVibrate->DeviceAddr, reg, val));

        return NV_FALSE;
    }

    return NV_TRUE;
}


void PegaVibModeInit(char *options)
{
	if (NvOsStrcmp(options, BOARD_PRSTR) == 0)
		Vib_mode = PR_MODE;
	else
		Vib_mode = ER_MODE;
 }

void PgeaVibModeSet(int mode)
{
	Vib_mode = mode;
}

int PgeaVibModeGet(void)
{
	return Vib_mode;
}

/**
 *  @brief Allocates a handle to the device. Configures the PWM
 *   control to the Vibro motor with default values. To change
 *   the amplitude and frequency use NvOdmVibrateSetParameter API.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return  NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibOpen(NvOdmVibDeviceHandle *hOdmVibrate)
{

#if 1	/* yuyang(20100615):Create I2C handle */
	const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvU32 Index = 0;
    NvU32 I2cInstance = 0;

    NV_ASSERT(hOdmVibrate);

    /* Allocate the handle */
    (*hOdmVibrate) = (NvOdmVibDeviceHandle)NvOdmOsAlloc(sizeof(NvOdmVibDevice));
    if (*hOdmVibrate == NULL)
    {
        NV_ODM_TRACE(("Error Allocating NvOdmPmuDevice. \n"));
        return NV_FALSE;
    }
    NvOsMemset((*hOdmVibrate), 0, sizeof(NvOdmVibDevice));

    /* Get the PMU handle */
    (*hOdmVibrate)->hOdmServicePmuDevice = NvOdmServicesPmuOpen();

    if (!(*hOdmVibrate)->hOdmServicePmuDevice)
    {
        NV_ODM_TRACE(("Error Opening Pmu device. \n"));
        NvOdmOsFree(*hOdmVibrate);
        *hOdmVibrate = NULL;
        return NV_FALSE;
    }

    // Get the peripheral connectivity information
    pConnectivity = NvOdmPeripheralGetGuid(VIBRATE_DEVICE_GUID);
    if (pConnectivity == NULL)
    {
        NV_ODM_TRACE(("Error pConnectivity NULL. \n"));
        return NV_FALSE;
    }

	for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
    {
        switch (pConnectivity->AddressList[Index].Interface)
        {
			case NvOdmIoModule_I2c:
                (*hOdmVibrate)->DeviceAddr = (pConnectivity->AddressList[Index].Address);
                I2cInstance = pConnectivity->AddressList[Index].Instance;
                NV_ODM_TRACE("%s: hTouch->DeviceAddr = 0x%x, I2cInstance = %x\n", __func__, (*hOdmVibrate)->DeviceAddr, I2cInstance); 
                break;
            case NvOdmIoModule_Vdd:
                (*hOdmVibrate)->VddId = pConnectivity->AddressList[Index].Address;
                NvOdmServicesPmuGetCapabilities((*hOdmVibrate)->hOdmServicePmuDevice, (*hOdmVibrate)->VddId, &((*hOdmVibrate)->RailCaps));
                break;
            default:
                break;
        }
    }

	(*hOdmVibrate)->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, I2cInstance);

	if (!(*hOdmVibrate)->hOdmI2c)
    {
        NV_ODM_TRACE(("NvOdm Touch : NvOdmI2cOpen Error \n"));
        return NV_FALSE;
    }
#else
    const NvOdmPeripheralConnectivity *pConnectivity = NULL;
    NvU32 Index = 0;

    NV_ASSERT(hOdmVibrate);

    /* Allocate the handle */
    (*hOdmVibrate) = (NvOdmVibDeviceHandle)NvOdmOsAlloc(sizeof(NvOdmVibDevice));
    if (*hOdmVibrate == NULL)
    {
        NV_ODM_TRACE(("Error Allocating NvOdmPmuDevice. \n"));
        return NV_FALSE;
    }
    NvOsMemset((*hOdmVibrate), 0, sizeof(NvOdmVibDevice));

    /* Get the PMU handle */
    (*hOdmVibrate)->hOdmServicePmuDevice = NvOdmServicesPmuOpen();
    if (!(*hOdmVibrate)->hOdmServicePmuDevice)
    {
        NV_ODM_TRACE(("Error Opening Pmu device. \n"));
        NvOdmOsFree(*hOdmVibrate);
        *hOdmVibrate = NULL;
        return NV_FALSE;
    }

        // Get the peripheral connectivity information
    pConnectivity = NvOdmPeripheralGetGuid(VIBRATE_DEVICE_GUID);
    if (pConnectivity == NULL)
        return NV_FALSE;

        // Search for the Vdd rail and set the proper volage to the rail.
    for (Index = 0; Index < pConnectivity->NumAddress; ++Index)
    {
        if (pConnectivity->AddressList[Index].Interface == NvOdmIoModule_Vdd)
        {
            (*hOdmVibrate)->VddId = pConnectivity->AddressList[Index].Address;
            NvOdmServicesPmuGetCapabilities((*hOdmVibrate)->hOdmServicePmuDevice, (*hOdmVibrate)->VddId, &((*hOdmVibrate)->RailCaps));
            break;
        }
    }
#endif	/* __yuyang(20100615) */

    return NV_TRUE;
}

/**
 *  @brief Closes the ODM device and destroys all allocated resources.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return None.
 */
void NvOdmVibClose(NvOdmVibDeviceHandle hOdmVibrate)
{
#if 1	/* yuyang(20100615):Close I2C handle */
    if (hOdmVibrate != NULL)
    {
        NvOdmServicesPmuClose(hOdmVibrate->hOdmServicePmuDevice);
        hOdmVibrate->hOdmServicePmuDevice = NULL;

        hOdmVibrate->VddId  = 0;
        hOdmVibrate->DeviceAddr = 0;

		if (hOdmVibrate->hOdmI2c)
	        NvOdmI2cClose(hOdmVibrate->hOdmI2c);

        NvOsMemset(&hOdmVibrate->RailCaps, 0, sizeof(NvOdmServicesPmuVddRailCapabilities));

        NvOdmOsFree(hOdmVibrate);
        hOdmVibrate = NULL;
    }
#else
    if (hOdmVibrate != NULL)
    {
        NvOdmServicesPmuClose(hOdmVibrate->hOdmServicePmuDevice);
        hOdmVibrate->hOdmServicePmuDevice = NULL;

        hOdmVibrate->VddId  = 0;

        NvOsMemset(&hOdmVibrate->RailCaps, 0, sizeof(NvOdmServicesPmuVddRailCapabilities));

        NvOdmOsFree(hOdmVibrate);
        hOdmVibrate = NULL;
    }
#endif	/* __yuyang(20100615) */
}


/**
 *  @brief Gets capabilities of the Vibrate device.
 *  @param hOdmVibrate    [IN] Opaque handle to the device.
 *  @param RequestedCaps  [IN] Specifies the capability to get.
 *  @param pCapsValue    [OUT] A pointer to the returned value.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibGetCaps(
    NvOdmVibDeviceHandle hOdmVibrate,
    NvOdmVibCaps RequestedCaps,
    NvU32 *pCapsValue)
{
    NV_ASSERT(hOdmVibrate);
    NV_ASSERT(pCapsValue);

    if (!hOdmVibrate || !pCapsValue)
    {
        return NV_FALSE;
    }

    return NV_TRUE;
}

/**
 *  @brief The frequency to the Vibro motor can be set
 *    using this function. A frequency less than zero will be
 *    clamped to zero and a frequency value beyond the max supported value
 *    will be clamped to the max supported value.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @param Freq         [IN] Frequency in Hz
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibSetFrequency(NvOdmVibDeviceHandle hOdmVibrate, NvS32 Freq)
{
    //AP20 Vibrator does'nt support setting Frequency
    return NV_TRUE;
}

/**
 *  @brief The dutycycle of the PWM driving the Vibro motor can be set
 *    using this function. A dutycycle less than zero will be
 *    clamped to zero and value beyond the max supported value
 *    will be clamped to the max supported value.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @param DCycle       [IN] Duty Cycle value in percentage (0%-100%)
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibSetDutyCycle(NvOdmVibDeviceHandle hOdmVibrate, NvS32 DCycle)
{
    //AP20 Vibrator does'nt support setting DutyCycle
    return NV_TRUE;
}

/**
 *  @brief Starts the Vibro with the frequency and duty-cycle set using the
 *    Set API.
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibStart(NvOdmVibDeviceHandle hOdmVibrate)
{
/** 
 *  yuyang(20100801)
 *  ER vibration behavior is opposite to PR
 */
 
#if 1
	int rv;

	if (Vib_mode) /* ER board */
	{
		rv = I2C_Write(hOdmVibrate, TPS6586x_R5B_PWM, 0xF0);
	}else{ /* PR board */
		rv = I2C_Write(hOdmVibrate, TPS6586x_R5B_PWM, 0x8F);
	}
	
	if (rv != NV_TRUE)
		NV_ODM_TRACE(("Vibrator on: failed!\n"));
#else
    NvU32 SettlingTime = 0;

    NV_ASSERT(hOdmVibrate);

    if (!hOdmVibrate)
    {
        return NV_FALSE;
    }

    if (hOdmVibrate->hOdmServicePmuDevice != NULL)
    {
        // Search for the Vdd rail and power Off the module
        if (hOdmVibrate->VddId)
        {
            NvOdmServicesPmuSetVoltage(hOdmVibrate->hOdmServicePmuDevice,
                   hOdmVibrate->VddId, hOdmVibrate->RailCaps.requestMilliVolts, &SettlingTime);

            if (SettlingTime)
                NvOdmOsWaitUS(SettlingTime);
        }
    }
#endif	/* __yuyang(20100801) */

    return NV_TRUE;
}

/**
 *  @brief Stops the Vibro motor
 *  @param hOdmVibrate  [IN] Opaque handle to the device.
 *  @return NV_TRUE on success and NV_FALSE on error
 */
NvBool
NvOdmVibStop(NvOdmVibDeviceHandle hOdmVibrate)
{
/** 
 *  yuyang(20100801)
 *  ER vibration behavior is opposite to PR
 */
 
#if 1
 	int rv;
	
	if (Vib_mode) /* ER board */
	{
		rv = I2C_Write(hOdmVibrate, TPS6586x_R5B_PWM, 0xFF);
	}else{ /* PR board */
		rv = I2C_Write(hOdmVibrate, TPS6586x_R5B_PWM, 0x00);
	}
	
	if (rv != NV_TRUE)
		NV_ODM_TRACE(("Vibrator off: failed!\n"));
#else
    NvU32 SettlingTime;

    NV_ASSERT(hOdmVibrate);

    if (!hOdmVibrate)
    {
        return NV_FALSE;
    }

    if (hOdmVibrate->hOdmServicePmuDevice != NULL)
    {
        // Search for the Vdd rail and power Off the module
        if (hOdmVibrate->VddId)
        {
            NvOdmServicesPmuSetVoltage(hOdmVibrate->hOdmServicePmuDevice,
                        hOdmVibrate->VddId, NVODM_VOLTAGE_OFF, &SettlingTime);

            if (SettlingTime)
                NvOdmOsWaitUS(SettlingTime);
        }
    }
#endif	/* __yuyang(20100801) */

    return NV_TRUE;
}
