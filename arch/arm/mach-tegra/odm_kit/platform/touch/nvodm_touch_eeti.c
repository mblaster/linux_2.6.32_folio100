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

#include "nvodm_touch_int.h"
#include "nvodm_services.h"
#include "nvodm_touch_eeti.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define EETI_I2C_SPEED_KHZ                          400
#define EETI_I2C_TIMEOUT                            1000
#define EETI_MAX_READ_BYTES                         10
#define EETI_BENCHMARK_SAMPLE                       0
#define EETI_SCREEN_ANGLE                           0    //0=Landscape, 1=Portrait
#define EETI_POR_DELAY                              100  //Dealy after Power-On Reset

#define EETI_TOUCH_DEVICE_GUID NV_ODM_GUID('e','e','t','i','t','o','u','c')

#define EETI_DEBOUNCE_TIME_MS 0

struct fingerReport_s {
    NvU8 state;
    NvU8 fingerId;
    NvU16 x;
    NvU16 y;
    NvU16 z;
};
struct fingerReport_s tempReport[4];

typedef struct EETI_TouchDeviceRec {
    NvOdmTouchDevice OdmTouch;
    NvOdmTouchCapabilities Caps;
    NvOdmServicesI2cHandle hOdmI2c;
    NvOdmServicesGpioHandle hGpio;
    NvOdmServicesPmuHandle hPmu;
    NvOdmGpioPinHandle hPin;
    NvOdmServicesGpioIntrHandle hGpioIntr;
    NvOdmOsSemaphoreHandle hIntSema;
    NvBool PrevFingers;
    NvU32 DeviceAddr;
    NvU32 SampleRate;
    NvU32 SleepMode;
    NvBool PowerOn;
    NvU32 VddId;    
    NvU32 ChipRevisionId; 
    NvU32 I2cClockSpeedKHz;
} EETI_TouchDevice;

static const NvOdmTouchCapabilities EETI_Capabilities =
{
    1, //IsMultiTouchSupported
    4, //MaxNumberOfFingerCoordReported;
    0, //IsRelativeDataSupported
    0, //MaxNumberOfRelativeCoordReported
    0, //MaxNumberOfWidthReported
    4, //MaxNumberOfPressureReported
    (NvU32)NvOdmTouchGesture_Not_Supported, //Gesture
    0, //IsWidthSupported
    1, //IsPressureSupported
    1, //IsFingersSupported
    0, //XMinPosition
    0, //YMinPosition
    0, //XMaxPosition
    0, //YMaxPosition
    0  // Available options: 	NvOdmTouchOrientation_H_FLIP, NvOdmTouchOrientation_V_FLIP, 
    // 						NvOdmTouchOrientation_XY_SWAP
};

static NvBool EETI_ProbeTouchDevice (EETI_TouchDevice* hTouch) {
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[EETI_MAX_READ_BYTES];
    NvOdmI2cStatus Error;    

    NvOdmOsMemset(arr, 0, EETI_MAX_READ_BYTES);

    arr[0] = 0x03;
    arr[1] = 0x01;
    arr[2] = (NvU8)'A';

    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 3;

    Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,
                                hTouch->I2cClockSpeedKHz,
                                EETI_I2C_TIMEOUT);

    if (Error != NvOdmI2cStatus_Success) {
        return NV_FALSE;
    }
    return NV_TRUE;

}

static NvBool EETI_GetTouchReport (EETI_TouchDevice* hTouch, NvU8 *ReportData, NvU32 *len) {

    /* Issue i2c transaction to get multi-touch report from the controller */

    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus Error;    

    NvOdmOsMemset(ReportData, 0, EETI_MAX_READ_BYTES);

    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = ReportData;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = EETI_MAX_READ_BYTES;

    do {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    hTouch->I2cClockSpeedKHz,
                                    EETI_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout);

    *len = TransactionInfo.NumBytes;

    if (Error != NvOdmI2cStatus_Success) {
        NvOdmOsPrintf("error!\r\n");
        return NV_FALSE;
    }

    return NV_TRUE;

}

#define INT_PIN_ACTIVE_STATE 0

static NvBool EETI_GetSample (EETI_TouchDevice* hTouch, NvOdmTouchCoordinateInfo* coord) {
    NvU8 Finger[EETI_MAX_READ_BYTES] = {0};

    NvU32 pinValue;
    NvU8 indx = 0;
    NvU8 i = 0;
    NvBool retval = NV_TRUE;
    NvU32 count = 0;
    NvOdmTouchRawI2cData *i2c_data = NULL;

    coord->fingerstate = NvOdmTouchSampleIgnore;
    coord->additionalInfo.Fingers = 0;
    i2c_data = (NvOdmTouchRawI2cData *)coord->pextrainfo;
    retval = EETI_GetTouchReport(hTouch, Finger, &count);

    if (retval == NV_FALSE)
        return retval;

    switch (Finger[0]) {
    case 0x01:
        break;
    case 0x03:
        if (i2c_data != NULL) {
            NvOdmOsMemcpy(i2c_data->datum, Finger, count);
            i2c_data->data_len = count;
        }
        break;
    case 0x04:
        indx = (Finger[1] & 0x7c) >> 2;
        tempReport[indx].fingerId = indx;
        tempReport[indx].state = Finger[1] & 0x1;
        tempReport[indx].x = ((NvU16)Finger[3] << 8) | (NvU16)Finger[2]; 
        tempReport[indx].y = ((NvU16)Finger[5] << 8) | (NvU16)Finger[4]; 
        tempReport[indx].z = (NvU16)Finger[6];
        coord->additionalInfo.Fingers = 4;
        coord->xcoord = coord->additionalInfo.multi_XYCoords[indx][0] = tempReport[indx].x;
        coord->ycoord = coord->additionalInfo.multi_XYCoords[indx][1] = tempReport[indx].y;
        coord->additionalInfo.Pressure[indx] = tempReport[indx].state;
#if 0
        NvOdmOsPrintf("indx = %d, x=%d, y=%d, z=%d\n",
                      indx,
                      coord->additionalInfo.multi_XYCoords[indx][0],
                      coord->additionalInfo.multi_XYCoords[indx][1],
                      coord->additionalInfo.Pressure[indx]);
#endif
        break;
    default:
        break;
    };

    return NV_TRUE;
}

static void InitOdmTouch (NvOdmTouchDevice* Dev) {
    Dev->Close              = EETI_Close;
    Dev->GetCapabilities    = EETI_GetCapabilities;
    Dev->ReadCoordinate     = EETI_ReadCoordinate;
    Dev->EnableInterrupt    = EETI_EnableInterrupt;
    Dev->HandleInterrupt    = EETI_HandleInterrupt;
    Dev->GetSampleRate      = EETI_GetSampleRate;
    Dev->SetSampleRate      = EETI_SetSampleRate;
    Dev->PowerControl       = EETI_PowerControl;
    Dev->PowerOnOff         = EETI_PowerOnOff;
    Dev->GetCalibrationData = EETI_GetCalibrationData;
    Dev->RawI2cWrite        = EETI_RawI2cWrite;
    Dev->RawI2cRead         = EETI_RawI2cRead;
    Dev->OutputDebugMessage = NV_FALSE;
}

NvU32 EETI_RawI2cWrite(NvOdmTouchDeviceHandle hDevice, NvOdmTouchRawI2cData *i2c_data) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    NvOdmI2cTransactionInfo TransactionInfo;
    NvU8 arr[EETI_MAX_READ_BYTES];
    NvOdmI2cStatus Error;    

    NvOdmOsMemset(arr, 0, EETI_MAX_READ_BYTES);

    NvOdmOsMemcpy(arr, i2c_data->datum, i2c_data->data_len);

    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = arr;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = i2c_data->data_len;

    Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                &TransactionInfo,
                                1,
                                hTouch->I2cClockSpeedKHz,
                                EETI_I2C_TIMEOUT);

    if (Error != NvOdmI2cStatus_Success) {
        return 0;
    }

    return TransactionInfo.NumBytes;
}

NvU32 EETI_RawI2cRead(NvOdmTouchDeviceHandle hDevice, NvOdmTouchRawI2cData *i2c_data) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;

    NvOdmI2cTransactionInfo TransactionInfo;
    NvOdmI2cStatus Error;    

    if (i2c_data == NULL)
        return NV_FALSE;

    NvOdmOsMemset(i2c_data->datum, 0, EETI_MAX_READ_BYTES);

    TransactionInfo.Address = hTouch->DeviceAddr;
    TransactionInfo.Buf = i2c_data->datum;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = EETI_MAX_READ_BYTES;

    do {
        Error = NvOdmI2cTransaction(hTouch->hOdmI2c,
                                    &TransactionInfo,
                                    1,
                                    hTouch->I2cClockSpeedKHz,
                                    EETI_I2C_TIMEOUT);
    } while (Error == NvOdmI2cStatus_Timeout);

    if (Error != NvOdmI2cStatus_Success) {
        NvOdmOsPrintf("error!\r\n");
        return NV_FALSE;
    }

    i2c_data->data_len = TransactionInfo.NumBytes;

    return TransactionInfo.NumBytes;
}

static void EETI_GpioIsr(void *arg) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)arg;

    /* Signal the touch thread to read the sample. After it is done reading the
     * sample it should re-enable the interrupt. */
    NvOdmOsSemaphoreSignal(hTouch->hIntSema);            
}

static NvBool EETI_Suspend(NvOdmTouchDeviceHandle hDevice) {
    NvOdmTouchRawI2cData i2c_data;
    static const NvS8 eeti_sleep_cmd[] = {0x03, 0x05, 0x0a, 0x03, 0x03, 0x3f, 0x02, 0x00, 0x00, 0x00};

    NvOdmOsMemcpy(i2c_data.datum, eeti_sleep_cmd, 10);
    i2c_data.data_len = 10;

    EETI_RawI2cWrite(hDevice, &i2c_data);

    return NV_TRUE;
}

static NvBool EETI_Resume(NvOdmTouchDeviceHandle hDevice) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;

    if (hDevice == NULL) {
        return NV_FALSE;
    }
#if 0
    NvOdmGpioInterruptMask(hTouch->hGpioIntr, NV_TRUE);
    NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPin, hTouch->hGpioIntr);
#endif
    NvOdmGpioConfig(hTouch->hGpio, hTouch->hPin, NvOdmGpioPinMode_Output);
    /* Send reset pulse to touch HW */
    NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 1);
    NvOsWaitUS(50);
    NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 0);
    NvOsSleepMS(50);       
    NvOdmGpioSetState(hTouch->hGpio, hTouch->hPin, 1);

    NvOdmGpioConfig(hTouch->hGpio, hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow);
#if 0
    if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
                                   hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow, EETI_GpioIsr,
                                   (void*)hTouch, EETI_DEBOUNCE_TIME_MS) == NV_FALSE) {
        return NV_FALSE;
    }
#endif
    return NV_TRUE;
}

NvBool EETI_ReadCoordinate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo* coord) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;

#if EETI_BENCHMARK_SAMPLE    
    NvU32 time = NvOdmOsGetTimeMS();
#endif
    NVODMTOUCH_PRINTF(("GpioIst+\n"));

    EETI_GetSample(hTouch, coord);


#if EETI_BENCHMARK_SAMPLE    
    NvOdmOsDebugPrintf("Touch sample time %d\n", NvOdmOsGetTimeMS() - time);
#endif

    return NV_TRUE;
}

void EETI_GetCapabilities (NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    *pCapabilities = hTouch->Caps;
}

NvBool EETI_PowerOnOff (NvOdmTouchDeviceHandle hDevice, NvBool OnOff) {
    if (!hDevice)
        return NV_FALSE;

    if (OnOff)
        EETI_Resume(hDevice);
    else
        EETI_Suspend(hDevice);

    return NV_TRUE;
}

NvBool EETI_Open (NvOdmTouchDeviceHandle* hDevice) {
    EETI_TouchDevice* hTouch;
    NvU32 i;
    NvU32 found = 0;
    NvU32 GpioPort = 0;
    NvU32 GpioPin = 0;
    NvU32 I2cInstance = 0;



    const NvOdmPeripheralConnectivity *pConnectivity = NULL;

    hTouch = NvOdmOsAlloc(sizeof(EETI_TouchDevice));
    if (!hTouch) return NV_FALSE;

    NvOdmOsMemset(hTouch, 0, sizeof(EETI_TouchDevice));
    NvOdmOsMemset(tempReport, 0, sizeof(struct fingerReport_s) * 4);

    /* set function pointers */
    InitOdmTouch(&hTouch->OdmTouch);

    pConnectivity = NvOdmPeripheralGetGuid(EETI_TOUCH_DEVICE_GUID);
    if (!pConnectivity) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : pConnectivity is NULL Error \n"));
        goto fail;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : didn't find any periperal in discovery query for touch device Error \n"));
        goto fail;
    }

    for (i = 0; i < pConnectivity->NumAddress; i++) {
        switch (pConnectivity->AddressList[i].Interface) {
        case NvOdmIoModule_I2c:
            hTouch->DeviceAddr = (pConnectivity->AddressList[i].Address << 1);
            I2cInstance = pConnectivity->AddressList[i].Instance;
            found |= 1;
            break;
        case NvOdmIoModule_Gpio:
            GpioPort = pConnectivity->AddressList[i].Instance;
            GpioPin = pConnectivity->AddressList[i].Address;
            found |= 2;
            break;
        case NvOdmIoModule_Vdd:
            hTouch->VddId = pConnectivity->AddressList[i].Address;
            found |= 4;
            break;
        default:
            break;
        }
    }

    if ((found & 3) != 3) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : peripheral connectivity problem \n"));
        goto fail;
    }

    if ((found & 4) != 0) {
        if (NV_FALSE == EETI_PowerOnOff(&hTouch->OdmTouch, 1))
            goto fail;
    } else {
        hTouch->VddId = 0xFF; 
    }

    hTouch->hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, I2cInstance);
    if (!hTouch->hOdmI2c) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmI2cOpen Error \n"));
        goto fail;
    }

    hTouch->hGpio = NvOdmGpioOpen();

    if (!hTouch->hGpio) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : NvOdmGpioOpen Error \n"));
        goto fail;
    }

    hTouch->hPin = NvOdmGpioAcquirePinHandle(hTouch->hGpio, GpioPort, GpioPin);
    if (!hTouch->hPin) {
        NVODMTOUCH_PRINTF(("NvOdm Touch : Couldn't get GPIO pin \n"));
        goto fail;
    }

    NvOdmGpioConfig(hTouch->hGpio,
                    hTouch->hPin,
                    NvOdmGpioPinMode_InputData);

    NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &i);

    /* set default capabilities */
    NvOdmOsMemcpy(&hTouch->Caps, &EETI_Capabilities, sizeof(NvOdmTouchCapabilities));

    /* set default I2C speed */
    hTouch->I2cClockSpeedKHz = EETI_I2C_SPEED_KHZ;
#if 1
    if (EETI_ProbeTouchDevice (hTouch) != NV_TRUE) {
        NvOdmOsPrintf("NvOdm Touch : Multitouch detection failure \n");
        goto fail;
    }
#endif
    hTouch->Caps.XMaxPosition = 32767;
    hTouch->Caps.YMaxPosition = 32767;

    *hDevice = &hTouch->OdmTouch;
    return NV_TRUE;

    fail:
    EETI_Close(&hTouch->OdmTouch);
    return NV_FALSE;
}

NvBool EETI_EnableInterrupt (NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hIntSema) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    NvOdmTouchCoordinateInfo coord;

    NV_ASSERT(hIntSema);

    /* can only be initialized once */
    if (hTouch->hGpioIntr || hTouch->hIntSema)
        return NV_FALSE;

    NvOdmOsMemset(&coord, 0, sizeof(NvOdmTouchCoordinateInfo));

    /* zero intr status */
    EETI_GetSample(hTouch, &coord);        

    hTouch->hIntSema = hIntSema;    

    if (NvOdmGpioInterruptRegister(hTouch->hGpio, &hTouch->hGpioIntr,
                                   hTouch->hPin, NvOdmGpioPinMode_InputInterruptLow, EETI_GpioIsr,
                                   (void*)hTouch, EETI_DEBOUNCE_TIME_MS) == NV_FALSE) {
        return NV_FALSE;
    }

    if (!hTouch->hGpioIntr)
        return NV_FALSE;

    return NV_TRUE;
}

NvBool EETI_HandleInterrupt(NvOdmTouchDeviceHandle hDevice) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    NvU32 pinValue;

    NvOdmGpioGetState(hTouch->hGpio, hTouch->hPin, &pinValue);
    if (!pinValue) {
        //interrupt pin is still LOW, read data until interrupt pin is released.
        return NV_FALSE;
    } else
        NvOdmGpioInterruptDone(hTouch->hGpioIntr);

    return NV_TRUE;
}

NvBool EETI_GetSampleRate (NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    pTouchSampleRate->NvOdmTouchSampleRateHigh = 80;
    pTouchSampleRate->NvOdmTouchSampleRateLow = 40;
    pTouchSampleRate->NvOdmTouchCurrentSampleRate = (hTouch->SampleRate >> 1);
    return NV_TRUE;
}

NvBool EETI_SetSampleRate (NvOdmTouchDeviceHandle hDevice, NvU32 rate) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;

    if (rate != 0 && rate != 1)
        return NV_FALSE;

    rate = 1 << rate;

    if (hTouch->SampleRate == rate)
        return NV_TRUE;

    hTouch->SampleRate = rate;
    return NV_TRUE;
}


NvBool EETI_PowerControl (NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;
    NvU32 SleepMode;

    NV_ASSERT(hTouch->VddId != 0xFF);

    switch (mode) {
    case NvOdmTouch_PowerMode_0:
        SleepMode = 0x0;
        break;
    case NvOdmTouch_PowerMode_1:
    case NvOdmTouch_PowerMode_2:
    case NvOdmTouch_PowerMode_3:
        SleepMode = 0x03;
        break;
    default:
        return NV_FALSE;
    }

    if (hTouch->SleepMode == SleepMode)
        return NV_TRUE;

    hTouch->SleepMode = SleepMode;    
    return NV_TRUE;
}

NvBool EETI_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer) {
#if EETI_SCREEN_ANGLE
    //Portrait
    static const NvS32 RawCoordBuffer[] = {2054, 3624, 3937, 809, 3832, 6546, 453, 6528, 231, 890};
#else
    //Landscape
    static NvS32 RawCoordBuffer[] = {2054, 3624, 3832, 6546, 453, 6528, 231, 890, 3937, 809};
#endif

    if (NumOfCalibrationData*2 != (sizeof(RawCoordBuffer)/sizeof(NvS32))) {
        NVODMTOUCH_PRINTF(("WARNING: number of calibration data isn't matched\n"));
        return NV_FALSE;
    }

    NvOdmOsMemcpy(pRawCoordBuffer, RawCoordBuffer, sizeof(RawCoordBuffer));

    return NV_TRUE;
}


void EETI_Close (NvOdmTouchDeviceHandle hDevice) {
    EETI_TouchDevice* hTouch = (EETI_TouchDevice*)hDevice;

    if (!hTouch) return;

    if (hTouch->hGpio) {
        if (hTouch->hPin) {
            if (hTouch->hGpioIntr)
                NvOdmGpioInterruptUnregister(hTouch->hGpio, hTouch->hPin, hTouch->hGpioIntr);

            NvOdmGpioReleasePinHandle(hTouch->hGpio, hTouch->hPin);
        }

        NvOdmGpioClose(hTouch->hGpio);
    }

    if (hTouch->hOdmI2c)
        NvOdmI2cClose(hTouch->hOdmI2c);

    NvOdmOsFree(hTouch);
}


