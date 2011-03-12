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

#ifndef INCLUDED_NVODM_TOUCH_EETI_H
#define INCLUDED_NVODM_TOUCH_EETI_H

#include "nvodm_touch_int.h"
#include "nvodm_services.h"

#if defined(__cplusplus)
extern "C"
{
#endif

NvBool EETI_Open( NvOdmTouchDeviceHandle *hDevice);

void EETI_GetCapabilities(NvOdmTouchDeviceHandle hDevice, NvOdmTouchCapabilities* pCapabilities);

NvBool EETI_ReadCoordinate( NvOdmTouchDeviceHandle hDevice, NvOdmTouchCoordinateInfo *coord);

NvBool EETI_EnableInterrupt(NvOdmTouchDeviceHandle hDevice, NvOdmOsSemaphoreHandle hInterruptSemaphore);

NvBool EETI_HandleInterrupt(NvOdmTouchDeviceHandle hDevice);

NvBool EETI_GetSampleRate(NvOdmTouchDeviceHandle hDevice, NvOdmTouchSampleRate* pTouchSampleRate);

NvBool EETI_SetSampleRate(NvOdmTouchDeviceHandle hDevice, NvU32 rate);

NvBool EETI_PowerControl(NvOdmTouchDeviceHandle hDevice, NvOdmTouchPowerModeType mode);

NvBool EETI_GetCalibrationData(NvOdmTouchDeviceHandle hDevice, NvU32 NumOfCalibrationData, NvS32* pRawCoordBuffer);

NvU32 EETI_RawI2cWrite(NvOdmTouchDeviceHandle hDevice, NvOdmTouchRawI2cData *i2c_data);

NvU32 EETI_RawI2cRead(NvOdmTouchDeviceHandle hDevice, NvOdmTouchRawI2cData *i2c_data);

NvBool EETI_PowerOnOff(NvOdmTouchDeviceHandle hDevice, NvBool OnOff);

void EETI_Close( NvOdmTouchDeviceHandle hDevice);


#if defined(__cplusplus)
}
#endif


#endif // INCLUDED_NVODM_TOUCH_EETI_H
