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
#ifndef INCLUDED_NVODM_CAPSENSOR_H
#define INCLUDED_NVODM_CAPSENSOR_H

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvassert.h"


typedef struct NvOdmCapRec *NvOdmAcrDeviceHandle;



///  Opaque handle to an cap sensor object.
typedef struct NvOdmCapRec *NvOdmCapHandle;


/**
 * Initializes the Cap Sensor and allocates resources used by the ODM
 * adaptation driver.
 *
 * @return A handle to the Cap Sensor if initialization is successful, or
 *         NULL if unsuccessful or no accelerometer exists.
 */
NvBool
NvOdmCapOpen(NvOdmCapHandle* hDevice);

/**
 * Disables the Cap Sensor and frees any resources used by the driver.
 *
 * @param hDevice The Cap Sensor handle.
 */
void
NvOdmCapClose(NvOdmCapHandle hDevice);


/**
 * Signals the waiting semaphore.
 *
 * @param hDevice The Cap Sensor handle.
 */
void
NvOdmCapSignal(NvOdmCapHandle hDevice);

void
NvOdmCapWaitInt(NvOdmCapHandle    hDevice);

NvU8
ReadButtonChange(NvOdmCapHandle hDevice);

NvU8
ReadButtonStatus(NvOdmCapHandle hDevice);

NvU32
ReadKeyEvent(NvOdmCapHandle hDevice);

NvBool
NvOdmCapProgSensitivity(NvOdmCapHandle hDevice, NvU32 value);

NvBool
NvOdmCapAdjustSensitivity(NvOdmCapHandle hDevice,NvU32 value);

NvBool
NvOdmCapReadSensitivity(NvOdmCapHandle hDevice,  NvU32* value);

NvBool
NvOdmCapReadDevInfo(NvOdmCapHandle hDevice, NvU32* value);

NvBool
NvOdmCapReadDMI(NvOdmCapHandle hDevice, NvU8* value, NvU8 address);

NvBool 
NvOdmCapCtlr(NvOdmCapHandle hDevice, NvU32 value, NvU32* RegValue);

NvBool
NvOdmCapSuspend(NvOdmCapHandle hDevice);

NvBool
NvOdmResume(NvOdmCapHandle hDevice);


#if defined(__cplusplus)
}
#endif
/** @} */
#endif // INCLUDED_NVODM_CAPSENSOR_H
