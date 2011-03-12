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

#ifndef INCLUDED_NVODM_DOCK_H
#define INCLUDED_NVODM_DOCK_H
#if defined(_cplusplus)
extern "C"
{
#endif


#include "nvodm_services.h"
#include "nvassert.h"

/**
 * Defines the DOCK context.
 */
typedef struct NvOdmDockRec *NvOdmDockHandle;

/**
 * Opens the DOCK device
 * @param Instance The DOCK instance number.
 * @return A DOCK device handle on success, or NULL on failure.
*/
NvBool NvOdmDockOpen(NvOdmDockHandle *hDevice);

/** 
 * Closes the DOCK device handle by clearing 
 * the related ODM-specific settings.
 * @param hOdmDock A handle to DOCK device.
*/
void NvOdmDockClose(NvOdmDockHandle hOdmDock);


void
NvOdmDockWaitInt(NvOdmDockHandle    hDevice);

NvBool
NvOdmGetDockState(NvOdmDockHandle hDevice);

NvBool
NvOdmGetDockInitialState(NvOdmDockHandle hDevice);

void
NvOdmDockSignalInt(NvOdmDockHandle hDevice);

void NvOdmDockInterruptMask(NvOdmDockHandle hDevice);

void NvOdmDockInterruptUnMask(NvOdmDockHandle hDevice);
#if defined(_cplusplus)
}
#endif

/** @} */

#endif // INCLUDED_NVODM_DOCK_H

