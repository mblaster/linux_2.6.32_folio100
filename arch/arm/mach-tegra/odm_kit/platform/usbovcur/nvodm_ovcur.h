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

#ifndef INCLUDED_NVODM_USBOVCUR_L_H
#define INCLUDED_NVODM_USBOVCUR_L_H

#if defined(_cplusplus)
extern "C"
{
#endif

#include "nvodm_services.h"
#include "nvodm_usbovcur.h"

typedef struct NvOdmUsbOvcurRec
{
    // Interrupt pin to ap15.
    NvOdmServicesGpioHandle hGpioINT;
    NvOdmGpioPinHandle      hPinINT;
    NvU32 GPIOPortINT;
    NvU32 GPIOPinINT;
    NvOdmOsSemaphoreHandle SemaphoreForINT;
    NvOdmServicesGpioIntrHandle hGpioInterrupt;

} NvOdmUsbOvcurevent;

#endif