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
 *
 */

#include "nvodm_cir.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvodm_query_gpio.h"
#include "nvrm_gpio.h"
#include "nvec.h"
#include <linux/module.h> /*add printk*/

// Module debug: 0=disable, 1=enable
#define NVODM_ENABLE_PRINTF      0

#if NVODM_ENABLE_PRINTF
#define NVODM_PRINTF(x) NvOdmOsDebugPrintf x
#else
#define NVODM_PRINTF(x)
#endif



static NvEcHandle s_NvEcHandle = NULL;  // nvec handle
static NvEcEventType EventTypes[] = {NvEcEventType_OEM1};  // get only CIR (0xE) events from EC
NvEcEvent CirEvent = {0};
static NvOdmOsSemaphoreHandle s_hCirKeyScanRecvSema = NULL;
static NvEcEventRegistrationHandle s_hEcEventRegistration = NULL;
static NvBool s_CirDeinit = NV_FALSE;



NvBool NvOdmCirInit(void) /*jolen 0701*/
{
    NvError NvStatus = NvError_Success;

    /* get nvec handle */
    NvStatus = NvEcOpen(&s_NvEcHandle, 0 /* instance */);
    if (NvStatus != NvError_Success)
    {
        goto fail;
    }
    /* create semaphore which can be used to send scan codes to the clients */
    s_hCirKeyScanRecvSema = NvOdmOsSemaphoreCreate(0);
    if (!s_hCirKeyScanRecvSema)
    {
        goto cleanup;
    }

    /* register for CIR events */
    NvStatus = NvEcRegisterForEvents(
                    s_NvEcHandle,       // nvec handle
                    &s_hEcEventRegistration,
                    (NvOsSemaphoreHandle)s_hCirKeyScanRecvSema,
                    sizeof(EventTypes)/sizeof(NvEcEventType),
                    EventTypes, // receive Ir remote scan codes
                    1,          // currently buffer only 1 packet from ECI at a time
                    sizeof(NvEcEvent));
    if (NvStatus != NvError_Success)
    {
        goto cleanup;
    }

    /* success */
    return NV_TRUE;

cleanup:
    (void)NvEcUnregisterForEvents(s_hEcEventRegistration);
    s_hEcEventRegistration = NULL;

    NvOdmOsSemaphoreDestroy(s_hCirKeyScanRecvSema);
    s_hCirKeyScanRecvSema = NULL;

    NvEcClose(s_NvEcHandle);
fail:
    s_NvEcHandle = NULL;

    return NV_FALSE;
}

void NvOdmCirDeInit(void)
{
	
    (void)NvEcUnregisterForEvents(s_hEcEventRegistration);
    s_hEcEventRegistration = NULL;

    s_CirDeinit = NV_TRUE;
    NvOdmOsSemaphoreSignal(s_hCirKeyScanRecvSema);
    NvOdmOsSemaphoreDestroy(s_hCirKeyScanRecvSema);
    s_hCirKeyScanRecvSema = NULL;

    NvEcClose(s_NvEcHandle);
    s_NvEcHandle = NULL;
}

/* Gets the actual scan code for a key press */
NvBool NvOdmCirGetKeyData(NvU8 showlog, NvU32 *pKeyScanCode, NvU8 *pRePeat, NvU32 Timeout)
{
    NvError NvStatus = NvError_Success;
    NvU32 OutCode, i;
    NvU8 RepeatKey;
	
    if (!pKeyScanCode || !pRePeat || s_CirDeinit)
    {
        return NV_FALSE;
    }

    if (Timeout != 0)
    {
        /* Use the timeout value */
        if (!NvOdmOsSemaphoreWaitTimeout(s_hCirKeyScanRecvSema, Timeout))
            return NV_FALSE; // timed out
    }
    else
    {
        /* wait till we receive a scan code from the EC */
        NvOdmOsSemaphoreWait(s_hCirKeyScanRecvSema);
    }
	//NvOsDebugPrintf("$$$$$$$ In kernel  cir nvodm_cir.c !! get key data  \n");
    // stop scanning
    if (s_CirDeinit)
        return NV_FALSE;

    if (s_hEcEventRegistration)
    {
        NvStatus = NvEcGetEvent(s_hEcEventRegistration, &CirEvent, sizeof(NvEcEvent));
        if (NvStatus != NvError_Success)
        {
            NV_ASSERT(!"Could not receive scan code");
            return NV_FALSE;
        }
        if (CirEvent.NumPayloadBytes == 0)
        {
            NV_ASSERT(!"Received Cir event with no scan codes");
            return NV_FALSE;
        }

		if(showlog){
		for (i = 0; i < CirEvent.NumPayloadBytes; i++){
			printk(KERN_INFO "EC Payload[%d]=0x%x\n",i,CirEvent.Payload[i]);
		}
		}
		
		RepeatKey = 0x0;	/*default is new key input*/
		if(CirEvent.Payload[1] == NV_ODM_CIR_SCAN_CODE_REPET){
			RepeatKey = NV_ODM_CIR_SCAN_CODE_REPET;
		}
				
		OutCode = CirEvent.Payload[3]; /*Ir Command */
		if(showlog)
			printk(KERN_INFO "nvec_cir OUT code: 0x%x,RepeatKey =%x\n", OutCode,RepeatKey );

        *pRePeat = RepeatKey ;
        *pKeyScanCode = OutCode;
        return NV_TRUE;
    }

    return NV_FALSE;
}

