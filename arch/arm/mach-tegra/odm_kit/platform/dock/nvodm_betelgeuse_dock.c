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
 
/*  NVIDIA Tegra ODM Kit  Dock Adaptation of the
 *  Dock Driver
 */

#include "nvodm_betelgeuse_dock.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_query_gpio.h"
#include "nvos.h"

#define NVODM_DOCK_ENABLE_PRINTF 0

#if NVODM_DOCK_ENABLE_PRINTF
    #define NVODM_DOCK_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODM_DOCK_PRINTF(x)
#endif



#define NV_DEBOUNCE_TIME_MS 5


static void GpioInterruptHandler(void *arg)
{
    NvU32 pinValue;
    NvOdmDockHandle hDevice =  (NvOdmDockHandle)arg; 
 
    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue); 
  
    if(pinValue == 0){
        hDevice->DockState = 1;
         NVODM_DOCK_PRINTF((
            "Dock\n"));
    }
    
    if(pinValue == 1){
        hDevice->DockState = 0;
         NVODM_DOCK_PRINTF((
            "UnDock \n"));
    }
    
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);            
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

static NvBool ConnectSemaphore(NvOdmDockHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback = (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODM_DOCK_PRINTF((
            "NvOdm Cap Sensor : NvOdmGpioOpen Error \n"));
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
  		
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODM_DOCK_PRINTF((
            "NvOdm Cap Sensor : NvOdmOsSemaphoreCreate Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }

  
    mode = NvOdmGpioPinMode_InputInterruptAny;
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if (!(hDevice->hGpioInterrupt))
    {
        NVODM_DOCK_PRINTF((
            "NvOdm Cap Sensor : NvOdmGpioInterruptRegister Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    
    
    return NV_TRUE;
}

void NvOdmDockInterruptMask(NvOdmDockHandle hDevice){
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_TRUE);
}

void NvOdmDockInterruptUnMask(NvOdmDockHandle hDevice){
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_FALSE);
}
    
NvBool NvOdmDockOpen(NvOdmDockHandle *hDevice)
{
    NvOdmDockHandle  hDock;
    const NvOdmGpioPinInfo *GpioPinInfo;
    hDock = NvOdmOsAlloc(sizeof(NvOdmDock));
    if (hDock == NULL)
    {
        NVODM_DOCK_PRINTF(("Error Allocating NvOdmDock. \n"));
        return NV_FALSE;
    }
    
    NvOdmOsMemset(hDock, 0, sizeof(NvOdmDock));     
   
   	/* Check the supported GPIOs */
	GpioPinInfo = NvOdmQueryGpioPinMap(NvOdmGpioPinGroup_Docking,
					0, &hDock->PinCount);
	   
    hDock->GPIOPortINT = GpioPinInfo->Port;
    hDock->GPIOPinINT  = GpioPinInfo->Pin;
    if (!ConnectSemaphore(hDock))
        goto error;

    *hDevice = hDock;
    return NV_TRUE;
error:
        NVODM_DOCK_PRINTF(("Error during Dock NvOdmDocOpen\n"));
        
        if (hDock)
        {
     
           NvOdmOsFree(hDock);
           *hDevice = NULL;
        }
        return NV_FALSE;
}

void NvOdmDockClose(NvOdmDockHandle hDevice)
{
    if (hDevice)
    {
        if (hDevice->SemaphoreForINT && hDevice->hGpioINT &&
            hDevice->hPinINT && hDevice->hGpioInterrupt)
        {
            NvOdmGpioInterruptUnregister(hDevice->hGpioINT,hDevice->hPinINT, hDevice->hGpioInterrupt);
            NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
            NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
            NvOdmGpioClose(hDevice->hGpioINT);
        }
     

    }
}

void
NvOdmDockWaitInt(NvOdmDockHandle hDevice)
{
    NV_ASSERT(hDevice);  
    NvOdmOsSemaphoreWait(hDevice->SemaphoreForINT);
}

void
NvOdmDockSignalInt(NvOdmDockHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT); 
}

NvBool
NvOdmGetDockState(NvOdmDockHandle hDevice)
{
     return  hDevice->DockState;
} 

NvBool
NvOdmGetDockInitialState(NvOdmDockHandle hDevice)
{
    NvU32 pinValue; 
    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue); 
    if(pinValue == 0){
        hDevice->DockState = 1;
         NVODM_DOCK_PRINTF((
            "Dock\n"));
    
    }
    
    if(pinValue == 1){
        hDevice->DockState = 0;
         NVODM_DOCK_PRINTF((
            "UnDock\n"));
    
    }
    
    return  (hDevice->DockState==1)? 1:0;
}       