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
#define NVODMUSBOVCUR_ENABLE_PRINTF 1

#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

/* paul merge smith begin */
//#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h> 
#include "nvodm_ovcur.h"
/* paul merge smith end */

extern NvOdmGpioPinHandle s_hOvrrCurPin;

#if NVODMUSBOVCUR_ENABLE_PRINTF
    #define NVODMUSBOVCUR_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODMUSBOVCUR_PRINTF(x)
#endif

#define NV_DEBOUNCE_TIME_MS 0
#define USBOVCUR_GUID NV_ODM_GUID('u','s','b','o','v','r','c','u')

NvU32 gUsbCurrLimitC = 1; /* initial pin state should be 1 */
NvOsMutexHandle usbCurrLimit_lock = NULL;
//NvOdmServicesGpioIntrHandle IntrHandle = NULL;

static struct proc_dir_entry  *proc_usbovc;
static struct proc_dir_entry  *proc_ioctl;

/* paul */
/* get global variable gUsbCurrLimitC value and set it into /proc/usbCurrLimitInfo for user space read */
static int tegra_usbCurrLimit_read_proc(char *page, char **start, off_t off,
       int count, int *eof, void *data)
{
       int len = 0;

	/*
       NV_DRIVER_TRACE (("tegra_usbCurrLimit_read_proc:start\n"));	
	*/
       NvOsMutexLock(usbCurrLimit_lock);
       len += snprintf(page+len, count-len,"%d", gUsbCurrLimitC);
       NvOsMutexUnlock(usbCurrLimit_lock);

       *eof = 1;
    /*   
       NV_DRIVER_TRACE (("tegra_usbCurrLimit_read_proc:end\n"));
    */ 
       return len;
}

static int tegra_usbCurrLimit_write_proc (struct file *file, const char *buffer, unsigned long count,
                   void *data)
{
	int len = 0;
	static char proc_buf[4];
	//return len;
	
	if ( copy_from_user(proc_buf, buffer, count) ) {
	        return -EFAULT;
	}
	if ( strncmp( proc_buf, "1", 1) == 0 )
	{
		gUsbCurrLimitC = 1;
		NvOsMutexLock(usbCurrLimit_lock);
		len += snprintf(proc_buf, count-len, "%d",gUsbCurrLimitC);
		NvOsMutexUnlock(usbCurrLimit_lock);
		return len;
	}
	else
	{
		NvOsDebugNprintf("tegra_usbCurrLimit_write_proc fail\n");
		return -1;
	}
}    

/* get GPIO_PU3 status and set it into global variable gUsbCurrLimitC */
static void UsbCurLimitGpioInterruptHandler(void *arg)
{
	NvU32 CurrSelectPinState = 0;
	NvOdmUsbOvcurHandle hDevice =  (NvOdmUsbOvcurHandle)arg;
	
	NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &CurrSelectPinState);

	gUsbCurrLimitC = CurrSelectPinState;
	
	NvOsDebugNprintf ("CurrSelectPinState = %d\n",CurrSelectPinState);

	NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
	NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
}


NvU32 procfile_init(void)
{
	proc_usbovc = proc_mkdir("usbCurrLimitInfo" ,NULL);
	if ( proc_usbovc == NULL )
	return -1;
	proc_ioctl = create_proc_entry("usbovc", S_IRWXUGO, proc_usbovc);
	if (proc_ioctl == NULL) {
	        NvOsDebugNprintf("Error: Could not initialize /proc/ \n");
	        return -ENOMEM;
	}
	proc_ioctl->read_proc  = tegra_usbCurrLimit_read_proc;
	proc_ioctl->write_proc = tegra_usbCurrLimit_write_proc;
	return 0;
}
/* paul merge smith end */



static NvBool ConnectSemaphore(NvOdmUsbOvcurHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)UsbCurLimitGpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODMUSBOVCUR_PRINTF(("USB GPIO_PU3 driver: NvOdmGpioOpenError \n"));
        return NV_FALSE;
    }

	if (s_hOvrrCurPin)
	hDevice->hPinINT = s_hOvrrCurPin;
	else
	{
	    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
	                           hDevice->GPIOPortINT,
	                           hDevice->GPIOPinINT);
	}
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODMUSBOVCUR_PRINTF(( "USB GPIO_PU3 driver: NvOdmOsSemaphoreCreate Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }


    /* create mutex for usb over current detect */
    NvOsMutexCreate(&usbCurrLimit_lock);
    procfile_init();

    mode = NvOdmGpioPinMode_InputInterruptFallingEdge;
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }
	
    if (!(hDevice->hGpioInterrupt))
    {
        NVODMUSBOVCUR_PRINTF(("USB GPIO_PU3 driver:GPIO_PU3 NvOdmGpioInterruptRegister Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    NVODMUSBOVCUR_PRINTF(("Nvodm USBOVC ConnectSemaphore Sucess"));
    return NV_TRUE;
}

void NvOdmUsbOvcurSuspend(NvOdmUsbOvcurHandle hDevice)
{
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_TRUE);
	NvOsDebugNprintf("NvOdmUsbOvcurSuspend Success \n");
}

void NvOdmUsbOvcurResume(NvOdmUsbOvcurHandle hDevice)
{
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_FALSE);   
	NvOsDebugNprintf("NvOdmUsbOvcurResume Success \n");
}



NvBool NvOdmUsbOvcurOpen(NvOdmUsbOvcurHandle* hDevice)
{
	NvU32 i;
	NvOdmUsbOvcurHandle	hUsbOvcur;
	
	const NvOdmPeripheralConnectivity *pConnectivity;
	NvBool FoundGpio = NV_FALSE;

	hUsbOvcur = NvOdmOsAlloc(sizeof(NvOdmUsbOvcurevent));
	if (hUsbOvcur == NULL)
	{
		NVODMUSBOVCUR_PRINTF(("USB GPIO_PU3 driver: Open fail\n"));
		return NV_FALSE;
	}
	NvOdmOsMemset(hUsbOvcur,0,sizeof(NvOdmUsbOvcurevent));
	
	
	// Info of ecompass with current setting
	
    pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(
                        USBOVCUR_GUID);
    if (!pConnectivity)
    {
        NvOdmOsDebugPrintf(("NvOdmPeripheralGetGuid doesn't detect\
           USB GPIO_PU3 device\n"));
        goto error;
    }
	
    if (pConnectivity->Class != NvOdmPeripheralClass_Other)
        goto error;

    for( i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch(pConnectivity->AddressList[i].Interface)
        {

            case NvOdmIoModule_Gpio:
                hUsbOvcur->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hUsbOvcur->GPIOPinINT = pConnectivity->AddressList[i].Address;				
                FoundGpio = NV_TRUE;
                break;
            default:
                break;
        }
    }
	
	
    if (!FoundGpio)
    {
        NVODMUSBOVCUR_PRINTF(("GPIO_PU3 driver: didn't find any periperal in discovery query for touch device Error \n"));
        goto error;
    }

    
	if (!ConnectSemaphore(hUsbOvcur))
		goto error;
	
	*hDevice = hUsbOvcur;
	return NV_TRUE;
    error:
		NVODMUSBOVCUR_PRINTF(("GPIO PU3 driver: Error during NvOdmUsbOvcurOpen\n"));
		// Release all of resources requested.
		if (hUsbOvcur){
            NvOdmOsFree(hUsbOvcur);
            *hDevice = NULL;		
		}
		return NV_FALSE;
}


void NvOdmUsbOvcurClose(NvOdmUsbOvcurHandle hDevice)
{
	if (hDevice)
	{
        if (hDevice->SemaphoreForINT && hDevice->hGpioINT &&
            hDevice->hPinINT && hDevice->hGpioInterrupt)
		{
            NvOdmGpioInterruptUnregister(hDevice->hGpioINT,
                hDevice->hPinINT, hDevice->hGpioInterrupt);
            NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
            NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
            NvOdmGpioClose(hDevice->hGpioINT);		
		}
	}
}


void NvOdmUsbOvcurWaitInt(NvOdmUsbOvcurHandle hDevice)
{
	NV_ASSERT(hDevice);
	NvOdmOsSemaphoreWait(hDevice->SemaphoreForINT);
}

void NvOdmUsbOvcurSignal(NvOdmUsbOvcurHandle hDevice)
{
	NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

