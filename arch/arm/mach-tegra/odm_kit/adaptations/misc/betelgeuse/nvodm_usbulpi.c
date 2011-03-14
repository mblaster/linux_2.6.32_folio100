/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/**
 * @file          nvodm_usbulpi.c
 * @brief         <b>Adaptation for USB ULPI </b>
 *
 * @Description : Implementation of the USB ULPI adaptation.
 */
#include "nvodm_usbulpi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvodm_services.h"
#include "nvos.h"

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
/* paul merge smith begin */
//#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h> 
/* paul merge smith end */
#endif
#define SMSC3317GUID NV_ODM_GUID('s','m','s','c','3','3','1','7')

#define MAX_CLOCKS 3

#define NVODM_PORT(x) ((x) - 'a')
#define ULPI_RESET_PORT NVODM_PORT('v')
#define ULPI_RESET_PIN 0
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
#define ULPI_OVRCURR_PORT NVODM_PORT('u')
#define ULPI_OVRCURR_PIN 3 
#endif
#ifdef NV_DRIVER_DEBUG
    #define NV_DRIVER_TRACE NvOsDebugPrintf
#else
    #define NV_DRIVER_TRACE (void)
#endif

typedef struct NvOdmUsbUlpiRec
{
     NvU64    CurrentGUID;
} NvOdmUsbUlpi;

static NvOdmServicesGpioHandle s_hGpio = NULL;
static NvOdmGpioPinHandle s_hResetPin = NULL;

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
static NvOdmGpioPinHandle s_hOvrrCurPin = NULL;

NvU32 gUsbCurrLimitC = 1; /* initial pin state should be 1 */
NvOsMutexHandle usbCurrLimit_lock = NULL;
NvOdmServicesGpioIntrHandle IntrHandle = NULL;
int i = 0;
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
	
	

	
	NvOdmGpioGetState(s_hGpio, s_hOvrrCurPin, &CurrSelectPinState);

	NvOsDebugNprintf ("###INterrupt#####\n");
	//gUsbCurrLimitC = ( CurrSelectPinState ) ? 1 : 0;
	gUsbCurrLimitC = CurrSelectPinState;
	
	NvOsDebugNprintf ("CurrSelectPinState = %d\n",CurrSelectPinState);
	NvOsDebugNprintf("======>i=%d, gUsbCurrLimitC=%d\n", i, gUsbCurrLimitC);
	i = i++;
	NvOdmGpioInterruptDone(IntrHandle);
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
}
/* paul merge smith end */
#endif
NvOdmUsbUlpiHandle NvOdmUsbUlpiOpen(NvU32 Instance)
{
    NvOdmUsbUlpi*pDevice = NULL;
    NvU32 ClockInstances[MAX_CLOCKS];
    NvU32 ClockFrequencies[MAX_CLOCKS];
    NvU32 NumClocks;
#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
/* paul */
    NvOdmInterruptHandler IntrHandler = (NvOdmInterruptHandler)UsbCurLimitGpioInterruptHandler;
/* end */
#endif
    pDevice = NvOdmOsAlloc(sizeof(NvOdmUsbUlpi));
    if(pDevice == NULL)
		return NULL;
    
    if(!NvOdmExternalClockConfig(SMSC3317GUID, NV_FALSE, ClockInstances,
					ClockFrequencies, &NumClocks))
    {
        NV_DRIVER_TRACE (("ERROR NvOdmUsbUlpiOpen: "
				"NvOdmExternalClockConfig fail\n"));
        goto ExitUlpiOdm;
    }
    NvOdmOsSleepMS(10);

    if (!s_hGpio)
        s_hGpio = NvOdmGpioOpen();
    if (!s_hGpio)
    {
        NV_DRIVER_TRACE (("ERROR NvOdmUsbUlpiOpen: "
				"Not able to open gpio handle\n"));
        goto ExitUlpiOdm;
    }

    if (!s_hResetPin)
        s_hResetPin = NvOdmGpioAcquirePinHandle(s_hGpio, ULPI_RESET_PORT,
							ULPI_RESET_PIN);
    if (!s_hResetPin)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
        NV_DRIVER_TRACE (("ERROR NvOdmGpioAcquirePinHandle: "
					"Not able to Acq pinhandle\n"));
        goto ExitUlpiOdm;
    }

#if defined(CONFIG_TEGRA_ODM_BETELGEUSE)
    if (!s_hOvrrCurPin)
        s_hOvrrCurPin = NvOdmGpioAcquirePinHandle(s_hGpio, ULPI_OVRCURR_PORT,
							ULPI_OVRCURR_PIN);

    if (!s_hOvrrCurPin)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
        NV_DRIVER_TRACE (("ERROR NvOdmGpioAcquirePinHandle: "
					"Not able to Acq pinhandle\n"));
        goto ExitUlpiOdm;
    }
    NvOdmGpioConfig(s_hGpio,s_hOvrrCurPin, NvOdmGpioPinMode_InputData);
#endif

    // Pull high on RESETB ( 22nd pin of smsc3315) 
    // config as out put pin
    NvOdmGpioConfig(s_hGpio,s_hResetPin, NvOdmGpioPinMode_Output);

    // Set low to write high on ULPI_RESETB pin
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x01);
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x0);
    NvOdmOsSleepMS(5);
    NvOdmGpioSetState(s_hGpio, s_hResetPin, 0x01);

#if defined(BUG_CONFIG_TEGRA_ODM_BETELGEUSE)
/* paul merge smith begin */

    /* create mutex for usb over current detect */
    NvOsMutexCreate(&usbCurrLimit_lock);

#if 0
    /* create /proc/usbCurrLimitInfo for user space read */ /* S_IRUGO */
    create_proc_read_entry("usbCurrLimitInfo", S_IRWXUGO, NULL, tegra_usbCurrLimit_read_proc, NULL);
#else

    procfile_init();
    
#endif
    /* register interrupt handler for GPIO_PU3 status */
    if (NvOdmGpioInterruptRegister(s_hGpio, &IntrHandle,
        	s_hOvrrCurPin, NvOdmGpioPinMode_InputInterruptLow,
        	IntrHandler, (void *)NULL, 0) == NV_FALSE)
    {
		NV_DRIVER_TRACE (("ERROR NvOdmGpioInterruptRegister: "
					"Not able to register intr hdlr for s_hCurLimitPin\n"));
    }

/* paul merge smith end */
#endif

    pDevice->CurrentGUID = SMSC3317GUID;
    return pDevice;

ExitUlpiOdm:
    NvOdmOsFree(pDevice);
    return NULL;
}

void NvOdmUsbUlpiClose(NvOdmUsbUlpiHandle hOdmUlpi)
{
    if (hOdmUlpi)
    {
        NvOdmOsFree(hOdmUlpi);
    }
    if (s_hResetPin)
    {
        NvOdmGpioReleasePinHandle(s_hGpio, s_hResetPin);
        s_hResetPin = NULL;
    }
    if (s_hGpio)
    {
        NvOdmGpioClose(s_hGpio);
        s_hGpio = NULL;
    }

}

