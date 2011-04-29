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


#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>

#include <nvodm_services.h>
#include <nvodm_usbovcur.h>

#define DRIVER_NAME "nvodm_usbovcur"

struct tegra_usbovcur_device_data
{
	NvOdmUsbOvcurDeviceHandle	hOdmUsbOvcur;
	struct task_struct	*task;
	NvBool			bThreadAlive;
	NvBool			show_log;
};

struct tegra_usbovcur_device_data *UsbOvcur_dev;



static NvS32 tegra_usbovcur_remove(struct platform_device *pdev)
{
	struct tegra_usbovcur_device_data *usbovcur = platform_get_drvdata(pdev);
	usbovcur->bThreadAlive = 0;
	return 0;
}

static int tegra_usbovcur_suspend(struct platform_device *pdev, pm_message_t state)
{
     struct tegra_usbovcur_device_data *usbovcur = platform_get_drvdata(pdev);
     NvOdmUsbOvcurSuspend (usbovcur->hOdmUsbOvcur);
     return 0;
}

static int tegra_usbovcur_resume(struct platform_device *pdev)
{
     struct tegra_usbovcur_device_data *usbovcur = platform_get_drvdata(pdev);
     NvOdmUsbOvcurResume (usbovcur->hOdmUsbOvcur);
	  return 0;
}

void close_odm_usbovcur(void)
{
	NvOdmUsbOvcurClose(UsbOvcur_dev->hOdmUsbOvcur);
	UsbOvcur_dev->hOdmUsbOvcur=0;
}

NvBool open_def_odm_usbovcur(void)
{
	NvBool err;
	
	err = NvOdmUsbOvcurOpen(&(UsbOvcur_dev->hOdmUsbOvcur));
	if (!err){
		pr_err("open_def_odm_usbovcur: NvOdmUsbOvcurOpen failed\n");
		return err;
	}
	return err;
}


/**
 * Thread that waits for the interrupt from ODM and then sends out the
 * corresponding events.
 */
static NvS32 tegra_usbovcur_thread (void *pdata)
{	
	struct tegra_usbovcur_device_data *UsbOvcur_dev =
		(struct tegra_usbovcur_device_data*)pdata;


	UsbOvcur_dev->bThreadAlive = 1;
    while(UsbOvcur_dev->bThreadAlive){
		NvOdmUsbOvcurWaitInt(UsbOvcur_dev->hOdmUsbOvcur);
		        
    }

	return -1;
}


/**
 * All the device spefic initializations happen here. 
 */
static NvS32 __init tegra_usbovcur_probe(struct platform_device *pdev)
{
	struct tegra_usbovcur_device_data *usbovcur = NULL;
	NvS32 err;
	NvBool ret;
	
	usbovcur = kzalloc(sizeof(*usbovcur), GFP_KERNEL);
	if (usbovcur == NULL) {
		err = -ENOMEM;
		pr_err("tegra_ovcur_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	UsbOvcur_dev = usbovcur;

	
	ret = open_def_odm_usbovcur();
	if (!ret) {
		pr_err("open_def_odm_usbovcur failed\n");
		goto allocate_dev_fail;
	}
	
	//start the Int thread.
	usbovcur->task = kthread_create(tegra_usbovcur_thread, 
		usbovcur, "tegra_ovcur_thread");
	if (usbovcur->task == NULL) {
		err = -1;
		goto thread_create_failed;
	}
	
    wake_up_process(usbovcur->task); 
  
	

	platform_set_drvdata(pdev, usbovcur);

     
	printk(KERN_INFO DRIVER_NAME "successfully registered\n");
	return 0;



thread_create_failed:
	//KillThreadHere!
allocate_dev_fail:
	close_odm_usbovcur();
	kfree(usbovcur);
	usbovcur = 0;
	err = -ENOMEM;

	return err;
}






static struct platform_driver tegra_ovcur_driver = {
	.probe	 = tegra_usbovcur_probe,
	.remove	 = tegra_usbovcur_remove,
	.suspend = tegra_usbovcur_suspend,
	.resume	 = tegra_usbovcur_resume,	
	.driver	= {
		.name = "tegra_overcurrent",
	},
};

static NvS32 __devinit tegra_ovcur_init(void)
{
	return platform_driver_register(&tegra_ovcur_driver);
}

static void __exit tegra_ovcur_exit(void)
{
	platform_driver_unregister(&tegra_ovcur_driver);
}

module_init(tegra_ovcur_init);
module_exit(tegra_ovcur_exit);

MODULE_DESCRIPTION("Tegra ODM USB OVCUR driver");
