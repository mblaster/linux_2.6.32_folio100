/*
 * drivers/input/keyboard/tegra-nvec-cir.c
 *
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
#include <linux/delay.h> /*for msleep*/
#include <linux/input.h>
#include <linux/device.h>
#include <linux/kthread.h>
//#include <linux/tegra_devices.h>
#include <linux/freezer.h>

#include "nvos.h"
#include "nvec.h"
#include "nvodm_services.h"
#include "nvodm_cir.h"
#include "nvec_device.h"

#define DRIVER_CIR "NvEc CIR driver"
#define DRIVER_CIR_LICENSE "GPL"


MODULE_AUTHOR("JolenPeng <pegatraon.com>");
MODULE_DESCRIPTION(DRIVER_CIR);
MODULE_LICENSE(DRIVER_CIR_LICENSE);


#define EC_CIR_NEC_CODE_FIRST 0x3F
#define EC_CIR_NEC_CODE_LAST 0xFF
#define EC_CIR_NEC_TOTAL_CODES (EC_CIR_NEC_CODE_LAST - EC_CIR_NEC_CODE_FIRST + 1)
/**
 * @brief This table consists of the scan codes which were added after the
 *		original scan codes were designed.
 *        To avoid moving the already designed buttons to accomodate these
 *        new buttons, the new scan codes are preceded by 'E0'.
 */
NvU16 cir_local_nec_tbl[EC_CIR_NEC_TOTAL_CODES] = {
	0,
	KEY_POWER,		/*v Power, 0xE319 0x40 */
	0,
	0,
	0,
	KEY_VOLUMEUP,	/*v VOL + , 0x44*/	
	KEY_UP,			/*v UP ,0x45*/
	KEY_VOLUMEDOWN,	/*v Vol - , 0x46*/
	KEY_LEFT,		/*v Left, 0x47*/
	KEY_ENTER,		/*V OK, 0x48*/
	KEY_RIGHT,		/*v Right, 0x49*/		
	0,		
	KEY_DOWN, 		/*v Down 0x4B*/
	0,		
	0,		
	0,
	KEY_STOP,				/*v(128) stop 0x4F*/
	KEY_PREVIOUSSONG,		/*v (165) Previous ,0xE319 0x50 */
	0,
	0,
	KEY_NEXTSONG,			/*v (163)Next 0xE319 0x53*/
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	0,		/* 0xE319 0x60 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	KEY_BACK, /*v (158) Back 0x6E*/
	0,
	0,		/* 0xE319 0x70 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	KEY_PLAYPAUSE,		 /*v (164) paly/pause 0x7c*/
	0,		
	0,
	0,
	0,		/* 0xE319 0x80 */
	0,
	0,
	0,
	KEY_MENU,	/*v (139)Menu 0x84*/	
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	KEY_HOME,	/*v 0x8C HOME*/	
	0,		
	0,
	0,
	0,		/* 0xE319 0x90 */
	KEY_SCREENLOCK, /*(152) CAPS ??Screen , 0x91*/
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	0,		/* 0xE319 0xa0 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	0,		/* 0xE319 0xb0 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	0,		/* 0xE319 0xc0 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	0,		/* 0xE319 0xd0 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	0,
	0,
	KEY_MEDIA,		/*TMP, 0xE319 0xe0 */
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,
	0,
	0,		
	0,		
	0,
	0,		
	0,		
	KEY_PROG1, 		/*A, (148) 0xee*/
	KEY_PROG2,		/*B ,(149) 0xef*/
	KEY_PROG3,		/*C, (202) 0xE319 0xf0 */
	0,
	0,
	0,
	0,		
	0,		
	KEY_BLUE, 	/*blue,(0x191) 0xF6*/
	KEY_RED, 	/*Red,(0x18e) 0xF7*/
	KEY_GREEN,	/*GREEN,(0x18f) 0xF8*/
	KEY_YELLOW, /*YELLOW(0x190) 0xF9*/		
	0,		
	0,
	0,		
	0,		
	0,
	0,/* 0xE319 0xFF */
};

struct nvec_cir
{
	struct input_dev		*input_dev;
	struct task_struct		*task;
	char					name[128];
	int						shutdown;
	NvBool			         	show_log;
	struct timer_list			rep_delay;/*add repeat stop timer count*/
	NvEcHandle				hNvec;
	NvEcEventRegistrationHandle	hEvent;
};

struct nvec_cir *cir;

static const char* parameter[] = {
	"log=",
};

static enum {
	COMMAND_LOG_EN = 0, /*en/disable log info*/
}cir_enum;

/**Function to parse the values sent through sysfs and sets them accordingly
 */
void change_cir_settings(NvU32 command, NvS32 value)
{

	switch (command) {
	case COMMAND_LOG_EN:
		cir->show_log = (value == 0 ? NV_FALSE : NV_TRUE);
		printk(KERN_INFO "CIR_show log=%d\n",value);
		break;
    
	default:
		break;
	}
	
}


/** Function to give out settings values to the caller modules/application
 * though the use of sysfs. This function is stub as of now, but can be used
 * to return the device specific caps/attributes to the caller modules/app
 *  This function maps to show of sysfs
 */
ssize_t read_sysfs_cir(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;	
	return len;
}

/** Function to take settings values to the caller modules/application
 * though the use of sysfs. This function can be used to set the 
 * device specific caps/attributes as specidied by the caller modules/app
 *  This function maps to store of sysfs
 */
#define READ_CIR_BUFFER_LENGTH 40 
 
 
 
ssize_t write_sysfs_cir(struct device *dev,	struct device_attribute *attr, const char *buffer, size_t count)
{
	NvS32  i = 0;
	NvS8 *input;
	NvS32 value;

	count = count > READ_CIR_BUFFER_LENGTH ? READ_CIR_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		goto err_0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	for (i=0; i<NV_ARRAY_SIZE(parameter); i++)  { 
		if (count > strlen(parameter[i]) &&	!strncmp(parameter[i], input, strlen(parameter[i]))) {
			value = simple_strtol(&input[strlen(parameter[i])],'\0', 16);
			cir_enum = i;
			goto proceed;
		}
	}
	goto err_0;

proceed:
		change_cir_settings(cir_enum, value);


	kfree(input);
	return count;
	
err_0:
	if(input)
		kfree(input);
	count = 0;
	return count;

}


/** 
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */

DEVICE_ATTR(nvec_cir, 0777, read_sysfs_cir, write_sysfs_cir);

NvS32 add_sysfs_entry(void)
{
	return device_create_file(&cir->input_dev->dev,&dev_attr_nvec_cir);
}

/** 
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */
NvS32 remove_sysfs_entry(void)
{
	device_remove_file(&cir->input_dev->dev, &dev_attr_nvec_cir);
	return 0;
}


/*get cir event thread fuction.*/
static int nvec_cir_recv(void *arg)
{
	struct input_dev *input_dev = (struct input_dev *)arg;
	struct nvec_cir *cir = input_get_drvdata(input_dev);

	/* cir event thread should be frozen before suspending the
	 * cir and NVEC drivers */
	set_freezable_with_signal();

	while (!cir->shutdown) {
		unsigned int repeat_pressed;
		NvU32 code; /*4byte payload */
		NvU8 repeat_key;
		static NvU8 repeat_count=0;

		if (!NvOdmCirGetKeyData(cir->show_log,&code, &repeat_key, 0)) {
			printk(KERN_INFO "nvec_cir: unhandled  , scancode %x\n", code);
			continue;
		}
		
		if (cir->shutdown)
			break;

	
		repeat_pressed = (repeat_key & NV_ODM_CIR_SCAN_CODE_REPET);
		if(repeat_pressed)
			repeat_count++;
		else
			repeat_count=0;

		
		
		//printk(KERN_INFO "nvec_cir_recv scancode %x,repeat_pressed=%x\n", code,repeat_pressed);
		if(cir->show_log){
			printk(KERN_INFO "repeat_count=%d",repeat_count);
			printk(KERN_INFO "keymap:: 0x%x (%d),repeat_pressed=%x\n", code, code,repeat_pressed);
			}

		if ((code >= EC_CIR_NEC_CODE_FIRST) &&	(code <= EC_CIR_NEC_CODE_LAST)) {
			code -= EC_CIR_NEC_CODE_FIRST;
			code = cir_local_nec_tbl[code];

			cir->input_dev->repeat_key = code;/*copy key code is because of send repeat stop key code! in cir_rep_stop_delay timer*/


			switch(code){			
			case KEY_SCREENLOCK :    	/*SCREEN lock button */
				if(!repeat_count){
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_REPET);
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_STOP);	
				}
					break;
			case KEY_POWER:    /*power button*/
				if(!repeat_count){
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_REPET);
					mod_timer(&cir->rep_delay, jiffies + (60*HZ/100));/*600ms*/
				}
					break;
			case KEY_HOME: 
			case KEY_PROG1: 
			case KEY_PROG2: 
			case KEY_PROG3:
			case KEY_BLUE:
			case KEY_RED:
			case KEY_GREEN:
			case KEY_YELLOW:	
			case KEY_ENTER:
			case KEY_MEDIA:   					/*Menu button, support long press*/
				if(repeat_count==0){
		    		input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_REPET);
					mod_timer(&cir->rep_delay, jiffies + (15*HZ/100));/*150ms*/
				}else{
					mod_timer(&cir->rep_delay, jiffies + (15*HZ/100));/*150ms*/
				}	
				break;
			default:
			
				if( repeat_count==0 || repeat_count==5 ||repeat_count==9 ||repeat_count==12){
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_REPET);
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_STOP);	
				}else if(repeat_count >= 14 && (repeat_count%2) == 0 ){
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_REPET);
					input_report_key(cir->input_dev, code, NV_ODM_CIR_SCAN_CODE_STOP);	
				}else{}
				break;
			}
			
		}
	}
	return 0;
}


/*kernel dynamic timer , time-out function!  send and stop key event*/
static void cir_rep_stop_delay(unsigned long arg)
{
	struct nvec_cir *cir = (struct nvec_cir*)arg;

	if(cir->show_log){
	printk(KERN_INFO "cir_rep_stop_delay \n");
	}
	
	input_report_key(cir->input_dev, cir->input_dev->repeat_key, NV_ODM_CIR_SCAN_CODE_STOP);		
}


static int nvec_cir_open(struct input_dev *dev)
{
	return 0;
}

static void nvec_cir_close(struct input_dev *dev)
{
	return;
}

static int __devinit nvec_cir_probe(struct nvec_device *pdev)
{
	int error;
	NvError nverr;
	//struct nvec_cir *cir;
	struct input_dev *input_dev;
	int i;
	
	NvOsDebugPrintf("### NVEC_CIR_PROBE ###\n");
	cir = kzalloc(sizeof(struct nvec_cir), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!cir || !input_dev) {
		error = -ENOMEM;
		goto fail;
	}

	init_timer(&cir->rep_delay);/*initial timer*/
	cir->rep_delay.expires = jiffies ; 
	cir->rep_delay.function = cir_rep_stop_delay;
	cir->rep_delay.data = (unsigned long)cir;
	
	cir->input_dev = input_dev;
	input_set_drvdata(input_dev, cir);
	nvec_set_drvdata(pdev, input_dev);

	if (!NvOdmCirInit()) {
		error = -ENODEV;
		pr_err("tegra_cir_probe: no cir\n");
		goto fail_cir_init;
	}

	cir->task = kthread_create(nvec_cir_recv, input_dev,"nvec_cir_thread"); 
	
	if (cir->task == NULL) {
		error = -ENOMEM;
		goto fail_thread_create;
	}
	wake_up_process(cir->task);

	if (!strlen(cir->name))
		snprintf(cir->name, sizeof(cir->name), "nvec cir");

	input_dev->name = cir->name;
	input_dev->open = nvec_cir_open;
	input_dev->close = nvec_cir_close;

	__set_bit(EV_KEY, input_dev->evbit); /*event type EV_KEY*/

	for (i=1; i<EC_CIR_NEC_TOTAL_CODES; i++) {
		__set_bit(cir_local_nec_tbl[i], input_dev->keybit);
	}

	/* get EC handle */
	nverr = NvEcOpen(&cir->hNvec, 0 /* instance */);
	if (nverr != NvError_Success) {
		error = -ENODEV;
		goto fail_input_register;
	}

	error = input_register_device(cir->input_dev);
	if (error)
		goto fail_input_register;
		
	error = add_sysfs_entry();
	if (error) {
		pr_err("nvec_cir_probe: add_sysfs_entry failed\n");
		goto sysfs_failed;
	}	

	return 0;
	
sysfs_failed:
	input_unregister_device(input_dev);
fail_input_register:
	(void)kthread_stop(cir->task);
fail_thread_create:
	NvOdmCirDeInit();
fail_cir_init:
fail:
	NvEcClose(cir->hNvec);
	cir->hNvec = NULL;
	input_free_device(input_dev);
	kfree(cir);
	printk(KERN_INFO "cir probe fail:");
	return error;	

}

static void nvec_cir_remove(struct nvec_device *dev)
{
	struct input_dev *input_dev = nvec_get_drvdata(dev);
	struct nvec_cir *cir = input_get_drvdata(input_dev);
  NvOsDebugPrintf("$$$ NVEC_CIR_REMOVE $$$\n");	
	
	remove_sysfs_entry();
	input_unregister_device(input_dev);
	(void)kthread_stop(cir->task);
	NvOdmCirDeInit();
	NvEcClose(cir->hNvec);
	cir->hNvec = NULL;
	cir->shutdown = 1;
	input_free_device(input_dev);
	kfree(cir);

}

static int nvec_cir_suspend(struct nvec_device *pdev, pm_message_t state)
{
  NvOsDebugPrintf("$$$ NVEC_CIR_SUSPEND $$$\n");
#if 0
#if CIR_SCANNING_DISABLED_IN_SUSPEND
	NvEcRequest Request = {0};
	NvEcResponse Response = {0};
	NvError err = NvError_Success;
#endif

#endif 
	return 0;
}

static int nvec_cir_resume(struct nvec_device *pdev)
{
  NvOsDebugPrintf("$$$NVEC_CIR_RESUME $$$\n");
#if 0

#if CIR_SCANNING_DISABLED_IN_SUSPEND
	NvEcRequest Request = {0};
	NvEcResponse Response = {0};
	NvError err = NvError_Success;
#endif


#endif
	return 0;
}

static struct nvec_driver nvec_cir_driver = {
	.name		= "nvec_cir",
	.probe		= nvec_cir_probe,
	.remove		= nvec_cir_remove,
	.suspend	= nvec_cir_suspend,
	.resume		= nvec_cir_resume,
};

static struct nvec_device nvec_cir_device = {
	.name	= "nvec_cir",
	.driver	= &nvec_cir_driver,
};

static int __init nvec_cir_init(void)
{
  
	int err;

	NvOsDebugPrintf("### NVEC_CIR_INIT ###\n");
	
	err = nvec_register_driver(&nvec_cir_driver);
	if (err)
	{
		pr_err("**nvec_cir_init: nvec_register_driver: fail\n");
		return err;
	}

	err = nvec_register_device(&nvec_cir_device);
	if (err)
	{
		pr_err("**nvec_cir_init: nvec_device_add: fail\n");
		nvec_unregister_driver(&nvec_cir_driver);
		return err;
	}

	return 0;
}

static void __exit nvec_cir_exit(void)
{
	NvOsDebugPrintf("$$$ NVEC_CIR_EXIT $$$\n");
	del_timer_sync(&cir->rep_delay);
	nvec_unregister_device(&nvec_cir_device);
	nvec_unregister_driver(&nvec_cir_driver);
}

module_init(nvec_cir_init);
module_exit(nvec_cir_exit);

