/*
 * drivers/input/misc/tegra_odm_capsensor.c
 *
 * Cap Sensor input device using NVIDIA Tegra ODM kit
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
#include <nvodm_capsensor.h>
#include <linux/earlysuspend.h>
#define DRIVER_NAME "nvodm_capsensor"
#define READ_BUFFER_LENGTH 40
#define DRIVER_CAP_BUTTON "Cap Sensor Button driver"
#define DRIVER_CAP_BUTTON_LICENSE "GPL"

MODULE_AUTHOR("Jn Hung <pegatraon.com>");
MODULE_DESCRIPTION(DRIVER_CAP_BUTTON);
MODULE_LICENSE(DRIVER_CAP_BUTTON_LICENSE);

static struct input_dev *cap_input_dev;
struct tegra_cap_device_data
{
	NvOdmAcrDeviceHandle	hOdmAcr;	
	struct task_struct	    *task;
	struct input_dev	    *input_dev;
	struct early_suspend    cap_early_suspend;
	NvU32			        sensitivity;
	NvU32			        chip_devinfo;
	NvBool			        bThreadAlive;
	NvBool			        show_log;	
	struct timeval		    tv;	
};

struct tegra_cap_device_data *cap_dev;
static const char* parameter[] = {
	"log=",
	"sensitivity=",
	"program=",	
	"devinfo=",	
	"dmi=",	
	"regctrl=",	
	"openclose=",
};

static enum {
	COMMAND_LOG = 0,
	COMMAND_SENSITIVITY, /* To adjust  or read sensitivity  for each button on Cap sensor board */
	COMMAND_PROGRAM,     /* To write Sensitivity value to DMI EEPROM  */	
	COMMAND_DEVINFO,     /* To read  Cap Sensor chip device information */
	COMMAND_READ_DMI,
	COMMAND_CTRL,
	COMMAND_OPENCLOSE,
}cap_enum;

static enum {
	CAP_KEY_SEARCH  = 0x08,
	CAP_KEY_BACK    = 0x10, 
	CAP_KEY_HOME    = 0x20, 
	CAP_KEY_MENU    = 0x80, 
	
}cap_sensor_key;


/** Function to close the ODM device. This function will help in switching
 * between power modes 
 */
void close_odm_cap(void)
{
	NvOdmCapClose(cap_dev->hOdmAcr);
	cap_dev->hOdmAcr = 0;
}

/** Function to open the ODM device with a set of default values. The values
 * are hardcoded as of now. Each time the device is closed/open, previous 
 * settings will be lost. This function will help in switching
 * between power modes
 */
NvBool open_def_odm_cap(void)
{
	NvBool err;
	
	err = NvOdmCapOpen(&(cap_dev->hOdmAcr));
	if (!err) {
		pr_err("open_def_odm_cap: NvOdmCapOpen failed\n");
		return err;
	}
	return err;
}

/**Function to parse the values sent through sysfs and sets them accordingly
 */
void change_nvodm_capsensor_settings(NvU32 command, NvS32 value)
{
   NvU32 sensitivity = 0;
   NvU32 cap_devinfo = 0;
   NvU8  DMI_Data = 0;
   NvU32 RegData = 0;
	switch (command) {
	case COMMAND_LOG:
		cap_dev->show_log = (value == 0 ? NV_FALSE : NV_TRUE);
		break;	
	case COMMAND_SENSITIVITY:		
		if( value > 0) {
		NvOdmCapAdjustSensitivity(cap_dev->hOdmAcr, value);
		cap_dev->sensitivity = value;		
		} else {
		 NvOdmCapReadSensitivity(cap_dev->hOdmAcr, &sensitivity);    
		 cap_dev->sensitivity = sensitivity;	
		 printk("\r\n Cap Sensitivity: Menu = 0x%x Home = 0x%x Back= 0x%x Search = 0x%x\n",((sensitivity>>24)&0xff),((sensitivity>>16)&0xff),((sensitivity>>8)&0xff),(sensitivity&0xff));   
		}    
		break;	
	case COMMAND_PROGRAM:	    
		printk("\r\n Programm Cap Sensor Sensitivity to DMI: %s\n",((NvOdmCapProgSensitivity(cap_dev->hOdmAcr, cap_dev->sensitivity))==NV_TRUE)? "Pass":"Fail");		
		break;	
    case COMMAND_DEVINFO:	    
		 NvOdmCapReadDevInfo(cap_dev->hOdmAcr, &cap_devinfo);    
		 cap_dev->chip_devinfo = cap_devinfo;	
		 printk("\r\n Cap Sensor Chip Info: ID = 0x%x FW_REV2_REG = 0x%x FW_REV1_REG = 0x%x FW_REV0_REG = 0x%x\n",((cap_devinfo>>24)&0xff),((cap_devinfo>>16)&0xff),((cap_devinfo>>8)&0xff),(cap_devinfo&0xff));   
		break;
    case COMMAND_READ_DMI:	 
		 NvOdmCapReadDMI(cap_dev->hOdmAcr, &DMI_Data, value); 		 
		 printk("\r\n DMI EEPROM Address = 0x%x Value = 0x%x\n",value,DMI_Data);   
		break;							
    case COMMAND_CTRL:         
        NvOdmCapCtlr(cap_dev->hOdmAcr, value, &RegData);
        if((value>>15)==1)
            printk("\r\n Write Reg:  Reg %d  Value = 0x%x\n",((value>>8)&0x7f),(value&0xff));              
        else
            printk("\r\n Read Reg:   Reg %d  Vlaue = 0x%x\n",((value>>8)&0x7f),RegData);              
		break;											
	case COMMAND_OPENCLOSE:
		if (value) {
			if (!cap_dev->hOdmAcr)
				open_def_odm_cap();
		} else {
			if (cap_dev->hOdmAcr)
				close_odm_cap();
		}
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
ssize_t read_sysfs_cap(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;	
	return len;
}

/** Function to take settings values to the caller modules/application
 * though the use of sysfs. This function can be used to set the 
 * device specific caps/attributes as specidied by the caller modules/app
 *  This function maps to store of sysfs
 */
ssize_t write_sysfs_cap(struct device *dev,
	struct device_attribute *attr, const char *buffer, size_t count)
{
	NvS32  i = 0;
	NvS8 *input;
	NvS32 value;

	count = count > READ_BUFFER_LENGTH ? READ_BUFFER_LENGTH : count;
	input = kzalloc(((int)count+1), GFP_KERNEL);
	if (!input) {
		goto err_0;
	}

	memcpy(input, buffer, count);

	input[count] = '\0';
	for (i=0; i<NV_ARRAY_SIZE(parameter); i++)  { 
		if (count > strlen(parameter[i]) &&
			!strncmp(parameter[i], input, strlen(parameter[i]))) {
			value = simple_strtol(&input[strlen(parameter[i])],
				'\0', 16);
			cap_enum = i;
			goto proceed;
		}
	}
	goto err_0;

proceed:
	if (cap_dev->hOdmAcr)
		change_nvodm_capsensor_settings(cap_enum, value);
	else
		count = 0;

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

DEVICE_ATTR(tegra_capsensor, 0777, read_sysfs_cap, write_sysfs_cap);
NvS32 add_sysfs_entry_capsensor(void)
{
	return device_create_file(&cap_dev->input_dev->dev,
		&dev_attr_tegra_capsensor);
}

/** 
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */
NvS32 remove_sysfs_entry_capsensor(void)
{
	device_remove_file(&cap_dev->input_dev->dev, 
		&dev_attr_tegra_capsensor);
	return 0;
}

/**
 * Thread that waits for the interrupt from ODM and then sends out the
 * corresponding events.
 */
static NvS32 tegra_cap_thread (void *pdata)
{
	int key_pressed = 0;
	struct tegra_cap_device_data *capsensor =
		(struct tegra_cap_device_data*)pdata;
	NvS32 KeyEvent = 0, ButtonChange=0, ButtonStatus = 0;
	
	capsensor->bThreadAlive = 1;
	while (capsensor->bThreadAlive) {
	 
	    NvOdmCapWaitInt(capsensor->hOdmAcr);	    		
		KeyEvent = ReadKeyEvent(capsensor->hOdmAcr);
		ButtonChange = ((KeyEvent>>8)&0xff);  /* Button Change */
		ButtonStatus = (KeyEvent&0xff);       /* Button Status */

#if 1
        if((ButtonChange==ButtonStatus)||(ButtonStatus==(CAP_KEY_SEARCH|CAP_KEY_MENU))){ 
		    key_pressed = 1;		    
		}else if((ButtonStatus==0)||(ButtonChange!=ButtonStatus)){
		    key_pressed = 0;
		}       

        if(ButtonChange==CAP_KEY_SEARCH){		  
	    
	        if((ButtonStatus==(CAP_KEY_SEARCH|CAP_KEY_MENU))&&(ButtonChange==CAP_KEY_SEARCH)){
	        ButtonChange = CAP_KEY_MENU;
	        }    
		    
//		    if((ButtonStatus==(CAP_KEY_SEARCH|CAP_KEY_MENU))&&(ButtonChange==CAP_KEY_MENU)){
//	        ButtonChange = CAP_KEY_SEARCH;
//	        }     

		    if((ButtonChange==(CAP_KEY_SEARCH|CAP_KEY_MENU))&&(ButtonStatus==CAP_KEY_MENU)){
	        ButtonChange = CAP_KEY_SEARCH;
	        }     

        }
        
        if(key_pressed == 0){
          if((ButtonChange==(CAP_KEY_SEARCH|CAP_KEY_MENU))&&(ButtonStatus==CAP_KEY_MENU)){
	        ButtonChange = CAP_KEY_SEARCH;
	        }     
        
          if((ButtonChange==(CAP_KEY_SEARCH|CAP_KEY_MENU))&&(ButtonStatus==CAP_KEY_SEARCH)){
	        ButtonChange = CAP_KEY_MENU;
	        }
        
        }
 #endif              
         	    
	    switch(ButtonChange)
	    {
	     case CAP_KEY_SEARCH:
	                input_report_key(capsensor->input_dev, KEY_SEARCH, key_pressed);     
	                break;
	     case CAP_KEY_BACK:
	                input_report_key(capsensor->input_dev, KEY_BACK, key_pressed);     
	                break;
	     case CAP_KEY_HOME:
	                input_report_key(capsensor->input_dev, KEY_HOME, key_pressed);     
	                break;
	     case CAP_KEY_MENU:
	                input_report_key(capsensor->input_dev, KEY_MENU, key_pressed);     
	                break;
	     default:           
	                if (capsensor->show_log) {
			            printk("Invalid Key Code: 0x%x\n", ButtonChange);
		            }
	     }    
	    
	    if (capsensor->show_log) {    
	        printk("\n\n>>>> key_pressed = %d KeyEvent = 0x%x  Button Change = 0x%x  Button Status =0x%x \n",key_pressed,KeyEvent, ButtonChange,ButtonStatus);
	    }
       
	    
	}
	
	return -1;
}
static int tegra_cap_early_suspend(struct platform_device *pdev, pm_message_t state)
{
     struct tegra_cap_device_data *capsensor = platform_get_drvdata(pdev);
     NvOdmCapSuspend(cap_dev->hOdmAcr);
     return 0;
}

static int tegra_cap_early_resume(struct platform_device *pdev)
{
     struct tegra_cap_device_data *capsensor = platform_get_drvdata(pdev);
     NvOdmResume(cap_dev->hOdmAcr);
	 return 0;
}

/**
 * All the device spefic initializations happen here. 
 */
static NvS32 __init tegra_cap_probe(struct platform_device *pdev)
{
	struct tegra_cap_device_data *capsensor = NULL;
	struct input_dev *input_dev = NULL;
	NvS32 err;
	NvBool ret;
	
	capsensor = kzalloc(sizeof(*capsensor), GFP_KERNEL);
	if (capsensor == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}
	cap_dev = capsensor;

	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_acc_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	cap_input_dev = input_dev;
	
	ret = open_def_odm_cap();
	if (!ret) {
		pr_err("open_def_odm_accl failed\n");
		goto allocate_dev_fail;
	}
	
	//start the Int thread.
	capsensor->task = kthread_create(tegra_cap_thread, 
		capsensor, "tegra_cap_thread");
	if (capsensor->task == NULL) {
		err = -1;
		goto thread_create_failed;
	}
	
    wake_up_process(capsensor->task); 
    
	capsensor->input_dev = input_dev;
	set_bit(EV_KEY, capsensor->input_dev->evbit);
	__set_bit(KEY_SEARCH, input_dev->keybit);
	__set_bit(KEY_HOME, input_dev->keybit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	

	platform_set_drvdata(pdev, capsensor);

	input_dev->name = "capsensor_tegra";
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_acc_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto input_register_device_failed;
	}

	err = add_sysfs_entry_capsensor();
	if (err) {
		pr_err("tegra_acc_probe: add_sysfs_entry failed\n");
		goto sysfs_failed;
	} 
     
        capsensor->cap_early_suspend.suspend = tegra_cap_early_suspend;
	capsensor->cap_early_suspend.resume  = tegra_cap_early_resume;
	register_early_suspend(&capsensor->cap_early_suspend);
	 
	printk(KERN_INFO DRIVER_NAME "successfully registered\n");
	return err;

sysfs_failed:
	input_unregister_device(input_dev);
input_register_device_failed:
	capsensor->bThreadAlive = 0;
thread_create_failed:
	//KillThreadHere!
allocate_dev_fail:
	close_odm_cap();
	input_free_device(input_dev);
	kfree(capsensor);
	capsensor = 0;
	err = -ENOMEM;

	return err;
}

static NvS32 tegra_cap_remove(struct platform_device *pdev)
{
	struct tegra_cap_device_data *capsensor = platform_get_drvdata(pdev);
	unregister_early_suspend(&capsensor->cap_early_suspend);
	capsensor->bThreadAlive = 0;
	remove_sysfs_entry_capsensor();
	input_unregister_device(cap_input_dev);
	return 0;
}

static struct platform_driver tegra_cap_driver = {
	.probe	 = tegra_cap_probe,
	.remove	 = tegra_cap_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND	
	.suspend = tegra_cap_suspend,
	.resume	 = tegra_cap_resume,	
#endif	
	.driver	= {
		.name = "tegra_capsensor",
	},
};

static NvS32 __devinit tegra_cap_init(void)
{
	return platform_driver_register(&tegra_cap_driver);
}

static void __exit tegra_cap_exit(void)
{
	platform_driver_unregister(&tegra_cap_driver);
}

module_init(tegra_cap_init);
module_exit(tegra_cap_exit);

MODULE_DESCRIPTION("Tegra ODM Cap Sensor driver");

