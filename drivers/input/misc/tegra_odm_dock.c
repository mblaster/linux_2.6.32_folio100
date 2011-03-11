/*
 * drivers/input/misc/tegra_odm_dock.c
 *
 * Dock input device using NVIDIA Tegra ODM kit
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
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/switch.h>
#include <mach/nvrm_linux.h>
#include <nvos.h>
#include <nvodm_query_gpio.h>
#include <nvodm_services.h>
#include <nvodm_dock.h>
#include <linux/earlysuspend.h>
#define DRIVER_DOCK "Dock driver"
#define DRIVER_DOCK_LICENSE "GPL"


MODULE_AUTHOR("Jn Hung <pegatraon.com>");
MODULE_DESCRIPTION(DRIVER_DOCK);
MODULE_LICENSE(DRIVER_DOCK_LICENSE);


struct tegra_dock_dev {

	struct task_struct	    *task;
	struct switch_dev       sdev;
	NvOdmDockHandle	        odm_dev;
	struct early_suspend dock_early_suspend;    
    NvBool  bThreadAlive;
};

enum {
	NO_DOCK,
	DESK_DOCK,
	
};
struct tegra_dock_dev *docking_dev;
//static unsigned int state = 0 , last_state = 0;
static ssize_t print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case NO_DOCK:
		return sprintf(buf, "Undock\n");
	case DESK_DOCK:
		return sprintf(buf, "Dock\n");
	
	}

	return -EINVAL;
}


static int dock_thread(void *pdata)
{
	struct tegra_dock_dev *dock = (struct tegra_dock_dev *)pdata;
   
    dock->bThreadAlive = 1;
	/* FIXME add a terminating condition */ 
	
	while(dock->bThreadAlive) 
	{		   
	    NvOdmDockWaitInt(dock->odm_dev);
	    
	    if (NvOdmGetDockState(dock->odm_dev))	
		    switch_set_state(&dock->sdev,  DESK_DOCK);
	     else
		    switch_set_state(&dock->sdev, NO_DOCK);	       	        
	}
	
	return 0;
}

static int tegra_dock_suspend(struct platform_device *pdev, pm_message_t state)
{
 
    NvOdmDockInterruptMask(docking_dev);  
    return 0;
}

static int tegra_dock_resume(struct platform_device *pdev)
{

    NvOdmDockInterruptUnMask(docking_dev);   
        
    NvOdmGetDockInitialState(docking_dev);
    
    NvOdmDockSignalInt(docking_dev);

	return 0;
}


static int __init tegra_dock_probe(struct platform_device *pdev)
{
	struct tegra_dock_dev *dock = NULL;
	NvOdmDockHandle odm_dev = NULL;
	int err,ret=0;
	NvBool error;	
	
		
	error = NvOdmDockOpen(&odm_dev);
	
	if (!error) {
		pr_err("tegra_dock_probe: dock not found4\n");
		err = -ENXIO;
		goto err_DockNotFound;
	}
  
	dock = kzalloc(sizeof(struct tegra_dock_dev), GFP_KERNEL);
	dock->odm_dev = odm_dev;
	docking_dev = odm_dev;
	dock->task = kthread_create(dock_thread, dock, "tegra_dock");
	if (dock->task == NULL) {
		err = -1;
		goto err_kthread_create_failed;
	}
	
	dock->sdev.name = "dock";
	dock->sdev.print_name = print_name;
	ret = switch_dev_register(&dock->sdev);
	if (ret) {
		pr_err("%s: error registering switch device %d\n",
			__func__, ret);
		goto error_switch_device_failed;
	}
	
	platform_set_drvdata(pdev, dock);	
	wake_up_process(dock->task );
	
	if(NvOdmGetDockInitialState(dock->odm_dev))
	  switch_set_state(&dock->sdev,  DESK_DOCK);
	 else
      switch_set_state(&dock->sdev, NO_DOCK);	  
		        
	dock->dock_early_suspend.suspend = tegra_dock_suspend;
	dock->dock_early_suspend.resume = tegra_dock_resume;
	register_early_suspend(&dock->dock_early_suspend);
	
	return 0;
error_switch_device_failed:	
    switch_dev_unregister(&dock->sdev);
    dock->bThreadAlive = 0;
err_kthread_create_failed:
	/* What to do? */
err_alloc_failed:
	kfree(dock);	
	NvOdmDockClose(odm_dev);
err_DockNotFound:	
	return err;
}


static int tegra_dock_remove(struct platform_device *pdev)
{
	struct tegra_dock_dev *dock = platform_get_drvdata(pdev);
    unregister_early_suspend(&dock->dock_early_suspend);
    dock->bThreadAlive = 0;
	NvOdmDockClose(dock->odm_dev);
	switch_dev_unregister(&dock->sdev);
	kfree(dock);
	return 0;
}

static struct platform_driver tegra_dock_driver = {
	.probe	= tegra_dock_probe,
	.remove	= tegra_dock_remove,
	.driver	= {
		.name = "tegra_dock",
		
	},
};

static int __devinit tegra_dock_init(void)
{
		
	return platform_driver_register(&tegra_dock_driver);
}

static void __exit tegra_dock_exit(void)
{
	platform_driver_unregister(&tegra_dock_driver);
}

module_init(tegra_dock_init);
module_exit(tegra_dock_exit);
MODULE_DESCRIPTION("Tegra Betelgeuse Dock driver");
