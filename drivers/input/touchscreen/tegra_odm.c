/*
 * drivers/input/touchscreen/tegra_odm.c
 *
 * Touchscreen class input driver for platforms using NVIDIA's Tegra ODM kit
 * driver interface
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

#define NV_DEBUG 0
//#define DEBUG 1

#include <linux/module.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/freezer.h>

#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/version.h>

#include <nvodm_services.h>
#include <nvodm_touch.h>

#define TOOL_PRESSURE	-1

#define TOOL_WIDTH_MAX		8

/* Ratio TOOL_WIDTH_FINGER / TOOL_WIDTH_MAX must be < 0.6f to be considered
 * "finger" press.
 */
#define TOOL_WIDTH_FINGER	0

/* Ratio TOOL_WIDTH_CHEEK / TOOL_WIDTH_MAX must be > 0.6f to be considered
 * "cheek" press.  (Cheek presses are ignored in phone mode, under assumption
 * that user accidentally pressed cheek against touch screen.)
 */

/* the kernel supports 5 fingers only as of now */
#define MAX_FINGERS	5
#define SUPPORTED_FINGERS 4

struct tegra_touch_driver_data {
    struct input_dev    *input_dev;
    struct task_struct  *task;
    NvOdmOsSemaphoreHandle  semaphore;
    NvOdmTouchDeviceHandle  hTouchDevice;
    NvBool          bPollingMode;
    NvU32           pollingIntervalMS;
    NvOdmTouchCapabilities  caps;
    NvU32           MaxX;
    NvU32           MinX;
    NvU32           MaxY;
    NvU32           MinY;
    int         shutdown;
    struct early_suspend    early_suspend;
};

#ifdef DEBUG
    #define TS_DEBUG(fmt,args...)  printk( KERN_DEBUG "[egalax_i2c]: " fmt, ## args)
    #define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__)
#else
    #define TS_DEBUG(fmt,args...)
    #define DBG()
#endif

#define MAX_I2C_LEN		10
#define FIFO_SIZE		PAGE_SIZE

static int global_major = 0; // dynamic major by default 
static int global_minor = 0;

struct egalax_char_dev {
    int OpenCnts;
    struct cdev cdev;
    struct tegra_touch_driver_data *touch;
    struct kfifo *pDataFiFo;
    unsigned char *pFiFoBuf;
    spinlock_t FiFoLock;
    struct semaphore sem;
    wait_queue_head_t fifo_inq;
};

static struct egalax_char_dev *p_char_dev = NULL;   // allocated in init_module
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static struct class *egalax_class;

static int egalax_cdev_open(struct inode *inode, struct file *filp) {
    struct egalax_char_dev *cdev;

    DBG();

    cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
    if ( cdev == NULL ) {
        TS_DEBUG(" No such char device node \n");
        return -ENODEV;
    }

    if ( !atomic_dec_and_test(&egalax_char_available) ) {
        atomic_inc(&egalax_char_available);
        return -EBUSY; /* already open */
    }

    cdev->OpenCnts++;
    filp->private_data = cdev;// Used by the read and write metheds

    TS_DEBUG(" egalax_cdev_open done \n");
    try_module_get(THIS_MODULE);
    return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp) {
    struct egalax_char_dev *cdev; // device information

    DBG();

    cdev = container_of(inode->i_cdev, struct egalax_char_dev, cdev);
    if ( cdev == NULL ) {
        TS_DEBUG(" No such char device node \n");
        return -ENODEV;
    }

    atomic_inc(&egalax_char_available); /* release the device */

    filp->private_data = NULL;
    cdev->OpenCnts--;
    kfifo_reset( cdev->pDataFiFo );

    TS_DEBUG(" egalax_cdev_release done \n");
    module_put(THIS_MODULE);
    return 0;
}

#define MAX_READ_BUF_LEN	50
static char fifo_read_buf[MAX_READ_BUF_LEN];
static ssize_t egalax_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset) {
    int read_cnt, ret;
    struct egalax_char_dev *cdev = file->private_data;

    DBG();

    if ( down_interruptible(&cdev->sem) )
        return -ERESTARTSYS;

    while ( kfifo_len(cdev->pDataFiFo)<1 ) { /* nothing to read */
        up(&cdev->sem); /* release the lock */
        if ( file->f_flags & O_NONBLOCK )
            return -EAGAIN;

        if ( wait_event_interruptible(cdev->fifo_inq, kfifo_len( cdev->pDataFiFo )>0) )
            return -ERESTARTSYS; /* signal: tell the fs layer to handle it */

        if ( down_interruptible(&cdev->sem) )
            return -ERESTARTSYS;
    }

    if (count > MAX_READ_BUF_LEN)
        count = MAX_READ_BUF_LEN;

    read_cnt = kfifo_get(cdev->pDataFiFo, fifo_read_buf, count);
    //TS_DEBUG("\"%s\" reading: real fifo get function with fifo len=%d\n", current->comm, kfifo_len(cdev->pDataFiFo));

    ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

    up(&cdev->sem);

    return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset) {
    struct egalax_char_dev *cdev = file->private_data;
    struct tegra_touch_driver_data *touch = NULL;
    NvOdmTouchRawI2cData *i2c_data = NULL;

    int ret=0;

    DBG();

    if ( down_interruptible(&cdev->sem) )
        return -ERESTARTSYS;

    if (count > MAX_I2C_LEN)
        count = MAX_I2C_LEN;

    touch = cdev->touch;

    i2c_data = (NvOdmTouchRawI2cData *)kmalloc(sizeof(NvOdmTouchRawI2cData),GFP_KERNEL);

    if (i2c_data == NULL) {
        up(&cdev->sem);
        return -ENOMEM;
    }

    i2c_data->data_len = count;
    TS_DEBUG("I2C %zu bytes.\n", i2c_data->data_len);

    if (copy_from_user((char *)i2c_data->datum, buf, count)) {
        up(&cdev->sem);
        kfree(i2c_data);
        return -EFAULT;
    }

    {
        int i;
        for (i = 0; i < 10; i++) TS_DEBUG("0x%02x ", i2c_data->datum[i]);
        TS_DEBUG("\n");
    }


    TS_DEBUG("I2C writing %zu bytes.\n", count);

    ret = (int)NvOdmTouchRawI2cWrite(touch->hTouchDevice, i2c_data);

    kfree(i2c_data);

    up(&cdev->sem);

    return ret;
}

static int egalax_cdev_ioctl(struct inode *inode, struct file * file, unsigned int cmd, unsigned long arg) {
    //struct egalax_char_dev *cdev = file->private_data;
    int rval = -EINVAL;

    switch (cmd) {
    default:
        break;
    }

    return rval;
}

static unsigned int egalax_cdev_poll(struct file *filp, struct poll_table_struct *wait) {
    struct egalax_char_dev *cdev = filp->private_data;
    unsigned int mask = 0;

    down(&cdev->sem);
    poll_wait(filp, &cdev->fifo_inq,  wait);

    if ( kfifo_len(cdev->pDataFiFo) > 0 )
        mask |= POLLIN | POLLRDNORM;    /* readable */
    if ( (FIFO_SIZE - kfifo_len(cdev->pDataFiFo)) > MAX_I2C_LEN )
        mask |= POLLOUT | POLLWRNORM;   /* writable */

    up(&cdev->sem);
    return mask;
}

static const struct file_operations egalax_cdev_fops = {
    .owner  = THIS_MODULE,
    .read   = egalax_cdev_read,
    .write  = egalax_cdev_write,
    .ioctl  = egalax_cdev_ioctl,
    .poll   = egalax_cdev_poll,
    .open   = egalax_cdev_open,
    .release= egalax_cdev_release,
};

static struct egalax_char_dev* setup_chardev(dev_t dev) {
    struct egalax_char_dev *pCharDev;
    int result;

    pCharDev = kmalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
    if (!pCharDev)
        goto fail_cdev;
    memset(pCharDev, 0, sizeof(struct egalax_char_dev));

    spin_lock_init( &pCharDev->FiFoLock );
    pCharDev->pFiFoBuf = kmalloc(sizeof(unsigned char)*FIFO_SIZE, GFP_KERNEL);
    if (!pCharDev->pFiFoBuf)
        goto fail_fifobuf;
    memset(pCharDev->pFiFoBuf, 0, sizeof(unsigned char)*FIFO_SIZE);

    pCharDev->pDataFiFo = kfifo_init(pCharDev->pFiFoBuf, FIFO_SIZE, GFP_KERNEL, &pCharDev->FiFoLock);
    if ( pCharDev->pDataFiFo==NULL )
        goto fail_kfifo;

    pCharDev->OpenCnts = 0;
    cdev_init(&pCharDev->cdev, &egalax_cdev_fops);
    pCharDev->cdev.owner = THIS_MODULE;
    sema_init(&pCharDev->sem, 1);
    init_waitqueue_head(&pCharDev->fifo_inq);

    result = cdev_add(&pCharDev->cdev, dev, 1);
    if (result) {
        TS_DEBUG(KERN_ERR "Error cdev ioctldev added\n");
        goto fail_kfifo;
    }

    return pCharDev; 

    fail_kfifo:
    kfree(pCharDev->pFiFoBuf);
    fail_fifobuf:
    kfree(pCharDev);
    fail_cdev:
    return NULL;
}

static void egalax_chrdev_exit(void) {
    dev_t devno = MKDEV(global_major, global_minor);

    if (p_char_dev) {
        // Get rid of our char dev entries
        if ( p_char_dev->pFiFoBuf )
            kfree(p_char_dev->pFiFoBuf);

        cdev_del(&p_char_dev->cdev);
        kfree(p_char_dev);
    }

    unregister_chrdev_region( devno, 1);

    if (!IS_ERR(egalax_class)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
        class_device_destroy(egalax_class, devno);
#else
        device_destroy(egalax_class, devno);
#endif 
        class_destroy(egalax_class);
    }
}

static int egalax_chrdev_init(struct platform_device *pdev) {
    struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);
    int result = 0;
    dev_t devno = 0;

    DBG();

    // Asking for a dynamic major unless directed otherwise at load time.
    if (global_major) {
        devno = MKDEV(global_major, global_minor);
        result = register_chrdev_region(devno, 1, "egalax_i2c");
    } else {
        result = alloc_chrdev_region(&devno, global_minor, 1, "egalax_i2c");
        global_major = MAJOR(devno);
    }

    if (result < 0) {
        TS_DEBUG(" egalax_i2c cdev can't get major number\n");
        return 0;
    }

    // allocate the character device
    p_char_dev = setup_chardev(devno);

    if (!p_char_dev) {
        result = -ENOMEM;
        goto fail;
    }
    p_char_dev->touch = touch;

    egalax_class = class_create(THIS_MODULE, "egalax_i2c");
    if (IS_ERR(egalax_class)) {
        TS_DEBUG("Err: failed in creating class.\n");
        result = -EFAULT;
        goto fail;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
    class_device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#else
    device_create(egalax_class, NULL, devno, NULL, "egalax_i2c");
#endif
    TS_DEBUG("register egalax_i2c cdev, major: %d minor: %d\n", MAJOR(devno), MINOR(devno));
    return result;
    fail:   
    egalax_chrdev_exit();
    return result;
}

#define NVODM_TOUCH_NAME "nvodm_touch"

#define swapv(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tegra_touch_early_suspend(struct early_suspend *es) {
    struct tegra_touch_driver_data *touch;
    touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

    if (touch && touch->hTouchDevice) {
        NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_FALSE);
    } else {
        pr_err("tegra_touch_early_suspend: NULL handles passed\n");
    }
}

static void tegra_touch_late_resume(struct early_suspend *es) {
    struct tegra_touch_driver_data *touch;
    touch = container_of(es, struct tegra_touch_driver_data, early_suspend);

    if (touch && touch->hTouchDevice) {
        NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_TRUE);
    } else {
        pr_err("tegra_touch_late_resume: NULL handles passed\n");
    }
}
#else
static int tegra_touch_suspend(struct platform_device *pdev, pm_message_t state) {
    struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

    if (touch && touch->hTouchDevice) {
        NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_FALSE);
        return 0;
    }
    pr_err("tegra_touch_suspend: NULL handles passed\n");
    return -1;
}

static int tegra_touch_resume(struct platform_device *pdev) {
    struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

    if (touch && touch->hTouchDevice) {
        NvOdmTouchPowerOnOff(touch->hTouchDevice, NV_TRUE);
        return 0;
    }
    pr_err("tegra_touch_resume: NULL handles passed\n");
    return -1;
}
#endif

static int tegra_touch_thread(void *pdata) {
    struct tegra_touch_driver_data *touch = (struct tegra_touch_driver_data*)pdata;
    NvOdmTouchCoordinateInfo c = {0};
    NvU32 x[MAX_FINGERS] = {0}, y[MAX_FINGERS] = {0}, i = 0;
    NvS32 Pressure[MAX_FINGERS] = {TOOL_PRESSURE};
    NvU32 Width[MAX_FINGERS] = {TOOL_WIDTH_FINGER};
    static NvU32 prev_x0 = 0, prev_y0 = 0;
    NvBool bKeepReadingSamples = NV_FALSE;
    NvU32 fingers = 0, index, offset;
    NvOdmTouchCapabilities *caps = &touch->caps;
    NvOdmTouchRawI2cData i2c_data;  
    int count = 0;

    /* touch event thread should be frozen before suspend */
    set_freezable_with_signal();

    for (;;) {
        if (touch->bPollingMode)
            msleep(touch->pollingIntervalMS);
        else
            NvOdmOsSemaphoreWait(touch->semaphore);

        i2c_data.data_len = 0;
        c.pextrainfo = (void *)&i2c_data;
        bKeepReadingSamples = NV_TRUE;
        while (bKeepReadingSamples) {
            if (!NvOdmTouchReadCoordinate(touch->hTouchDevice, &c)) {
                pr_err("Couldn't read touch sample\n");
                bKeepReadingSamples = NV_FALSE;
                continue;
            }
            if (i2c_data.data_len != 0) {
                /* push data into kfifo */
                count = i2c_data.data_len;
                if (count > 0 && p_char_dev->OpenCnts>0 ) {
                    kfifo_put(p_char_dev->pDataFiFo, (u8 *)i2c_data.datum, count);
                    wake_up_interruptible( &p_char_dev->fifo_inq );
                }
                bKeepReadingSamples = NV_FALSE;
                if (!touch->bPollingMode &&
                    !NvOdmTouchHandleInterrupt(touch->hTouchDevice)) {
                    /* Some more data to read keep going */
                    bKeepReadingSamples = NV_TRUE;
                }

                continue;
            }

            for (i = 0; i < SUPPORTED_FINGERS; i++) {
                x[i] = c.additionalInfo.multi_XYCoords[i][0];
                y[i] = c.additionalInfo.multi_XYCoords[i][1];
                Pressure[i] = c.additionalInfo.Pressure[i];
                Width[i] = TOOL_WIDTH_FINGER;

                if (c.additionalInfo.Pressure[i] == 0) {
                    c.additionalInfo.Pressure[i] = -1;
                }
            }

            /* transformation from touch to screen orientation */
            if (caps->Orientation & NvOdmTouchOrientation_V_FLIP) {
                for (i = 0; i < fingers; i++) {
                    y[i] = caps->YMaxPosition +
                           caps->YMinPosition - y[i];
                }
            }
            if (caps->Orientation & NvOdmTouchOrientation_H_FLIP) {
                for (i = 0; i < fingers; i++) {
                    x[i] = caps->XMaxPosition +
                           caps->XMinPosition - x[i];
                }
            }
            if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
                for (i = 0; i < fingers; i++)
                    swapv(x[i],y[i]);
            }

            /* report co-ordinates to the multi-touch stack */
            for (i = 0; i < SUPPORTED_FINGERS; i++) {
                if (Pressure[i] >= 0) {
                    //printk("indx = %d x=%d y=%d z=%d\n", i, x[i], y[i], Pressure[i]);
                    input_report_abs(touch->input_dev,
                                     ABS_MT_TOUCH_MAJOR, Pressure[i]);
                    input_report_abs(touch->input_dev,
                                     ABS_MT_WIDTH_MAJOR, Width[i]);
                    input_report_abs(touch->input_dev,
                                     ABS_MT_POSITION_X, x[i]);
                    input_report_abs(touch->input_dev,
                                     ABS_MT_POSITION_Y, y[i]);
                    input_mt_sync(touch->input_dev);
                }
            }
            input_sync(touch->input_dev);

            bKeepReadingSamples = NV_FALSE;
            if (!touch->bPollingMode &&
                !NvOdmTouchHandleInterrupt(touch->hTouchDevice)) {
                /* Some more data to read keep going */
                bKeepReadingSamples = NV_TRUE;
            }
        }
    }

    return 0;
}

static int __init tegra_touch_probe(struct platform_device *pdev) {
    struct tegra_touch_driver_data *touch = NULL;
    struct input_dev *input_dev = NULL;
    int err, i = 0, offset = 0;
    NvOdmTouchCapabilities *caps;

    touch = kzalloc(sizeof(struct tegra_touch_driver_data), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (input_dev == NULL || touch == NULL) {
        input_free_device(input_dev);
        kfree(touch);
        err = -ENOMEM;
        pr_err("tegra_touch_probe: Failed to allocate input device\n");
        return err;
    }
    touch->semaphore = NvOdmOsSemaphoreCreate(0);
    if (!touch->semaphore) {
        err = -1;
        pr_err("tegra_touch_probe: Semaphore creation failed\n");
        goto err_semaphore_create_failed;
    }

    if (!NvOdmTouchDeviceOpen(&touch->hTouchDevice)) {
        err = -1;
        pr_err("tegra_touch_probe: NvOdmTouchDeviceOpen failed\n");
        goto err_open_failed;
    }
    touch->bPollingMode = NV_FALSE;
    if (!NvOdmTouchEnableInterrupt(touch->hTouchDevice, touch->semaphore)) {
        err = -1;
        pr_err("tegra_touch_probe: Interrupt failed, polling mode\n");
        touch->bPollingMode = NV_TRUE;
        touch->pollingIntervalMS = 10;
    }

    touch->task =
    kthread_create(tegra_touch_thread, touch, "tegra_touch_thread");

    if (touch->task == NULL) {
        err = -1;
        goto err_kthread_create_failed;
    }
    wake_up_process( touch->task );

    touch->input_dev = input_dev;
    touch->input_dev->name = NVODM_TOUCH_NAME;

    /* get hardware capabilities */
    NvOdmTouchDeviceGetCapabilities(touch->hTouchDevice, &touch->caps);
    caps = &touch->caps;

    /* Will generate sync at the end of all input */
    set_bit(EV_SYN, touch->input_dev->evbit);
    /* Event is key input type */
    set_bit(EV_KEY, touch->input_dev->evbit);
    /* Input values are in absoulte values */
    set_bit(EV_ABS, touch->input_dev->evbit);
    /* supported virtual keys */
    set_bit(BTN_TOUCH, touch->input_dev->keybit);
    for (i = 0; i < (caps->MaxNumberOfFingerCoordReported - 1); i++) {
        set_bit(BTN_2 + i, touch->input_dev->keybit);
    }

    /* expose multi-touch capabilities */
    set_bit(ABS_MT_TOUCH_MAJOR, touch->input_dev->keybit);
    set_bit(ABS_MT_POSITION_X, touch->input_dev->keybit);
    set_bit(ABS_MT_POSITION_Y, touch->input_dev->keybit);
    set_bit(ABS_X, touch->input_dev->keybit);
    set_bit(ABS_Y, touch->input_dev->keybit);

    if (caps->Orientation & NvOdmTouchOrientation_XY_SWAP) {
        touch->MaxY = caps->XMaxPosition;
        touch->MinY = caps->XMinPosition;
        touch->MaxX = caps->YMaxPosition;
        touch->MinX = caps->YMinPosition;

    } else {
        touch->MaxX = caps->XMaxPosition;
        touch->MinX = caps->XMinPosition;
        touch->MaxY = caps->YMaxPosition;
        touch->MinY = caps->YMinPosition;
    }

    input_set_abs_params(touch->input_dev, ABS_X, touch->MinX,
                         touch->MaxX, 0, 0);
    input_set_abs_params(touch->input_dev, ABS_Y, touch->MinY,
                         touch->MaxY, 0, 0);
    for (i = 0; i < caps->MaxNumberOfFingerCoordReported; i++) {
        input_set_abs_params(touch->input_dev, ABS_HAT0X + offset, touch->MinX,
                             touch->MaxX, 0, 0);
        input_set_abs_params(touch->input_dev, ABS_HAT0Y + offset, touch->MinY,
                             touch->MaxY, 0, 0);
        offset += 2;
    }
    input_set_abs_params(touch->input_dev, ABS_MT_POSITION_X,
                         touch->MinX, touch->MaxX, 0, 0);
    input_set_abs_params(touch->input_dev, ABS_MT_POSITION_Y,
                         touch->MinY, touch->MaxY, 0, 0);

    if (caps->IsPressureSupported) {
        input_set_abs_params(touch->input_dev, ABS_MT_TOUCH_MAJOR,
                             0, caps->MaxNumberOfPressureReported, 0, 0);
        input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0,
                             caps->MaxNumberOfPressureReported, 0, 0);
    } else {
        input_set_abs_params(touch->input_dev, ABS_MT_TOUCH_MAJOR,
                             0, TOOL_PRESSURE, 0, 0);
        input_set_abs_params(touch->input_dev, ABS_PRESSURE, 0,
                             TOOL_PRESSURE, 0, 0);
    }
    if (caps->IsWidthSupported) {
        input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0,
                             caps->MaxNumberOfWidthReported, 0, 0);
        input_set_abs_params(touch->input_dev, ABS_MT_WIDTH_MAJOR, 0,
                             caps->MaxNumberOfWidthReported, 0, 0);
    } else {
        input_set_abs_params(touch->input_dev, ABS_TOOL_WIDTH, 0,
                             TOOL_WIDTH_MAX, 0, 0);
        input_set_abs_params(touch->input_dev, ABS_MT_WIDTH_MAJOR, 0,
                             TOOL_WIDTH_MAX, 0, 0);
    }

    platform_set_drvdata(pdev, touch);

    egalax_chrdev_init(pdev);

    err = input_register_device(touch->input_dev);
    if (err) {
        pr_err("tegra_touch_probe: Unable to register input device\n");
        goto err_input_register_device_failed;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    touch->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    touch->early_suspend.suspend = tegra_touch_early_suspend;
    touch->early_suspend.resume = tegra_touch_late_resume;
    register_early_suspend(&touch->early_suspend);
#endif
    printk(KERN_INFO NVODM_TOUCH_NAME 
           ": Successfully registered the ODM touch driver %x\n", (NvU32)touch->hTouchDevice);
    return 0;

    err_input_register_device_failed:
    NvOdmTouchDeviceClose(touch->hTouchDevice);
    err_kthread_create_failed:
    /* FIXME How to destroy the thread? Maybe we should use workqueues? */
    err_open_failed:
    NvOdmOsSemaphoreDestroy(touch->semaphore);
    err_semaphore_create_failed:
    input_free_device(touch->input_dev);
    kfree(touch);
    return err;
}

static int tegra_touch_remove(struct platform_device *pdev) {
    struct tegra_touch_driver_data *touch = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&touch->early_suspend);
#endif
    touch->shutdown = 1;
    /* FIXME How to destroy the thread? Maybe we should use workqueues? */
    input_unregister_device(touch->input_dev);
    /* NvOsSemaphoreDestroy(touch->semaphore); */
    input_unregister_device(touch->input_dev);
    kfree(touch);
    return 0;
}


static struct platform_driver tegra_touch_driver = {
    .probe    = tegra_touch_probe,
    .remove  = tegra_touch_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = tegra_touch_suspend,
    .resume  = tegra_touch_resume,
#endif
    .driver  = {
        .name   = "tegra_touch",
    },
};

static int __devinit tegra_touch_init(void) {
    return platform_driver_register(&tegra_touch_driver);
}

static void __exit tegra_touch_exit(void) {
    egalax_chrdev_exit();
    platform_driver_unregister(&tegra_touch_driver);
}

module_init(tegra_touch_init);
module_exit(tegra_touch_exit);

MODULE_DESCRIPTION("Tegra ODM touch driver");


