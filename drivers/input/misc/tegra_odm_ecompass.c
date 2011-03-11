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
//#include <linux/tegra_devices.h>
#include <linux/miscdevice.h> /* for misc register, let user to use ioctl */

#include <nvodm_services.h>
#include <nvodm_ecompass.h>

#include <linux/workqueue.h>	/* 20100817 Daniel */
#include <linux/earlysuspend.h>

#define DRIVER_NAME	"nvodm_ecompass"
#define READ_BUFFER_LENGTH 20

#define AKM8975_DEBUG		1	
#define AKM8975_DEBUG_MSG	1	
#define AKM8975_DEBUG_FUNC	0
#define AKM8975_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define AKM8975_RETRY_COUNT	10
#define AKM8975_DEFAULT_DELAY	100

#if AKM8975_DEBUG_MSG
#define AKMDBG(format, ...)	printk(KERN_INFO "AKM8975 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM8975_DEBUG_FUNC
#define AKMFUNC(func) printk(KERN_INFO "AKM8975 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

static struct input_dev *e_input_dev;

struct tegra_ecompass_device_data
{
	NvOdmEcompassDeviceHandle	hOdmEcom;
	struct task_struct	*task;
	struct input_dev	*input_dev;
	struct early_suspend akm_early_suspend;
	NvBool			bThreadAlive;
	NvBool			show_log;
};

struct tegra_ecompass_device_data *ecompass_dev;

static const char* parameter[] = {
	"log=",
	"frequency=",
	"forcemotion=",
	"forcetap=",
	"timetap=",
	"openclose=",
	"GET_ID=",
	"GET_MEAS=",
};

static enum {
	COMMAND_LOG = 0,
	COMMAND_FREQUENCY,
	COMMAND_FORCEMOTION,
	COMMAND_FORCETAP,
	COMMAND_TIMETAP,
	COMMAND_OPENCLOSE,
	COMMAND_ID,
	COMMAND_GET_MEAS,
}ecompass_enum;

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);	/* 20100817 Daniel */
static DECLARE_WAIT_QUEUE_HEAD(open_wq);		/* 20100817 Daniel */

static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t mv_flag;

static int failure_count = 0;
static short akmd_delay = AKM8975_DEFAULT_DELAY;
static atomic_t suspend_flag = ATOMIC_INIT(0);

static int AKI2C_RxData(char *rxData, int length);
static int AKI2C_TxData(char *txData, int length);

void close_odm_ecompass(void)
{
	NvOdmecompassClose(ecompass_dev->hOdmEcom);
	ecompass_dev->hOdmEcom=0;
}

NvBool open_def_odm_ecompass(void)
{
	NvBool err;
	
	err = NvOdmEcompassOpen(&(ecompass_dev->hOdmEcom));
	if (!err){
		pr_err("open_def_odm_ecompass: NvOdmEcompassOpen failed\n");
		return err;
	}
	return err;
}

/**Function to parse the values sent through sysfs and sets them accordingly
 */
void change_nvodm_ecompass_settings(NvU32 command,NvU32 value)
{
	switch (command){
	case COMMAND_ID:
		NvOdmEcompassGetID(ecompass_dev->hOdmEcom);
		break;
	case COMMAND_GET_MEAS:
		NvOdmEcompassGetMeasure(ecompass_dev->hOdmEcom);
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
ssize_t read_sysfs_ecompass(struct device *dev,
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
ssize_t write_sysfs_ecompass(struct device *dev,
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
				'\0', 10);
			ecompass_enum = i;
			goto proceed;
		}
	}
	goto err_0;

proceed:
	if (ecompass_dev->hOdmEcom)
		change_nvodm_ecompass_settings(ecompass_enum, value);
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

DEVICE_ATTR(tegra_ecompass, 0777, read_sysfs_ecompass, write_sysfs_ecompass);
NvS32 add_sysfs_entry_ecompass(void)
{
	return device_create_file(&ecompass_dev->input_dev->dev,
		&dev_attr_tegra_ecompass);
}

/** 
 * Function to register sysfs "device" and register the corresponding read
 * write functions
 */
NvS32 remove_sysfs_entry_ecompass(void)
{
	device_remove_file(&ecompass_dev->input_dev->dev, 
		&dev_attr_tegra_ecompass);
	return 0;
}

/**
 * Thread that waits for the interrupt from ODM and then sends out the
 * corresponding events.
 */
static NvS32 tegra_ecompass_thread (void *pdata)
{	
	struct tegra_ecompass_device_data *ecompass =
		(struct tegra_ecompass_device_data*)pdata;
	#if 0
	NvS16 x=0, y=0, z=0;
	ecompass->bThreadAlive = 1;
	
	while (ecompass->bThreadAlive) {
		NvOdmecompassWaitInt(ecompass->hOdmEcom);
		NvOdmEcompassGetaxis(
			ecompass->hOdmEcom, &x, &y, &z);
		if (ecompass->show_log) {
			printk(KERN_INFO "Ecompass: x=%d, y=%d, z=%d\n", x, y, z);
		}
	}
	#else
	char buffer[SENSOR_DATA_SIZE];

	ecompass->bThreadAlive = 1;
    while(ecompass->bThreadAlive){
		NvOdmecompassWaitInt(ecompass->hOdmEcom);
		memset(buffer, 0, SENSOR_DATA_SIZE);
		
		NvodmEcompassGetData(ecompass->hOdmEcom,buffer);
	
        #if 0
		int i;
        printk(KERN_INFO "\n");
		printk(KERN_INFO " compass thread :\n");
        for (i=0;i<SENSOR_DATA_SIZE;i++){
            printk(KERN_INFO " Reg %d : [%02x]\n",i,buffer[i]);
        }
        #endif
		
		mutex_lock(&sense_data_mutex);
		memcpy(sense_data, buffer, SENSOR_DATA_SIZE);
        atomic_set(&data_ready, 1);
		wake_up(&data_ready_wq);		/* 20100817 Daniel */
		mutex_unlock(&sense_data_mutex);        
    }
    #endif
	return -1;
}

static int AKI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
#if AKM8975_DEBUG_DATA
	int i;
	char addr = rxData[0];
#endif
	for (loop_i = 0; loop_i < AKM8975_RETRY_COUNT; loop_i++) {
		if(NvOdmEcompassRxI2C(ecompass_dev->hOdmEcom,rxData,length)==NV_TRUE){	
			break;
		}
		mdelay(10);
	}
	
	if (loop_i >= AKM8975_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, AKM8975_RETRY_COUNT);
		return -EIO;
	}
#if AKM8975_DEBUG_DATA
	printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
	for (i = 0; i < length; i++) {
		printk(KERN_INFO " %02x", rxData[i]);
	}
    printk(KERN_INFO "\n");
#endif	
	return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

#if AKM8975_DEBUG_DATA
	int i;
	for (i = 0; i < (length); i++) {
		printk(KERN_INFO " %d:[%02x] ", i,txData[i]);
	}
	printk(KERN_INFO "\n");
#endif

	for (loop_i = 0; loop_i < AKM8975_RETRY_COUNT; loop_i++) {
		if(NvOdmEcompassTxI2C(ecompass_dev->hOdmEcom,txData, length)==NV_TRUE){		
			break;
		}
		mdelay(10);
	}
	
	if (loop_i >= AKM8975_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, AKM8975_RETRY_COUNT);
		return -EIO;
	}
#if AKM8975_DEBUG_DATA
	printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
	for (i = 0; i < (length-1); i++) {
		printk(KERN_INFO " %02x", txData[i + 1]);
	}
	printk(KERN_INFO "\n");
#endif
	return 0;
}

static int AKECS_SetMode_SngMeasure(void)
{
	char buffer[2];
	
	atomic_set(&data_ready, 0);
	
	/* Set measure mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_MODE_SNG_MEASURE;
	
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_SelfTest(void)
{
	char buffer[2];
	
	/* Set measure mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_MODE_SELF_TEST;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_FUSEAccess(void)
{
	char buffer[2];
	
	/* Set measure mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_MODE_FUSE_ACCESS;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];
	
	/* Set powerdown mode */
	buffer[0] = AK8975_REG_CNTL;
	buffer[1] = AK8975_MODE_POWERDOWN;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode(char mode)
{
	int ret;
	
	switch (mode) {
		case AK8975_MODE_SNG_MEASURE:
			ret = AKECS_SetMode_SngMeasure();
			break;
		case AK8975_MODE_SELF_TEST:
			ret = AKECS_SetMode_SelfTest();
			break;
		case AK8975_MODE_FUSE_ACCESS:
			ret = AKECS_SetMode_FUSEAccess();
			break;
		case AK8975_MODE_POWERDOWN:
			ret = AKECS_SetMode_PowerDown();
			/* wait at least 100us after changing mode */
			udelay(100);
			break;
		default:
			AKMDBG("%s: Unknown mode(%d)", __func__, mode);
			return -EINVAL;
	}

	return ret;
}

static int AKECS_GetData(char *rbuf, int size)
{
	int i=0;
#ifdef AKM8975_DEBUG
	/* This function is not exposed, so parameters 
	 should be checked internally.*/
	if ((rbuf == NULL) || (size < SENSOR_DATA_SIZE)) {
		return -EINVAL;
	}
#endif
	wait_event_interruptible_timeout(data_ready_wq,atomic_read(&data_ready), 1000);		/* 20100817 Daniel */
	// Event occur or timeout expires wake up	
	#if 0
	while(!(atomic_read(&data_ready))&&(i<AKM8975_DEFAULT_DELAY)){
			i++;
			msleep(1);
	}
	if (i==AKM8975_DEFAULT_DELAY){
		AKMDBG("%s: time out occur.\n", __func__);	/* Daniel */
		//return -1;	//20100806 Daniel
	}
	#endif
	if (!atomic_read(&data_ready)) {
		AKMDBG("%s: data_ready is not set.", __func__);
		if (!atomic_read(&suspend_flag)) {
			AKMDBG("%s: suspend_flag is not set.", __func__);
			failure_count++;
			if (failure_count >= MAX_FAILURE_COUNT) {
				printk(KERN_ERR
					   "AKM8975 AKECS_GetData: successive %d failure.\n",
					   failure_count);
				atomic_set(&open_flag, -1);
				wake_up(&open_wq); /* 20100817 Daniel */
				failure_count = 0;
			}
		}
		return -1;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(rbuf, sense_data, size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);
	
	failure_count = 0;
	return 0;
}

static void AKECS_SetYPR(short *rbuf)
{
	//struct akm8975_data *data = i2c_get_clientdata(this_client);
#if AKM8975_DEBUG_DATA
	printk(KERN_INFO "AKM8975 %s:\n", __func__);
	printk(KERN_INFO "  yaw =%6d, pitch =%6d, roll =%6d\n",
		   rbuf[0], rbuf[1], rbuf[2]);
	printk(KERN_INFO "  tmp =%6d, m_stat =%6d, g_stat =%6d\n",
		   rbuf[3], rbuf[4], rbuf[5]);
	printk(KERN_INFO "  Acceleration[LSB]: %6d,%6d,%6d\n",
	       rbuf[6], rbuf[7], rbuf[8]);
	printk(KERN_INFO "  Geomagnetism[LSB]: %6d,%6d,%6d\n",
	       rbuf[9], rbuf[10], rbuf[11]);
#endif
	/* Report magnetic sensor information */
	if (atomic_read(&m_flag)) {
		input_report_abs(ecompass_dev->input_dev, ABS_RX, rbuf[0]);
		input_report_abs(ecompass_dev->input_dev, ABS_RY, rbuf[1]);
		input_report_abs(ecompass_dev->input_dev, ABS_RZ, rbuf[2]);
		input_report_abs(ecompass_dev->input_dev, ABS_RUDDER, rbuf[4]);
	}
	
	/* Report acceleration sensor information */
	if (atomic_read(&a_flag)) {
		input_report_abs(ecompass_dev->input_dev, ABS_X, rbuf[6]);
		input_report_abs(ecompass_dev->input_dev, ABS_Y, rbuf[7]);
		input_report_abs(ecompass_dev->input_dev, ABS_Z, rbuf[8]);
		input_report_abs(ecompass_dev->input_dev, ABS_WHEEL, rbuf[5]);
	}
	
	/* Report magnetic vector information */
	if (atomic_read(&mv_flag)) {
		input_report_abs(ecompass_dev->input_dev, ABS_HAT0X, rbuf[9]);
		input_report_abs(ecompass_dev->input_dev, ABS_HAT0Y, rbuf[10]);
		input_report_abs(ecompass_dev->input_dev, ABS_BRAKE, rbuf[11]);
	}
	
	input_sync(ecompass_dev->input_dev);
}

static int AKECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));	/* 20100817 Daniel */
	/* Condition False enter sleep */
	/*
	while((atomic_read(&open_flag) == 0))
	{
		msleep(1);
	}
	*/
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));	/* 20100817 Daniel */
	/* Condition False enter sleep */
	/*
	while((atomic_read(&open_flag) > 0))
	{
		msleep(1);
	}
	*/
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&mv_flag, 1);
}

/***** akm_aot functions ***************************************/
static int akm_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;

	AKMFUNC("akm_aot_open");
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq); 	/* 20100817 Daniel */
			ret = 0;
		}
	}
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
	AKMFUNC("akm_aot_release");
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq); /* 20100817 Daniel */
	return 0;
}

static int
akm_aot_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
	
	switch (cmd) {
		case ECS_IOCTL_APP_SET_MFLAG:
		case ECS_IOCTL_APP_SET_AFLAG:
		case ECS_IOCTL_APP_SET_MVFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			if (flag < 0 || flag > 1) {
				return -EINVAL;
			}
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	switch (cmd) {
		case ECS_IOCTL_APP_SET_MFLAG:
			atomic_set(&m_flag, flag);
			AKMDBG("MFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MFLAG:
			flag = atomic_read(&m_flag);
			break;
		case ECS_IOCTL_APP_SET_AFLAG:
			atomic_set(&a_flag, flag);
			AKMDBG("AFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_AFLAG:
			flag = atomic_read(&a_flag);
			break;
		case ECS_IOCTL_APP_SET_MVFLAG:
			atomic_set(&mv_flag, flag);
			AKMDBG("MVFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MVFLAG:
			flag = atomic_read(&mv_flag);
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			akmd_delay = flag;
			AKMDBG("Delay is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_DELAY:
			flag = akmd_delay;
			break;
		default:
			return -ENOTTY;
	}
	
	switch (cmd) {
		case ECS_IOCTL_APP_GET_MFLAG:
		case ECS_IOCTL_APP_GET_AFLAG:
		case ECS_IOCTL_APP_GET_MVFLAG:
		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag))) {
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	return 0;
}

/***** akmd functions ********************************************/
static int akmd_open(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_open");
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	AKMFUNC("akmd_release");
	AKECS_CloseDone();
	return 0;
}

static int
akmd_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	
	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	char rwbuf[RWBUF_SIZE];		/* for READ/WRITE */
	char mode;					/* for SET_MODE*/
	short value[12];			/* for SET_YPR */
	short delay;				/* for GET_DELAY */
	int status;					/* for OPEN/CLOSE_STATUS */
	int ret = -1;				/* Return value. */
	/*AKMDBG("%s (0x%08X).", __func__, cmd);*/
		
	switch (cmd) {
		case ECS_IOCTL_WRITE:
		case ECS_IOCTL_READ:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_MODE:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&mode, argp, sizeof(mode))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_YPR:
			if (argp == NULL) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&value, argp, sizeof(value))) {
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	switch (cmd) {
		case ECS_IOCTL_WRITE:
			AKMFUNC("IOCTL_WRITE");
			if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_READ:
			AKMFUNC("IOCTL_READ");
			if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_SET_MODE:
			AKMFUNC("IOCTL_SET_MODE");
			ret = AKECS_SetMode(mode);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_GETDATA:
			AKMFUNC("IOCTL_GET_DATA");
			ret = AKECS_GetData(sData, SENSOR_DATA_SIZE);
			if (ret < 0) {
				return ret;
			}
			break;
		case ECS_IOCTL_SET_YPR:
			AKECS_SetYPR(value);
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
			AKMFUNC("IOCTL_GET_OPEN_STATUS");
			status = AKECS_GetOpenStatus();
			AKMDBG("AKECS_GetOpenStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_CLOSE_STATUS:
			AKMFUNC("IOCTL_GET_CLOSE_STATUS");
			status = AKECS_GetCloseStatus();
			AKMDBG("AKECS_GetCloseStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_DELAY:
			AKMFUNC("IOCTL_GET_DELAY");
			delay = akmd_delay;
			break;
		default:
			return -ENOTTY;
	}
	
	switch (cmd) {
		case ECS_IOCTL_READ:
			if (copy_to_user(argp, &rwbuf, rwbuf[0]+1)) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GETDATA:
			if (copy_to_user(argp, &sData, sizeof(sData))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
		case ECS_IOCTL_GET_CLOSE_STATUS:
			if (copy_to_user(argp, &status, sizeof(status))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_DELAY:
			if (copy_to_user(argp, &delay, sizeof(delay))) {
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	return 0;
}

static NvS32 akm8975_early_suspend(struct platform_device *pdev, pm_message_t state)
{
	AKMFUNC("akm8975_early_suspend");
	atomic_set(&suspend_flag, 1);
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq); /* 20100817 Daniel */
	//ifndef disable_irq
	//disable_irq(this_client->irq);
	//#endif
	NvOdmEcompasspioInterruptMask(ecompass_dev->hOdmEcom);	
	AKMDBG("suspended with flag=%d", atomic_read(&reserve_open_flag));
	return 0;
}

static NvS32 akm8975_early_resume(struct platform_device *pdev)
{
	AKMFUNC("akm8975_early_resume");
	NvOdmEcompasspioInterruptUnMask(ecompass_dev->hOdmEcom);
	//#ifndef disable_irq
	//enable_irq(this_client->irq);
	//#endif
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq); /* 20100817 Daniel */
	AKMDBG("resumed with flag=%d", atomic_read(&reserve_open_flag));
	return 0;
}

/*********************************************/
static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.ioctl = akmd_ioctl,
};

static struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.ioctl = akm_aot_ioctl,
};

static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8975_dev",
	.fops = &akmd_fops,
};

static struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm8975_aot",
	.fops = &akm_aot_fops,
};
/*********************************************/

/**
 * All the device spefic initializations happen here. 
 */
static NvS32 __init tegra_ecompass_probe(struct platform_device *pdev)
{
	struct tegra_ecompass_device_data *ecompass = NULL;
	struct input_dev *input_dev = NULL;
	NvS32 err;
	NvBool ret;
	
	ecompass = kzalloc(sizeof(*ecompass), GFP_KERNEL);
	if (ecompass == NULL) {
		err = -ENOMEM;
		pr_err("tegra_ecompass_probe: Failed to memory\n");
		goto allocate_dev_fail;
	}	
	ecompass_dev = ecompass;
	
	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		err = -ENOMEM;
		pr_err("tegra_ecompass_probe: Failed to allocate input device\n");
		goto allocate_dev_fail;
	}
	/* Copy for a global variable */
	e_input_dev = input_dev;
	
	ret = open_def_odm_ecompass();
	if (!ret) {
		pr_err("open_def_odm_ecompass failed\n");
		goto allocate_dev_fail;
	}	
	
	//start the Int thread.
	ecompass->task = kthread_create(tegra_ecompass_thread,ecompass, "tegra_ecompass_thread");
	if (ecompass->task == NULL){
		err = -1;
		goto thread_create_failed;
	}
	wake_up_process(ecompass->task);
	
	ecompass->input_dev = input_dev;
	
	/* Setup input device */
	set_bit(EV_ABS, ecompass->input_dev->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(ecompass->input_dev, ABS_RX, 0, 23040, 0, 0);
	/* pitch (-180, 180) */
	input_set_abs_params(ecompass->input_dev, ABS_RY, -11520, 11520, 0, 0);
	/* roll (-90, 90) */
	input_set_abs_params(ecompass->input_dev, ABS_RZ, -5760, 5760, 0, 0);
	/* x-axis acceleration (720 x 8G) */
	input_set_abs_params(ecompass->input_dev, ABS_X, -5760, 5760, 0, 0);
	/* y-axis acceleration (720 x 8G) */
	input_set_abs_params(ecompass->input_dev, ABS_Y, -5760, 5760, 0, 0);
	/* z-axis acceleration (720 x 8G) */
	input_set_abs_params(ecompass->input_dev, ABS_Z, -5760, 5760, 0, 0);
	/* temparature */
	/*
	input_set_abs_params(ecompass->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
	 */
	/* status of magnetic sensor */
	input_set_abs_params(ecompass->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
	input_set_abs_params(ecompass->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
	/* x-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(ecompass->input_dev, ABS_HAT0X, -20480, 20479, 0, 0);
	/* y-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(ecompass->input_dev, ABS_HAT0Y, -20480, 20479, 0, 0);
	/* z-axis of raw magnetic vector (-4096, 4095) */
	input_set_abs_params(ecompass->input_dev, ABS_BRAKE, -20480, 20479, 0, 0);	
	
	platform_set_drvdata(pdev, ecompass);

	/* Set name */
	input_dev->name = "compass";
	
	/* Register */
	err = input_register_device(input_dev);
	if (err) {
		pr_err("tegra_ecompass_probe: Unable to register %s\
				input device\n", input_dev->name);
		goto input_register_device_failed;
	}	

	err = misc_register(&akmd_device);
	if (err) {
		printk(KERN_ERR
			   "AKM8975 akm8975_probe: akmd_device register failed\n");
		goto exit7;
	}
	
	err = misc_register(&akm_aot_device);
	if (err) {
		printk(KERN_ERR
		       "AKM8975 akm8975_probe: akm_aot_device register failed\n");
		goto exit8;
	}	
	
	err = add_sysfs_entry_ecompass();
	if (err) {
		pr_err("tegra_ecompass_probe: add_sysfs_entry failed\n");
		goto sysfs_failed;
	}

	mutex_init(&sense_data_mutex);

	init_waitqueue_head(&data_ready_wq);	/* 20100817 Daniel */
	init_waitqueue_head(&open_wq);			/* 20100817 Daniel */
	
	/* As default, report all information */
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&mv_flag, 1);
	
	ecompass_dev->akm_early_suspend.suspend = akm8975_early_suspend;
	ecompass_dev->akm_early_suspend.resume = akm8975_early_resume;
	register_early_suspend(&ecompass_dev->akm_early_suspend);
	
	printk(KERN_INFO DRIVER_NAME "successfully registered \n");
	return err;	
	
sysfs_failed:
	input_unregister_device(input_dev);
exit8:
	misc_deregister(&akmd_device);
exit7:
	input_unregister_device(ecompass_dev->input_dev);
input_register_device_failed:	
	ecompass->bThreadAlive = 0;
thread_create_failed:
	//KillThreadHere!
allocate_dev_fail:
	close_odm_ecompass();
	input_free_device(input_dev);
	kfree(ecompass);
	ecompass=0;
	err = -ENOMEM;
	
	return err;
}

static NvS32 tegra_ecompass_remove(struct platform_device *pdev)
{
    AKMFUNC("akm8975_remove");
	unregister_early_suspend(&ecompass_dev->akm_early_suspend);
	misc_deregister(&akm_aot_device);
	misc_deregister(&akmd_device);
	remove_sysfs_entry_ecompass();
	input_unregister_device(e_input_dev);
	kfree(ecompass_dev);
	AKMDBG("successfully removed.");
	return 0;
}

static struct platform_driver tegra_ecompass_driver = {
	.probe 	= tegra_ecompass_probe,
	.remove = tegra_ecompass_remove,
	//.suspend = akm8975_early_suspend,
	//.resume = akm8975_early_resume,	
	.driver = {
		.name = AKM8975_I2C_NAME,
	},
};

static NvS32 __devinit tegra_ecompass_init(void)
{
	return platform_driver_register(&tegra_ecompass_driver);
}

static void __exit tegra_ecompass_exit(void)
{
	platform_driver_unregister(&tegra_ecompass_driver);
}

module_init(tegra_ecompass_init);
module_exit(tegra_ecompass_exit);

MODULE_DESCRIPTION("AKM8975 compass driver");

