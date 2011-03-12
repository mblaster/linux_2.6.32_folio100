/*
 * Copyright (c) 2010 NVIDIA Corporation.
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

/*  NVIDIA Tegra ODM Kit Sample Accelerometer Adaptation of the
 *  WinCE Accelerometer Driver
 */

#include "nvodm_accelerometer_lsm303dlh.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#include <linux/kernel.h>

#include <linux/delay.h>

#define EEPROM_ID_E1206 0x0C06

#define NVODMACCELEROMETER_ENABLE_PRINTF 0

#define NVODMACCELEROMETER_I2C_SPEED_KHZ	400

#if NVODMACCELEROMETER_ENABLE_PRINTF
    #define NVODMACCELEROMETER_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODMACCELEROMETER_PRINTF(x)
#endif

#define NV_BMA150_MAX_FORCE_IN_REG 512 // It indicates force register length.
#define NV_DEBOUNCE_TIME_MS 0

#define ENABLE_XYZ_POLLING 0
static NvU32 PollingTime = 300; // (1000 * 225)/2*375(ms)
#define NV_BMA150_MAX_SAMPLE_RATE   12000 //Hz
#define NV_BMA150_MIN_SAMPLE_RATE   50 //Hz
// sw polling time is slower than hw sample rate 225 time
#define NV_BMA150_POLLING_FACTOR    225
static NvU32 CurrSampleRate = 2*375; // Current Sample Rate

//derick 20100716 START, add polling interval and g_rance
static NvU8 g_range = LSM303DLH_G_2G;
static NvU32 poll_interval = 100;
//derick 20100716 END

//define self test MAX and MIN value
#define MAX_X 55
#define MAX_Y 55
#define MAX_Z 55
#define MIN_X 5
#define MIN_Y 5
#define MIN_Z 5

#if 1	/* for CALIBRATION */ 
extern int tegra_board_nvodm_board_id(void);

#define NVODM_QUERY_I2C_CLOCK_SPEED      100    // kHz
#define NVODM_QUERY_I2C_EEPROM_ADDRESS   0xA0   // I2C device base address for EEPROM (7'h50)
#define NVODM_QUERY_CALIBRATION_START    0xFD
#define CAL_0 0
#define CAL_64 64
#define TOLERANCE 6  //10%
 
NvS32 CALIBRATION_X;
NvS32 CALIBRATION_Y;
NvS32 CALIBRATION_Z;
NvS32 OUT_X_NOST[4] = {0};
NvS32 OUT_Y_NOST[4] = {0};
NvS32 OUT_Z_NOST[4] = {0};
NvBool success[4]={NV_FALSE};

/* Calibration data format 
 * X:1 byte, Y:1 byte, Z:1 byte
 *
 *	+---------------+
 *  |7|6|5|4|3|2|1|0|
 *  +---------------+
 *	bit:  
 *		7: reserve for the sign inversion of an axis
 *		6: dirty bit, 0:used, 1:unused
 *	  5~0: Calibration value (signed)
 */
#endif	/* for CALIBRATION */

typedef struct BmaSampleRateRec
{
    // Range register value
    NvU8 RangeReg;
    // Bandwidth register value
    NvU8 BandWidthReg;
    // SampleRate(Hz) = Full scale acceleration range * BandWidth(in Hz)
    NvU32 SampleRate;
} BmaSampleRate;

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	3,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR1000}, {
	10,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR400}, {
	20,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR100}, {
	100,	LSM303DLH_ACC_PM_NORMAL | LSM303DLH_ACC_ODR50}, {
	200,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR10}, {
	500,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR5}, {
	1000,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR2}, {
	2000,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODR1}, {
	0,	LSM303DLH_ACC_ODR1000 | LSM303DLH_ACC_ODRHALF},};

BmaSampleRate OutputRate[] = {
    {0, 0, 2*25},
    {0, 1, 2*50},
    {0, 2, 2*100},
    {0, 3, 2*190},
    {1, 2, 4*100},
    {0, 4, 2*375},
    {1, 3, 4*190},
    {2, 2, 8*100},
    {0, 5, 2*750},
    {2, 3, 8*190},
    {0, 6, 2*1500},
    {1, 6, 4*1500},
    {2, 6, 8*1500}
};

//FIXME: protect this variable using spinlock.
static volatile int g_WaitCounter = 0;
static void LSM303DLH_ResetInterrupt(NvOdmAccelHandle hDevice);
NvBool lsm303dlh_acc_disable(NvOdmAccelHandle hDevice);

static void
SetPowerRail(
    NvOdmServicesPmuHandle hPMUDevice,
    NvU32 Id,
    NvBool IsEnable)
{
    NvOdmServicesPmuVddRailCapabilities vddrailcap;
    NvU32 settletime;

    if (hPMUDevice && Id)
    {
        NvOdmServicesPmuGetCapabilities(hPMUDevice, Id, &vddrailcap);
        if (IsEnable)
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id,
                vddrailcap.requestMilliVolts, &settletime);
        }
        else
        {
            NvOdmServicesPmuSetVoltage(hPMUDevice, Id,
                vddrailcap.MinMilliVolts, &settletime);
        }
        NvOdmOsWaitUS(settletime);
    }
}

static void GpioInterruptHandler(void *arg)
{
#if 1
    NvU32 pinValue;
    NvOdmAccelHandle hDevice =  (NvOdmAccelHandle)arg;

    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue);
//printk("\r\nLSM303DLH Interrupt Enter[%d]\n", pinValue);
    if (pinValue == 1)
    {
        //NVODMACCELEROMETER_PRINTF(("\r\nLSM303DLH Interrupt"));
//printk("\r\nLSM303DLH Interrupt.....................\n");
       g_WaitCounter = 10;
       //LSM303DLH_ResetInterrupt(hDevice);
    } else
        printk("\r\nLSM303DLH non-Interrupt\n");//NVODMACCELEROMETER_PRINTF(("\r\nLSM303DLH non-Interrupt"));

    if (pinValue == 1)
    {
        NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
    }
msleep(300); //derick 20100729, This is for GpioInterruptHandler to handle interrupt slowly
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
#endif
    return;
}

static NvBool ConnectSemaphore(NvOdmAccelHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODMACCELEROMETER_PRINTF(("NvOdm Accelerometer : NvOdmGpioOpen Error \n"));
	 printk("NvOdm Accelerometer : NvOdmGpioOpen Error \n");
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODMACCELEROMETER_PRINTF(("NvOdm Accelerometer : NvOdmOsSemaphoreCreate Error \n"));
	 printk("NvOdm Accelerometer : NvOdmOsSemaphoreCreate Error \n");
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }

    mode = NvOdmGpioPinMode_InputInterruptHigh;
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
	NVODMACCELEROMETER_PRINTF(( "NvOdm Accelerometer : NvOdmGpioInterruptRegister Error!! \n"));
	printk( "NvOdm Accelerometer : NvOdmGpioInterruptRegister Error!! \n");
        return NV_FALSE;
    }

    if (!(hDevice->hGpioInterrupt))
    {
        NVODMACCELEROMETER_PRINTF(("NvOdm Accelerometer : NvOdmGpioInterruptRegister Error \n"));
	 printk("NvOdm Accelerometer : NvOdmGpioInterruptRegister Error \n");
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
printk("NvOdm Accelerometer : NvOdmGpio ConnectSemaphore success!! \n");
    return NV_TRUE;
}

static NvBool
WriteReg(
    NvOdmAccelHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ACCELRATOR_PACKET_SIZE-1 ) )
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdmI2c Set Regs Failed, max size is %d bytes\n",
            I2C_ACCELRATOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len+1;

    // Write the accelerator RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMACCELEROMETER_I2C_SPEED_KHZ,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    return NV_TRUE;
}

/*
    To Write Calibration value into DMI EEPROM
*/
static NvOdmI2cStatus
NvOdmDMII2cWrite8(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 I2cAddr,
    NvU8 Offset,
    NvU8 *pData)
{
    NvU8 WriteBuffer[4];
    NvOdmI2cStatus Error;    
    NvOdmI2cTransactionInfo TransactionInfo;

    WriteBuffer[0] = Offset;
    WriteBuffer[1] = pData[0];
    WriteBuffer[2] = pData[1];
    WriteBuffer[3] = pData[2];

    TransactionInfo.Address = I2cAddr;
    TransactionInfo.Buf = WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 4;

    Error = NvOdmI2cTransaction(
			      			 hOdmI2c, 
			      			 &TransactionInfo, 
			      			 1, 
			      			 NVODM_QUERY_I2C_CLOCK_SPEED, 
			      			 NV_WAIT_INFINITE);
    return Error;
}
/*
    To Read Calibration value from DMI EEPROM
*/
static NvOdmI2cStatus
NvOdmDMII2cRead8(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 I2cAddr,
    NvU8 Offset,
    NvU8 *pData)
{
    NvU8 ReadBuffer[4];
    NvOdmI2cStatus Error;    
    NvOdmI2cTransactionInfo TransactionInfo[2];

    ReadBuffer[0] = Offset;

    TransactionInfo[0].Address = I2cAddr;
    TransactionInfo[0].Buf = &ReadBuffer[0];
    TransactionInfo[0].Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo[0].NumBytes = 1;

    TransactionInfo[1].Address = (I2cAddr | 0x1);
    TransactionInfo[1].Buf = &ReadBuffer[0];
    TransactionInfo[1].Flags = 0;
    TransactionInfo[1].NumBytes = 3;

    // Read data from ROM at the specified offset
    Error = NvOdmI2cTransaction(
        					hOdmI2c, 
        					&TransactionInfo[0],
        					2, 
        					NVODM_QUERY_I2C_CLOCK_SPEED, 
        					NV_WAIT_INFINITE);

    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }
    
    pData[0] = ReadBuffer[0];
    pData[1] = ReadBuffer[1];
    pData[2] = ReadBuffer[2];
    
    return Error;
}

NvBool
lsm303dlh_acc_calibration_write(NvOdmAccelHandle hDevice, NvS32 cal_x, NvS32 cal_y, NvS32 cal_z)
{
    NV_ASSERT(hDevice);  
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvOdmI2cStatus Error; 
	NvU8 dmi[3]={0};

    if(tegra_board_nvodm_board_id()==1){  
    	hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    }else{
     	hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, 0);
    }
    
    if (!hOdmI2c)
		return NV_FALSE;

  	dmi[0] = cal_x & 0x3F;
   	dmi[1] = cal_y & 0x3F;
	dmi[2] = cal_z & 0X3F;

	//printk("dmi[0]=%d(%x), dmi[1]=%d(%x), dmi[2]=%d(%x)\n", dmi[0],dmi[0],dmi[1],dmi[1],dmi[2],dmi[2]);
   
	Error = NvOdmDMII2cWrite8(
   						hOdmI2c, 
   						NVODM_QUERY_I2C_EEPROM_ADDRESS, 
   						NVODM_QUERY_CALIBRATION_START, 
   						dmi);
   
  	if ((Error != NvOdmI2cStatus_Success))
        goto error_0;          

  	NvOdmI2cClose(hOdmI2c);    
  	printk("Write calibration data success.\n");
   	return NV_TRUE; 
    
error_0:    
   	NvOdmI2cClose(hOdmI2c); 
  	printk("Write calibration data failed\n");
   	return NV_FALSE; 
}

NvBool
lsm303dlh_acc_calibration_read(NvOdmAccelHandle hDevice)
{
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvOdmI2cStatus Error; 
	NvS8 dmiValue[3]={0};

    if(tegra_board_nvodm_board_id()==1){  
   		hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    }else{
     	hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, 0);
    }
    
    if (!hOdmI2c)
    	return NV_FALSE;

	Error = NvOdmDMII2cRead8(
						hOdmI2c, 
						NVODM_QUERY_I2C_EEPROM_ADDRESS, 
						NVODM_QUERY_CALIBRATION_START, 
						dmiValue);
	
	if ((Error != NvOdmI2cStatus_Success))
	{
		NvOdmI2cClose(hOdmI2c);    
    	return NV_FALSE; 	
    }

	if ( (dmiValue[0]&0x40)||(dmiValue[1]&0x40)||(dmiValue[2]&0x40) )
	{
		CALIBRATION_X = 0;
		CALIBRATION_Y = 0;
		CALIBRATION_Z = 0;
		printk("Do not detect G-sensor calibration data!\n");
	}
	else
	{
		CALIBRATION_X = ( dmiValue[0] & 0x20 ) ? (dmiValue[0] | 0xFFFFFFE0) : dmiValue[0];
		CALIBRATION_Y = ( dmiValue[1] & 0x20 ) ? (dmiValue[1] | 0xFFFFFFE0) : dmiValue[1];
		CALIBRATION_Z = ( dmiValue[2] & 0x20 ) ? (dmiValue[2] | 0xFFFFFFE0) : dmiValue[2];
		
		printk("Detect G-sensor calibration data!\n");     
	}
	
	printk("CALIBRATION_X=%d, CALIBRATION_Y=%d, CALIBRATION_Z=%d\n", CALIBRATION_X, CALIBRATION_Y, CALIBRATION_Z);     

    NvOdmI2cClose(hOdmI2c);    
    return NV_TRUE; 
}

static NvBool
ReadReg(
    NvOdmAccelHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ACCELRATOR_PACKET_SIZE-1 ) )
    {
        NVODMACCELEROMETER_PRINTF((
            "NvOdmI2c Get Regs Failed, max size is %d bytes\n",
            I2C_ACCELRATOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    // Write the accelerometor RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMACCELEROMETER_I2C_SPEED_KHZ,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    s_ReadBuffer[0] = 0;
    TransactionInfo.Address = (hDevice->nDevAddr| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified RegAddr
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMACCELEROMETER_I2C_SPEED_KHZ,
            I2C_ACCELRATOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
    return NV_TRUE;
}

static NvBool lsm303dlh_acc_device_hw_init(NvOdmAccelHandle hAccel)
{
	NvU8 TestVal;
	int res;
	
   	TestVal = 0x07;
    	res = WriteReg(hAccel, CTRL_REG1, &TestVal, 1);
       if(res != NV_TRUE){
		return NV_FALSE;
       }
	TestVal = 0x00;
	res = WriteReg(hAccel, CTRL_REG2, &TestVal, 1);
       if(res != NV_TRUE){
		return NV_FALSE;
       }
	TestVal = 0x00;
	res = WriteReg(hAccel, CTRL_REG3, &TestVal, 1);
       if(res != NV_TRUE){
		return NV_FALSE;
       }
	TestVal = 0x00;
	res = WriteReg(hAccel, CTRL_REG4, &TestVal, 1);
       if(res != NV_TRUE){
		return NV_FALSE;
       }
	TestVal = 0x00;
	res = WriteReg(hAccel, CTRL_REG5, &TestVal, 1);
       if(res != NV_TRUE){
		return NV_FALSE;
       }
	return NV_TRUE;
}

static NvBool lsm303dlh_acc_update_odr(NvOdmAccelHandle hAccel, int poll_interval)
{
	int err = -1;
	int i;
	NvU8 TestVal;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		TestVal = odr_table[i].mask;
		if (poll_interval <= odr_table[i].cutoff)
			break;
	}
printk("1..poll_internal[%d], TestVal[0x%x]\n", poll_interval, TestVal); //derick 20100714 debug
	TestVal |= LSM303DLH_ACC_ENABLE_ALL_AXES;
printk("2..poll_internal[%d], TestVal[0x%x]\n", poll_interval, TestVal); //derick 20100714 debug
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if(WriteReg(hAccel, CTRL_REG1, &TestVal, 1) != NV_TRUE){
		return NV_FALSE;
	}
	return NV_TRUE;
}

static NvBool lsm303dlh_acc_read_reg(NvOdmAccelHandle hAccel)
{
	NvU8 Data[6];
	
	ReadReg(hAccel, AUTO_INCREMENT |CTRL_REG1, &Data[0], 6);
	printk("CTRL_REG1=0x%x\n", Data[0]);
	printk("CTRL_REG2=0x%x\n", Data[1]);
	printk("CTRL_REG3=0x%x\n", Data[2]);
	printk("CTRL_REG4=0x%x\n", Data[3]);
	printk("CTRL_REG5=0x%x\n", Data[4]);
	printk("CTRL_REG6=0x%x\n", Data[5]);
	return NV_TRUE;
}

static NvBool lsm303dlh_acc_init_interrupt(NvOdmAccelHandle hDevice)
{
	NvU8 Data = 0x02;   //0x06

	if(WriteReg(hDevice, CTRL_REG3, &Data, 1) != NV_TRUE){
		return NV_FALSE;
	}
	return NV_TRUE;
}

static NvBool lsm303dlh_Init(NvOdmAccelHandle hAccel)
{
    NvU8 TestVal;

    ReadReg(hAccel, CHIP_ID_REG, &TestVal, 1);
    if (TestVal != LSM303DLH_CHIP_ID)
    {
        NVODMACCELEROMETER_PRINTF(("Unknown LS303DLH ID = 0x%x\n", TestVal));
        goto error;
    }
    NVODMACCELEROMETER_PRINTF(("LSM303DLH ID is 0x%x\n", TestVal));

#if 1 //derick 20100714 hw init
	//gsensor initial
	if(lsm303dlh_acc_device_hw_init(hAccel) != NV_TRUE){
		goto error;
	}

	/* Set configuration register 4, which contains g range setting
	 *  NOTE: this is a straight overwrite because this driver does
	 *  not use any of the other configuration bits in this
	 *  register.  Should this become untrue, we will have to read
	 *  out the value and only change the relevant bits --XX----
	 *  (marked by X) */
       if(WriteReg(hAccel, CTRL_REG4, &g_range, 1) != NV_TRUE){
		goto error;
       }

	if(lsm303dlh_acc_update_odr(hAccel, poll_interval) != NV_TRUE){
		goto error;
	}
#if 1
	if(lsm303dlh_acc_init_interrupt(hAccel) != NV_TRUE){
		goto error;
	}
#endif
	if(lsm303dlh_acc_disable(hAccel) != NV_TRUE){
		goto error;		
	}

	lsm303dlh_acc_calibration_read(hAccel);
	
	//lsm303dlh_acc_device_power_off(acc); //close polling
	lsm303dlh_acc_read_reg(hAccel);
#endif
#if 0
    // Init Hw
    if (!ReadReg(hAccel, RANGE_BWIDTH_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xE0;
    TestVal |= 0x04; //Set bandwidth to 375hz
    if (!WriteReg(hAccel, RANGE_BWIDTH_REG, &TestVal, 1))
        goto error;

#if !(ENABLE_XYZ_POLLING)
    if (!ReadReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
    // Enable Advanced interrupt(6), latch int(4)
    TestVal |= (0 << 3) | (1 << 6) | (1 << 4);
    if (!WriteReg(hAccel, SMB150_CONF2_REG, &TestVal, 1))
        goto error;
#endif

    // Init Hw end
    // Set mode
    if (!ReadReg(hAccel, SMB150_CTRL_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xFE;
    if (!WriteReg(hAccel, SMB150_CTRL_REG, &TestVal, 1))
        goto error;
    // Set mode end

    // Set motion thres
    if (!ReadReg(hAccel, MOTION_THRS_REG, &TestVal, 1))
        goto error;
    TestVal = 0x0A;
    if (!WriteReg(hAccel, MOTION_THRS_REG, &TestVal, 1))
        goto error;
    // Set motion thres end

    // Set any motion int
    if (!ReadReg(hAccel, SMB150_CONF1_REG, &TestVal, 1))
        goto error;
    TestVal &= 0xFC;
    TestVal |= (1 << 6) | (1 << 1) | (1 << 0);
    if (!WriteReg(hAccel, SMB150_CONF1_REG, &TestVal, 1))
        goto error;
    // Set any motion int end
#endif
    NVODMACCELEROMETER_PRINTF(("\n lsm303dlh_Init passed\n"));
    return NV_TRUE;
error:
    NVODMACCELEROMETER_PRINTF(("\n lsm303dlh_Init failed\n"));
    return NV_FALSE;
}

static NvBool
LS303DLH_ReadXYZ(
    NvOdmAccelHandle hDevice,
    NvS32* X,
    NvS32* Y,
    NvS32* Z)
{
    NvU8 Data[6];
    NvBool NewData = 0;

    if (!ReadReg(hDevice, AUTO_INCREMENT | X_AXIS_LSB_REG, &Data[0], 6))
        return NV_FALSE;
    NewData = ( (Data[0] & 0x1) || (Data[2] & 0x1) || (Data[4] & 0x1) ) ? 1 : 0;
#if 0
    *X = ((Data[1] << 2) | (Data[0] >> 6));
    *Y = ((Data[3] << 2) | (Data[2] >> 6));
    *Z = ((Data[5] << 2) | (Data[4] >> 6));

    // Preserve sign bits.
    *X = *X << ((sizeof(*X)*8) - 10);
    *X = *X >> ((sizeof(*X)*8) - 10);
    *Y = *Y << ((sizeof(*Y)*8) - 10);
    *Y = *Y >> ((sizeof(*Y)*8) - 10);
    *Z = *Z << ((sizeof(*Z)*8) - 10);
    *Z = *Z >> ((sizeof(*Z)*8) - 10);
#else
	*X = (NvS32)Data[1];
	*Y = (NvS32)Data[3];
	*Z = (NvS32)Data[5];

	*X = (*X & 0x80) ? (*X | 0xFFFFFF00) : (*X);
	*Y = (*Y & 0x80) ? (*Y | 0xFFFFFF00) : (*Y);
	*Z = (*Z & 0x80) ? (*Z | 0xFFFFFF00) : (*Z);
#endif
    return NewData;
}

static void LSM303DLH_ResetInterrupt(NvOdmAccelHandle hDevice)
{
	NvU8 Data;

    ReadReg(hDevice, INT1_SRC, &Data, 1);
	//printk("INT1_SRC[0x%x]\n", Data);
#if 0
	NvU8 Data[6];

    ReadReg(hDevice, AUTO_INCREMENT | X_AXIS_LSB_REG, &Data[0], 6);
#endif
#if 0
    NvU8 Data = 0x02;

    WriteReg(hDevice, CTRL_REG3, &Data, 1);
#endif
}

NvBool NvOdmAccelOpen(NvOdmAccelHandle* hDevice)
{
    NvU32 i;
    NvOdmAccelHandle  hAccel;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;
    NvOdmBoardInfo BoardInfo;
    // Accelerometer is supported only on E1206.
#if 0
    if (!NvOdmPeripheralGetBoardInfo(EEPROM_ID_E1206, &BoardInfo))
    {
        NVODMACCELEROMETER_PRINTF(("\n Accelerometer is not supported \n"));
        return NV_FALSE;
    }
#endif
    hAccel = NvOdmOsAlloc(sizeof(NvOdmAccel));
    if (hAccel == NULL)
    {
        NVODMACCELEROMETER_PRINTF(("Error Allocating NvOdmAccel. \n"));
        return NV_FALSE;
    }
    NvOdmOsMemset(hAccel, 0, sizeof(NvOdmAccel));
    hAccel->nBusType = NV_ACCELEROMETER_BUS_I2C;

    // Info of accelerometer with current setting.
    hAccel->Caption.MaxForceInGs = 2000;
    hAccel->Caption.MaxTapTimeDeltaInUs = 255;
    hAccel->Caption.NumMotionThresholds = 1;
    hAccel->Caption.SupportsFreefallInt = 0;
    hAccel->Caption.MaxSampleRate = 100;
    hAccel->Caption.MinSampleRate = 3;
    hAccel->PowerState = NvOdmAccelPower_Fullrun;
    hAccel->AxisXMapping = NvOdmAccelAxis_Y;
    hAccel->AxisXDirection = 1;
    hAccel->AxisYMapping = NvOdmAccelAxis_X;
    hAccel->AxisYDirection = -1;
    hAccel->AxisZMapping = NvOdmAccelAxis_Z;
    hAccel->AxisZDirection = 1;

    hAccel->hPmu = NvOdmServicesPmuOpen();
    if (!hAccel->hPmu)
    {
        //NVODMACCELEROMETER_PRINTF(("NvOdmServicesPmuOpen Error \n"));
	printk("NvOdmServicesPmuOpen Error \n");
        goto error;
    }
    pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(
                        NV_ODM_GUID('l','s','m','3','0','3','a','c'));
    if (!pConnectivity)
    {
        //NvOdmOsDebugPrintf(("NvOdmPeripheralGetGuid doesn't detect BMA150 accelerometer device\n"));
	printk("NvOdmPeripheralGetGuid doesn't detect BMA150 accelerometer device\n");
        goto error;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_Other)
        goto error;

    for( i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch(pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
            case NvOdmIoModule_I2c_Pmu:
                hAccel->I2CChannelId = pConnectivity->AddressList[i].Instance;
                hAccel->nDevAddr = (NvU8)pConnectivity->AddressList[i].Address;
                FoundI2cModule = NV_TRUE;
                IoModule = pConnectivity->AddressList[i].Interface;
                break;
            case NvOdmIoModule_Gpio:
                hAccel->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hAccel->GPIOPinINT = pConnectivity->AddressList[i].Address;
                FoundGpio = NV_TRUE;
                break;
#if 0
            case NvOdmIoModule_Vdd:
                hAccel->VddId = pConnectivity->AddressList[i].Address;
                // Power on accelerometer according to Vddid
                SetPowerRail(hAccel->hPmu, hAccel->VddId, NV_TRUE);
                break;
#endif
            default:
                break;
        }
    }

	printk(KERN_INFO "lsm303dlh accel driver: I2C Channel ID [%X] \n",hAccel->I2CChannelId);
	printk(KERN_INFO "lsm303dlh accel driver: I2C Device address [%X] \n", hAccel->nDevAddr);					
	printk(KERN_INFO "lsm303dlh accel driver: I2C I/O module  [%X] \n", IoModule);					
	printk(KERN_INFO "lsm303dlh accel driver: GPIO Port INT [%X] \n",hAccel->GPIOPortINT);
	printk(KERN_INFO "lsm303dlh accel driver: GPIO Pin INT [%X] \n",hAccel->GPIOPinINT);	
#if 1
    if (!FoundGpio || !FoundI2cModule)
    {
        NVODMACCELEROMETER_PRINTF(("Accelerometer : didn't find any periperal in discovery query for touch device Error \n"));
	printk("Accelerometer : didn't find any periperal in discovery query for touch device Error \n");
        goto error;
    }
#endif
    // Open I2C handle.
    hAccel->hOdmI2C = NvOdmI2cOpen(IoModule, hAccel->I2CChannelId);
    if (!hAccel->hOdmI2C)
        goto error;

    hAccel->RegsRead  = ReadReg;
    hAccel->RegsWrite = WriteReg;

    if (!lsm303dlh_Init(hAccel))
        goto error;

    if (!ConnectSemaphore(hAccel))
        goto error;

    *hDevice = hAccel;
    return NV_TRUE;
    error:
        NVODMACCELEROMETER_PRINTF(("Error during LSM303DLH NvOdmAccelOpen\n"));
        // Release all of resources requested.
        if (hAccel)
        {
            SetPowerRail(hAccel->hPmu, hAccel->VddId, NV_FALSE);
            NvOdmServicesPmuClose(hAccel->hPmu);
            hAccel->hPmu = NULL;
            NvOdmI2cClose(hAccel->hOdmI2C);
            hAccel->hOdmI2C = NULL;
            NvOdmOsFree(hAccel);
            *hDevice = NULL;
        }
        return NV_FALSE;
}

void NvOdmAccelClose(NvOdmAccelHandle hDevice)
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
        NvOdmI2cClose(hDevice->hOdmI2C);

        // Power off accelermeter
        SetPowerRail(hDevice->hPmu, hDevice->VddId, NV_FALSE);
        if (hDevice->hPmu)
        {
            //NvAccelerometerSetPowerOn(0);
            NvOdmServicesPmuClose(hDevice->hPmu);
        }
    }
}

NvBool
NvOdmAccelSetIntForceThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
NvOdmAccelSetIntTimeThreshold(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType IntType,
    NvU32             IntNum,
    NvU32             Threshold)
{
    return NV_TRUE;
}

NvBool
NvOdmAccelSetIntEnable(
    NvOdmAccelHandle  hDevice,
    NvOdmAccelIntType  IntType,
    NvOdmAccelAxisType IntAxis,
    NvU32              IntNum,
    NvBool             Toggle)
{
    return NV_TRUE;
}

void
NvOdmAccelWaitInt(
    NvOdmAccelHandle    hDevice,
    NvOdmAccelIntType  *IntType,
    NvOdmAccelAxisType *IntMotionAxis,
    NvOdmAccelAxisType *IntTapAxis)
{
    NV_ASSERT(hDevice);
    NV_ASSERT(IntType);
    NV_ASSERT(IntMotionAxis);
    NV_ASSERT(IntTapAxis);
#if 1
    if ((g_WaitCounter > 0) || ENABLE_XYZ_POLLING)
    {
        NvOdmOsSemaphoreWaitTimeout( hDevice->SemaphoreForINT, PollingTime);
        g_WaitCounter--;
    }
    else
#endif
        NvOdmOsSemaphoreWait( hDevice->SemaphoreForINT);
}

void NvOdmAccelSignal(NvOdmAccelHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

NvBool
NvOdmAccelGetAcceleration(
    NvOdmAccelHandle hDevice,
    NvS32           *AccelX,
    NvS32           *AccelY,
    NvS32           *AccelZ)
{
    NvS32 data;
    NvBool NewData = 0;
    NvS32 TempAccelX = 0;
    NvS32 TempAccelY = 0;
    NvS32 TempAccelZ = 0;

    NV_ASSERT(NULL != hDevice);
    NV_ASSERT(NULL != AccelX);
    NV_ASSERT(NULL != AccelY);
    NV_ASSERT(NULL != AccelZ);
    NewData = LS303DLH_ReadXYZ(hDevice, &TempAccelX, &TempAccelY, &TempAccelZ);
	//printk("accel data[%d, %d, %d]\n", TempAccelX, TempAccelY, TempAccelZ); //derick
#if 0
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelX+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
#else
	
#if 1	/* CALIBRATION */
	TempAccelX -= CALIBRATION_X;
	TempAccelY -= CALIBRATION_Y;
	TempAccelZ -= CALIBRATION_Z;
#endif	/* CALIBRATION */

	data = TempAccelX;
#endif
    switch(hDevice->AxisXMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }
#if 0
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelY+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
#else
	data = TempAccelY;
#endif
    switch(hDevice->AxisYMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }
#if 0
    data = (((NvS32)(hDevice->Caption.MaxForceInGs))*TempAccelZ+(NvS32)(NV_BMA150_MAX_FORCE_IN_REG/2))/
             (NvS32)NV_BMA150_MAX_FORCE_IN_REG;
#else
	data = TempAccelZ;
#endif
    switch(hDevice->AxisZMapping)
    {
        case NvOdmAccelAxis_X:
            *AccelX = data*hDevice->AxisXDirection;
            break;
        case NvOdmAccelAxis_Y:
            *AccelY = data*hDevice->AxisYDirection;
            break;
        case NvOdmAccelAxis_Z:
            *AccelZ = data*hDevice->AxisZDirection;
            break;
        default:
            return NV_FALSE;
    }
#if 0 //derick 20100715
    NVODMACCELEROMETER_PRINTF(("\naccel output, x=%d,y=%d,z=%d, NewData=%d",
        *AccelX, *AccelY, *AccelZ, NewData));
#endif
    return NewData;
}

NvOdmAccelerometerCaps NvOdmAccelGetCaps(NvOdmAccelHandle hDevice)
{
    NV_ASSERT(NULL != hDevice);
    return hDevice->Caption;
}

#if 0
NvBool NvOdmAccelSetSampleRate(NvOdmAccelHandle hDevice, NvU32 SampleRate)
{
    NvU8 BandWidthReg, RangeReg, Val;
    NvU32 i;
    NvS32 index;

    if (!ReadReg(hDevice, RANGE_BWIDTH_REG, &Val, 1))
        goto error;

    index = -1;
    if (SampleRate <= NV_BMA150_MIN_SAMPLE_RATE)
    {
        index = 0;
    }
    else if (SampleRate >= NV_BMA150_MAX_SAMPLE_RATE)
    {
        index = NV_ARRAY_SIZE(OutputRate)-1;
    }
    else
    {
        for (i = 0; i < NV_ARRAY_SIZE(OutputRate); i++)
        {
            if ((SampleRate >= OutputRate[i].SampleRate) &&
                (SampleRate < OutputRate[i+1].SampleRate))
            {
                index = i;
                break;
            }
        }
    }

    if (index != -1)
    {
        BandWidthReg = OutputRate[index].BandWidthReg;
        RangeReg = OutputRate[index].RangeReg;
        Val = Val & 0xE0;
        Val = Val | BandWidthReg | (RangeReg << 3);
        if (!WriteReg(hDevice, RANGE_BWIDTH_REG, &Val, 1))
            goto error;
        CurrSampleRate = OutputRate[index].SampleRate;
        PollingTime = (1000 * NV_BMA150_POLLING_FACTOR)/CurrSampleRate; //ms
        return NV_TRUE;
    }
error:
    return NV_FALSE;
}
#endif

NvBool NvOdmAccelGetSampleRate(NvOdmAccelHandle hDevice, NvU32 *pSampleRate)
{
    *pSampleRate = CurrSampleRate;
    return NV_TRUE;
}

NvBool
NvOdmAccelSetPowerState(
    NvOdmAccelHandle hDevice,
    NvOdmAccelPowerType PowerState)
{
    return NV_TRUE;
}

NvBool 
lsm303dlh_acc_enable(NvOdmAccelHandle hDevice)
{
	NvU8 Data = 0x00;
	
	//enable interrupt
	Data = 0x02; //0x06
	if(WriteReg(hDevice, CTRL_REG3, &Data, 1) != NV_TRUE){
		return NV_FALSE;
	}
	//power on
	Data = 0x27;    
	if(WriteReg(hDevice, CTRL_REG1, &Data, 1) != NV_TRUE){
		return NV_FALSE;
	}
	return NV_TRUE;
}

NvBool 
lsm303dlh_acc_disable(NvOdmAccelHandle hDevice)
{
	NvU8 Data = 0x00;
	//disable interrupt
	Data = 0x00;
	if(WriteReg(hDevice, CTRL_REG3, &Data, 1) != NV_TRUE){
		return NV_FALSE;
	}
	//power off
	Data = 0x00;    
	if(WriteReg(hDevice, CTRL_REG1, &Data, 1) != NV_TRUE){
		return NV_FALSE;
	}

	return NV_TRUE;
}

NvBool
lsm303dlh_acc_self_test(NvOdmAccelHandle hDevice)
{
	int xyz[3] = { 0 };
	NvS32 AccelX = 0;
	NvS32 AccelY = 0;
	NvS32 AccelZ = 0;
	int err;
	int OUT_X_NOST=0,OUT_Y_NOST=0,OUT_Z_NOST=0;
	int OUT_X_ST=0,OUT_Y_ST=0,OUT_Z_ST=0;
	int DIFF_X,DIFF_Y,DIFF_Z;
	int i = 0;
	u8 shift;
	NvU8 Data;  //u8 buf[2];
	
	for(i=0; i<5; i++){
		xyz[0] = 0;
		xyz[1] = 0;
		xyz[2] = 0;
		//mutex_lock(lock);
		err = LS303DLH_ReadXYZ(hDevice, &AccelX, &AccelY, &AccelZ);
		if (err < 0)
			printk("get_acceleration_data failed\n");
		//mutex_unlock(&acc->lock);
		printk("i[%d], x[%d], y[%d], z[%d]\n",i , AccelX, AccelY, AccelZ);
		OUT_X_NOST += AccelX;
		OUT_Y_NOST += AccelY;
		OUT_Z_NOST += AccelZ;
	}
	OUT_X_NOST /= 5;
	OUT_Y_NOST /= 5;
	OUT_Z_NOST /= 5;
	printk("NOST, x[%d], y[%d], z[%d]\n", OUT_X_NOST, OUT_X_NOST, OUT_Z_NOST);
	printk("self-test START >>>\n");

	Data = 0x02 | LSM303DLH_G_2G;
	if(WriteReg(hDevice, CTRL_REG4, &Data, 1) != NV_TRUE)
		printk("set self test REG4 ERROR!!");

#if 0
	for(i=0; i<10; i++){
		//mutex_lock(&acc->lock);
		err = LS303DLH_ReadXYZ(hDevice, &AccelX, &AccelY, &AccelZ);
		if (err < 0)
			printk("get_acceleration_data failed\n");
		//mutex_unlock(&acc->lock);
	}
#endif
	msleep(1000);
	for(i=0; i<5; i++){
		AccelX= 0;
		AccelY= 0;
		AccelZ= 0;
		//mutex_lock(&acc->lock);
		err = LS303DLH_ReadXYZ(hDevice, &AccelX, &AccelY, &AccelZ);
		if (err < 0){
			printk("get_acceleration_data failed\n");
			return NV_FALSE;
		}
		//mutex_unlock(&acc->lock);
		printk("i[%d], x[%d], y[%d], z[%d]\n",i , AccelX, AccelY, AccelZ);
		OUT_X_ST += AccelX;
		OUT_Y_ST += AccelY;
		OUT_Z_ST += AccelZ;
	}	
	OUT_X_ST /= 5;
	OUT_Y_ST /= 5;
	OUT_Z_ST /= 5;
	printk("ST, x[%d], y[%d], z[%d]\n", OUT_X_ST, OUT_X_ST, OUT_Z_ST);

	// check the moving range
    	DIFF_X=abs(OUT_X_NOST-OUT_X_ST);	//+
		DIFF_Y=abs(OUT_Y_NOST-OUT_Y_ST);	//-
    	DIFF_Z=abs(OUT_Z_NOST-OUT_Z_ST);	//+
	printk("    DIFF_X=%d ,DIFF_Y=%d ,DIFF_Z=%d\n",DIFF_X,DIFF_Y,DIFF_Z);

	Data = 0x00 | LSM303DLH_G_2G;
	if(WriteReg(hDevice, CTRL_REG4, &Data, 1) != NV_TRUE)
	{
		printk("set self test REG4 ERROR!!\n");
	   	printk("self-test END <<<\n");
       	return NV_FALSE;	
	}
	if((DIFF_X > MAX_X)||(DIFF_X < MIN_X)||(DIFF_Y > MAX_Y)||(DIFF_Y < MIN_Y)||(DIFF_Z > MAX_Z)||(DIFF_Z < MIN_Z))
    {
       	printk("SELFTEST FAILED! X/Y/Z selftest moving out of limit!\n");
       	printk("self-test END <<<\n");
       	return NV_FALSE;
	}else{
		printk("SELFTEST SUCCESS!\n");
		printk("self-test END <<<\n");
		return NV_TRUE;
	}
}

NvBool
lsm303dlh_acc_calibration(NvOdmAccelHandle hDevice, NvS32 axis)
{
	NvS32 AccelX = 0;
	NvS32 AccelY = 0;
	NvS32 AccelZ = 0;
	NvS32 OUT_X = 0, OUT_Y = 0, OUT_Z = 0;
	NvS32 times = 10, err_times = 0;
	NvS32 cal_num = 4;
	NvBool err;
	NvS32 i = 0;

	NvOsSleepMS(1000);

	/* Acquire XYZ value 10 times */
	while((i < times) && (err_times < times))
	{
		err = LS303DLH_ReadXYZ(hDevice, &AccelX, &AccelY, &AccelZ);
		//printk("AccelX=%d, AccelY=%d, AccelZ=%d\n",AccelX, AccelY, AccelZ);

		if (err < 0)
		{
			err_times++;
		}
		else
		{
			OUT_X_NOST[axis-1] += AccelX;
			OUT_Y_NOST[axis-1] += AccelY;
			OUT_Z_NOST[axis-1] += AccelZ;
			i++;
		}
	}

	if (err_times)
	{
		printk("get_acceleration_data failed.\n");
		return NV_FALSE;
	}
	
	/* Average value */
	OUT_X_NOST[axis-1] /= times;
	OUT_Y_NOST[axis-1] /= times;
	OUT_Z_NOST[axis-1] /= times;

	/* Calibration */
	switch(axis)
	{
		case 1:	/* Z-axis calibration */
			printk("Z-axis calibration.\n");
			OUT_X_NOST[0] -= CAL_0;
			OUT_Y_NOST[0] -= CAL_0;
			OUT_Z_NOST[0] -= CAL_64;
			break;

		case 2:	/* X-axis calibration */
			printk("(-X)-axis calibration.\n");
			OUT_X_NOST[1] -= -CAL_64;
			OUT_Y_NOST[1] -= CAL_0;
			OUT_Z_NOST[1] -= CAL_0;
			break;	

		case 3:	/* Y-axis calibration */
			printk("Y-axis calibration.\n");
			OUT_X_NOST[2] -= CAL_0;
			OUT_Y_NOST[2] -= CAL_64;
			OUT_Z_NOST[2] -= CAL_0;
			break;
			
		case 4:	/* (-Y)-axis calibration */
			printk("(-Y)-axis calibration.\n");
			OUT_X_NOST[3] -= CAL_0;
			OUT_Y_NOST[3] -= -CAL_64;
			OUT_Z_NOST[3] -= CAL_0;
			break;
			
	}

	if ((abs(OUT_X_NOST[axis-1]) > TOLERANCE)||(abs(OUT_Y_NOST[axis-1]) > TOLERANCE)||(abs(OUT_Z_NOST[axis-1]) > TOLERANCE) )
	{
		printk("G-sensor calibration failed.\n");		
		return NV_FALSE;
	}

	//printk("Calibration value:%d, %d, %d\n", OUT_X_NOST[axis-1], OUT_Y_NOST[axis-1], OUT_Z_NOST[axis-1]);

	success[axis-1] = NV_TRUE;
	
	if (success[0] && success[1] && success[2] && success[3])
	{
		for(i=0; i<cal_num; i++)
		{
			OUT_X += OUT_X_NOST[i];
			OUT_Y += OUT_Y_NOST[i];
			OUT_Z += OUT_Z_NOST[i];
		}

		OUT_X /= cal_num;
		OUT_Y /= cal_num;
		OUT_Z /= cal_num;
		
		printk("G-sensor calibration value:%d, %d, %d\n", OUT_X, OUT_Y, OUT_Z);
		
		err = lsm303dlh_acc_calibration_write(hDevice, OUT_X, OUT_Y, OUT_Z);
		if (err == NV_FALSE)
		{
			printk("Write calibration data failed.\n");
			return NV_FALSE;
		}

		err = lsm303dlh_acc_calibration_read(hDevice);
		if (err == NV_FALSE)
		{
			printk("Read calibration data failed.\n");
			return NV_FALSE;
		}
	}
	
	return NV_TRUE;
}
