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

#include "nvodm_ecompass_akm8975.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define NVODMECOMPASS_ENABLE_PRINTF 1

#if NVODMECOMPASS_ENABLE_PRINTF
    #define NVODMECOMPASS_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODMECOMPASS_PRINTF(x)
#endif

#define NV_DEBOUNCE_TIME_MS 0

#define NVODMECOMPASS_I2C_SPEED_KHZ	400

#define ECOMPASS_GUID NV_ODM_GUID('a','k','m','_','8','9','7','5')

static NvU8 ecompass_buffer[SENSOR_DATA_SIZE];

static NvBool
ReadReg(
    NvOdmEcompassHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len);
	
static NvBool
WriteReg(
    NvOdmEcompassHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len);	

static void GpioInterruptHandler(void *arg)
{
    NvU32 pinValue;
	NvU8 ret;
	NvU8 buffer[SENSOR_DATA_SIZE];
	NvU8 i;
    NvOdmEcompassHandle hDevice =  (NvOdmEcompassHandle)arg;
    NvOdmGpioGetState(hDevice->hGpioINT, hDevice->hPinINT, &pinValue);

    if (pinValue == 1)
    {
		ret=ReadReg(hDevice, AK8975_REG_ST1, buffer, SENSOR_DATA_SIZE);
		if (ret<0){
			NVODMECOMPASS_PRINTF(("AKM8975 compass driver: I2C failed\n"));
			return;
		}
		/* Check ST bit */
		if ((buffer[0] & 0x01) != 0x01) 
		{
			NVODMECOMPASS_PRINTF(("AKM8975 akm8975_work_func: ST is not set\n"));
			return;
		}		
		/* Check ST2 bit */
		if (((buffer[7]&0x04)==0x04)||((buffer[7]&0x08)==0x08))
		{
			NVODMECOMPASS_PRINTF(("AKM8975 akm8975_work_func: Data is Fail\n"));
			return;
		}
		NvOdmOsMemcpy(ecompass_buffer,buffer,SENSOR_DATA_SIZE);
        #if 0
        NVODMECOMPASS_PRINTF(("\n"));
		NVODMECOMPASS_PRINTF((" GPIO handle :\n"));
        for (i=0;i<SENSOR_DATA_SIZE;i++){
            NVODMECOMPASS_PRINTF((" Reg %d : [%02x]\n",i,ecompass_buffer[i]));
        }
        #endif
		NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
    }
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

void NvOdmEcompasspioInterruptMask(NvOdmEcompassHandle hDevice){
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_TRUE);
}

void NvOdmEcompasspioInterruptUnMask(NvOdmEcompassHandle hDevice){
	NvOdmGpioInterruptMask(hDevice->hGpioInterrupt, NV_FALSE);
}

static NvBool ConnectSemaphore(NvOdmEcompassHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODMECOMPASS_PRINTF(("AKM8975 compass driver: NvOdmGpioOpenError \n"));
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODMECOMPASS_PRINTF(( "AKM8975 compass driver: NvOdmOsSemaphoreCreate Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }

    mode = NvOdmGpioPinMode_InputInterruptHigh;
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }
	
    if (!(hDevice->hGpioInterrupt))
    {
        NVODMECOMPASS_PRINTF(("AKM8975 compass driver: NvOdm Ecompass NvOdmGpioInterruptRegister Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    return NV_TRUE;
}

static NvBool
WriteReg(
    NvOdmEcompassHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ECOMPASS_PACKET_SIZE-1 ) )
    {
        NVODMECOMPASS_PRINTF(("NvOdmI2c Set Regs Failed, max size is %d bytes\n", I2C_ECOMPASS_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len+1;

    // Write the accelerator RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMECOMPASS_I2C_SPEED_KHZ, 
            I2C_ECOMPASS_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    return NV_TRUE;
}

static NvBool
ReadReg(
    NvOdmEcompassHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_ECOMPASS_PACKET_SIZE-1 ) )
    {
        NVODMECOMPASS_PRINTF(("NvOdmI2c Get Regs Failed, max size is %d bytes\n", I2C_ECOMPASS_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    TransactionInfo.Address = hDevice->nDevAddr;
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    // Write the accelerometor RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMECOMPASS_I2C_SPEED_KHZ,
            I2C_ECOMPASS_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    s_ReadBuffer[0] = 0;
    TransactionInfo.Address = (hDevice->nDevAddr| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified RegAddr
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, NVODMECOMPASS_I2C_SPEED_KHZ,
            I2C_ECOMPASS_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);
    return NV_TRUE;
}

static NvBool AKM8975_Init(NvOdmEcompassHandle hEcompass){

	NvU8 TestVal;
	
	/* Check Connection */
	if(!ReadReg(hEcompass,AK8975_REG_WIA,&TestVal, 1))
		goto error;
	
	if (TestVal!=0x48)
	{
		NVODMECOMPASS_PRINTF(("AKM8975 compass driver : Unknown ID 0x%x \n",TestVal));
		goto error;
	}
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: ID is 0x%x \n",TestVal));
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: Init Passed \n"));
	return NV_TRUE;
error:
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: Init Failed \n"));
	return NV_FALSE;
}

static NvBool
akm8975_ReadXYZ(
	NvOdmEcompassHandle hDevice,
	NvS16* X,
    NvS16* Y,
    NvS16* Z)
{
	NvU8 Data[13];
	NvBool	NewData = 0;
	NvU8 i=0;
	
	if (!ReadReg(hDevice, AK8975_REG_WIA, &Data[0], 1))
		return NV_FALSE;
	
	if (!ReadReg(hDevice, AK8975_REG_INFO, &Data[1], 1))
		return NV_FALSE;
		
	if (!ReadReg(hDevice, AK8975_REG_ST1, &Data[2], 1))
		return NV_FALSE;		

	if (!ReadReg(hDevice, AK8975_REG_ST2, &Data[9], 1))
		return NV_FALSE;			
	
	for (i=0;i<6;i++)
	{
	if (!ReadReg(hDevice, AK8975_REG_HXL+i, &Data[3+i], 1))
		return NV_FALSE;		
	}	

	NewData = ((Data[0] == 0x48)&& !(Data[9] & 0x04) && (Data[2] & 0x01)) ? 1:0;

	NVODMECOMPASS_PRINTF((" X: [%x] [%x] Y: [%x] [%x] Z: [%x] [%x]",	Data[4],Data[3], Data[6], Data[5], Data[8], Data[7]));
	*X = 0xFFFF;
	*Y = 0xFFFF;
	*Z = 0xFFFF;
	return NewData;
}

NvBool NvOdmEcompassOpen(NvOdmEcompassHandle* hDevice)
{
	NvU32 i;
	NvOdmEcompassHandle	hEcompass;
	NvOdmIoModule IoModule = NvOdmIoModule_I2c;
	const NvOdmPeripheralConnectivity *pConnectivity;
	NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;

	hEcompass = NvOdmOsAlloc(sizeof(NvOdmEcompass));
	if (hEcompass == NULL)
	{
		NVODMECOMPASS_PRINTF(("AKM8975 compass driver: Open fail\n"));
		return NV_FALSE;
	}
	NvOdmOsMemset(hEcompass,0,sizeof(NvOdmEcompass));
	hEcompass->nBusType = NV_ECOMPASS_BUS_I2C;
	
	// Info of ecompass with current setting
	
    pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(
                        ECOMPASS_GUID);
    if (!pConnectivity)
    {
        NvOdmOsDebugPrintf(("NvOdmPeripheralGetGuid doesn't detect\
           AKM8975/B device\n"));
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
                hEcompass->I2CChannelId = pConnectivity->AddressList[i].Instance;
                hEcompass->nDevAddr = (NvU8)pConnectivity->AddressList[i].Address;				
                FoundI2cModule = NV_TRUE;
                IoModule = pConnectivity->AddressList[i].Interface;
                break;
            case NvOdmIoModule_Gpio:
                hEcompass->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hEcompass->GPIOPinINT = pConnectivity->AddressList[i].Address;				
                FoundGpio = NV_TRUE;
                break;
            default:
                break;
        }
    }

	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: I2C Channel ID [%X] \n",hEcompass->I2CChannelId));
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: I2C Device address [%X] \n", hEcompass->nDevAddr));					
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: I2C I/O module  [%X] \n", IoModule));					
	NVODMECOMPASS_PRINTF(( "AKM8975 compass driver: GPIO Port INT [%X] \n",hEcompass->GPIOPortINT));
	NVODMECOMPASS_PRINTF(( "AKM8975 compass driver: GPIO Pin INT [%X] \n",hEcompass->GPIOPinINT));	
	
	
    if (!FoundGpio || !FoundI2cModule)
    {
        NVODMECOMPASS_PRINTF(("AKM8975 compass driver: didn't find any periperal in discovery query for touch device Error \n"));
        goto error;
    }

    // Open I2C handle.
    hEcompass->hOdmI2C = NvOdmI2cOpen(IoModule, hEcompass->I2CChannelId);
    if (!hEcompass->hOdmI2C)
        goto error;
		
    hEcompass->RegsRead  = ReadReg;
    hEcompass->RegsWrite = WriteReg;		
	
	if (!AKM8975_Init(hEcompass))
		goto error;
	if (!ConnectSemaphore(hEcompass))
		goto error;
	
	*hDevice = hEcompass;
	return NV_TRUE;
    error:
		NVODMECOMPASS_PRINTF(("AKM8975 compass driver: Error during NvOdmEcompassOpen\n"));
		// Release all of resources requested.
		if (hEcompass){
            NvOdmI2cClose(hEcompass->hOdmI2C);
            hEcompass->hOdmI2C = NULL;
            NvOdmOsFree(hEcompass);
            *hDevice = NULL;		
		}
		return NV_FALSE;
}

void NvOdmecompassClose(NvOdmEcompassHandle hDevice)
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
	}
}

NvBool
NvOdmEcompass_Self_test(NvOdmEcompassHandle hDevice)
{
	NvU8 TestVal;

	if (!ReadReg(hDevice,AK8975_REG_CNTL,&TestVal, 1))
		goto access_error;	
	
	TestVal&=0x00;
	TestVal|=0x40;
	if (!WriteReg(hDevice, AK8975_REG_ASTC, &TestVal, 1))
		goto access_error;

access_error:	
	NVODMECOMPASS_PRINTF(("AKM8975 compass driver: Access Error\n"));
	return NV_FALSE;
}

void NvOdmecompassWaitInt(NvOdmEcompassHandle hDevice)
{
	NV_ASSERT(hDevice);
	NvOdmOsSemaphoreWait(hDevice->SemaphoreForINT);
}

void NvOdmecompassSignal(NvOdmEcompassHandle hDevice)
{
	NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}

NvBool 
NvOdmEcompassGetaxis(
	NvOdmEcompassHandle hDevice,
	NvS16	*X_axis,
	NvS16	*Y_axis,
	NvS16	*Z_axis)
{
	NvBool	NewData = 0;
	NvS16	TempX = 0;
	NvS16	TempY = 0;
	NvS16	TempZ = 0;
	
	NV_ASSERT(NULL != hDevice);
	NV_ASSERT(NULL != X_axis);
	NV_ASSERT(NULL != Y_axis);
	NV_ASSERT(NULL != Z_axis);
	
	NewData = akm8975_ReadXYZ(hDevice, &TempX, &TempY, &TempZ);
	
	return NewData;
}

NvBool
NvOdmEcompassTxI2C(NvOdmEcompassHandle hDevice,char *txData, int length)
{
	if (!WriteReg(hDevice, txData[0], &txData[1], (length-1)))
		return NV_FALSE;
    
    return NV_TRUE;
}

NvBool
NvOdmEcompassRxI2C(NvOdmEcompassHandle hDevice, char *rxData, int length)
{
	if(!ReadReg(hDevice,rxData[0],rxData, length)){
		return NV_FALSE;
	}   
	return NV_TRUE;
}

NvBool
NvOdmEcompassGetID(NvOdmEcompassHandle hDevice)
{
	NvU8 TestVal;
	
	/* Check Connection */
	if(!ReadReg(hDevice,AK8975_REG_WIA,&TestVal, 1)){
		NVODMECOMPASS_PRINTF(("AKM8975 : Access Fail \n"));
		return NV_FALSE;
	}
	
	if (TestVal!=0x48){
		NVODMECOMPASS_PRINTF(("AKM8975 : Unknown ID 0x%x \n",TestVal));
		return NV_FALSE;
	}
	NVODMECOMPASS_PRINTF(("AKM8975 : 0x%x \n",TestVal));
	return NV_TRUE;
}

NvBool
NvOdmEcompassGetMeasure(NvOdmEcompassHandle hDevice)
{
	NvU8 TestVal;
	
	if (!ReadReg(hDevice,AK8975_REG_CNTL,&TestVal, 1))
	{
		NVODMECOMPASS_PRINTF(("AKM8975 : Access Fail \n"));
		return NV_FALSE;	
	}

	TestVal&=0x00;
	TestVal|=AK8975_MODE_SNG_MEASURE;
	if (!WriteReg(hDevice, AK8975_REG_CNTL, &TestVal, 1))
	{
		NVODMECOMPASS_PRINTF(("AKM8975 : Access Fail \n"));
		return NV_FALSE;
	}
	return NV_TRUE;
}

NvBool
NvodmEcompassGetData(NvOdmEcompassHandle hDevice, char* databuffer)
{
	NvOdmOsMemcpy(databuffer, ecompass_buffer, SENSOR_DATA_SIZE);
}

