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
 
/*  NVIDIA Tegra ODM Kit  Cap Sensor Adaptation of the
 *  Cap Sensor Button Driver
 */

#include "nvodm_cap_sensor.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

#define NVODM_CAPSENSOR_ENABLE_PRINTF 1

#if NVODM_CAPSENSOR_ENABLE_PRINTF
    #define NVODM_CAPSENSOR_PRINTF(x) \
    do { \
        NvOdmOsPrintf x; \
    } while (0)
#else
    #define NVODM_CAPSENSOR_PRINTF(x)
#endif

static volatile unsigned int KeyEvent = 0;
#define NV_DEBOUNCE_TIME_MS 0
#define NVODM_QUERY_I2C_CLOCK_SPEED      100    // kHz
#define NVODM_QUERY_I2C_EEPROM_ADDRESS   0xA0   // I2C device base address for EEPROM (7'h50)
#define NVODM_QUERY_SENSITIVITY_START    0x7C   // 0x7C,0x7D,0x7E,0x7F

static unsigned int  cap_sensitivity = 0x08151515; // Default Sensitivity for Cap Sensor board
extern int tegra_board_nvodm_board_id(void);
static int dmi_flag = 0;
static NvU32 sensitivity  = 0xffffffff;        
static void GpioInterruptHandler(void *arg)
{
     
    NvOdmCapHandle hDevice =  (NvOdmCapHandle)arg;  
    KeyEvent = 0;  
    KeyEvent =(ReadButtonChange(hDevice)<<8);     
    KeyEvent |=ReadButtonStatus(hDevice);  
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);               
    NvOdmGpioInterruptDone(hDevice->hGpioInterrupt);
    return;
}

static NvBool ConnectSemaphore(NvOdmCapHandle hDevice)
{
    NvOdmGpioPinMode mode;
    NvOdmInterruptHandler callback =
        (NvOdmInterruptHandler)GpioInterruptHandler;

    hDevice->hGpioINT = (NvOdmServicesGpioHandle)NvOdmGpioOpen();
    if (!(hDevice->hGpioINT))
    {
        NVODM_CAPSENSOR_PRINTF((
            "NvOdm Cap Sensor : NvOdmGpioOpen Error \n"));
        return NV_FALSE;
    }

    hDevice->hPinINT = NvOdmGpioAcquirePinHandle(hDevice->hGpioINT,
                           hDevice->GPIOPortINT,
                           hDevice->GPIOPinINT);
    
    hDevice->SemaphoreForINT = NvOdmOsSemaphoreCreate(0);

    if (!(hDevice->SemaphoreForINT))
    {
        NVODM_CAPSENSOR_PRINTF((
            "NvOdm Cap Sensor : NvOdmOsSemaphoreCreate Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        return NV_FALSE;
    }

   // mode = NvOdmGpioPinMode_InputInterruptHigh;  
    mode =  NvOdmGpioPinMode_InputInterruptRisingEdge;
      
    if (NvOdmGpioInterruptRegister(hDevice->hGpioINT,
        &hDevice->hGpioInterrupt, hDevice->hPinINT, mode, callback,
        hDevice, NV_DEBOUNCE_TIME_MS) == NV_FALSE)
    {
        return NV_FALSE;
    }

    if (!(hDevice->hGpioInterrupt))
    {
        NVODM_CAPSENSOR_PRINTF((
            "NvOdm Cap Sensor : NvOdmGpioInterruptRegister Error \n"));
        NvOdmGpioClose(hDevice->hGpioINT);
        NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
        return NV_FALSE;
    }
    return NV_TRUE;
}
/*
    To Write Sensitivity into DMI EEPROM
*/
static NvOdmI2cStatus
NvOdmDMII2cWrite8(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 I2cAddr,
    NvU8 Offset,
    NvU32 pData)
{
    NvU8 WriteBuffer[5];
    NvOdmI2cStatus Error;    
    NvOdmI2cTransactionInfo TransactionInfo;

    WriteBuffer[0] = Offset;
    /*WriteBuffer[4] = (pData&0xff);
    WriteBuffer[3] = ((pData>>8)&0xff);
    WriteBuffer[2] = ((pData>>16)&0xff);
    WriteBuffer[1] = ((pData>>24)&0xff);
    */
    WriteBuffer[1] = (pData&0xff);
    WriteBuffer[2] = ((pData>>8)&0xff);
    WriteBuffer[3] = ((pData>>16)&0xff);
    WriteBuffer[4] = ((pData>>24)&0xff);
    
    TransactionInfo.Address = I2cAddr;
    TransactionInfo.Buf = WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 5;

    Error = NvOdmI2cTransaction(
        hOdmI2c, &TransactionInfo, 1, NVODM_QUERY_I2C_CLOCK_SPEED, NV_WAIT_INFINITE);
    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }

    return Error;
}
/*
    To Read Sensitivity from DMI EEPROM
*/
static NvOdmI2cStatus
NvOdmDMII2cRead8(
    NvOdmServicesI2cHandle hOdmI2c,
    NvU8 I2cAddr,
    NvU8 Offset,
    NvU8 *pData)
{
    NvU8 ReadBuffer[1];
    NvOdmI2cStatus Error;    
    NvOdmI2cTransactionInfo TransactionInfo;

    ReadBuffer[0] = Offset;

    TransactionInfo.Address = I2cAddr;
    TransactionInfo.Buf = ReadBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    Error = NvOdmI2cTransaction(
        hOdmI2c, &TransactionInfo, 1, NVODM_QUERY_I2C_CLOCK_SPEED, NV_WAIT_INFINITE);
    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }

    NvOdmOsMemset(ReadBuffer, 0, sizeof(ReadBuffer));  

    TransactionInfo.Address = (I2cAddr | 0x1);
    TransactionInfo.Buf = ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = 1;

    // Read data from ROM at the specified offset
    Error = NvOdmI2cTransaction(
        hOdmI2c, &TransactionInfo, 1, NVODM_QUERY_I2C_CLOCK_SPEED, NV_WAIT_INFINITE);
    if (Error != NvOdmI2cStatus_Success)
    {
        return Error;
    }
    *pData = ReadBuffer[0];
    return Error;
}

/*
    Write Cap Sensor Register to the PIC16F722
*/
static NvBool
WriteReg(
    NvOdmCapHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{
    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_CAPSENSOR_PACKET_SIZE-1 ) )
    {
        NVODM_CAPSENSOR_PRINTF((
            "NvOdmI2c Set Regs Failed, max size is %d bytes\n",
            I2C_CAPSENSOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    NvOdmOsMemcpy(&s_WriteBuffer[1], value, len);

    TransactionInfo.Address = (hDevice->nDevAddr<<1);
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = len+1;

    // Write the accelerator RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 100,
            I2C_CAPSENSOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    return NV_TRUE;
}

/*
    Read Cap Sensor Register  form the PIC16F722
*/

static NvBool
ReadReg(
    NvOdmCapHandle hDevice, // NvOdmAccelHandle hDevice,
    NvU8 RegAddr,
    NvU8* value,
    NvU32 len)
{

    NvOdmI2cTransactionInfo TransactionInfo;

    if ( (NULL == hDevice) || (NULL == value) ||
         (len > I2C_CAPSENSOR_PACKET_SIZE-1 ) )
    {
        NVODM_CAPSENSOR_PRINTF((
            "NvOdmI2c Get Regs Failed, max size is %d bytes\n",
            I2C_CAPSENSOR_PACKET_SIZE-1));
        return NV_FALSE;
    }

    s_WriteBuffer[0] = RegAddr;
    TransactionInfo.Address = (hDevice->nDevAddr<<1);
    TransactionInfo.Buf = s_WriteBuffer;
    TransactionInfo.Flags = NVODM_I2C_IS_WRITE;
    TransactionInfo.NumBytes = 1;

    // Write the accelerometor RegAddr (from where data is to be read).
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 100,
            I2C_CAPSENSOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    s_ReadBuffer[0] = 0;
    TransactionInfo.Address = ((hDevice->nDevAddr<<1)| 0x1);
    TransactionInfo.Buf = s_ReadBuffer;
    TransactionInfo.Flags = 0;
    TransactionInfo.NumBytes = len;

    //Read the data from the eeprom at the specified RegAddr
    if (NvOdmI2cTransaction(hDevice->hOdmI2C, &TransactionInfo, 1, 100,
            I2C_CAPSENSOR_TRANSACTION_TIMEOUT) != NvOdmI2cStatus_Success)
        return NV_FALSE;

    NvOdmOsMemcpy(value, &s_ReadBuffer[0], len);

    return NV_TRUE;
}

static NvU32 
NvOdmReadSensitivityFromDMI(void)
{
    NvU8  RetVal = 0;
    NvU32 Sensitivity  = 0;
    NvOdmI2cStatus Error; 
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    if(tegra_board_nvodm_board_id()==1){  
     hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    }else{
     hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, 0);
    }
    if (!hOdmI2c)
        goto error1; // Use default sensitivity setting in the chip
     
     Error = NvOdmDMII2cRead8(
            hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, NVODM_QUERY_SENSITIVITY_START, &RetVal);
     
     if ((Error != NvOdmI2cStatus_Success)||(RetVal>=0x5f)){     
         goto error2; 
     }   
     Sensitivity |=RetVal;     
     
     Error = NvOdmDMII2cRead8(
            hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, NVODM_QUERY_SENSITIVITY_START+1, &RetVal);
     
     if ((Error != NvOdmI2cStatus_Success)||(RetVal>=0x5f)){     
         goto error2; 
     }   
     Sensitivity |=(RetVal<<8);     
     
     Error = NvOdmDMII2cRead8(
            hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, NVODM_QUERY_SENSITIVITY_START+2, &RetVal);
     
     if ((Error != NvOdmI2cStatus_Success)||(RetVal>=0x5f)){     
         goto error2; 
     }   
     Sensitivity |=(RetVal<<16);    
     
     Error = NvOdmDMII2cRead8(
            hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, NVODM_QUERY_SENSITIVITY_START+3, &RetVal);
     
     if ((Error != NvOdmI2cStatus_Success)||(RetVal>=0x5f)){     
         goto error2; 
     }   
     Sensitivity |=(RetVal<<24);    
     
     NvOdmI2cClose(hOdmI2c);  
     return Sensitivity;       
 error2:
     NvOdmI2cClose(hOdmI2c);  
 error1:   
    return 0xFFFFFFFF;
}

static NvBool 
SetLowestSensitivity(NvOdmCapHandle hCap)
{
    // Init Hw    
    NvU8 TestVal = 0x5F;  //Set lowest sensitivity to 0x5F BTN0,BTN1,BTN2,BTN6  unused button
    
    if (!WriteReg(hCap, BTN0_SENSITIVITY_REG, &TestVal, 1))
        goto error;
    
    if (!WriteReg(hCap, BTN1_SENSITIVITY_REG, &TestVal, 1))
        goto error;
    
    if (!WriteReg(hCap, BTN2_SENSITIVITY_REG, &TestVal, 1))
        goto error;

    if (!WriteReg(hCap, BTN6_SENSITIVITY_REG, &TestVal, 1))
        goto error; 
     return NV_TRUE;
error:     
     return NV_FALSE;    
}  
 
static NvBool 
Cap_Sensor_Init(NvOdmCapHandle hCap)
{
    NvU8 TestVal = 0;   
  //  NvU32 sensitivity  = 0xffffffff;        
    
    ReadReg(hCap, CHIP_ID_REG, &TestVal, 1);       
    
    if (TestVal != CAPSENSOR_CHIP_ID)
    {
        NVODM_CAPSENSOR_PRINTF(("Unknown Cap Sensor ID = 0x%x\n", TestVal));
        goto error;
    }
    
    if(dmi_flag==0){
        sensitivity = NvOdmReadSensitivityFromDMI();
        dmi_flag = 1;
    }
    
    if(sensitivity==0xffffffff)      // Use default sensitivity
     sensitivity = cap_sensitivity;  /* Default Sensitivity 0x10(Menu) 0x15(Home) 0x15(Back) 0x15(Serach) */
    
    if(NvOdmCapAdjustSensitivity(hCap,sensitivity)==NV_FALSE)   
      goto error;
    
    if(SetLowestSensitivity(hCap)==NV_FALSE) 
      goto error;       
    
    NVODM_CAPSENSOR_PRINTF(("\n Cap Sensor_Init passed"));
    return NV_TRUE;
error:
    NVODM_CAPSENSOR_PRINTF(("\n Cap Sensor_Init failed"));   
    return NV_FALSE;
}

NvU8 
ReadButtonChange(NvOdmCapHandle hDevice)
{
  NvU8 Data;
  if (!ReadReg(hDevice, BUTTONS_CHANGE_REG, &Data, 1))
   return 0xFF;
   
   return Data;  
}

NvU8 
ReadButtonStatus(NvOdmCapHandle hDevice)
{
  NvU8 Data;
  if (!ReadReg(hDevice, BUTTONS_STATUS_REG, &Data, 1))
    return 0xFF;
   return Data;  
}
/*
    Read keyEvent
*/
NvU32 
ReadKeyEvent(NvOdmCapHandle hDevice)
{
     return KeyEvent;  
}
/*
    To write  sensitivity into DMI EEPROM
*/
NvBool
NvOdmCapProgSensitivity(NvOdmCapHandle hDevice, NvU32 value)
{
    NV_ASSERT(hDevice);  
    NvOdmServicesI2cHandle hOdmI2c = NULL;
    NvU8 Data = 0;
    NvOdmI2cStatus Error; 

    if(tegra_board_nvodm_board_id()==1){  
    hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    }else{
     hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, 0);
    }
    
    if (!hOdmI2c)
     return NV_FALSE;  
   
   Error = NvOdmDMII2cWrite8(hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, NVODM_QUERY_SENSITIVITY_START,   value);
   if ((Error != NvOdmI2cStatus_Success))
        goto error_0;          
   
    NvOdmI2cClose(hOdmI2c);    
    return NV_TRUE; 
error_0:    
    NvOdmI2cClose(hOdmI2c);    
    return NV_FALSE; 
     
}

/*
    Read device ID and Firmware Info.
*/
NvBool
NvOdmCapReadDevInfo(NvOdmCapHandle hDevice, NvU32* value)
{
   NV_ASSERT(hDevice);  
   NvU8 Data;
  if (!ReadReg(hDevice, CHIP_ID_REG, &Data, 1))
     goto error;
     *value = (Data<<24);
  if (!ReadReg(hDevice, FW_REV2_REG, &Data, 1))
     goto error;
     *value |= (Data<<16);
  if (!ReadReg(hDevice, FW_REV1_REG, &Data, 1))
     goto error;
     *value |= (Data<<8);
  if (!ReadReg(hDevice, FW_REV0_REG, &Data, 1))
     goto error;      
     *value |= Data ;     
    return NV_TRUE; 
error:   
    return NV_FALSE; 
}
/*
    To Adjust sensitivity of Cap sensor button
*/
NvBool
NvOdmCapAdjustSensitivity(NvOdmCapHandle hDevice, NvU32 value)
{
    NvU8 TestVal;
   
    
    TestVal = (value&0xff);     
    if (!WriteReg(hDevice, BTN3_SENSITIVITY_REG, &TestVal, 1))
        goto error;
     TestVal = ((value>>8)&0xff);
    if (!WriteReg(hDevice, BTN4_SENSITIVITY_REG, &TestVal, 1))
        goto error;
    TestVal = ((value>>16)&0xff);
    if (!WriteReg(hDevice, BTN5_SENSITIVITY_REG, &TestVal, 1))
        goto error;
    TestVal = ((value>>24)&0xff);
    if (!WriteReg(hDevice, BTN7_SENSITIVITY_REG, &TestVal, 1))
        goto error;    
    return NV_TRUE;    
error:
    NVODM_CAPSENSOR_PRINTF(("\n Cap Sensor_Init failed"));
    return NV_FALSE;    
}
/*
    Read Currenet sensitivity setting
*/
NvBool
NvOdmCapReadDMI(NvOdmCapHandle hDevice,  NvU8* value, NvU8 address)
{
  NvU8 Data;
  NvOdmI2cStatus Error; 
  NvOdmServicesI2cHandle hOdmI2c = NULL;
  
    if(tegra_board_nvodm_board_id()==1){
        hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c_Pmu, 0);
    }else{
        hOdmI2c = NvOdmI2cOpen(NvOdmIoModule_I2c, 0);
    }  
    
    if (!hOdmI2c) return NV_FALSE;  
    
    Error = NvOdmDMII2cRead8(
            hOdmI2c, NVODM_QUERY_I2C_EEPROM_ADDRESS, address, &Data);
     
    if ((Error != NvOdmI2cStatus_Success)) goto error;   
    *value = Data;  
    
    NvOdmI2cClose(hOdmI2c);
    return NV_TRUE;     
error:
    NvOdmI2cClose(hOdmI2c);
    NVODM_CAPSENSOR_PRINTF(("\n Read DMI EEPROM  failed\n"));
    return NV_FALSE;     
}
    
    
/*
    Read Currenet sensitivity setting
*/
NvBool
NvOdmCapReadSensitivity(NvOdmCapHandle hDevice,  NvU32* value)
{
  NvU8 Data;
  if (!ReadReg(hDevice, BTN7_SENSITIVITY_REG, &Data, 1))
     goto error;
     *value = (Data<<24);
  if (!ReadReg(hDevice, BTN5_SENSITIVITY_REG, &Data, 1))
     goto error;
     *value |= (Data<<16);
  if (!ReadReg(hDevice, BTN4_SENSITIVITY_REG, &Data, 1))
     goto error;
     *value |= (Data<<8);
  if (!ReadReg(hDevice, BTN3_SENSITIVITY_REG, &Data, 1))
     goto error;      
     *value |= Data ; 
     return NV_TRUE; 
error:
    NVODM_CAPSENSOR_PRINTF(("\n Read Cap Sensor Sensitivity failed\n"));
    return NV_FALSE;     
}
/*
    Read and Write Cap Sensor Register
*/
NvBool 
NvOdmCapCtlr(NvOdmCapHandle hDevice, NvU32 value, NvU32* RegValue)
{
    NvU8 TestVal,CapReg=0; 
    CapReg = ((value>>8)&0x7f);    
    if((value>>15)==1)
    {          
        TestVal  = (value&0xff);
        if (!WriteReg(hDevice, CapReg, &TestVal, 1))
            goto error;
     }
     else
     {     
        if (!ReadReg(hDevice, CapReg, &TestVal, 1))
        goto error;
        *RegValue = TestVal;
     }
    return NV_TRUE;    
error:
    NVODM_CAPSENSOR_PRINTF(("\n R/W Register Fail"));
    return NV_FALSE;    
}
    
NvBool NvOdmCapOpen(NvOdmCapHandle* hDevice)
{
    NvU32 i;
    NvOdmCapHandle  hCap;
    NvOdmIoModule IoModule = NvOdmIoModule_I2c;
    const NvOdmPeripheralConnectivity *pConnectivity;
    NvBool FoundGpio = NV_FALSE, FoundI2cModule = NV_FALSE;
   
    if(tegra_board_nvodm_board_id()==1){
        IoModule = NvOdmIoModule_I2c_Pmu;
    }
    
    hCap = NvOdmOsAlloc(sizeof(NvOdmCapSensor));
    
    if (hCap == NULL)
    {
        NVODM_CAPSENSOR_PRINTF(("Error Allocating NvOdmCapSensor. \n"));
        return NV_FALSE;
    }
    
    NvOdmOsMemset(hCap, 0, sizeof(NvOdmCapSensor));  
    
    if(tegra_board_nvodm_board_id()==1){   
      pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid( NV_ODM_GUID('c','a','p','p','w','r',' ',' '));   
    } else {
      pConnectivity = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid( NV_ODM_GUID('c','a','p','g','e','n','1',' '));
    }  
       
    if (!pConnectivity)
    {
        NvOdmOsDebugPrintf(("NvOdmPeripheralGetGuid doesn't detect\
            Cap Sensor device\n"));
        goto error;
    }

    if (pConnectivity->Class != NvOdmPeripheralClass_HCI)
        goto error;

    for( i = 0; i < pConnectivity->NumAddress; i++)
    {
        switch(pConnectivity->AddressList[i].Interface)
        {
            case NvOdmIoModule_I2c:
            case NvOdmIoModule_I2c_Pmu:
                hCap->I2CChannelId = pConnectivity->AddressList[i].Instance;
                hCap->nDevAddr = (NvU8)pConnectivity->AddressList[i].Address;                
                FoundI2cModule = NV_TRUE;
                IoModule = pConnectivity->AddressList[i].Interface;
                break;
            case NvOdmIoModule_Gpio:
                hCap->GPIOPortINT = pConnectivity->AddressList[i].Instance;
                hCap->GPIOPinINT = pConnectivity->AddressList[i].Address;                
                FoundGpio = NV_TRUE;
                break;
          
            default:
                break;
        }
    }

    if (!FoundGpio || !FoundI2cModule)
    {
        NVODM_CAPSENSOR_PRINTF(("Cap Sensor  : didn't find any periperal\
            in discovery query for touch device Error \n"));
        goto error;
    }

    // Open I2C handle.
    hCap->hOdmI2C = NvOdmI2cOpen(IoModule, hCap->I2CChannelId);
    if (!hCap->hOdmI2C)
        goto error;

    hCap->RegsRead  = ReadReg;
    hCap->RegsWrite = WriteReg;

    if (!Cap_Sensor_Init(hCap))
        goto error;
   
    if (!ConnectSemaphore(hCap))
        goto error;

    *hDevice = hCap;
    return NV_TRUE;
error:
        NVODM_CAPSENSOR_PRINTF(("Error during Cap Sensor NvOdmCapOpen\n"));
        // Release all of resources requested.
        if (hCap)
        {
   
            NvOdmI2cClose(hCap->hOdmI2C);
            hCap->hOdmI2C = NULL;
            NvOdmOsFree(hCap);
            *hDevice = NULL;
        }
        return NV_FALSE;
}

/*
    Set Cap Sensor Board into Suspend
*/
NvBool
NvOdmCapSuspend(NvOdmCapHandle hDevice)
{
    NvU8  TestVal;          
    TestVal = 0x1;     
    if (!WriteReg(hDevice,PCTRL_REG, &TestVal, 1))
        goto error;
    NVODM_CAPSENSOR_PRINTF(("\n Set Cap Sensor Board into Suspend\n"));
     return NV_TRUE; 
error:
    NVODM_CAPSENSOR_PRINTF(("\n Fail to set Cap Sensor Board into Suspend\n"));
    return NV_FALSE;     
}

/*
    Set Cap Sensor Board into Resume 
*/
NvBool
NvOdmResume(NvOdmCapHandle hDevice)
{
    NvU8  TestVal = 0x02,retry = 5;     
       
    if (!WriteReg(hDevice,PCTRL_REG, &TestVal, 1)) goto error;        
        
    while(1)  
    {
      ReadReg(hDevice, PCTRL_REG, &TestVal, 1);        
      if(((TestVal&0x02)==0)||((retry--)==0)) break;
         
    }   
       
    if(Cap_Sensor_Init(hDevice)== NV_FALSE)
     goto error;  
    
    NVODM_CAPSENSOR_PRINTF(("\n!!!!!!!!!! Cap Sensor Resume from Suspend !!!!!!!!!!!!!\n"));  
    return NV_TRUE;    
error:
    NVODM_CAPSENSOR_PRINTF(("\n!!!!!!!!!! Fail set Cap Sensor Board into Resume !!!!!!!!!!!!!"));
    return NV_FALSE;    
}

void NvOdmCapClose(NvOdmCapHandle hDevice)
{
    if (hDevice)
    {
        if (hDevice->SemaphoreForINT && hDevice->hGpioINT &&
            hDevice->hPinINT && hDevice->hGpioInterrupt)
        {
            NvOdmGpioInterruptUnregister(hDevice->hGpioINT,hDevice->hPinINT, hDevice->hGpioInterrupt);
            NvOdmOsSemaphoreDestroy(hDevice->SemaphoreForINT);
            NvOdmGpioReleasePinHandle(hDevice->hGpioINT, hDevice->hPinINT);
            NvOdmGpioClose(hDevice->hGpioINT);
        }
        NvOdmI2cClose(hDevice->hOdmI2C);  

    }
}

void
NvOdmCapWaitInt(NvOdmCapHandle hDevice)
{
    NV_ASSERT(hDevice);  
    NvOdmOsSemaphoreWait(hDevice->SemaphoreForINT);
}

void NvOdmCapSignal(NvOdmCapHandle hDevice)
{
    NvOdmOsSemaphoreSignal(hDevice->SemaphoreForINT);
}
