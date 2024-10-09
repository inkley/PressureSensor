/*
 Author:    Jerry Black
 User:      Tyler Inkley

Inkley_PressureSensor
• For use with second Evaluation Board (with pressure sensors)
• Store in some of the flash space on the Tiva chip
• Allows you to get complete real time data sets from the sensor
• Still includes a ring buffer in RAM that stores the data
• If you run the command to get sensor data, it will pull the latest sample
• Left the I2C routine in the firmware, to allow for some flexibility
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_can.h"
#include "driverlib/adc.h"
#include "inc/hw_i2c.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/flash.h"

uint32_t BuildVersion  = 1002;

//CAN Settings
#define CAN_ID      0x107
#define CAN_BAUD    500000 //500K

//I2C Settings
#define NUM_I2C_DATA 8
#define SLAVE_ADDRESS 0x3C

//Inkley Sensor Commands
#define icmdReadVersion         01 // Read Version
#define icmdReadData            02 // Sensor Read Data
#define icmdFlashStart          03 // Start recording data into flash
#define icmdFlashReadPos        04 // Read Flash at position
#define icmdFlashEraseFull      05 // Erase Flash
#define icmdFlashSetSampleSize  06 //Set flash sample size.
#define icmdFlashStatus         07// Get Status of flash reading. Read percent 100 means done.


#define SYSTICK_TIMING   1000 //1000 = 1mS   1 = 1 second  10 = 100mS 100 = 10mS

#define ADC_ReadTimeOut 100
#define I2C_TimeOut 10000


uint32_t GlobalTimer=0;
#define HeartBeatTime 10000 // In MilliSeconds.
uint32_t HeatbeatTrigger=0;

//Set the flash space to be used as NVR
//Set the Index to the end of flash space so we done start record data tell it get set back
//  to flash user space

#define FlashUserSpace  0x30000
uint32_t FlashIndex=0x40000;
uint32_t FlashSampleSize=0x10000;


int TimeOutClock = 0;
int I2C_TimeOutClock = 0;

uint32_t I2C_RcvCommand=0;
uint32_t I2C_RvcCommandParam=0;

bool I2C_RcvNewCommand = false;

typedef struct {
    uint32_t * bufdata;
    int head;
    int tail;
    int maxlen;
} circ_bbuf_t;

#define SENSORBUFSIZE 1024
uint32_t SensorBufferData[SENSORBUFSIZE];

circ_bbuf_t SensorBuf;

void Init_circ_bbuf(circ_bbuf_t *c)
{
    c->bufdata = SensorBufferData;
    c->maxlen = SENSORBUFSIZE;
    c->head=0;
    c->tail=0;
}

int circ_bbuf_push(circ_bbuf_t *c, uint32_t data)
{
    int next;

    next = c->head + 1;  // next is where head will point to after this write.
    if (next >= c->maxlen)
        next = 0;

    if (next == c->tail)  // if the head + 1 == tail, circular buffer is full
        return -1;

    c->bufdata[c->head] = data;  // Load data and then move
    c->head = next;             // head to next data offset.
    return 0;  // return success to indicate successful push.
}

int circ_bbuf_pop(circ_bbuf_t *c, uint32_t *data)
{
    int next;

    if (c->head == c->tail)  // if the head == tail, we don't have any data
        return -1;

    next = c->tail + 1;  // next is where tail will point to after this read.
    if(next >= c->maxlen)
        next = 0;

    *data = c->bufdata[c->tail];  // Read data and then move
    c->tail = next;              // tail to next offset.
    return 0;  // return success to indicate successful push.
}

//Global CAN
#define CAN_F_EMPTY     0
#define CAN_F_NEW       1
#define CAN_F_OVERRUN   2

typedef struct {
    char FLAGS;
    short ID;
    char MSG[8];
}CAN_MSG_T;

CAN_MSG_T CAN_RECV;

//utility functions
void DelayMS(unsigned int delay)
{
    SysCtlDelay(( SysCtlClockGet() / 3 / 1000) * delay );
}

uint32_t bit_clear(uint32_t number,uint32_t bit)
{
    return number & ~((uint32_t)1 << bit);
}

uint32_t bit_toogle(uint32_t number,uint32_t bit)
{
    return number ^ ((uint32_t)1 << bit);
}

uint32_t bit_set(uint32_t number, uint32_t bit)
{
    return number | ((uint32_t)1 << bit);
}

bool bit_check(uint32_t number,uint32_t bit)
{
    return (number >> bit) & (uint32_t)1;
}

//*****************************************************************************
//
// The interrupt handler for the for Systick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    uint32_t pui32ADC0Value[1];
    // SysTick set to interrupt every 1mS (1000 times a second)
    //Trigger a Read from the ADC0

    ADCProcessorTrigger(ADC0_BASE, 3);
    TimeOutClock=0;

    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
        if(TimeOutClock++>ADC_ReadTimeOut){
            ADCIntClear(ADC0_BASE, 3);
            return;
        }
    }


    ADCIntClear(ADC0_BASE, 3);


    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
    circ_bbuf_push(&SensorBuf,pui32ADC0Value[0]);

    //if FlashIndex is set to Flash user starting address (0x30000) then start dumping the ADC data into flash.
    if(FlashIndex<FlashUserSpace+FlashSampleSize)
    {
       FlashProgram(&pui32ADC0Value[0],FlashIndex+=4,4);
    }
    GlobalTimer++;
}

//*****************************************************************************
//
// The interrupt handler for the for I2C0 data slave interrupt.
//
//*****************************************************************************
void
I2C0SlaveIntHandler(void)
{
    //
    // Clear the I2C0 interrupt flag.
    //
    I2CSlaveIntClear(I2C0_BASE);

    //
    // Read the data from the slave.
    //
    I2C_RcvCommand = I2CSlaveDataGet(I2C0_BASE);

    I2C_RvcCommandParam = I2CSlaveDataGet(I2C0_BASE);

    //
    // Set a flag to indicate that the interrupt occurred.
    //
    I2C_RcvNewCommand= true;
}

void Init_ADC()
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);


    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_D | ADC_CTL_CH0 |

                                  ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 3);


    ADCIntClear(ADC0_BASE, 3);
}

void Init_Systick (void)
{


    SysTickPeriodSet(SysCtlClockGet()/SYSTICK_TIMING); //Set SYSTICK to interrupt every 1mS

    //
    // Enable the SysTick Interrupt.
    //
    SysTickIntEnable();

    //
    // Enable SysTick.
    //
    SysTickEnable();
}

void Init_I2C(void)
{
    //
        // The I2C0 peripheral must be enabled before use.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

        //
        // For this example I2C0 is used with PortB[3:2].  The actual port and
        // pins used may be different on your part, consult the data sheet for
        // more information.  GPIO port B needs to be enabled so these pins can
        // be used.
        // TODO: change this to whichever GPIO port you are using.
        //
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

        //
        // Configure the pin muxing for I2C0 functions on port B2 and B3.
        // This step is not necessary if your part does not support pin muxing.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);

        //
        // Select the I2C function for these pins.  This function will also
        // configure the GPIO pins pins for I2C operation, setting them to
        // open-drain operation with weak pull-ups.  Consult the data sheet
        // to see which functions are allocated per pin.
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

        //
        // Enable loopback mode.  Loopback mode is a built in feature that helps
        // for debug the I2Cx module.  It internally connects the I2C master and
        // slave terminals, which effectively lets you send data as a master and
        // receive data as a slave.  NOTE: For external I2C operation you will need
        // to use external pull-ups that are faster than the internal pull-ups.
        // Refer to the datasheet for more information.
        //
        //HWREG(I2C0_BASE + I2C_O_MCR) |= 0x01;

        //
        // Enable the I2C0 interrupt on the processor (NVIC).
        //
        IntEnable(INT_I2C0);

        //
        // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
        // gives you the ability to only enable specific interrupts.  For this case
        // we are only interrupting when the slave device receives data.
        //
        I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);

        //
        // Enable and initialize the I2C0 master module.  Use the system clock for
        // the I2C0 module.  The last parameter sets the I2C data transfer rate.
        // If false the data rate is set to 100kbps and if true the data rate will
        // be set to 400kbps.  For this example we will use a data rate of 100kbps.
        //
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        //
        // Enable the I2C0 slave module.
        //
        I2CSlaveEnable(I2C0_BASE);

        //
        // Set the slave address to SLAVE_ADDRESS.  In loopback mode, it's an
        // arbitrary 7-bit number (set in a macro above) that is sent to the
        // I2CMasterSlaveAddrSet function.
        //
        I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);



}

void I2C_SendData(uint32_t SData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 24));

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 16));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 8));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };

    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    I2C_TimeOutClock = I2C_TimeOut;
    while(I2CMasterBusy(I2C0_BASE))
    {
        if(--I2C_TimeOutClock==0)break;
    };
}

uint32_t CANPollCheck(unsigned char *candata,int MsgID,unsigned char Response)
{

    int rValue=0;
    uint32_t ulNewData;

    tCANMsgObject sMsgObjectRx;

    sMsgObjectRx.ui32MsgLen = 8;
    sMsgObjectRx.pui8MsgData = candata;

    ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);

    while(ulNewData & (1 << (MsgID - 1))) // (0x1 << i) & ulIDStatus
    {
            //
            // Read the message out of the message object.
            //
           CANMessageGet(CAN0_BASE, MsgID, &sMsgObjectRx, true);
           rValue++;
           ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
     }

    return rValue;
}



void IntCAN0Handler(void)
{
    uint32_t ulStatus,ulNewData;
    tCANMsgObject tempCANMsgObject;
    uint8_t CANMsg[8];
    unsigned char CANSlot=1;

    tempCANMsgObject.pui8MsgData = CANMsg;
    tempCANMsgObject.ui32MsgLen=8;


    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    CANIntClear(CAN0_BASE, ulStatus);

      //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus != CAN_INT_INTID_STATUS)
    {
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL); //Read cont status

        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
        if(ulNewData)
        {
                if(ulNewData & (1 << (CANSlot - 1))) //Check for new data in  message box 1
                {
                    CANMessageGet(CAN0_BASE, CANSlot,&tempCANMsgObject, true); //Get Data and Clear Pending flag
                    if(tempCANMsgObject.ui32MsgID == CAN_ID) // only get CAN messages sent to our CAN ID
                    {
                        CAN_RECV.ID = tempCANMsgObject.ui32MsgID;
                        memcpy(CAN_RECV.MSG,CANMsg,8);

                        if(bit_check(CAN_RECV.FLAGS,CAN_F_NEW )) //Already have a message in the buffer so overrun condition.
                        {
                            CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS,CAN_F_OVERRUN);
                        }
                        CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS,CAN_F_NEW);
                    }
                }
        }
    }
}


uint32_t CANSendINT(unsigned long CANID, uint32_t pui8MsgData)
{
        unsigned long TimeOut=0;
        tCANMsgObject sCANMessage;
        //uint32_t ui32MsgData;

        //ui32MsgData = 0;
        sCANMessage.ui32MsgID = CANID;
        sCANMessage.ui32Flags = 0;
        sCANMessage.ui32MsgLen = 4;
        sCANMessage.pui8MsgData =(uint8_t *)&pui8MsgData;

        TimeOut=0;
        CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);


        while(CANStatusGet(CAN0_BASE,CAN_STS_TXREQUEST) !=0)
        {
                TimeOut++;
                SysCtlDelay(SysCtlClockGet() / 30000);
                if(TimeOut>0x0001000)
                {
                     return 0xffffffff;
                }
        }
        return 0;
}
uint32_t CANSendMSG(unsigned long CANID, uint8_t *pui8MsgData)
{
        unsigned long TimeOut=0;
        tCANMsgObject sCANMessage;
        //uint32_t ui32MsgData;

        //ui32MsgData = 0;
        sCANMessage.ui32MsgID = CANID;
        sCANMessage.ui32Flags = 0;
        sCANMessage.ui32MsgLen = 8;
        sCANMessage.pui8MsgData = pui8MsgData;

        TimeOut=0;
        CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);


        while(CANStatusGet(CAN0_BASE,CAN_STS_TXREQUEST) !=0)
        {
                TimeOut++;
                SysCtlDelay(SysCtlClockGet() / 30000);
                if(TimeOut>0x0001000)
                {
                     return 0xffffffff;
                }
        }
        return 0;
}

//Setup a Can Message location for Receiving
void CANListnerEX(int MsgID)
{
   tCANMsgObject sMsgObjectRx;
   sMsgObjectRx.ui32MsgID = 0;
   sMsgObjectRx.ui32MsgIDMask = 0;
   sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_EXTENDED_ID;
   sMsgObjectRx.ui32MsgLen = 8;
   sMsgObjectRx.pui8MsgData = (unsigned char *)0xffffffff;
   CANMessageSet(CAN0_BASE, MsgID, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
}

void Init_CAN(uint32_t Baud)
{
    //Using Port B pins 4 and 5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //Using CAN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    //Init CAN Perph.
    CANInit(CAN0_BASE);

    //Set baud rate
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), Baud);

    //Tell which interrupts to trigger
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //Enable Interrupts
    IntEnable(INT_CAN0);

    //Enable CAN
    CANEnable(CAN0_BASE);

    CANRetrySet(CAN0_BASE, true);

    DelayMS(10);
    CANListnerEX(1); //broadcast lister will receive on mail box 1.
    DelayMS(10);
}
/**
 * main.c
 */
int main(void)
{

    uint32_t BufDataVar =0;
    uint32_t CANID_tmp=0;
    uint32_t CANVAL_tmp=0;
    uint8_t CAN_RESP[8];
    uint8_t CAN_CMD_REQUEST=0;

    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
              SYSCTL_XTAL_16MHZ);


    Init_ADC();
    Init_Systick();
    Init_I2C();
    Init_circ_bbuf(&SensorBuf);
    Init_CAN(CAN_BAUD);

    //Erase the upper flash space to be used for logging.
    FlashErase(FlashUserSpace);

    //Turn of the CAN bus listening using message 1 of the 32 message box in the CAN Perph.
    CANListnerEX(1);

    while(1)
     {
        if(bit_check(CAN_RECV.FLAGS,CAN_F_NEW))
        {
            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS,CAN_F_NEW);
            CANID_tmp = (CAN_RECV.MSG[1]<<8 ) + CAN_RECV.MSG[2];
            CANVAL_tmp = (CAN_RECV.MSG[3]<< 24) + (CAN_RECV.MSG[4]<< 16) + (CAN_RECV.MSG[5]<< 8) + (CAN_RECV.MSG[6]);

            CAN_RESP[0]=0x08; // Length for msg
            CAN_RESP[1] = (CAN_ID >> 8) & 0xFF;
            CAN_RESP[2] = CAN_ID & 0xFF;
            CAN_RESP[3] = CAN_RECV.MSG[0]; //CMD send
            CAN_RESP[4] = 0x00;
            CAN_RESP[5] = 0x00;
            CAN_RESP[6] = 0x00;
            CAN_RESP[7] = 0x00;

            CAN_CMD_REQUEST = CAN_RECV.MSG[0];
            switch(CAN_CMD_REQUEST)
            {
                 case icmdReadVersion: // Read Version
                     CAN_RESP[4] = (uint8_t)(((uint32_t)BuildVersion) >> 24);
                     CAN_RESP[5] = (uint8_t)(((uint32_t)BuildVersion) >> 16);
                     CAN_RESP[6] = (uint8_t)(((uint32_t)BuildVersion) >> 8);
                     CAN_RESP[7] = (uint8_t)(uint32_t)BuildVersion;
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;

                 case icmdReadData: // Sensor Read Data
                     circ_bbuf_pop(&SensorBuf,&BufDataVar);
                     CAN_RESP[4] = (uint8_t)(BufDataVar >> 24);
                     CAN_RESP[5] = (uint8_t)(BufDataVar >> 16);
                     CAN_RESP[6] = (uint8_t)(BufDataVar >> 8);
                     CAN_RESP[7] = (uint8_t)(BufDataVar);
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;

                 case icmdFlashReadPos: // Read Flash at position
                     CAN_RESP[4] = (uint8_t)(FlashIndex >> 24);
                     CAN_RESP[5] = (uint8_t)(FlashIndex >> 16);
                     CAN_RESP[6] = (uint8_t)(FlashIndex >> 8);
                     CAN_RESP[7] = (uint8_t)(FlashIndex);
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;

                 case icmdFlashEraseFull: // Erase Flash
                     FlashErase(FlashUserSpace);
                     CAN_RESP[4] = (uint8_t)(FlashUserSpace >> 24);
                     CAN_RESP[5] = (uint8_t)(FlashUserSpace >> 16);
                     CAN_RESP[6] = (uint8_t)(FlashUserSpace >> 8);
                     CAN_RESP[7] = (uint8_t)(FlashUserSpace);
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;

                 case icmdFlashStart:
                     FlashIndex = FlashUserSpace;
                     CAN_RESP[4] = (uint8_t)(FlashIndex >> 24);
                     CAN_RESP[5] = (uint8_t)(FlashIndex >> 16);
                     CAN_RESP[6] = (uint8_t)(FlashIndex >> 8);
                     CAN_RESP[7] = (uint8_t)(FlashIndex);
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;
                 case icmdFlashSetSampleSize:
                    if(CANVAL_tmp==0 || CANVAL_tmp>0x10000)CANVAL_tmp=0x10000;
                    FlashSampleSize = CANVAL_tmp;
                    CAN_RESP[4] = (uint8_t)(FlashSampleSize >> 24);
                    CAN_RESP[5] = (uint8_t)(FlashSampleSize >> 16);
                    CAN_RESP[6] = (uint8_t)(FlashSampleSize >> 8);
                    CAN_RESP[7] = (uint8_t)(FlashSampleSize);
                    CANSendMSG(CANID_tmp,CAN_RESP);
                    break;
                 case icmdFlashStatus:
                     CANVAL_tmp = ((FlashIndex-FlashUserSpace)/FlashSampleSize)*100;
                     CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                     CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                     CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                     CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                     CANSendMSG(CANID_tmp,CAN_RESP);
                     break;
             }
            bit_clear(CAN_RECV.FLAGS,CAN_F_NEW);
            HeatbeatTrigger=GlobalTimer+HeartBeatTime;
        }
        if(bit_check(CAN_RECV.FLAGS,CAN_F_OVERRUN))
        {
            //detected an over run condition
            bit_clear(CAN_RECV.FLAGS,CAN_F_OVERRUN);
        }

        if(I2C_RcvNewCommand)
        {
            switch(I2C_RcvCommand)
            {
                case icmdReadVersion: // Read Version
                    I2C_SendData(BuildVersion);
                    break;

                case icmdReadData: // Sensor Read Data
                    circ_bbuf_pop(&SensorBuf,&BufDataVar);
                    I2C_SendData(BufDataVar);
                    break;

                case icmdFlashReadPos: // Read Flash at position
                    I2C_SendData(FlashIndex);
                    break;

                case icmdFlashEraseFull: // Erase Flash
                    FlashErase(FlashUserSpace);
                    break;

                case icmdFlashStart:
                    FlashIndex = FlashUserSpace;
                    break;

            }
            I2C_RcvNewCommand=false;
        }
        if(GlobalTimer > HeatbeatTrigger)
        {
            //Every 1 sec of IDLE transfer send a broadcast out with the global timer value.
            CAN_RESP[0] = 0x08; // Length for msg
            CAN_RESP[1] = (CAN_ID >> 8) & 0xFF;
            CAN_RESP[2] = CAN_ID & 0xFF;
            CAN_RESP[3] = 0x7F; //Heart Beat send.
            CAN_RESP[4] = (uint8_t)(GlobalTimer >> 24);
            CAN_RESP[5] = (uint8_t)(GlobalTimer >> 16);
            CAN_RESP[6] = (uint8_t)(GlobalTimer >> 8);
            CAN_RESP[7] = (uint8_t)(GlobalTimer);


            CANSendMSG(0x7DF,CAN_RESP);

            HeatbeatTrigger=GlobalTimer+HeartBeatTime;
        }
        IntCAN0Handler();
     }

}
