/*
 Author:    Jerry Black
 User:      Tyler Inkley

Inkley_PressureSensor

• Designed for use with an evaluation board equipped with pressure sensors
• Collects real-time pressure sensor data and temporarily stores it in a circular buffer in RAM
• Provides the option to store sensor data permanently in the Tiva chip's flash memory
• Supports communication with external modules over the CAN bus for data exchange and control
• Implements CAN commands for retrieving sensor data, managing flash memory (start, erase, read), and setting sample sizes
• Provides I2C communication for additional command flexibility, allowing interaction with sensor data and memory functions
• Sends periodic heartbeat messages over CAN to signal system activity and health
• The system is designed to handle overrun conditions in CAN communication and to ensure the integrity of stored data
• Includes SysTick-based interrupts for timed operations and ADC sampling for sensor readings
*/

//*****************************************************************************
//
// Firmware Libraries
//
//*****************************************************************************

// Standard C libraries
#include <stdbool.h>                // For boolean types
#include <stdint.h>                 // For fixed-width integer types

// Tiva C Series-specific hardware headers (memory mapping, interrupts, peripherals)
#include "inc/hw_memmap.h"          // Memory map definitions for the Tiva C Series
#include "inc/hw_ints.h"            // Interrupt definitions for the Tiva C Series
#include "inc/hw_can.h"             // CAN controller definitions for the Tiva C Series
#include "inc/hw_i2c.h"             // I2C hardware definitions

// Tiva C Series Driver Library headers (peripheral drivers and system control)
#include "driverlib/adc.h"          // ADC driver library (for analog-to-digital conversions)
#include "driverlib/can.h"          // CAN bus driver library
#include "driverlib/gpio.h"         // GPIO driver library
#include "driverlib/pin_map.h"      // Pin mapping definitions
#include "driverlib/interrupt.h"    // Interrupt controller driver library
#include "driverlib/sysctl.h"       // System control driver library (clock, power, etc.)
#include "driverlib/uart.h"         // UART driver library
#include "driverlib/i2c.h"          // I2C driver library
#include "driverlib/systick.h"      // SysTick timer driver library
#include "driverlib/flash.h"        // Flash memory driver library (for storing sensor data)

// Utility libraries for Tiva C Series
#include "utils/uartstdio.h"        // UART standard I/O utility functions

//*****************************************************************************
//
// Global Settings and Sensor Commands
//
//*****************************************************************************

uint32_t BuildVersion = 1002;       // Firmware version for this build

// CAN Bus Settings
#define CAN_ID      0x107           // CAN bus ID for the sensor module
#define CAN_BAUD    500000          // CAN bus baud rate set to 500 Kbps

//I2C Settings
#define NUM_I2C_DATA 8              // Number of data bytes expected for I2C communication
#define SLAVE_ADDRESS 0x3C          // I2C slave address for the sensor module

// Inkley Sensor Commands
/*
#define icmdReadVersion         01  // Command to read the version of the sensor
#define icmdReadData            02  // Command to read sensor data
#define icmdFlashStart          03  // Command to start recording data into flash memory
#define icmdFlashReadPos        04  // Command to read data from a specific position in flash memory
#define icmdFlashEraseFull      05  // Command to erase the entire flash memory
#define icmdFlashSetSampleSize  06  // Command to set the sample size for flash memory
#define icmdFlashStatus         07  // Command to get the status of the flash memory read (e.g., percentage complete)
#define icmdFlashGetData        08  // Get Flash sample from sensor module and store it locally
#define icmdFlashGenCSV         09  // Generate a CSV file from the flash data stored locally
*/

enum {
    icmdReadVersion = 0x01,         // Read sensor firmware version
    icmdReadData,                   // Retrieve current sensor data
    icmdFlashStart,                 // Start recording data into flash memory
    icmdFlashReadPos,               // Read data from a specific flash memory position
    icmdFlashEraseFull,             // Erase all data in flash memory
    icmdFlashSetSampleSize,         // Set the size of samples to store in flash
    icmdFlashStatus,                // Retrieve flash memory operation status
    icmdFlashGetData,               // Fetch raw data from flash memory
    icmdFlashGenCSV                 // Generate CSV-formatted output from flash data
};

//*****************************************************************************
//
// System Timing Settings: Defines system tick timing and timeouts
//
//*****************************************************************************

#define SYSTICK_TIMING   1000      // SysTick timer set to 1 millisecond intervals
#define ADC_ReadTimeOut 100        // Timeout for ADC reads
#define I2C_TimeOut 10000          // Timeout for I2C communication

uint32_t GlobalTimer = 0;          // Global timer for various time-based operations
#define HeartBeatTime 10000        // Heartbeat signal interval (10 seconds)
uint32_t HeatbeatTrigger = 0;      // Timer to track heartbeat signals

//*****************************************************************************
//
// Flash Memory Settings: Defines user space and sample size in flash memory
//
//*****************************************************************************

#define FlashUserSpace  0x30000    // Start address of user flash memory space
uint32_t FlashIndex = 0x40000;     // Current index in flash memory (initially set to end of space)
uint32_t FlashSampleSize = 0x10000;// Size of each sample stored in flash memory (64KB)

//*****************************************************************************
//
// I2C Command Handling: Variables to handle incoming I2C commands and timeouts
//
//*****************************************************************************

int TimeOutClock = 0;              // Timeout clock for general operations
int I2C_TimeOutClock = 0;          // Timeout clock for I2C communication

uint32_t I2C_RcvCommand = 0;       // Stores the last received I2C command
uint32_t I2C_RvcCommandParam = 0;  // Stores the parameter for the received I2C command

bool I2C_RcvNewCommand = false;    // Flag indicating whether a new I2C command has been received

//*****************************************************************************
//
// Circular Buffer Implementation: Provides functions for initializing, pushing,
// and popping data in a circular buffer; this buffer is used to temporarily store
// sensor data in RAM before writing to flash memory
//
//*****************************************************************************

// Structure for a circular buffer
typedef struct {
    uint32_t *bufdata;  // Pointer to buffer data (array)
    int head;           // Head index (where new data is written)
    int tail;           // Tail index (where data is read from)
    int maxlen;         // Maximum length of the buffer (capacity)
} circ_bbuf_t;

//*****************************************************************************
//
// Buffer Settings: Defines the buffer size and initializes the buffer array
//
//*****************************************************************************

#define SENSORBUFSIZE 1024                // Size of the circular buffer (1024 elements)
uint32_t SensorBufferData[SENSORBUFSIZE]; // Array to hold sensor data
circ_bbuf_t SensorBuf;                    // Circular buffer structure instance

//*****************************************************************************
//
// Init_circ_bbuf: Initializes the circular buffer structure; this sets the buffer
// data pointer, the maximum size, and initializes the head and tail pointers
//
// \param c - Pointer to the circular buffer structure
//
//*****************************************************************************

void Init_circ_bbuf(circ_bbuf_t *c)
{
    c->bufdata = SensorBufferData;   // Set the buffer data array
    c->maxlen = SENSORBUFSIZE;       // Set the buffer capacity
    c->head = 0;                     // Initialize the head pointer to the start
    c->tail = 0;                     // Initialize the tail pointer to the start
}

//*****************************************************************************
//
// circ_bbuf_push: Pushes new data into the circular buffer; if the buffer is full,
// the function returns an error (-1); otherwise, it updates the head pointer
//
// \param c - Pointer to the circular buffer structure
// \param data - The data to be pushed into the buffer
//
// \return 0 if successful, -1 if the buffer is full
//
//*****************************************************************************

int circ_bbuf_push(circ_bbuf_t *c, uint32_t data)
{
    int next;

    // Calculate the next position for the head; if head reaches the max length,
    // it wraps around to the start of the buffer
    next = c->head + 1;
    if (next >= c->maxlen)
        next = 0;

    // If the next position is the tail, the buffer is full (cannot push data)
    if (next == c->tail)
        return -1;

    // Store the data at the current head position, then move the head to the next position
    c->bufdata[c->head] = data;
    c->head = next;

    return 0;  // Return success
}

//*****************************************************************************
//
// circ_bbuf_pop: Pops data from the circular buffer; if the buffer is empty (head
// equals tail), the function returns an error (-1); otherwise, it updates the tail pointer
//
// \param c - Pointer to the circular buffer structure
// \param data - Pointer to store the popped data
//
// \return 0 if successful, -1 if the buffer is empty
//
//*****************************************************************************

int circ_bbuf_pop(circ_bbuf_t *c, uint32_t *data)
{
    int next;

    // If head equals tail, the buffer is empty (no data to pop)
    if (c->head == c->tail)
        return -1;

    // Calculate the next position for the tail; if tail reaches the max length,
    // it wraps around to the start of the buffer
    next = c->tail + 1;
    if (next >= c->maxlen)
        next = 0;

    // Retrieve the data from the current tail position, then move the tail to the next position
    *data = c->bufdata[c->tail];
    c->tail = next;

    return 0;  // Return success
}

//*****************************************************************************
//
// Global CAN and Utility Functions: Defines global CAN flags, the CAN message
// structure, and utility functions for timing, bit manipulation, and flag checking
//
//*****************************************************************************

//*****************************************************************************
//
// Global CAN Flags: Defines the status flags for CAN message handling
//
//*****************************************************************************

#define CAN_F_EMPTY     0  // Flag indicating the CAN buffer is empty
#define CAN_F_NEW       1  // Flag indicating a new CAN message has been received
#define CAN_F_OVERRUN   2  // Flag indicating a CAN buffer overrun (data loss)

//*****************************************************************************
//
// CAN Message Structure: Defines the structure of a CAN message, including its
// ID, status flags, and message data (up to 8 bytes)
//
//*****************************************************************************

typedef struct {
    char FLAGS;            // Status flags for the CAN message (e.g., new message, overrun)
    short ID;              // CAN message ID
    char MSG[8];           // CAN message data (up to 8 bytes)
} CAN_MSG_T;

CAN_MSG_T CAN_RECV;        // Global variable to store the received CAN message

//*****************************************************************************
//
// Utility Functions: Provides various utility functions for timing (delays) and
// bit manipulation (setting, clearing, toggling, and checking bits)
//
//*****************************************************************************

//*****************************************************************************
//
// DelayMS: Delays the execution for a specified number of milliseconds
//
// \param delay - The number of milliseconds to delay
//
//*****************************************************************************

void DelayMS(unsigned int delay)
{
    // SysCtlDelay provides a delay based on the system clock; the formula is used
    // to generate a delay in milliseconds
    SysCtlDelay((SysCtlClockGet() / 3 / 1000) * delay);
}

//*****************************************************************************
//
// bit_clear: Clears a specific bit in a given number
//
// \param number - The original number
// \param bit - The bit to clear
//
// \return The modified number with the specified bit cleared
//
//*****************************************************************************

uint32_t bit_clear(uint32_t number, uint32_t bit)
{
    // Use bitwise AND and NOT to clear the bit at the specified position
    return number & ~((uint32_t)1 << bit);
}

//*****************************************************************************
//
// bit_toggle: Toggles a specific bit in a given number
//
// \param number - The original number
// \param bit - The bit to toggle
//
// \return The modified number with the specified bit toggled
//
//*****************************************************************************

uint32_t bit_toggle(uint32_t number, uint32_t bit)
{
    // Use bitwise XOR to toggle the bit at the specified position
    return number ^ ((uint32_t)1 << bit);
}

//*****************************************************************************
//
// bit_set: Sets a specific bit in a given number
//
// \param number - The original number
// \param bit - The bit to set
//
// \return The modified number with the specified bit set
//
//*****************************************************************************

uint32_t bit_set(uint32_t number, uint32_t bit)
{
    // Use bitwise OR to set the bit at the specified position
    return number | ((uint32_t)1 << bit);
}

//*****************************************************************************
//
// bit_check: Checks if a specific bit in a given number is set
//
// \param number - The number to check
// \param bit - The bit to check
//
// \return True if the bit is set, false otherwise
//
//*****************************************************************************
bool bit_check(uint32_t number, uint32_t bit)
{
    // Shift the bit to the right and check if it is set
    return (number >> bit) & (uint32_t)1;
}

//*****************************************************************************
//
// SysTick Interrupt Handler: Handles system tick interrupts that occur
// periodically (every 1 millisecond); it triggers ADC reads, stores sensor
// data in the circular buffer, and optionally dumps the data to flash memory
//
//*****************************************************************************

void SysTickIntHandler(void)
{
    uint32_t pui32ADC0Value[1];  // Buffer to store ADC result

    // Trigger an ADC read (ADC0, sequencer 3); SysTick is set to trigger every 1ms
    ADCProcessorTrigger(ADC0_BASE, 3);
    TimeOutClock = 0;

    // Wait for the ADC conversion to complete or timeout
    while (!ADCIntStatus(ADC0_BASE, 3, false))
    {
        if (TimeOutClock++ > ADC_ReadTimeOut)
        {
            // If timeout occurs, clear the interrupt and return
            ADCIntClear(ADC0_BASE, 3);
            return;
        }
    }

    // Clear the ADC interrupt once data is ready
    ADCIntClear(ADC0_BASE, 3);

    // Retrieve the ADC data (from sequencer 3) and store it in the buffer
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);

    // Push the ADC value into the circular buffer for real-time data processing
    circ_bbuf_push(&SensorBuf, pui32ADC0Value[0]);

    // If FlashIndex points to valid user flash space, store ADC data into flash memory
    if (FlashIndex < FlashUserSpace + FlashSampleSize)
    {
        if((FlashIndex & 0x7ff)==0x400)
               {
                   FlashErase(FlashIndex);  //Erase 0x400 black at a time
               }

        // Write the ADC value to flash memory (increment FlashIndex after writing 4 bytes)
        FlashProgram(&pui32ADC0Value[0], FlashIndex += 4, 4);
    }

    // Increment the global timer for time-based operations
    GlobalTimer++;
}

//*****************************************************************************
//
// I2C0 Slave Interrupt Handler: Handles interrupts for I2C0 data communication
// in slave mode; tt reads incoming data from the master device and sets a flag
// to indicate a new command has been received
//
//*****************************************************************************

void I2C0SlaveIntHandler(void)
{
    // Clear the I2C0 interrupt flag to acknowledge and reset the interrupt
    I2CSlaveIntClear(I2C0_BASE);

    // Read the command data from the I2C master device
    I2C_RcvCommand = I2CSlaveDataGet(I2C0_BASE);

    // Read the command parameter from the I2C master device
    I2C_RvcCommandParam = I2CSlaveDataGet(I2C0_BASE);

    // Set a flag to indicate that a new I2C command has been received
    I2C_RcvNewCommand = true;
}

//*****************************************************************************
//
// ADC Initialization: Configures the ADC0 peripheral for analog data collection;
// this function sets up the ADC to use GPIO pins and configures the sequence
// for sampling
//
//*****************************************************************************

void Init_ADC()
{
    // Enable the ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable GPIO Port E for the ADC pins (PE2 and PE3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure the GPIO pins for ADC input (PE2, PE3)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // Configure ADC sequencer 3 to be triggered by the processor
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    // Configure the steps in the ADC sequence; this configures sequencer step 0 to
    // read channel 0 (ADC_CTL_CH0), generate an interrupt, and end the sequence
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_D | ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // Enable ADC sequencer 3 for sampling
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear any pending ADC interrupts to ensure a clean start
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
//
// SysTick Initialization: Configures the system tick timer (SysTick) to generate
// an interrupt every 1 millisecond; this is used for time-based tasks
//
//*****************************************************************************

void Init_Systick (void)
{
    // Set the SysTick period for 1ms based on the system clock
    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_TIMING);    // Set SYSTICK to interrupt every 1ms

    // Enable the SysTick Interrupt to handle periodic tasks
    SysTickIntEnable();

    // Enable the SysTick Timer to start the timer operation
    SysTickEnable();
}

//*****************************************************************************
//
// I2C Initialization: Configures the I2C0 peripheral for communication in both
// master and slave modes; this function sets up the GPIO pins for I2C, configures
// interrupts, and enables the I2C master and slave modules
//
//*****************************************************************************

void Init_I2C(void)
{
    // Enable the I2C0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Enable GPIO Port B for I2C0 pins (PB2, PB3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pins for I2C0 (PB2 = SCL, PB3 = SDA)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Configure GPIO pins for I2C operation (open-drain, with weak pull-ups)
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Enable I2C0 interrupts on the processor
    IntEnable(INT_I2C0);

    // Enable I2C0 slave interrupts, specifically when the slave receives data
    I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);

    // Initialize the I2C0 master module using the system clock, with a data rate of 100kbps
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    // Enable the I2C0 slave module
    I2CSlaveEnable(I2C0_BASE);

    // Set the I2C0 slave address for communication
    I2CSlaveInit(I2C0_BASE, SLAVE_ADDRESS);
}

//*****************************************************************************
//
// I2C_SendData: Sends a 32-bit word of data over the I2C bus in burst mode;
// the function sends the data byte-by-byte, handling each step of the I2C
// transaction (start, continue, and finish)
//
// \param SData - The 32-bit data to send
//
//*****************************************************************************

void I2C_SendData(uint32_t SData)
{
    // Set the I2C slave address (with the write flag)
    I2CMasterSlaveAddrSet(I2C0_BASE, SLAVE_ADDRESS, false);

    // Send the most significant byte (MSB) of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 24));

    // Start the I2C burst transmission
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until the master is ready or timeout occurs
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the second byte of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 16));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    // Wait until the master is ready or timeout occurs
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the third byte of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData >> 8));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

    // Wait until the master is ready or timeout occurs
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }

    // Send the least significant byte (LSB) of the 32-bit data
    I2CMasterDataPut(I2C0_BASE, (uint8_t)(SData));
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait until the master is ready or timeout occurs
    I2C_TimeOutClock = I2C_TimeOut;
    while (I2CMasterBusy(I2C0_BASE))
    {
        if (--I2C_TimeOutClock == 0) break;
    }
}

//*****************************************************************************
//
// CANPollCheck: Polls the CAN bus for a specific message ID and checks if new
// data is available; if new data is present, it reads the CAN message and returns
// the number of messages received
//
// \param candata - Pointer to the buffer where the received CAN data will be stored
// \param MsgID - The CAN message ID to check for
// \param Response - (Not used in the function, but could be used for handling specific responses)
//
// \return The number of messages received with the specified message ID
//
//*****************************************************************************

uint32_t CANPollCheck(unsigned char *candata, int MsgID, unsigned char Response)
{
    int rValue = 0;                         // Counter for the number of received messages
    uint32_t ulNewData;                     // Holds the status of new CAN data

    tCANMsgObject sMsgObjectRx;             // CAN message object for receiving data

    // Set up the CAN message object to expect 8 bytes of data
    sMsgObjectRx.ui32MsgLen = 8;
    sMsgObjectRx.pui8MsgData = candata;     // Point the message data buffer to 'candata'

    // Get the status of new data available on the CAN bus
    ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);

    // Loop while there is new data for the specified message ID
    while (ulNewData & (1 << (MsgID - 1)))
    {
        // Read the message from the specified message object (MsgID) and store it in sMsgObjectRx
        // 'true' indicates that the message should be cleared from the message object after reading
        CANMessageGet(CAN0_BASE, MsgID, &sMsgObjectRx, true);
        rValue++;                           // Increment the counter for each received message

        // Check again if there is more new data for the specified message ID
        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    }

    return rValue;                          // Return the number of messages received
}

//*****************************************************************************
//
// CAN0 Interrupt Handler: Handles interrupts on the CAN0 interface; it checks
// for new messages on the CAN bus and processes any messages received for the
// specified CAN ID; also sets flags to handle message overrun conditions
//
//*****************************************************************************

void IntCAN0Handler(void)
{
    uint32_t ulStatus, ulNewData;           // Variables to store interrupt and new data status
    tCANMsgObject tempCANMsgObject;         // Temporary CAN message object
    uint8_t CANMsg[8];                      // Buffer to hold received CAN data (8 bytes)
    unsigned char CANSlot = 1;              // Slot in which the CAN message will be stored

    // Set up the temporary CAN message object to receive 8 bytes of data
    tempCANMsgObject.pui8MsgData = CANMsg;
    tempCANMsgObject.ui32MsgLen = 8;

    // Get the cause of the interrupt and clear it
    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);
    CANIntClear(CAN0_BASE, ulStatus);

    // If the interrupt is not a controller status interrupt, handle it
    if (ulStatus != CAN_INT_INTID_STATUS)
    {
        // Get the controller status
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        // Check if new data is available on the CAN bus
        ulNewData = CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);

        if (ulNewData)
        {
            // Check if there is new data for the specified CAN slot
            if (ulNewData & (1 << (CANSlot - 1)))
            {
                // Get the CAN message and clear the pending flag
                CANMessageGet(CAN0_BASE, CANSlot, &tempCANMsgObject, true);

                // If the message ID matches our CAN_ID, process it
                if (tempCANMsgObject.ui32MsgID == CAN_ID)
                {
                    CAN_RECV.ID = tempCANMsgObject.ui32MsgID;   // Store the message ID
                    memcpy(CAN_RECV.MSG, CANMsg, 8);            // Copy the message data to the CAN_RECV buffer

                    // Check if there is already a message in the buffer (overrun condition)
                    if (bit_check(CAN_RECV.FLAGS, CAN_F_NEW))
                    {
                        CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS, CAN_F_OVERRUN);    // Set the overrun flag
                    }
                    CAN_RECV.FLAGS = bit_set(CAN_RECV.FLAGS, CAN_F_NEW);            // Set the flag indicating a new message
                }
            }
        }
    }
}

//*****************************************************************************
//
// CANSendINT: Sends a 4-byte integer over the CAN bus using the specified CAN ID;
// this function waits for the transmission to complete and returns an error if
// the transmission times out
//
// \param CANID - The ID of the CAN message to send
// \param pui8MsgData - The 4-byte integer data to send
//
// \return 0 if successful, or 0xffffffff if a timeout occurred
//
//*****************************************************************************

uint32_t CANSendINT(unsigned long CANID, uint32_t pui8MsgData)
{
    unsigned long TimeOut = 0;                          // Variable to track timeout conditions
    tCANMsgObject sCANMessage;                          // CAN message object for sending data

    // Set up the CAN message object with the given CAN ID and 4-byte data
    sCANMessage.ui32MsgID = CANID;                      // Set the message ID
    sCANMessage.ui32Flags = 0;                          // No special flags are used
    sCANMessage.ui32MsgLen = 4;                         // The message length is 4 bytes
    sCANMessage.pui8MsgData = (uint8_t *)&pui8MsgData;  // Set the data pointer

    // Send the message using message object 32 (arbitrary choice)
    CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);

    // Wait for the CAN message to be transmitted
    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) != 0)
    {
        TimeOut++;
        SysCtlDelay(SysCtlClockGet() / 30000);          // Delay to avoid tight looping

        // If the transmission times out, return an error
        if (TimeOut > 0x0001000)
        {
            return 0xffffffff;                          // Return error code for timeout
        }
    }

    return 0;                                           // Return success
}

//*****************************************************************************
//
// CANSendMSG: Sends an 8-byte message over the CAN bus using the specified CAN ID;
// this function waits for the transmission to complete and returns an error if
// the transmission times out
//
// \param CANID - The ID of the CAN message to send
// \param pui8MsgData - Pointer to the 8-byte data to send
//
// \return 0 if successful, or 0xffffffff if a timeout occurred
//
//*****************************************************************************

uint32_t CANSendMSG(unsigned long CANID, uint8_t *pui8MsgData)
{
    unsigned long TimeOut = 0;                  // Variable to track timeout conditions
    tCANMsgObject sCANMessage;                  // CAN message object for sending data

    // Set up the CAN message object with the given CAN ID and 8-byte data
    sCANMessage.ui32MsgID = CANID;              // Set the message ID
    sCANMessage.ui32Flags = 0;                  // No special flags are used
    sCANMessage.ui32MsgLen = 8;                 // The message length is 8 bytes
    sCANMessage.pui8MsgData = pui8MsgData;      // Set the data pointer

    // Send the message using message object 32 (arbitrary choice)
    CANMessageSet(CAN0_BASE, 32, &sCANMessage, MSG_OBJ_TYPE_TX);

    // Wait for the CAN message to be transmitted
    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) != 0)
    {
        TimeOut++;
        SysCtlDelay(SysCtlClockGet() / 30000);  // Delay to avoid tight looping

        // If the transmission times out, return an error
        if (TimeOut > 0x0001000)
        {
            return 0xffffffff;                  // Return error code for timeout
        }
    }

    return 0;                                   // Return success
}

//*****************************************************************************
//
// CANListnerEX: Sets up a CAN message object for receiving data; this function
// configures a message object to use an extended ID filter and allocates space
// for receiving 8 bytes of data
//
// \param MsgID - The message ID to configure for receiving
//
//*****************************************************************************

void CANListnerEX(int MsgID)
{
    tCANMsgObject sMsgObjectRx;                                             // CAN message object for receiving data

    // Configure the message object to receive messages with extended IDs
    sMsgObjectRx.ui32MsgID = 0;                                             // Accept all message IDs (no specific ID)
    sMsgObjectRx.ui32MsgIDMask = 0;                                         // Mask for extended ID filtering
    sMsgObjectRx.ui32Flags = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_EXTENDED_ID;   // Use ID filter and extended ID
    sMsgObjectRx.ui32MsgLen = 8;                                            // Expect 8 bytes of data
    sMsgObjectRx.pui8MsgData = (unsigned char *)0xffffffff;                 // Set dummy data pointer

    // Configure the CAN message object for receiving messages
    CANMessageSet(CAN0_BASE, MsgID, &sMsgObjectRx, MSG_OBJ_TYPE_RX);
}

//*****************************************************************************
//
// Init_CAN: Initializes the CAN0 peripheral for communication; this function sets
// up the CAN pins, configures the baud rate, enables interrupts, and prepares the
// CAN bus for operation
//
// \param Baud - The baud rate for CAN communication
//
//*****************************************************************************

void Init_CAN(uint32_t Baud)
{
    // Enable the GPIO port B peripheral (for CAN RX and TX pins)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for CAN0 functions on port B4 (CAN0RX) and B5 (CAN0TX)
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);

    // Configure the GPIO pins for CAN operation
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Enable the CAN0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

    // Initialize the CAN0 controller
    CANInit(CAN0_BASE);

    // Set the baud rate for CAN communication
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), Baud);

    // Enable the desired CAN interrupts (master, error, and status interrupts)
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Enable CAN0 interrupts on the processor (NVIC)
    IntEnable(INT_CAN0);

    // Enable the CAN0 controller
    CANEnable(CAN0_BASE);

    // Enable automatic retries for CAN messages that fail to transmit
    CANRetrySet(CAN0_BASE, true);

    // Small delay to allow CAN initialization to complete
    DelayMS(10);

    // Set up a CAN listener on mailbox 1 to receive broadcast messages
    CANListnerEX(1);

    // Small delay to ensure CAN listener is fully initialized
    DelayMS(10);
}

//*****************************************************************************
//
// Main Function: Main loop of the Inkley_PressureSensor program; it handles CAN
// communication, processes sensor data, and manages flash memory for storing sensor
// readings
//
//*****************************************************************************

int main(void)
{
    uint32_t BufDataVar = 0;            // Variable to store buffer data (from the sensor)
    uint32_t CANID_tmp = 0;             // Temporary variable for the received CAN ID
    uint32_t CANVAL_tmp = 0;            // Temporary variable for the received CAN value
    uint32_t lop = 0;                   // Loop iterator variable
    uint8_t CAN_RESP[8];                // Array for storing CAN response data
    uint8_t CAN_CMD_REQUEST = 0;        // Stores the command requested via CAN

    // Set the system clock to 40MHz (SYSCTL_SYSDIV_10 = divide by 10, 400MHz PLL)
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize system peripherals (ADC, SysTick, I2C, Circular Buffer, CAN)
    Init_ADC();
    Init_Systick();
    Init_I2C();
    Init_circ_bbuf(&SensorBuf);
    Init_CAN(CAN_BAUD);

    // Erase the flash memory area that will be used for logging sensor data
    FlashErase(FlashUserSpace);

    // Turn on CAN bus listener using mailbox 1
    CANListnerEX(1);

    //*************************************************************************
    //
    // Main program loop: Processes incoming CAN messages, handles I2C commands,
    // manages flash memory, and sends periodic heartbeat messages
    //
    //*************************************************************************
    while (1)
    {
        // Check if a new CAN message has been received
        if (bit_check(CAN_RECV.FLAGS, CAN_F_NEW))
        {
            // Clear the new message flag
            CAN_RECV.FLAGS = bit_clear(CAN_RECV.FLAGS, CAN_F_NEW);

            // Extract the CAN ID and value from the received message
            CANID_tmp = (CAN_RECV.MSG[1] << 8) + CAN_RECV.MSG[2];
            CANVAL_tmp = (CAN_RECV.MSG[3] << 24) + (CAN_RECV.MSG[4] << 16) + (CAN_RECV.MSG[5] << 8) + CAN_RECV.MSG[6];

            // Prepare the response structure with basic info
            CAN_RESP[0] = 0x08;             // Message length
            CAN_RESP[1] = (CAN_ID >> 8) & 0xFF;
            CAN_RESP[2] = CAN_ID & 0xFF;
            CAN_RESP[3] = CAN_RECV.MSG[0];  // Command sent
            CAN_RESP[4] = 0x00;
            CAN_RESP[5] = 0x00;
            CAN_RESP[6] = 0x00;
            CAN_RESP[7] = 0x00;

            // Process the CAN command request
            CAN_CMD_REQUEST = CAN_RECV.MSG[0];
            switch (CAN_CMD_REQUEST)
            {
                case icmdReadVersion:           // Read Version
                    // Send firmware version as the response
                    CAN_RESP[4] = (uint8_t)(BuildVersion >> 24);
                    CAN_RESP[5] = (uint8_t)(BuildVersion >> 16);
                    CAN_RESP[6] = (uint8_t)(BuildVersion >> 8);
                    CAN_RESP[7] = (uint8_t)(BuildVersion);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdReadData:              // Read Sensor Data
                    // Retrieve the latest sensor data from the circular buffer
                    circ_bbuf_pop(&SensorBuf, &BufDataVar);
                    CAN_RESP[4] = (uint8_t)(BufDataVar >> 24);
                    CAN_RESP[5] = (uint8_t)(BufDataVar >> 16);
                    CAN_RESP[6] = (uint8_t)(BufDataVar >> 8);
                    CAN_RESP[7] = (uint8_t)(BufDataVar);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashReadPos:          // Read Flash Memory Position
                    // Return the current flash index
                    CAN_RESP[4] = (uint8_t)(FlashIndex >> 24);
                    CAN_RESP[5] = (uint8_t)(FlashIndex >> 16);
                    CAN_RESP[6] = (uint8_t)(FlashIndex >> 8);
                    CAN_RESP[7] = (uint8_t)(FlashIndex);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashEraseFull:        // Erase Flash Memory
                    // Erase the user flash space and return its address
                    FlashErase(FlashUserSpace);
                    CAN_RESP[4] = (uint8_t)(FlashUserSpace >> 24);
                    CAN_RESP[5] = (uint8_t)(FlashUserSpace >> 16);
                    CAN_RESP[6] = (uint8_t)(FlashUserSpace >> 8);
                    CAN_RESP[7] = (uint8_t)(FlashUserSpace);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashStart:            // Start Flash Recording
                    // Set the flash index to the user flash space and return the index
                    FlashIndex = FlashUserSpace;
                    CAN_RESP[4] = (uint8_t)(FlashIndex >> 24);
                    CAN_RESP[5] = (uint8_t)(FlashIndex >> 16);
                    CAN_RESP[6] = (uint8_t)(FlashIndex >> 8);
                    CAN_RESP[7] = (uint8_t)(FlashIndex);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashSetSampleSize:    // Set Flash Sample Size
                    // Set the sample size and return it
                    if (CANVAL_tmp == 0 || CANVAL_tmp > 0x10000) CANVAL_tmp = 0x10000;
                    FlashSampleSize = CANVAL_tmp;
                    CAN_RESP[4] = (uint8_t)(FlashSampleSize >> 24);
                    CAN_RESP[5] = (uint8_t)(FlashSampleSize >> 16);
                    CAN_RESP[6] = (uint8_t)(FlashSampleSize >> 8);
                    CAN_RESP[7] = (uint8_t)(FlashSampleSize);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashStatus:           // Get Flash Status
                    // Calculate the percentage of flash space used
                    CANVAL_tmp = ((FlashIndex - FlashUserSpace) / FlashSampleSize) * 100;
                    CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                    CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                    CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                    CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                    CANSendMSG(CANID_tmp, CAN_RESP);
                    break;

                case icmdFlashGetData:          // Fetch raw data from flash memory
                    // Send size of sample with first response
                    CANVAL_tmp = FlashSampleSize;
                    CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                    CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                    CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                    CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                    CANSendMSG(CANID_tmp,CAN_RESP);
                    for(lop=FlashUserSpace;lop <= FlashUserSpace+FlashSampleSize;lop+=4)
                    {
                           CANVAL_tmp = *((uint32_t *)lop);
                           CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                           CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                           CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                           CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                           CANSendMSG(CANID_tmp,CAN_RESP);
                    }
                   CANVAL_tmp = 0x0;    // Dend zero to end stream
                   CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                   CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                   CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                   CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                   CANSendMSG(CANID_tmp,CAN_RESP);
                   break;
            }

            // Reset the new message flag and set the heartbeat timer
            bit_clear(CAN_RECV.FLAGS, CAN_F_NEW);
            HeatbeatTrigger = GlobalTimer + HeartBeatTime;
        }

        // Check for CAN overrun condition (if new data arrived before the previous was processed)
        if (bit_check(CAN_RECV.FLAGS, CAN_F_OVERRUN))
        {
            // Clear the overrun flag
            bit_clear(CAN_RECV.FLAGS, CAN_F_OVERRUN);
        }

        // Process any I2C commands received
        if (I2C_RcvNewCommand)
        {
            switch (I2C_RcvCommand)
            {
                case icmdReadVersion:       // Read Version
                    I2C_SendData(BuildVersion);
                    break;

                case icmdReadData:          // Read Sensor Data
                    circ_bbuf_pop(&SensorBuf, &BufDataVar);
                    I2C_SendData(BufDataVar);
                    break;

                case icmdFlashReadPos:      // Read Flash Memory Position
                    I2C_SendData(FlashIndex);
                    break;

                case icmdFlashEraseFull:    // Erase Flash Memory
                    FlashErase(FlashUserSpace);
                    break;

                case icmdFlashStart:        // Start Flash Recording
                    FlashIndex = FlashUserSpace;
                    break;
            }
            I2C_RcvNewCommand = false;      // Reset the I2C new command flag
        }

        // Check if it's time to send a heartbeat message (every 10 seconds)
        if (GlobalTimer > HeatbeatTrigger)
        {
            // Prepare and send a heartbeat message with the global timer value
            CAN_RESP[0] = 0x08;  // Message length
            CAN_RESP[1] = (CAN_ID >> 8) & 0xFF;
            CAN_RESP[2] = CAN_ID & 0xFF;
            CAN_RESP[3] = 0x7F;  // Heartbeat command
            CAN_RESP[4] = (uint8_t)(GlobalTimer >> 24);
            CAN_RESP[5] = (uint8_t)(GlobalTimer >> 16);
            CAN_RESP[6] = (uint8_t)(GlobalTimer >> 8);
            CAN_RESP[7] = (uint8_t)(GlobalTimer);

                     CANSendMSG(0x7DF, CAN_RESP);  // Send heartbeat message to broadcast address

                     // Reset the heartbeat timer
                     HeatbeatTrigger = GlobalTimer + HeartBeatTime;
                 }

                 // Call the CAN interrupt handler to process incoming messages
                 IntCAN0Handler();
             }
         }
