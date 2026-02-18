/******************************************************************************
 * File:        main.c
 * Project:     Inkley_PressureSensor (TM4C / Tiva C)
 * Author:      Tyler Inkley
 *
 * Description:
 *   Firmware for a two-channel differential pressure sensor evaluation module.
 *   Samples two ADC channels at a fixed rate (nominal 1000 Hz) and supports:
 *     - Real-time streaming of packed Pressure1/Pressure2 ADC counts over CAN
 *     - CAN command/response control interface (version, start/stop, status, etc.)
 *     - Periodic heartbeat messages for basic “alive” indication when idle
 *
 * Data Output (Realtime Broadcast):
 *   CAN ID: CAN_BC_ID (0x7DF), 8-byte payload
 *     [0] = frame type (0x05)
 *     [1] = packed sensor id (0x12)
 *     [2..3] = Pressure1 (uint16, big-endian)  // ADC0 SS2 step0 (PE3 / CH0)
 *     [4..5] = Pressure2 (uint16, big-endian)  // ADC0 SS2 step1 (PE2 / CH1)
 *     [6..7] = reserved (0)
 *
 * Command Interface (Unicast):
 *   RX CAN ID: CAN_ID (0x107)
 *   Incoming payload (8 bytes):
 *     [0] = command
 *     [1..2] = response CAN ID (destination for ACK/response)
 *     [3..6] = uint32 value (argument), big-endian
 *     [7] = reserved
 *   Responses:
 *     8-byte payload: [0]=len, [1..2]=src id, [3]=cmd echo, [4..7]=uint32 value
 *
 * Notes:
 *   - CAN receive buffer is ISR-written and main-loop read; volatile-safe copy
 *     helpers are used to avoid casting away volatile.
 *   - CANSendMSG_Obj() allows caller-selected TX mailbox usage.
 *
 * Revision / Build:
 *   BuildVersion: 1003
 ******************************************************************************/

//*****************************************************************************
//
// Firmware Libraries
//
//*****************************************************************************

// Standard C libraries
#include <stdbool.h>                // For boolean types
#include <stdint.h>                 // For fixed-width integer types
#include <string.h>                 // For memcpy(), memset(), and other standard string/memory utilities

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
// Volatile-safe memory helpers
//
// The CAN receive structure (CAN_RECV) is declared volatile because it is
// written inside an interrupt service routine (ISR) and read in main().
// Standard memcpy()/memset() take non-volatile pointers (void*), which triggers
// compiler warnings when used with volatile buffers.
//
// These helpers perform simple byte-wise operations so we can safely initialize
// or copy volatile buffers without casting away volatile.
//
//*****************************************************************************
static void vmemset(volatile uint8_t *dst, uint8_t val, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
    {
        dst[i] = val;
    }
}

static void vmemcpy(volatile uint8_t *dst, const uint8_t *src, uint32_t n)
{
    uint32_t i;
    for (i = 0; i < n; i++)
    {
        dst[i] = src[i];
    }
}

//*****************************************************************************
//
// Global Settings and Sensor Commands
//
//*****************************************************************************

uint32_t BuildVersion = 1003;       // Firmware version for this build

// Time Out
uint32_t TimeOutCounter = 0;

#define PressureSensor1     0x01
#define PressureSensor2     0x02
#define TemperatureSensor1  0x03
#define TemperatureSensor2  0x04


// CAN Bus Settings
#define CAN_ID      0x107           // CAN bus ID for the sensor module
#define CAN_BC_ID   0x7DF           //CAN ID for sensor data broadcast.
#define CAN_BAUD    500000          // CAN bus baud rate set to 500 Kbps
uint8_t CAN_BUF[8];

// Inkley Sensor Commands
#define icmdReadVersion         0x01  // Request firmware version (returns BuildVersion)
#define icmdStreamRealtime      0x02  // Start real-time streaming (no RAM buffering)
#define icmdStreamBuffered      0x03  // Start streaming from RAM/flash buffer (if implemented)
#define icmdStopStreaming       0x04  // Stop all streaming modes
#define icmdStreamingStatus     0x05  // Query current streaming mode / status
#define icmdStreamBufferSet     0x06  // Set stream buffer size (bytes/samples) for buffered mode

//*****************************************************************************
//
// System Timing Settings: Defines system tick timing and timeouts
//
//*****************************************************************************

#define SYSTICK_TIMING   1000      // SysTick timer set to 1 millisecond intervals
#define ADC_ReadTimeOut  100        // Timeout for ADC reads

uint32_t GlobalTimer = 0;          // Global timer for various time-based operations
#define HeartBeatTime 10000        // Heart beat signal interval (10 seconds)
uint32_t HeatbeatTrigger = 0;      // Timer to track heart beat signals

//*****************************************************************************
//
// Stream Buffer Settings
//
//*****************************************************************************

uint8_t Streaming = 0;
uint32_t StreamBufferSize = 8192;
uint32_t StreamBufferIndex = 0;

    #define smStopped      0x00
    #define smRealTime     0x01
    #define smBuffered     0x02
uint32_t StreamingMode=smStopped;

//*****************************************************************************
//
// Buffer Settings: Defines the buffer size and initializes the buffer array
//
//*****************************************************************************

#define SENSORBUFSIZE 1024                // Size of the circular buffer (1024 elements)
uint32_t SensorBufferData[SENSORBUFSIZE]; // Array to hold sensor data
//circ_bbuf_t SensorBuf;                    // Circular buffer structure instance

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
    uint8_t  FLAGS;             // Bitfield: CAN_F_NEW, CAN_F_OVERRUN, etc.
    uint32_t ID;                // Received CAN message ID (11-bit standard fits here)
    uint8_t  MSG[8];            // Received CAN payload (8 bytes)
} CAN_MSG_T;

volatile CAN_MSG_T CAN_RECV;    // Updated in CAN ISR, read in main loop

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
uint32_t CANSendMSG(unsigned long CANID, uint8_t *pui8MsgData);
uint32_t CANSendMSG_Obj(unsigned long CANID, uint8_t *pui8MsgData, uint32_t txObj);
void CANListenerStd(int MsgObj);
void IntCAN0Handler(void);   // CAN0 interrupt service routine (name must match startup vector)

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
    uint32_t adcVals[2];

    ADCProcessorTrigger(ADC0_BASE, 2);
    TimeOutCounter = 0;

    while(!ADCIntStatus(ADC0_BASE, 2, false))
    {
        if(TimeOutCounter++ > ADC_ReadTimeOut)
        {
            ADCIntClear(ADC0_BASE, 2);
            return;
        }
    }

    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, adcVals);  // [0]=CH0(PE3), [1]=CH1(PE2)

    if (StreamingMode == smRealTime)
    {
        uint16_t p1 = (uint16_t)(adcVals[0] & 0x0FFF);
        uint16_t p2 = (uint16_t)(adcVals[1] & 0x0FFF);

        CAN_BUF[0] = 0x05;   // frame type
        CAN_BUF[1] = 0x12;   // packed P1+P2 id
        CAN_BUF[2] = (uint8_t)(p1 >> 8);
        CAN_BUF[3] = (uint8_t)(p1);
        CAN_BUF[4] = (uint8_t)(p2 >> 8);
        CAN_BUF[5] = (uint8_t)(p2);
        CAN_BUF[6] = 0x00;
        CAN_BUF[7] = 0x00;

        CANSendMSG_Obj(CAN_BC_ID, CAN_BUF, 31);
    }

    GlobalTimer++;
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // PE3 = AIN0 (CH0) -> Pressure1
    // PE2 = AIN1 (CH1) -> Pressure2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // Use Sequencer 2 (SS2) because SS3 only supports 1 sample
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);

    // Step 0: CH0
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);

    // Step 1: CH1 + interrupt + end
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
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
    uint32_t ulStatus;                    // Variable to store interrupt cause / message object number
    tCANMsgObject tempCANMsgObject;       // Temporary CAN message object
    uint8_t CANMsg[8];                    // Buffer to hold received CAN data (8 bytes)
    uint32_t MsgObj;                      // Message object number that generated the interrupt

    // Set up the temporary CAN message object to receive 8 bytes of data
    tempCANMsgObject.pui8MsgData = CANMsg;
    tempCANMsgObject.ui32MsgLen  = 8;

    // Get the cause of the interrupt
    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    // If this is a controller status interrupt, read/clear status and return
    if (ulStatus == CAN_INT_INTID_STATUS)
    {
        // Get the controller status (clears error/status sources internally)
        (void)CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        // Clear the interrupt in the controller
        CANIntClear(CAN0_BASE, ulStatus);
        return;
    }

    // Otherwise, ulStatus is the message object number that caused the interrupt
    MsgObj = ulStatus;

    // Clear the interrupt for this message object
    CANIntClear(CAN0_BASE, MsgObj);

    // Get the CAN message and clear NEWDAT for this message object ('true' clears it)
    CANMessageGet(CAN0_BASE, MsgObj, &tempCANMsgObject, true);

    // If the message ID matches our CAN_ID, process it
    if (tempCANMsgObject.ui32MsgID == CAN_ID)
    {
        CAN_RECV.ID = tempCANMsgObject.ui32MsgID;                       // Store the message ID (no cast)

        // Copy the 8-byte payload into the volatile receive buffer.
        //
        // CAN_RECV.MSG is volatile because it is shared between the CAN ISR (writer)
        // and main() (reader). Standard memcpy() takes a non-volatile void* destination,
        // which triggers a compiler warning if we pass a volatile buffer.
        //
        // vmemcpy() performs a simple byte-wise copy without casting away volatile.
        vmemcpy(CAN_RECV.MSG, CANMsg, 8);

        // Check if there is already a message in the buffer (overrun condition)
        if (bit_check(CAN_RECV.FLAGS, CAN_F_NEW))
        {
            CAN_RECV.FLAGS = (uint8_t)bit_set(CAN_RECV.FLAGS, CAN_F_OVERRUN);    // Set the overrun flag
        }
        CAN_RECV.FLAGS = (uint8_t)bit_set(CAN_RECV.FLAGS, CAN_F_NEW);            // Set the flag indicating a new message
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
// CANSendMSG_Obj: Sends an 8-byte message over the CAN bus using the specified
// CAN ID and a caller-selected transmit message object (mailbox).
//
// This is the same as CANSendMSG(), but allows the caller to choose which TX
// message object number to use (e.g., to avoid collisions with other TX paths
// or to dedicate separate mailboxes for different message types).
//
// \param CANID       - The CAN message ID to send
// \param pui8MsgData - Pointer to the 8-byte data to send
// \param txObj       - CAN message object number to use for transmission (1..32)
//
// \return 0 if successful, or 0xffffffff if a timeout occurred
//
//*****************************************************************************

uint32_t CANSendMSG_Obj(unsigned long CANID, uint8_t *pui8MsgData, uint32_t txObj)
{
    unsigned long TimeOut = 0;
    tCANMsgObject sCANMessage;

    sCANMessage.ui32MsgID = CANID;
    sCANMessage.ui32Flags = 0;
    sCANMessage.ui32MsgLen = 8;
    sCANMessage.pui8MsgData = pui8MsgData;

    CANMessageSet(CAN0_BASE, txObj, &sCANMessage, MSG_OBJ_TYPE_TX);

    while (CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST) != 0)
    {
        TimeOut++;
        SysCtlDelay(SysCtlClockGet() / 30000);
        if (TimeOut > 0x0001000) return 0xffffffff;
    }
    return 0;
}


//*****************************************************************************
//
// CANListenerStd: Configures a CAN message object (mailbox) to receive standard
// 11-bit CAN frames that match this module's CAN_ID.
//
// This function sets up an RX mailbox with:
//  - An 11-bit ID filter (ui32MsgID + ui32MsgIDMask)
//  - RX interrupt enabled so the CAN ISR can be triggered on reception
//  - An expected payload length of 8 bytes
//
// \param MsgObj - The CAN message object number (mailbox index) to configure
//                 for reception (e.g., 1..32 on TM4C CAN controller).
//
// \note This config is intended for STANDARD (11-bit) CAN IDs. If your sender
//       uses EXTENDED (29-bit) IDs, you must configure the mailbox differently.
//
//*****************************************************************************

void CANListenerStd(int MsgObj)
{
    tCANMsgObject rx;

    rx.ui32MsgID     = CAN_ID;                                          // Accept only frames with ID == CAN_ID (0x107)
    rx.ui32MsgIDMask = 0x7FF;                                           // 11-bit mask (all bits must match)
    rx.ui32Flags     = MSG_OBJ_USE_ID_FILTER | MSG_OBJ_RX_INT_ENABLE;   // Filter + RX interrupt
    rx.ui32MsgLen    = 8;                                               // Expect 8 data bytes
    rx.pui8MsgData = 0;                                                 // Data is provided when calling CANMessageGet() (ISR supplies buffer)

    CANMessageSet(CAN0_BASE, MsgObj, &rx, MSG_OBJ_TYPE_RX);
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
    CANListenerStd(1);

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
    //*************************************************************************
    // Local variables used in the main control loop
    //*************************************************************************
    uint32_t CANID_tmp  = 0;            // Parsed return CAN ID / destination ID extracted from received command
    uint32_t CANVAL_tmp = 0;            // Parsed 32-bit value extracted from received CAN payload (command argument)
    uint8_t  CAN_RESP[8];               // Outgoing CAN response payload (8-byte data field)
    uint8_t  CAN_CMD_REQUEST = 0;       // Command byte from incoming CAN message (CAN_RECV.MSG[0])

    //*****************************************************************************
    // Initialize the global CAN receive structure
    //
    // CAN_RECV is updated inside the CAN ISR and read/cleared in the main loop.
    // Initializing it here ensures a known state before interrupts begin firing.
    //
    // NOTE: We use vmemset() instead of memset() because CAN_RECV.MSG is volatile
    // (ISR-written), and standard memset() expects a non-volatile void*.
    //*****************************************************************************
    CAN_RECV.FLAGS = 0;                 // Clear NEW/OVERRUN status flags
    CAN_RECV.ID    = 0;                 // Clear last received CAN ID
    vmemset(CAN_RECV.MSG, 0, sizeof(CAN_RECV.MSG));  // Clear last received payload

    //*************************************************************************
    // System clock configuration
    // Sets the MCU clock used by SysTick timing, ADC timing, CAN bit timing, etc.
    // This config targets ~40 MHz system clock using the PLL and 16 MHz crystal.
    //*************************************************************************
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //*************************************************************************
    // Peripheral initialization
    // Init_ADC():     Configure ADC0 sequencer for PE3(AIN0) + PE2(AIN1)
    // Init_Systick(): Configure 1 ms SysTick ISR for periodic ADC sampling/streaming
    // Init_CAN():     Configure CAN0 pins/bitrate/mailboxes and register ISR callback
    //*************************************************************************
    Init_ADC();
    Init_Systick();
    Init_CAN(CAN_BAUD);

    //*************************************************************************
    // Global interrupt enable
    // Must be called AFTER peripheral setup so interrupts don't fire into
    // uninitialized handlers/structures.
    //*************************************************************************
    IntMasterEnable();

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
            CAN_RECV.FLAGS = (uint8_t)bit_clear(CAN_RECV.FLAGS, CAN_F_NEW);

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
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamRealtime:
                    StreamingMode = smRealTime;
                    CAN_RESP[4] = (uint8_t)(StreamingMode >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamingMode >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamingMode >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamingMode);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStopStreaming:
                    StreamingMode = smStopped;
                    CAN_RESP[4] = (uint8_t)(StreamingMode >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamingMode >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamingMode >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamingMode);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamBuffered:
                    StreamingMode = smBuffered;
                    CAN_RESP[4] = (uint8_t)(StreamingMode >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamingMode >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamingMode >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamingMode);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamingStatus:
                    CAN_RESP[4] = (uint8_t)(StreamingMode >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamingMode >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamingMode >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamingMode);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamBufferSet:    // Set Buffer size
                    // StreamBufferSize = 8192
                    if (CANVAL_tmp == 0 || CANVAL_tmp > 32768) CANVAL_tmp = 8192;
                    StreamBufferSize = CANVAL_tmp;
                    CAN_RESP[4] = (uint8_t)(StreamBufferSize >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamBufferSize >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamBufferSize >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamBufferSize);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;
            }
            // Reset the new message flag and set the heartbeat timer
            HeatbeatTrigger = GlobalTimer + HeartBeatTime;
        }

        // Check for CAN overrun condition (if new data arrived before the previous was processed)
        if (bit_check(CAN_RECV.FLAGS, CAN_F_OVERRUN))
        {
            // Clear the overrun flag
            CAN_RECV.FLAGS = (uint8_t)bit_clear(CAN_RECV.FLAGS, CAN_F_OVERRUN);
        }

        // Check if it's time to send a heartbeat message (every 10 seconds)
        if (StreamingMode == smStopped)
        {
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

                CANSendMSG(CAN_BC_ID, CAN_RESP);    // Send heartbeat message to broadcast address

                // Reset the heartbeat timer
                HeatbeatTrigger = GlobalTimer + HeartBeatTime;
            }
        }
    } // end while(1)
} // end main()
