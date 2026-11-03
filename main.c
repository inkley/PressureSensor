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
 *     - Periodic heartbeat messages for basic �alive� indication when idle
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
 *   BuildVersion: 1011
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

uint32_t BuildVersion = 1011;       // Firmware version for this build

// Time Out
uint32_t TimeOutCounter = 0;

#define PressureSensor1     0x01
#define PressureSensor2     0x02
#define TemperatureSensor1  0x03
#define TemperatureSensor2  0x04


// CAN Bus Settings
#define CAN_ID      0x107           // CAN bus ID for the sensor module
#define CAN_BC_ID   0x7DF           // CAN ID for sensor data broadcast.
#define CAN_BAUD    1000000         // CAN bus baud rate set to 1 Mbps
uint8_t CAN_BUF[8];

// Inkley Sensor Commands
#define icmdReadVersion         0x01  // Request firmware version (returns BuildVersion)
#define icmdStreamRealtime      0x02  // Start real-time streaming with RAM buffering
#define icmdStreamBuffered      0x03  // Dump buffered samples from RAM
#define icmdStopStreaming       0x04  // Stop all streaming modes
#define icmdStreamingStatus     0x05  // Query streaming and RAM buffer status
#define icmdStreamBufferSet     0x06  // Set stream buffer size (bytes/samples) for buffered mode
#define icmdBufferedSampleData  0x07  // Buffered sample payload frame tag
#define icmdReadFlashData       icmdBufferedSampleData  // Historical alias

//*****************************************************************************
//
// System Timing Settings: Defines system tick timing and timeouts
//
//*****************************************************************************

//*****************************************************************************
//
// System Timing Settings: Defines system tick timing and timeouts
//
//*****************************************************************************

#define SYSTICK_TIMING   1000      // SysTick timer set to 1 millisecond intervals
#define ADC_ReadTimeOut  100        // Timeout for ADC reads

volatile uint32_t GlobalTimer = 0;          // Serviced SysTick count; wraps modulo 2^32 (~49.7 days at 1 kHz)
#define HeartBeatTime 10000        // Heart beat signal interval (10 seconds)
volatile uint32_t HeartbeatTrigger = HeartBeatTime; // Next heartbeat deadline, modulo 2^32

//*****************************************************************************
//
// Stream Buffer Settings
//
//*****************************************************************************

#define SENSORBUFSIZE 4094                // Size of the circular RAM buffer (sample pairs)

// Streaming mode is used by the SysTick ISR and the main loop; mark volatile.
    #define smStopped      0x00
    #define smRealTime     0x01
    #define smBuffered     0x02
volatile uint32_t StreamingMode = smStopped;

// RAM sample buffering
volatile uint32_t StreamBufferCount = 0;        // Number of samples currently in RAM buffer
uint32_t StreamBufferSize = SENSORBUFSIZE;     // Maximum number of samples to buffer before stopping
bool StreamBufferFull = false;        // Indicates the RAM buffer has filled
volatile uint32_t StreamBufferWriteIndex = 0;   // Next circular-buffer slot to write

// RAM buffer dump state (responding to icmdStreamBuffered)
bool buffer_streaming_active = false;
uint32_t buffer_stream_index = 0;
uint32_t buffer_stream_count = 0;
uint32_t buffer_stream_resp_id = 0;

// Buffer sample holding for packing two samples into one CAN frame
static bool g_hasPendingSample = false;
static uint16_t g_pendingP1 = 0;
static uint16_t g_pendingP2 = 0;

// When streaming, reduce CAN bus load by decimating transmission.
// Set to 1 for full-rate (every Systick), 2 for half-rate, etc.
#define STREAM_DECIMATE 2
static uint32_t g_streamTick = 0;

// Pending CAN TX frame produced by the SysTick ISR and consumed by the main loop.
volatile bool g_can_tx_pending = false;
uint8_t g_can_tx_msg[8];

#define FRAME_TYPE_P1P2_PACKED2 0x06  // Two module samples per CAN frame

//*****************************************************************************
//
// Buffer Settings: Defines the buffer size and initializes the buffer array
//
//*****************************************************************************

typedef struct { uint64_t ticks; uint32_t attempt; uint16_t p1,p2; } MeasuredPair;
typedef struct { uint64_t sequence,attempt,previous,current,missing,sample; uint32_t reasons,validity,detail,reserved; } MeasuredEvent;
typedef char MeasuredPairSize[(sizeof(MeasuredPair)==16)?1:-1];
typedef char MeasuredEventSize[(sizeof(MeasuredEvent)==64)?1:-1];
static union { uint64_t alignment; uint32_t raw[4096]; MeasuredPair measured[1024]; } capture_arena;
#define SensorBufferData capture_arena.raw
static volatile bool acquisition_enabled = false;
#include "timing_diag.h"
static uint32_t M2Verify(void),M2Persist(void),M2Recovery(void),M2Lifecycle(uint32_t,uint32_t);
static bool M2JournalValid(const uint8_t *);
static uint32_t M2JournalWrite(uint32_t,uint32_t,uint32_t),M2FinishClear(const uint8_t *);
static volatile bool m2_active=false;
#include "flash_log.h"
#include "flash_lifecycle.h"
#include "measured_capture.h"
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
// RAM Buffer Storage Helpers
//
//*****************************************************************************

static void StreamBuffer_Reset(void)
{
    StreamBufferCount = 0;
    StreamBufferWriteIndex = 0;
    StreamBufferFull = false;
}

static void StreamBuffer_Add(uint32_t packed_sample)
{
    if (StreamBufferSize == 0)
    {
        return;
    }

    SensorBufferData[StreamBufferWriteIndex] = packed_sample;
    StreamBufferWriteIndex++;
    if (StreamBufferWriteIndex >= StreamBufferSize)
    {
        StreamBufferWriteIndex = 0;
    }

    if (StreamBufferCount < StreamBufferSize)
    {
        StreamBufferCount++;
    }
    else
    {
        StreamBufferFull = true;
    }
}

static uint32_t StreamBuffer_GetCount(void)
{
    return StreamBufferCount;
}

static uint32_t StreamBuffer_GetStatusWord(void)
{
    bool was_disabled=IntMasterDisable();
    uint32_t count = StreamBuffer_GetCount();
    uint32_t capacity = m2_active?1024:StreamBufferSize;
    uint32_t status = 0;
    uint32_t mode=(m2_active && acquisition_enabled && log_state==LOG_CAPTURING)?3:StreamingMode;

    if (count > 0x0FFF)
    {
        count = 0x0FFF;
    }

    if (capacity > 0x0FFF)
    {
        capacity = 0x0FFF;
    }

    status |= (mode & 0x0F) << 28;
    status |= ((StreamBufferFull || (m2_active && count>=capacity)) ? 1UL : 0UL) << 27;
    status |= (buffer_streaming_active ? 1UL : 0UL) << 26;
    status |= (capacity & 0x0FFF) << 12;
    status |= (count & 0x0FFF);

    if(!was_disabled) IntMasterEnable();
    return status;
}

static bool StreamBuffer_ReadEntry(uint32_t index, uint32_t *out)
{
    uint32_t count = StreamBuffer_GetCount();
    uint32_t start;

    if (!out || index >= count)
    {
        return false;
    }

    if (StreamBufferFull)
    {
        start = StreamBufferWriteIndex;
    }
    else
    {
        start = 0;
    }

    *out = SensorBufferData[(start + index) % StreamBufferSize];
    return true;
}

//*****************************************************************************
//
// SysTick Interrupt Handler: Handles system tick interrupts that occur
// periodically (every 1 millisecond); it triggers ADC reads, stores sensor
// data in the circular RAM buffer, and prepares real-time CAN frames.
//
//*****************************************************************************

void SysTickIntHandler(void)
{
    uint32_t adcVals[4];
    uint64_t acquisition_began, trigger_proxy=0;

    // Count every serviced tick, including ADC timeout and packed-frame early returns.
    // This is not a count of successful acquisitions or missed hardware ticks.
    GlobalTimer++;
    TimingTick();

    if (!acquisition_enabled) return;
    if (log_state == LOG_CAPTURING) log_attempts++;
    acquisition_began=TimingAttempt();
    if(m2_active && log_state==LOG_CAPTURING) m2_attempts++;
    trigger_proxy=TimingNow();
    ADCProcessorTrigger(ADC0_BASE, 2);
    TimeOutCounter = 0;

    while(!ADCIntStatus(ADC0_BASE, 2, false))
    {
        if(TimeOutCounter++ > ADC_ReadTimeOut)
        {
            ADCIntClear(ADC0_BASE, 2);
            TimingResult(acquisition_began,1);
            if(m2_active && log_state==LOG_CAPTURING) { M2Record(trigger_proxy,0,0,1); return; }
            if (log_state == LOG_CAPTURING) { log_timeouts++; acquisition_enabled=false; log_state=LOG_FAULT; }
            return;
        }
    }

    ADCIntClear(ADC0_BASE, 2);
    if (ADCSequenceDataGet(ADC0_BASE, 2, adcVals) != 2 ||
        ADCSequenceOverflow(ADC0_BASE, 2) || ADCSequenceUnderflow(ADC0_BASE, 2))
    {
        TimingResult(acquisition_began,2);
        if(m2_active && log_state==LOG_CAPTURING) { M2Record(trigger_proxy,0,0,2); return; }
        acquisition_enabled=false;
        if(log_state==LOG_CAPTURING) log_state=LOG_FAULT;
        return; // never store an incomplete pair
    } // [0]=CH0(PE3), [1]=CH1(PE2)
    TimingResult(acquisition_began,0);
    if(m2_active && log_state==LOG_CAPTURING) { M2Record(trigger_proxy,adcVals[0],adcVals[1],0); M2BenchDelay(); return; }

    if (log_state == LOG_CAPTURING)
    {
        SensorBufferData[StreamBufferCount++] = ((adcVals[0]&0xFFF)<<16)|(adcVals[1]&0xFFF);
        StreamBufferWriteIndex = StreamBufferCount % StreamBufferSize;
        if (StreamBufferCount >= log_limit) { acquisition_enabled=false; log_state=LOG_RAM_READY; }
        return;
    }

    if (StreamingMode == smRealTime || timing_mode==2)
    {
        uint16_t p1 = (uint16_t)(adcVals[0] & 0x0FFF);
        uint16_t p2 = (uint16_t)(adcVals[1] & 0x0FFF);
        uint32_t packed_sample = ((uint32_t)p1 << 16) | (uint32_t)p2;

        if(!timing_mode) {
            if(StreamBufferCount>=StreamBufferSize) timing_data[TD_RAM_OVERWRITES]++;
            StreamBuffer_Add(packed_sample);
        }

        // Decimate transmission to reduce CAN bus load and avoid adapter/driver overflow.
        // The module still samples at 1 kHz, but sends only once every STREAM_DECIMATE ticks.
        g_streamTick++;

        // Always keep the latest sample as the pending one.
        if (!g_hasPendingSample)
        {
            g_pendingP1 = p1;
            g_pendingP2 = p2;
            g_hasPendingSample = true;
            return;
        }

        // Only send a CAN frame on the configured decimation interval.
        if ((g_streamTick % STREAM_DECIMATE) != 0)
        {
            // Update the pending sample for the next send.
            g_pendingP1 = p1;
            g_pendingP2 = p2;
            return;
        }

        // Never overwrite a frame still owned by the main-loop transmitter.
        if (g_can_tx_pending) { timing_data[TD_TX_DROPS]++; return; }

        // Prepare a packed frame containing the previous sample + current sample
        // (defer actual CAN transmission to the main loop to keep ISR short)
        g_can_tx_msg[0] = FRAME_TYPE_P1P2_PACKED2;
        g_can_tx_msg[1] = 0x12;

        // Previous sample (A)
        g_can_tx_msg[2] = (uint8_t)(g_pendingP1 >> 4);
        g_can_tx_msg[3] = (uint8_t)((g_pendingP1 & 0x0F) << 4) | (uint8_t)((g_pendingP2 >> 8) & 0x0F);
        g_can_tx_msg[4] = (uint8_t)(g_pendingP2 & 0xFF);

        // Current sample (B)
        g_can_tx_msg[5] = (uint8_t)(p1 >> 4);
        g_can_tx_msg[6] = (uint8_t)((p1 & 0x0F) << 4) | (uint8_t)((p2 >> 8) & 0x0F);
        g_can_tx_msg[7] = (uint8_t)(p2 & 0xFF);

        if (!g_can_tx_pending)
        {
            g_can_tx_pending = true;  // main loop will send this frame
        }

        // Update pending sample so the next frame sends the latest data.
        g_pendingP1 = p1;
        g_pendingP2 = p2;
    }

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
    SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {}

    // PE3 = AIN0 (CH0) -> Pressure1
    // PE2 = AIN1 (CH1) -> Pressure2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    // Use Sequencer 2 (SS2) because SS3 only supports 1 sample
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 2);

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
    if(tempCANMsgObject.ui32Flags & MSG_OBJ_DATA_LOST) timing_data[TD_RX_HARDWARE_LOSS]++;
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
            timing_data[TD_RX_SOFTWARE_LOSS]++;
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
// communication, processes sensor data, and manages RAM buffering for storing sensor
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
    // Nominal 20 MHz system clock: 200 MHz PLL path / 10, using the 16 MHz crystal.
    //*************************************************************************
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //*************************************************************************
    // Peripheral initialization
    // Init_ADC():     Configure ADC0 sequencer for PE3(AIN0) + PE2(AIN1)
    // Init_Systick(): Configure 1 ms SysTick ISR for periodic ADC sampling/streaming
    // Init_CAN():     Configure CAN0 pins/bitrate/mailboxes and register ISR callback
    //*************************************************************************
    TimingInit();
    Init_ADC();
    Init_Systick();
    Init_CAN(CAN_BAUD);

    StreamBuffer_Reset();
    LogLifecycleBoot(); // read-only recovery; explicit clear resumes deletion

    //*************************************************************************
    // Global interrupt enable
    // Must be called AFTER peripheral setup so interrupts don't fire into
    // uninitialized handlers/structures.
    //*************************************************************************
    IntMasterEnable();

    //*************************************************************************
    //
    // Main program loop: Processes incoming CAN messages, handles I2C commands,
    // manages RAM buffering, and sends periodic heartbeat messages
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
            CANVAL_tmp = ((uint32_t)CAN_RECV.MSG[3] << 24) | ((uint32_t)CAN_RECV.MSG[4] << 16) | ((uint32_t)CAN_RECV.MSG[5] << 8) | (uint32_t)CAN_RECV.MSG[6];

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
            if (CAN_CMD_REQUEST>=0x0F && CAN_CMD_REQUEST<=0x3F)
            {
                uint32_t result=0, k;
                if(CAN_CMD_REQUEST>=0x30 && CAN_CMD_REQUEST<=0x3E) {
                    if(CAN_CMD_REQUEST==0x37) {
                        result=M2BuildBlock(CANVAL_tmp);
                        if(!result) {
                            CAN_RESP[6]=1; CANSendMSG_Obj(CANID_tmp,CAN_RESP,32);
                            for(k=0;k<256;k++) {
                                CAN_RESP[1]=(uint8_t)(k>>8); CAN_RESP[2]=(uint8_t)k; CAN_RESP[3]=0x3B;
                                memcpy(CAN_RESP+4,((uint8_t *)log_image)+k*4,4);
                                if(CANSendMSG_Obj(CANID_tmp,CAN_RESP,32)) break;
                            }
                            continue;
                        }
                    } else result=M2Command(CAN_CMD_REQUEST,CANVAL_tmp);
                }
                else if(CAN_CMD_REQUEST==0x0F) result=0x49500104UL;
                else if(CAN_CMD_REQUEST==0x2A) {
                    uint32_t words=TimingSnapshot();
                    CAN_RESP[6]=(uint8_t)(words>>8); CAN_RESP[7]=(uint8_t)words;
                    CANSendMSG_Obj(CANID_tmp,CAN_RESP,32);
                    for(k=0;k<words;k++) {
                        CAN_RESP[1]=(uint8_t)(k>>8); CAN_RESP[2]=(uint8_t)k; CAN_RESP[3]=0x2D;
                        memcpy(CAN_RESP+4,timing_snapshot+k,4);
                        if(CANSendMSG_Obj(CANID_tmp,CAN_RESP,32)!=0) break;
                    }
                    continue;
                }
                else if(CAN_CMD_REQUEST==0x2B) {
                    if(acquisition_enabled) result=LOG_ERROR(1); else { TimingReset(); result=0; }
                }
                else if(CAN_CMD_REQUEST==0x2F) {
                    if(CANVAL_tmp>2 || StreamingMode!=smStopped || buffer_streaming_active ||
                       (acquisition_enabled && !timing_mode)) result=LOG_ERROR(1);
                    else if(CANVAL_tmp && (timing_mode || log_state==LOG_FAULT || m2_adc_fault)) result=LOG_ERROR(1);
                    else {
                        bool was_disabled=IntMasterDisable();
                        acquisition_enabled=false; timing_mode=CANVAL_tmp; g_can_tx_pending=false; g_hasPendingSample=false; g_streamTick=0;
                        if(timing_mode) acquisition_enabled=true;
                        if(!was_disabled) IntMasterEnable();
                        result=timing_mode;
                    }
                }
                else if(CAN_CMD_REQUEST==0x16) result=(log_state<<24)|StreamBufferCount;
                else if(CAN_CMD_REQUEST==0x17) result=LogVerify();
                else if(CAN_CMD_REQUEST==0x1B) result=LogLifecycleRecoveryStatus();
                else if(CAN_CMD_REQUEST==0x1C) {
                    if(StreamingMode!=smStopped || buffer_streaming_active) result=LOG_ERROR(1);
                    else { TimingStorageBegin(4); result=LogScratchTest(CANVAL_tmp); TimingStorageEnd(); }
                }
                else if(CAN_CMD_REQUEST>=0x20 && CAN_CMD_REQUEST<=0x27) {
                    if(acquisition_enabled || StreamingMode!=smStopped || buffer_streaming_active) result=LOG_ERROR(1);
                    else {
                        uint8_t *part=(CAN_CMD_REQUEST<0x24)?lifecycle_request:lifecycle_target;
                        k=(CAN_CMD_REQUEST&3)*4;
                        part[k]=(uint8_t)(CANVAL_tmp>>24); part[k+1]=(uint8_t)(CANVAL_tmp>>16);
                        part[k+2]=(uint8_t)(CANVAL_tmp>>8); part[k+3]=(uint8_t)CANVAL_tmp;
                        lifecycle_mask|=1UL<<(CAN_CMD_REQUEST-0x20);
                    }
                }
                else if(CAN_CMD_REQUEST==0x28 || CAN_CMD_REQUEST==0x29) {
                    if(acquisition_enabled || StreamingMode!=smStopped || buffer_streaming_active) result=LOG_ERROR(1);
                    else { TimingStorageBegin(CAN_CMD_REQUEST==0x28?2:3); result=LogLifecycleCommand(CAN_CMD_REQUEST==0x28?1:2,CANVAL_tmp); TimingStorageEnd(); }
                }
                else if(CAN_CMD_REQUEST==0x18)
                {
                    if(acquisition_enabled || buffer_streaming_active || CANVAL_tmp>=128) result=LOG_ERROR(1);
                    else {
                        const uint8_t *fp=(const uint8_t *)(LOG_BASE+CANVAL_tmp*1024);
                        CAN_RESP[6]=1; // 256 words, followed by tagged frames
                        CANSendMSG_Obj(CANID_tmp,CAN_RESP,32);
                        for(k=0;k<256;k++) {
                            CAN_RESP[1]=(uint8_t)(k>>8); CAN_RESP[2]=(uint8_t)k; CAN_RESP[3]=0x1A;
                            memcpy(CAN_RESP+4,fp+k*4,4);
                            if(CANSendMSG_Obj(CANID_tmp,CAN_RESP,32)!=0) break;
                        }
                        HeartbeatTrigger=GlobalTimer+HeartBeatTime;
                        continue;
                    }
                }
                else if(StreamingMode!=smStopped || buffer_streaming_active || timing_mode) result=LOG_ERROR(1);
                else if(CAN_CMD_REQUEST>=0x10 && CAN_CMD_REQUEST<=0x13)
                {
                    if(log_state!=LOG_EMPTY && log_state!=LOG_CLEARED) result=LOG_ERROR(2);
                    else {
                        k=(CAN_CMD_REQUEST-0x10)*4;
                        log_uuid[k]=(uint8_t)(CANVAL_tmp>>24); log_uuid[k+1]=(uint8_t)(CANVAL_tmp>>16);
                        log_uuid[k+2]=(uint8_t)(CANVAL_tmp>>8); log_uuid[k+3]=(uint8_t)CANVAL_tmp;
                        log_uuid_mask|=1UL<<(CAN_CMD_REQUEST-0x10);
                    }
                }
                else if(CAN_CMD_REQUEST==0x14)
                {
                    uint8_t nonzero=0;
                    for(k=0;k<16;k++) nonzero|=log_uuid[k];
                    if(log_state!=LOG_EMPTY && log_state!=LOG_CLEARED) result=LOG_ERROR(2);
                    else if(!CANVAL_tmp || CANVAL_tmp>StreamBufferSize) result=LOG_ERROR(3);
                    else if(log_uuid_mask!=15 || !nonzero) result=LOG_ERROR(4);
                    else if(!LogNewIdentity()) result=LOG_ERROR(8);
                    else if(m2_adc_fault) result=LOG_ERROR(2);
                    else if(!LogStorageReady()) { log_state=LOG_OCCUPIED; result=LOG_ERROR(5); }
                    else {
                        acquisition_enabled=false; m2_active=false; StreamBuffer_Reset(); g_can_tx_pending=false; g_hasPendingSample=false;
                        log_limit=CANVAL_tmp; log_attempts=0; log_timeouts=0; log_state=LOG_CAPTURING;
                        acquisition_enabled=true; result=CANVAL_tmp;
                    }
                }
                else if(CAN_CMD_REQUEST==0x15) { TimingStorageBegin(1); result=LogPersist(); TimingStorageEnd(); }
                else result=LOG_ERROR(2);
                log_last_result=result;
                CAN_RESP[4]=(uint8_t)(result>>24); CAN_RESP[5]=(uint8_t)(result>>16);
                CAN_RESP[6]=(uint8_t)(result>>8); CAN_RESP[7]=(uint8_t)result;
                CANSendMSG_Obj(CANID_tmp,CAN_RESP,32);
                HeartbeatTrigger=GlobalTimer+HeartBeatTime;
                continue;
            }
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
                    if (log_state != LOG_EMPTY || timing_mode || m2_adc_fault) { CAN_RESP[4]=0x80; CAN_RESP[7]=2; CANSendMSG_Obj(CANID_tmp,CAN_RESP,32); break; }
                    acquisition_enabled=false;
                    StreamingMode = smRealTime;
                    StreamBuffer_Reset();
                    buffer_streaming_active = false;
                    g_streamTick = 0;
                    // Reset any pending sample when starting real-time streaming.
                    g_hasPendingSample = false;
                    g_can_tx_pending = false;
                    acquisition_enabled=true;
                    CAN_RESP[4] = (uint8_t)(StreamingMode >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamingMode >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamingMode >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamingMode);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStopStreaming:
                    if(m2_active) M2Stop(1);
                    acquisition_enabled=false; timing_mode=0;
                    if(log_state==LOG_CAPTURING) log_state=LOG_RAM_READY;
                    g_can_tx_pending=false;
                    StreamingMode = smStopped;
                    buffer_streaming_active = false;

                    // Acknowledge stop streaming; buffered data remains in RAM.
                    CAN_RESP[4] = 0;
                    CAN_RESP[5] = 0;
                    CAN_RESP[6] = 0;
                    CAN_RESP[7] = 0;

                // Reset pending sample state used for packed CAN streaming
                    g_hasPendingSample = false;

                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamBuffered:
                    if(acquisition_enabled || m2_active) { CAN_RESP[4]=0x80; CAN_RESP[7]=1; CANSendMSG_Obj(CANID_tmp,CAN_RESP,32); break; }
                    // Start dumping stored data from the RAM ring buffer (if present)
                    buffer_stream_count = StreamBuffer_GetCount();
                    if (buffer_stream_count > 0)
                    {
                        buffer_streaming_active = true;
                        buffer_stream_index = 0;
                        buffer_stream_resp_id = CANID_tmp;  // send data back to the requester

                        // Reply with confirmation and record size
                        CAN_RESP[4] = (uint8_t)(buffer_stream_count >> 24);
                        CAN_RESP[5] = (uint8_t)(buffer_stream_count >> 16);
                        CAN_RESP[6] = (uint8_t)(buffer_stream_count >> 8);
                        CAN_RESP[7] = (uint8_t)(buffer_stream_count);
                    }
                    else
                    {
                        // No recorded data available
                        CAN_RESP[4] = 0;
                        CAN_RESP[5] = 0;
                        CAN_RESP[6] = 0;
                        CAN_RESP[7] = 0;
                    }
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamingStatus:
                    CANVAL_tmp = StreamBuffer_GetStatusWord();
                    CAN_RESP[4] = (uint8_t)(CANVAL_tmp >> 24);
                    CAN_RESP[5] = (uint8_t)(CANVAL_tmp >> 16);
                    CAN_RESP[6] = (uint8_t)(CANVAL_tmp >> 8);
                    CAN_RESP[7] = (uint8_t)(CANVAL_tmp);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;

                case icmdStreamBufferSet:    // Set Buffer size
                    if(log_state!=LOG_EMPTY && log_state!=LOG_CLEARED) { CAN_RESP[4]=0x80; CAN_RESP[7]=2; CANSendMSG_Obj(CANID_tmp,CAN_RESP,32); break; }
                    if (CANVAL_tmp == 0 || CANVAL_tmp > SENSORBUFSIZE) CANVAL_tmp = SENSORBUFSIZE;
                    StreamBufferSize = CANVAL_tmp;
                    StreamBuffer_Reset();
                    CAN_RESP[4] = (uint8_t)(StreamBufferSize >> 24);
                    CAN_RESP[5] = (uint8_t)(StreamBufferSize >> 16);
                    CAN_RESP[6] = (uint8_t)(StreamBufferSize >> 8);
                    CAN_RESP[7] = (uint8_t)(StreamBufferSize);
                    CANSendMSG_Obj(CANID_tmp, CAN_RESP, 32);
                    break;
            }
            // Reset the new message flag and set the heartbeat timer
            HeartbeatTrigger = GlobalTimer + HeartBeatTime;
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
            uint32_t now = GlobalTimer;
            // Modular deadline comparison: valid for deadlines within half the
            // uint32_t range, with the main loop checking at least that often.
            if ((uint32_t)(now - HeartbeatTrigger) < 0x80000000UL)
            {
                // Prepare and send a heartbeat message with the global timer value
                CAN_RESP[0] = 0x08;  // Message length
                CAN_RESP[1] = (CAN_ID >> 8) & 0xFF;
                CAN_RESP[2] = CAN_ID & 0xFF;
                CAN_RESP[3] = 0x7F;  // Heartbeat command
                CAN_RESP[4] = (uint8_t)(now >> 24);
                CAN_RESP[5] = (uint8_t)(now >> 16);
                CAN_RESP[6] = (uint8_t)(now >> 8);
                CAN_RESP[7] = (uint8_t)(now);

                CANSendMSG(CAN_BC_ID, CAN_RESP);    // Send heartbeat message to broadcast address

                // Schedule from the same snapshot used in the heartbeat payload.
                HeartbeatTrigger = now + HeartBeatTime;
            }
        }

        // If a buffer-dump request is active, send stored RAM samples one-by-one.
        // This allows the host to request and receive buffered data after recording.
          if (buffer_streaming_active)
        {
            uint32_t entry;
            if (StreamBuffer_ReadEntry(buffer_stream_index, &entry))
            {
                // Send the sample pair as the response payload (in bytes 4..7).
                // Bytes 1..2 carry a 16-bit sequence index so the host can
                // detect dropped or reordered buffered playback frames.
                CAN_RESP[0] = 0x08;  // Message length
                CAN_RESP[1] = (uint8_t)(buffer_stream_index >> 8);
                CAN_RESP[2] = (uint8_t)(buffer_stream_index);
                CAN_RESP[3] = icmdBufferedSampleData;
                CAN_RESP[4] = (uint8_t)(entry >> 24);
                CAN_RESP[5] = (uint8_t)(entry >> 16);
                CAN_RESP[6] = (uint8_t)(entry >> 8);
                CAN_RESP[7] = (uint8_t)(entry);

                CANSendMSG_Obj(buffer_stream_resp_id, CAN_RESP, 32);
                buffer_stream_index++;

                if (buffer_stream_index >= buffer_stream_count)
                {
                    // Finished streaming buffered samples.
                    buffer_streaming_active = false;
                    // Notify completion using a response frame with count=0
                    CAN_RESP[4] = 0;
                    CAN_RESP[5] = 0;
                    CAN_RESP[6] = 0;
                    CAN_RESP[7] = 0;
                    CANSendMSG_Obj(buffer_stream_resp_id, CAN_RESP, 32);
                }
            }
            else
            {
                // No data available; stop streaming.
                buffer_streaming_active = false;
            }
        }

        // Transmit any pending CAN frame prepared in the SysTick ISR.
        // Doing CAN TX here keeps the ISR short and avoids blocking inside the ISR.
        if (g_can_tx_pending)
        {
            uint8_t tx_msg[8];
            bool was_disabled=IntMasterDisable();
            memcpy(tx_msg, g_can_tx_msg, sizeof(tx_msg));
            g_can_tx_pending = false;
            if(!was_disabled) IntMasterEnable();
            if(CANSendMSG_Obj(CAN_BC_ID, tx_msg, 31)!=0) timing_data[TD_TX_FAILURES]++;
        }
    } // end while(1)
} // end main()
