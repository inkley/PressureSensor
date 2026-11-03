/* Volatile timing telemetry. WTIMER0 runs at system clock, including debugger halt.
 * No per-sample timestamps are added to saved nominal-only recordings here.
 */
#include "driverlib/timer.h"
#include "driverlib/flash.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"

enum {
 TD_HZ, TD_NOW, TD_ATTEMPTS, TD_COMPLETE, TD_TIMEOUT, TD_ADC_ERROR,
 TD_FIRST, TD_LAST, TD_MIN_INTERVAL, TD_MAX_INTERVAL, TD_INTERVAL_BAD,
 TD_EST_MISSING, TD_MIN_ADC, TD_MAX_ADC, TD_TX_DROPS, TD_RAM_OVERWRITES,
 TD_RX_SOFTWARE_LOSS, TD_RX_HARDWARE_LOSS, TD_TX_FAILURES,
 TD_ERASE_CALLS, TD_ERASE_TOTAL, TD_ERASE_MAX, TD_PROGRAM_CALLS,
 TD_PROGRAM_TOTAL, TD_PROGRAM_MAX, TD_UNSAFE_FLASH, TD_FLASH_ERRORS,
 TD_STORAGE_COMMANDS, TD_STORAGE_KIND, TD_STORAGE_TICKS, TD_STORAGE_ATTEMPTS,
 TD_STORAGE_FLASH_CALLS, TD_MIN_SYSTICK, TD_MAX_SYSTICK, TD_SYSTICK_LONG,
 TD_DIAG_MODE, TD_COUNT
};
static volatile uint64_t timing_data[TD_COUNT];
static uint32_t timing_snapshot[6+2*TD_COUNT];
static uint32_t timing_sequence=0;
static uint64_t timing_previous_tick=0, timing_storage_begin=0, timing_storage_attempts=0, timing_storage_calls=0;
static volatile uint32_t timing_mode=0; /* 0 normal, 1 acquisition only, 2 acquisition + CAN; no RAM writes */
static uint32_t timing_hz=20000000;
static uint64_t TimingNow(void) { return TimerValueGet64(WTIMER0_BASE); }
static void TimingReset(void)
{
    uint32_t i; bool disabled=IntMasterDisable();
    for(i=0;i<TD_COUNT;i++) timing_data[i]=0;
    timing_data[TD_HZ]=timing_hz;
    timing_previous_tick=0;
    if(!disabled) IntMasterEnable();
}
static void TimingInit(void)
{
    timing_hz=SysCtlClockGet();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER0)) {}
    TimerDisable(WTIMER0_BASE,TIMER_BOTH);
    TimerConfigure(WTIMER0_BASE,TIMER_CFG_PERIODIC_UP);
    TimerLoadSet64(WTIMER0_BASE,0xFFFFFFFFFFFFFFFFULL);
    TimerControlStall(WTIMER0_BASE,TIMER_BOTH,false);
    TimerEnable(WTIMER0_BASE,TIMER_BOTH);
    TimingReset();
}
static void TimingMinMax(uint32_t lo, uint32_t hi, uint64_t value)
{
    if(!timing_data[lo] || value<timing_data[lo]) timing_data[lo]=value;
    if(value>timing_data[hi]) timing_data[hi]=value;
}
static void TimingTick(void)
{
    uint64_t now=TimingNow(), delta;
    if(timing_previous_tick) {
        delta=now-timing_previous_tick;
        TimingMinMax(TD_MIN_SYSTICK,TD_MAX_SYSTICK,delta);
        if(delta>(uint64_t)(timing_hz/1000+timing_hz/100000)) timing_data[TD_SYSTICK_LONG]++;
    }
    timing_previous_tick=now;
}
static uint64_t TimingAttempt(void)
{
    uint64_t now=TimingNow(),delta,period=timing_hz/1000,tolerance=timing_hz/100000;
    if(timing_data[TD_ATTEMPTS]) {
        delta=now-timing_data[TD_LAST]; TimingMinMax(TD_MIN_INTERVAL,TD_MAX_INTERVAL,delta);
        if(delta<period-tolerance || delta>period+tolerance) timing_data[TD_INTERVAL_BAD]++;
        if(delta>=2*period) timing_data[TD_EST_MISSING]+=delta/period-1;
    } else timing_data[TD_FIRST]=now;
    timing_data[TD_LAST]=now; timing_data[TD_ATTEMPTS]++;
    return now;
}
static void TimingResult(uint64_t began, uint32_t result)
{
    if(result==0) { timing_data[TD_COMPLETE]++; TimingMinMax(TD_MIN_ADC,TD_MAX_ADC,TimingNow()-began); }
    else if(result==1) timing_data[TD_TIMEOUT]++;
    else timing_data[TD_ADC_ERROR]++;
}
static int32_t TimingFlashErase(uint32_t address)
{
    uint64_t began,elapsed; int32_t result;
    if(acquisition_enabled) { timing_data[TD_UNSAFE_FLASH]++; return -1; }
    began=TimingNow(); result=FlashErase(address); elapsed=TimingNow()-began;
    timing_data[TD_ERASE_CALLS]++; timing_data[TD_ERASE_TOTAL]+=elapsed;
    if(elapsed>timing_data[TD_ERASE_MAX]) timing_data[TD_ERASE_MAX]=elapsed;
    if(result) timing_data[TD_FLASH_ERRORS]++;
    return result;
}
static int32_t TimingFlashProgram(uint32_t *data,uint32_t address,uint32_t count)
{
    uint64_t began,elapsed; int32_t result;
    if(acquisition_enabled) { timing_data[TD_UNSAFE_FLASH]++; return -1; }
    began=TimingNow(); result=FlashProgram(data,address,count); elapsed=TimingNow()-began;
    timing_data[TD_PROGRAM_CALLS]++; timing_data[TD_PROGRAM_TOTAL]+=elapsed;
    if(elapsed>timing_data[TD_PROGRAM_MAX]) timing_data[TD_PROGRAM_MAX]=elapsed;
    if(result) timing_data[TD_FLASH_ERRORS]++;
    return result;
}
static void TimingStorageBegin(uint32_t kind)
{
    timing_storage_begin=TimingNow(); timing_storage_attempts=timing_data[TD_ATTEMPTS];
    timing_storage_calls=timing_data[TD_ERASE_CALLS]+timing_data[TD_PROGRAM_CALLS];
    timing_data[TD_STORAGE_KIND]=kind;
}
static void TimingStorageEnd(void)
{
    timing_data[TD_STORAGE_COMMANDS]++; timing_data[TD_STORAGE_TICKS]=TimingNow()-timing_storage_begin;
    timing_data[TD_STORAGE_ATTEMPTS]=timing_data[TD_ATTEMPTS]-timing_storage_attempts;
    timing_data[TD_STORAGE_FLASH_CALLS]=timing_data[TD_ERASE_CALLS]+timing_data[TD_PROGRAM_CALLS]-timing_storage_calls;
}
static uint32_t TimingSnapshotCRC(const uint8_t *p,uint32_t n)
{
    uint32_t crc=0xFFFFFFFFUL,i,b;
    for(i=0;i<n;i++) { crc^=p[i]; for(b=0;b<8;b++) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0); }
    return crc^0xFFFFFFFFUL;
}
static uint32_t TimingSnapshot(void)
{
    uint32_t i,words=6+2*TD_COUNT; bool disabled=IntMasterDisable();
    timing_data[TD_NOW]=TimingNow(); timing_data[TD_DIAG_MODE]=timing_mode;
    timing_snapshot[0]=0x314D4954UL; timing_snapshot[1]=1; timing_snapshot[2]=++timing_sequence;
    timing_snapshot[3]=TD_COUNT; timing_snapshot[4]=3; /* CPU-relative timer; runs through debug halt */
    for(i=0;i<TD_COUNT;i++) { timing_snapshot[5+2*i]=(uint32_t)timing_data[i]; timing_snapshot[6+2*i]=(uint32_t)(timing_data[i]>>32); }
    if(!disabled) IntMasterEnable();
    timing_snapshot[words-1]=TimingSnapshotCRC((const uint8_t *)timing_snapshot,(words-1)*4);
    return words;
}
