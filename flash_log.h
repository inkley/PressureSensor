/* Stage-one single-record persistence. Included by main.c after RAM globals.
 * All serialized accesses explicit; only reserved, erased flash is writable.
 */
#include "driverlib/flash.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"

#define LOG_BASE 0x20000UL
#define LOG_END  0x40000UL
#define LOG_BLOCK 1024UL
#define LOG_DATA_FIRST 9UL
#define LOG_ERROR(n) (0x80000000UL | (n))
#define LOG_EMPTY 0UL
#define LOG_CAPTURING 1UL
#define LOG_RAM_READY 2UL
#define LOG_COMMITTED 3UL
#define LOG_FAULT 4UL
#define LOG_OCCUPIED 5UL
#define LOG_ACKNOWLEDGED 8UL
static volatile uint32_t log_state = LOG_EMPTY;
static volatile uint32_t log_limit = 0;
static volatile uint32_t log_attempts = 0;
static volatile uint32_t log_timeouts = 0;
static uint8_t log_uuid[16];
static uint32_t log_uuid_mask = 0;
static uint32_t log_last_result = 0;
static uint32_t log_image[256]; /* aligned, 1 KiB staging; never a second sample array */

static void LogPut16(uint8_t *p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void LogPut32(uint8_t *p, uint32_t v) { uint32_t i; for(i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static void LogPut64(uint8_t *p, uint64_t v) { uint32_t i; for(i=0;i<8;i++) p[i]=(uint8_t)(v>>(8*i)); }
static uint32_t LogGet32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static uint32_t LogGet16(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8); }
static uint64_t LogGet64(const uint8_t *p) { return (uint64_t)LogGet32(p)|((uint64_t)LogGet32(p+4)<<32); }
static uint32_t LogCRCFeed(uint32_t crc, const uint8_t *p, uint32_t n)
{
    uint32_t i,b;
    for(i=0;i<n;i++) { crc ^= p[i]; for(b=0;b<8;b++) crc=(crc>>1)^((crc&1)?0xEDB88320UL:0); }
    return crc;
}
extern uint8_t __flash_log_start__, __flash_log_end__;
static bool LogBoundsValid(void)
{
    return (uint32_t)&__flash_log_start__==LOG_BASE && (uint32_t)&__flash_log_end__==LOG_END;
}
static bool LogBlank(void)
{
    uint32_t a;
    if(!LogBoundsValid()) return false;
    for(a=LOG_BASE;a<LOG_END;a+=4) if(*(const volatile uint32_t *)a!=0xFFFFFFFFUL) return false;
    return true;
}
static uint32_t LogBlocks(uint32_t count) { return (count+118)/119; }
static void LogHeader(uint32_t kind, uint32_t seq, uint32_t count, uint32_t slot)
{
    uint8_t *p=(uint8_t *)log_image;
    memset(p,0xFF,LOG_BLOCK); memset(p,0,64);
    memcpy(p,"IPLG",4); LogPut16(p+4,1); LogPut16(p+6,kind);
    memcpy(p+8,log_uuid,16); LogPut32(p+24,seq); LogPut16(p+28,count);
    LogPut16(p+30,10); /* NOMINAL_TIME_ONLY | DEGRADED */
    LogPut64(p+32,slot); LogPut64(p+40,(uint64_t)slot*1000); LogPut32(p+48,1000);
}
static void LogSeal(void)
{
    uint8_t *p=(uint8_t *)log_image;
    LogPut32(p+1016,LogCRCFeed(0xFFFFFFFFUL,p,1016)^0xFFFFFFFFUL);
    memcpy(p+1020,"CMIT",4);
}
static bool LogBlockValid(uint32_t address)
{
    const uint8_t *p;
    if(address<LOG_BASE || address>LOG_END-LOG_BLOCK || (address&1023)) return false;
    p=(const uint8_t *)address;
    return memcmp(p,"IPLG",4)==0 && (p[4]==1 || p[4]==2) && p[5]==0 &&
           memcmp(p+1020,"CMIT",4)==0 &&
           LogGet32(p+1016)==(LogCRCFeed(0xFFFFFFFFUL,p,1016)^0xFFFFFFFFUL);
}
static uint32_t LogProgramErasedBlock(uint32_t block)
{
    uint32_t address=LOG_BASE+block*LOG_BLOCK, i;
    if(!LogBoundsValid() || block>=128 || address<LOG_BASE || address>LOG_END-LOG_BLOCK) return LOG_ERROR(5);
    for(i=0;i<256;i++) if(*(const volatile uint32_t *)(address+i*4)!=0xFFFFFFFFUL) return LOG_ERROR(5);
    if(TimingFlashProgram(log_image,address,1020)!=0) return LOG_ERROR(6);
    if(memcmp((const void *)address,log_image,1020)!=0) return LOG_ERROR(7);
    if(TimingFlashProgram(log_image+255,address+1020,4)!=0) return LOG_ERROR(6);
    if(memcmp((const void *)address,log_image,1024)!=0 || !LogBlockValid(address)) return LOG_ERROR(7);
    return 0;
}
static uint32_t LogWriteBlock(uint32_t block)
{
    uint32_t address=LOG_BASE+block*LOG_BLOCK, i;
    if(!LogBoundsValid() || block>=128 || address<LOG_BASE || address>LOG_END-LOG_BLOCK) return LOG_ERROR(5);
    for(i=0;i<256;i++) if(*(const volatile uint32_t *)(address+i*4)!=0xFFFFFFFFUL) return LOG_ERROR(5);
    if(TimingFlashErase(address)!=0) return LOG_ERROR(6);
    return LogProgramErasedBlock(block);
}
static bool LogFF(const uint8_t *p, uint32_t n)
{
    uint32_t i;
    for(i=0;i<n;i++) if(p[i]!=0xFF) return false;
    return true;
}
static bool LogCommonValid(const uint8_t *p)
{
    uint32_t i; uint8_t any=0;
    if(p[4]!=1 || !LogBlockValid((uint32_t)p) || LogGet16(p+30)!=10 || LogGet32(p+48)!=1000) return false;
    for(i=0;i<16;i++) any|=p[8+i];
    if(!any) return false;
    for(i=52;i<64;i++) if(p[i]) return false;
    return true;
}
static uint32_t LogVerify(void)
{
    if(*(const uint16_t *)(LOG_BASE+2048+4)==2) return M2Verify();
    const uint8_t *m=(const uint8_t *)(LOG_BASE+2*LOG_BLOCK);
    uint32_t blocks,count,i,j,total=0,crc=0xFFFFFFFFUL,n;
    if(!LogBoundsValid() || !LogCommonValid(m) || LogGet16(m+6)!=3 || LogGet16(m+28)!=1) return LOG_ERROR(7);
    if(LogGet64(m+104)>SENSORBUFSIZE) return LOG_ERROR(7);
    count=(uint32_t)LogGet64(m+104); blocks=LogGet32(m+160);
    if(!count || blocks!=LogBlocks(count) || blocks>119 || LogGet32(m+164)!=0) return LOG_ERROR(7);
    if(LogGet32(m+24)!=blocks || LogGet32(m+188)!=1 || !LogFF(m+192,824)) return LOG_ERROR(7);
    if(LogGet64(m+32)!=0 || LogGet64(m+40)!=0 || LogGet64(m+112)!=count || LogGet64(m+120)!=0 ||
       LogGet64(m+128)!=0 || LogGet64(m+136)!=0 || LogGet64(m+144)!=count-1 ||
       LogGet64(m+152)!=(uint64_t)count*1000 || LogGet64(m+180)!=0 || LogGet32(m+172)!=0xC1 ||
       (LogGet32(m+176)!=1 && LogGet32(m+176)!=3)) return LOG_ERROR(7);
    for(i=0;i<blocks;i++) {
        const uint8_t *p=(const uint8_t *)(LOG_BASE+(LOG_DATA_FIRST+i)*LOG_BLOCK);
        if(!LogCommonValid(p) || LogGet16(p+6)!=1 || memcmp(p+8,m+8,16)!=0 || LogGet32(p+24)!=i) return LOG_ERROR(7);
        n=LogGet16(p+28);
        if(n!=((count-total>119)?119:count-total) || LogGet64(p+32)!=total || LogGet64(p+40)!=(uint64_t)total*1000) return LOG_ERROR(7);
        for(j=0;j<n;j++) {
            const uint8_t *record=p+64+j*8;
            if(LogGet32(record)!=j*1000 || LogGet16(record+4)>4095 || LogGet16(record+6)>4095) return LOG_ERROR(7);
        }
        if(!LogFF(p+64+n*8,952-n*8)) return LOG_ERROR(7);
        total+=n; crc=LogCRCFeed(crc,p,1024);
    }
    if(total!=count || (crc^0xFFFFFFFFUL)!=LogGet32(m+168)) return LOG_ERROR(7);
    return count;
}
/* Read-only scan: 0 blank, 1 committed, 2 incomplete/damaged/unsupported,
 * 3 committed plus unreferenced programmed blocks. No auto-reclaim. */
static uint32_t LogRecoveryStatus(void)
{
    uint32_t result,blocks,i; bool extra=false;
    if(!LogBoundsValid()) return LOG_ERROR(5);
    if(LogBlank()) return 0;
    result=LogVerify();
    if(result&0x80000000UL) return 2UL<<24;
    blocks=LogBlocks(result);
    for(i=0;i<128;i++) {
        if(i==2 || (i>=LOG_DATA_FIRST && i<LOG_DATA_FIRST+blocks)) continue;
        if(!LogFF((const uint8_t *)(LOG_BASE+i*LOG_BLOCK),LOG_BLOCK)) extra=true;
    }
    return ((extra?3UL:1UL)<<24)|result;
}
static bool LogStorageReady(void);
static uint32_t LogPersist(void)
{
    if(m2_active) return M2Persist();
    uint32_t count=StreamBufferCount, blocks,i,j,n,base=0,crc=0xFFFFFFFFUL,result=0;
    bool disabled;
    uint8_t *p=(uint8_t *)log_image;
    if(log_state==LOG_COMMITTED || log_state==LOG_ACKNOWLEDGED) return LogVerify(); /* retry, never rewrite */
    if(log_state!=LOG_RAM_READY || acquisition_enabled || !count || log_timeouts) return LOG_ERROR(2);
    if(!LogStorageReady()) return LOG_ERROR(5);
    blocks=LogBlocks(count);
    disabled=IntMasterDisable(); /* 20 MHz; stalled CPU/interrupts deliberately excluded from capture */
    for(i=0;i<blocks;i++) {
        n=(count-base>119)?119:count-base;
        LogHeader(1,i,n,base);
        for(j=0;j<n;j++) {
            uint32_t sample=SensorBufferData[base+j];
            LogPut32(p+64+j*8,j*1000); LogPut16(p+68+j*8,sample>>16); LogPut16(p+70+j*8,sample&0xFFFF);
        }
        LogSeal(); result=LogWriteBlock(LOG_DATA_FIRST+i);
        if(result) break;
        crc=LogCRCFeed(crc,p,1024); base+=n;
    }
    if(!result) {
        LogHeader(3,blocks,1,0); memset(p+64,0,128);
        /* Device ID unavailable in this stage; zero means unknown, never use DID0 as unique ID. */
        LogPut64(p+104,count); LogPut64(p+112,log_attempts); LogPut64(p+120,log_timeouts);
        LogPut64(p+144,count-1); LogPut64(p+152,(uint64_t)count*1000);
        LogPut32(p+160,blocks); LogPut32(p+168,crc^0xFFFFFFFFUL);
        LogPut32(p+172,0xC1); /* NOMINAL_ONLY | TIME_UNCERTAIN | BEST_EFFORT */
        LogPut32(p+176,(count==log_limit)?3:1); LogPut32(p+188,1);
        LogSeal(); result=LogWriteBlock(2);
    }
    if(!disabled) IntMasterEnable();
    if(!result) { result=LogVerify(); if(result==count) { log_state=LOG_COMMITTED; return result; } }
    log_state=LOG_FAULT; return result; /* source RAM unchanged, no automatic reclaim */
}

/* Scratch-only bench injection. Never called by normal capture/persist/recovery.
 * op1 omits commit, op2 finishes it, op3 writes stale-CRC fixture,
 * op4 writes a valid fixture, op5 erases scratch. Not a real power-cut simulator.
 */
static volatile uint32_t log_test_phase=0;
static uint32_t LogScratchTest(uint32_t argument)
{
    uint32_t op=argument&0xFF, count,blocks,i,address=LOG_BASE+7*LOG_BLOCK,result=0;
    uint32_t manifest_crc;
    bool disabled;
    uint8_t *p=(uint8_t *)log_image;
    if(*(const uint8_t *)(LOG_BASE+2052)!=1) return LOG_ERROR(2);
    if((argument&0xFFFFFF00UL)!=0x53524300UL || op<1 || op>5 || acquisition_enabled) return LOG_ERROR(2);
    count=LogVerify();
    if(count&0x80000000UL) return count;
    blocks=LogBlocks(count);
    /* Restrict this disposable fixture to the current single-record layout. */
    for(i=0;i<128;i++) {
        if(i==2 || i==7 || (i>=9 && i<9+blocks)) continue;
        if(!LogFF((const uint8_t *)(LOG_BASE+i*LOG_BLOCK),1024)) return LOG_ERROR(5);
    }
    manifest_crc=LogGet32((const uint8_t *)(LOG_BASE+2*LOG_BLOCK+1016));
    LogHeader(1,0,1,0);
    memcpy(p+8,(const void *)(LOG_BASE+2*LOG_BLOCK+8),16);
    LogPut32(p+64,0); LogPut16(p+68,1); LogPut16(p+70,2); LogSeal();
    disabled=IntMasterDisable();
    if(op==2) {
        if(memcmp((const void *)address,p,1020)!=0 || !LogFF((const uint8_t *)(address+1020),4)) result=LOG_ERROR(2);
        else { log_test_phase=5; if(TimingFlashProgram(log_image+255,address+1020,4)) result=LOG_ERROR(6); }
    } else {
        log_test_phase=1;
        if(TimingFlashErase(address)) result=LOG_ERROR(6);
        if(!result && !LogFF((const uint8_t *)address,1024)) result=LOG_ERROR(7);
        log_test_phase=2;
        if(!result && op!=5) {
            if(op==3) p[68]^=1; /* changed BEFORE first programming, CRC deliberately stale */
            log_test_phase=3;
            if(TimingFlashProgram(log_image,address,1020)) result=LOG_ERROR(6);
            log_test_phase=4;
            if(!result && op!=1) {
                log_test_phase=5;
                if(TimingFlashProgram(log_image+255,address+1020,4)) result=LOG_ERROR(6);
            }
        }
    }
    log_test_phase=6;
    if(!disabled) IntMasterEnable();
    if(!result && (LogVerify()!=count || LogGet32((const uint8_t *)(LOG_BASE+2*LOG_BLOCK+1016))!=manifest_crc)) result=LOG_ERROR(7);
    if(!result) {
        if(op==5) { if(!LogFF((const uint8_t *)address,1024)) result=LOG_ERROR(7); }
        else if(op==1 || op==3) { if(LogBlockValid(address)) result=LOG_ERROR(7); }
        else if(!LogCommonValid((const uint8_t *)address)) result=LOG_ERROR(7);
    }
    return result;
}
