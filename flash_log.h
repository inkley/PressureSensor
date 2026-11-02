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
static volatile uint32_t log_state = LOG_EMPTY;
static volatile uint32_t log_limit = 0;
static volatile uint32_t log_attempts = 0;
static volatile uint32_t log_timeouts = 0;
static uint8_t log_uuid[16];
static uint32_t log_uuid_mask = 0;
static uint32_t log_last_result = 0;
static uint32_t log_image[256]; /* aligned, 1 KiB staging; never a second sample array */
static volatile bool acquisition_enabled = false;

static void LogPut16(uint8_t *p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void LogPut32(uint8_t *p, uint32_t v) { uint32_t i; for(i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static void LogPut64(uint8_t *p, uint64_t v) { uint32_t i; for(i=0;i<8;i++) p[i]=(uint8_t)(v>>(8*i)); }
static uint32_t LogGet32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
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
    return memcmp(p,"IPLG",4)==0 && p[4]==1 && p[5]==0 &&
           memcmp(p+1020,"CMIT",4)==0 &&
           LogGet32(p+1016)==(LogCRCFeed(0xFFFFFFFFUL,p,1016)^0xFFFFFFFFUL);
}
static uint32_t LogWriteBlock(uint32_t block)
{
    uint32_t address=LOG_BASE+block*LOG_BLOCK, i;
    if(!LogBoundsValid() || block>=128 || address<LOG_BASE || address>LOG_END-LOG_BLOCK) return LOG_ERROR(5);
    for(i=0;i<256;i++) if(*(const volatile uint32_t *)(address+i*4)!=0xFFFFFFFFUL) return LOG_ERROR(5);
    /* Erase only a pre-checked blank target, never existing recordings. */
    if(FlashErase(address)!=0) return LOG_ERROR(6);
    if(FlashProgram(log_image,address,1020)!=0) return LOG_ERROR(6);
    if(memcmp((const void *)address,log_image,1020)!=0) return LOG_ERROR(7);
    if(FlashProgram(log_image+255,address+1020,4)!=0) return LOG_ERROR(6);
    if(memcmp((const void *)address,log_image,1024)!=0 || !LogBlockValid(address)) return LOG_ERROR(7);
    return 0;
}
static uint32_t LogVerify(void)
{
    const uint8_t *m=(const uint8_t *)(LOG_BASE+2*LOG_BLOCK);
    uint32_t blocks,count,i,total=0,crc=0xFFFFFFFFUL,n;
    if(!LogBlockValid(LOG_BASE+2*LOG_BLOCK) || m[6]!=3 || m[7]!=0 || m[28]!=1 || m[29]!=0) return LOG_ERROR(7);
    count=LogGet32(m+64+40); blocks=LogGet32(m+64+96);
    if(!count || count>SENSORBUFSIZE || blocks!=LogBlocks(count) || blocks>119 || LogGet32(m+64+100)!=0) return LOG_ERROR(7);
    if(LogGet32(m+24)!=blocks || LogGet32(m+64+124)!=1) return LOG_ERROR(7);
    for(i=0;i<blocks;i++) {
        const uint8_t *p=(const uint8_t *)(LOG_BASE+(LOG_DATA_FIRST+i)*LOG_BLOCK);
        if(!LogBlockValid((uint32_t)p) || p[6]!=1 || p[7]!=0 || memcmp(p+8,m+8,16)!=0 || LogGet32(p+24)!=i) return LOG_ERROR(7);
        n=(uint32_t)p[28]|((uint32_t)p[29]<<8);
        if(n!=((count-total>119)?119:count-total)) return LOG_ERROR(7);
        total+=n; crc=LogCRCFeed(crc,p,1024);
    }
    if(total!=count || (crc^0xFFFFFFFFUL)!=LogGet32(m+64+104)) return LOG_ERROR(7);
    return count;
}
static uint32_t LogPersist(void)
{
    uint32_t count=StreamBufferCount, blocks,i,j,n,base=0,crc=0xFFFFFFFFUL,result=0;
    bool disabled;
    uint8_t *p=(uint8_t *)log_image;
    if(log_state==LOG_COMMITTED) return LogVerify(); /* retry, never rewrite */
    if(log_state!=LOG_RAM_READY || acquisition_enabled || !count || log_timeouts) return LOG_ERROR(2);
    if(!LogBlank()) return LOG_ERROR(5);
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
