/* Measured v2 bounded capture. No acquisition-time flash operations. */
static MeasuredEvent m2_events[32];
static uint8_t m2_domain[16],m2_domain_stage[16];
static uint32_t m2_domain_mask=0,m2_rcc=0,m2_rcc2=0,m2_pllstat=0;
static bool m2_bench=false,m2_bench_delayed=false;
/* Profiles: 1 gap, 2 rejected ADC pair, 3 forced timeout, 4 event overflow. */
static uint32_t m2_bench_profile=0,m2_bench_protected_crc=0;
static bool m2_domain_assigned=false,m2_adc_fault=false,m2_capture_armed=false;
static uint32_t m2_capture_token=0;
static volatile uint32_t m2_attempts=0,m2_errors=0,m2_timeouts=0,m2_event_count=0,m2_event_lost=0;
static volatile uint32_t m2_quality=0,m2_reasons=0,m2_stop_reason=0;
static uint64_t m2_accepted=0,m2_first=0,m2_last=0,m2_stop=0,m2_missing=0;
extern uint32_t __STACK_BOTTOM,__STACK_TOP;
static uint32_t M2StackWatermark(void)
{
    uint32_t p=(uint32_t)&__STACK_BOTTOM,end=(uint32_t)&__STACK_TOP,free=0; bool disabled;
    if(acquisition_enabled || (end-p)!=4096) return LOG_ERROR(2);
    disabled=IntMasterDisable();
    while(p<end && HWREG(p)==0xA5A5A5A5UL) { free+=4; p+=4; }
    if(free>4032) free=4032;
    if(!disabled) IntMasterEnable(); return free;
}
static void M2BenchDelay(void)
{
    uint64_t start;
    if(!m2_bench || !acquisition_enabled) return;
    if(m2_bench_profile==1) { if(m2_bench_delayed || m2_attempts!=16) return; }
    else if(m2_bench_profile!=4) return;
    m2_bench_delayed=true; start=TimingNow();
    while(TimingNow()-start<100000ULL) {} /* fixed5 ms; intentionally delays SysTick service */
}
static uint32_t M2Blocks(uint32_t n) { return (n+56)/57; }
static uint32_t M2EventBlocks(uint32_t n) { return (n+13)/14; }
static uint32_t M2Flags(void) { return 17 | (m2_event_count?2:0) | (m2_quality?4:0) | (m2_event_lost?8:0) | ((m2_quality&8)?32:0); }
static void M2Stop(uint32_t reason)
{
    if(acquisition_enabled && m2_active) {
        acquisition_enabled=false; m2_stop=TimingNow(); m2_stop_reason=reason;
        log_state=LOG_RAM_READY;
    }
}
static void M2Record(uint64_t tick,uint32_t p1,uint32_t p2,uint32_t failure)
{
    uint32_t reason=0,valid=5,index=m2_attempts-1; uint64_t previous=m2_last,missing=0,delta=0;
    if(!index) m2_first=tick;
    else if(tick<=previous) { reason|=32; m2_quality|=32; failure=2; }
    else {
        delta=tick-previous;
        if(delta<19800) reason|=4;
        if(delta>20200) reason|=8;
        if(reason&12) m2_quality|=2;
        if(delta>=40000) { missing=delta/20000-1; m2_missing+=missing; reason|=16; valid|=32; m2_quality|=8; }
        valid|=2;
    }
    m2_last=tick;
    if(failure) {
        if(failure==1) { m2_timeouts++; reason|=1; } else { m2_errors++; reason|=2; }
        m2_quality|=1; m2_adc_fault=true;
    } else {
        MeasuredPair *r=&capture_arena.measured[StreamBufferCount];
        r->ticks=tick; r->attempt=index; r->p1=(uint16_t)(p1&4095); r->p2=(uint16_t)(p2&4095);
        StreamBufferCount++; valid|=8;
    }
    if(reason) {
        m2_reasons|=reason;
        if(m2_event_count<32) {
            MeasuredEvent *e=&m2_events[m2_event_count];
            memset(e,0,sizeof(*e)); e->sequence=m2_event_count; e->attempt=index;
            e->previous=(valid&2)?previous:0; e->current=tick; e->missing=missing;
            e->sample=(valid&8)?StreamBufferCount-1:0; e->reasons=reason; e->validity=valid;
            m2_event_count++;
        } else { m2_event_lost++; m2_quality|=16; m2_reasons|=1024; M2Stop(4); }
    }
    if(failure) M2Stop((reason&32)?5:3);
    else if(StreamBufferCount>=log_limit) M2Stop(2);
}
static void M2Header(uint32_t kind,uint32_t seq,uint32_t n,uint32_t size,uint32_t schema)
{
    uint8_t *p=(uint8_t *)log_image;
    memset(p,0xFF,1024); memset(p,0,96); memcpy(p,"IPLG",4);
    LogPut16(p+4,2); LogPut16(p+6,kind); memcpy(p+8,log_uuid,16); memcpy(p+24,m2_domain,16);
    LogPut32(p+40,seq); LogPut16(p+44,n); LogPut16(p+46,size);
    LogPut32(p+64,20000000); LogPut32(p+68,1000); LogPut32(p+72,M2Flags());
    LogPut16(p+76,96); LogPut32(p+80,schema);
}
static uint32_t M2BuildBlock(uint32_t block)
{
    uint32_t i,n,base,d=M2Blocks(StreamBufferCount),e=M2EventBlocks(m2_event_count),crc=0xFFFFFFFFUL;
    uint8_t *p=(uint8_t *)log_image,*r;
    if(!m2_active || acquisition_enabled || !StreamBufferCount || m2_attempts!=StreamBufferCount+m2_errors+m2_timeouts) return LOG_ERROR(2);
    if(block==3) {
        M2Header(5,0,1,256,m2_bench?2:1); r=p+96; memset(r,0,256);
        LogPut32(r,m2_bench?2:1); LogPut32(r+4,256); LogPut32(r+28,1);
        LogPut64(r+32,20000000); LogPut64(r+40,1); LogPut64(r+48,1000000);
        LogPut32(r+56,64); LogPut32(r+60,15); LogPut64(r+64,m2_first); LogPut64(r+72,m2_first);
        LogPut64(r+80,m2_accepted); LogPut32(r+112,50); LogPut32(r+116,6);
        LogPut32(r+120,log_limit); LogPut32(r+124,32);
        LogPut32(r+136,m2_rcc); LogPut32(r+140,m2_rcc2);
        LogPut32(r+144,m2_pllstat); LogPut32(r+148,1000); LogPut32(r+152,m2_bench?m2_bench_profile:0);
    } else if(block>=9 && block<9+d) {
        base=(block-9)*57; n=StreamBufferCount-base; if(n>57) n=57;
        M2Header(1,1+block-9,n,16,1); LogPut64(p+48,base); LogPut64(p+56,capture_arena.measured[base].ticks);
        for(i=0;i<n;i++) {
            MeasuredPair *v=&capture_arena.measured[base+i]; r=p+96+i*16;
            LogPut64(r,v->ticks); LogPut32(r+8,v->attempt); LogPut16(r+12,v->p1); LogPut16(r+14,v->p2);
        }
    } else if(block>=4 && block<4+e) {
        base=(block-4)*14; n=m2_event_count-base; if(n>14) n=14;
        M2Header(2,1+d+block-4,n,64,1);
        for(i=0;i<n;i++) {
            MeasuredEvent *v=&m2_events[base+i]; r=p+96+i*64;
            LogPut64(r,v->sequence); LogPut64(r+8,v->attempt); LogPut64(r+16,v->previous);
            LogPut64(r+24,v->current); LogPut64(r+32,v->missing); LogPut64(r+40,v->sample);
            LogPut32(r+48,v->reasons); LogPut32(r+52,v->validity); LogPut32(r+56,0); LogPut32(r+60,0);
        }
    } else if(block==2) {
        if(M2BuildBlock(3)) return LOG_ERROR(2); crc=LogCRCFeed(crc,p,1024);
        for(i=0;i<d;i++) { M2BuildBlock(9+i); crc=LogCRCFeed(crc,p,1024); }
        for(i=0;i<e;i++) { M2BuildBlock(4+i); crc=LogCRCFeed(crc,p,1024); }
        M2Header(3,1+d+e,1,256,2); r=p+96; memset(r,0,256);
        LogPut32(r,2); LogPut32(r+4,256); LogPut64(r+8,StreamBufferCount); LogPut64(r+16,m2_attempts);
        LogPut64(r+24,m2_timeouts); LogPut64(r+32,m2_errors); LogPut64(r+40,m2_event_count);
        LogPut64(r+48,m2_event_count+m2_event_lost); LogPut64(r+56,m2_event_lost); LogPut64(r+64,m2_missing);
        LogPut64(r+72,capture_arena.measured[0].ticks); LogPut64(r+80,capture_arena.measured[StreamBufferCount-1].ticks);
        LogPut64(r+88,(m2_quality&32)?0:m2_stop); LogPut64(r+96,m2_first); LogPut64(r+104,m2_last);
        LogPut64(r+112,m2_accepted); LogPut32(r+120,d); LogPut32(r+124,e); LogPut32(r+128,1);
        LogPut32(r+132,crc^0xFFFFFFFFUL); LogPut32(r+136,m2_quality); LogPut32(r+140,m2_stop_reason);
        LogPut32(r+176,m2_reasons); LogPut32(r+180,1); LogPut64(r+184,(m2_quality&32)?4:7);
    } else return LOG_ERROR(3);
    LogSeal(); return 0;
}
static bool M2Common(const uint8_t *p,uint32_t kind,uint32_t size,uint32_t schema)
{
    uint32_t n=LogGet16(p+44),i;
    if(!LogBlockValid((uint32_t)p) || p[4]!=2 || LogGet16(p+6)!=kind || LogGet16(p+46)!=size ||
       LogGet16(p+76)!=96 || LogGet16(p+78) || LogGet32(p+80)!=schema ||
       LogGet32(p+64)!=20000000 || LogGet32(p+68)!=1000 || !LifecycleNonzero(p+8) || !LifecycleNonzero(p+24) ||
       (LogGet32(p+72)&17)!=17 || (LogGet32(p+72)&~63UL) || !n || n>920/size) return false;
    if(kind!=1 && (n!=1 && kind!=2)) return false;
    if(kind!=1 && (LogGet64(p+48) || LogGet64(p+56))) return false;
    for(i=84;i<96;i++) if(p[i]) return false;
    return LogFF(p+96+n*size,920-n*size);
}
static uint32_t M2Verify(void)
{
    const uint8_t *m=(const uint8_t *)(LOG_BASE+2048),*meta=(const uint8_t *)(LOG_BASE+3072),*p,*r;
    uint32_t d,e,n,total=0,ev=0,i,j,k,crc=0xFFFFFFFFUL,flags;
    uint64_t last=0,first=0,attempt=0,first_attempt=0;
    if(!M2Common(m,3,256,2) || LogGet16(m+44)!=1 || !M2Common(meta,5,256,1)) return LOG_ERROR(7);
    r=m+96; if(LogGet32(r)!=2 || LogGet32(r+4)!=256 || LogGet64(r+8)>1024) return LOG_ERROR(7);
    n=(uint32_t)LogGet64(r+8); d=LogGet32(r+120); e=LogGet32(r+124); flags=LogGet32(m+72);
    if(!n || d!=M2Blocks(n) || e>3 || LogGet64(r+40)>32 || e!=M2EventBlocks((uint32_t)LogGet64(r+40)) ||
       LogGet64(r+16)!=n+LogGet64(r+24)+LogGet64(r+32) || LogGet64(r+24)+LogGet64(r+32)>1 ||
       LogGet64(r+48)!=LogGet64(r+40)+LogGet64(r+56) || LogGet64(r+56)>1 ||
       LogGet32(r+128)!=1 || LogGet32(m+40)!=1+d+e || LogGet32(r+180)!=1 ||
       LogGet32(r+140)<1 || LogGet32(r+140)>6 || (LogGet32(r+136)&~255UL) || (LogGet64(r+184)&~31ULL)) return LOG_ERROR(7);
    for(i=144;i<176;i++) if(r[i]) return LOG_ERROR(7);
    for(i=192;i<256;i++) if(r[i]) return LOG_ERROR(7);
    for(k=0;k<1+d+e;k++) {
        uint32_t page=(k==0)?3:((k<=d)?9+k-1:4+k-d-1),kind=(k==0)?5:((k<=d)?1:2);
        p=(const uint8_t *)(LOG_BASE+page*1024);
        if(!M2Common(p,kind,kind==5?256:kind==1?16:64,1) || LogGet32(p+40)!=k ||
           memcmp(p+8,m+8,32) || LogGet32(p+72)!=flags) return LOG_ERROR(7);
        if(kind==1) {
            uint32_t amount=LogGet16(p+44);
            if(LogGet64(p+48)!=total || amount!=((n-total>57)?57:n-total)) return LOG_ERROR(7);
            for(j=0;j<amount;j++) {
                r=p+96+j*16;
                if(LogGet16(r+12)>4095 || LogGet16(r+14)>4095 || LogGet32(r+8)!=total || LogGet32(r+8)>=LogGet64(m+112)) return LOG_ERROR(7);
                if(total && (LogGet64(r)<=last || LogGet32(r+8)<=attempt)) return LOG_ERROR(7);
                if(!total) { first=LogGet64(r); first_attempt=LogGet32(r+8); }
                if(!j && LogGet64(p+56)!=LogGet64(r)) return LOG_ERROR(7);
                last=LogGet64(r); attempt=LogGet32(r+8); total++;
            }
        } else if(kind==2) {
            uint32_t amount=LogGet16(p+44);
            if(amount!=((LogGet64(m+136)-ev>14)?14:LogGet64(m+136)-ev)) return LOG_ERROR(7);
            for(j=0;j<amount;j++) {
                r=p+96+j*64;
                if(LogGet64(r)!=ev++ || !LogGet32(r+48) || (LogGet32(r+48)&~4095UL) ||
                   (LogGet32(r+52)&~63UL) || (LogGet32(r+52)&48)==48 || LogGet32(r+56) || LogGet32(r+60)) return LOG_ERROR(7);
                if((LogGet32(r+52)&1) && LogGet64(r+8)>=LogGet64(m+112)) return LOG_ERROR(7);
            }
        }
        crc=LogCRCFeed(crc,p,1024);
    }
    r=meta+96;
    if(LogGet32(r)!=1 || LogGet32(r+4)!=256 || LogGet32(r+24) || !LogFF(meta+352,664) ||
       LogGet32(r+28)!=1 || LogGet64(r+32)!=20000000 || LogGet64(r+40)!=1 || LogGet64(r+48)!=1000000 ||
       LogGet32(r+56)!=64 || LogGet32(r+60)!=15 || LogGet64(r+64)!=LogGet64(r+72) ||
       LogGet64(r+80)>LogGet64(r+72) || LogGet32(r+112)!=50 || LogGet32(r+116)!=6 ||
       LogGet32(r+120)<n || LogGet32(r+120)>1024 || LogGet32(r+124)!=32 || LogGet32(r+128) ||
       LogGet32(r+132) || LogGet32(r+148)!=1000 || LogGet32(r+152)) return LOG_ERROR(7);
    for(i=8;i<24;i++) if(r[i]) return LOG_ERROR(7);
    for(i=88;i<112;i++) if(r[i]) return LOG_ERROR(7);
    for(i=156;i<256;i++) if(r[i]) return LOG_ERROR(7);
    r=m+96;
    if(total!=n || ev!=LogGet64(r+40) || (crc^0xFFFFFFFFUL)!=LogGet32(r+132) ||
       first!=LogGet64(r+72) || last!=LogGet64(r+80) || LogGet64(r+96)!=LogGet64(meta+168) ||
       LogGet64(r+112)!=LogGet64(meta+176)) return LOG_ERROR(7);
    if(!(LogGet32(r+136)&1) && (first_attempt!=0 || attempt!=n-1 || LogGet64(r+96)!=first || LogGet64(r+104)!=last)) return LOG_ERROR(7);
    if((LogGet64(r+184)&2) && LogGet64(r+88)<LogGet64(r+104)) return LOG_ERROR(7);
    return n;
}
static uint32_t M2Persist(void)
{
    uint32_t i,d=M2Blocks(StreamBufferCount),e=M2EventBlocks(m2_event_count),result=0;
    bool disabled;
    if(m2_bench) return LOG_ERROR(2); /* bench data is NEVER persisted */
    if(log_state==LOG_COMMITTED || log_state==LOG_ACKNOWLEDGED) return M2Verify();
    if(acquisition_enabled || !m2_active || !StreamBufferCount || !LogStorageReady()) return LOG_ERROR(2);
    disabled=IntMasterDisable();
    for(i=0;i<1+d+e;i++) {
        uint32_t page=(i==0)?3:(i<=d?9+i-1:4+i-d-1);
        result=M2BuildBlock(page); if(!result) result=LogWriteBlock(page); if(result) break;
    }
    if(!result) { result=M2BuildBlock(2); if(!result) result=LogWriteBlock(2); }
    if(!disabled) IntMasterEnable();
    if(!result) { result=M2Verify(); if(result==StreamBufferCount) { log_state=LOG_COMMITTED; return result; } }
    log_state=LOG_FAULT; return result;
}
static uint32_t M2Command(uint32_t command,uint32_t value)
{
    uint32_t i,part; bool disabled;
    if(command==0x30) {
        if(value==0) return 0x49500201UL;
        if(value==1) return 1024;
        if(value==2) return 32;
        if(value==3) return m2_domain_assigned?1:0;
        if(value==8) return 0x424E4301UL;
        if(value==9) return 0x424E4302UL;
        if(value>=4 && value<8 && m2_domain_assigned) { const uint8_t *q=m2_domain+(value-4)*4; return ((uint32_t)q[0]<<24)|((uint32_t)q[1]<<16)|((uint32_t)q[2]<<8)|q[3]; }
        return LOG_ERROR(3);
    }
    if(acquisition_enabled || StreamingMode!=smStopped || buffer_streaming_active || timing_mode) return LOG_ERROR(1);
    if(command==0x3E) {
        if(value!=0x5354414BUL) return LOG_ERROR(3);
        return M2StackWatermark();
    }
    if(command==0x3D) {
        uint32_t result;
        if(!m2_bench || lifecycle_mask!=255 || memcmp(lifecycle_target,log_uuid,16)) return LOG_ERROR(8);
        lifecycle_mask=0; result=M2BuildBlock(2);
        if(result || LogGet32(((uint8_t *)log_image)+1016)!=value) return LOG_ERROR(8);
        if((M2Verify()&0x80000000UL) || LogGet32((const uint8_t *)(LOG_BASE+2048+1016))!=m2_bench_protected_crc) return LOG_ERROR(7);
        m2_bench=false; m2_active=false; StreamBufferCount=0; StreamBufferWriteIndex=0; StreamBufferFull=false;
        log_uuid_mask=0; LogLifecycleBoot(); return 0; /* RAM-only discard; no flash calls */
    }
    if(command>=0x31 && command<=0x34) {
        part=command-0x31;
        if(!part) { m2_domain_mask=0; memset(m2_domain_stage,0,16); }
        if(m2_domain_mask!=((1UL<<part)-1)) { m2_domain_mask=0; return LOG_ERROR(4); }
        for(i=0;i<4;i++) m2_domain_stage[part*4+i]=(uint8_t)(value>>(24-8*i));
        m2_domain_mask|=1UL<<part; return value;
    }
    if(command==0x35) {
        if(m2_domain_mask!=15 || !LifecycleNonzero(m2_domain_stage) ||
           value!=(LogCRCFeed(0xFFFFFFFFUL,m2_domain_stage,16)^0xFFFFFFFFUL)) return LOG_ERROR(4);
        m2_domain_mask=0;
        if(m2_domain_assigned) return memcmp(m2_domain,m2_domain_stage,16)?LOG_ERROR(8):1;
        {
            const uint8_t *old=0,*manifest=(const uint8_t *)(LOG_BASE+2048); uint32_t oldidx=0;
            if(manifest[4]==2 && !(M2Verify()&0x80000000UL) && memcmp(manifest+24,m2_domain_stage,16)==0) return LOG_ERROR(8);
            if(LogJournalLatest(&old,&oldidx)==1 && old[4]==2 && memcmp(old+24,m2_domain_stage,16)==0) return LOG_ERROR(8);
        }
        memcpy(m2_domain,m2_domain_stage,16); m2_domain_assigned=true; return 1;
    }
    if(command==0x38) {
        if(!m2_domain_assigned || log_uuid_mask!=15 || (log_state!=LOG_EMPTY && log_state!=LOG_CLEARED)) return LOG_ERROR(2);
        m2_capture_token=value; m2_capture_armed=true; return value;
    }
    if(command>=0x39 && command<=0x3C) {
        const uint8_t *m=(const uint8_t *)(LOG_BASE+2048);
        if(m2_adc_fault) return LOG_ERROR(11);
        if(!m2_domain_assigned || m2_active || StreamBufferCount || SysCtlClockGet()!=20000000) return LOG_ERROR(2);
        if((log_state!=LOG_COMMITTED && log_state!=LOG_ACKNOWLEDGED) || lifecycle_mask!=255 ||
           !LifecycleNonzero(lifecycle_request) || memcmp(lifecycle_request,m+8,16)==0 || memcmp(lifecycle_target,m+8,16) || LogGet32(m+1016)!=value ||
           (M2Verify()&0x80000000UL)) return LOG_ERROR(8);
        i=M2StackWatermark(); if(i<512 || i>4096) return LOG_ERROR(14);
        lifecycle_mask=0; memcpy(log_uuid,lifecycle_request,16); log_uuid_mask=15;
        m2_bench=true; m2_bench_profile=command-0x38; m2_bench_delayed=false; m2_bench_protected_crc=value; value=64;
    }
    if(command==0x36 || (command>=0x39 && command<=0x3C)) {
        uint8_t count_bytes[4]; uint32_t crc;
        for(i=0;i<4;i++) count_bytes[i]=(uint8_t)(value>>(24-8*i));
        crc=LogCRCFeed(0xFFFFFFFFUL,log_uuid,16); crc=LogCRCFeed(crc,m2_domain,16);
        crc=LogCRCFeed(crc,count_bytes,4)^0xFFFFFFFFUL;
        if(command==0x36 && (!m2_capture_armed || crc!=m2_capture_token)) return LOG_ERROR(4);
        m2_capture_armed=false;
        if(!m2_domain_assigned) return LOG_ERROR(12);
        if(m2_adc_fault) return LOG_ERROR(11);
        if(SysCtlClockGet()!=20000000) return LOG_ERROR(13);
        if(!value || value>1024) return LOG_ERROR(3);
        if(log_uuid_mask!=15 || !LifecycleNonzero(log_uuid)) return LOG_ERROR(4);
        if(command==0x36) {
            if(log_state!=LOG_EMPTY && log_state!=LOG_CLEARED) return LOG_ERROR(2);
            if(!LogStorageReady() || !LogNewIdentity()) return LOG_ERROR(5);
            m2_bench=false;
        }
        disabled=IntMasterDisable(); m2_active=true; m2_attempts=0; m2_errors=0; m2_timeouts=0;
        m2_event_count=0; m2_event_lost=0; m2_quality=0; m2_reasons=0; m2_stop_reason=0;
        m2_first=m2_last=m2_stop=m2_missing=0; m2_accepted=TimingNow();
        m2_rcc=HWREG(SYSCTL_RCC); m2_rcc2=HWREG(SYSCTL_RCC2); m2_pllstat=HWREG(SYSCTL_PLLSTAT);
        StreamBufferCount=0; StreamBufferWriteIndex=0; StreamBufferFull=false;
        log_limit=value; log_attempts=0; log_timeouts=0; log_state=LOG_CAPTURING;
        acquisition_enabled=true; if(!disabled) IntMasterEnable(); return value;
    }
    return LOG_ERROR(3);
}
static bool M2Owned(uint32_t page,uint32_t d,uint32_t e)
{
    return page==2 || page==3 || (page>=4 && page<4+e) || (page>=9 && page<9+d);
}
static bool M2JournalValid(const uint8_t *p)
{
    const uint8_t *r=p+96; uint32_t n,d,e,i;
    if(!M2Common(p,4,96,2) || LogGet16(p+44)!=1 || memcmp(p+8,r,16) ||
       !LifecycleNonzero(r+32) || LogGet16(r+22) || (LogGet16(r+20)!=1 && LogGet16(r+20)!=2) ||
       !LogGet64(r+24) || LogGet32(p+40)!=(uint32_t)LogGet64(r+24) || LogGet32(r+80)!=2) return false;
    n=LogGet32(r+52); d=LogGet32(r+48); e=LogGet32(r+60);
    if(!n || n>1024 || d!=M2Blocks(n) || e>3 || LogGet32(r+56)!=1) return false;
    for(i=84;i<96;i++) if(r[i]) return false;
    for(i=0;i<128;i++) if(((r[64+i/8]>>(i%8))&1)!=(M2Owned(i,d,e)?1:0)) return false;
    return true;
}
static bool M2SafeFootprint(const uint8_t *journal)
{
    const uint8_t *r=journal+96,*m=(const uint8_t *)(LOG_BASE+2048); uint32_t i;
    if(!M2JournalValid(journal)) return false;
    if(LogBodyBlank()) return true;
    /* Matching committed envelope remains the proof needed for retry. A damaged
     * manifest/new torn capture cannot be distinguished from old erase debris. */
    if(!M2Common(m,3,256,2) || memcmp(m+8,journal+8,32) || LogGet32(m+1016)!=LogGet32(r+16) ||
       LogGet32(r+48)!=LogGet32(m+216) || LogGet32(r+60)!=LogGet32(m+220) || LogGet32(r+52)!=LogGet64(m+104)) return false;
    for(i=2;i<128;i++) {
        const uint8_t *p=(const uint8_t *)(LOG_BASE+i*1024);
        bool owned=(r[64+i/8]>>(i%8))&1;
        if(!owned && !LogFF(p,1024)) return false;
        if(owned && memcmp(p,"IPLG",4)==0 && memcmp(p+8,journal+8,32)) return false;
    }
    return true;
}
static uint32_t M2JournalWrite(uint32_t op,uint32_t digest,uint32_t count)
{
    const uint8_t *old=0,*m=(const uint8_t *)(LOG_BASE+2048); uint32_t oldidx=0,target,i,d,e,result;
    uint64_t generation=1; int32_t found=LogJournalLatest(&old,&oldidx); uint8_t *p=(uint8_t *)log_image,*r;
    if(found<0) return LOG_ERROR(9);
    if(found==1) { generation=LogJGeneration(old); if(generation==0xFFFFFFFFFFFFFFFFULL) return LOG_ERROR(9); generation++; }
    target=(found==1)?1-oldidx:0; d=LogGet32(m+216); e=LogGet32(m+220);
    M2Header(4,(uint32_t)generation,1,96,2); memcpy(p+8,m+8,32); LogPut32(p+72,LogGet32(m+72));
    r=p+96; memset(r,0,96); memcpy(r,lifecycle_target,16); LogPut32(r+16,digest); LogPut16(r+20,op);
    LogPut64(r+24,generation); memcpy(r+32,lifecycle_request,16); LogPut32(r+48,d); LogPut32(r+52,count);
    LogPut32(r+56,1); LogPut32(r+60,e); LogPut32(r+80,2);
    for(i=2;i<128;i++) if(M2Owned(i,d,e)) r[64+i/8]|=1U<<(i%8);
    LogSeal(); if(TimingFlashErase(LOG_BASE+target*1024)) return LOG_ERROR(6);
    result=LogProgramErasedBlock(target);
    if(!result && !M2JournalValid((const uint8_t *)(LOG_BASE+target*1024))) result=LOG_ERROR(7);
    return result;
}
static uint32_t M2FinishClear(const uint8_t *journal)
{
    const uint8_t *r=journal+96; uint32_t i,address;
    if(LogJOperation(journal)!=2 || !M2SafeFootprint(journal)) return LOG_ERROR(8);
    for(i=3;i<128;i++) if((r[64+i/8]>>(i%8))&1) {
        address=LOG_BASE+i*1024;
        if(TimingFlashErase(address) || !LogFF((const uint8_t *)address,1024)) return LOG_ERROR(6);
    }
    if(TimingFlashErase(LOG_BASE+2048) || !LogBodyBlank()) return LOG_ERROR(6);
    StreamBufferCount=0; StreamBufferWriteIndex=0; StreamBufferFull=false; m2_active=false;
    log_uuid_mask=0; log_limit=0; log_state=LOG_CLEARED; return 0;
}
static uint32_t M2Recovery(void)
{
    const uint8_t *j=0,*m=(const uint8_t *)(LOG_BASE+2048); uint32_t idx=0,n,d,e,i; int32_t found;
    bool extra=false;
    found=LogJournalLatest(&j,&idx); n=M2Verify();
    if(found<0) return (7UL<<24)|((n&0x80000000UL)?0:n);
    if(!(n&0x80000000UL)) {
        d=LogGet32(m+216); e=LogGet32(m+220);
        for(i=2;i<128;i++) if(!M2Owned(i,d,e) && !LogFF((const uint8_t *)(LOG_BASE+i*1024),1024)) extra=true;
        if(found==1 && j[4]==2 && memcmp(j+8,m+8,32)==0 && LogJDigest(j)==LogGet32(m+1016)) {
            if(extra) return (3UL<<24)|n;
            if(LogGet32(j+144)!=d || LogGet32(j+156)!=e || LogGet32(j+148)!=n) return (7UL<<24)|n;
            return ((LogJOperation(j)==1?4UL:6UL)<<24)|n;
        }
        return ((extra?3UL:1UL)<<24)|n;
    }
    if(found==1 && j[4]==2 && LogJOperation(j)==2) {
        if(LogBodyBlank()) return 5UL<<24;
        if(M2SafeFootprint(j)) return 6UL<<24;
    }
    return 2UL<<24;
}
static uint32_t M2Lifecycle(uint32_t op,uint32_t digest)
{
    const uint8_t *j=0,*m=(const uint8_t *)(LOG_BASE+2048); uint32_t idx=0,n,result,i,d,e;
    int32_t found=LogJournalLatest(&j,&idx); bool disabled;
    if(found<0) return LOG_ERROR(9);
    n=M2Verify();
    if(op==2 && found==1 && j[4]==2 && LogJOperation(j)==2 && memcmp(LogJPayload(j),lifecycle_target,16)==0 && LogJDigest(j)==digest) {
        if(LogBodyBlank()) { log_state=LOG_CLEARED; return 0; }
        if(!(n&0x80000000UL) && memcmp(m+8,lifecycle_target,16)) return LOG_ERROR(8);
        disabled=IntMasterDisable(); result=M2FinishClear(j); if(!disabled) IntMasterEnable(); return result;
    }
    if(n&0x80000000UL) return LOG_ERROR(7);
    if(memcmp(m+8,lifecycle_target,16) || LogGet32(m+1016)!=digest) return LOG_ERROR(8);
    d=LogGet32(m+216); e=LogGet32(m+220);
    for(i=2;i<128;i++) if(!M2Owned(i,d,e) && !LogFF((const uint8_t *)(LOG_BASE+i*1024),1024)) return LOG_ERROR(5);
    if(found==1 && j[4]==2 && memcmp(j+8,m+8,32)==0 && LogJDigest(j)==digest) {
        if(LogGet32(j+144)!=d || LogGet32(j+156)!=e || LogGet32(j+148)!=n) return LOG_ERROR(9);
        if(LogJOperation(j)==2) return LOG_ERROR(2);
        if(op==1) { log_state=LOG_ACKNOWLEDGED; return n; }
    } else if(op==2) return LOG_ERROR(10);
    disabled=IntMasterDisable(); result=M2JournalWrite(op,digest,n);
    if(!result && op==2) {
        found=LogJournalLatest(&j,&idx); result=(found==1)?M2FinishClear(j):LOG_ERROR(9);
    }
    if(!disabled) IntMasterEnable();
    if(!result && op==1) { log_state=LOG_ACKNOWLEDGED; return n; }
    if(result) LogLifecycleBoot(); return result;
}
