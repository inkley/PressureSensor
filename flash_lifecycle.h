/* Durable single-record ACK/delete lifecycle. No boot-time erases.
 * Existing v1 DATA/MANIFEST remain unchanged; journal record schema is specified
 * in FLASH_LIFECYCLE.md. New host commands are capability-gated.
 */
#define LOG_CLEARED 6UL
#define LOG_CLEAR_PENDING 7UL
static uint8_t lifecycle_request[16], lifecycle_target[16];
static uint32_t lifecycle_mask=0;
static bool LifecycleNonzero(const uint8_t *p)
{
    uint32_t i; uint8_t any=0; for(i=0;i<16;i++) any|=p[i]; return any!=0;
}
static bool LogJournalValid(const uint8_t *p)
{
    uint32_t i,count;
    if(!LogBlockValid((uint32_t)p) || LogGet16(p+6)!=4 || LogGet16(p+28)!=1 || LogGet16(p+30)!=2 ||
       LogGet64(p+32)!=0 || LogGet64(p+40)!=0 || LogGet32(p+48)!=1000 ||
       !LifecycleNonzero(p+8) || memcmp(p+8,p+64,16)!=0 || !LifecycleNonzero(p+96) ||
       LogGet16(p+86)!=0 || !LogGet64(p+88) || LogGet32(p+24)!=(uint32_t)LogGet64(p+88)) return false;
    if(LogGet16(p+84)!=1 && LogGet16(p+84)!=2) return false;
    for(i=52;i<64;i++) if(p[i]) return false;
    for(i=120;i<128;i++) if(p[i]) return false;
    count=LogGet32(p+116);
    return count>0 && count<=SENSORBUFSIZE && LogGet32(p+112)==LogBlocks(count) && LogFF(p+128,888);
}
/* No valid snapshot: zero if truly blank, -1 if ambiguous/partial metadata.
 * A valid newest snapshot survives an interrupted write to the other page.
 */
static int32_t LogJournalLatest(const uint8_t **latest, uint32_t *index)
{
    const uint8_t *a=(const uint8_t *)LOG_BASE, *b=a+1024;
    bool va=LogJournalValid(a), vb=LogJournalValid(b);
    if(!va && !vb) return (LogFF(a,2048)?0:-1);
    if(va && vb && LogGet64(a+88)==LogGet64(b+88) && memcmp(a,b,1024)!=0) return -1;
    *index=(!va || (vb && LogGet64(b+88)>LogGet64(a+88)))?1:0;
    *latest=(*index?b:a); return 1;
}
static bool LogBodyBlank(void)
{
    return LogBoundsValid() && LogFF((const uint8_t *)(LOG_BASE+2048),LOG_END-LOG_BASE-2048);
}
static bool LogStorageReady(void)
{
    const uint8_t *j=0; uint32_t index=0; int32_t found;
    if(!LogBodyBlank()) return false;
    found=LogJournalLatest(&j,&index);
    return found==0 || (found==1 && LogGet16(j+84)==2);
}
static bool LogNewIdentity(void)
{
    const uint8_t *j=0; uint32_t index=0;
    if(LogJournalLatest(&j,&index)==1 && memcmp(j+64,log_uuid,16)==0) return false;
    return true;
}
static bool LogClearFootprintSafe(const uint8_t *id, uint32_t blocks)
{
    uint32_t i;
    const uint8_t *m=(const uint8_t *)(LOG_BASE+2048);
    bool matching_manifest;
    if(!LogBoundsValid() || !blocks || blocks>LogBlocks(SENSORBUFSIZE)) return false;
    matching_manifest=LogBlockValid((uint32_t)m) && LogGet16(m+6)==3 && memcmp(m+8,id,16)==0 && LogGet32(m+160)==blocks;
    /* DATA erase precedes manifest erase. Without the matching manifest,
     * nonblank DATA might belong to a later interrupted capture: never apply
     * the old deletion intent to it, even if its header is only partly written. */
    if(!matching_manifest && !LogFF((const uint8_t *)(LOG_BASE+9*1024),blocks*1024)) return false;
    for(i=2;i<128;i++) {
        const uint8_t *p=(const uint8_t *)(LOG_BASE+i*1024);
        bool owned=(i==2 || (i>=9 && i<9+blocks));
        if(!owned && !LogFF(p,1024)) return false; /* includes scratch and event areas */
        if(owned && memcmp(p,"IPLG",4)==0 && memcmp(p+8,id,16)!=0) return false;
    }
    return true;
}
static uint32_t LogLifecycleRecoveryStatus(void)
{
    const uint8_t *j=0, *m=(const uint8_t *)(LOG_BASE+2048);
    uint32_t index=0,count,blocks,i; int32_t found;
    bool extra=false;
    if(!LogBoundsValid()) return LOG_ERROR(5);
    found=LogJournalLatest(&j,&index);
    count=LogVerify();
    if(found<0) return (7UL<<24)|((count&0x80000000UL)?0:count);
    if(!(count&0x80000000UL)) {
        if(found==1 && memcmp(j+64,m+8,16)==0 && LogGet32(j+80)==LogGet32(m+1016)) {
            if(LogGet16(j+84)==2) return (6UL<<24)|count;
            return (4UL<<24)|count;
        }
        blocks=LogBlocks(count);
        for(i=2;i<128;i++) {
            if(i==2 || (i>=9 && i<9+blocks)) continue;
            if(!LogFF((const uint8_t *)(LOG_BASE+i*1024),1024)) extra=true;
        }
        return ((extra?3UL:1UL)<<24)|count;
    }
    if(found==1 && LogGet16(j+84)==2) {
        if(LogBodyBlank()) return 5UL<<24;
        if(LogClearFootprintSafe(j+64,LogGet32(j+112))) return 6UL<<24;
    }
    return LogRecoveryStatus();
}
static void LogLifecycleBoot(void)
{
    uint32_t status=LogLifecycleRecoveryStatus(), code=status>>24;
    acquisition_enabled=false;
    if(status==0) log_state=LOG_EMPTY;
    else if(code==1 || code==3 || code==4) {
        memcpy(log_uuid,(const void *)(LOG_BASE+2048+8),16);
        log_state=(code==4)?LOG_ACKNOWLEDGED:LOG_COMMITTED;
    } else if(code==5) log_state=LOG_CLEARED;
    else if(code==6) log_state=LOG_CLEAR_PENDING;
    else log_state=LOG_OCCUPIED;
}
/* Called with interrupts masked. New snapshot committed before old one reused. */
static uint32_t LogJournalWrite(uint32_t operation, uint32_t digest, uint32_t count)
{
    const uint8_t *old=0; uint32_t old_index=0,target,result; uint64_t generation=1;
    int32_t found=LogJournalLatest(&old,&old_index); uint8_t *p=(uint8_t *)log_image;
    if(found<0) return LOG_ERROR(9);
    target=(found==1)?1-old_index:0;
    if(found==1) {
        generation=LogGet64(old+88);
        if(generation==0xFFFFFFFFFFFFFFFFULL) return LOG_ERROR(9);
        generation++;
    }
    LogHeader(4,(uint32_t)generation,1,0);
    memcpy(p+8,lifecycle_target,16); LogPut16(p+30,2); memset(p+64,0,64);
    memcpy(p+64,lifecycle_target,16); LogPut32(p+80,digest); LogPut16(p+84,operation);
    LogPut64(p+88,generation); memcpy(p+96,lifecycle_request,16);
    LogPut32(p+112,LogBlocks(count)); LogPut32(p+116,count); LogSeal();
    if(TimingFlashErase(LOG_BASE+target*1024)) return LOG_ERROR(6);
    result=LogProgramErasedBlock(target); /* erased once above; marker still programmed last */
    if(result) return result;
    if(!LogJournalValid((const uint8_t *)(LOG_BASE+target*1024))) return LOG_ERROR(7);
    return 0;
}
static uint32_t LogFinishClear(const uint8_t *intent)
{
    uint32_t i,blocks=LogGet32(intent+112),address;
    if(LogGet16(intent+84)!=2 || !LogClearFootprintSafe(intent+64,blocks)) return LOG_ERROR(8);
    /* Re-erase the entire authorized footprint on retry, even FF-looking pages. */
    for(i=0;i<blocks;i++) {
        address=LOG_BASE+(9+i)*1024;
        if(TimingFlashErase(address) || !LogFF((const uint8_t *)address,1024)) return LOG_ERROR(6);
    }
    address=LOG_BASE+2048; /* manifest last; durable intent stays in journal */
    if(TimingFlashErase(address) || !LogFF((const uint8_t *)address,1024)) return LOG_ERROR(6);
    if(!LogBodyBlank()) return LOG_ERROR(7);
    StreamBufferCount=0; StreamBufferWriteIndex=0; StreamBufferFull=false;
    log_uuid_mask=0; log_limit=0; log_attempts=0; log_timeouts=0;
    log_state=LOG_CLEARED; return 0;
}
static uint32_t LogLifecycleCommand(uint32_t operation, uint32_t digest)
{
    const uint8_t *j=0, *m=(const uint8_t *)(LOG_BASE+2048);
    uint32_t index=0,result,count; int32_t found; bool disabled;
    if(log_state==LOG_RAM_READY || log_state==LOG_CAPTURING || log_state==LOG_FAULT) return LOG_ERROR(2);
    if(acquisition_enabled || lifecycle_mask!=255 || !LifecycleNonzero(lifecycle_request) || !LifecycleNonzero(lifecycle_target)) return LOG_ERROR(2);
    lifecycle_mask=0; found=LogJournalLatest(&j,&index);
    if(found<0) return LOG_ERROR(9);
    if(operation==1 && found==1 && LogGet16(j+84)==2 && memcmp(j+64,lifecycle_target,16)==0 && LogGet32(j+80)==digest) return LOG_ERROR(2);
    count=LogVerify();
    /* ID-bound duplicate CLEAR is safe after reboot; never erase a newer record. */
    if(operation==2 && found==1 && LogGet16(j+84)==2 && memcmp(j+64,lifecycle_target,16)==0 && LogGet32(j+80)==digest) {
        if(!(count&0x80000000UL) && memcmp(m+8,lifecycle_target,16)!=0) return LOG_ERROR(8);
        if(LogBodyBlank()) { log_state=LOG_CLEARED; return 0; }
        disabled=IntMasterDisable(); result=LogFinishClear(j); if(!disabled) IntMasterEnable();
        if(result) LogLifecycleBoot();
        return result;
    }
    if(count&0x80000000UL) return LOG_ERROR(7);
    if(memcmp(m+8,lifecycle_target,16)!=0 || LogGet32(m+1016)!=digest) return LOG_ERROR(8);
    if(!LogClearFootprintSafe(lifecycle_target,LogBlocks(count))) return LOG_ERROR(5);
    if(operation==1 && found==1 && LogGet16(j+84)==1 && memcmp(j+64,lifecycle_target,16)==0 && LogGet32(j+80)==digest) {
        log_state=LOG_ACKNOWLEDGED; return count;
    }
    /* Require matching durable ACK before normal CLEAR. No implicit discard mode. */
    if(operation==2 && !(found==1 && LogGet16(j+84)==1 && memcmp(j+64,lifecycle_target,16)==0 && LogGet32(j+80)==digest)) return LOG_ERROR(10);
    disabled=IntMasterDisable();
    result=LogJournalWrite(operation,digest,count);
    if(!result && operation==2) {
        found=LogJournalLatest(&j,&index);
        result=(found==1)?LogFinishClear(j):LOG_ERROR(9);
    }
    if(!disabled) IntMasterEnable();
    if(result) { LogLifecycleBoot(); return result; }
    if(operation==1) { log_state=LOG_ACKNOWLEDGED; return count; }
    return 0;
}
