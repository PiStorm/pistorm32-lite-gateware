#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/timer.h>
#include <exec/memory.h>
#include <exec/execbase.h>
#include <devices/timer.h>
#include <stdio.h>
#include <string.h>

extern struct ExecBase *SysBase;
struct Device *TimerBase = NULL;
static struct MsgPort *timerPort = NULL;
static struct timerequest *timerReq = NULL;
static ULONG eclock_freq = 0;

static int init_timer(void) {
    timerPort = CreateMsgPort();
    if (!timerPort) return 0;
    timerReq = (struct timerequest *)CreateIORequest(timerPort, sizeof(struct timerequest));
    if (!timerReq) {
        DeleteMsgPort(timerPort);
        return 0;
    }
    if (OpenDevice(TIMERNAME, UNIT_ECLOCK, (struct IORequest *)timerReq, 0) != 0) {
        DeleteIORequest((struct IORequest *)timerReq);
        DeleteMsgPort(timerPort);
        return 0;
    }
    TimerBase = (struct Device *)timerReq->tr_node.io_Device;
    struct EClockVal dummy;
    eclock_freq = ReadEClock(&dummy);
    return 1;
}

static void cleanup_timer(void) {
    if (timerReq) {
        CloseDevice((struct IORequest *)timerReq);
        DeleteIORequest((struct IORequest *)timerReq);
        timerReq = NULL;
    }
    if (timerPort) {
        DeleteMsgPort(timerPort);
        timerPort = NULL;
    }
}

static inline ULONG get_eclock(void) {
    struct EClockVal ev;
    ReadEClock(&ev);
    return ev.ev_lo;
}

static void fmt_mbs(char *buf, ULONG ticks, ULONG bytes) {
    if (!ticks || !eclock_freq) { sprintf(buf, "0.00"); return; }
    unsigned long long mbs_x100 = ((unsigned long long)bytes * (unsigned long long)eclock_freq * 100ULL) / ((unsigned long long)ticks * 1048576ULL);
    sprintf(buf, "%lu.%02lu", (ULONG)(mbs_x100 / 100), (ULONG)(mbs_x100 % 100));
}

// ---------------------------------------------------------------------------
// Benchmark Assembly Kernels
// ---------------------------------------------------------------------------
static void bench_read32(volatile ULONG *ptr, ULONG count) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.l (%0)+, d0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(ptr), "+d"(count)
        :
        : "d0", "cc", "memory"
    );
}

static void bench_read16(volatile UWORD *ptr, ULONG count) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.w (%0)+, d0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(ptr), "+d"(count)
        :
        : "d0", "cc", "memory"
    );
}

static void bench_write32(volatile ULONG *ptr, ULONG count) {
    __asm__ __volatile__ (
        "moveq  #0, d0\n\t"
        "1:\n\t"
        "move.l d0, (%0)+\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(ptr), "+d"(count)
        :
        : "d0", "cc", "memory"
    );
}

static void bench_write16(volatile UWORD *ptr, ULONG count) {
    __asm__ __volatile__ (
        "moveq  #0, d0\n\t"
        "1:\n\t"
        "move.w d0, (%0)+\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(ptr), "+d"(count)
        :
        : "d0", "cc", "memory"
    );
}

struct TestResult {
    char name[48];
    ULONG addr;
    ULONG size_kb;
    char r32[16];
    char r16[16];
    char w32[16];
    char w16[16];
};

static void run_test(struct TestResult *res, const char *name, void *addr, ULONG size_bytes, int repeats) {
    strncpy(res->name, name, 47);
    res->addr = (ULONG)addr;
    res->size_kb = size_bytes / 1024;

    ULONG ops32 = size_bytes / 4;
    ULONG ops16 = size_bytes / 2;
    ULONG total_bytes = size_bytes * repeats;

    // Read 32
    ULONG t0 = get_eclock();
    for (int r = 0; r < repeats; r++) bench_read32((volatile ULONG *)addr, ops32);
    ULONG dt_r32 = get_eclock() - t0;
    fmt_mbs(res->r32, dt_r32, total_bytes);

    // Read 16
    t0 = get_eclock();
    for (int r = 0; r < repeats; r++) bench_read16((volatile UWORD *)addr, ops16);
    ULONG dt_r16 = get_eclock() - t0;
    fmt_mbs(res->r16, dt_r16, total_bytes);

    // Write 32
    t0 = get_eclock();
    for (int r = 0; r < repeats; r++) bench_write32((volatile ULONG *)addr, ops32);
    ULONG dt_w32 = get_eclock() - t0;
    fmt_mbs(res->w32, dt_w32, total_bytes);

    // Write 16
    t0 = get_eclock();
    for (int r = 0; r < repeats; r++) bench_write16((volatile UWORD *)addr, ops16);
    ULONG dt_w16 = get_eclock() - t0;
    fmt_mbs(res->w16, dt_w16, total_bytes);
}

int main(void) {
    printf("=======================================================================\n");
    printf("     Amiga 1200 / PiStorm32-lite Comprehensive Memory Benchmark\n");
    printf("=======================================================================\n\n");

    if (!init_timer()) {
        printf("Failed to init EClock timer!\n");
        return 20;
    }
    printf("E-Clock Frequency: %lu Hz\n\n", eclock_freq);

    printf("--- Exec Memory List (ExecBase->MemList) ---\n");
    Forbid();
    struct Node *node;
    for (node = SysBase->MemList.lh_Head; node->ln_Succ != NULL; node = node->ln_Succ) {
        struct MemHeader *mh = (struct MemHeader *)node;
        const char *t = "FAST";
        if (mh->mh_Attributes & MEMF_CHIP) t = "CHIP";
        printf("  [%-4s] %-18s @ 0x%08lX-0x%08lX | Size: %7lu KB | Free: %7lu KB | Pri: %3d\n",
               t, mh->mh_Node.ln_Name ? mh->mh_Node.ln_Name : "unnamed",
               (ULONG)mh->mh_Lower, (ULONG)mh->mh_Upper,
               ((ULONG)mh->mh_Upper - (ULONG)mh->mh_Lower) / 1024,
               mh->mh_Free / 1024, (int)mh->mh_Node.ln_Pri);
    }
    Permit();
    printf("\n");

    struct TestResult results[3];
    int num_results = 0;

    // Test 1: PCMCIA SRAM (allocate 512 KB safely from the card.resource pool)
    APTR sram_buf = AllocAbs(512 * 1024, (APTR)0x00610000);
    if (!sram_buf) {
        // Fallback: direct pointer if not claimed
        sram_buf = (APTR)0x00610000;
    }
    run_test(&results[num_results++], "PCMCIA SRAM (Gayle 16-bit bus)", sram_buf, 512 * 1024, 2);
    if (sram_buf && (ULONG)sram_buf == 0x00610000) FreeMem(sram_buf, 512 * 1024);

    // Test 2: Motherboard Chip RAM
    APTR chip_buf = AllocMem(256 * 1024, MEMF_CHIP);
    if (chip_buf) {
        run_test(&results[num_results++], "Motherboard Chip RAM (Alice 16-bit)", chip_buf, 256 * 1024, 4);
        FreeMem(chip_buf, 256 * 1024);
    }

    // Test 3: PiStorm Fast RAM
    APTR fast_buf = AllocMem(1024 * 1024, MEMF_FAST);
    if (fast_buf) {
        run_test(&results[num_results++], "PiStorm Fast RAM (ARM 32-bit DDR)", fast_buf, 1024 * 1024, 10);
        FreeMem(fast_buf, 1024 * 1024);
    }

    printf("=======================================================================\n");
    printf("                         BENCHMARK RESULTS\n");
    printf("=======================================================================\n");
    printf("%-35s | Read 32-bit | Read 16-bit | Write 32-bit | Write 16-bit\n", "Memory Region");
    printf("------------------------------------+-------------+-------------+--------------+-------------\n");
    for (int i = 0; i < num_results; i++) {
        printf("%-35s | %8s MB/s| %8s MB/s|  %8s MB/s| %8s MB/s\n",
               results[i].name,
               results[i].r32, results[i].r16,
               results[i].w32, results[i].w16);
    }
    printf("=======================================================================\n");

    cleanup_timer();
    return 0;
}
