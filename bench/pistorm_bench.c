#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/timer.h>
#include <proto/expansion.h>
#include <devices/timer.h>
#include <libraries/configvars.h>
#include <exec/memory.h>
#include <stdio.h>
#include <stdlib.h>

struct Device *TimerBase = NULL;
struct ExpansionBase *ExpansionBase = NULL;

static struct MsgPort *timerPort = NULL;
static struct timerequest *timerReq = NULL;
static ULONG eclock_freq = 0;

// Virtual Zorro-II Hardware Register Map (Slave 0)
#define ZREG_MAGIC       0x00
#define ZREG_DEVINFO     0x04
#define ZREG_STATUS      0x08
#define ZREG_SCRATCHPAD  0x0C
#define ZREG_INT_STATUS  0x10
#define ZREG_INT_ENABLE  0x14
#define ZREG_INT_FORCE   0x18
#define ZREG_PREF_CTRL   0x1C
#define ZREG_DIAG_STATUS 0x20
#define ZREG_PREF_LAUNCH 0x24
#define ZREG_PREF_HIT    0x28
#define ZREG_BUS_CAPTURE 0x2C

static volatile ULONG *zorro_dev = NULL;

static void print_diag_status(const char *label) {
    if (!zorro_dev) return;
    ULONG st = zorro_dev[ZREG_DIAG_STATUS / 4];
    ULONG launches = zorro_dev[ZREG_PREF_LAUNCH / 4];
    ULONG hits = zorro_dev[ZREG_PREF_HIT / 4];
    ULONG pref_ctrl = zorro_dev[ZREG_PREF_CTRL / 4];
    ULONG bus_cap = zorro_dev[ZREG_BUS_CAPTURE / 4];

    int halt_sync      = (st >> 31) & 1;
    int reset_sync     = (st >> 30) & 1;
    int req_hit_cur    = (st >> 29) & 1;
    int req_int        = (st >> 28) & 1;
    int req_act        = (st >> 27) & 1;
    int term_norm      = (st >> 26) & 1;
    ULONG dsack        = (st >> 24) & 0x3;
    int is_bm          = (st >> 23) & 1;
    int pref_eff       = (st >> 22) & 1;
    int next_pref      = (st >> 21) & 1;
    int chained_pref   = (st >> 20) & 1;
    int can_pref       = (st >> 19) & 1;
    int pref_valid     = (st >> 18) & 1;
    int is_pref_cyc    = (st >> 17) & 1;
    ULONG req_hit      = (st >> 15) & 0x3;
    int pref_eligible  = (st >> 14) & 1;
    ULONG size         = (st >> 12) & 0x3;
    ULONG port_width   = (st >> 10) & 0x3;
    ULONG fsm_state    = st & 0x3FF;

    ULONG cyc_cnt   = (bus_cap >> 24) & 0xFF;
    ULONG addr_lo   = (bus_cap >> 16) & 0xFF;
    int last_rw     = (bus_cap >> 15) & 1;
    int last_norm   = (bus_cap >> 14) & 1;
    int last_sz_le  = (bus_cap >> 13) & 1;
    int term_elig   = (bus_cap >> 12) & 1;
    ULONG size_s5   = (bus_cap >> 10) & 3;
    ULONG pw_s5     = (bus_cap >> 8) & 3;
    ULONG dsack_s5  = (bus_cap >> 6) & 3;
    ULONG dsack_s4  = (bus_cap >> 4) & 3;
    ULONG dsack_trm = (bus_cap >> 2) & 3;
    ULONG live_dsk  = bus_cap & 3;

    const char *pw_str = (port_width == 3) ? "32-bit" : (port_width == 1) ? "16-bit" : (port_width == 0) ? "8-bit" : "unknown";
    const char *pw_s5_str = (pw_s5 == 3) ? "32-bit" : (pw_s5 == 1) ? "16-bit" : (pw_s5 == 0) ? "8-bit" : "unknown";

    printf("  [FPGA DIAG: %s]\n", label);
    printf("    Raw DIAG_STATUS=0x%08lX | PREF_CTRL=0x%08lX\n", st, pref_ctrl);
    printf("    Telemetry: Launches=%lu | Hits=%lu | Live State=0x%03lX\n", launches, hits, fsm_state);
    printf("    Hardware:  Port=%s | Size=%lu | /DSACK=0x%lX | BusMaster=%d | HaltSync=%d | ResetSync=%d | TermNorm=%d\n",
           pw_str, size, dsack, is_bm, halt_sync, reset_sync, term_norm);
    printf("    Prefetch:  EffEn=%d | CtrlEn=%lu | Elig=%d | CanPref=%d | Valid=%d | InCyc=%d | HitBits=0x%lX\n",
           pref_eff, pref_ctrl, pref_eligible, can_pref, pref_valid, is_pref_cyc, req_hit);
    printf("    Flow:      NextAllowed=%d | ChainedAllowed=%d | ReqAct=%d | ReqInt=%d | ReqHit=%d\n",
           next_pref, chained_pref, req_act, req_int, req_hit_cur);
    printf("    Physical Bus Capture (+$2C = 0x%08lX):\n", bus_cap);
    printf("      Cycles=%lu | LastAddr=0x..%02lX | RW=%s | TermNorm=%d | SzLe=%d | TermElig=%d\n",
           cyc_cnt, addr_lo, last_rw ? "READ" : "WRITE", last_norm, last_sz_le, term_elig);
    printf("      PortAtS5=%s (%lu) | SizeAtS5=%lu | /DSACK: @Term=0x%lX, @S4=0x%lX, @S5=0x%lX, Live=0x%lX\n",
           pw_s5_str, pw_s5, size_s5, dsack_trm, dsack_s4, dsack_s5, live_dsk);
}

static int init_timer(void) {
    timerPort = CreateMsgPort();
    if (!timerPort) return 0;

    timerReq = (struct timerequest *)CreateIORequest(timerPort, sizeof(struct timerequest));
    if (!timerReq) {
        DeleteMsgPort(timerPort);
        timerPort = NULL;
        return 0;
    }

    if (OpenDevice(TIMERNAME, UNIT_ECLOCK, (struct IORequest *)timerReq, 0) != 0) {
        DeleteIORequest((struct IORequest *)timerReq);
        DeleteMsgPort(timerPort);
        timerReq = NULL;
        timerPort = NULL;
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

static void fmt_ns(char *buf, ULONG ticks, ULONG count) {
    if (!count || !eclock_freq) { sprintf(buf, "0.0"); return; }
    unsigned long long total_ns = ((unsigned long long)ticks * 1000000000ULL) / eclock_freq;
    unsigned long long ns_op_x10 = (total_ns * 10ULL) / count;
    sprintf(buf, "%lu.%lu", (ULONG)(ns_op_x10 / 10), (ULONG)(ns_op_x10 % 10));
}

static void fmt_mbs(char *buf, ULONG ticks, ULONG bytes) {
    if (!ticks) { sprintf(buf, "0.0"); return; }
    unsigned long long mbs_x10 = ((unsigned long long)bytes * (unsigned long long)eclock_freq * 10ULL) / ((unsigned long long)ticks * 1048576ULL);
    sprintf(buf, "%lu.%lu", (ULONG)(mbs_x10 / 10), (ULONG)(mbs_x10 % 10));
}

static ULONG ticks_to_ns_int(ULONG ticks, ULONG count) {
    if (!count || !eclock_freq) return 0;
    return (ULONG)(((unsigned long long)ticks * 1000000000ULL) / ((unsigned long long)eclock_freq * count));
}

// ---------------------------------------------------------------------------
// Assembly Benchmark Kernels
// ---------------------------------------------------------------------------

// 1. Sequential 32-bit Read
static void bench_readl_seq(volatile ULONG *buf, ULONG iters) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.l (%0)+, d0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 2. Strided 32-bit Read (+32 bytes stride: 100% prefetch miss)
static void bench_readl_stride(volatile ULONG *buf, ULONG iters) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.l (%0), d0\n\t"
        "lea    32(%0), %0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 3. Backward 32-bit Read (-4 bytes: 100% prefetch miss)
static void bench_readl_back(volatile ULONG *buf, ULONG iters) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.l -(%0), d0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 4. Sequential 16-bit Read
static void bench_readw_seq(volatile UWORD *buf, ULONG iters) {
    __asm__ __volatile__ (
        "1:\n\t"
        "move.w (%0)+, d0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 5. Sequential 32-bit Write
static void bench_writel_seq(volatile ULONG *buf, ULONG iters) {
    __asm__ __volatile__ (
        "moveq  #0, d0\n\t"
        "1:\n\t"
        "move.l d0, (%0)+\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 6. Strided 32-bit Write
static void bench_writel_stride(volatile ULONG *buf, ULONG iters) {
    __asm__ __volatile__ (
        "moveq  #0, d0\n\t"
        "1:\n\t"
        "move.l d0, (%0)\n\t"
        "lea    32(%0), %0\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        :
        : "d0", "cc", "memory"
    );
}

// 7. Pure CPU ALU delay (10, 50, 100 instructions)
static void bench_pure_alu(ULONG iters, ULONG count_ops) {
    volatile ULONG c = count_ops;
    __asm__ __volatile__ (
        "moveq #1, d0\n\t"
        "moveq #1, d1\n\t"
        "moveq #2, d2\n\t"
        "1:\n\t"
        "move.l %1, d3\n\t"
        "2:\n\t"
        "add.l  d0, d1\n\t"
        "eor.l  d1, d2\n\t"
        "subq.l #1, d3\n\t"
        "bne.b  2b\n\t"
        "subq.l #1, %0\n\t"
        "bne.b  1b\n\t"
        : "+d"(iters)
        : "d"(c)
        : "d0", "d1", "d2", "d3", "cc"
    );
}

// 8. Sequential Read + CPU ALU delay
static void bench_readl_seq_alu(volatile ULONG *buf, ULONG iters, ULONG count_ops) {
    volatile ULONG c = count_ops;
    __asm__ __volatile__ (
        "moveq #1, d1\n\t"
        "moveq #2, d2\n\t"
        "1:\n\t"
        "move.l (%0)+, d0\n\t"
        "move.l %2, d3\n\t"
        "2:\n\t"
        "add.l  d0, d1\n\t"
        "eor.l  d1, d2\n\t"
        "subq.l #1, d3\n\t"
        "bne.b  2b\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        : "d"(c)
        : "d0", "d1", "d2", "d3", "cc", "memory"
    );
}

// 9. Strided Read + CPU ALU delay
static void bench_readl_stride_alu(volatile ULONG *buf, ULONG iters, ULONG count_ops) {
    volatile ULONG c = count_ops;
    __asm__ __volatile__ (
        "moveq #1, d1\n\t"
        "moveq #2, d2\n\t"
        "1:\n\t"
        "move.l (%0), d0\n\t"
        "lea    32(%0), %0\n\t"
        "move.l %2, d3\n\t"
        "2:\n\t"
        "add.l  d0, d1\n\t"
        "eor.l  d1, d2\n\t"
        "subq.l #1, d3\n\t"
        "bne.b  2b\n\t"
        "subq.l #1, %1\n\t"
        "bne.b  1b\n\t"
        : "+a"(buf), "+d"(iters)
        : "d"(c)
        : "d0", "d1", "d2", "d3", "cc", "memory"
    );
}

#define BUF_SIZE (256 * 1024)
#define NUM_OPS_32 (BUF_SIZE / 4)       // 65536
#define NUM_OPS_STRIDE (BUF_SIZE / 32)  // 8192
#define NUM_OPS_16 (BUF_SIZE / 2)       // 131072

static void run_mem_tests(const char *name, ULONG *buf, int is_chip) {
    ULONG t_start, t_end, dt;
    char s_ns[32], s_mbs[32];

    printf("============================================================\n");
    printf("  Testing %s (256 KB at 0x%08lX)\n", name, (ULONG)buf);
    printf("============================================================\n");

    if (is_chip && zorro_dev) {
        zorro_dev[ZREG_PREF_LAUNCH / 4] = 0; // Clear counters
        print_diag_status("Baseline Before Read Tests");
    }

    // 1. Sequential Read 32-bit
    t_start = get_eclock();
    bench_readl_seq(buf, NUM_OPS_32);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_32);
    fmt_mbs(s_mbs, dt, BUF_SIZE);
    printf("1. Sequential 32-bit Read:    %7s ns/op  |  %6s MB/s\n", s_ns, s_mbs);

    if (is_chip && zorro_dev) {
        print_diag_status("After Seq 32-bit Read (64K ops)");
        zorro_dev[ZREG_PREF_LAUNCH / 4] = 0; // Reset counters
    }

    // 2. Strided Read 32-bit (+32 bytes)
    t_start = get_eclock();
    bench_readl_stride(buf, NUM_OPS_STRIDE);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_STRIDE);
    fmt_mbs(s_mbs, dt, NUM_OPS_STRIDE * 4);
    printf("2. Strided 32-bit Read (+32): %7s ns/op  |  %6s MB/s\n", s_ns, s_mbs);

    if (is_chip && zorro_dev) {
        print_diag_status("After Strided 32-bit Read (+32)");
        zorro_dev[ZREG_PREF_LAUNCH / 4] = 0; // Reset counters
    }

    // 3. Backward Read 32-bit (-4 bytes)
    t_start = get_eclock();
    bench_readl_back(buf + NUM_OPS_32, NUM_OPS_32);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_32);
    fmt_mbs(s_mbs, dt, BUF_SIZE);
    printf("3. Backward 32-bit Read (-4): %7s ns/op  |  %6s MB/s\n", s_ns, s_mbs);

    if (is_chip && zorro_dev) {
        print_diag_status("After Backward 32-bit Read (-4)");
    }

    // 4. Sequential Read 16-bit
    t_start = get_eclock();
    bench_readw_seq((UWORD *)buf, NUM_OPS_16);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_16);
    fmt_mbs(s_mbs, dt, BUF_SIZE);
    printf("4. Sequential 16-bit Read:    %7s ns/op  |  %6s MB/s\n", s_ns, s_mbs);

    // 5. Sequential Write 32-bit
    t_start = get_eclock();
    bench_writel_seq(buf, NUM_OPS_32);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_32);
    fmt_mbs(s_mbs, dt, BUF_SIZE);
    printf("5. Sequential 32-bit Write:   %7s ns/op  |  %6s MB/s\n", s_ns, s_mbs);

    // 6. Strided Write 32-bit (+32 bytes)
    t_start = get_eclock();
    bench_writel_stride(buf, NUM_OPS_STRIDE);
    t_end = get_eclock();
    dt = t_end - t_start;
    fmt_ns(s_ns, dt, NUM_OPS_STRIDE);
    fmt_mbs(s_mbs, dt, NUM_OPS_STRIDE * 4);
    printf("6. Strided 32-bit Write (+32): %6s ns/op  |  %6s MB/s\n\n", s_ns, s_mbs);

    if (is_chip) {
        printf("--- Prefetch Overlap Sweep (CPU Think-Time Concurrency) ---\n");
        printf("Testing whether background prefetch hides CPU computation time:\n\n");
        printf("  ALU Loops |  Pure CPU  |  Seq Read  |  Strided Read | Hidden Time\n");
        printf("------------+-----------+------------+---------------+-------------\n");

        static const ULONG alu_counts[] = {5, 20, 50, 100, 250, 500};
        for (int i = 0; i < 6; i++) {
            ULONG ops = alu_counts[i];

            // Pure ALU
            t_start = get_eclock();
            bench_pure_alu(NUM_OPS_STRIDE, ops);
            t_end = get_eclock();
            ULONG t_alu = ticks_to_ns_int(t_end - t_start, NUM_OPS_STRIDE);

            // Seq Read + ALU
            t_start = get_eclock();
            bench_readl_seq_alu(buf, NUM_OPS_STRIDE, ops);
            t_end = get_eclock();
            ULONG t_seq_a = ticks_to_ns_int(t_end - t_start, NUM_OPS_STRIDE);

            // Strided Read + ALU
            t_start = get_eclock();
            bench_readl_stride_alu(buf, NUM_OPS_STRIDE, ops);
            t_end = get_eclock();
            ULONG t_str_a = ticks_to_ns_int(t_end - t_start, NUM_OPS_STRIDE);

            long hidden = (long)t_str_a - (long)t_seq_a;
            if (hidden < 0) hidden = 0;

            printf("  %5lu     | %7lu ns | %8lu ns | %9lu ns   | %7ld ns\n",
                   ops, t_alu, t_seq_a, t_str_a, hidden);
        }

        printf("\nInterpretation:\n");
        printf("* If 'Hidden Time' > 0: CPU work is running IN PARALLEL with the\n");
        printf("  FPGA background prefetch on the Amiga motherboard bus!\n");
        printf("* If 'Hidden Time' ~ 0: Background prefetch is not overlapping.\n\n");
    }
}

int main(int argc, char **argv) {
    printf("============================================================\n");
    printf("  PiStorm32-lite Comprehensive Bus & Prefetch Benchmark\n");
    printf("  Compiled with m68k-amigaos-gcc\n");
    printf("============================================================\n\n");

    if (!init_timer()) {
        printf("FATAL: Could not initialize timer.device (UNIT_ECLOCK)!\n");
        return 20;
    }
    printf("High-Resolution Timer: E-Clock frequency = %lu Hz (%lu ns/tick)\n\n",
           eclock_freq, 1000000000UL / eclock_freq);

    // Check for PiStorm Virtual Zorro Board
    ExpansionBase = (struct ExpansionBase *)OpenLibrary("expansion.library", 36);
    if (ExpansionBase) {
        struct ConfigDev *cd = NULL;
        while ((cd = FindConfigDev(cd, 28020, 50)) != NULL) {
            printf("[AUTOCONFIG] PiStorm Virtual Zorro-II Board detected!\n");
            printf("             Base Address: 0x%08lX | Size: %ld KB\n",
                   (ULONG)cd->cd_BoardAddr, cd->cd_BoardSize / 1024);

            zorro_dev = (volatile ULONG *)cd->cd_BoardAddr;

            ULONG magic = zorro_dev[ZREG_MAGIC / 4];
            ULONG devinfo = zorro_dev[ZREG_DEVINFO / 4];
            ULONG status = zorro_dev[ZREG_STATUS / 4];

            printf("             Magic: 0x%08lX (\"%c%c%c%c\") | DevInfo: 0x%08lX | Status: 0x%08lX\n",
                   magic,
                   (char)(magic >> 24), (char)(magic >> 16), (char)(magic >> 8), (char)magic,
                   devinfo, status);

            // Test scratchpad register at offset 0x0C
            zorro_dev[ZREG_SCRATCHPAD / 4] = 0xCAFEBABE;
            ULONG r = zorro_dev[ZREG_SCRATCHPAD / 4];
            printf("             Scratchpad ($0C) R/W Test: 0x%08lX %s\n",
                   r, (r == 0xCAFEBABE) ? "(PASSED)" : "(FAILED)");

            print_diag_status("Post-AutoConfig Initial State");
            printf("\n");
        }
        CloseLibrary((struct Library *)ExpansionBase);
    }

    // Allocate Buffers
    ULONG *chip_buf = (ULONG *)AllocMem(BUF_SIZE, MEMF_CHIP | MEMF_CLEAR);
    if (!chip_buf) {
        printf("FATAL: Failed to allocate 256 KB of Chip RAM!\n");
        cleanup_timer();
        return 20;
    }

    if (zorro_dev) {
        // Clear counters
        zorro_dev[ZREG_PREF_LAUNCH / 4] = 0;
        volatile ULONG *chip_ptr = chip_buf;

        // Step 1: Read ONE 32-bit longword from Chip RAM
        volatile ULONG val0 = chip_ptr[0];

        // Step 2: Idle delay to give FPGA bus master idle cycle time to launch speculative read
        for (volatile int i = 0; i < 200; i++);

        // Read intermediate diagnostics
        ULONG st_after_delay = zorro_dev[ZREG_DIAG_STATUS / 4];
        ULONG launch_after_delay = zorro_dev[ZREG_PREF_LAUNCH / 4];

        // Step 3: Read sequential next 32-bit longword (target of prefetch)
        volatile ULONG val1 = chip_ptr[1];

        // Step 4: Sample hardware diagnostics
        ULONG st_after_hit = zorro_dev[ZREG_DIAG_STATUS / 4];
        ULONG launch_after_hit = zorro_dev[ZREG_PREF_LAUNCH / 4];
        ULONG hit_after_hit = zorro_dev[ZREG_PREF_HIT / 4];

        printf("--- Prefetch Hardware Diagnostic Test ---\n");
        printf("  Read longword 0: 0x%08lX from Chip RAM at 0x%08lX\n", val0, (ULONG)&chip_ptr[0]);
        printf("  After 200-cycle idle delay: Launches=%lu | Status=0x%08lX\n", launch_after_delay, st_after_delay);
        printf("  Read longword 1: 0x%08lX from Chip RAM at 0x%08lX\n", val1, (ULONG)&chip_ptr[1]);
        printf("  After sequential read:      Launches=%lu | Hits=%lu | Status=0x%08lX\n",
               launch_after_hit, hit_after_hit, st_after_hit);
        print_diag_status("After Single Prefetch Test");
        printf("\n");
    }

    ULONG *fast_buf = (ULONG *)AllocMem(BUF_SIZE, MEMF_FAST | MEMF_CLEAR);

    // Raise task priority during benchmark to suppress OS task preemption
    struct Task *me = FindTask(NULL);
    BYTE old_pri = SetTaskPri(me, 20);

    // Run Chip RAM tests
    run_mem_tests("CHIP RAM", chip_buf, 1);

    // Run Fast RAM tests (if available)
    if (fast_buf) {
        run_mem_tests("FAST RAM (Emu68 / Pi Memory)", fast_buf, 0);
        FreeMem(fast_buf, BUF_SIZE);
    } else {
        printf("Note: No Fast RAM allocated, skipping Fast RAM baseline.\n");
    }

    // Restore priority and cleanup
    SetTaskPri(me, old_pri);
    FreeMem(chip_buf, BUF_SIZE);
    cleanup_timer();

    printf("============================================================\n");
    printf("  Benchmark Completed Successfully.\n");
    printf("============================================================\n");
    return 0;
}
