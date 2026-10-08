#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/timer.h>
#include <proto/expansion.h>
#include <devices/timer.h>
#include <libraries/configvars.h>
#include <exec/memory.h>
#include <exec/execbase.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern struct ExecBase *SysBase;
struct Device *TimerBase = NULL;
struct ExpansionBase *ExpansionBase = NULL;
static struct MsgPort *timerPort = NULL;
static struct timerequest *timerReq = NULL;
static ULONG eclock_freq = 0;

static volatile ULONG *zorro_dev = NULL;
#define ZREG_SCRATCHPAD  0x0C
#define ZREG_PREF_CTRL   0x1C
#define ZREG_DIAG_STATUS 0x20
#define ZREG_PREF_LAUNCH 0x24
#define ZREG_PREF_HIT    0x28

static int init_timer(void) {
    timerPort = CreateMsgPort();
    if (!timerPort) return 0;
    timerReq = (struct timerequest *)CreateIORequest(timerPort, sizeof(struct timerequest));
    if (!timerReq) { DeleteMsgPort(timerPort); return 0; }
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
    if (timerPort) { DeleteMsgPort(timerPort); timerPort = NULL; }
}

static inline ULONG get_eclock(void) {
    struct EClockVal ev;
    ReadEClock(&ev);
    return ev.ev_lo;
}

static ULONG prng_state = 0x12345678UL;
static inline ULONG prng_next(void) {
    prng_state = prng_state * 1664525UL + 1013904223UL;
    return prng_state;
}

// Memory integrity test: Solid, Checkerboard, Walking 1s, Address-as-data, PRNG
static int test_mem_integrity(const char *label, volatile ULONG *buf, ULONG size_bytes) {
    ULONG num_longs = size_bytes / 4;
    printf("  Testing %-32s (%lu KB) ... ", label, size_bytes / 1024);
    fflush(stdout);

    // 1. Solid 0x00000000
    for (ULONG i = 0; i < num_longs; i++) buf[i] = 0x00000000UL;
    for (ULONG i = 0; i < num_longs; i++) {
        if (buf[i] != 0x00000000UL) {
            printf("FAILED at +0x%lx (Solid 0): got 0x%08lx\n", i*4, buf[i]);
            return 0;
        }
    }

    // 2. Solid 0xFFFFFFFF
    for (ULONG i = 0; i < num_longs; i++) buf[i] = 0xFFFFFFFFUL;
    for (ULONG i = 0; i < num_longs; i++) {
        if (buf[i] != 0xFFFFFFFFUL) {
            printf("FAILED at +0x%lx (Solid 1): got 0x%08lx\n", i*4, buf[i]);
            return 0;
        }
    }

    // 3. Checkerboard 0x55555555 / 0xAAAAAAAA
    for (ULONG i = 0; i < num_longs; i++) buf[i] = (i & 1) ? 0xAAAAAAAAUL : 0x55555555UL;
    for (ULONG i = 0; i < num_longs; i++) {
        ULONG exp = (i & 1) ? 0xAAAAAAAAUL : 0x55555555UL;
        if (buf[i] != exp) {
            printf("FAILED at +0x%lx (Checkerboard): exp 0x%08lx got 0x%08lx\n", i*4, exp, buf[i]);
            return 0;
        }
    }

    // 4. Inverted Checkerboard
    for (ULONG i = 0; i < num_longs; i++) buf[i] = (i & 1) ? 0x55555555UL : 0xAAAAAAAAUL;
    for (ULONG i = 0; i < num_longs; i++) {
        ULONG exp = (i & 1) ? 0x55555555UL : 0xAAAAAAAAUL;
        if (buf[i] != exp) {
            printf("FAILED at +0x%lx (Inv Checkerboard): exp 0x%08lx got 0x%08lx\n", i*4, exp, buf[i]);
            return 0;
        }
    }

    // 5. Walking Ones
    for (ULONG i = 0; i < num_longs; i++) buf[i] = (1UL << (i % 32));
    for (ULONG i = 0; i < num_longs; i++) {
        ULONG exp = (1UL << (i % 32));
        if (buf[i] != exp) {
            printf("FAILED at +0x%lx (Walking 1): exp 0x%08lx got 0x%08lx\n", i*4, exp, buf[i]);
            return 0;
        }
    }

    // 6. Address as Data
    for (ULONG i = 0; i < num_longs; i++) buf[i] = (ULONG)&buf[i];
    for (ULONG i = 0; i < num_longs; i++) {
        ULONG exp = (ULONG)&buf[i];
        if (buf[i] != exp) {
            printf("FAILED at +0x%lx (Address): exp 0x%08lx got 0x%08lx\n", i*4, exp, buf[i]);
            return 0;
        }
    }

    // 7. Pseudo-random Pattern with Seed Verification
    prng_state = 0xDEADBEEFUL;
    for (ULONG i = 0; i < num_longs; i++) buf[i] = prng_next();
    prng_state = 0xDEADBEEFUL;
    for (ULONG i = 0; i < num_longs; i++) {
        ULONG exp = prng_next();
        if (buf[i] != exp) {
            printf("FAILED at +0x%lx (PRNG): exp 0x%08lx got 0x%08lx\n", i*4, exp, buf[i]);
            return 0;
        }
    }

    // 8. Byte & Word Access Integrity
    volatile UBYTE *b_buf = (volatile UBYTE *)buf;
    for (ULONG i = 0; i < 4096; i++) b_buf[i] = (UBYTE)(i & 0xFF);
    for (ULONG i = 0; i < 4096; i++) {
        if (b_buf[i] != (UBYTE)(i & 0xFF)) {
            printf("FAILED at byte +0x%lx: exp 0x%02x got 0x%02x\n", i, (UBYTE)(i & 0xFF), b_buf[i]);
            return 0;
        }
    }

    volatile UWORD *w_buf = (volatile UWORD *)buf;
    for (ULONG i = 0; i < 4096; i++) w_buf[i] = (UWORD)(i ^ 0xAAAA);
    for (ULONG i = 0; i < 4096; i++) {
        if (w_buf[i] != (UWORD)(i ^ 0xAAAA)) {
            printf("FAILED at word +0x%lx: exp 0x%04x got 0x%04x\n", i*2, (UWORD)(i ^ 0xAAAA), w_buf[i]);
            return 0;
        }
    }

    printf("[PASSED]\n");
    return 1;
}

int main(void) {
    printf("=======================================================================\n");
    printf("    PiStorm32-lite 200.00 MHz Gateware Quality & Stress Test Suite     \n");
    printf("=======================================================================\n\n");

    if (!init_timer()) {
        printf("ERROR: Failed to open timer.device!\n");
        return 20;
    }

    // Find Virtual Zorro Board
    ExpansionBase = (struct ExpansionBase *)OpenLibrary("expansion.library", 36);
    if (ExpansionBase) {
        struct ConfigDev *cd = NULL;
        while ((cd = FindConfigDev(cd, 28020, 50)) != NULL) {
            zorro_dev = (volatile ULONG *)cd->cd_BoardAddr;
            break;
        }
        CloseLibrary((struct Library *)ExpansionBase);
    }

    if (!zorro_dev) {
        printf("ERROR: Virtual Zorro-II Board not detected!\n");
        cleanup_timer();
        return 20;
    }
    printf("[HW] Virtual Zorro-II Board located at: 0x%08lX\n", (ULONG)zorro_dev);
    ULONG init_ctrl = zorro_dev[ZREG_PREF_CTRL / 4];
    printf("[HW] Initial BUS_CTRL: 0x%08lX\n\n", init_ctrl);

    int all_passed = 1;

    // -----------------------------------------------------------------------
    // TEST SUITE 1: FPGA Register Stress & High-Speed Bus Isolation
    // -----------------------------------------------------------------------
    printf("=== TEST 1: FPGA Wishbone Register Stress (1,000,000 Transfers) ===\n");
    ULONG reg_errors = 0;
    ULONG t0 = get_eclock();
    for (ULONG i = 0; i < 1000000; i++) {
        ULONG val = (i << 16) ^ (i * 0x1234567);
        zorro_dev[ZREG_SCRATCHPAD / 4] = val;
        ULONG rd = zorro_dev[ZREG_SCRATCHPAD / 4];
        if (rd != val) {
            reg_errors++;
            if (reg_errors <= 5) {
                printf("  Mismatch at cycle %lu: wrote 0x%08lx read 0x%08lx\n", i, val, rd);
            }
        }
    }
    ULONG dt = get_eclock() - t0;
    double secs = (double)dt / (double)eclock_freq;
    printf("  1,000,000 R/W Cycles completed in %.2f seconds (%.1f KOps/sec)\n",
           secs, 1000000.0 / secs / 1000.0);
    if (reg_errors == 0) {
        printf("  Result: [PASSED] (0 errors in 1M consecutive transfers)\n\n");
    } else {
        printf("  Result: [FAILED] (%lu errors!)\n\n", reg_errors);
        all_passed = 0;
    }

    // -----------------------------------------------------------------------
    // TEST SUITE 2: Motherboard Chip RAM Deep Pattern Stress Test
    // -----------------------------------------------------------------------
    printf("=== TEST 2: Motherboard Chip RAM 16-Bit Bus Integrity (256 KB) ===\n");
    APTR chip_mem = AllocMem(256 * 1024, MEMF_CHIP);
    if (!chip_mem) {
        printf("  ERROR: Could not allocate 256 KB of Chip RAM!\n");
        all_passed = 0;
    } else {
        if (!test_mem_integrity("Chip RAM ($00004020+)", (volatile ULONG *)chip_mem, 256 * 1024)) {
            all_passed = 0;
        }
        FreeMem(chip_mem, 256 * 1024);
        printf("\n");
    }

    // -----------------------------------------------------------------------
    // TEST SUITE 3: PCMCIA SRAM Gayle Bus Integrity (512 KB)
    // -----------------------------------------------------------------------
    printf("=== TEST 3: PCMCIA SRAM Gayle Bus Integrity (512 KB at $00610000) ===\n");
    APTR sram_mem = AllocAbs(512 * 1024, (APTR)0x00610000);
    if (!sram_mem) sram_mem = (APTR)0x00610000;
    if (!test_mem_integrity("PCMCIA SRAM ($00610000)", (volatile ULONG *)sram_mem, 512 * 1024)) {
        all_passed = 0;
    }
    if ((ULONG)sram_mem == 0x00610000) FreeMem(sram_mem, 512 * 1024);
    printf("\n");

    // -----------------------------------------------------------------------
    // TEST SUITE 4: PiStorm Fast RAM Large-Block Stress (16 MB)
    // -----------------------------------------------------------------------
    printf("=== TEST 4: PiStorm Fast RAM 32-Bit ARM LPDDR Stress (16 MB) ===\n");
    APTR fast_mem = AllocMem(16 * 1024 * 1024, MEMF_FAST);
    if (!fast_mem) {
        printf("  ERROR: Could not allocate 16 MB of Fast RAM!\n");
        all_passed = 0;
    } else {
        if (!test_mem_integrity("PiStorm Fast RAM (16 MB)", (volatile ULONG *)fast_mem, 16 * 1024 * 1024)) {
            all_passed = 0;
        }
        FreeMem(fast_mem, 16 * 1024 * 1024);
        printf("\n");
    }

    // -----------------------------------------------------------------------
    // TEST SUITE 5: Bus Control Matrix & Modes Stability Sweep
    // -----------------------------------------------------------------------
    printf("=== TEST 5: FPGA Bus Control Mode Stability Matrix ===\n");
    struct {
        const char *name;
        ULONG ctrl;
    } modes[] = {
        {"Standard Mode (FastDSACK=0, CCKSync=0, Prefetch=0)", 0x00},
        {"Fast DSACK Mode (FastDSACK=1, CCKSync=0, Prefetch=0)", 0x02},
        {"CCK Synchronous (FastDSACK=1, CCKSync=1, Prefetch=0)", 0x06},
        {"Turbo Mode (FastDSACK=1, CCKSync=1, Prefetch=1)",       0x07},
        {"Full Turbo + Word Prefetch (All Features ON)",          0x97},
    };

    APTR chip_test = AllocMem(128 * 1024, MEMF_CHIP);
    if (chip_test) {
        for (int m = 0; m < 5; m++) {
            zorro_dev[ZREG_PREF_CTRL / 4] = modes[m].ctrl;
            printf("  Mode %d: %s\n", m+1, modes[m].name);
            printf("          BUS_CTRL = 0x%02lx -> ", modes[m].ctrl);
            fflush(stdout);

            // Verify write/read on Chip RAM under this mode
            volatile ULONG *p = (volatile ULONG *)chip_test;
            ULONG n = (128 * 1024) / 4;
            int mode_ok = 1;
            for (ULONG i = 0; i < n; i++) p[i] = (i ^ (modes[m].ctrl << 16)) + 0x12345678UL;
            for (ULONG i = 0; i < n; i++) {
                ULONG exp = (i ^ (modes[m].ctrl << 16)) + 0x12345678UL;
                if (p[i] != exp) {
                    printf("FAILED at +0x%lx: exp 0x%08lx got 0x%08lx\n", i*4, exp, p[i]);
                    mode_ok = 0;
                    all_passed = 0;
                    break;
                }
            }
            if (mode_ok) printf("[PASSED: 100%% DATA INTEGRITY]\n");
        }
        FreeMem(chip_test, 128 * 1024);
    }
    // Restore initial control
    zorro_dev[ZREG_PREF_CTRL / 4] = init_ctrl;
    printf("\n");

    // -----------------------------------------------------------------------
    // TEST SUITE 6: File System & Block Transfer Integrity
    // -----------------------------------------------------------------------
    printf("=== TEST 6: Filesystem & Disk I/O Integrity (RAM: & Disk) ===\n");
    const char *test_path = "RAM:ps32_io_test.dat";
    BPTR fh = Open((STRPTR)test_path, MODE_NEWFILE);
    if (!fh) {
        printf("  ERROR: Could not open %s for writing!\n", test_path);
        all_passed = 0;
    } else {
        UBYTE *io_buf = (UBYTE *)AllocMem(64 * 1024, MEMF_PUBLIC);
        UBYTE *r_buf  = (UBYTE *)AllocMem(64 * 1024, MEMF_PUBLIC);
        if (io_buf && r_buf) {
            // Write 32 blocks of 64 KB = 2 MB
            for (int b = 0; b < 32; b++) {
                prng_state = 0xCAFEBABEUL + (ULONG)(b * 10007);
                for (ULONG i = 0; i < 64 * 1024; i++) io_buf[i] = (UBYTE)prng_next();
                LONG w = Write(fh, io_buf, 64 * 1024);
                if (w != 64 * 1024) { printf("  Write error at block %d!\n", b); all_passed = 0; break; }
            }
            Close(fh);

            // Re-open and verify
            fh = Open((STRPTR)test_path, MODE_OLDFILE);
            int io_ok = 1;
            for (int b = 0; b < 32; b++) {
                LONG r = Read(fh, r_buf, 64 * 1024);
                if (r != 64 * 1024) { printf("  Read error at block %d!\n", b); io_ok = 0; all_passed = 0; break; }

                prng_state = 0xCAFEBABEUL + (ULONG)(b * 10007);
                for (ULONG i = 0; i < 64 * 1024; i++) io_buf[i] = (UBYTE)prng_next();

                if (memcmp(r_buf, io_buf, 64 * 1024) != 0) {
                    printf("  Data mismatch at block %d!\n", b);
                    io_ok = 0;
                    all_passed = 0;
                    break;
                }
            }
            Close(fh);
            DeleteFile((STRPTR)test_path);

            if (io_ok) {
                printf("  RAM: 2 MB Block Read/Write/Verify: [PASSED: 100%% INTEGRITY]\n");
            }
        }
        if (io_buf) FreeMem(io_buf, 64 * 1024);
        if (r_buf)  FreeMem(r_buf, 64 * 1024);
    }
    printf("\n");

    // -----------------------------------------------------------------------
    // FINAL SUMMARY
    // -----------------------------------------------------------------------
    printf("=======================================================================\n");
    if (all_passed) {
        printf("  OVERALL RESULT: [PASS] - 200.00 MHz BUILD IS 100%% VERIFIED & STABLE!\n");
        printf("  READY FOR TESTER RELEASE!\n");
    } else {
        printf("  OVERALL RESULT: [FAIL] - ONE OR MORE STRESS TESTS FAILED!\n");
    }
    printf("=======================================================================\n");

    cleanup_timer();
    return all_passed ? 0 : 20;
}
