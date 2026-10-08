/*
 * PiStorm32-lite Bus Control Utility for AmigaOS
 * Target: Motorola 68020+ / AmigaOS 2.0+ (V36+)
 *
 * Controls the PiStorm32 Virtual Zorro-II BUS_CTRL register (+0x1C):
 *   Bit 0: prefetch_ctrl_en     (32-bit speculative read prefetch)
 *   Bit 1: fast_dsack_en        (Fast DSACK termination, skips S4_NOP)
 *   Bit 2: cck_sync_en          (7.09 MHz CCK phase synchronization)
 *   Bit 3: force_phase_invert   (Manual CCK phase invert)
 *   Bit 7: enable_word_prefetch (16-bit word read prefetch)
 */

#include <exec/types.h>
#include <exec/libraries.h>
#include <libraries/expansionbase.h>
#include <libraries/configvars.h>
#include <intuition/intuition.h>

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/intuition.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ZORRO_MANUF_ID    28020
#define ZORRO_PROD_ID     50

#define ZREG_MAGIC        0x00
#define ZREG_DEVINFO      0x04
#define ZREG_STATUS       0x08
#define ZREG_SCRATCHPAD   0x0C
#define ZREG_PREF_CTRL    0x1C
#define ZREG_GIT_HASH     0x38
#define ZREG_BUILD_DATE   0x3C
#define ZREG_BUILD_INFO   0x40

#define MAGIC_PS32        0x50533332 /* 'PS32' */

#define CTRL_PREFETCH_32  (1 << 0)
#define CTRL_FAST_DSACK   (1 << 1)
#define CTRL_CCK_SYNC     (1 << 2)
#define CTRL_PHASE_INV    (1 << 3)
#define CTRL_PREFETCH_16  (1 << 7)

#define VAL_NO_FAST_DSACK (CTRL_PREFETCH_32 | CTRL_PREFETCH_16) /* 0x81: No FastDSACK, No CCK Sync */          /* 0x85 */
#define VAL_TURBO_OFF     (0x00)                                                         /* 0x00 */
#define VAL_TURBO_ON      (CTRL_PREFETCH_32 | CTRL_FAST_DSACK | CTRL_CCK_SYNC | CTRL_PREFETCH_16) /* 0x87 */

struct ExpansionBase *ExpansionBase = NULL;
struct IntuitionBase *IntuitionBase = NULL;

static void show_gui_message(const char *title, const char *msg) {
    if (IntuitionBase) {
        struct EasyStruct es;
        es.es_StructSize = sizeof(struct EasyStruct);
        es.es_Flags = 0;
        es.es_Title = (UBYTE *)title;
        es.es_TextFormat = (UBYTE *)msg;
        es.es_GadgetFormat = (UBYTE *)"OK";
        EasyRequestArgs(NULL, &es, NULL, NULL);
    }
}

static void print_active_bits(ULONG val) {
    printf("  [%c] Fast DSACK (S4_NOP Bypass): %s\n",
           (val & CTRL_FAST_DSACK) ? '+' : '-',
           (val & CTRL_FAST_DSACK) ? "ENABLED" : "DISABLED (Mediator TX Safe)");
    printf("  [%c] 32-Bit Speculative Prefetch: %s\n",
           (val & CTRL_PREFETCH_32) ? '+' : '-',
           (val & CTRL_PREFETCH_32) ? "ENABLED" : "DISABLED");
    printf("  [%c] 16-Bit Word Read Prefetch:   %s\n",
           (val & CTRL_PREFETCH_16) ? '+' : '-',
           (val & CTRL_PREFETCH_16) ? "ENABLED" : "DISABLED");
    printf("  [%c] 7.09 MHz CCK Phase Sync:     %s\n",
           (val & CTRL_CCK_SYNC) ? '+' : '-',
           (val & CTRL_CCK_SYNC) ? "ENABLED" : "DISABLED");
    if (val & CTRL_PHASE_INV) {
        printf("  [!] Manual Phase Invert:         ACTIVE\n");
    }
}

int main(int argc, char **argv) {
    int is_workbench = (argc == 0);
    int target_mode = -1; /* -1: default according to build target, 0: status only, 1: apply target_val */
    ULONG target_val = 0;
    const char *action_desc = "Unknown Action";

#if defined(MODE_NO_FAST_DSACK)
    target_mode = 1;
    target_val = VAL_NO_FAST_DSACK;
    action_desc = "Fast DSACK Disabled (Mediator TX Safe)";
#elif defined(MODE_TURBO_OFF)
    target_mode = 1;
    target_val = VAL_TURBO_OFF;
    action_desc = "All Accelerations Disabled (Golden Parity)";
#elif defined(MODE_TURBO_ON)
    target_mode = 1;
    target_val = VAL_TURBO_ON;
    action_desc = "Ultra Turbo Mode Enabled";
#endif

    /* CLI argument parsing if not built as fixed action or if args provided */
    if (!is_workbench && argc > 1) {
        if (strcmp(argv[1], "nofastdsack") == 0 || strcmp(argv[1], "safemediator") == 0) {
            target_mode = 1;
            target_val = VAL_NO_FAST_DSACK;
            action_desc = "Fast DSACK Disabled (Mediator TX Safe)";
        } else if (strcmp(argv[1], "off") == 0 || strcmp(argv[1], "disable") == 0 || strcmp(argv[1], "stock") == 0) {
            target_mode = 1;
            target_val = VAL_TURBO_OFF;
            action_desc = "All Accelerations Disabled (Golden Parity)";
        } else if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "turbo") == 0 || strcmp(argv[1], "all") == 0) {
            target_mode = 1;
            target_val = VAL_TURBO_ON;
            action_desc = "Ultra Turbo Mode Enabled";
        } else if (strcmp(argv[1], "status") == 0 || strcmp(argv[1], "-s") == 0) {
            target_mode = 0; /* Just report status */
            action_desc = "Status Report";
        } else if (argv[1][0] == '$' || (argv[1][0] == '0' && (argv[1][1] == 'x' || argv[1][1] == 'X')) ||
                   (argv[1][0] >= '0' && argv[1][0] <= '9')) {
            const char *p = (argv[1][0] == '$') ? argv[1] + 1 : argv[1];
            target_val = strtoul(p, NULL, 0);
            target_mode = 1;
            action_desc = "Custom Value Applied";
        } else if (strcmp(argv[1], "help") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "?") == 0) {
            printf("Usage: %s [nofastdsack | off | on | status | <hex_val>]\n", argv[0]);
            printf("  nofastdsack : Disable Fast DSACK (fix Mediator TX glitches, keep prefetch)\n");
            printf("  off         : Turn OFF all accelerations (100%% Golden Reference mode)\n");
            printf("  on          : Turn ON Ultra Turbo mode (Prefetch + Fast DSACK + CCK Sync)\n");
            printf("  status      : Read and display live PiStorm32 bus control settings\n");
            return 0;
        }
    }

    ExpansionBase = (struct ExpansionBase *)OpenLibrary((UBYTE *)"expansion.library", 36);
    if (!ExpansionBase) {
        if (!is_workbench) printf("ERROR: Failed to open expansion.library V36+!\n");
        return 20;
    }

    IntuitionBase = (struct IntuitionBase *)OpenLibrary((UBYTE *)"intuition.library", 36);

    struct ConfigDev *cd = NULL;
    volatile ULONG *zdev = NULL;

    while ((cd = FindConfigDev(cd, ZORRO_MANUF_ID, ZORRO_PROD_ID)) != NULL) {
        volatile ULONG *base = (volatile ULONG *)cd->cd_BoardAddr;
        if (base && base[ZREG_MAGIC / 4] == MAGIC_PS32) {
            zdev = base;
            break;
        }
    }

    if (!zdev) {
        const char *err = "PiStorm32 Virtual Zorro board not found!\nEnsure you have the refactor firmware flashed.";
        if (is_workbench) {
            show_gui_message("PiStorm32 Error", err);
        } else {
            printf("ERROR: %s\n", err);
        }
        if (IntuitionBase) CloseLibrary((struct Library *)IntuitionBase);
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    ULONG prev_val = zdev[ZREG_PREF_CTRL / 4] & 0xFF;
    ULONG new_val = prev_val;

    if (target_mode == 1) {
        zdev[ZREG_PREF_CTRL / 4] = target_val;
        new_val = zdev[ZREG_PREF_CTRL / 4] & 0xFF;
    }

    if (is_workbench) {
        char msg_buf[256];
        snprintf(msg_buf, sizeof(msg_buf),
                 "PiStorm32-lite: %s\n\nBase: 0x%08lX\nBUS_CTRL: 0x%02lX -> 0x%02lX\nFast DSACK: %s",
                 action_desc,
                 (ULONG)cd->cd_BoardAddr,
                 prev_val, new_val,
                 (new_val & CTRL_FAST_DSACK) ? "ENABLED" : "DISABLED (Safe)");
        show_gui_message("PiStorm32 Bus Control", msg_buf);
    } else {
        printf("============================================================\n");
        printf("  PiStorm32-lite Bus Control: %s\n", action_desc);
        printf("============================================================\n");
        printf("[OK] Found Virtual Zorro Board at: 0x%08lX\n", (ULONG)cd->cd_BoardAddr);

        ULONG git_hash = zdev[ZREG_GIT_HASH / 4];
        ULONG build_date = zdev[ZREG_BUILD_DATE / 4];
        ULONG build_info = zdev[ZREG_BUILD_INFO / 4];
        if (git_hash != 0) {
            printf("Firmware: Commit %08lx%s (Built %04lx-%02lx-%02lx | Target %lu MHz, %lux PLL)\n",
                   git_hash, (build_info & 1) ? "-dirty" : "",
                   (build_date >> 16) & 0xFFFF, (build_date >> 8) & 0xFF, build_date & 0xFF,
                   (build_info >> 8) & 0xFF, (build_info >> 4) & 0x0F);
        }

        if (target_mode == 1) {
            printf("Previous BUS_CTRL: 0x%02lX\n", prev_val);
            printf("New BUS_CTRL:      0x%02lX\n\n", new_val);
        } else {
            printf("Current BUS_CTRL:  0x%02lX\n\n", new_val);
        }
        printf("Active Settings:\n");
        print_active_bits(new_val);
        printf("------------------------------------------------------------\n");
        ULONG launches = zdev[0x24 / 4];
        ULONG hits = zdev[0x28 / 4];
        printf("Prefetch Telemetry: %lu launches, %lu hits (%.1f%% hit rate)\n",
               launches, hits, launches ? (100.0 * hits / launches) : 0.0);
        printf("------------------------------------------------------------\n");
        if (!(new_val & CTRL_FAST_DSACK)) {
            printf(">> Standard Motorola S4_NOP hold cycle active (Mediator TX safe).\n");
        } else {
            printf(">> Fast DSACK termination active (~88ns dead time reduction).\n");
        }
        printf("\n");
    }

    if (IntuitionBase) CloseLibrary((struct Library *)IntuitionBase);
    CloseLibrary((struct Library *)ExpansionBase);
    return 0;
}
