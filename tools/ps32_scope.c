/*
 * PiStorm32-lite Live Hardware Bus Scope & Logic Analyzer
 * Target: Motorola 68020+ / AmigaOS 2.0+ (V36+) / RTG & Native
 *
 * Real-time hardware oscilloscope rendering Amiga 1200 bus cycles
 * captured by the PiStorm32 FPGA Virtual Zorro diagnostic telemetry engine.
 */

#include <exec/types.h>
#include <exec/libraries.h>
#include <exec/memory.h>
#include <libraries/expansionbase.h>
#include <libraries/configvars.h>
#include <intuition/intuition.h>
#include <graphics/gfxbase.h>
#include <graphics/view.h>
#include <graphics/rastport.h>

#include <proto/exec.h>
#include <proto/expansion.h>
#include <proto/intuition.h>
#include <proto/graphics.h>

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
#define ZREG_DIAG_STATUS  0x20
#define ZREG_PREF_LAUNCH  0x24
#define ZREG_PREF_HIT     0x28
#define ZREG_BUS_CAPTURE  0x2C
#define ZREG_CYCLE_TIMING 0x30
#define ZREG_CLOCK_PHASE  0x34

#define MAGIC_PS32        0x50533332 /* 'PS32' */

#define CTRL_PREFETCH_32  (1 << 0)
#define CTRL_FAST_DSACK   (1 << 1)
#define CTRL_CCK_SYNC     (1 << 2)
#define CTRL_PHASE_INV    (1 << 3)
#define CTRL_PREFETCH_16  (1 << 7)

#define MODE_TURBO_ON_VAL      0x87
#define MODE_NO_FAST_DSACK_VAL 0x81
#define MODE_TURBO_OFF_VAL     0x00

struct ExpansionBase *ExpansionBase = NULL;
struct IntuitionBase *IntuitionBase = NULL;
struct GfxBase *GfxBase = NULL;

static volatile ULONG *zdev = NULL;
static struct Window *win = NULL;
static struct RastPort *rp = NULL;
static struct ColorMap *cm = NULL;

/* Scope Color Pens */
static LONG pen_bg       = 0;
static LONG pen_grid     = 1;
static LONG pen_text     = 1;
static LONG pen_text_dim = 2;
static LONG pen_clk      = 3;
static LONG pen_as       = 2;
static LONG pen_ds       = 3;
static LONG pen_dsack    = 1;
static LONG pen_data     = 2;
static LONG pen_accent   = 3;

static int custom_pens_allocated = 0;
static LONG allocated_pens[10];
static int alloc_pen_count = 0;

static LONG alloc_color(ULONG r, ULONG g, ULONG b, LONG fallback) {
    if (GfxBase && GfxBase->LibNode.lib_Version >= 39 && cm) {
        struct TagItem tags[] = {
            { OBP_Precision, PRECISION_IMAGE },
            { TAG_DONE, 0 }
        };
        LONG p = ObtainBestPenA(cm, r << 24, g << 24, b << 24, tags);
        if (p >= 0) {
            allocated_pens[alloc_pen_count++] = p;
            return p;
        }
    }
    return fallback;
}

static void init_pens(struct Screen *scr) {
    cm = scr->ViewPort.ColorMap;
    if (GfxBase->LibNode.lib_Version >= 39) {
        pen_bg       = alloc_color(0x06, 0x0a, 0x12, 1); // Pure dark background (fallback Pen 1: Black)
        pen_grid     = alloc_color(0x1a, 0x26, 0x38, 0); // Dark blue-grey grid (fallback Pen 0: Grey)
        pen_text     = alloc_color(0xff, 0xff, 0xff, 2); // Pure crisp white (fallback Pen 2: White)
        pen_text_dim = alloc_color(0x88, 0xd4, 0xff, 2); // High-contrast ice blue (fallback Pen 2: White)
        pen_clk      = alloc_color(0x00, 0xee, 0xff, 2); // Bright neon cyan
        pen_as       = alloc_color(0x00, 0xff, 0x66, 2); // Neon lime green
        pen_ds       = alloc_color(0xff, 0xbb, 0x00, 3); // Vivid amber gold
        pen_dsack    = alloc_color(0xff, 0x33, 0x88, 3); // Neon magenta / hot pink
        pen_data     = alloc_color(0x38, 0xa8, 0xff, 2); // Electric sky blue
        pen_accent   = alloc_color(0xff, 0xee, 0x33, 2); // Electric yellow / gold
        custom_pens_allocated = 1;
    } else {
        pen_bg       = 1; // Black
        pen_grid     = 0; // Grey
        pen_text     = 2; // White
        pen_text_dim = 2; // White
        pen_clk      = 2; // White
        pen_as       = 3; // Blue/Orange
        pen_ds       = 3; // Blue/Orange
        pen_dsack    = 2; // White
        pen_data     = 3; // Blue/Orange
        pen_accent   = 2; // White
    }
}

static void free_pens(void) {
    if (custom_pens_allocated && cm && GfxBase->LibNode.lib_Version >= 39) {
        for (int i = 0; i < alloc_pen_count; i++) {
            ReleasePen(cm, allocated_pens[i]);
        }
    }
}

static void draw_digital_trace(int x0, int y_high, int y_low, int width, const int *transitions, int num_trans, LONG pen) {
    SetAPen(rp, pen);
    int cur_x = x0;
    int cur_y = transitions[0] ? y_high : y_low;
    Move(rp, cur_x, cur_y);

    for (int i = 1; i < num_trans; i += 2) {
        int next_x = x0 + transitions[i];
        int next_y = transitions[i + 1] ? y_high : y_low;
        if (next_x > x0 + width) next_x = x0 + width;

        Draw(rp, next_x, cur_y);
        Draw(rp, next_x, next_y);
        cur_x = next_x;
        cur_y = next_y;
        if (cur_x >= x0 + width) break;
    }
    if (cur_x < x0 + width) {
        Draw(rp, x0 + width, cur_y);
    }
}

static void draw_bus_box(int x0, int y_mid, int height, int start_x, int end_x, const char *label, LONG pen) {
    int top = y_mid - height / 2;
    int bot = y_mid + height / 2;
    SetAPen(rp, pen);

    /* Left crossover */
    Move(rp, x0 + start_x - 4, y_mid);
    Draw(rp, x0 + start_x, top);
    Draw(rp, x0 + end_x, top);
    Draw(rp, x0 + end_x + 4, y_mid);

    Move(rp, x0 + start_x - 4, y_mid);
    Draw(rp, x0 + start_x, bot);
    Draw(rp, x0 + end_x, bot);
    Draw(rp, x0 + end_x + 4, y_mid);

    /* Idle lines before and after */
    Move(rp, x0, y_mid);
    Draw(rp, x0 + start_x - 4, y_mid);
    Move(rp, x0 + end_x + 4, y_mid);
    Draw(rp, x0 + 510, y_mid);

    if (label && (end_x - start_x > 30)) {
        SetAPen(rp, pen_text);
        SetBPen(rp, pen_bg);
        SetDrMd(rp, JAM2);
        Move(rp, x0 + start_x + (end_x - start_x) / 2 - 16, y_mid + 3);
        Text(rp, (STRPTR)label, strlen(label));
    }
}

static void render_scope(int is_frozen) {
    /* Read live telemetry registers */
    ULONG ctrl    = zdev[ZREG_PREF_CTRL / 4];
    ULONG status  = zdev[ZREG_DIAG_STATUS / 4];
    ULONG capture = zdev[ZREG_BUS_CAPTURE / 4];
    ULONG timing  = zdev[ZREG_CYCLE_TIMING / 4];
    ULONG clk_ph  = zdev[ZREG_CLOCK_PHASE / 4];
    ULONG launch  = zdev[ZREG_PREF_LAUNCH / 4];
    ULONG hits    = zdev[ZREG_PREF_HIT / 4];

    ULONG as_ticks       = (timing >> 22) & 0x3FF;
    ULONG as_to_dsack_t  = (timing >> 12) & 0x3FF;
    ULONG ws_14m         = (timing >> 8) & 0x0F;
    ULONG dsack_lead     = (timing >> 4) & 0x0F;
    int   dsack_at_high  = (timing >> 3) & 1;
    int   cck_phase      = (timing >> 2) & 1;
    int   is_read        = (timing >> 1) & 1;

    ULONG as_ns          = (as_ticks * 5495UL) / 1000UL;
    ULONG as_to_dsack_ns = (as_to_dsack_t * 5495UL) / 1000UL;

    ULONG addr_lo        = (capture >> 16) & 0xFF;
    ULONG seq_num        = (capture >> 24) & 0xFF;
    int   last_rw        = (capture >> 15) & 1;
    ULONG pw_s5          = (capture >> 8) & 3;

    ULONG clk_per_t      = (clk_ph >> 16) & 0xFF;
    ULONG clk_hi_t       = (clk_ph >> 8) & 0xFF;
    ULONG clk_lo_t       = clk_ph & 0xFF;
    ULONG clk_per_ns     = (clk_per_t * 5495UL) / 1000UL;

    int fast_dsack_en    = (ctrl >> 1) & 1;
    int pref_en          = ctrl & 1;
    int cck_en           = (ctrl >> 2) & 1;

    const char *mode_str = (fast_dsack_en && cck_en) ? "ULTRA TURBO (0x87)" :
                           (!fast_dsack_en && !cck_en && pref_en) ? "NO FAST DSACK (0x81 - MEDIATOR SAFE)" :
                           (!pref_en && !fast_dsack_en && !cck_en) ? "STOCK GOLDEN REF (0x00)" : "CUSTOM MODE";

    const char *pw_str = (pw_s5 == 3) ? "32-bit" : (pw_s5 == 1) ? "16-bit" : "8-bit";

    int left = win->BorderLeft + 8;
    int top  = win->BorderTop + 4;
    int w    = win->Width - win->BorderLeft - win->BorderRight - 16;

    SetDrMd(rp, JAM2);
    SetBPen(rp, pen_bg);

    /* 1. Top Header Banner */
    SetAPen(rp, pen_bg);
    RectFill(rp, left, top, left + w, top + 44);

    char buf[128];
    SetAPen(rp, pen_accent);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, top + 11);
    Text(rp, (STRPTR)"PISTORM32-LITE 182 MHz BUS TIMING ANALYZER", 43);

    SetAPen(rp, is_frozen ? pen_dsack : pen_as);
    snprintf(buf, sizeof(buf), "[%s]", is_frozen ? "FREEZE (SPACE)" : "LIVE PROFILING");
    Move(rp, left + 370, top + 11);
    Text(rp, (STRPTR)buf, strlen(buf));

    SetAPen(rp, pen_text_dim);
    snprintf(buf, sizeof(buf), "Mode: %s", mode_str);
    Move(rp, left + 4, top + 26);
    Text(rp, (STRPTR)buf, strlen(buf));

    SetAPen(rp, pen_text);
    snprintf(buf, sizeof(buf), "Cycle #%03lu | %s %s @ $..%02lX | /AS: %luns | Latency to /DSACK: %luns",
             seq_num, last_rw ? "READ" : "WRITE", pw_str, addr_lo, as_ns, as_to_dsack_ns);
    Move(rp, left + 4, top + 40);
    Text(rp, (STRPTR)buf, strlen(buf));

    /* 2. Waveform Canvas */
    int c_x = left + 64;
    int c_y = top + 54;
    int c_w = 510;
    int c_h = 240;

    SetAPen(rp, pen_bg);
    RectFill(rp, left, c_y - 4, left + w, c_y + c_h + 10);

    /* Draw Scope Graticule (100 ns grid lines) */
    /* Scale: 500 pixels = 1250 ns -> 100 ns = 40 pixels (0.4 px/ns) */
    SetAPen(rp, pen_grid);
    for (int ns = 0; ns <= 1250; ns += 100) {
        int gx = c_x + (ns * 4) / 10;
        Move(rp, gx, c_y);
        Draw(rp, gx, c_y + c_h);

        /* Time markers at top */
        if (ns % 200 == 0) {
            snprintf(buf, sizeof(buf), "%d", ns);
            SetAPen(rp, pen_text_dim);
            SetBPen(rp, pen_bg);
            Move(rp, gx - 8, c_y - 6);
            Text(rp, (STRPTR)buf, strlen(buf));
            SetAPen(rp, pen_grid);
        }
    }

    /* Trace 1: MC_CLK (14.18 MHz) */
    SetAPen(rp, pen_clk);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 16);
    Text(rp, (STRPTR)"MC_CLK", 6);

    int clk_half = 14; /* 70ns period = 28px -> half = 14px */
    int clk_trans[40];
    int t_idx = 0;
    clk_trans[t_idx++] = 0; /* start low */
    int cx = 0;
    for (int k = 0; k < 18; k++) {
        cx += clk_half;
        clk_trans[t_idx++] = cx;
        clk_trans[t_idx++] = (k % 2 == 0) ? 1 : 0;
    }
    draw_digital_trace(c_x, c_y + 6, c_y + 22, c_w, clk_trans, t_idx, pen_clk);

    /* Trace 2: /AS (Address Strobe) */
    SetAPen(rp, pen_as);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 54);
    Text(rp, (STRPTR)"/AS", 3);

    int as_start_px = 28;
    int as_width_px = (as_ns * 4) / 10;
    if (as_width_px < 20) as_width_px = 227; /* default fallback ~569ns */
    if (as_width_px > c_w - as_start_px - 10) as_width_px = c_w - as_start_px - 10;
    int as_end_px = as_start_px + as_width_px;

    int as_trans[] = {
        1, /* starts high */
        as_start_px, 0, /* goes low */
        as_end_px,   1  /* goes high */
    };
    draw_digital_trace(c_x, c_y + 42, c_y + 58, c_w, as_trans, 5, pen_as);

    /* Trace 3: /DS (Data Strobe) */
    SetAPen(rp, pen_ds);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 92);
    Text(rp, (STRPTR)"/DS", 3);

    int ds_start_px = is_read ? as_start_px : (as_start_px + 28);
    int ds_trans[] = {
        1, /* starts high */
        ds_start_px, 0,
        as_end_px,   1
    };
    draw_digital_trace(c_x, c_y + 80, c_y + 96, c_w, ds_trans, 5, pen_ds);

    /* Trace 4: /DSACK (Acknowledge from Slave) */
    SetAPen(rp, pen_dsack);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 130);
    Text(rp, (STRPTR)"/DSACK", 6);

    int dsack_px = as_start_px + (as_to_dsack_ns * 4) / 10;
    if (dsack_px <= as_start_px) dsack_px = as_start_px + 140;
    if (dsack_px >= as_end_px) dsack_px = as_end_px - 20;

    int dsack_trans[] = {
        1,
        dsack_px,  0,
        as_end_px + 8, 1
    };
    draw_digital_trace(c_x, c_y + 118, c_y + 134, c_w, dsack_trans, 5, pen_dsack);

    /* Trace 5: R/W */
    SetAPen(rp, pen_text);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 168);
    Text(rp, (STRPTR)"R/W", 3);
    int rw_trans[] = {
        is_read ? 1 : 0
    };
    draw_digital_trace(c_x, c_y + 156, c_y + 172, c_w, rw_trans, 1, is_read ? pen_clk : pen_ds);

    /* Trace 6: DATA[31:0] Bus Window */
    SetAPen(rp, pen_data);
    SetBPen(rp, pen_bg);
    Move(rp, left + 4, c_y + 206);
    Text(rp, (STRPTR)"DATA", 4);
    int data_start = is_read ? dsack_px : ds_start_px;
    draw_bus_box(c_x, c_y + 204, 16, data_start, as_end_px, is_read ? "READ DATA" : "WRITE DATA", pen_data);

    /* Phase S markers above /AS */
    SetBPen(rp, pen_bg);
    SetAPen(rp, pen_accent);
    Move(rp, c_x + 8, c_y + 36); Text(rp, (STRPTR)"S0", 2);
    Move(rp, c_x + as_start_px + 2, c_y + 36); Text(rp, (STRPTR)"S1", 2);
    Move(rp, c_x + as_start_px + 16, c_y + 36); Text(rp, (STRPTR)"S2", 2);
    Move(rp, c_x + as_start_px + 30, c_y + 36); Text(rp, (STRPTR)"S3", 2);
    if (!fast_dsack_en) {
        SetAPen(rp, pen_dsack);
        Move(rp, c_x + as_end_px - 36, c_y + 36); Text(rp, (STRPTR)"S4(NOP)", 7);
    }
    SetAPen(rp, pen_accent);
    Move(rp, c_x + as_end_px - 8, c_y + 36); Text(rp, (STRPTR)"S5", 2);

    /* 3. Measurement Bottom Bar */
    int b_y = c_y + c_h + 16;
    SetAPen(rp, pen_bg);
    RectFill(rp, left, b_y, left + w, b_y + 80);

    SetAPen(rp, pen_grid);
    Move(rp, left, b_y); Draw(rp, left + w, b_y);

    SetBPen(rp, pen_bg);
    SetAPen(rp, pen_accent);
    snprintf(buf, sizeof(buf), "HARDWARE MEASUREMENTS (5.5ns Resolution):");
    Move(rp, left + 4, b_y + 14); Text(rp, (STRPTR)buf, strlen(buf));

    SetAPen(rp, pen_text);
    snprintf(buf, sizeof(buf), "  /AS: %luns (%lut) | S3 WaitStates: %lu (%luns) | Raw DSACK: %s",
             as_ns, as_ticks, ws_14m, ws_14m * 70UL, dsack_at_high ? "HIGH (S4/S2)" : "LOW (S3/S1)");
    Move(rp, left + 4, b_y + 29); Text(rp, (STRPTR)buf, strlen(buf));

    snprintf(buf, sizeof(buf), "  14MHz Clock: %lu.%02lu MHz (Per %luns, Hi %luns, Lo %luns) | CCK Phase: %d",
             (1000000UL / (clk_per_ns ? clk_per_ns : 70)) / 1000UL,
             ((1000000UL / (clk_per_ns ? clk_per_ns : 70)) % 1000UL) / 10UL,
             clk_per_ns, (clk_hi_t * 5495UL) / 1000UL, (clk_lo_t * 5495UL) / 1000UL, cck_phase);
    Move(rp, left + 4, b_y + 44); Text(rp, (STRPTR)buf, strlen(buf));

    ULONG hit_pct = (launch > 0) ? (hits * 100UL) / launch : 0;
    snprintf(buf, sizeof(buf), "  Prefetch: %lu reqs, %lu hits (%lu%%) | S4 Hold: %s",
             launch, hits, hit_pct, fast_dsack_en ? "BYPASS (Turbo)" : "ACTIVE (Safe)");
    Move(rp, left + 4, b_y + 59); Text(rp, (STRPTR)buf, strlen(buf));

    /* Key commands banner */
    SetAPen(rp, pen_clk);
    snprintf(buf, sizeof(buf), "KEYS: [R]Read [W]Write [1]Turbo [2]Safe [3]Stock [SPC]Freeze [Q]Quit");
    Move(rp, left + 4, b_y + 74); Text(rp, (STRPTR)buf, strlen(buf));
}

int main(int argc, char **argv) {
    ExpansionBase = (struct ExpansionBase *)OpenLibrary((UBYTE *)"expansion.library", 36);
    if (!ExpansionBase) {
        printf("ERROR: Failed to open expansion.library V36+!\n");
        return 20;
    }

    IntuitionBase = (struct IntuitionBase *)OpenLibrary((UBYTE *)"intuition.library", 36);
    if (!IntuitionBase) {
        printf("ERROR: Failed to open intuition.library V36+!\n");
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    GfxBase = (struct GfxBase *)OpenLibrary((UBYTE *)"graphics.library", 36);
    if (!GfxBase) {
        printf("ERROR: Failed to open graphics.library V36+!\n");
        CloseLibrary((struct Library *)IntuitionBase);
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    struct ConfigDev *cd = NULL;
    while ((cd = FindConfigDev(cd, ZORRO_MANUF_ID, ZORRO_PROD_ID)) != NULL) {
        volatile ULONG *base = (volatile ULONG *)cd->cd_BoardAddr;
        if (base && base[ZREG_MAGIC / 4] == MAGIC_PS32) {
            zdev = base;
            break;
        }
    }

    if (!zdev) {
        printf("ERROR: PiStorm32 Virtual Zorro board not found!\n");
        CloseLibrary((struct Library *)GfxBase);
        CloseLibrary((struct Library *)IntuitionBase);
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    struct Screen *pub_screen = LockPubScreen(NULL);
    if (!pub_screen) {
        printf("ERROR: Could not lock default public screen!\n");
        CloseLibrary((struct Library *)GfxBase);
        CloseLibrary((struct Library *)IntuitionBase);
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    init_pens(pub_screen);

    win = OpenWindowTags(NULL,
        WA_Title, (ULONG)"PiStorm32-lite Bus Scope & Logic Analyzer",
        WA_Left, 20,
        WA_Top, 20,
        WA_Width, 640,
        WA_Height, 445,
        WA_IDCMP, IDCMP_CLOSEWINDOW | IDCMP_RAWKEY | IDCMP_INTUITICKS,
        WA_Flags, WFLG_CLOSEGADGET | WFLG_DRAGBAR | WFLG_DEPTHGADGET | WFLG_ACTIVATE,
        WA_PubScreen, (ULONG)pub_screen,
        TAG_DONE);

    UnlockPubScreen(NULL, pub_screen);

    if (!win) {
        printf("ERROR: Failed to open Intuition window!\n");
        free_pens();
        CloseLibrary((struct Library *)GfxBase);
        CloseLibrary((struct Library *)IntuitionBase);
        CloseLibrary((struct Library *)ExpansionBase);
        return 20;
    }

    rp = win->RPort;

    int c_left = win->BorderLeft;
    int c_top  = win->BorderTop;
    int c_w    = win->Width - win->BorderLeft - win->BorderRight;
    int c_h    = win->Height - win->BorderTop - win->BorderBottom;
    SetAPen(rp, pen_bg);
    SetBPen(rp, pen_bg);
    RectFill(rp, c_left, c_top, c_left + c_w - 1, c_top + c_h - 1);

    /* Allocate small test chip buffer for live triggering */
    volatile ULONG *chip_test_buf = (volatile ULONG *)AllocMem(1024, MEMF_CHIP | MEMF_CLEAR);

    int is_frozen = 0;
    int running = 1;
    int tick_count = 0;

    render_scope(is_frozen);

    while (running) {
        struct IntuiMessage *msg;
        while ((msg = (struct IntuiMessage *)GetMsg(win->UserPort)) != NULL) {
            ULONG msg_class = msg->Class;
            UWORD msg_code  = msg->Code;
            ReplyMsg((struct Message *)msg);

            if (msg_class == IDCMP_CLOSEWINDOW) {
                running = 0;
            } else if (msg_class == IDCMP_RAWKEY) {
                if (msg_code == 0x45 || msg_code == 0x10) { /* ESC or 'Q' */
                    running = 0;
                } else if (msg_code == 0x40) { /* Space: Freeze / Unfreeze */
                    is_frozen = !is_frozen;
                    render_scope(is_frozen);
                } else if (msg_code == 0x13) { /* 'R': Test Read */
                    if (chip_test_buf) {
                        volatile ULONG dummy = chip_test_buf[0];
                        (void)dummy;
                    }
                    render_scope(is_frozen);
                } else if (msg_code == 0x11) { /* 'W': Test Write */
                    if (chip_test_buf) {
                        chip_test_buf[0] = 0xCAFEBABE;
                    }
                    render_scope(is_frozen);
                } else if (msg_code == 0x01) { /* '1': Turbo ON */
                    zdev[ZREG_PREF_CTRL / 4] = MODE_TURBO_ON_VAL;
                    render_scope(is_frozen);
                } else if (msg_code == 0x02) { /* '2': No Fast DSACK (Safe) */
                    zdev[ZREG_PREF_CTRL / 4] = MODE_NO_FAST_DSACK_VAL;
                    render_scope(is_frozen);
                } else if (msg_code == 0x03) { /* '3': Turbo OFF */
                    zdev[ZREG_PREF_CTRL / 4] = MODE_TURBO_OFF_VAL;
                    render_scope(is_frozen);
                }
            } else if (msg_class == IDCMP_INTUITICKS) {
                tick_count++;
                if (!is_frozen && (tick_count % 2 == 0)) { /* Refresh ~5-10 times/sec */
                    render_scope(is_frozen);
                }
            }
        }
        ULONG sigs = Wait((1 << win->UserPort->mp_SigBit) | SIGBREAKF_CTRL_C);
        if (sigs & SIGBREAKF_CTRL_C) {
            running = 0;
        }
    }

    if (chip_test_buf) {
        FreeMem((APTR)chip_test_buf, 1024);
    }

    CloseWindow(win);
    free_pens();

    CloseLibrary((struct Library *)GfxBase);
    CloseLibrary((struct Library *)IntuitionBase);
    CloseLibrary((struct Library *)ExpansionBase);

    return 0;
}
