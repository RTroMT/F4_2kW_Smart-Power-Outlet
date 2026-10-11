#include "ui.h"
#include "ili9341.h"
#include "xpt2046.h"
#include "flash_store.h"
#include <stdio.h>
#include <string.h>

/* ===================================================================================== */
/*  Settings                                                                             */
/* ===================================================================================== */

/* ---- Layout (made for the 320 x 240 landscape screen) ---- */
#define SCREEN_W          320
#define SCREEN_H          240
#define TAB_H             22                    /* tab bar at the top           */
#define FOOT_Y            218                   /* status bar at the bottom     */
#define FOOT_H            22
#define CONTENT_Y         26                    /* page area: CONTENT_Y .. FOOT_Y-1 */

/* ---- Touch and gestures ---- */
#define TOUCH_POLL_MS         8u                /* read the touch controller at most this often */
#define TOUCH_RELEASE_MISSES  3u                /* "no touch" readings in a row = finger lifted */
#define TAP_SLOP_PX           14                /* moved less than this = still a tap           */
#define SWIPE_MIN_PX          56                /* horizontal distance that counts as a swipe   */
#define REPEAT_DELAY_MS       450u              /* hold a +/- button: auto repeat after ...     */
#define REPEAT_PERIOD_MS      120u              /* ... then every ...                           */

/* ---- Calibration ---- */
#define CAL_INSET             24                /* the crosses sit this far from the screen edges */
#define CAL_SAMPLES           10u               /* readings averaged per cross                    */
#define CAL_JITTER_RAW        80                /* readings must stay this close to the first one */
#define CAL_MAX_ERROR_PX      10.0f             /* worse than this: ask the user to try again     */
#define CAL_ERROR_SHOW_MS     1800u
#define SETTINGS_MAGIC        0x31544553UL      /* "SET1" */

/* ---- Application ---- */
#define MAX_POWER_W           2000.0f           /* the load bar is full at this power */
#define TIMER_MIN_S           60u               /* timer range: 1 minute ... 99 minutes */
#define TIMER_MAX_S           (99u * 60u)
#define TIMER_DEFAULT_S       (10u * 60u)

/* ---- Palette ---- */
#define C_BG        ILI9341_RGB(3, 5, 14)       /* near black, a little blue */
#define C_PANEL     ILI9341_RGB(8, 15, 30)
#define C_PANEL_HI  ILI9341_RGB(14, 28, 52)
#define C_CYAN      ILI9341_RGB(0, 230, 255)
#define C_CYAN_DIM  ILI9341_RGB(0, 95, 120)
#define C_MAGENTA   ILI9341_RGB(255, 40, 170)
#define C_GREEN     ILI9341_RGB(40, 255, 120)
#define C_AMBER     ILI9341_RGB(255, 180, 0)
#define C_RED       ILI9341_RGB(255, 55, 75)
#define C_TEXT      ILI9341_RGB(220, 240, 255)
#define C_TEXT_DIM  ILI9341_RGB(95, 130, 160)

/* ===================================================================================== */
/*  Types and state                                                                      */
/* ===================================================================================== */

typedef struct { int16_t x, y, w, h; } rect_t;

typedef enum { MODE_RUN = 0, MODE_CAL } ui_mode_t;

typedef enum {
    CAL_WAIT_TOUCH,        /* waiting for a finger on the cross       */
    CAL_WAIT_RELEASE,      /* cross captured, waiting for the finger to lift */
    CAL_VERIFY,            /* calibration computed: test it, then SAVE or RETRY */
    CAL_ERROR              /* taps were not good enough               */
} cal_phase_t;

typedef enum { TOUCH_NONE = 0, TOUCH_DOWN, TOUCH_MOVE, TOUCH_UP } touch_event_t;

/* A text field remembers what it shows, so only the characters that changed are redrawn. */
#define FIELD_MAX 12
typedef struct {
    char     text[FIELD_MAX + 1];
    uint16_t fg;
} field_t;

/* What is stored in flash */
typedef struct {
    uint32_t           magic;
    uint8_t            rotation;
    uint8_t            pad[3];
    xpt2046_cal_t      cal;
} ui_settings_t;

/* ---- general ---- */
static ui_hooks_t s_hooks;
static ui_mode_t  s_mode = MODE_RUN;
static ui_page_t  s_page = UI_PAGE_MONITOR;
static bool       s_request_cal = false;
static bool       s_save_failed = false;

/* ---- data from the application ---- */
static ui_meas_t  s_meas;
static bool       s_meas_dirty   = true;
static bool       s_output_on    = false;
static bool       s_output_dirty = true;

/* ---- text fields ---- */
static field_t f_v, f_i, f_p, f_pf, f_s, f_pct;     /* MONITOR  */
static field_t f_load;                              /* SSR      */
static field_t f_timer;                             /* TIMER    */
static field_t f_link, f_out;                       /* footer   */

/* ---- widgets that remember their last drawn state ---- */
static int     s_bar_level      = -1;               /* MONITOR load bar, lit segments (-1 = not drawn) */
static int8_t  s_switch_drawn   = -1;               /* SSR toggle: 0 off, 1 on, -1 not drawn           */
static int     s_timer_bar      = -1;
static int8_t  s_timer_btn      = -1;               /* TIMER start button state                        */

/* ---- timer ---- */
static uint16_t s_timer_set_s       = TIMER_DEFAULT_S;
static uint16_t s_timer_remaining_s = 0u;
static bool     s_timer_running     = false;
static uint32_t s_timer_end_ms      = 0u;
static bool     s_timer_dirty       = true;

/* ---- touch ---- */
static struct {
    bool     down;
    uint8_t  misses;
    int16_t  x, y;                  /* last known position */
    uint32_t last_poll_ms;
} s_touch;

static struct {
    int16_t  x0, y0;                /* where the finger landed */
    bool     swiped;                /* a page change was already made during this touch */
    bool     moved;                 /* moved too far to be a tap */
    bool     consumed;              /* the press itself did something (+/- button) */
    int8_t   repeat_idx;            /* held +/- button, -1 = none */
    uint32_t next_repeat_ms;
} s_gest;

/* ---- calibration ---- */
static struct {
    cal_phase_t phase;
    uint8_t     step;               /* which cross (0..3) */
    uint8_t     n;                  /* readings collected for this cross */
    uint32_t    sum_x, sum_y;
    uint16_t    first_x, first_y;
    uint8_t     release_count;
    uint16_t    raw[4][2];
    int16_t     scr[4][2];
    xpt2046_cal_t result;
    uint32_t    error_since_ms;
} s_cal;

/* ---- fixed rectangles ---- */
#define TILE_W 150
#define TILE_H 70
static const rect_t k_tiles[4] = {
    {   8,  30, TILE_W, TILE_H },        /* voltage      */
    { 162,  30, TILE_W, TILE_H },        /* current      */
    {   8, 104, TILE_W, TILE_H },        /* active power */
    { 162, 104, TILE_W, TILE_H },        /* power factor */
};
static const rect_t k_bar_panel   = {   8, 178, 304,  34 };
static const rect_t k_switch_btn  = {  70,  72, 180,  92 };
static const rect_t k_timer_adj[4] = {
    {  12, 126, 68, 26 }, {  88, 126, 68, 26 }, { 164, 126, 68, 26 }, { 240, 126, 68, 26 },
};
static const rect_t k_timer_start = {  70, 162, 180,  34 };
static const rect_t k_setup_recal = {  20,  56, 280,  40 };
static const rect_t k_setup_flip  = {  20, 104, 280,  40 };
static const rect_t k_verify_retry = {  20, 180, 120,  40 };
static const rect_t k_verify_save  = { 180, 180, 120,  40 };

static const char *const k_tab_names[UI_PAGE_COUNT] = { "MONITOR", "SSR", "TIMER", "SETUP" };
static const int16_t     k_timer_delta_s[4]   = { -300, -60, 60, 300 };
static const char *const k_timer_adj_label[4] = { "-5", "-1", "+1", "+5" };

/* ===================================================================================== */
/*  Small drawing helpers (plain ints so the call sites stay readable)                   */
/* ===================================================================================== */

static void fill(int x, int y, int w, int h, uint16_t c)
{
    ILI9341_FillRect((int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, c);
}

static void hline(int x, int y, int w, uint16_t c) { fill(x, y, w, 1, c); }
static void vline(int x, int y, int h, uint16_t c) { fill(x, y, 1, h, c); }

static void line(int x0, int y0, int x1, int y1, uint16_t c)
{
    ILI9341_DrawLine((int16_t)x0, (int16_t)y0, (int16_t)x1, (int16_t)y1, c);
}

static void text(int x, int y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    ILI9341_DrawString((int16_t)x, (int16_t)y, s, fg, bg, scale);
}

static void text_center(int cx, int y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    text(cx - (int)ILI9341_TextWidth(s, scale) / 2, y, s, fg, bg, scale);
}

static void text_right(int right_x, int y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    text(right_x - (int)ILI9341_TextWidth(s, scale), y, s, fg, bg, scale);
}

static bool rect_hit(const rect_t *r, int x, int y)
{
    return x >= r->x && x < r->x + r->w && y >= r->y && y < r->y + r->h;
}

static int clamp_i(int v, int lo, int hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static int abs_i(int v) { return (v < 0) ? -v : v; }

/* ---- fields ---- */

static void field_reset(field_t *f)
{
    memset(f, 0, sizeof(*f));
}

/**
 * Show `str` in a field of `width` characters (padded with spaces).
 * Only characters that differ from what is already on the screen are drawn.
 * A different colour redraws the whole field.
 */
static void field_draw(field_t *f, int x, int y, const char *str, uint8_t width,
                       uint16_t fg, uint16_t bg, uint8_t scale)
{
    char padded[FIELD_MAX + 1];
    const size_t len = strlen(str);

    if (width > FIELD_MAX) {
        width = FIELD_MAX;
    }
    for (uint8_t i = 0u; i < width; i++) {
        padded[i] = (i < len) ? str[i] : ' ';
    }
    padded[width] = '\0';

    const bool recolour = (f->fg != fg);
    for (uint8_t i = 0u; i < width; i++) {
        if (recolour || f->text[i] != padded[i]) {
            ILI9341_DrawChar((int16_t)(x + i * ILI9341_CHAR_W * scale), (int16_t)y, padded[i], fg, bg, scale);
        }
    }
    memcpy(f->text, padded, (size_t)width + 1u);
    f->fg = fg;
}

/* ---- the neon panel look: chamfered corners (top-left and bottom-right) ---- */

/* Chamfered outline, 1 pixel wide. */
static void draw_outline(int x, int y, int w, int h, int cut, uint16_t color)
{
    const int r = x + w - 1;        /* right column */
    const int b = y + h - 1;        /* bottom row   */

    hline(x + cut, y, w - cut, color);              /* top edge    */
    vline(r, y, h - cut, color);                    /* right edge  */
    line(r, b - cut, r - cut, b, color);            /* bottom-right chamfer */
    hline(x, b, w - cut, color);                    /* bottom edge */
    vline(x, y + cut, h - cut, color);              /* left edge   */
    line(x, y + cut, x + cut, y, color);            /* top-left chamfer */
}

/* Fill the same shape: a rectangle with two cut corners. */
static void fill_chamfer(int x, int y, int w, int h, int cut, uint16_t color)
{
    for (int i = 0; i < cut; i++) {
        fill(x + cut - i, y + i,           w - (cut - i), 1, color);    /* top rows start further right   */
        fill(x,           y + h - cut + i, w - (i + 1),   1, color);    /* bottom rows end further left   */
    }
    fill(x, y + cut, w, h - 2 * cut, color);
}

/** Panel = filled shape + bright outline + a faint inner outline (the "glow"). */
static void draw_panel(int x, int y, int w, int h, int cut, uint16_t fill_color, uint16_t line_color)
{
    fill_chamfer(x, y, w, h, cut, fill_color);
    draw_outline(x, y, w, h, cut, line_color);
    draw_outline(x + 1, y + 1, w - 2, h - 2, (cut > 1) ? cut - 1 : 0, ILI9341_Blend(fill_color, line_color, 70));
}

static void draw_button(const rect_t *r, const char *label, uint16_t color, bool enabled, bool highlight, uint8_t scale)
{
    const uint16_t line_color = enabled ? color : C_TEXT_DIM;
    const uint16_t fill_color = (enabled && highlight) ? ILI9341_Blend(C_PANEL, color, 55) : C_PANEL;
    const uint16_t text_color = enabled ? color : C_TEXT_DIM;

    draw_panel(r->x, r->y, r->w, r->h, 6, fill_color, line_color);
    text_center(r->x + r->w / 2, r->y + (r->h - 7 * scale) / 2, label, text_color, fill_color, scale);
}

/* ===================================================================================== */
/*  Number formatting (no floating point printf: it is not available in newlib-nano)     */
/* ===================================================================================== */

static uint32_t scaled(float value, float factor)
{
    if (value < 0.0f) {
        value = 0.0f;
    }
    return (uint32_t)(value * factor + 0.5f);
}

static void fmt_voltage(char *b, size_t n, float v, bool ok)
{
    if (!ok) { (void)snprintf(b, n, "---.-"); return; }
    const uint32_t t = scaled(v, 10.0f);
    (void)snprintf(b, n, "%lu.%lu", (unsigned long)(t / 10u), (unsigned long)(t % 10u));
}

static void fmt_current(char *b, size_t n, float a, bool ok)
{
    if (!ok) { (void)snprintf(b, n, "-.---"); return; }
    if (a < 10.0f) {
        const uint32_t t = scaled(a, 1000.0f);
        (void)snprintf(b, n, "%lu.%03lu", (unsigned long)(t / 1000u), (unsigned long)(t % 1000u));
    } else {
        const uint32_t t = scaled(a, 100.0f);
        (void)snprintf(b, n, "%lu.%02lu", (unsigned long)(t / 100u), (unsigned long)(t % 100u));
    }
}

static void fmt_power(char *b, size_t n, float w, bool ok)
{
    if (!ok) { (void)snprintf(b, n, "--.--"); return; }
    if (w < 100.0f) {
        const uint32_t t = scaled(w, 100.0f);
        (void)snprintf(b, n, "%lu.%02lu", (unsigned long)(t / 100u), (unsigned long)(t % 100u));
    } else if (w < 1000.0f) {
        const uint32_t t = scaled(w, 10.0f);
        (void)snprintf(b, n, "%lu.%lu", (unsigned long)(t / 10u), (unsigned long)(t % 10u));
    } else {
        (void)snprintf(b, n, "%lu", (unsigned long)scaled(w, 1.0f));
    }
}

static void fmt_pf(char *b, size_t n, float pf, bool ok)
{
    if (!ok) { (void)snprintf(b, n, "-.--"); return; }
    const uint32_t t = scaled(pf, 100.0f);
    (void)snprintf(b, n, "%lu.%02lu", (unsigned long)(t / 100u), (unsigned long)(t % 100u));
}

/* ===================================================================================== */
/*  Output (the SSR) and timer logic                                                     */
/* ===================================================================================== */

/** Single place that changes the output: remembers it, tells the hardware, stops the timer on OFF. */
static void set_output(bool on)
{
    if (s_output_on == on) {
        return;
    }
    s_output_on = on;
    s_output_dirty = true;
    s_timer_dirty = true;

    if (!on && s_timer_running) {
        s_timer_running = false;                 /* switching OFF cancels the countdown */
    }
    if (s_hooks.set_output != NULL) {
        s_hooks.set_output(on);
    }
}

/** Change the preset time (only while the timer is not running). */
static void timer_adjust(int delta_s)
{
    if (s_timer_running) {
        return;
    }
    const int v = clamp_i((int)s_timer_set_s + delta_s, (int)TIMER_MIN_S, (int)TIMER_MAX_S);
    s_timer_set_s = (uint16_t)v;
    s_timer_dirty = true;
}

/** START (only when the output is ON) / STOP. */
static void timer_toggle(uint32_t now)
{
    if (s_timer_running) {
        s_timer_running = false;
    } else if (s_output_on) {
        s_timer_running = true;
        s_timer_end_ms = now + (uint32_t)s_timer_set_s * 1000u;
        s_timer_remaining_s = s_timer_set_s;
    }
    s_timer_dirty = true;
}

/** Called every pass: count down, switch the output off when time is up. */
static void timer_tick(uint32_t now)
{
    if (!s_timer_running) {
        return;
    }

    const int32_t left_ms = (int32_t)(s_timer_end_ms - now);
    if (left_ms <= 0) {
        s_timer_running = false;
        s_timer_remaining_s = 0u;
        s_timer_dirty = true;
        set_output(false);                       /* time is up: output OFF */
        return;
    }

    const uint16_t left_s = (uint16_t)((left_ms + 999) / 1000);
    if (left_s != s_timer_remaining_s) {
        s_timer_remaining_s = left_s;
        s_timer_dirty = true;
    }
}

/* ===================================================================================== */
/*  Chrome: tab bar and status bar                                                       */
/* ===================================================================================== */

static void draw_tabs(void)
{
    const int tab_w = SCREEN_W / UI_PAGE_COUNT;

    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        const bool active = (i == (int)s_page);
        const int x = i * tab_w;
        const uint16_t bg = active ? C_PANEL_HI : C_BG;

        fill(x, 0, tab_w, TAB_H, bg);
        text_center(x + tab_w / 2, 7, k_tab_names[i], active ? C_CYAN : C_TEXT_DIM, bg, 1);
        if (active) {
            fill(x, TAB_H - 3, tab_w, 3, C_CYAN);              /* neon underline */
        }
    }
    ILI9341_FillGradientH(0, TAB_H, SCREEN_W, 2, C_CYAN, C_MAGENTA);
}

static void draw_page_dots(void)
{
    for (int i = 0; i < UI_PAGE_COUNT; i++) {
        const int x = SCREEN_W - 10 - (UI_PAGE_COUNT * 14 - 6) + i * 14;
        const int y = FOOT_Y + 8;
        if (i == (int)s_page) {
            fill(x, y, 8, 8, C_CYAN);
        } else {
            fill(x, y, 8, 8, C_BG);
            ILI9341_DrawRect((int16_t)x, (int16_t)y, 8, 8, C_TEXT_DIM);
        }
    }
}

static void draw_footer(void)
{
    fill(0, FOOT_Y, SCREEN_W, FOOT_H, C_BG);
    ILI9341_FillGradientH(0, FOOT_Y, SCREEN_W, 2, C_MAGENTA, C_CYAN);
    draw_page_dots();
    field_reset(&f_link);
    field_reset(&f_out);
}

/** Link state on the left, output state in the middle. Only changed characters are redrawn. */
static void update_footer(void)
{
    const int y = FOOT_Y + 8;

    field_draw(&f_link, 10, y, s_meas.link_ok ? "LINK OK" : "NO DATA", 7,
               s_meas.link_ok ? C_GREEN : C_RED, C_BG, 1);

    if (s_save_failed) {
        field_draw(&f_out, 118, y, "CAL NOT SAVED", 12, C_AMBER, C_BG, 1);
    } else {
        field_draw(&f_out, 128, y, s_output_on ? "OUT ON" : "OUT OFF", 7,
                   s_output_on ? C_GREEN : C_TEXT_DIM, C_BG, 1);
    }
}

/* ===================================================================================== */
/*  Page: MONITOR                                                                        */
/* ===================================================================================== */

#define BAR_X          50
#define BAR_Y          188
#define BAR_SEGS       20
#define BAR_SEG_W      9
#define BAR_SEG_PITCH  11
#define BAR_SEG_H      14

static uint16_t bar_color(int i)
{
    return (i < 12) ? C_CYAN : ((i < 16) ? C_AMBER : C_RED);
}

static void draw_bar_segment(int i, bool lit)
{
    const uint16_t c = bar_color(i);
    fill(BAR_X + i * BAR_SEG_PITCH, BAR_Y, BAR_SEG_W, BAR_SEG_H, lit ? c : ILI9341_Blend(C_PANEL, c, 45));
}

/** Redraw only the segments between the old and the new level. */
static void update_bar(int level)
{
    if (level == s_bar_level) {
        return;
    }

    int from = 0, to = BAR_SEGS;                  /* [from, to) */
    if (s_bar_level >= 0) {
        from = (level < s_bar_level) ? level : s_bar_level;
        to   = (level < s_bar_level) ? s_bar_level : level;
    }
    for (int i = from; i < to; i++) {
        draw_bar_segment(i, i < level);
    }
    s_bar_level = level;
}

static void draw_monitor_static(void)
{
    static const struct {
        const char *label;
        const char *unit;
        const char *sub;
        uint16_t    color;
    } k_info[4] = {
        { "VOLTAGE",      "V", "AC RMS",  C_CYAN    },
        { "CURRENT",      "A", "AC RMS",  C_MAGENTA },
        { "ACTIVE POWER", "W", "",        C_GREEN   },
        { "POWER FACTOR", "",  "COS PHI", C_AMBER   },
    };

    for (int i = 0; i < 4; i++) {
        const rect_t *r = &k_tiles[i];
        draw_panel(r->x, r->y, r->w, r->h, 8, C_PANEL, k_info[i].color);
        text(r->x + 10, r->y + 6, k_info[i].label, C_TEXT_DIM, C_PANEL, 1);
        text_right(r->x + r->w - 9, r->y + 6, k_info[i].unit, k_info[i].color, C_PANEL, 1);
        text(r->x + 10, r->y + 58, k_info[i].sub, C_TEXT_DIM, C_PANEL, 1);
    }

    draw_panel(k_bar_panel.x, k_bar_panel.y, k_bar_panel.w, k_bar_panel.h, 6, C_PANEL, C_CYAN_DIM);
    text(16, 191, "LOAD", C_TEXT_DIM, C_PANEL, 1);

    field_reset(&f_v);  field_reset(&f_i);  field_reset(&f_p);
    field_reset(&f_pf); field_reset(&f_s);  field_reset(&f_pct);
    s_bar_level = -1;
}

static void update_monitor(void)
{
    char b[24];
    const bool ok = s_meas.link_ok;
    const uint16_t dim = C_TEXT_DIM;

    /* voltage and current */
    fmt_voltage(b, sizeof(b), s_meas.voltage, ok);
    field_draw(&f_v, k_tiles[0].x + 8, k_tiles[0].y + 22, b, 5, ok ? C_CYAN : dim, C_PANEL, 4);

    fmt_current(b, sizeof(b), s_meas.current, ok);
    field_draw(&f_i, k_tiles[1].x + 8, k_tiles[1].y + 22, b, 5, ok ? C_MAGENTA : dim, C_PANEL, 4);

    /* power: the colour warns when the load gets high */
    const float load = s_meas.power / MAX_POWER_W;
    const uint16_t pcol = !ok ? dim : ((load > 0.85f) ? C_RED : ((load > 0.60f) ? C_AMBER : C_GREEN));
    fmt_power(b, sizeof(b), s_meas.power, ok);
    field_draw(&f_p, k_tiles[2].x + 8, k_tiles[2].y + 22, b, 5, pcol, C_PANEL, 4);

    char s[8];
    fmt_power(s, sizeof(s), s_meas.apparent, ok);
    (void)snprintf(b, sizeof(b), "S %s VA", s);
    field_draw(&f_s, k_tiles[2].x + 10, k_tiles[2].y + 58, b, 12, dim, C_PANEL, 1);

    fmt_pf(b, sizeof(b), s_meas.pf, ok);
    field_draw(&f_pf, k_tiles[3].x + 8, k_tiles[3].y + 22, b, 5, ok ? C_AMBER : dim, C_PANEL, 4);

    /* load bar + percent */
    int level = ok ? (int)(load * (float)BAR_SEGS + 0.5f) : 0;
    level = clamp_i(level, 0, BAR_SEGS);
    update_bar(level);

    const int pct = ok ? clamp_i((int)(load * 100.0f + 0.5f), 0, 999) : 0;
    (void)snprintf(b, sizeof(b), "%3d%%", pct);
    field_draw(&f_pct, 276, 191, b, 4, ok ? C_TEXT : dim, C_PANEL, 1);
}

/* ===================================================================================== */
/*  Page: SSR                                                                            */
/* ===================================================================================== */

static void draw_switch_toggle(void)
{
    const rect_t *r = &k_switch_btn;
    const bool on = s_output_on;
    const uint16_t line_color = on ? C_GREEN : C_TEXT_DIM;
    const uint16_t fill_color = on ? ILI9341_Blend(C_PANEL, C_GREEN, 40) : C_PANEL;

    draw_panel(r->x, r->y, r->w, r->h, 14, fill_color, line_color);
    text_center(r->x + r->w / 2, r->y + (r->h - 42) / 2, on ? "ON" : "OFF", line_color, fill_color, 6);
    s_switch_drawn = on ? 1 : 0;
}

static void draw_ssr_static(void)
{
    text_center(SCREEN_W / 2, 32, "SSR OUTPUT", C_CYAN, C_BG, 2);
    text_center(SCREEN_W / 2, 52, "SOLID STATE RELAY", C_TEXT_DIM, C_BG, 1);
    draw_switch_toggle();
    text_center(SCREEN_W / 2, 174, "TAP TO SWITCH", C_TEXT_DIM, C_BG, 1);
    field_reset(&f_load);
}

static void update_ssr(void)
{
    if (s_switch_drawn != (s_output_on ? 1 : 0)) {
        draw_switch_toggle();
    }

    char p[8], b[24];
    fmt_power(p, sizeof(p), s_meas.power, s_meas.link_ok);
    (void)snprintf(b, sizeof(b), "LOAD %s W", p);
    field_draw(&f_load, 88, 192, b, 12, (s_output_on && s_meas.link_ok) ? C_TEXT : C_TEXT_DIM, C_BG, 2);
}

/* ===================================================================================== */
/*  Page: TIMER                                                                          */
/* ===================================================================================== */

#define TBAR_X         11
#define TBAR_Y         108
#define TBAR_SEGS      30
#define TBAR_SEG_W     8
#define TBAR_SEG_PITCH 10
#define TBAR_SEG_H     10

static void draw_timer_bar_segment(int i, bool lit)
{
    fill(TBAR_X + i * TBAR_SEG_PITCH, TBAR_Y, TBAR_SEG_W, TBAR_SEG_H,
         lit ? C_CYAN : ILI9341_Blend(C_BG, C_CYAN, 40));
}

static void update_timer_bar(int level)
{
    if (level == s_timer_bar) {
        return;
    }
    int from = 0, to = TBAR_SEGS;
    if (s_timer_bar >= 0) {
        from = (level < s_timer_bar) ? level : s_timer_bar;
        to   = (level < s_timer_bar) ? s_timer_bar : level;
    }
    for (int i = from; i < to; i++) {
        draw_timer_bar_segment(i, i < level);
    }
    s_timer_bar = level;
}

/** START / STOP button and the +/- buttons follow the timer state. */
static void draw_timer_buttons(void)
{
    const bool run = s_timer_running;

    for (int i = 0; i < 4; i++) {
        draw_button(&k_timer_adj[i], k_timer_adj_label[i], C_CYAN, !run, false, 2);
    }

    if (run) {
        draw_button(&k_timer_start, "STOP", C_RED, true, true, 3);
    } else {
        draw_button(&k_timer_start, "START", C_GREEN, s_output_on, s_output_on, 3);
    }

    /* hint under the button */
    fill(0, 200, SCREEN_W, 10, C_BG);
    if (!run && !s_output_on) {
        text_center(SCREEN_W / 2, 202, "SWITCH THE OUTPUT ON FIRST", C_AMBER, C_BG, 1);
    }
    s_timer_btn = run ? 2 : (s_output_on ? 1 : 0);
}

static void draw_timer_static(void)
{
    text_center(SCREEN_W / 2, 32, "AUTO-OFF TIMER", C_CYAN, C_BG, 2);
    field_reset(&f_timer);
    s_timer_bar = -1;
    update_timer_bar(0);
    draw_timer_buttons();
}

static void update_timer(void)
{
    char b[8];
    const uint16_t secs = s_timer_running ? s_timer_remaining_s : s_timer_set_s;

    (void)snprintf(b, sizeof(b), "%02u:%02u", (unsigned)(secs / 60u), (unsigned)(secs % 60u));
    field_draw(&f_timer, 70, 54, b, 5, s_timer_running ? C_CYAN : C_TEXT, C_BG, 6);

    int level = 0;
    if (s_timer_running && s_timer_set_s > 0u) {
        level = (int)(((uint32_t)s_timer_remaining_s * TBAR_SEGS + s_timer_set_s - 1u) / s_timer_set_s);
        level = clamp_i(level, 0, TBAR_SEGS);
    }
    update_timer_bar(level);

    const int8_t state = s_timer_running ? 2 : (s_output_on ? 1 : 0);
    if (state != s_timer_btn) {
        draw_timer_buttons();
    }
}

/* ===================================================================================== */
/*  Page: SETUP                                                                          */
/* ===================================================================================== */

static void draw_setup_static(void)
{
    text_center(SCREEN_W / 2, 32, "SETUP", C_CYAN, C_BG, 2);
    draw_button(&k_setup_recal, "RECALIBRATE TOUCH", C_CYAN,    true, false, 2);
    draw_button(&k_setup_flip,  "FLIP SCREEN 180",   C_MAGENTA, true, false, 2);

    text_center(SCREEN_W / 2, 164, "HLW8032 SMART OUTLET", C_TEXT_DIM, C_BG, 1);
    text_center(SCREEN_W / 2, 178, "UI NEOFUTURE V1.0", C_TEXT_DIM, C_BG, 1);
    text_center(SCREEN_W / 2, 198, "SWIPE LEFT / RIGHT TO CHANGE TAB", C_CYAN_DIM, C_BG, 1);
}

/* ===================================================================================== */
/*  Page handling                                                                        */
/* ===================================================================================== */

static void draw_page_static(void)
{
    switch (s_page) {
    case UI_PAGE_MONITOR: draw_monitor_static(); break;
    case UI_PAGE_SSR:     draw_ssr_static();     break;
    case UI_PAGE_TIMER:   draw_timer_static();   break;
    case UI_PAGE_SETUP:   draw_setup_static();   break;
    default: break;
    }
}

/** Everything that changes at run time: refresh the footer and the visible page. */
static void update_dynamic(void)
{
    update_footer();
    switch (s_page) {
    case UI_PAGE_MONITOR: update_monitor(); break;
    case UI_PAGE_SSR:     update_ssr();     break;
    case UI_PAGE_TIMER:   update_timer();   break;
    default: break;
    }
    s_meas_dirty = s_output_dirty = s_timer_dirty = false;
}

/** The whole screen from scratch. */
static void draw_all(void)
{
    ILI9341_FillScreen(C_BG);
    draw_tabs();
    draw_footer();
    draw_page_static();
    update_dynamic();
}

/** Switch to another page (clamped, ignored when it is the current one). */
static void go_page(int page)
{
    page = clamp_i(page, 0, UI_PAGE_COUNT - 1);
    if (page == (int)s_page) {
        return;
    }
    s_page = (ui_page_t)page;

    draw_tabs();
    fill(0, CONTENT_Y, SCREEN_W, FOOT_Y - CONTENT_Y, C_BG);
    draw_page_dots();
    draw_page_static();
    update_dynamic();
}

/* ===================================================================================== */
/*  Touch input                                                                          */
/* ===================================================================================== */

/**
 * Turn the touch controller readings into DOWN / MOVE / UP events.
 * A single missing reading must not count as "finger lifted" (the controller sometimes
 * skips a noisy sample), so the finger is considered gone after a few misses in a row.
 */
static touch_event_t touch_poll(uint32_t now, int16_t *x, int16_t *y)
{
    if ((uint32_t)(now - s_touch.last_poll_ms) < TOUCH_POLL_MS) {
        return TOUCH_NONE;
    }
    s_touch.last_poll_ms = now;

    int16_t px, py;
    if (XPT2046_Read(&px, &py)) {
        s_touch.misses = 0u;
        s_touch.x = (int16_t)clamp_i(px, 0, SCREEN_W - 1);
        s_touch.y = (int16_t)clamp_i(py, 0, SCREEN_H - 1);
        *x = s_touch.x;
        *y = s_touch.y;

        if (!s_touch.down) {
            s_touch.down = true;
            return TOUCH_DOWN;
        }
        return TOUCH_MOVE;
    }

    if (s_touch.down && ++s_touch.misses >= TOUCH_RELEASE_MISSES) {
        s_touch.down = false;
        s_touch.misses = 0u;
        *x = s_touch.x;
        *y = s_touch.y;
        return TOUCH_UP;
    }
    return TOUCH_NONE;
}

/** A short touch that did not move: act on what is under the finger. */
static void handle_tap(int x, int y, uint32_t now)
{
    /* tab bar: any page */
    if (y < TAB_H + 2) {
        go_page(x / (SCREEN_W / UI_PAGE_COUNT));
        return;
    }

    switch (s_page) {
    case UI_PAGE_SSR:
        if (rect_hit(&k_switch_btn, x, y)) {
            set_output(!s_output_on);
        }
        break;

    case UI_PAGE_TIMER:
        if (rect_hit(&k_timer_start, x, y)) {
            timer_toggle(now);
        }
        break;

    case UI_PAGE_SETUP:
        if (rect_hit(&k_setup_recal, x, y)) {
            UI_RequestCalibration();
        } else if (rect_hit(&k_setup_flip, x, y)) {
            const ili9341_rotation_t r = (ILI9341_GetRotation() == ILI9341_ROT_90) ? ILI9341_ROT_270 : ILI9341_ROT_90;
            ILI9341_SetRotation(r);              /* the old calibration is wrong now */
            UI_RequestCalibration();
        }
        break;

    default:
        break;
    }
}

/** Finger just landed. The +/- buttons act immediately and then auto-repeat while held. */
static void gesture_down(int x, int y, uint32_t now)
{
    s_gest.x0 = (int16_t)x;
    s_gest.y0 = (int16_t)y;
    s_gest.swiped = false;
    s_gest.moved = false;
    s_gest.consumed = false;
    s_gest.repeat_idx = -1;

    if (s_page == UI_PAGE_TIMER && !s_timer_running) {
        for (int i = 0; i < 4; i++) {
            if (rect_hit(&k_timer_adj[i], x, y)) {
                timer_adjust(k_timer_delta_s[i]);
                s_gest.consumed = true;
                s_gest.repeat_idx = (int8_t)i;
                s_gest.next_repeat_ms = now + REPEAT_DELAY_MS;
            }
        }
    }
}

/**
 * Finger moved. A mostly horizontal drag of SWIPE_MIN_PX or more changes the page right
 * away (without waiting for the finger to lift), once per touch.
 */
static void gesture_move(int x, int y, uint32_t now)
{
    const int dx = x - s_gest.x0;
    const int dy = y - s_gest.y0;

    if (abs_i(dx) > TAP_SLOP_PX || abs_i(dy) > TAP_SLOP_PX) {
        s_gest.moved = true;
    }

    if (!s_gest.swiped && abs_i(dx) >= SWIPE_MIN_PX && abs_i(dx) >= 2 * abs_i(dy)) {
        s_gest.swiped = true;
        s_gest.repeat_idx = -1;
        go_page((int)s_page + ((dx < 0) ? 1 : -1));        /* swipe left = next page */
        return;
    }

    if (s_gest.repeat_idx >= 0 && !s_gest.moved && (int32_t)(now - s_gest.next_repeat_ms) >= 0) {
        timer_adjust(k_timer_delta_s[s_gest.repeat_idx]);
        s_gest.next_repeat_ms = now + REPEAT_PERIOD_MS;
    }
}

static void gesture_up(uint32_t now)
{
    if (!s_gest.swiped && !s_gest.consumed && !s_gest.moved) {
        handle_tap(s_gest.x0, s_gest.y0, now);
    }
}

static void handle_touch(uint32_t now)
{
    int16_t x = 0, y = 0;

    switch (touch_poll(now, &x, &y)) {
    case TOUCH_DOWN: gesture_down(x, y, now); break;
    case TOUCH_MOVE: gesture_move(x, y, now); break;
    case TOUCH_UP:   gesture_up(now);         break;
    default: break;
    }
}

/* ===================================================================================== */
/*  Touch calibration                                                                    */
/* ===================================================================================== */

/** Position of cross number `step`: the four corners, clockwise from the top-left. */
static void cal_target(uint8_t step, int *x, int *y)
{
    const int w = (int)ILI9341_Width();
    const int h = (int)ILI9341_Height();

    *x = (step == 0u || step == 3u) ? CAL_INSET : (w - 1 - CAL_INSET);
    *y = (step == 0u || step == 1u) ? CAL_INSET : (h - 1 - CAL_INSET);
}

static void cal_draw_target(uint8_t step, uint16_t color)
{
    int x, y;
    cal_target(step, &x, &y);

    hline(x - 14, y, 29, color);
    vline(x, y - 14, 29, color);
    ILI9341_DrawCircle((int16_t)x, (int16_t)y, 7, color);
    fill(x - 1, y - 1, 3, 3, color);
}

/** Title + instruction + the cross to tap. */
static void cal_draw_step(void)
{
    char b[32];

    fill(0, 0, SCREEN_W, SCREEN_H, C_BG);
    text_center(SCREEN_W / 2, 88, "TOUCH CALIBRATION", C_CYAN, C_BG, 2);
    (void)snprintf(b, sizeof(b), "TAP THE CROSS  %u / 4", (unsigned)(s_cal.step + 1u));
    text_center(SCREEN_W / 2, 118, b, C_TEXT, C_BG, 1);
    text_center(SCREEN_W / 2, 134, "USE THE TIP OF A FINGER OR A STYLUS", C_TEXT_DIM, C_BG, 1);

    for (uint8_t i = 0u; i < 4u; i++) {                      /* crosses already done stay green */
        if (i < s_cal.step) {
            cal_draw_target(i, C_GREEN);
        }
    }
    cal_draw_target(s_cal.step, C_MAGENTA);
}

static void cal_reset_samples(void)
{
    s_cal.n = 0u;
    s_cal.sum_x = s_cal.sum_y = 0u;
    s_cal.release_count = 0u;
}

static void cal_start(void)
{
    s_mode = MODE_CAL;
    s_touch.down = false;
    s_touch.misses = 0u;
    s_cal.phase = CAL_WAIT_TOUCH;
    s_cal.step = 0u;
    cal_reset_samples();
    cal_draw_step();
}

static void cal_draw_verify(void)
{
    fill(0, 0, SCREEN_W, SCREEN_H, C_BG);
    text_center(SCREEN_W / 2, 20, "CALIBRATION DONE", C_GREEN, C_BG, 2);
    text_center(SCREEN_W / 2, 46, "DRAW ON THE SCREEN TO TEST IT", C_TEXT, C_BG, 1);
    text_center(SCREEN_W / 2, 60, "THEN PRESS SAVE", C_TEXT_DIM, C_BG, 1);
    draw_button(&k_verify_retry, "RETRY", C_AMBER, true, false, 2);
    draw_button(&k_verify_save,  "SAVE",  C_GREEN, true, true,  2);
}

/** All four crosses captured: compute the calibration and decide what happens next. */
static void cal_finish(uint32_t now)
{
    float err = 0.0f;

    for (uint8_t i = 0u; i < 4u; i++) {
        int x, y;
        cal_target(i, &x, &y);
        s_cal.scr[i][0] = (int16_t)x;
        s_cal.scr[i][1] = (int16_t)y;
    }

    if (XPT2046_ComputeCalibration(s_cal.raw, s_cal.scr, &s_cal.result, &err) && err <= CAL_MAX_ERROR_PX) {
        XPT2046_SetCalibration(&s_cal.result);               /* active now, so the test below uses it */
        s_cal.phase = CAL_VERIFY;
        cal_draw_verify();
    } else {
        s_cal.phase = CAL_ERROR;
        s_cal.error_since_ms = now;
        fill(0, 0, SCREEN_W, SCREEN_H, C_BG);
        text_center(SCREEN_W / 2, 96, "CALIBRATION ERROR", C_RED, C_BG, 2);
        text_center(SCREEN_W / 2, 126, "THE TAPS WERE NOT PRECISE ENOUGH", C_TEXT_DIM, C_BG, 1);
        text_center(SCREEN_W / 2, 140, "TRYING AGAIN ...", C_TEXT_DIM, C_BG, 1);
    }
}

/** Store the calibration (this stalls for 1 - 2 s) and go to the normal screens. */
static void cal_save_and_exit(void)
{
    ui_settings_t st;
    memset(&st, 0, sizeof(st));
    st.magic    = SETTINGS_MAGIC;
    st.rotation = (uint8_t)ILI9341_GetRotation();
    st.cal      = s_cal.result;

    fill(60, 100, 200, 40, C_PANEL);
    ILI9341_DrawRect(60, 100, 200, 40, C_CYAN);
    text_center(SCREEN_W / 2, 116, "SAVING ...", C_CYAN, C_PANEL, 2);

    s_save_failed = !FlashStore_Write(&st, (uint32_t)sizeof(st));

    s_mode = MODE_RUN;
    s_touch.down = false;
    s_page = UI_PAGE_MONITOR;
    draw_all();
}

/** Calibration state machine, called instead of the normal UI while calibrating. */
static void cal_task(uint32_t now)
{
    /* error message: show it for a while, then start over */
    if (s_cal.phase == CAL_ERROR) {
        if ((uint32_t)(now - s_cal.error_since_ms) >= CAL_ERROR_SHOW_MS) {
            cal_start();
        }
        return;
    }

    /* test screen: draw a trail under the finger, buttons act when the finger lifts */
    if (s_cal.phase == CAL_VERIFY) {
        int16_t x = 0, y = 0;
        const touch_event_t e = touch_poll(now, &x, &y);

        if ((e == TOUCH_DOWN || e == TOUCH_MOVE) && y < 170) {
            fill(x - 1, y - 1, 3, 3, C_MAGENTA);
        } else if (e == TOUCH_UP) {
            if (rect_hit(&k_verify_save, x, y)) {
                cal_save_and_exit();
            } else if (rect_hit(&k_verify_retry, x, y)) {
                cal_start();
            }
        }
        return;
    }

    /* the two capture phases read the controller directly */
    if ((uint32_t)(now - s_touch.last_poll_ms) < TOUCH_POLL_MS) {
        return;
    }
    s_touch.last_poll_ms = now;

    if (s_cal.phase == CAL_WAIT_RELEASE) {
        if (XPT2046_IsPressed()) {
            s_cal.release_count = 0u;
        } else if (++s_cal.release_count >= 4u) {                     /* finger really gone */
            s_cal.step++;
            cal_reset_samples();
            if (s_cal.step >= 4u) {
                cal_finish(now);
            } else {
                s_cal.phase = CAL_WAIT_TOUCH;
                cal_draw_step();
            }
        }
        return;
    }

    /* CAL_WAIT_TOUCH: average CAL_SAMPLES steady readings */
    uint16_t rx, ry;
    if (!XPT2046_ReadRaw(&rx, &ry)) {
        return;
    }
    if (s_cal.n == 0u) {
        s_cal.first_x = rx;
        s_cal.first_y = ry;
    } else if (abs_i((int)rx - (int)s_cal.first_x) > CAL_JITTER_RAW ||
               abs_i((int)ry - (int)s_cal.first_y) > CAL_JITTER_RAW) {
        cal_reset_samples();                                          /* finger is still moving: start over */
        return;
    }

    s_cal.sum_x += rx;
    s_cal.sum_y += ry;
    s_cal.n++;

    if (s_cal.n >= CAL_SAMPLES) {
        s_cal.raw[s_cal.step][0] = (uint16_t)(s_cal.sum_x / s_cal.n);
        s_cal.raw[s_cal.step][1] = (uint16_t)(s_cal.sum_y / s_cal.n);
        cal_draw_target(s_cal.step, C_GREEN);                         /* captured */
        s_cal.phase = CAL_WAIT_RELEASE;
        s_cal.release_count = 0u;
    }
}

/* ===================================================================================== */
/*  Public API                                                                           */
/* ===================================================================================== */

/**
 * Start the UI.
 *   1) load the stored settings; if they are valid, use their rotation and calibration
 *   2) otherwise start the calibration
 *   3) the output always starts OFF
 */
void UI_Init(const ui_hooks_t *hooks)
{
    memset(&s_hooks, 0, sizeof(s_hooks));
    if (hooks != NULL) {
        s_hooks = *hooks;
    }

    s_output_on = true;                       /* force set_output() below to really run */
    set_output(false);

    s_page = UI_PAGE_MONITOR;
    s_save_failed = false;
    memset(&s_meas, 0, sizeof(s_meas));

    ui_settings_t st;
    const bool have = FlashStore_Read(&st, (uint32_t)sizeof(st)) &&
                      st.magic == SETTINGS_MAGIC &&
                      (st.rotation == (uint8_t)ILI9341_ROT_90 || st.rotation == (uint8_t)ILI9341_ROT_270);

    if (have) {
        ILI9341_SetRotation((ili9341_rotation_t)st.rotation);
        XPT2046_SetCalibration(&st.cal);
        s_mode = MODE_RUN;
        draw_all();
    } else {
        if (ILI9341_Width() != SCREEN_W) {                    /* the layout needs landscape */
            ILI9341_SetRotation(ILI9341_ROT_90);
        }
        cal_start();
    }
}

void UI_RequestCalibration(void)
{
    s_request_cal = true;
}

void UI_SetMeasurements(const ui_meas_t *meas)
{
    if (meas != NULL) {
        s_meas = *meas;
        s_meas_dirty = true;
    }
}

bool UI_IsOutputOn(void) { return s_output_on; }
ui_page_t UI_GetPage(void) { return s_page; }

/**
 * One pass of the UI. Order matters:
 *   1) a calibration request starts the calibration screens
 *   2) calibrating? then only the calibration runs
 *   3) timer, touch/gestures, then redraw what changed
 */
void UI_Task(void)
{
    const uint32_t now = HAL_GetTick();

    if (s_request_cal) {
        s_request_cal = false;
        if (s_mode != MODE_CAL) {
            cal_start();
        }
    }

    if (s_mode == MODE_CAL) {
        cal_task(now);
        return;
    }

    timer_tick(now);
    handle_touch(now);

    if (s_meas_dirty || s_output_dirty || s_timer_dirty) {
        update_dynamic();
    }
}