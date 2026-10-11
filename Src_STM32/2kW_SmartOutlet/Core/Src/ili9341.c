#include "ili9341.h"
#include "spi.h"
#include <string.h>

/* ============================ Constants ============================ */
#define SPI_TIMEOUT_MS    100u
#define BUF_BYTES         2048u              /* RAM used to stream pixels in big chunks */

#define NATIVE_WIDTH      240u               /* panel size in rotation 0 */
#define NATIVE_HEIGHT     320u

/* ILI9341 command codes (datasheet) */
#define CMD_SWRESET       0x01u
#define CMD_SLPOUT        0x11u
#define CMD_DISPON        0x29u
#define CMD_CASET         0x2Au              /* column address set */
#define CMD_PASET         0x2Bu              /* page (row) address set */
#define CMD_RAMWR         0x2Cu              /* memory write: pixel data follows */
#define CMD_MADCTL        0x36u              /* memory access control (rotation, RGB/BGR) */

/* Font geometry */
#define FONT_FIRST        32
#define FONT_LAST         126
#define GLYPH_COLS        5
#define GLYPH_ROWS        7
#define SCALE_MAX         16

/* ============================ State ============================ */
static uint8_t  s_buf[BUF_BYTES];            /* pixel streaming buffer (big-endian RGB565) */
static uint16_t s_width  = NATIVE_WIDTH;     /* current size, depends on rotation */
static uint16_t s_height = NATIVE_HEIGHT;
static ili9341_rotation_t s_rotation = ILI9341_ROT_0;   /* last rotation that was set */

/* ===================== Register setup (standard ILI9341 init) ===================== */
typedef struct {
    uint8_t cmd;
    uint8_t len;                             /* number of parameter bytes */
    uint8_t data[15];
} init_step_t;

static const init_step_t k_init_steps[] = {
    { 0xEFu,  3, { 0x03, 0x80, 0x02 } },
    { 0xCFu,  3, { 0x00, 0xC1, 0x30 } },
    { 0xEDu,  4, { 0x64, 0x03, 0x12, 0x81 } },
    { 0xE8u,  3, { 0x85, 0x00, 0x78 } },
    { 0xCBu,  5, { 0x39, 0x2C, 0x00, 0x34, 0x02 } },
    { 0xF7u,  1, { 0x20 } },
    { 0xEAu,  2, { 0x00, 0x00 } },
    { 0xC0u,  1, { 0x23 } },                              /* power control 1 (VRH) */
    { 0xC1u,  1, { 0x10 } },                              /* power control 2 */
    { 0xC5u,  2, { 0x3E, 0x28 } },                        /* VCOM control 1 */
    { 0xC7u,  1, { 0x86 } },                              /* VCOM control 2 */
    { 0x37u,  1, { 0x00 } },                              /* vertical scroll start = 0 */
    { 0x3Au,  1, { 0x55 } },                              /* pixel format: 16 bit / pixel */
    { 0xB1u,  2, { 0x00, 0x18 } },                        /* frame rate control */
    { 0xB6u,  3, { 0x08, 0x82, 0x27 } },                  /* display function control */
    { 0xF2u,  1, { 0x00 } },                              /* 3-gamma function off */
    { 0x26u,  1, { 0x01 } },                              /* gamma curve 1 */
    { 0xE0u, 15, { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                   0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 } },   /* positive gamma */
    { 0xE1u, 15, { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                   0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F } },   /* negative gamma */
};

/* ===================== Font: classic 5x7, columns, bit0 = top row ===================== */
/* ASCII 32 ' ' ... 126 '~', 5 bytes per character */
static const uint8_t k_font5x7[(FONT_LAST - FONT_FIRST + 1) * GLYPH_COLS] = {
    0x00, 0x00, 0x00, 0x00, 0x00,   /* ' ' */
    0x00, 0x00, 0x5F, 0x00, 0x00,   /* '!' */
    0x00, 0x07, 0x00, 0x07, 0x00,   /* '"' */
    0x14, 0x7F, 0x14, 0x7F, 0x14,   /* '#' */
    0x24, 0x2A, 0x7F, 0x2A, 0x12,   /* '$' */
    0x23, 0x13, 0x08, 0x64, 0x62,   /* '%' */
    0x36, 0x49, 0x55, 0x22, 0x50,   /* '&' */
    0x00, 0x05, 0x03, 0x00, 0x00,   /* ''' */
    0x00, 0x1C, 0x22, 0x41, 0x00,   /* '(' */
    0x00, 0x41, 0x22, 0x1C, 0x00,   /* ')' */
    0x14, 0x08, 0x3E, 0x08, 0x14,   /* '*' */
    0x08, 0x08, 0x3E, 0x08, 0x08,   /* '+' */
    0x00, 0x50, 0x30, 0x00, 0x00,   /* ',' */
    0x08, 0x08, 0x08, 0x08, 0x08,   /* '-' */
    0x00, 0x60, 0x60, 0x00, 0x00,   /* '.' */
    0x20, 0x10, 0x08, 0x04, 0x02,   /* '/' */
    0x3E, 0x51, 0x49, 0x45, 0x3E,   /* '0' */
    0x00, 0x42, 0x7F, 0x40, 0x00,   /* '1' */
    0x42, 0x61, 0x51, 0x49, 0x46,   /* '2' */
    0x21, 0x41, 0x45, 0x4B, 0x31,   /* '3' */
    0x18, 0x14, 0x12, 0x7F, 0x10,   /* '4' */
    0x27, 0x45, 0x45, 0x45, 0x39,   /* '5' */
    0x3C, 0x4A, 0x49, 0x49, 0x30,   /* '6' */
    0x01, 0x71, 0x09, 0x05, 0x03,   /* '7' */
    0x36, 0x49, 0x49, 0x49, 0x36,   /* '8' */
    0x06, 0x49, 0x49, 0x29, 0x1E,   /* '9' */
    0x00, 0x36, 0x36, 0x00, 0x00,   /* ':' */
    0x00, 0x56, 0x36, 0x00, 0x00,   /* ';' */
    0x08, 0x14, 0x22, 0x41, 0x00,   /* '<' */
    0x14, 0x14, 0x14, 0x14, 0x14,   /* '=' */
    0x00, 0x41, 0x22, 0x14, 0x08,   /* '>' */
    0x02, 0x01, 0x51, 0x09, 0x06,   /* '?' */
    0x32, 0x49, 0x79, 0x41, 0x3E,   /* '@' */
    0x7E, 0x11, 0x11, 0x11, 0x7E,   /* 'A' */
    0x7F, 0x49, 0x49, 0x49, 0x36,   /* 'B' */
    0x3E, 0x41, 0x41, 0x41, 0x22,   /* 'C' */
    0x7F, 0x41, 0x41, 0x22, 0x1C,   /* 'D' */
    0x7F, 0x49, 0x49, 0x49, 0x41,   /* 'E' */
    0x7F, 0x09, 0x09, 0x09, 0x01,   /* 'F' */
    0x3E, 0x41, 0x49, 0x49, 0x7A,   /* 'G' */
    0x7F, 0x08, 0x08, 0x08, 0x7F,   /* 'H' */
    0x00, 0x41, 0x7F, 0x41, 0x00,   /* 'I' */
    0x20, 0x40, 0x41, 0x3F, 0x01,   /* 'J' */
    0x7F, 0x08, 0x14, 0x22, 0x41,   /* 'K' */
    0x7F, 0x40, 0x40, 0x40, 0x40,   /* 'L' */
    0x7F, 0x02, 0x0C, 0x02, 0x7F,   /* 'M' */
    0x7F, 0x04, 0x08, 0x10, 0x7F,   /* 'N' */
    0x3E, 0x41, 0x41, 0x41, 0x3E,   /* 'O' */
    0x7F, 0x09, 0x09, 0x09, 0x06,   /* 'P' */
    0x3E, 0x41, 0x51, 0x21, 0x5E,   /* 'Q' */
    0x7F, 0x09, 0x19, 0x29, 0x46,   /* 'R' */
    0x46, 0x49, 0x49, 0x49, 0x31,   /* 'S' */
    0x01, 0x01, 0x7F, 0x01, 0x01,   /* 'T' */
    0x3F, 0x40, 0x40, 0x40, 0x3F,   /* 'U' */
    0x1F, 0x20, 0x40, 0x20, 0x1F,   /* 'V' */
    0x3F, 0x40, 0x38, 0x40, 0x3F,   /* 'W' */
    0x63, 0x14, 0x08, 0x14, 0x63,   /* 'X' */
    0x07, 0x08, 0x70, 0x08, 0x07,   /* 'Y' */
    0x61, 0x51, 0x49, 0x45, 0x43,   /* 'Z' */
    0x00, 0x7F, 0x41, 0x41, 0x00,   /* '[' */
    0x02, 0x04, 0x08, 0x10, 0x20,   /* '\' */
    0x00, 0x41, 0x41, 0x7F, 0x00,   /* ']' */
    0x04, 0x02, 0x01, 0x02, 0x04,   /* '^' */
    0x40, 0x40, 0x40, 0x40, 0x40,   /* '_' */
    0x00, 0x01, 0x02, 0x04, 0x00,   /* '`' */
    0x20, 0x54, 0x54, 0x54, 0x78,   /* 'a' */
    0x7F, 0x48, 0x44, 0x44, 0x38,   /* 'b' */
    0x38, 0x44, 0x44, 0x44, 0x20,   /* 'c' */
    0x38, 0x44, 0x44, 0x48, 0x7F,   /* 'd' */
    0x38, 0x54, 0x54, 0x54, 0x18,   /* 'e' */
    0x08, 0x7E, 0x09, 0x01, 0x02,   /* 'f' */
    0x0C, 0x52, 0x52, 0x52, 0x3E,   /* 'g' */
    0x7F, 0x08, 0x04, 0x04, 0x78,   /* 'h' */
    0x00, 0x44, 0x7D, 0x40, 0x00,   /* 'i' */
    0x20, 0x40, 0x44, 0x3D, 0x00,   /* 'j' */
    0x7F, 0x10, 0x28, 0x44, 0x00,   /* 'k' */
    0x00, 0x41, 0x7F, 0x40, 0x00,   /* 'l' */
    0x7C, 0x04, 0x18, 0x04, 0x78,   /* 'm' */
    0x7C, 0x08, 0x04, 0x04, 0x78,   /* 'n' */
    0x38, 0x44, 0x44, 0x44, 0x38,   /* 'o' */
    0x7C, 0x14, 0x14, 0x14, 0x08,   /* 'p' */
    0x08, 0x14, 0x14, 0x18, 0x7C,   /* 'q' */
    0x7C, 0x08, 0x04, 0x04, 0x08,   /* 'r' */
    0x48, 0x54, 0x54, 0x54, 0x20,   /* 's' */
    0x04, 0x3F, 0x44, 0x40, 0x20,   /* 't' */
    0x3C, 0x40, 0x40, 0x20, 0x7C,   /* 'u' */
    0x1C, 0x20, 0x40, 0x20, 0x1C,   /* 'v' */
    0x3C, 0x40, 0x30, 0x40, 0x3C,   /* 'w' */
    0x44, 0x28, 0x10, 0x28, 0x44,   /* 'x' */
    0x0C, 0x50, 0x50, 0x50, 0x3C,   /* 'y' */
    0x44, 0x64, 0x54, 0x4C, 0x44,   /* 'z' */
    0x00, 0x08, 0x36, 0x41, 0x00,   /* '{' */
    0x00, 0x00, 0x7F, 0x00, 0x00,   /* '|' */
    0x00, 0x41, 0x36, 0x08, 0x00,   /* '}' */
    0x08, 0x04, 0x08, 0x10, 0x08,   /* '~' */
};


/* ===================================================================================== */
/*  Level 1: pins and raw SPI                                                            */
/* ===================================================================================== */

static inline void cs_low(void)  { HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET);   }
static inline void dc_low(void)  { HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET); }
static inline void dc_high(void) { HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);   }

/* Send raw bytes (blocking). HAL_SPI_Transmit returns when the last bit has left the pin. */
static void spi_write(const uint8_t *data, uint16_t len)
{
    (void)HAL_SPI_Transmit(&ILI9341_SPI, (uint8_t *)data, len, SPI_TIMEOUT_MS);
}

/* Send one command byte (DC low), then leave DC high so parameter / pixel bytes can follow.
 * CS must already be low. */
static void write_cmd(uint8_t cmd)
{
    dc_low();
    spi_write(&cmd, 1u);
    dc_high();
}

/* Send a whole command with its parameters, framed by CS. */
static void send_command(uint8_t cmd, const uint8_t *params, uint8_t len)
{
    cs_low();
    write_cmd(cmd);
    if (len > 0u) {
        spi_write(params, len);
    }
    cs_high();
}


/* ===================================================================================== */
/*  Level 2: address window and pixel streaming                                          */
/* ===================================================================================== */

/**
 * Choose the rectangle that the next pixel data will fill. CS must be low.
 *
 * The controller then writes the pixels we send left-to-right, top-to-bottom inside this
 * rectangle by itself, so one window + one data burst paints a whole rectangle.
 * Steps: column range -> row range -> "memory write" command (pixel bytes follow).
 */
static void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];

    /* 1) columns x0..x1 (each value is 16 bit, high byte first) */
    write_cmd(CMD_CASET);
    d[0] = (uint8_t)(x0 >> 8);  d[1] = (uint8_t)(x0 & 0xFFu);
    d[2] = (uint8_t)(x1 >> 8);  d[3] = (uint8_t)(x1 & 0xFFu);
    spi_write(d, 4u);

    /* 2) rows y0..y1 */
    write_cmd(CMD_PASET);
    d[0] = (uint8_t)(y0 >> 8);  d[1] = (uint8_t)(y0 & 0xFFu);
    d[2] = (uint8_t)(y1 >> 8);  d[3] = (uint8_t)(y1 & 0xFFu);
    spi_write(d, 4u);

    /* 3) from now on every byte is pixel data */
    write_cmd(CMD_RAMWR);
}

/**
 * Send `count` pixels of the same colour. CS must be low and the window already set.
 *
 * The colour is copied into s_buf once, then that buffer is sent over and over, so a big
 * rectangle costs a few large SPI transfers instead of one transfer per pixel.
 */
static void stream_color(uint16_t color, uint32_t count)
{
    const uint8_t hi = (uint8_t)(color >> 8);
    const uint8_t lo = (uint8_t)(color & 0xFFu);

    /* 1) fill the buffer with as many pixels as it can hold (or as many as we need) */
    uint32_t chunk_px = BUF_BYTES / 2u;
    if (count < chunk_px) {
        chunk_px = count;
    }
    for (uint32_t i = 0; i < chunk_px; i++) {
        s_buf[2u * i]      = hi;
        s_buf[2u * i + 1u] = lo;
    }

    /* 2) send it until all pixels are out */
    while (count > 0u) {
        uint32_t n = (count < chunk_px) ? count : chunk_px;
        spi_write(s_buf, (uint16_t)(n * 2u));
        count -= n;
    }
}

/* Clip a rectangle to the screen. Returns false when nothing is left to draw. */
static bool clip_rect(int16_t *x, int16_t *y, int16_t *w, int16_t *h)
{
    if (*w <= 0 || *h <= 0) {
        return false;
    }

    int32_t x0 = *x;
    int32_t y0 = *y;
    int32_t x1 = (int32_t)*x + *w;           /* exclusive */
    int32_t y1 = (int32_t)*y + *h;           /* exclusive */

    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 > (int32_t)s_width)  { x1 = (int32_t)s_width;  }
    if (y1 > (int32_t)s_height) { y1 = (int32_t)s_height; }

    if (x0 >= x1 || y0 >= y1) {
        return false;
    }

    *x = (int16_t)x0;
    *y = (int16_t)y0;
    *w = (int16_t)(x1 - x0);
    *h = (int16_t)(y1 - y0);
    return true;
}


/* ===================================================================================== */
/*  Init, rotation, backlight                                                            */
/* ===================================================================================== */

void ILI9341_SetBacklight(bool on)
{
#if defined(ILI9341_BL_PORT) && defined(ILI9341_BL_PIN)
    HAL_GPIO_WritePin(ILI9341_BL_PORT, ILI9341_BL_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
}

/**
 * Orientation and colour order are both set by one register (MADCTL).
 * The 0x08 bit selects BGR order, which is what the usual ILI9341 modules need so that
 * ILI9341_RED really looks red.
 */
void ILI9341_SetRotation(ili9341_rotation_t rotation)
{
    static const uint8_t k_madctl[4] = { 0x48u, 0x28u, 0x88u, 0xE8u };

    if ((unsigned)rotation > 3u) {
        rotation = ILI9341_ROT_0;
    }

    /* 1) tell the panel */
    send_command(CMD_MADCTL, &k_madctl[rotation], 1u);

    /* 2) remember rotation and the size we have to clip to */
    s_rotation = rotation;
    if (rotation == ILI9341_ROT_0 || rotation == ILI9341_ROT_180) {
        s_width  = NATIVE_WIDTH;
        s_height = NATIVE_HEIGHT;
    } else {
        s_width  = NATIVE_HEIGHT;
        s_height = NATIVE_WIDTH;
    }
}

uint16_t ILI9341_Width(void)  { return s_width;  }
uint16_t ILI9341_Height(void) { return s_height; }
ili9341_rotation_t ILI9341_GetRotation(void) { return s_rotation; }

/**
 * Bring the panel from power-up to a cleared, visible picture.
 */
void ILI9341_Init(void)
{
    /* 1) idle levels of the control lines */
    cs_high();
    dc_high();
    ILI9341_SetBacklight(false);

    /* 2) hardware reset (only when the RST pin is wired) */
#ifdef ILI9341_RST_PORT
    HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(120);
#endif

    /* 3) software reset */
    send_command(CMD_SWRESET, NULL, 0u);
    HAL_Delay(150);

    /* 4) register setup: power, timing, gamma, 16-bit pixels ... */
    for (uint32_t i = 0; i < (sizeof(k_init_steps) / sizeof(k_init_steps[0])); i++) {
        send_command(k_init_steps[i].cmd, k_init_steps[i].data, k_init_steps[i].len);
    }

    /* 5) wake the panel up and switch the display on */
    send_command(CMD_SLPOUT, NULL, 0u);
    HAL_Delay(120);
    send_command(CMD_DISPON, NULL, 0u);
    HAL_Delay(20);

    /* 6) orientation, clear the random power-up content, then show the picture */
    ILI9341_SetRotation(ILI9341_DEFAULT_ROTATION);
    ILI9341_FillScreen(ILI9341_BLACK);
    ILI9341_SetBacklight(true);
}


/* ===================================================================================== */
/*  Basic shapes                                                                         */
/* ===================================================================================== */

/**
 * Fill a rectangle with one colour. Every other filled shape is built on this function.
 */
void ILI9341_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    /* 1) keep only the part that is on the screen */
    if (!clip_rect(&x, &y, &w, &h)) {
        return;
    }

    /* 2) window + one burst of pixels */
    cs_low();
    set_window((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    stream_color(color, (uint32_t)w * (uint32_t)h);
    cs_high();
}

void ILI9341_FillScreen(uint16_t color)
{
    ILI9341_FillRect(0, 0, (int16_t)s_width, (int16_t)s_height, color);
}

void ILI9341_DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= (int16_t)s_width || y >= (int16_t)s_height) {
        return;
    }

    const uint8_t px[2] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFFu) };

    cs_low();
    set_window((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
    spi_write(px, 2u);
    cs_high();
}

void ILI9341_DrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    ILI9341_FillRect(x, y, w, 1, color);
}

void ILI9341_DrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    ILI9341_FillRect(x, y, 1, h, color);
}

/* Rectangle outline, 1 pixel wide, made of four thin filled rectangles. */
void ILI9341_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    ILI9341_FillRect(x,                  y,                  w, 1, color);   /* top    */
    ILI9341_FillRect(x,                  (int16_t)(y + h - 1), w, 1, color);   /* bottom */
    ILI9341_FillRect(x,                  y,                  1, h, color);   /* left   */
    ILI9341_FillRect((int16_t)(x + w - 1), y,                  1, h, color);   /* right  */
}

/**
 * Straight line (Bresenham). Horizontal and vertical lines take the fast rectangle path;
 * slanted lines are drawn pixel by pixel, which is slow, so use them sparingly.
 */
void ILI9341_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    /* 1) fast cases */
    if (y0 == y1) {
        if (x1 < x0) { int16_t t = x0; x0 = x1; x1 = t; }
        ILI9341_FillRect(x0, y0, (int16_t)(x1 - x0 + 1), 1, color);
        return;
    }
    if (x0 == x1) {
        if (y1 < y0) { int16_t t = y0; y0 = y1; y1 = t; }
        ILI9341_FillRect(x0, y0, 1, (int16_t)(y1 - y0 + 1), color);
        return;
    }

    /* 2) general case: step along the line, keep the rounding error in `err` */
    int32_t dx =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int32_t dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int32_t sx = (x0 < x1) ? 1 : -1;
    int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx + dy;
    int32_t x = x0;
    int32_t y = y0;

    for (;;) {
        ILI9341_DrawPixel((int16_t)x, (int16_t)y, color);
        if (x == x1 && y == y1) {
            break;
        }
        int32_t e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

/* Integer square root (floor), the radii here are small so a simple loop is fine. */
static uint32_t isqrt_u32(uint32_t n)
{
    uint32_t r = 0u;
    while ((r + 1u) * (r + 1u) <= n) {
        r++;
    }
    return r;
}

/* Circle outline (midpoint algorithm): compute one octant, mirror it 8 times. */
void ILI9341_DrawCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    if (r < 0) {
        return;
    }

    int32_t x = r;
    int32_t y = 0;
    int32_t err = 1 - r;

    while (x >= y) {
        ILI9341_DrawPixel((int16_t)(cx + x), (int16_t)(cy + y), color);
        ILI9341_DrawPixel((int16_t)(cx - x), (int16_t)(cy + y), color);
        ILI9341_DrawPixel((int16_t)(cx + x), (int16_t)(cy - y), color);
        ILI9341_DrawPixel((int16_t)(cx - x), (int16_t)(cy - y), color);
        ILI9341_DrawPixel((int16_t)(cx + y), (int16_t)(cy + x), color);
        ILI9341_DrawPixel((int16_t)(cx - y), (int16_t)(cy + x), color);
        ILI9341_DrawPixel((int16_t)(cx + y), (int16_t)(cy - x), color);
        ILI9341_DrawPixel((int16_t)(cx - y), (int16_t)(cy - x), color);

        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

/* Filled circle: one horizontal line per row, as wide as the circle is on that row. */
void ILI9341_FillCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    if (r < 0) {
        return;
    }

    for (int32_t dy = -r; dy <= r; dy++) {
        int32_t half = (int32_t)isqrt_u32((uint32_t)((int32_t)r * r - dy * dy));
        ILI9341_FillRect((int16_t)(cx - half), (int16_t)(cy + dy), (int16_t)(2 * half + 1), 1, color);
    }
}

/**
 * Filled rectangle with rounded corners (radius r), the base shape for cards and buttons.
 *
 * The middle part is one plain rectangle. In the top and bottom `r` rows the line gets
 * shorter towards the edge, following a quarter circle: `inset` is how far each end is
 * pulled in on that row.
 */
void ILI9341_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    if (w <= 0 || h <= 0) {
        return;
    }

    /* 1) the radius cannot be bigger than half of the shortest side */
    int16_t max_r = (int16_t)(((w < h) ? w : h) / 2);
    if (r > max_r) { r = max_r; }
    if (r < 0)     { r = 0;     }

    /* 2) straight middle part */
    ILI9341_FillRect(x, (int16_t)(y + r), w, (int16_t)(h - 2 * r), color);

    /* 3) rounded top and bottom rows, mirrored */
    for (int32_t i = 0; i < r; i++) {
        int32_t dist  = (int32_t)r - i;                                   /* distance to the arc centre line */
        int32_t inset = (int32_t)r - (int32_t)isqrt_u32((uint32_t)((int32_t)r * r - dist * dist));
        int16_t line_w = (int16_t)(w - 2 * inset);

        ILI9341_FillRect((int16_t)(x + inset), (int16_t)(y + i),         line_w, 1, color);   /* top    */
        ILI9341_FillRect((int16_t)(x + inset), (int16_t)(y + h - 1 - i), line_w, 1, color);   /* bottom */
    }
}

/* ===================================================================================== */
/*  Colour blending and gradients                                                        */
/* ===================================================================================== */

/**
 * Mix two RGB565 colours channel by channel (5 bit red, 6 bit green, 5 bit blue).
 * t = 0 -> a, t = 255 -> b.
 */
uint16_t ILI9341_Blend(uint16_t a, uint16_t b, uint8_t t)
{
    const uint32_t ia = 255u - t;
    const uint32_t r  = (((a >> 11) & 0x1Fu) * ia + ((b >> 11) & 0x1Fu) * t + 127u) / 255u;
    const uint32_t g  = (((a >> 5)  & 0x3Fu) * ia + ((b >> 5)  & 0x3Fu) * t + 127u) / 255u;
    const uint32_t bl = (((a)       & 0x1Fu) * ia + ((b)       & 0x1Fu) * t + 127u) / 255u;

    return (uint16_t)((r << 11) | (g << 5) | bl);
}

/**
 * Vertical gradient: one window for the whole rectangle, then one burst per row.
 * The colour of a row depends on its position inside the ORIGINAL rectangle, so a
 * rectangle that is partly off-screen still shows the right part of the gradient.
 */
void ILI9341_FillGradientV(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t top, uint16_t bottom)
{
    int16_t cx = x, cy = y, cw = w, ch = h;
    if (!clip_rect(&cx, &cy, &cw, &ch)) {
        return;
    }

    cs_low();
    set_window((uint16_t)cx, (uint16_t)cy, (uint16_t)(cx + cw - 1), (uint16_t)(cy + ch - 1));

    for (int32_t row = 0; row < ch; row++) {
        const int32_t pos = (int32_t)(cy - y) + row;                    /* row inside the original rectangle */
        const uint8_t t   = (h > 1) ? (uint8_t)((pos * 255) / (h - 1)) : 0u;
        stream_color(ILI9341_Blend(top, bottom, t), (uint32_t)cw);
    }
    cs_high();
}

/**
 * Horizontal gradient: build ONE row of colours in the buffer, then send that row for
 * every line of the rectangle.
 */
void ILI9341_FillGradientH(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t left, uint16_t right)
{
    int16_t cx = x, cy = y, cw = w, ch = h;
    if (!clip_rect(&cx, &cy, &cw, &ch)) {
        return;
    }

    /* 1) the row of colours */
    for (int32_t col = 0; col < cw; col++) {
        const int32_t pos = (int32_t)(cx - x) + col;                    /* column inside the original rectangle */
        const uint8_t t   = (w > 1) ? (uint8_t)((pos * 255) / (w - 1)) : 0u;
        const uint16_t c  = ILI9341_Blend(left, right, t);
        s_buf[2 * col]     = (uint8_t)(c >> 8);
        s_buf[2 * col + 1] = (uint8_t)(c & 0xFFu);
    }

    /* 2) the same row for every line */
    cs_low();
    set_window((uint16_t)cx, (uint16_t)cy, (uint16_t)(cx + cw - 1), (uint16_t)(cy + ch - 1));
    for (int32_t row = 0; row < ch; row++) {
        spi_write(s_buf, (uint16_t)(cw * 2));
    }
    cs_high();
}

/**
 * Copy an RGB565 image to the screen. `pixels` are normal uint16_t values (row after row);
 * they are converted to the byte order the panel wants while being sent.
 */
void ILI9341_DrawBitmap565(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pixels)
{
    if (pixels == NULL) {
        return;
    }

    /* 1) visible part of the image */
    int16_t cx = x, cy = y, cw = w, ch = h;
    if (!clip_rect(&cx, &cy, &cw, &ch)) {
        return;
    }
    const int32_t skip_x = (int32_t)cx - x;       /* columns cut off on the left */
    const int32_t skip_y = (int32_t)cy - y;       /* rows cut off at the top     */

    /* 2) one window for the visible part, then its rows one after the other */
    cs_low();
    set_window((uint16_t)cx, (uint16_t)cy, (uint16_t)(cx + cw - 1), (uint16_t)(cy + ch - 1));

    for (int32_t row = 0; row < ch; row++) {
        const uint16_t *src = pixels + (skip_y + row) * (int32_t)w + skip_x;
        int32_t done = 0;

        while (done < cw) {
            int32_t n = cw - done;
            if (n > (int32_t)(BUF_BYTES / 2u)) {
                n = (int32_t)(BUF_BYTES / 2u);
            }
            for (int32_t i = 0; i < n; i++) {
                s_buf[2 * i]     = (uint8_t)(src[done + i] >> 8);
                s_buf[2 * i + 1] = (uint8_t)(src[done + i] & 0xFFu);
            }
            spi_write(s_buf, (uint16_t)(n * 2));
            done += n;
        }
    }
    cs_high();
}


/* ===================================================================================== */
/*  Text                                                                                 */
/* ===================================================================================== */

/**
 * Draw one character in a (6*scale) x (8*scale) cell, background included.
 *
 * Fast path (cell fully on screen): one window for the whole cell, then the cell is sent
 * row by row. A glyph row is built once as a line of pixels and sent `scale` times.
 * Slow path (cell touches the screen edge): paint the background, then every lit glyph
 * dot as a small clipped square.
 */
void ILI9341_DrawChar(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale)
{
    /* 1) sanitize the arguments */
    if (scale == 0u)         { scale = 1u; }
    if (scale > SCALE_MAX)   { scale = SCALE_MAX; }
    if (c < FONT_FIRST || c > FONT_LAST) { c = '?'; }

    const uint8_t *glyph = &k_font5x7[((int)c - FONT_FIRST) * GLYPH_COLS];
    const int16_t cell_w = (int16_t)(ILI9341_CHAR_W * scale);
    const int16_t cell_h = (int16_t)(ILI9341_CHAR_H * scale);

    /* 2) slow path: the cell is not completely on the screen */
    if (x < 0 || y < 0 || (int32_t)x + cell_w > (int32_t)s_width || (int32_t)y + cell_h > (int32_t)s_height) {
        ILI9341_FillRect(x, y, cell_w, cell_h, bg);
        for (int32_t col = 0; col < GLYPH_COLS; col++) {
            for (int32_t row = 0; row < GLYPH_ROWS; row++) {
                if ((glyph[col] >> row) & 1u) {
                    ILI9341_FillRect((int16_t)(x + col * scale), (int16_t)(y + row * scale), scale, scale, fg);
                }
            }
        }
        return;
    }

    /* 3) fast path */
    const uint8_t fg_hi = (uint8_t)(fg >> 8), fg_lo = (uint8_t)(fg & 0xFFu);
    const uint8_t bg_hi = (uint8_t)(bg >> 8), bg_lo = (uint8_t)(bg & 0xFFu);

    cs_low();
    set_window((uint16_t)x, (uint16_t)y, (uint16_t)(x + cell_w - 1), (uint16_t)(y + cell_h - 1));

    for (int32_t row = 0; row < ILI9341_CHAR_H; row++) {
        /* 3a) build one line of the cell: 6 glyph columns, each `scale` pixels wide.
         *     Column 5 and row 7 are the blank spacing around the 5x7 glyph. */
        uint32_t pos = 0u;
        for (int32_t col = 0; col < ILI9341_CHAR_W; col++) {
            const bool on = (col < GLYPH_COLS) && (row < GLYPH_ROWS) && (((glyph[col] >> row) & 1u) != 0u);
            for (uint8_t s = 0; s < scale; s++) {
                s_buf[pos++] = on ? fg_hi : bg_hi;
                s_buf[pos++] = on ? fg_lo : bg_lo;
            }
        }

        /* 3b) the same line is repeated `scale` times to make the glyph taller */
        for (uint8_t s = 0; s < scale; s++) {
            spi_write(s_buf, (uint16_t)pos);
        }
    }
    cs_high();
}

/* Draw a string. '\n' returns to the starting x and moves one text line down. */
void ILI9341_DrawString(int16_t x, int16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (text == NULL) {
        return;
    }
    if (scale == 0u)       { scale = 1u; }
    if (scale > SCALE_MAX) { scale = SCALE_MAX; }

    int16_t cursor_x = x;

    while (*text != '\0') {
        if (*text == '\n') {
            cursor_x = x;
            y = (int16_t)(y + ILI9341_CHAR_H * scale);
        } else {
            ILI9341_DrawChar(cursor_x, y, *text, fg, bg, scale);
            cursor_x = (int16_t)(cursor_x + ILI9341_CHAR_W * scale);
        }
        text++;
    }
}

uint16_t ILI9341_TextWidth(const char *text, uint8_t scale)
{
    if (text == NULL) {
        return 0u;
    }
    if (scale == 0u)       { scale = 1u; }
    if (scale > SCALE_MAX) { scale = SCALE_MAX; }

    return (uint16_t)(strlen(text) * ILI9341_CHAR_W * scale);
}
