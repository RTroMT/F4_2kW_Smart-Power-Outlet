#ifndef ILI9341_H
#define ILI9341_H

#include "main.h"            /* STM32 HAL (GPIO + SPI types) */
#include <stdint.h>
#include <stdbool.h>

/**
 * ILI9341 2.4" TFT (240 x 320, RGB565) driver for STM32 HAL, SPI in polling mode.
 *
 * What it gives you:   init, rotation, fill / rectangle / line / circle / round-rect,
 *                      built-in 5x7 font (scalable), RGB565 bitmaps.
 * Touch is NOT handled here (the module's XPT2046 is read over plain GPIO, separate file).
 *
 * ---- CubeMX settings ------------------------------------------------------------------
 *  SPI1   : Master, 8-bit, MSB first, CPOL Low, CPHA 1 Edge, NSS Software.
 *           Prescaler 4 -> 84 MHz / 4 = 21 MHz (safe). Prescaler 2 (42 MHz) works on many
 *           modules with short wires; if the picture gets noisy go back to 4.
 *  GPIO   : CS, DC, RST (and optional backlight) as Output Push-Pull, no pull,
 *           speed Very High. Give them the pins you wired in the macros below.
 *  Wiring : SCK -> SPI1_SCK, SDI(MOSI) -> SPI1_MOSI. The display's SDO(MISO) is unused.
 *           LED (backlight) -> 3V3 through the module's own resistor, or to a GPIO (BL macros).
 *
 * ---- Blocking time (important with the HLW8032 ring buffer) -----------------------------
 *  Everything is polled, so a call blocks until the SPI transfer is finished.
 *  At 21 MHz a full-screen fill takes about 60 ms. The HLW8032 DMA ring holds ~290 ms of
 *  data, so redraw only what changed (numbers, bars) and keep every single call short.
 */

/* ======================= Hardware connection (EDIT THESE) ======================= */
#ifndef ILI9341_SPI
#define ILI9341_SPI         hspi1            /* SPI handle created by CubeMX        */
#endif

#ifndef ILI9341_CS_PORT
#define ILI9341_CS_PORT     GPIOA            /* chip select                         */
#define ILI9341_CS_PIN      GPIO_PIN_4
#endif

#ifndef ILI9341_DC_PORT
#define ILI9341_DC_PORT     GPIOB            /* data / command                      */
#define ILI9341_DC_PIN      GPIO_PIN_0
#endif

#ifndef ILI9341_RST_PORT
#define ILI9341_RST_PORT    GPIOB            /* reset (delete these two lines if the
                                                module's RST is tied to 3V3)        */
#define ILI9341_RST_PIN     GPIO_PIN_1
#endif

/* Optional backlight pin. Leave undefined when the LED pin is wired to 3V3.
 *   #define ILI9341_BL_PORT  GPIOB
 *   #define ILI9341_BL_PIN   GPIO_PIN_2
 */

/* Orientation used right after ILI9341_Init() */
#ifndef ILI9341_DEFAULT_ROTATION
#define ILI9341_DEFAULT_ROTATION   ILI9341_ROT_90    /* landscape 320 x 240 */
#endif

/* ============================= Types and colours ============================= */
typedef enum {
    ILI9341_ROT_0 = 0,      /* portrait   240 x 320 */
    ILI9341_ROT_90,         /* landscape  320 x 240 */
    ILI9341_ROT_180,        /* portrait   240 x 320, upside down */
    ILI9341_ROT_270         /* landscape  320 x 240, upside down */
} ili9341_rotation_t;

/* Colour = 16-bit RGB565 */
#define ILI9341_RGB(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | (((b) & 0xF8) >> 3)))

#define ILI9341_BLACK      0x0000
#define ILI9341_WHITE      0xFFFF
#define ILI9341_RED        0xF800
#define ILI9341_GREEN      0x07E0
#define ILI9341_BLUE       0x001F
#define ILI9341_YELLOW     0xFFE0
#define ILI9341_CYAN       0x07FF
#define ILI9341_MAGENTA    0xF81F
#define ILI9341_ORANGE     0xFD20
#define ILI9341_GREY       0x8410
#define ILI9341_DARKGREY   0x4208
#define ILI9341_NAVY       0x000F

/* Font cell: every character occupies 6 x 8 pixels at scale 1 (5x7 glyph + spacing) */
#define ILI9341_CHAR_W     6
#define ILI9341_CHAR_H     8

/* ================================ Functions ================================ */

/** Reset the panel, load the register setup, clear it to black, turn the backlight on. */
void ILI9341_Init(void);

/** Change orientation. Coordinates and ILI9341_Width()/Height() follow the new rotation. */
void ILI9341_SetRotation(ili9341_rotation_t rotation);
uint16_t ILI9341_Width(void);                    /* current width  in pixels */
uint16_t ILI9341_Height(void);                   /* current height in pixels */
ili9341_rotation_t ILI9341_GetRotation(void);    /* rotation set last */

/** Backlight on/off (does nothing when no ILI9341_BL_* pin is defined). */
void ILI9341_SetBacklight(bool on);

/* ---- Shapes. All of them clip to the screen: x/y may be negative or too large. ---- */
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ILI9341_DrawPixel(int16_t x, int16_t y, uint16_t color);
void ILI9341_DrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void ILI9341_DrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void ILI9341_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ILI9341_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);   /* outline */
void ILI9341_DrawCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color);          /* outline */
void ILI9341_FillCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color);
void ILI9341_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);

/* ---- Colour blending and gradients (neon / futuristic looks) ---- */
/** Mix two RGB565 colours: t = 0 gives a, t = 255 gives b, in between is linear. */
uint16_t ILI9341_Blend(uint16_t a, uint16_t b, uint8_t t);

/** Rectangle whose colour changes from `top` (first row) to `bottom` (last row). */
void ILI9341_FillGradientV(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t top, uint16_t bottom);

/** Rectangle whose colour changes from `left` (first column) to `right` (last column). */
void ILI9341_FillGradientH(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t left, uint16_t right);

/** Draw w x h pixels of RGB565 data (row by row, normal uint16_t values). */
void ILI9341_DrawBitmap565(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pixels);

/* ---- Text. (x, y) is the top-left corner of the first character cell. ----
 * The background colour is painted too, so drawing a new value over an old one needs no
 * erasing step and does not flicker. scale 1..16 multiplies the 6x8 cell.
 * Characters outside 32..126 are shown as '?'. '\n' starts a new line.                */
void ILI9341_DrawChar(int16_t x, int16_t y, char c, uint16_t fg, uint16_t bg, uint8_t scale);
void ILI9341_DrawString(int16_t x, int16_t y, const char *text, uint16_t fg, uint16_t bg, uint8_t scale);

/** Width in pixels of a single-line string (for centring / right alignment). */
uint16_t ILI9341_TextWidth(const char *text, uint8_t scale);

#endif /* ILI9341_H */