#ifndef XPT2046_H
#define XPT2046_H

#include "main.h"            /* STM32 HAL + the pin labels CubeMX generated */
#include <stdint.h>
#include <stdbool.h>

/**
 * XPT2046 resistive touch controller (the touch chip on the 2.4" ILI9341 modules),
 * read by "bit-banging" it over 5 plain GPIO pins. No extra SPI peripheral is used.
 *
 * ---- Pins (CubeMX user labels, see the macros below) -------------------------------------
 *   TOUCH_CS   output  chip select (idle high)
 *   TOUCH_CLK  output  clock       (idle low)
 *   TOUCH_DIN  output  MCU -> XPT2046 (the module's T_DIN)
 *   TOUCH_DO   input   XPT2046 -> MCU (the module's T_DO), pull-up
 *   TOUCH_IRQ  input   goes LOW while the screen is pressed (the module's T_IRQ), pull-up
 *
 * ---- How a touch becomes a screen position ---------------------------------------------
 *   1) XPT2046_ReadRaw()  returns the 12-bit ADC values of the X and Y plates (0..4095).
 *   2) A calibration turns raw values into pixels:
 *          x = a*raw_x + b*raw_y + c        y = d*raw_x + e*raw_y + f
 *      The 6 numbers come from XPT2046_ComputeCalibration(), which is fed with 4 touches
 *      on known points (the four corners). Using an "affine" formula (not just scale and
 *      offset) also copes with swapped / mirrored axes, so it does not matter how the
 *      panel is mounted or how it is rotated.
 */

/* ======================= Pins (default = the CubeMX labels) ======================= */
#ifndef XPT2046_CS_PORT
#define XPT2046_CS_PORT    TOUCH_CS_GPIO_Port
#define XPT2046_CS_PIN     TOUCH_CS_Pin
#define XPT2046_CLK_PORT   TOUCH_CLK_GPIO_Port
#define XPT2046_CLK_PIN    TOUCH_CLK_Pin
#define XPT2046_DIN_PORT   TOUCH_DIN_GPIO_Port
#define XPT2046_DIN_PIN    TOUCH_DIN_Pin
#define XPT2046_DO_PORT    TOUCH_DO_GPIO_Port
#define XPT2046_DO_PIN     TOUCH_DO_Pin
#define XPT2046_IRQ_PORT   TOUCH_IRQ_GPIO_Port
#define XPT2046_IRQ_PIN    TOUCH_IRQ_Pin
#endif

/* Busy-wait length of every half clock period. The chip allows 2 MHz at most; with the
 * GPIO call overhead the default gives roughly 0.5..1 MHz at 84 MHz core clock. */
#ifndef XPT2046_BIT_DELAY_LOOPS
#define XPT2046_BIT_DELAY_LOOPS   4u
#endif

/* ================================== Types ================================== */

/** screen_x = a*raw_x + b*raw_y + c,   screen_y = d*raw_x + e*raw_y + f */
typedef struct {
    float a, b, c;
    float d, e, f;
} xpt2046_cal_t;

/* ================================ Functions ================================ */

/** Put the pins in their idle state. The pins themselves are configured by CubeMX. */
void XPT2046_Init(void);

/** Cheap check of the IRQ pin: true while something presses the screen. */
bool XPT2046_IsPressed(void);

/**
 * Read the raw X / Y values (12 bit) of the current touch.
 * Several readings are taken and filtered. Returns false when nobody touches the
 * screen or when the readings are not stable (finger just landed or just left).
 */
bool XPT2046_ReadRaw(uint16_t *raw_x, uint16_t *raw_y);

/**
 * Read the touch position in screen pixels (needs a calibration).
 * The result may be a little outside the screen near the edges: clip it in the caller.
 */
bool XPT2046_Read(int16_t *x, int16_t *y);

/** Use this calibration from now on. */
void XPT2046_SetCalibration(const xpt2046_cal_t *cal);
bool XPT2046_GetCalibration(xpt2046_cal_t *cal);        /* false when none is set */
bool XPT2046_HasCalibration(void);

/** raw -> screen with a given calibration (pure maths, no hardware access). */
void XPT2046_Transform(const xpt2046_cal_t *cal, uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y);

/**
 * Calculate a calibration from 4 touches.
 *   raw[i][0..1]    raw X / Y measured while the user touched point i
 *   screen[i][0..1] where point i really is on the screen (pixels)
 * A least-squares fit is used, so a slightly imprecise tap is averaged out.
 *
 * Returns false when the points cannot give a usable calibration (e.g. the user tapped
 * the same place four times). *max_error_px (optional) gets the biggest distance, in
 * pixels, between where a tap should be and where the calibration puts it: a big value
 * means the taps were sloppy and the calibration should be repeated.
 */
bool XPT2046_ComputeCalibration(const uint16_t raw[4][2], const int16_t screen[4][2],
                                xpt2046_cal_t *out, float *max_error_px);

#endif /* XPT2046_H */