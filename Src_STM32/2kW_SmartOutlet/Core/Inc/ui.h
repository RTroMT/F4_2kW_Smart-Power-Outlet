#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * "Neo-future" touch user interface for the 320 x 240 ILI9341 + XPT2046.
 *
 * Pages (swipe left / right anywhere, or tap a tab at the top):
 *   MONITOR  voltage, current, active power, power factor, load bar
 *   SSR      big ON / OFF switch for the outlet (solid state relay)
 *   TIMER    auto-off countdown
 *   SETUP    touch re-calibration, flip the screen
 *
 * First start (nothing stored in flash): the user is asked to tap 4 crosses in the screen
 * corners; the result is checked, tested on screen and saved to flash. Holding a finger
 * on the screen while powering up forces the calibration again.
 *
 * The UI never blocks: UI_Task() does a little work per call, so it can run in the same
 * main loop as the HLW8032 receiver. Only saving the calibration stalls for 1 - 2 s.
 */

#define UI_PAGE_COUNT   4

typedef enum {
    UI_PAGE_MONITOR = 0,
    UI_PAGE_SSR,
    UI_PAGE_TIMER,
    UI_PAGE_SETUP
} ui_page_t;

/** Measurements shown on the MONITOR page. Units: V, A, W, VA, power factor 0..1. */
typedef struct {
    float voltage;
    float current;
    float power;           /* active power */
    float apparent;        /* apparent power */
    float pf;
    bool  link_ok;         /* false -> the HLW8032 is silent: values are shown as dashes */
} ui_meas_t;

/** Hardware access that the UI needs from the application. */
typedef struct {
    void (*set_output)(bool on);    /* switch the SSR; may be NULL while there is no hardware */
} ui_hooks_t;

/** Call once after ILI9341_Init() and XPT2046_Init(). Output starts OFF. */
void UI_Init(const ui_hooks_t *hooks);

/** Start the touch calibration (e.g. because a finger was on the screen at power-up). */
void UI_RequestCalibration(void);

/** Hand over new measurements (call when a new HLW8032 frame arrived or the link state changed). */
void UI_SetMeasurements(const ui_meas_t *meas);

/** Call on every pass of the main loop: touch, gestures, timer, redraw of what changed. */
void UI_Task(void);

bool      UI_IsOutputOn(void);
ui_page_t UI_GetPage(void);

#endif /* UI_H */