#include "xpt2046.h"

/* ============================ Constants ============================ */
/* Control bytes: start bit | channel | 12 bit | differential | power-down between reads */
#define CMD_READ_X        0xD0u
#define CMD_READ_Y        0x90u

#define SAMPLES           5u        /* readings per axis, the middle ones are averaged */
#define MAX_SPREAD_RAW    120u      /* readings further apart than this are not stable */
#define RAW_MIN           40u       /* ~0 or ~4095 means "open circuit / no touch"     */
#define RAW_MAX           4055u

/* ============================ State ============================ */
static xpt2046_cal_t s_cal;
static bool          s_has_cal = false;

/* ===================================================================================== */
/*  Level 1: the five pins                                                               */
/* ===================================================================================== */

static inline void cs_low(void)   { HAL_GPIO_WritePin(XPT2046_CS_PORT,  XPT2046_CS_PIN,  GPIO_PIN_RESET); }
static inline void cs_high(void)  { HAL_GPIO_WritePin(XPT2046_CS_PORT,  XPT2046_CS_PIN,  GPIO_PIN_SET);   }
static inline void clk_low(void)  { HAL_GPIO_WritePin(XPT2046_CLK_PORT, XPT2046_CLK_PIN, GPIO_PIN_RESET); }
static inline void clk_high(void) { HAL_GPIO_WritePin(XPT2046_CLK_PORT, XPT2046_CLK_PIN, GPIO_PIN_SET);   }

static inline void din_write(bool level)
{
    HAL_GPIO_WritePin(XPT2046_DIN_PORT, XPT2046_DIN_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline bool do_read(void)
{
    return HAL_GPIO_ReadPin(XPT2046_DO_PORT, XPT2046_DO_PIN) == GPIO_PIN_SET;
}

/* Short busy-wait. `volatile` stops the compiler from removing the loop. */
static void bit_delay(void)
{
    for (volatile uint32_t i = 0u; i < XPT2046_BIT_DELAY_LOOPS; i++) {
        __NOP();
    }
}

/* ===================================================================================== */
/*  Level 2: one conversion                                                              */
/* ===================================================================================== */

/**
 * Run one conversion and return its 12-bit result.
 *
 * Timing (SPI mode 0: the chip takes DIN on the rising clock edge and changes DOUT on the
 * falling edge):
 *     8 clocks  : control byte, MSB first (start bit, channel, 12-bit mode, power mode)
 *     1 clock   : "busy" bit, always 0
 *    12 clocks  : conversion result, MSB first
 *     3 clocks  : zeros
 * All 16 bits after the control byte are shifted in, then shifted right by 3.
 */
static uint16_t xpt_transfer(uint8_t cmd)
{
    uint16_t value = 0u;

    cs_low();
    clk_low();

    /* 1) control byte: put the bit on DIN, give the rising edge, go back low */
    for (int8_t bit = 7; bit >= 0; bit--) {
        din_write(((cmd >> bit) & 1u) != 0u);
        bit_delay();
        clk_high();
        bit_delay();
        clk_low();
    }
    din_write(false);

    /* 2) let the chip finish the conversion (a few microseconds) */
    for (uint8_t i = 0u; i < 6u; i++) {
        bit_delay();
    }

    /* 3) 16 clocks: busy bit + 12 result bits + 3 zeros. DOUT is stable at the rising edge. */
    for (uint8_t i = 0u; i < 16u; i++) {
        bit_delay();
        clk_high();
        bit_delay();
        value = (uint16_t)((value << 1) | (do_read() ? 1u : 0u));
        clk_low();
    }

    cs_high();
    return (uint16_t)((value >> 3) & 0x0FFFu);
}

/* Sort a small array (insertion sort). */
static void sort_u16(uint16_t *v, uint8_t n)
{
    for (uint8_t i = 1u; i < n; i++) {
        const uint16_t key = v[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && v[j] > key) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = key;
    }
}

/**
 * Turn SAMPLES readings into one value: sort them, throw away the lowest and the highest,
 * average the rest. Returns false when the remaining readings are too far apart.
 */
static bool filter_samples(uint16_t *s, uint16_t *result)
{
    sort_u16(s, (uint8_t)SAMPLES);

    if ((uint16_t)(s[SAMPLES - 2u] - s[1]) > MAX_SPREAD_RAW) {
        return false;                                  /* noisy: finger moving or just landing */
    }

    const uint32_t sum = (uint32_t)s[1] + (uint32_t)s[2] + (uint32_t)s[3];
    *result = (uint16_t)(sum / 3u);

    return (*result >= RAW_MIN) && (*result <= RAW_MAX);
}

/* ===================================================================================== */
/*  Public: reading                                                                      */
/* ===================================================================================== */

void XPT2046_Init(void)
{
    cs_high();
    clk_low();
    din_write(false);
    (void)xpt_transfer(CMD_READ_X);       /* one conversion so the chip starts in a known state */
}

bool XPT2046_IsPressed(void)
{
    return HAL_GPIO_ReadPin(XPT2046_IRQ_PORT, XPT2046_IRQ_PIN) == GPIO_PIN_RESET;
}

bool XPT2046_ReadRaw(uint16_t *raw_x, uint16_t *raw_y)
{
    uint16_t xs[SAMPLES];
    uint16_t ys[SAMPLES];

    /* 1) nothing to do when the screen is not pressed */
    if (!XPT2046_IsPressed()) {
        return false;
    }

    /* 2) the first conversion after power-down is not reliable: read it and drop it */
    (void)xpt_transfer(CMD_READ_X);

    /* 3) take the real readings, X and Y alternately */
    for (uint8_t i = 0u; i < SAMPLES; i++) {
        xs[i] = xpt_transfer(CMD_READ_X);
        ys[i] = xpt_transfer(CMD_READ_Y);
    }

    /* 4) the finger must still be there, otherwise the numbers are meaningless */
    if (!XPT2046_IsPressed()) {
        return false;
    }

    /* 5) filter both axes */
    uint16_t fx, fy;
    if (!filter_samples(xs, &fx) || !filter_samples(ys, &fy)) {
        return false;
    }

    if (raw_x != NULL) { *raw_x = fx; }
    if (raw_y != NULL) { *raw_y = fy; }
    return true;
}

/* ===================================================================================== */
/*  Public: calibration                                                                  */
/* ===================================================================================== */

void XPT2046_SetCalibration(const xpt2046_cal_t *cal)
{
    if (cal != NULL) {
        s_cal = *cal;
        s_has_cal = true;
    }
}

bool XPT2046_GetCalibration(xpt2046_cal_t *cal)
{
    if (!s_has_cal || cal == NULL) {
        return false;
    }
    *cal = s_cal;
    return true;
}

bool XPT2046_HasCalibration(void)
{
    return s_has_cal;
}

/* float -> int16 with rounding and without overflow */
static int16_t round_to_i16(float v)
{
    v += (v >= 0.0f) ? 0.5f : -0.5f;
    if (v >  32767.0f) { v =  32767.0f; }
    if (v < -32768.0f) { v = -32768.0f; }
    return (int16_t)v;
}

void XPT2046_Transform(const xpt2046_cal_t *cal, uint16_t raw_x, uint16_t raw_y, int16_t *x, int16_t *y)
{
    const float rx = (float)raw_x;
    const float ry = (float)raw_y;

    if (x != NULL) { *x = round_to_i16(cal->a * rx + cal->b * ry + cal->c); }
    if (y != NULL) { *y = round_to_i16(cal->d * rx + cal->e * ry + cal->f); }
}

bool XPT2046_Read(int16_t *x, int16_t *y)
{
    uint16_t rx, ry;

    if (!s_has_cal || !XPT2046_ReadRaw(&rx, &ry)) {
        return false;
    }
    XPT2046_Transform(&s_cal, rx, ry, x, y);
    return true;
}

/* Determinant of a 3x3 matrix. */
static double det3(const double m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

/* Solve M * p = rhs with Cramer's rule. Returns false for a (nearly) singular matrix. */
static bool solve3(const double m[3][3], const double rhs[3], double p[3])
{
    const double d = det3(m);
    if (d > -1e-9 && d < 1e-9) {
        return false;
    }

    for (int col = 0; col < 3; col++) {
        double t[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                t[r][c] = (c == col) ? rhs[r] : m[r][c];
            }
        }
        p[col] = det3(t) / d;
    }
    return true;
}

/**
 * Least-squares fit of  screen = a*u + b*v + c  (u, v = centred raw values / 1000) for x
 * and for y, then converted back to plain raw units.
 * Centring and scaling keep the numbers small so the 3x3 system is well conditioned.
 */
bool XPT2046_ComputeCalibration(const uint16_t raw[4][2], const int16_t screen[4][2],
                                xpt2046_cal_t *out, float *max_error_px)
{
    if (raw == NULL || screen == NULL || out == NULL) {
        return false;
    }

    /* 1) centre of the raw points */
    double mx = 0.0, my = 0.0;
    for (int i = 0; i < 4; i++) {
        mx += raw[i][0];
        my += raw[i][1];
    }
    mx /= 4.0;
    my /= 4.0;

    /* 2) sums for the normal equations */
    double suu = 0, suv = 0, svv = 0, su = 0, sv = 0;
    double tux = 0, tvx = 0, tx = 0, tuy = 0, tvy = 0, ty = 0;
    for (int i = 0; i < 4; i++) {
        const double u = ((double)raw[i][0] - mx) / 1000.0;
        const double v = ((double)raw[i][1] - my) / 1000.0;
        const double X = screen[i][0];
        const double Y = screen[i][1];

        suu += u * u;  suv += u * v;  svv += v * v;  su += u;  sv += v;
        tux += u * X;  tvx += v * X;  tx += X;
        tuy += u * Y;  tvy += v * Y;  ty += Y;
    }

    const double m[3][3] = { { suu, suv, su }, { suv, svv, sv }, { su, sv, 4.0 } };
    const double rhs_x[3] = { tux, tvx, tx };
    const double rhs_y[3] = { tuy, tvy, ty };
    double px[3], py[3];

    if (!solve3(m, rhs_x, px) || !solve3(m, rhs_y, py)) {
        return false;
    }

    /* 3) back to raw units:  X = a_u*(rx-mx)/1000 + ... */
    const double a = px[0] / 1000.0, b = px[1] / 1000.0, c = px[2] - a * mx - b * my;
    const double d = py[0] / 1000.0, e = py[1] / 1000.0, f = py[2] - d * mx - e * my;

    out->a = (float)a;  out->b = (float)b;  out->c = (float)c;
    out->d = (float)d;  out->e = (float)e;  out->f = (float)f;

    /* 4) how far is every tap from where the calibration puts it? */
    if (max_error_px != NULL) {
        float worst = 0.0f;
        for (int i = 0; i < 4; i++) {
            int16_t x, y;
            XPT2046_Transform(out, raw[i][0], raw[i][1], &x, &y);
            float ex = (float)(x - screen[i][0]);
            float ey = (float)(y - screen[i][1]);
            if (ex < 0.0f) { ex = -ex; }
            if (ey < 0.0f) { ey = -ey; }
            if (ex > worst) { worst = ex; }
            if (ey > worst) { worst = ey; }
        }
        *max_error_px = worst;
    }
    return true;
}