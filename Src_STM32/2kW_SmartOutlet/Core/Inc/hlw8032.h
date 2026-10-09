//
// Created by Admin on 10/8/2026.
//

#ifndef INC_2KW_SMARTOUTLET_HLW8032_H
#define INC_2KW_SMARTOUTLET_HLW8032_H


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#define HLW8032_FRAME_LENGTH 24

/* Frame layout (datasheet REV 1.5, 24 bytes, UART 4800 8E1, LSB first):
 *  [0]      State REG
 *  [1]      Check REG (always 0x5A)
 *  [2..4]   Voltage parameter   [5..7]   Voltage REG
 *  [8..10]  Current parameter   [11..13] Current REG
 *  [14..16] Power parameter     [17..19] Power REG
 *  [20]     Data update REG (bit7 PF carry, bit6 V, bit5 I, bit4 P)
 *  [21..22] PF REG
 *  [23]     CheckSum (low 8 bits of sum of bytes 2..22)
 */

typedef enum {
    HLW8032_OK = 0,
    HLW8032_ERR_ARG,        /* NULL pointer                                   */
    HLW8032_ERR_LEN,        /* len < HLW8032_FRAME_LEN                        */
    HLW8032_ERR_CHECKSUM,   /* checksum mismatch -> corrupted / misaligned    */
    HLW8032_ERR_STATE,      /* State REG is 0xAA or an undocumented value     */
    HLW8032_ERR_PARAM       /* State REG 0xFx with bit0 = 1: params unusable  */
} HLW8032_Status_t;

typedef struct {
    float voltage;
    float current;
    float power_active;
    float power_apparent;
    float power_factor;

    bool voltage_overflow;
    bool current_overflow;
    bool power_overflow;

    bool voltage_updated;   /* this frame refreshed the value */
    bool current_updated;
    bool power_updated;

    bool is_valid;
}   HLW8032_Data_t;

typedef struct {
    /**@brief
     *  V_COEF IS CALCULATED BY FORMULA: voltage_coef = (R_divider_top / R_divider_bottom) / 1000
     *  THE BOARD IS CONFIG WITH 4 x 470k OVER 1k  -> V_COEF = 1.88
     *
     *  I_COEF IS CALCULATED BY FORMULA: current_coef = 1 / (R_shunt * 1000)
     *  THE BOARD IS CONFIG WITH SHUNT 0,001R -> I_COEF = 1.0
     */

    float voltage_coef;
    float current_coef;
}   HLW8032_Config_t;

HLW8032_Status_t HLW8032_Parse_Data(const uint8_t *in_data, size_t len, HLW8032_Data_t *out_data, const HLW8032_Config_t *config);
void HLW8032_Init(HLW8032_Data_t *out_data);

#endif //INC_2KW_SMARTOUTLET_HLW8032_H
