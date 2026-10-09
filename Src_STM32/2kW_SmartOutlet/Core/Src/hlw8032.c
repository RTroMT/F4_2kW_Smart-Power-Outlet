#include "hlw8032.h"

void HLW8032_Init(HLW8032_Data_t *out){
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

HLW8032_Status_t HLW8032_Parse_Data(const uint8_t *in_data, size_t len, HLW8032_Data_t *out_data, const HLW8032_Config_t *config) {
    //  Edge case #1: NULL pointer
    if (in_data == NULL || out_data == NULL || config == NULL)  return HLW8032_ERR_ARG;

    //  Edge case #2: invalid length
    if (len < HLW8032_FRAME_LENGTH) return HLW8032_ERR_LEN;
    out_data->is_valid = false;

    //  Variables
    uint8_t crc = 0;
    uint8_t state_reg = in_data[0];
    uint8_t updata_reg = in_data[20];

    bool v_ready = (updata_reg & (1 << 6)) != 0;
    bool i_ready = (updata_reg & (1 << 5)) != 0;
    bool p_ready = (updata_reg & (1 << 4)) != 0;

    //  Checksum check
    for (int i = 2; i < 23; i++) {
        crc += in_data[i];
    }
    if (crc != in_data[23]) return HLW8032_ERR_CHECKSUM;

    //  Check state register for invalid/overflow value
    if (state_reg != 0x55 && (state_reg & 0xf0) != 0xf0) return HLW8032_ERR_STATE;
    if ((state_reg & 0xf0) == 0xf0 && (state_reg & 0x01)) return HLW8032_ERR_PARAM;

    //  Init overflow bool
    out_data->voltage_overflow = false;
    out_data->current_overflow = false;
    out_data->power_overflow = false;
    //  Init UPDATED bool value
    //  Reset only after the frame is validated, so rejected frames leave out_data untouched
    out_data->voltage_updated = false;
    out_data->current_updated = false;
    out_data->power_updated   = false;

    if ((state_reg & 0xf0) == 0xf0) {
        /**@brief
         *  -   The overflow flag indicate that the register is overflow
         *      which equal to the real value is too small
         *  -   Therefore, this code will turn 0 the flag and in calculation
         *      will have to set the real value to 0 if the flag is true
         */
        if ((state_reg & 0x02) == 0x02) {
            out_data->power_overflow = true;
            out_data->power_active = 0.0f;
            out_data->power_updated = true;
            //  out_data->power_apparent = 0.0f;
        }
        if ((state_reg & 0x04) == 0x04) {
            out_data->current_overflow = true;
            out_data->current = 0.0f;
            out_data->current_updated = true;
        }
        if ((state_reg & 0x08) == 0x08) {
            out_data->voltage_overflow = true;
            out_data->voltage = 0.0f;
            out_data->voltage_updated = true;
        }
    }



    //  Parse raw register value and calculated real values
    if (!out_data->voltage_overflow) {
        uint32_t voltage_param_raw = (uint32_t)((uint32_t)in_data[2] << 16 | (uint32_t)in_data[3] << 8 | in_data[4]) ;
        uint32_t voltage_value_raw = (uint32_t)((uint32_t)in_data[5] << 16 | (uint32_t)in_data[6] << 8 | in_data[7]);

        if (v_ready && (voltage_value_raw != 0)) {
            out_data->voltage = (float)(((float)voltage_param_raw / (float)voltage_value_raw) * config->voltage_coef);
            out_data->voltage_updated = true;
        }
    }

    if (!out_data->current_overflow) {
        uint32_t current_param_raw = (uint32_t)((uint32_t)in_data[8] << 16  | (uint32_t)in_data[9] << 8  | in_data[10]);
        uint32_t current_value_raw = (uint32_t)((uint32_t)in_data[11] << 16 | (uint32_t)in_data[12] << 8 | in_data[13]);

        if (i_ready && (current_value_raw != 0)) {
            out_data->current = (float)(((float)current_param_raw / (float)current_value_raw) * config->current_coef);
            out_data->current_updated = true;
        }
    }

    if (!out_data->power_overflow) {
        uint32_t power_param_raw = ((uint32_t)in_data[14] << 16) | ((uint32_t)in_data[15] << 8) | in_data[16];
        uint32_t power_value_raw = ((uint32_t)in_data[17] << 16) | ((uint32_t)in_data[18] << 8) | in_data[19];

        if (p_ready && (power_value_raw != 0)) {
            out_data->power_active = ((float)power_param_raw / (float)power_value_raw) * config->voltage_coef * config->current_coef;
            out_data->power_updated = true;
        }
    }

    out_data->power_apparent = out_data->voltage * out_data->current;
    if (out_data->power_apparent > 0.001f) {
        out_data->power_factor = out_data->power_active / out_data->power_apparent;
        if (out_data->power_factor > 1.0f) {
            out_data->power_factor = 1.0f;
        }
    } else {
        out_data->power_factor = 0.0f;
    }

    out_data->is_valid = true;
    return HLW8032_OK;
}
