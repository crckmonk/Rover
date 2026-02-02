#include "lsm303.h"

#include <math.h>

/*
    TODO: 
        - Refactor to fit code style
        - implement oversampling/Filtering
        - Use DMA 
        - implement temperature reading
        - Calibration - DONE

*/

#define DECLINATION_ANGLE 8.0f // Determined by location


/* private variables */
static I2C_HandleTypeDef *lsm303dlhc_i2c = NULL;
static LSM303_MagGain_t lsm303dlhc_mag_magin = LSM303DLHC_MAGGAIN_1_3;
static bool lsm303dlhc_mag_auto_range = false;
static float lsm303dlhc_acc_mg_lsb = 0.001f;   // 1, 2, 4 or 12 mg per lsb
static float lsm303dlhc_mag_gauss_lsb_xy = 1100.0f;  // Varies with gain
static float lsm303dlhc_mag_gauss_lsb_z = 980.0f;   // Varies with gain
static LSM303_MagCalibration_t mag_cal = {0, 0, 0, 1.0f, 1.0f, 1.0f};
static LSM303_AccCalibration_t acc_cal = {0, 0, 0};


/* private functions */
static LSM303_Result_t lsm303dlhc_read_i2c(uint8_t address, uint8_t reg, uint8_t *data);
static LSM303_Result_t lsm303dlhc_write_i2c(uint8_t address, uint8_t reg, uint8_t data);

LSM303_Result_t LSM303_InitAcc(I2C_HandleTypeDef *i2c, const LSM303_AccInit_t *init) {
    uint8_t reg1_a;
    lsm303dlhc_i2c = i2c;

    if (lsm303dlhc_i2c == NULL || init == NULL) {
        return LSM303DLHC_ERROR;
    }

    /* set control registers */
    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG1_A, init->ctrl_reg1_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG2_A, init->ctrl_reg2_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG3_A, init->ctrl_reg3_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (LSM303_SetAccScale(init->ctrl_reg4_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG5_A, init->ctrl_reg5_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG6_A, init->ctrl_reg6_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    /* LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check if we are connected or not */
    if (lsm303dlhc_read_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG1_A, &reg1_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }
    if (reg1_a != init->ctrl_reg1_a) {
        return LSM303DLHC_ERROR;
    }

    return LSM303DLHC_OK;
}

LSM303_Result_t LSM303_SetAccScale(uint8_t ctrl_reg4_a) {
    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_ACC, LSM303DLHC_REG_ACC_CTRL_REG4_A, ctrl_reg4_a) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if ((ctrl_reg4_a & 0b110000) == LSM303DLHC_ACR4A_FS10_1MG) {
        lsm303dlhc_acc_mg_lsb = 0.001f;
    } else if ((ctrl_reg4_a & 0b110000) == LSM303DLHC_ACR4A_FS10_2MG) {
        lsm303dlhc_acc_mg_lsb = 0.002f;
    } else if ((ctrl_reg4_a & 0b110000) == LSM303DLHC_ACR4A_FS10_4MG) {
        lsm303dlhc_acc_mg_lsb = 0.004f;
    } else if ((ctrl_reg4_a & 0b110000) == LSM303DLHC_ACR4A_FS10_12MG) {
        lsm303dlhc_acc_mg_lsb = 0.012f;
    }

    return LSM303DLHC_OK;
}

LSM303_Result_t LSM303_ReadAccRaw(LSM303_RawData_t *data) {
    uint8_t reg = LSM303DLHC_REG_ACC_OUT_X_L_A | 0x80;
    uint8_t buf[6] = { 0 };

    if (HAL_I2C_Master_Transmit(lsm303dlhc_i2c, LSM303DLHC_ADDR_ACC, &reg, 1, 1000) != HAL_OK) {
        return LSM303DLHC_ERROR;
    }

    if (HAL_I2C_Master_Receive(lsm303dlhc_i2c, LSM303DLHC_ADDR_ACC, buf, 6, 1000) != HAL_OK) {
        return LSM303DLHC_ERROR;
    }

    /* raw data (low byte first) */
    data->x = (int16_t) (buf[LSM303DLHC_ACC_XLO] | (buf[LSM303DLHC_ACC_XHI] << 8)) >> 4;
    data->y = (int16_t) (buf[LSM303DLHC_ACC_YLO] | (buf[LSM303DLHC_ACC_YHI] << 8)) >> 4;
    data->z = (int16_t) (buf[LSM303DLHC_ACC_ZLO] | (buf[LSM303DLHC_ACC_ZHI] << 8)) >> 4;

    return LSM303DLHC_OK;
}

void LSM303_ConvertAcc(LSM303_Data_t *conv, const LSM303_RawData_t *raw) {
    conv->x = (float) raw->x * lsm303dlhc_acc_mg_lsb * LSM303DLHC_ACC_SENSORS_GRAVITY_STANDARD;
    conv->y = (float) raw->y * lsm303dlhc_acc_mg_lsb * LSM303DLHC_ACC_SENSORS_GRAVITY_STANDARD;
    conv->z = (float) raw->z * lsm303dlhc_acc_mg_lsb * LSM303DLHC_ACC_SENSORS_GRAVITY_STANDARD;
}

LSM303_Result_t LSM303_InitMag(I2C_HandleTypeDef *i2c, const LSM303_MagInit_t *init) {
    uint8_t cra_reg_m;
    lsm303dlhc_i2c = i2c;
    lsm303dlhc_mag_auto_range = init->auto_range;

    if (lsm303dlhc_i2c == NULL || init == NULL) {
        return LSM303DLHC_ERROR;
    }

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_MAG, LSM303DLHC_REG_MAG_MR_REG_M, (uint8_t) init->op) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (LSM303_SetMagRate(init->rate) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (LSM303_SetMagGain(init->gain) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    /* LSM303DLHC has no WHOAMI register so read CRA_REG_M to check the set value */
    if (lsm303dlhc_read_i2c(LSM303DLHC_ADDR_MAG, LSM303DLHC_REG_MAG_CRA_REG_M, &cra_reg_m) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    if (((cra_reg_m >> 2) & 0x07) != (uint8_t) init->rate) {
        return LSM303DLHC_ERROR;
    }

    return LSM303DLHC_OK;
}

LSM303_Result_t LSM303_SetMagRate(LSM303_MagRate_t rate) {
    uint8_t reg_m = ((uint8_t) rate & 0x07) << 2;

    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_MAG, LSM303DLHC_REG_MAG_CRA_REG_M, reg_m) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    } else {
        return LSM303DLHC_OK;
    }
}

LSM303_Result_t LSM303_SetMagGain(LSM303_MagGain_t gain) {
    if (lsm303dlhc_write_i2c(LSM303DLHC_ADDR_MAG, LSM303DLHC_REG_MAG_CRB_REG_M, (uint8_t) gain) != LSM303DLHC_OK) {
        return LSM303DLHC_ERROR;
    }

    lsm303dlhc_mag_magin = gain;

    switch (lsm303dlhc_mag_magin) {
    case LSM303DLHC_MAGGAIN_1_3:
        lsm303dlhc_mag_gauss_lsb_xy = 1100;
        lsm303dlhc_mag_gauss_lsb_z = 980;
        break;
    case LSM303DLHC_MAGGAIN_1_9:
        lsm303dlhc_mag_gauss_lsb_xy = 855;
        lsm303dlhc_mag_gauss_lsb_z = 760;
        break;
    case LSM303DLHC_MAGGAIN_2_5:
        lsm303dlhc_mag_gauss_lsb_xy = 670;
        lsm303dlhc_mag_gauss_lsb_z = 600;
        break;
    case LSM303DLHC_MAGGAIN_4_0:
        lsm303dlhc_mag_gauss_lsb_xy = 450;
        lsm303dlhc_mag_gauss_lsb_z = 400;
        break;
    case LSM303DLHC_MAGGAIN_4_7:
        lsm303dlhc_mag_gauss_lsb_xy = 400;
        lsm303dlhc_mag_gauss_lsb_z = 355;
        break;
    case LSM303DLHC_MAGGAIN_5_6:
        lsm303dlhc_mag_gauss_lsb_xy = 330;
        lsm303dlhc_mag_gauss_lsb_z = 295;
        break;
    case LSM303DLHC_MAGGAIN_8_1:
        lsm303dlhc_mag_gauss_lsb_xy = 230;
        lsm303dlhc_mag_gauss_lsb_z = 205;
        break;
    }

    return LSM303DLHC_OK;
}

LSM303_Result_t LSM303_ReadMagRaw(LSM303_RawData_t *data) {
    bool reading_valid = false;
    uint8_t reg_mg = 0;
    uint8_t reg_mag_out = LSM303DLHC_REG_MAG_OUT_X_H_M;
    uint8_t buf[6] = { 0 };

    while (reading_valid == false) {
        if (lsm303dlhc_read_i2c(LSM303DLHC_ADDR_MAG, LSM303DLHC_REG_MAG_SR_REG_Mg, &reg_mg) != LSM303DLHC_OK) {
            return LSM303DLHC_ERROR;
        }

        if (!(reg_mg & 0x1)) {
            return LSM303DLHC_ERROR;
        }

        if (HAL_I2C_Master_Transmit(lsm303dlhc_i2c, LSM303DLHC_ADDR_MAG, &reg_mag_out, 1, 1000) != HAL_OK) {
            return LSM303DLHC_ERROR;
        }

        if (HAL_I2C_Master_Receive(lsm303dlhc_i2c, LSM303DLHC_ADDR_MAG, buf, 6, 1000) != HAL_OK) {
            return LSM303DLHC_ERROR;
        }

        /* raw data (low byte first) */
        data->x = (int16_t) (buf[LSM303DLHC_MAG_XLO] | (buf[LSM303DLHC_MAG_XHI] << 8));
        data->y = (int16_t) (buf[LSM303DLHC_MAG_YLO] | (buf[LSM303DLHC_MAG_YHI] << 8));
        data->z = (int16_t) (buf[LSM303DLHC_MAG_ZLO] | (buf[LSM303DLHC_MAG_ZHI] << 8));

        /* make sure the sensor isn't saturating if auto-ranging is enabled */
        if (lsm303dlhc_mag_auto_range == false) {
            reading_valid = true;
        } else {
            /* check if the sensor is saturating or not */
            if ((data->x >= 2040) | (data->x <= -2040) | (data->y >= 2040) | (data->y <= -2040) | (data->z >= 2040) | (data->z <= -2040)) {
                /* saturating .... increase the range if we can */
                switch (lsm303dlhc_mag_magin) {
                case LSM303DLHC_MAGGAIN_5_6:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_8_1) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                case LSM303DLHC_MAGGAIN_4_7:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_5_6) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                case LSM303DLHC_MAGGAIN_4_0:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_4_7) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                case LSM303DLHC_MAGGAIN_2_5:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_4_0) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                case LSM303DLHC_MAGGAIN_1_9:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_2_5) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                case LSM303DLHC_MAGGAIN_1_3:
                    if (LSM303_SetMagGain(LSM303DLHC_MAGGAIN_1_9) != LSM303DLHC_OK) {
                        return LSM303DLHC_ERROR;
                    } else {
                        reading_valid = false;
                    }
                    break;

                    /* cannot change the range */
                default:
                    reading_valid = true;
                    break;
                }
            } else {
                /* all values are withing range */
                reading_valid = true;
            }
        }
    }

    return LSM303DLHC_OK;
}

void LSM303_ConvertMag(LSM303_Data_t *conv, const LSM303_RawData_t *raw) {
    conv->x = ((float) raw->x / lsm303dlhc_mag_gauss_lsb_xy) * LSM303DLHC_MAG_SENSORS_GAUSS_TO_MICROTESLA;
    conv->y = ((float) raw->y / lsm303dlhc_mag_gauss_lsb_xy) * LSM303DLHC_MAG_SENSORS_GAUSS_TO_MICROTESLA;
    conv->z = ((float) raw->z / lsm303dlhc_mag_gauss_lsb_z) * LSM303DLHC_MAG_SENSORS_GAUSS_TO_MICROTESLA;
}

void LSM303_CalibrateAccelerometer(uint16_t samples){
    if (samples == 0) return;
    LSM303_RawData_t acc_current;
    int32_t acc_x_sum = 0;
    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;
    for(uint16_t i = 0; i < samples; i++){
        LSM303_ReadAccRaw(&acc_current);
        acc_x_sum += acc_current.x;
        acc_y_sum += acc_current.y;
        acc_z_sum += acc_current.z;
        HAL_Delay(10);
    }
    acc_cal.x_bias = (int16_t)(acc_x_sum / samples);
    acc_cal.y_bias = (int16_t)(acc_y_sum / samples);
    acc_cal.z_bias = (int16_t)(acc_z_sum / samples);
}

void LSM303_2DMagCalibration(uint8_t seconds){
    LSM303_RawData_t mag_min = { .x = INT16_MAX, .y = INT16_MAX };
    LSM303_RawData_t mag_max = { .x = INT16_MIN, .y = INT16_MIN };
    LSM303_RawData_t mag_current;

    uint32_t start_time = HAL_GetTick();
    while(HAL_GetTick() - start_time < seconds * 1000){
        if(LSM303_ReadMagRaw(&mag_current) == LSM303DLHC_OK){

            mag_max.x = mag_max.x < mag_current.x ? mag_current.x: mag_max.x;
            mag_max.y = mag_max.y < mag_current.y ? mag_current.y: mag_max.y;

            mag_min.x = mag_min.x > mag_current.x ? mag_current.x: mag_min.x;
            mag_min.y = mag_min.y > mag_current.y ? mag_current.y: mag_min.y;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }
        HAL_Delay(50);
    }
    // Hard iron correction (offset)
    mag_cal.x_offset = (mag_max.x + mag_min.x) / 2;
    mag_cal.y_offset = ( mag_max.y + mag_min.y) / 2;
    mag_cal.z_offset = 0;
    
    // Soft iron correction (scale)
    float x_range = (mag_max.x - mag_min.x) / 2.0f;
    float y_range = ( mag_max.y - mag_min.y) / 2.0f;
    float avg_range = (x_range + y_range) / 2.0f;
    
    mag_cal.x_scale = avg_range / x_range;
    mag_cal.y_scale = avg_range / y_range;
    mag_cal.z_scale = 1.0f;
}

void LSM303_MagCalibrationReset(){
    mag_cal.x_max = INT16_MIN;
    mag_cal.y_max = INT16_MIN;
    mag_cal.z_max = INT16_MIN;
    mag_cal.x_min = INT16_MAX;
    mag_cal.y_min = INT16_MAX;
    mag_cal.z_min = INT16_MAX;
    mag_cal.x_offset = 0;
    mag_cal.y_offset = 0;
    mag_cal.z_offset = 0;
    mag_cal.x_scale = 1.0f;
    mag_cal.y_scale = 1.0f;
    mag_cal.z_scale = 1.0f;
}

void LSM303_MagCalibrationUpdateRange(LSM303_RawData_t *raw){
    if(raw->x > mag_cal.x_max) mag_cal.x_max = raw->x;
    if(raw->y > mag_cal.y_max) mag_cal.y_max = raw->y;
    if(raw->z > mag_cal.z_max) mag_cal.z_max = raw->z;

    if(raw->x < mag_cal.x_min) mag_cal.x_min = raw->x;
    if(raw->y < mag_cal.y_min) mag_cal.y_min = raw->y;
    if(raw->z < mag_cal.z_min) mag_cal.z_min = raw->z;
}

void LSM303_MagCalibrationCompute(){
    // Hard iron correction (offset)
    mag_cal.x_offset = (mag_cal.x_max + mag_cal.x_min) / 2;
    mag_cal.y_offset = ( mag_cal.y_max + mag_cal.y_min) / 2;
    mag_cal.z_offset = (mag_cal.z_max + mag_cal.z_min) / 2;
    
    // Soft iron correction (scale)
    float x_range = (mag_cal.x_max - mag_cal.x_min) / 2.0f;
    float y_range = ( mag_cal.y_max - mag_cal.y_min) / 2.0f;
    float z_range = (mag_cal.z_max - mag_cal.z_min) / 2.0f;
    float avg_range = (x_range + y_range + z_range) / 3.0f;
    
    mag_cal.x_scale = avg_range / x_range;
    mag_cal.y_scale = avg_range / y_range;
    mag_cal.z_scale = avg_range / z_range;
}

void LSM303_ApplyMagCalibration(LSM303_RawData_t *raw, LSM303_RawData_t *calibrated){
    calibrated->x = (int16_t)((raw->x - mag_cal.x_offset) * mag_cal.x_scale);
    calibrated->y = (int16_t)((raw->y - mag_cal.y_offset) * mag_cal.y_scale);
    calibrated->z = (int16_t)((raw->z - mag_cal.z_offset) * mag_cal.z_scale);
}

void LSM303_ApplyAccCalibration(LSM303_RawData_t *raw, LSM303_RawData_t *calibrated){
    calibrated->x = raw->x - acc_cal.x_bias;
    calibrated->y = raw->y - acc_cal.y_bias;
    calibrated->z = raw->z - acc_cal.z_bias;
}

float LSM303_ApplyTiltCompensation(LSM303_RawData_t *magData_raw, LSM303_RawData_t *magData_comp,LSM303_RawData_t *accData){
    
    float roll = atan2f((float)accData->y, (float)accData->z);
    float pitch = atan2f(-(float)accData->x,sqrtf((float)accData->y * (float)accData->y + (float)accData->z * (float)accData->z));

    float magXh = (float)magData_raw->x * cosf(pitch) + (float)magData_raw->y * sinf(roll) * sinf(pitch) + (float)magData_raw->z * cosf(roll) * sinf(pitch);
    float magYh = (float)magData_raw->y * cosf(roll) - (float)magData_raw->z * sinf(roll);
    magData_comp->x = (int16_t)magXh;
    magData_comp->y = (int16_t)magYh;
    magData_comp->z = magData_raw->z;
}

float LSM303_GetHeadingDegrees(LSM303_RawData_t *magData){
    float heading = 0.0f;

    heading = (atan2f(magData->y, magData->x) * (180.0f / M_PI)) + DECLINATION_ANGLE;

    if (heading < 0.0f) {
        heading += 360.0f;
    }
    if (heading >= 360.0f) {
        heading -= 360.0f;
    }

    return heading;

}

void LSM303_GetCalibrationData(LSM303_AccCalibration_t *accCalibration, LSM303_MagCalibration_t *magCalibration){
    if(accCalibration != NULL){
        *accCalibration = acc_cal;
    }
    if(magCalibration != NULL){
        *magCalibration = mag_cal;
    }
}

/* private functions */
static LSM303_Result_t lsm303dlhc_read_i2c(uint8_t address, uint8_t reg, uint8_t *data) {
    if (HAL_I2C_Master_Transmit(lsm303dlhc_i2c, address, &reg, 1, 1000) != HAL_OK) {
        return LSM303DLHC_ERROR;
    }

    if (HAL_I2C_Master_Receive(lsm303dlhc_i2c, address, data, 1, 1000) != HAL_OK) {
        return LSM303DLHC_ERROR;
    }

    return LSM303DLHC_OK;
}

static LSM303_Result_t lsm303dlhc_write_i2c(uint8_t address, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { reg, data };

    if (HAL_I2C_Master_Transmit(lsm303dlhc_i2c, address, buf, 2, 1000) != HAL_OK) {
        return LSM303DLHC_ERROR;
    } else {
        return LSM303DLHC_OK;
    }
}
