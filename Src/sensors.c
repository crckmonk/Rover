#include "sensors.h"
#include "adc.h"
#include "i2c.h"

#define SENSOR_QMC588

extern I2C_HandleTypeDef hi2c1;
static SensorData_t sensor_data = {0};
static QMC_Handle_t mag;


static HAL_StatusTypeDef Sensors_InitQMC588(){
    return QMC5883_Init(&mag, &hi2c1, QMC_Rate_50);
}


HAL_StatusTypeDef Sensors_Init(void){
    #ifdef SENSOR_QMC588
    return Sensors_InitQMC588();
    #endif

}

void Sensors_Update(void) {

    #ifdef SENSOR_QMC588
    if (QMC5883_Read(&mag) == HAL_OK) {
        sensor_data.heading_deg = mag.heading;
    } else {
        DEBUG_PRINTF(DBG_ERROR, "[SENSOR] QMC5883 Read Error\r\n");
    }
    #endif
    
    sensor_data.vbat_mv = ADC_ReadBattmv();
    sensor_data.last_update = HAL_GetTick();
    sensor_data.valid = 1;
}


float Sensors_GetHeading(void){
    return sensor_data.heading_deg;
}


uint16_t Sensors_GetBatterymv(void){
    return sensor_data.vbat_mv;
}


const SensorData_t* Sensors_GetData(void) {
    return &sensor_data;
}