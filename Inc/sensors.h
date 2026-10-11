#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "lsm303.h"
#include "qmc5883.h"

typedef struct {
    float heading_deg;
    int16_t acc_x, acc_y, acc_z;
    int16_t mag_x, mag_y, mag_z;
    uint16_t vbat_mv;
    uint32_t last_update;
    uint8_t valid;
} SensorData_t;

HAL_StatusTypeDef Sensors_Init(void);
void Sensors_Update(void);
float Sensors_GetHeading(void);
uint16_t Sensors_GetBatterymv(void);
// Called by scheduler
const SensorData_t* Sensors_GetData(void);

#endif