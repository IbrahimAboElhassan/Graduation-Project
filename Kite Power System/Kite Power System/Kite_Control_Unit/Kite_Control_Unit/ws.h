#ifndef WIND_SENSOR_H_
#define WIND_SENSOR_H_

#include "CPU_CONFIGURATION.h"
#include "ADC.h"

// Constants for voltage-to-wind-speed conversion  
#define WIND_SENSOR_OFFSET 0.01f    // Voltage offset               // 0.4f 
#define WIND_SENSOR_SLOPE  0.04f   // Voltage-to-wind-speed slope  // 0.04f

void WindSensor_Init(void);
float WindSensor_GetSpeed(void);

#endif /* WIND_SENSOR_H_ */