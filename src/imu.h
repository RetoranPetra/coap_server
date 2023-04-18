#ifndef __IMU_H__
#define __IMU_H__
#include <zephyr/kernel.h>
void ICM20600_startup(void);
int32_t getRawAccelerationX(void);
int32_t getRawAccelerationY(void);
int32_t getRawAccelerationZ(void);
int16_t getRawGyroscopeX(void);
int16_t getRawGyroscopeY(void);
int16_t getRawGyroscopeZ(void);
int16_t getTemperature(void);
void imuTestLoop(void);
#endif
