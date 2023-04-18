#ifndef __ICM20600_H__
#define __ICM20600_H__

uint16_t ICM20600_startup(void);
int32_t getRawAccelerationX(void);
int32_t getRawAccelerationY(void);
int32_t getRawAccelerationZ(void);
int16_t getRawGyroscopeX(void);
int16_t getRawGyroscopeY(void);
int16_t getRawGyroscopeZ(void);
int16_t getTemperature(void);
#endif
