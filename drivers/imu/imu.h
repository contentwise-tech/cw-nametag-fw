#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "drivers/system/system.h"

    int8_t initIMU(void);
    float inuReadTemperature(void);
    int8_t readFreeFall(void);
    int8_t readSingleTap(void);
    int8_t readDoubleTap(void);
    uint8_t readPedometer(void);
#endif /* IMU_IMU_H_ */
