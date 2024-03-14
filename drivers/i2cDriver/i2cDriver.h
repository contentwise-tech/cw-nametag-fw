#ifndef I2CDRIVER_I2CDRIVER_H_
#define I2CDRIVER_I2CDRIVER_H_

#include "drivers/system/system.h"

    void initI2c(void);
    int8_t i2cReadRegister(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t rxDataSize);
    int8_t i2cWriteRegister(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff, uint8_t txDataSize);
#endif /* I2CDRIVER_I2CDRIVER_H_ */
