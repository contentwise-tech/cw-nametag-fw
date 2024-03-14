
#include "i2cDriver.h"
#include "em_i2c.h"
#include <stdio.h>

gpioStr SDA_PIN = {.port = gpioPortB, .pin = 2};
gpioStr SCL_PIN = {.port = gpioPortB, .pin = 3};

void initI2c(void){
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

    CMU_ClockEnable(cmuClock_I2C0, true);
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(SDA_PIN.port, SDA_PIN.pin, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(SCL_PIN.port, SCL_PIN.pin, gpioModeWiredAndPullUpFilter, 1);

    GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                                        | (SDA_PIN.port << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                                        | (SDA_PIN.pin << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
    GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                                        | (SCL_PIN.port << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                                        | (SCL_PIN.pin << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
    GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

    I2C_Init(I2C0, &i2cInit);

    I2C0->CTRL = I2C_CTRL_AUTOSN;
}

int8_t i2cReadRegister(uint16_t followerAddress, uint8_t targetAddress, uint8_t *rxBuff, uint8_t rxDataSize){
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;

    // Initialize I2C transfer
    i2cTransfer.addr          = followerAddress;
    i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write target address before reading
    i2cTransfer.buf[0].data   = &targetAddress;
    i2cTransfer.buf[0].len    = 1;
    i2cTransfer.buf[1].data   = rxBuff;
    i2cTransfer.buf[1].len    = rxDataSize;

    result = I2C_TransferInit(I2C0, &i2cTransfer);

    // Send data
    while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }

    if (result != i2cTransferDone) {
        return -1;
    }

    return 0;
}

#define I2C_TXBUFFER_SIZE 128
static uint8_t txBufferFifo[I2C_TXBUFFER_SIZE + 1] = {0};

int8_t i2cWriteRegister(uint16_t followerAddress, uint8_t targetAddress, uint8_t *txBuff, uint8_t txDataSize){
    // Transfer structure
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;

    txBufferFifo[0] = targetAddress;
    for(int i = 0; i < txDataSize; i++)
    {
        txBufferFifo[i + 1] = txBuff[i];
    }

    // Initialize I2C transfer
    i2cTransfer.addr          = followerAddress;
    i2cTransfer.flags         = I2C_FLAG_WRITE;
    i2cTransfer.buf[0].data   = txBufferFifo;
    i2cTransfer.buf[0].len    = txDataSize + 1;
    i2cTransfer.buf[1].data   = NULL;
    i2cTransfer.buf[1].len    = 0;

    result = I2C_TransferInit(I2C0, &i2cTransfer);

    // Send data
    while (result == i2cTransferInProgress) {
      result = I2C_Transfer(I2C0);
    }

    if (result != i2cTransferDone) {
      return -1;
    }
    return 0;
}
