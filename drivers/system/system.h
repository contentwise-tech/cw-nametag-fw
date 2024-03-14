#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_
  #include "em_cmu.h"
  #include "em_chip.h"
  #include "stdint.h"

    void delayMs(uint32_t msDelay);
    uint64_t getDelayMs(uint64_t startTime);

    typedef struct gpioStr{
      GPIO_Port_TypeDef port;
      unsigned int pin;
    }gpioStr;

#endif /* SYSTEM_SYSTEM_H_ */
