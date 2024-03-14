#include "system.h"
#include "sl_sleeptimer.h"

uint64_t  getDelayMs(uint64_t startTime){
    uint64_t tickDelay;
    uint64_t delay;
    uint64_t ticks = sl_sleeptimer_get_tick_count64();

    if (ticks >= startTime){
        tickDelay = ticks - startTime;
    } else {
        tickDelay = (UINT64_MAX - startTime) + ticks;
    }

    //sl_status_t sl_sleeptimer_tick64_to_ms(uint64_t tick, uint64_t *ms);
    sl_sleeptimer_tick64_to_ms(tickDelay, &delay);
    return delay;
}

void delayMs(uint32_t msDelay){

    uint64_t ticks = sl_sleeptimer_get_tick_count64();

    while (getDelayMs(ticks) < msDelay){

    }
}


