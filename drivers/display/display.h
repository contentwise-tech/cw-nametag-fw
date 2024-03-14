#ifndef DISPLAY_DISPLAY_H_
#define DISPLAY_DISPLAY_H_
    #include "drivers/system/system.h"

    typedef struct fontHeaderStr{
        uint16_t firstChar;
        uint16_t lastChar;
        uint8_t fontSize;
        uint8_t *fontData;
    }fontHeaderStr;

    uint8_t initDisplay(void);
    void initDispayDriver(uint8_t temperature);
    void displaySendString(uint32_t x, uint32_t y, uint8_t *str, uint32_t strLen, fontHeaderStr *font);

    void flushImageRam(void);

    void clearDisplayRam(void);
    void sendQrCode(void);
#endif /* DISPLAY_DISPLAY_H_ */
