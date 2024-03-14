#include "display.h"

#include "em_gpio.h"
#include "em_usart.h"

#include "drivers/imu/imu.h"
#include "stdio.h"

#define DISP_LINES      104
#define DISP_COLUMN     212
#define DISP_FRAME_LEN  2756

static uint8_t dataBuffer[DISP_FRAME_LEN] = {0};

static uint8_t qrCode66x66[] = {  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0xf,0xff,
                                  0xcc,0x33,0x3c,0xcc,0xff,0xfc,0x0,0xf,0xff,0xcc,0x33,0x3c,0xcc,0xff,0xfc,0x0,0xc,0x0,0xc3,0xf0,0xcc,0xcc,0xc0,0xc,0x0,0xc,0x0,0xc3,0xf0,0xcc,0xcc,0xc0,
                                  0xc,0x0,0xc,0xfc,0xcc,0xc,0x3,0xcc,0xcf,0xcc,0x0,0xc,0xfc,0xcc,0xc,0x3,0xcc,0xcf,0xcc,0x0,0xc,0xfc,0xc3,0xc,0xc0,0x30,0xcf,0xcc,0x0,0xc,0xfc,0xc3,0xc,
                                  0xc0,0x30,0xcf,0xcc,0x0,0xc,0xfc,0xc3,0xfc,0xfc,0xfc,0xcf,0xcc,0x0,0xc,0xfc,0xc3,0xfc,0xfc,0xfc,0xcf,0xcc,0x0,0xc,0x0,0xc3,0xcf,0xf0,0x3c,0xc0,0xc,0x0,
                                  0xc,0x0,0xc3,0xcf,0xf0,0x3c,0xc0,0xc,0x0,0xf,0xff,0xcc,0xcc,0xcc,0xcc,0xff,0xfc,0x0,0xf,0xff,0xcc,0xcc,0xcc,0xcc,0xff,0xfc,0x0,0x0,0x0,0xc,0xcc,0x30,0xc0,
                                  0x0,0x0,0x0,0x0,0x0,0xc,0xcc,0x30,0xc0,0x0,0x0,0x0,0xc,0xc3,0xc3,0x3,0xf3,0xc3,0xc0,0xcc,0x0,0xc,0xc3,0xc3,0x3,0xf3,0xc3,0xc0,0xcc,0x0,0x0,0xfc,0x30,0xcf,
                                  0xc,0xf0,0xf,0x30,0x0,0x0,0xfc,0x30,0xcf,0xc,0xf0,0xf,0x30,0x0,0xc,0xcc,0xff,0xf,0x30,0xf,0x0,0xcc,0x0,0xc,0xcc,0xff,0xf,0x30,0xf,0x0,0xcc,0x0,0x3,0xc3,
                                  0xf,0xf3,0xc,0xc0,0xf3,0xf0,0x0,0x3,0xc3,0xf,0xf3,0xc,0xc0,0xf3,0xf0,0x0,0xc,0xff,0xcf,0xc3,0x33,0xcc,0xff,0xc,0x0,0xc,0xff,0xcf,0xc3,0x33,0xcc,0xff,0xc,
                                  0x0,0x3,0xc,0xf,0x3c,0xcf,0x3c,0xcc,0x3c,0x0,0x3,0xc,0xf,0x3c,0xcf,0x3c,0xcc,0x3c,0x0,0x3,0xfc,0xff,0x0,0xff,0x0,0xff,0xcc,0x0,0x3,0xfc,0xff,0x0,0xff,0x0,
                                  0xff,0xcc,0x0,0xc,0xcf,0x30,0xcc,0x30,0xff,0xcf,0xcc,0x0,0xc,0xcf,0x30,0xcc,0x30,0xff,0xcf,0xcc,0x0,0xf,0xc0,0xf0,0xf,0xfc,0x3f,0xf,0xf0,0x0,0xf,0xc0,0xf0,
                                  0xf,0xfc,0x3f,0xf,0xf0,0x0,0xf,0xf0,0x3,0x3,0x0,0xc3,0xc0,0xc0,0x0,0xf,0xf0,0x3,0x3,0x0,0xc3,0xc0,0xc0,0x0,0xc,0x3c,0xcc,0x30,0x0,0xf0,0x30,0x30,0x0,0xc,
                                  0x3c,0xcc,0x30,0x0,0xf0,0x30,0x30,0x0,0x0,0xc0,0xf,0xf3,0xcf,0xff,0x3f,0xc0,0x0,0x0,0xc0,0xf,0xf3,0xcf,0xff,0x3f,0xc0,0x0,0x3,0x0,0xf3,0xfc,0x3f,0xf,0xfc,
                                  0x3c,0x0,0x3,0x0,0xf3,0xfc,0x3f,0xf,0xfc,0x3c,0x0,0x0,0x0,0x0,0x30,0x3,0xc,0xf,0x30,0x0,0x0,0x0,0x0,0x30,0x3,0xc,0xf,0x30,0x0,0xf,0xff,0xc3,0xcf,0x3f,0xcc,
                                  0xcc,0x0,0x0,0xf,0xff,0xc3,0xcf,0x3f,0xcc,0xcc,0x0,0x0,0xc,0x0,0xcf,0xf3,0xff,0x3c,0xc,0x3c,0x0,0xc,0x0,0xcf,0xf3,0xff,0x3c,0xc,0x3c,0x0,0xc,0xfc,0xc0,0xc0,
                                  0x30,0x3f,0xff,0xfc,0x0,0xc,0xfc,0xc0,0xc0,0x30,0x3f,0xff,0xfc,0x0,0xc,0xfc,0xc0,0xf0,0x0,0xcf,0xf,0x3c,0x0,0xc,0xfc,0xc0,0xf0,0x0,0xcf,0xf,0x3c,0x0,0xc,
                                  0xfc,0xcc,0xc0,0x3,0x3,0x3,0x0,0x0,0xc,0xfc,0xcc,0xc0,0x3,0x3,0x3,0x0,0x0,0xc,0x0,0xc3,0x3,0x3,0xc,0xc,0xc0,0x0,0xc,0x0,0xc3,0x3,0x3,0xc,0xc,0xc0,0x0,0xf,
                                  0xff,0xcf,0xcf,0xcf,0xc3,0xcf,0xcc,0x0,0xf,0xff,0xcf,0xcf,0xcf,0xc3,0xcf,0xcc,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
                                  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

typedef struct QrCodeStr{
    uint32_t xSize;
    uint32_t ySize;
    uint32_t size;
    const uint8_t *data;
}QrCodeStr;

static QrCodeStr qrCode = {.xSize = 66, .ySize = 66, .size = sizeof(qrCode66x66), .data = qrCode66x66};

gpioStr DISP_BS = {.port = gpioPortA, .pin = 8};
gpioStr DISP_DC = {.port = gpioPortA, .pin = 7};
gpioStr DISP_CS = {.port = gpioPortA, .pin = 6};
gpioStr DISP_CLK = {.port = gpioPortA, .pin = 5};
gpioStr DISP_MOSI = {.port = gpioPortA, .pin = 4};
gpioStr DISP_nBUSY = {.port = gpioPortB, .pin = 0};
gpioStr DISP_nRST = {.port = gpioPortA, .pin = 0};

static void dispSendCommand(uint8_t command, uint8_t *data, uint32_t dataLen){
    GPIO_PinOutClear(DISP_DC.port, DISP_DC.pin);
    GPIO_PinOutClear(DISP_CS.port, DISP_CS.pin);
    USART_SpiTransfer(USART0, command);

    GPIO_PinOutSet(DISP_CS.port, DISP_CS.pin);
    GPIO_PinOutSet(DISP_DC.port, DISP_DC.pin);

    for(uint32_t n = 0; n < dataLen; n++){
        GPIO_PinOutClear(DISP_CS.port, DISP_CS.pin);
        USART_SpiTransfer(USART0, data[n]);
        GPIO_PinOutSet(DISP_CS.port, DISP_CS.pin);
    }
}

void clearDisplayRam(void){
    for(uint32_t n = 0; n < DISP_FRAME_LEN; n++){
        dataBuffer[n] = 0;
    }
}

void initDispayDriver(uint8_t temperature){
    uint8_t commandData;
    GPIO_PinOutSet(DISP_nRST.port, DISP_nRST.pin);
    delayMs(5);
    GPIO_PinOutClear(DISP_nRST.port, DISP_nRST.pin);
    delayMs(5);
    GPIO_PinOutSet(DISP_nRST.port, DISP_nRST.pin);
    delayMs(5);

    commandData = 0x0E;
    dispSendCommand(0x00, &commandData, 1);     // sw restart
    delayMs(5);
    commandData = temperature;
    dispSendCommand(0xe5, &commandData, 1);     // set temperature
    commandData = 0x02;
    dispSendCommand(0xe0, &commandData, 1);     // Active Temperature
}

static void sendImageBuffer(void){
    dispSendCommand(0x10, dataBuffer, DISP_FRAME_LEN);

    GPIO_PinOutClear(DISP_DC.port, DISP_DC.pin);
    GPIO_PinOutClear(DISP_CS.port, DISP_CS.pin);
    USART_SpiTransfer(USART0, 0x13);

    GPIO_PinOutSet(DISP_CS.port, DISP_CS.pin);
    GPIO_PinOutSet(DISP_DC.port, DISP_DC.pin);

    for(uint32_t n = 0; n < DISP_FRAME_LEN; n++){
        GPIO_PinOutClear(DISP_CS.port, DISP_CS.pin);
        USART_SpiTransfer(USART0, 0);
        GPIO_PinOutSet(DISP_CS.port, DISP_CS.pin);
    }
}

static void updateImage(void){
    uint8_t ansBuffer;
    while(GPIO_PinInGet(DISP_nBUSY.port, DISP_nBUSY.pin) == 0){
    }

    dispSendCommand(0x04, &ansBuffer, 0);
    dispSendCommand(0x04, &ansBuffer, 0);

    while(GPIO_PinInGet(DISP_nBUSY.port, DISP_nBUSY.pin) == 0){
    }

    dispSendCommand(0x12, &ansBuffer, 0);
    dispSendCommand(0x12, &ansBuffer, 0);

    while(GPIO_PinInGet(DISP_nBUSY.port, DISP_nBUSY.pin) == 0){
    }
}

static void turnOffDcDc(void){
    uint8_t ansBuffer;
    dispSendCommand(0x02, &ansBuffer, 0);
    dispSendCommand(0x02, &ansBuffer, 0);

    while(GPIO_PinInGet(DISP_nBUSY.port, DISP_nBUSY.pin) == 0){
    }
}

static void putChar(uint32_t x, uint32_t y, uint8_t charLen, uint8_t lineLen, uint8_t *charData){
    if ((x < DISP_COLUMN) && (y < DISP_LINES)){

        uint32_t ansColumn = x;
        uint32_t ansLine = y;

        for (uint8_t n = 0; n < 8; n++){
            for (uint8_t j = 0; j < charLen; j++){
                for (uint8_t i = 0; i < lineLen; i++){

                    uint32_t xPixel = ansColumn + n;
                    uint8_t yPixel = ansLine + (charLen - 1 - j);
                    uint8_t pixel = (charData[(j * lineLen) + i] & (1 << n));
                    uint32_t bufferByte = 0;
                    uint8_t pixelByte = yPixel % 8;
                    bufferByte = ((xPixel * DISP_LINES) / 8) + (yPixel / 8);

                    bufferByte += i * DISP_LINES;
                    if (pixel != 0){
                        dataBuffer[bufferByte] = dataBuffer[bufferByte] | (0x80 >> pixelByte);
                    } else {
                        dataBuffer[bufferByte] = dataBuffer[bufferByte] & (~(0x80 >> pixelByte));
                    }
                }
            }
        }
    }
}

void sendQrCode(void){
    uint32_t columnSize = 0;
    uint32_t x = 212 - qrCode.xSize;
    uint32_t screenLine;

    columnSize = qrCode.xSize / 8;
    if (qrCode.xSize % 8)
        columnSize++;

    screenLine = qrCode.ySize / 8;
    if (qrCode.ySize % 8){
        screenLine++;
    }

    for (uint32_t column = 0; column < qrCode.xSize; column++){
        for (uint32_t line = 0; line < screenLine; line++){
            uint32_t dataCursor = (columnSize * column) + line;

            uint8_t data = qrCode.data[dataCursor];
            volatile uint8_t ansData = 0;

            for (uint8_t pixel = 0; pixel < 8; pixel++){
                ansData = ansData << 1;
                if (data & 0x01){
                    ansData = ansData | 0x01;
                }
                data = data >> 1;
            }

            dataBuffer[((column + x) * DISP_LINES/8) + (screenLine - line - 1)] = ansData;
        }
    }
}

void displaySendString(uint32_t x, uint32_t y, uint8_t *str, uint32_t strLen, fontHeaderStr *font){
    for (uint8_t cursor = 0; cursor < strLen; cursor++){
        uint8_t dataChar = str[cursor];
        if ((dataChar < font->firstChar) || (dataChar > font->lastChar)){
            dataChar = ' ';
        }
        uint32_t info = 8 + ((dataChar - font->firstChar) * 4);
        uint32_t dataAddress = font->fontData[info + 1];
        dataAddress = dataAddress | (font->fontData[info + 2] << 8);
        uint8_t *data = &font->fontData[dataAddress];
        uint8_t charLen = font->fontData[info];

        uint8_t lineLen = 1;
        if (charLen > 16){
            lineLen = 3;
        } else if (charLen > 8){
            lineLen = 2;
        }
        putChar(x, y, font->fontSize, lineLen, data);
        x = x + charLen + 2;
    }
}

void flushImageRam(void){
    sendImageBuffer();
    updateImage();
    turnOffDcDc();
}

uint8_t initDisplay(void){
    CMU_ClockEnable(cmuClock_GPIO, true);

    GPIO_PinModeSet(DISP_BS.port, DISP_BS.pin, gpioModePushPull, 0);
    GPIO_PinModeSet(DISP_DC.port, DISP_DC.pin, gpioModePushPull, 0);
    GPIO_PinModeSet(DISP_nRST.port, DISP_nRST.pin, gpioModePushPull, 0);
    GPIO_PinModeSet(DISP_nBUSY.port, DISP_nBUSY.pin, gpioModeInput, 0);

    GPIO_PinModeSet(DISP_CS.port, DISP_CS.pin, gpioModePushPull, 1);
    GPIO_PinModeSet(DISP_CLK.port, DISP_CLK.pin, gpioModePushPull, 0);
    GPIO_PinModeSet(DISP_MOSI.port, DISP_MOSI.pin, gpioModePushPull, 0);

    CMU_ClockEnable(cmuClock_USART0, true);

    // Default asynchronous initializer (main mode, 1 Mbps, 8-bit data)
    USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

    init.msbf = true;   // MSB first transmission for SPI compatibility

    /*
     * Route USART0 CS, TX, and CLK to the specified pins.
     */
    //GPIO->USARTROUTE[0].CSROUTE = (DISP_CS.port << _GPIO_USART_CSROUTE_PORT_SHIFT) | (DISP_CS.pin << _GPIO_USART_CSROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].TXROUTE = (DISP_MOSI.port << _GPIO_USART_TXROUTE_PORT_SHIFT) | (DISP_MOSI.pin << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].CLKROUTE = (DISP_CLK.port << _GPIO_USART_CLKROUTE_PORT_SHIFT) | (DISP_CLK.pin << _GPIO_USART_CLKROUTE_PIN_SHIFT);

    // Enable USART interface pins
    GPIO->USARTROUTE[0].ROUTEEN = /*GPIO_USART_ROUTEEN_CSPEN |    // CS*/
                                  GPIO_USART_ROUTEEN_TXPEN |    // MOSI
                                  GPIO_USART_ROUTEEN_CLKPEN;

    // Configure and enable USART0
    USART_InitSync(USART0, &init);

    return 0;
}