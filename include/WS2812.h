void func(void);
#ifndef __WS2812_H__
#define __WS2812_H__
#include "stdio.h"

#define CANT_LEDS   1
#define SAMPLE_RATE (93750)
#define ZERO_BUFFER 48

#define PERIPH_I2S_NUM 0

typedef enum {
    STATUS_LED_INITIALIZING,
    STATUS_LED_ERROR,
    STATUS_LED_WAITING_ARM,
    STATUS_LED_ARMED,
    STATUS_LED_FAILSAFE,
    STATUS_LED_CALIBRATION_IMU
} StatusLed;

typedef enum {
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_PURPLE,
    COLOR_YELLOW,
    COLOR_SKY_BLUE,
    COLOR_WHITE
} LedColors;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} pixel_color_t;

typedef struct {
    uint8_t blink;
    uint16_t tOnMs;
    uint16_t tOffMs;
    pixel_color_t colors[2];
} status_pixel_control_t;

void statusLedInit(uint8_t gpioPin);
void statusLedUpdate(uint8_t status);

#endif
