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
    STATUS_LED_FAILSAFE,
} StatusLed;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} pixel_color_t;

void sw2812Init(uint8_t gpioPin);
void updateStatus(uint8_t status);
void updatePixel(pixel_color_t *pixels); // TODO: borrar

#endif
