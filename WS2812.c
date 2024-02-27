#include <stdio.h>
#include "include/WS2812.h"
#include "driver/i2s.h"
#include "include/ws2812.h"

#define TAG "ws2812"

static uint8_t out_buffer[CANT_LEDS * 12] = {0};
static uint8_t off_buffer[ZERO_BUFFER] = {0};
static uint16_t size_buffer;

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};


void updatePixel(pixel_color_t *pixels) {
  size_t bytes_written = 0;

  for (uint16_t i = 0; i < CANT_LEDS; i++) {
    int loc = i * 12;

    out_buffer[loc] = bitpatterns[pixels[i].green >> 6 & 0x03];
    out_buffer[loc + 1] = bitpatterns[pixels[i].green >> 4 & 0x03];
    out_buffer[loc + 2] = bitpatterns[pixels[i].green >> 2 & 0x03];
    out_buffer[loc + 3] = bitpatterns[pixels[i].green & 0x03];

    out_buffer[loc + 4] = bitpatterns[pixels[i].red >> 6 & 0x03];
    out_buffer[loc + 5] = bitpatterns[pixels[i].red >> 4 & 0x03];
    out_buffer[loc + 6] = bitpatterns[pixels[i].red >> 2 & 0x03];
    out_buffer[loc + 7] = bitpatterns[pixels[i].red & 0x03];

    out_buffer[loc + 8] = bitpatterns[pixels[i].blue >> 6 & 0x03];
    out_buffer[loc + 9] = bitpatterns[pixels[i].blue >> 4 & 0x03];
    out_buffer[loc + 10] = bitpatterns[pixels[i].blue >> 2 & 0x03];
    out_buffer[loc + 11] = bitpatterns[pixels[i].blue & 0x03];
  }

  i2s_write(PERIPH_I2S_NUM, out_buffer, size_buffer, &bytes_written, portMAX_DELAY);
  i2s_write(PERIPH_I2S_NUM, off_buffer, ZERO_BUFFER, &bytes_written, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(10));
  i2s_zero_dma_buffer(PERIPH_I2S_NUM);
}

void sw2812Init(uint8_t _gpioPin) {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = 16,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .use_apll = false,
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = -1,
        .ws_io_num = -1,
        .data_out_num = _gpioPin,
        .data_in_num = -1
    };

    size_buffer = CANT_LEDS * 12;
    i2s_config.dma_buf_len = size_buffer;
    i2s_driver_install(PERIPH_I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(PERIPH_I2S_NUM, &pin_config);
}