#include <stdio.h>
#include "include/WS2812.h"
#include "driver/i2s.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "WS2812"

const pixel_color_t colors[8] = {
    {.red = 0x00,.green = 0x00,.blue = 0x00},
    {.red = 0xff,.green = 0x00,.blue = 0x00},
    {.red = 0x00,.green = 0xff,.blue = 0x00},
    {.red = 0x00,.green = 0x00,.blue = 0xff},
    {.red = 0xff,.green = 0x00,.blue = 0xff},
    {.red = 0xff,.green = 0xff,.blue = 0x00},
    {.red = 0x00,.green = 0xff,.blue = 0xff},
    {.red = 0xff,.green = 0xff,.blue = 0xff}
};

static uint8_t out_buffer[CANT_LEDS * 12] = {0};
static uint8_t off_buffer[ZERO_BUFFER] = {0};
static uint16_t size_buffer;
static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

TaskHandle_t xControlStatusLed;
status_pixel_control_t statusLedControl;

static void statusLedHandler(void *pvParameters);

void updatePixel(pixel_color_t pixel) {
  size_t bytes_written = 0;
  out_buffer[0] = bitpatterns[pixel.green >> 6 & 0x03];
  out_buffer[1] = bitpatterns[pixel.green >> 4 & 0x03];
  out_buffer[2] = bitpatterns[pixel.green >> 2 & 0x03];
  out_buffer[3] = bitpatterns[pixel.green & 0x03];

  out_buffer[4] = bitpatterns[pixel.red >> 6 & 0x03];
  out_buffer[5] = bitpatterns[pixel.red >> 4 & 0x03];
  out_buffer[6] = bitpatterns[pixel.red >> 2 & 0x03];
  out_buffer[7] = bitpatterns[pixel.red & 0x03];

  out_buffer[8] = bitpatterns[pixel.blue >> 6 & 0x03];
  out_buffer[9] = bitpatterns[pixel.blue >> 4 & 0x03];
  out_buffer[10] = bitpatterns[pixel.blue >> 2 & 0x03];
  out_buffer[11] = bitpatterns[pixel.blue & 0x03];

  i2s_write(PERIPH_I2S_NUM, out_buffer, size_buffer, &bytes_written, portMAX_DELAY);
  i2s_write(PERIPH_I2S_NUM, off_buffer, ZERO_BUFFER, &bytes_written, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(10));
  i2s_zero_dma_buffer(PERIPH_I2S_NUM);
}

void statusLedInit(uint8_t _gpioPin) {
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

    size_buffer = 12;
    i2s_config.dma_frame_num = size_buffer;   
    i2s_driver_install(PERIPH_I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(PERIPH_I2S_NUM, &pin_config);

    statusLedUpdate(STATUS_LED_INITIALIZING);
    xTaskCreate(statusLedHandler,"status led task",4096,NULL,3,&xControlStatusLed);
}

void statusLedUpdate(uint8_t status) {

  switch(status) {
    case STATUS_LED_INITIALIZING:
      statusLedControl.blink = false;
      statusLedControl.colors[0] = colors[COLOR_WHITE];
      statusLedControl.tOnMs = 500;
    break;
    case STATUS_LED_ERROR:
      statusLedControl.blink = false;
      statusLedControl.colors[0] = colors[COLOR_RED];
      statusLedControl.tOnMs = 500;
    break;

    case STATUS_LED_WAITING_ARM:
      statusLedControl.blink = true;
      statusLedControl.colors[0] = colors[COLOR_YELLOW];
      statusLedControl.colors[1] = colors[COLOR_GREEN];
      statusLedControl.tOnMs = 500;
      statusLedControl.tOffMs = 500;
    break;

    case STATUS_LED_ARMED:
      statusLedControl.blink = false;
      statusLedControl.colors[0] = colors[COLOR_GREEN];
      statusLedControl.tOnMs = 500;
      statusLedControl.tOffMs = 500;
    break;

    case STATUS_LED_FAILSAFE:
      statusLedControl.blink = true;
      statusLedControl.colors[0] = colors[COLOR_PURPLE];
      statusLedControl.colors[1] = colors[COLOR_BLACK];
      statusLedControl.tOnMs = 500;
      statusLedControl.tOffMs = 200;
    break;

    case STATUS_LED_CALIBRATION_IMU:
      statusLedControl.blink = true;
      statusLedControl.colors[0] = colors[COLOR_BLUE];
      statusLedControl.colors[1] = colors[COLOR_BLACK];
      statusLedControl.tOnMs = 50;
      statusLedControl.tOffMs = 200;
    break;

    // default:
    //   statusLedControl.blink = false;
    //   statusLedControl.colors[0] = colors[COLOR_BLACK];
    //   statusLedControl.tOnMs = 500;
    // break;
  }
}

static void statusLedHandler(void *pvParameters) {
  while(true){
    updatePixel(colors[COLOR_BLUE]);//updatePixel(statusLedControl.colors[0]);
    vTaskDelay(pdMS_TO_TICKS(100));//pdMS_TO_TICKS(statusLedControl.tOnMs));
    // if (statusLedControl.blink) {
      updatePixel(colors[COLOR_BLACK]);//updatePixel(statusLedControl.colors[1]);
      vTaskDelay(pdMS_TO_TICKS(100));//vTaskDelay(pdMS_TO_TICKS(statusLedControl.tOffMs));
    // }
  }

  vTaskDelete(NULL);
}
