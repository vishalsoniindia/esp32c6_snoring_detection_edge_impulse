/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK 0

/*
 ** NOTE: If you run into TFLite arena allocation issue.
/* Includes ---------------------------------------------------------------- */
#include <Snoring_Detection_by_vishalsoniindia_inferencing.h>
#include <Adafruit_NeoPixel.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"

#define MODE_PIN (1)
#define LED_PIN 8   // Your NeoPixel data pin
#define NUM_LEDS 1  // Number of LEDs in the strip

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

/** Audio buffers, pointers and selectors */
typedef struct {
  int16_t *buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
static bool record_status = true;

/**
 * @brief      Arduino setup function
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, LOW);  //Configure the Microphone to Receive Left Channel Data
                                //digitalWrite(MODE_PIN,HIGH);//Configure the Microphone to Receive Right Channel Data

  strip.begin();
  strip.show();  // Initialize all pixels to OFF

  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  while (!Serial)
    ;
  Serial.println("Edge Impulse Inferencing Demo");

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tInterval: ");
  ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
  ei_printf(" ms.\n");
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
  ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);

  if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
    ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
    return;
  }

  ei_printf("Recording...\n");
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop() {
  bool m = microphone_inference_record();
  if (!m) {
    ei_printf("ERR: Failed to record audio...\n");
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
  if (r != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", r);
    return;
  }

  // print the predictions
  //   ei_printf("Predictions ");
  //   ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
  //             result.timing.dsp, result.timing.classification, result.timing.anomaly);
  //   ei_printf(": \n");
  /*
    snoring: 0.035156
    z_openset: 0.964844
    */
    static unsigned long last_time_led = 0;

  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if ("snoring" == result.classification[ix].label) {
      if (result.classification[ix].value > 0.8) {
        ei_printf("Snoring_detected\n");
        strip.setPixelColor(0, strip.Color(255, 0, 0));
        strip.show();
        last_time_led = millis();
      } else {
        if(millis()-last_time_led > 3000){
        strip.setPixelColor(0, strip.Color(0, 255, 0));
        strip.show();
        }
      }
    }
    // ei_printf("    %s: ", result.classification[ix].label);
    // ei_printf_float(result.classification[ix].value);
    // ei_printf("\n");
  }



#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: ");
  ei_printf_float(result.anomaly);
  ei_printf("\n");
#endif
}

static void audio_inference_callback(uint32_t n_bytes) {
  for (int i = 0; i < n_bytes >> 1; i++) {
    inference.buffer[inference.buf_count++] = sampleBuffer[i];

    if (inference.buf_count >= inference.n_samples) {
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

static void capture_samples(void *arg) {

  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {

    /* read data at once from i2s */
    i2s_read((i2s_port_t)0, (void *)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read <= 0) {
      ei_printf("Error in I2S read : %d", bytes_read);
    } else {
      if (bytes_read < i2s_bytes_to_read) {
        ei_printf("Partial I2S read");
      }

      // scale the data (otherwise the sound is too quiet)
      int16_t left;
      int sampleIndex = 0;

      for (int x = 0; x < i2s_bytes_to_read / 4; x++) {
        left = sampleBuffer[x * 2];  // index 0,2,4,6 = LEFT
        sampleBuffer[sampleIndex++] = left * 8;
      }


      if (record_status) {
        audio_inference_callback(i2s_bytes_to_read / 2);
      } else {
        break;
      }
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

  if (inference.buffer == NULL) {
    return false;
  }

  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
    ei_printf("Failed to start I2S!");
  }

  ei_sleep(100);

  record_status = true;

  xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void *)sample_buffer_size, 10, NULL);

  return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void) {
  bool ret = true;

  while (inference.buf_ready == 0) {
    delay(10);
  }

  inference.buf_ready = 0;
  return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

  return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void) {
  i2s_deinit();
  ei_free(inference.buffer);
}


static int i2s_init(uint32_t sampling_rate) {

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampling_rate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,       // MSM261 outputs stereo
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Required by DFRobot MSM261
    .intr_alloc_flags = 0,
    .dma_buf_count = 16,  // EXACT values used by DFRobot
    .dma_buf_len = 60,
    .use_apll = false,
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = 10,
    .ws_io_num = 4,
    .data_out_num = -1,
    .data_in_num = 0
  };

  esp_err_t ret;

  ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (ret != ESP_OK) Serial.println("I2S install failed!");

  ret = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (ret != ESP_OK) Serial.println("I2S pin config failed!");

  ret = i2s_zero_dma_buffer(I2S_NUM_0);
  return ret;
}


static int i2s_deinit(void) {
  i2s_driver_uninstall((i2s_port_t)0);  //stop & destroy i2s driver
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
