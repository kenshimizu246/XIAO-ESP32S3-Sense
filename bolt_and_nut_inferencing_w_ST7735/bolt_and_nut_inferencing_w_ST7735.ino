#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>

//#include <Screw_inferencing.h>
#include <bolt_and_nut_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

#define TFT_CS    1
#define TFT_RST   2
#define TFT_DC    3
#define TFT_MOSI  9
#define TFT_SCK   7

#define BTN_CAPTURE 4

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           96
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           96
#define EI_CAMERA_FRAME_BYTE_SIZE                 1

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const uint16_t imgModelWidth = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
const uint16_t imgModelHeight = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;

const uint16_t imgWidth = 240;
const uint16_t imgHeight = 240;

unsigned long lastCaptureTime = 0; // Last shooting time
int imageCount = 1;                // File Counter
bool camera_sign = false;          // Check camera status
volatile bool init_done = false; // needs volatile in the interruption.

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
uint8_t *snapshot_buf; //points to the output of the capture

WiFiServer server(80);

// capure and display photo
void capture_photo() {
  
  tft.fillScreen(ST77XX_BLACK);
  
  // Take a photo
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }

  ei_printf("fb->width:%d, fb->height:%d\n", fb->width, fb->height);

  ei::image::processing::resize_image(fb->buf, fb->width, fb->height, snapshot_buf, imgModelWidth, imgModelHeight, 1);

  // int res = ei::image::processing::crop_and_interpolate_image(snapshot_buf, imgModelWidth, imgModelHeight, snapshot_buf, imgModelWidth, imgModelHeight, 1);
  // if(EIDSP_OK != res){
  //   Serial.print("crop_and_interpolate_image failed! ");
  //   Serial.println(res);
  //   return;
  // }
  
  ei_printf("imgModelWidth:%d, imgModelHeight:%d\n", imgModelWidth, imgModelHeight);

  display_photo(snapshot_buf, imgModelWidth, imgModelHeight);

  // Release image buffer
  esp_camera_fb_return(fb);
}

void inferencing(){
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", err);
      return;
  }
  
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  uint8_t tline = 0;
  bool bb_found = result.bounding_boxes[0].value > 0;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
      auto bb = result.bounding_boxes[ix];
      if (bb.value == 0) {
          continue;
      }
      ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
      char text[24];
      sprintf(text, "%s %.02f", bb.label, bb.value);
      tft.setCursor(96,tline);
      tft.print(text);
      tft.drawCircle(bb.x, bb.y, 5, ST77XX_YELLOW);
      tline += 10;
  }
  if (!bb_found) {
      ei_printf("    No objects found\n");
  }

  Serial.println("Photo saved to file");
}

// display
void display_photo(uint8_t * data, uint32_t w, uint32_t h){
  uint32_t buff_size = w * h * 2;
  uint16_t * buff = (uint16_t *)malloc(buff_size);

  // b1111100000000000 -> 0xF800 -> 63488
  // b0000011111100000 -> 0x07E0 -> 2016
  // b0000000000011111 -> 0x001F -> 31
  // b1111111111111111 -> 0xFFFF -> 65535
  uint32_t x = 0, y = 0, k, j = 0;
  uint16_t color = 0;
  uint16_t c, r, g, b;
  for(uint32_t y = 0; y < h; y++){
    for(uint32_t x = 0; x < w; x++){
      k = (y * w) + x;
      b = (data[k] >> 3) & 0x1F;
      g = b << (5 + 1); // 5 + 1 because green has 6 bits instead of others like 5 bits.
      r = b << (5 + 6);
      color = r | g | b;  // 5, 6, 5
      buff[k] = color;
    }
  }

  tft.drawRGBBitmap(0, 0, buff, w, h);

  free(buff);
  
  Serial.printf("draw done!\n");
}

void setup() {
  init_done = false;

  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextWrap(true);
  tft.print("Start init...");
    
  Serial.begin(115200);
  // while(!Serial); // When the serial monitor is turned on, the program starts to execute

  // QQVGA (Quarter QVGA) 160 120
  // QCIF (Quarter CIF) 176 144
  // QVGA (Quarter VGA) 320 240

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  //config.frame_size = FRAMESIZE_UXGA;
  //config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.frame_size = FRAMESIZE_240X240;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_LATEST; // CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;  
  config.jpeg_quality = 12;
  config.fb_count = 1;

//    FRAMESIZE_240X240,  // 240x240
//    FRAMESIZE_QVGA,     // 320x240

//    PIXFORMAT_RGB565,    // 2BPP/RGB565
//    PIXFORMAT_YUV422,    // 2BPP/YUV422
//    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  // Best option for face detection/recognition
  config.frame_size = FRAMESIZE_240X240; // FRAMESIZE_240X240; //FRAMESIZE_96X96;
#if CONFIG_IDF_TARGET_ESP32S3
  config.fb_count = 2;
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  camera_sign = true; // Camera initialization check passes
  
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextWrap(true);
  tft.print("Init done!");
  
  init_done = true;
}

void loop() {
  if(init_done && camera_sign){
    delay(1000);
    Serial.print("\nPicture Capture Command is sent...");
    Serial.println(imageCount);
    
    // allocation must be before capture_photo because it copys picture into the snapshot_buf.
    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    capture_photo();
    Serial.println("capture_photo()");

    inferencing();
    Serial.println("inferencing()");

    free(snapshot_buf);
    imageCount++;
  }
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 1;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix++;
        pixels_left--;
    }
    // and done!
    return 0;
}