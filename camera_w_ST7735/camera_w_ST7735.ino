#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <apple_vs_banana_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

#define TFT_CS    1
#define TFT_RST   2
#define TFT_DC    3
#define TFT_MOSI  9
#define TFT_SCK   7

#define BTN_CAPTURE 4

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const uint16_t imageWidth = 160;
const uint16_t imageHeight = 120;

unsigned long lastCaptureTime = 0; // Last shooting time
int imageCount = 1;                // File Counter
bool camera_sign = false;          // Check camera status
uint32_t buff_size = 0;
uint16_t * buff = NULL;
bool action_capture = false;
volatile bool init_done = false; // needs volatile in the interruption.

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// capure and display photo
void capture_photo() {
  
  tft.fillScreen(ST77XX_BLACK);
  
  // Take a photo
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }
  
  display_photo(fb->buf, fb->len, imageWidth, imageHeight);
      
  // Release image buffer
  esp_camera_fb_return(fb);

  char text[8];
  //tft.drawNumber(detector.getPersonScore(), 100, 100);
  sprintf(text, "%d", 100);
  tft.setCursor(0, 121);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextWrap(true);
  tft.print(text);
  delay(1000);

  Serial.println("Photo saved to file");
}

// display
void display_photoy(uint8_t * data, size_t len, uint32_t w, uint32_t h){
  Serial.printf("data len: %d\n", len);

  buff_size = imageWidth * imageHeight * 2;
  buff = (uint16_t *)malloc(buff_size);

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

  tft.startWrite();
  for(uint32_t y = 0; y < h; y++){
    for(uint32_t x = 0; x < w; x++){
      k = (y * w) + x;
      tft.writePixel(x, y, buff[k]);
    }
  }
  tft.endWrite();

  delay(3000);
  free(buff);
    
  // Serial.printf("r: %X\n", r);
  // Serial.printf("g: %X\n", g);
  // Serial.printf("b: %X\n", b);
  // Serial.printf("color: %X\n", color);
  Serial.printf("draw done!\n");
}

// display
void display_photoz(uint8_t * data, size_t len, uint32_t w, uint32_t h){
  Serial.printf("data len: %d\n", len);

  // b1111100000000000 -> 0xF800 -> 63488
  // b0000011111100000 -> 0x07E0 -> 2016
  // b0000000000011111 -> 0x001F -> 31
  // b1111111111111111 -> 0xFFFF -> 65535
  uint32_t k, j = 0;
  uint16_t color = 0;
  uint16_t c, r, g, b;
  tft.startWrite();
  for(uint32_t y = 0; y < h; y++){
    for(uint32_t x = 0; x < w; x++){
      k = (y * w) + x;
      b = (data[k] >> 3) & 0x1F;
      g = b << (5 + 1); // 5 + 1 because green has 6 bits instead of others like 5 bits.
      r = b << (5 + 6);
      color = r | g | b;  // 5, 6, 5
      //buff[k] = color;
      tft.writePixel(x, y, color);
    }
  }
  tft.endWrite();
  delay(3000);
    
  // Serial.printf("r: %X\n", r);
  // Serial.printf("g: %X\n", g);
  // Serial.printf("b: %X\n", b);
  // Serial.printf("color: %X\n", color);
  Serial.printf("draw done!\n");
}

// display
void display_photo(uint8_t * data, size_t len, uint32_t w, uint32_t h){
  Serial.printf("data len: %d\n", len);

  // b1111100000000000 -> 0xF800 -> 63488
  // b0000011111100000 -> 0x07E0 -> 2016
  // b0000000000011111 -> 0x001F -> 31
  // b1111111111111111 -> 0xFFFF -> 65535
  // b00111111 -> 0x003F
  // b00011111 -> 0x001F
  uint32_t k;
  uint16_t color = 0;
  uint8_t r, g, b;
  tft.startWrite();
  for(uint32_t y = 0; y < h; y++){
    for(uint32_t x = 0; x < w; x++){
      k = (y * w * 2) + x * 2;
      color = (data[k] << 8) | data[k+1];
      tft.writePixel(x, y, color);

      r = (color >> 11) << 3;
      g = (color >> 5) << 2;
      b = (color) << 3;
    }
  }
  tft.endWrite();

  ei::signal_t signal;
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);

  delay(3000);
    
  // Serial.printf("r: %X\n", r);
  // Serial.printf("g: %X\n", g);
  // Serial.printf("b: %X\n", b);
  // Serial.printf("color: %X\n", color);
  Serial.printf("draw done!\n");
}

void setup() {
  init_done = false;

  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);

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
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

//    FRAMESIZE_96X96,    // 96x96
//    FRAMESIZE_QQVGA,    // 160x120
//    FRAMESIZE_240X240,  // 240x240
//    FRAMESIZE_QVGA,     // 320x240

//    PIXFORMAT_RGB565,    // 2BPP/RGB565
//    PIXFORMAT_YUV422,    // 2BPP/YUV422
//    PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
//    PIXFORMAT_JPEG,      // JPEG/COMPRESSED
//    PIXFORMAT_RGB888,    // 3BPP/RGB888
    
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

  action_capture = false;
  //pinMode(BTN_CAPTURE, INPUT);
  pinMode(BTN_CAPTURE, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(BTN_CAPTURE), do_capture, RISING);
  
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextWrap(true);
  tft.print("Init done!");
  
  init_done = true;
}

void do_capture(){
  action_capture = true;
}

void loop() {

  if(action_capture){
    Serial.println("\nChange action_capture flag...");
    tft.setCursor(0, 121);
    tft.setTextSize(1);
    tft.print("do_capture...");
    Serial.println("\nDone action_capture flag...");
  }
  //if(init_done && camera_sign && action_capture){
  if(init_done && camera_sign){
    Serial.println("\nPicture Capture Command is sent");
    capture_photo();
    tft.setCursor(0, 121);
    tft.print("done_capture...");
    Serial.println("capture_photo()");
    imageCount++;
    action_capture = false;
  }
}
