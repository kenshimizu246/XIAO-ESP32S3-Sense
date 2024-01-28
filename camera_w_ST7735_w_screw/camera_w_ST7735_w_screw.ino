#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <Screw_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

#define TFT_CS    1
#define TFT_RST   2
#define TFT_DC    3
#define TFT_MOSI  9
#define TFT_SCK   7

#define BTN_CAPTURE 4

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           96
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           96
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const uint16_t imageWidth = 96;
const uint16_t imageHeight = 96;

unsigned long lastCaptureTime = 0; // Last shooting time
int imageCount = 1;                // File Counter
bool camera_sign = false;          // Check camera status
uint32_t buff_size = 0;
uint16_t * buff = NULL;
bool action_capture = false;
volatile bool init_done = false; // needs volatile in the interruption.

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
uint8_t *snapshot_buf; //points to the output of the capture

// capure and display photo
void capture_photo() {
  
  // tft.fillScreen(ST77XX_BLACK);
  
  // Take a photo
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }
  
  display_photo(fb->buf, fb->len, imageWidth, imageHeight);
      
  // Release image buffer
  esp_camera_fb_return(fb);

  //delay(1000);

  Serial.println("Photo saved to file");
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
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
  uint32_t k, j;
  uint16_t color = 0;
  uint8_t r, g, b;
  tft.startWrite();
  for(uint32_t y = 0; y < h; y++){
    for(uint32_t x = 0; x < w; x++){
      k = (y * w * 2) + x * 2;
      color = (data[k] << 8) | data[k+1];
      tft.writePixel(x, y, color);

      j = (y * w * 3) + x * 3;
      snapshot_buf[j] = (color >> 11) << 3; // Red
      snapshot_buf[j + 1] = (color >> 5) << 2; // Green
      snapshot_buf[j + 2] = (color) << 3; // Blue
    }
  }
  tft.endWrite();

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  Serial.printf("EI_IMPULSE_ERROR: %d\n", err);
  Serial.printf("bounding_boxes_count: %d\n", result.bounding_boxes_count);

  char text[8];
  //tft.drawNumber(detector.getPersonScore(), 100, 100);
  sprintf(text, "%d : %d", err, result.bounding_boxes_count);
  //sprintf(text, "%d : %d", 0, 0);
  tft.setCursor(0, 121);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextWrap(true);
  tft.print(text);

  //delay(10000);
    
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
  config.frame_size = FRAMESIZE_96X96;
  //config.pixel_format = PIXFORMAT_RGB565;
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
  
  ei_sleep(2000);
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
  //if(init_done && camera_sign && action_capture){
  if(init_done && camera_sign){
    Serial.println("\nPicture Capture Command is sent");

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    capture_photo();
    tft.setCursor(0, 121);
    tft.print("done_capture...");
    Serial.println("capture_photo()");
    free(snapshot_buf);

    imageCount++;
    action_capture = false;
  }
}
