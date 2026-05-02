#include <Adafruit_NeoPixel.h>

// Argument 1: Number of pixels (1)
// Argument 2: Pin number (48)
// Argument 3: Pixel type (GRBW + 800KHz)
Adafruit_NeoPixel LED_RGB(1, 48, NEO_GRBW + NEO_KHZ800);

void setup() {
  LED_RGB.begin();
  LED_RGB.setBrightness(50); // Set brightness to about 60%
}

void loop() {
  // Set pixel 0 to Red
  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(255, 0, 0)));
  LED_RGB.show();
  delay(2000);

  // Set pixel 0 to Green
  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 255, 0)));
  LED_RGB.show();
  delay(2000);

  // Set pixel 0 to Blue
  LED_RGB.setPixelColor(0, uint32_t(LED_RGB.Color(0, 0, 255)));
  LED_RGB.show();
  delay(2000);
}