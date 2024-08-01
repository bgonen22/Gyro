#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FastLED.h>

#define DATA_PIN      4        // Arduino pin connected to strip
#define NUM_LEDS      60       // Total number of RGB LEDs on strip
#define TRACE_SIZE 4            // Length of each tgrace
#define TRACES_DIST 30          // the distance between trace tail to the next head 
#define COLOR_SHIFT 1 // the jump in the color each iteration
const byte fadeAmt = 100; // amount of fade (0-255)
const byte fade_odds = 60; // precent to fade

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  Serial.begin(9600);
  while (!Serial); // wait for Serial monitor to open

  if (!bno.begin()) {
    Serial.println("No BNO055 detected, check wiring or I2C address!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  const float radius = 1.0;  // Adjust the radius as needed
  
  Serial.println("Finished setup");
}

// byte hue; // = HUE_RED;

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  static byte hue = 5;
  imu::Vector<3> angVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  float velocity = sqrt(angVelocity.x() * angVelocity.x() +
                        angVelocity.y() * angVelocity.y() +
                        angVelocity.z() * angVelocity.z());

  hue = (hue + COLOR_SHIFT) % 256;
  Serial.println(hue);
  static double head = 0.0;
  const double head_speed = 0.5;

  for (int i = head; i > head - TRACE_SIZE; i--) {
    for (int j = i; j > 0; j = j - TRACE_SIZE - TRACES_DIST) {
      leds[j].setHue(hue);
    }
  }

  head += head_speed;
  if (head >= NUM_LEDS + TRACE_SIZE) {
    head = head - TRACE_SIZE - TRACES_DIST;
  }

  for (int j = 0; j < NUM_LEDS; j++) {
    if (random(100) < fade_odds) {
      leds[j] = leds[j].fadeToBlackBy(fadeAmt);
    }
  }

  FastLED.show();
  int current_delay = 100 / max(velocity / 5, 1);
  delay(current_delay);
}

