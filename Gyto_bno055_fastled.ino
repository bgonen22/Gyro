#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FastLED.h>

#define DATA_PIN      4        // Arduino pin connected to strip
#define NUM_LEDS      30       // Total number of RGB LEDs on strip
#define TRACE_SIZE 4            // Length of each tgrace
#define TRACES_DIST 30          // the distance between trace tail to the next head 

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

CRGB leds[NUM_LEDS];

void yawRotation(float angle, float old[3], float result[3]) {
  float cosA = cos(angle);
  float sinA = sin(angle);
  result[0] = cosA * old[0] - sinA * old[1];
  result[1] = sinA * old[0] + cosA * old[1];
  result[2] = old[2];
}

void pitchRotation(float angle, float old[3], float result[3]) {
  float cosA = cos(angle);
  float sinA = sin(angle);
  result[0] = old[0];
  result[1] = cosA * old[1] - sinA * old[2];
  result[2] = sinA * old[1] + cosA * old[2];
}

void rollRotation(float angle, float old[3], float result[3]) {
  float cosA = cos(angle);
  float sinA = sin(angle);
  result[0] = cosA * old[0] + sinA * old[2];
  result[1] = old[1];
  result[2] = -sinA * old[0] + cosA * old[2];
}

float original[NUM_LEDS][3];

void calculateLEDPositions(int totalLEDs, float radius) {
  for (int i = 0; i < totalLEDs; ++i) {
    float angle = 2 * PI * i / totalLEDs; // Calculate the angle for each LED
    float x = radius * cos(angle);        // X coordinate
    float z = radius * sin(angle);        // Y coordinate
    original[i][0] = x;
    original[i][1] = 0;
    original[i][2] = z;
  }
}

void cartesian_to_hsv(float x, float y, float z, uint8_t& hue, uint8_t& saturation, uint8_t& value) {
  // Normalize coordinates to [0, 1]
  float normalized_x = (x + 1) / 2.0;
  float normalized_y = (y + 1) / 2.0;
  float normalized_z = (z + 1) / 2.0;

  // Calculate hue using arctangent
  float angle = atan2(normalized_y - 0.5, normalized_x - 0.5);

  // Map angle to [0, 1]
  float hue_float = (angle + PI) / (2 * PI);

  // Convert hue to 16-bit integer range [0, 65535]
  hue = hue_float * 255;

  // Saturation is the distance from the center in 2D
  saturation = static_cast<uint8_t>(sqrt(sq(normalized_x - 0.5) + sq(normalized_y - 0.5)) * 255);

  // Value is based on the z-coordinate
  value = static_cast<uint8_t>(normalized_z * 255);
}

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
  calculateLEDPositions(NUM_LEDS, radius);

  Serial.println("Finished setup");
}
const byte fadeAmt = 100; // 
void loop() {
  // FastLED.clear();
  sensors_event_t event;
  bno.getEvent(&event);

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float ypr[3];
  ypr[0] = euler.x();
  ypr[1] = euler.y();
  ypr[2] = euler.z();
  float result[3];
  // uint16_t hue;
  uint8_t saturation, value;
  byte hue; // = HUE_RED;
  static double head = 0.0;

  yawRotation(ypr[0], original[(int)head], result);
  pitchRotation(ypr[1], result, result);
  rollRotation(ypr[2], result, result);
  cartesian_to_hsv(result[0], result[1], result[2], hue, saturation, value);
  Serial.print("hue, sat, val ");
  Serial.print(hue);
  const int deltaHue  = 4;
  const double head_speed = 0.5;

  for (int i = head; i > head-TRACE_SIZE; i--) { // Run on one trace
    hue += deltaHue;
    for (int j = i; j > 0; j=j-TRACE_SIZE-TRACES_DIST) { // duplicate the trace till the begining of the strip
      leds[j].setHue(hue);
    }
  }
  head += head_speed;
  if (head == NUM_LEDS + TRACE_SIZE) {
    head = head - TRACE_SIZE - TRACES_DIST;
  }
  // Randomly fade the LEDs
  for (int j = 0; j < NUM_LEDS; j++)
      if (random(3) > 1)
          leds[j] = leds[j].fadeToBlackBy(fadeAmt);  

  FastLED.show();
  delay(20);
}
