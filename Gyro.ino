#include <Arduino.h>
#include <math.h>
// Watchdog timer to reset the device if it freezes. 
#include <avr/wdt.h>

#include <Adafruit_NeoPixel.h>  // NeoPixel library from Adafruit
#define PIXELPIN       4        // Arduino pin connected to strip
#define NUMPIXELS      60       // Total number of RGB LEDs on strip
#define TRACE_SIZE 4            // Length of each tgrace
#define TRACES_DIST 5           // the distance between trace tail to the next head 

#ifdef __AVR__
  #include <avr/power.h>        // AVR Specific power library
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// MotionApps utilizes the "Digital Motion Processor" (DMP) on the MPU-6050
// to filter and fuse raw sensor data into useful quantities like quaternions,
// Euler angles, or Yaw/Pitch/Roll inertial angles
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5.0v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// yaw/pitch/roll angles (in degrees) calculated from the quaternions
// coming from the FIFO. Note this also requires gravity vector
// calculations. Also note that yaw/pitch/roll angles suffer from gimbal
// lock (for more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
#define BRIGHTNESS 80
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===                NEOPIXEL AHRS ROUTINE                     ===
// ================================================================

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example in the lbrary folder for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);

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

float original[NUMPIXELS][3]; // Original point on the ring
void calculateLEDPositions(int totalLEDs, float radius) {
  for (int i = 0; i < totalLEDs; ++i) {
    float angle = 2 * PI * i / totalLEDs; // Calculate the angle for each LED
    float x = radius * cos(angle);        // X coordinate
    float z = radius * sin(angle);        // Y coordinate
    original[i][0] = x;
    original[i][1] = 0;
    original[i][2] = z;
    // int hue = map(i, 0, totalLEDs, 0, 255); // Map LED index to hue for colorful visualization
    // pixels.setPixelColor(i,pixels.Color(hue, 255, 255));
  }
    Serial.println(F("Initializing led pos"));
}

void setup() {
  // One watchdog timer will reset the device if it is unresponsive
  // for a second or more
  wdt_enable(WDTO_8S);
    
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  pixels.begin();
  pixels.show(); // Initialize all pixels to 'off'

  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(210);
  mpu.setYGyroOffset(50);
  mpu.setZGyroOffset(5);
  mpu.setXAccelOffset(-1647); // 1688 factory default for my test chip
  mpu.setYAccelOffset(1281); // 1688 factory default for my test chip
  mpu.setZAccelOffset(871); // 1688 factory default for my test chip
  
  const float radius = 1.0;  // Adjust the radius as needed  
  calculateLEDPositions(NUMPIXELS, radius);
    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    // attachInterrupt(0, dmpDataReady, RISING);
    // mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
   // Configure the MPU6050 to measure acceleration
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  Serial.println("finished setup");

}

void print_data(float result[3], float yaw_deg, float pitch_deg, float roll_deg) {
 
  Serial.print("Yaw Pitch Roll: ("); 
  Serial.print(yaw_deg * 180/M_PI);
  Serial.print(", ");
  Serial.print(pitch_deg* 180/M_PI);
  Serial.print(", ");
  Serial.print(roll_deg* 180/M_PI);
  Serial.println(")");
  // Display the new coordinates
  Serial.print(" New Coordinates: (");
  Serial.print(result[0]);
  Serial.print(", ");
  Serial.print(result[1]);
  Serial.print(", ");
  Serial.print(result[2]);
  Serial.println(")");
}
void cartesian_to_hsv(float x, float y, float z, uint16_t& hue, uint8_t& saturation, uint8_t& value) {
  // Normalize coordinates to [0, 1]
  float normalized_x = (x + 1) / 2.0;
  float normalized_y = (y + 1) / 2.0;
  float normalized_z = (z + 1) / 2.0;

  // Calculate hue using arctangent
  float angle = atan2(normalized_y - 0.5, normalized_x - 0.5);

  // Map angle to [0, 1]
  float hue_float = (angle + PI) / (2 * PI);

  // Convert hue to 16-bit integer range [0, 65535]
  hue = hue_float * 65535;

  // Saturation is the distance from the center in 2D
  saturation = static_cast<uint8_t>(sqrt(sq(normalized_x - 0.5) + sq(normalized_y - 0.5)) * 255);

  // Value is based on the z-coordinate
  value = static_cast<uint8_t>(normalized_z * 255);
}
int head = NUMPIXELS;
// Variables to store previous gyro values for integration
float prevGyroX = 0;
float prevGyroY = 0;
float prevGyroZ = 0;
unsigned long prevTime = 0;
void loop () {
  // Serial.println("start loop");

   // Read gyroscope data
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Get current time
  unsigned long currentTime = millis();
  
  // Calculate time difference since last loop iteration
  float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
  
  // Calculate angular velocities (degrees per second)
  float gyroX = gx / 131.0; // Sensitivity scale factor for 250 degrees/s
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
  
  // Integrate angular velocities to get angles
  float roll = prevGyroX + gyroX * dt;
  float pitch = prevGyroY + gyroY * dt;
  float yaw = prevGyroZ + gyroZ * dt;
  // Find the maximum angular velocity among the three axes
  float maxAngularVelocity = max(max(abs(gyroX), abs(gyroY)), abs(gyroZ));
  Serial.println(maxAngularVelocity);

  // Output angular velocities
  // Serial.print("Roll speed: ");
  // Serial.println(gyroX);
  // Serial.print("Pitch speed: ");
  // Serial.println(gyroY);
  // Serial.print("Yaw speed: ");
  // Serial.println(gyroZ);
  
  // Update previous values for next iteration
  prevGyroX = roll;
  prevGyroY = pitch;
  prevGyroZ = yaw;
  prevTime = currentTime;

  pixels.clear();
  wdt_reset();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

   // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.resetFIFO();

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
  // display Euler angles in radians
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  float yaw_deg = ypr[0]  ;//* 180/M_PI;
  float pitch_deg = ypr[1] ;// * 180/M_PI;
  float roll_deg = ypr[2]  ;//* 180/M_PI;
  float result[3]; // Variable to store the new coordinates
  for (int i = head; i > head-TRACE_SIZE; i--) { // Run on one trace
    for (int j = i; j > 0; j=j-TRACE_SIZE-TRACES_DIST) { // duplicate the trace till the begining of the strip
      yawRotation(yaw_deg, original[j], result);
      pitchRotation(pitch_deg, result, result);
      rollRotation(roll_deg, result, result);

      // Convert 3D Cartesian coordinates to HSV
      uint16_t hue;
      uint8_t saturation, value;
      cartesian_to_hsv(result[0], result[1], result[2], hue, saturation, value);
      // Serial.println(j);


      // print_data(result,yaw_deg,pitch_deg,roll_deg);

      // Set NeoPixel color
      pixels.setPixelColor(j, pixels.gamma32(pixels.ColorHSV(hue, saturation, value-(i*10))));
      // pixels.rainbow(0);
      // Serial.print("hoe, sat, val ");
      // Serial.print(hue);
      // Serial.print(",");
      // Serial.print(saturation);
      // Serial.print(",");
      // Serial.print(value);
      // Serial.println(")");
    } // for j
  } // for i
  head ++;
  if (head == NUMPIXELS + TRACE_SIZE) {
    head = head - TRACE_SIZE - TRACES_DIST;
  }
  // print_data(result,yaw_deg,pitch_deg,roll_deg);
  // Print HSV values to serial monitor
  // Serial.print("H: ");
  // Serial.print(hueByte);
  // Serial.print(" S: ");
  // Serial.print(saturationByte);
  // Serial.print(" V: ");
  // Serial.println(valueByte);

  pixels.show();

  int current_delay = 300/(maxAngularVelocity+1);
  delay(current_delay);
  // Serial.println("end loop");

  // }
}

