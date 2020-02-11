#include <Arduino.h>

// Private libraries
#include "Wire.h"
#include <IR_Receiver.h>

//IMU
#include "MPU6050.h"
#include "I2Cdev.h"


MPU6050 accelgyro;
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

#define LED_PIN 13
bool blinkState = false;

void setup() {
  Wire.begin();      // join I2C bus
  Serial.begin(38400);    //  initialize serial communication
  //IR_Setup();
  delay(2000);

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);  // configure LED pin
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
  delay(50);
}



// void loop() {
//   // IR_decode();
//   unsigned long val = IR_receive();
//   if (val == OFF)
//       Serial.println(val);
//   delay(50);
// }