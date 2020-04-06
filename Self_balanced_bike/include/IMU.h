#ifndef _IMU_H
#define _IMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define TO_ANGLE 180/M_PI

// MPU control/status vars
extern MPU6050 mpu;
extern bool dmpReady;  // set true if DMP init was successful
extern uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
extern uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
extern uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
extern uint16_t fifoCount;     // count of all bytes currently in FIFO
extern uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
extern Quaternion q;           // [w, x, y, z]         quaternion container
extern VectorFloat gravity;    // [x, y, z]            gravity vector

extern volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high
extern bool waitForCmd;    // To wait for a character sent before starting
extern bool printData;  // Print data to debug

// Protoypes
void dmpDataReady();
bool IMU_Setup();
bool checkConnection();
void IMU_Compute(float* ypr);

#endif