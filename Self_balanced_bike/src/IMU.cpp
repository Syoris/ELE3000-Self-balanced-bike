#include "IMU.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

volatile bool mpuInterrupt;     // indicates whether MPU interrupt pin has gone high
bool waitForCmd = false;    // To wait for a character sent before starting
bool printData = false;  // Print data to debug


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void dmpDataReady() {
    mpuInterrupt = true;
}

// To Initialise MPU6050 unit and make sure it's properly connected
bool IMU_Setup(){
    // join I2C bus
    Wire.begin();

    // Initialize device MPU
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    if(waitForCmd){
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
    }

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    return checkConnection();
}

bool checkConnection(){
   if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Teensy external interrupt 15)..."));
        attachInterrupt(15, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        return true;
    } 
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

void IMU_Compute(float* ypr){
    // Wait for MPU interrupt or extra packet(s) available
    if(mpuInterrupt){
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
        
        if(fifoCount > packetSize) {
            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));
                // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } 
            else if (mpuIntStatus & 0x02) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                ypr[0] = ypr[0]; //*TO_ANGLE;
                ypr[1] = ypr[1]; //*TO_ANGLE;
                ypr[2] = ypr[2]; //*TO_ANGLE;

                if (printData){
                    Serial.print("ypr\t");
                    Serial.print(ypr[0]);
                    Serial.print("\t");
                    Serial.print(ypr[1]);
                    Serial.print("\t");
                    Serial.println(ypr[2]);
                }
            }
        }
    } 
}

