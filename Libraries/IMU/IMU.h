/*
  IMU.h - Library for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#ifndef I2CDEV_h
#define I2CDEV_h
#include "I2Cdev.h"
#endif
#ifndef MPU6050_6Axis_MotionApps20_h
#define MPU6050_6Axis_MotionApps20_h
#include "MPU6050_6Axis_MotionApps20.h"
#endif
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class IMU
{
    public:
        IMU();
        bool isStable();
        bool initialSetup();
        void getYPR();
        void dmpDataReady();
        
        MPU6050 mpu;
        
        bool dmpReady;
        uint8_t mpuIntStatus; //trying to get uint8_t
        uint8_t devStatus;
        uint16_t packetSize;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];
        volatile bool mpuInterrupt;
        
        Quaternion q;           // [w, x, y, z]         quaternion container
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        VectorFloat gravity;    // [x, y, z]            gravity vector

        bool stable;
        
    private:
        
};

#endif
