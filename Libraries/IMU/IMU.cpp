/*
  IMU.cpp - Function Definitions for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#include "IMU.h"

IMU::IMU()
{
    dmpReady = false;
    stable = false;
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif      
}

bool IMU::initialSetup()
{
     mpu.initialize();
    
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    if(mpu.testConnection())
    {
        devStatus = mpu.dmpInitialize();
        
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(67);
        mpu.setYGyroOffset(7);
        mpu.setZGyroOffset(85);
        mpu.setXAccelOffset(1665);
        mpu.setYAccelOffset(1709);
        mpu.setZAccelOffset(1855);
        
        if(devStatus == 0)
        {
            mpu.setDMPEnabled(true); //enable the dmp
            // attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING); //attachs the interrupt to pin 2
            mpuIntStatus = mpu.getIntStatus(); //reads registers and also clears
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
            Serial.print("Attached interrupt of packetsize: ");
            Serial.print(packetSize);
            Serial.print("\n");
        }
        else
        {
            //Errors: 1 = initial memory load failed, 2 = DMP Configuration updates failed
            Serial.print("dmp initialization failed : Code ");
            Serial.print(devStatus);
        }
        
        Serial.println("Stabilizing for 15 seconds ....");
        delay(1000*15); //wait the mpu unit to handle itself
        Serial.println("Finished Stabilization. Going to loop phase.");
        stable = true;
        
        return true;
    }
    else
    {
        Serial.println("failed to connect to mpu");
        stable = false;
        
        return false;
    }
}

void IMU::getYPR()
{
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //wait for MPU interrup or extra packet(s) available
    }  

    mpuInterrupt = false; //reset the interrupt flag
    mpuIntStatus = mpu.getIntStatus(); //get INT_STATUS byte

    fifoCount = mpu.getFIFOCount(); //get current FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) //check for overflow
    {
        mpu.resetFIFO();
        Serial.println("FIFO Overflow!");
    }
    else
    {
        //    while (fifoCount < packetSize) //wiat for correct available data length, should be a very short wait
        //    {
        //      fifoCount = mpu.getFIFOCount();
        //    }

        mpu.getFIFOBytes(fifoBuffer, packetSize); //read a packet from FIFO

        fifoCount -= packetSize; //track FIFO count here in case there is > 1 packet available. This lets us immediately read more without waiting for an interrupt

        // grab data from sensor
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        for (int i = 0; i < 3; i = i + 1)
        {
            ypr[i] = ypr[i] * 180 / M_PI;
        }
    }
}

bool IMU::isStable()
{
    return stable;
}

void IMU::dmpDataReady()
{
    mpuInterrupt = true;
}
