/*
  Drone.h - Library for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#ifndef Drone_h
#define Drone_h

#include "Arduino.h"

class Drone
{
    public:
       Drone(int esc1, int esc2, int esc3, int esc4);
       
       //motor controls
       void roll();
       void yaw();
       void pitch();
       
       //sensor reading
       //void read ultrasonic sensor
       //void read infrared sensor
       //etc..
       
    private:
        int esc1_pin;
        int esc2_pin;
        int esc3_pin;
        int esc4_pin; 
};

#endif
