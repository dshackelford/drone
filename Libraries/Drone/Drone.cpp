/*
  Drone.cpp - Function Definitions for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#include "Drone.h"

Drone::Drone(int esc1, int esc2, int esc3, int esc4)
{
    esc1_pin = esc1;
    esc2_pin = esc2;
    esc3_pin = esc3;
    esc4_pin = esc4;
}

void Drone::roll()
{
    Serial.println("roll!");
}

void Drone::pitch()
{
    Serial.println("Pitch");
}

void Drone::yaw()
{
    Serial.println("yaw!");
}
