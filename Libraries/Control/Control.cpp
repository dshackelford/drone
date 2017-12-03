/*
  Control.cpp - Function Definitions for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#include "Control.h"

Control::Control(double kpInit, double kiInit, double kdInit, double max, double min)
{
    kp = kpInit;
    ki = kiInit;
    kd = kdInit;
    
    prevError = 0;
    sumError = 0;
    dError = 0;
    
    //max output settings
    maxOutput = max; //i.e. 2000 microseconds
    minOutput = min; //i.e. 500 microseconds
    diffOutput = abs(max - min);
}

double Control::getOutput(double current, double throttle, double refInit)
{
    //Proportional
    error = refInit - current;
    
    //Derivative
    double currentTime = millis();
    dt = (currentTime - prevTime)/1000;
    dError = (prevError - error)/dt;
    
    //Integral
    if(error < 5 && error > -5) //only need to accumulate error close to operating range.
    {
       sumError = error + sumError;
    }
    else
    {
        sumError = 0;
    }
    
    double output = error*kp + kd*dError + ki*sumError;
    
    //we know the largest difference from the currenlty fastest operating value
    if(output < -diffOutput)
    {
        output = -diffOutput;
    }
    else if(output > diffOutput)
    {
        output = diffOutput;
    }
    
    output = boundVal(output + throttle,maxOutput,minOutput);
    
    //Set values for next iteration
    prevTime = currentTime;
    prevError = error;
    
    return output;
}

double Control::boundVal(double val, double high, double low)
{
    if(val < low)
    {
        val = low;
    }
    else if(val > high)
    {
        val = high;
    }
    
    return val;
}

void Control::setReference(double refInit)
{
    ref = refInit;
}

void Control::setKp(double kpInit)
{
    kp = kpInit;
}

void Control::setKi(double kiInit)
{
    ki = kiInit;
}

void Control::setKd(double kdInit)
{
    kd = kdInit;
}

void Control::setError(double errorInit)
{
    error = errorInit;
}

void Control::setPrevError(double prevErrorInit)
{
    prevError = prevErrorInit;
}

void Control::setSumError(double sumErrorInit)
{
    sumError = sumErrorInit;
}
