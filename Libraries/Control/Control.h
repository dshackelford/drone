/*
  Control.h - Library for controlling DIY Drones.
  Created by Dylan J. Shackelford, July 26, 2017.
  Released into the public domain.
*/

#ifndef CONTROL_h
#define CONTROL_h

#include "Arduino.h"

class Control
{
    public:
        Control(double kpInit, double kiInit, double kdInit, double max, double min);
        
        double getOutput(double current, double throttle, double refInit); //where the PID will
        
        void setKp(double kpInit);
        void setKi(double kiInit);
        void setKd(double kdInit);
    
        void setReference(double refInit);
        void setPrevTime(double prevTime);
        void setDt(double dtInit);
    
        void setError(double errorInit);
        void setPrevError(double prevErrorInit);
        void setSumError(double sumErrorInit);
    
        void setMaxOutput(double maxOutputInit);
        void setMinOutput(double minOutputInit);
    
    private:
        double boundVal(double val, double high, double low);
    
        double kp;
        double ki;
        double kd; 
        double ref;
        double dt;
        double error;
        double prevError;
        double sumError;
        double dError;
        double prevTime;
        double maxOutput;
        double minOutput;
        double diffOutput;
};

#endif
