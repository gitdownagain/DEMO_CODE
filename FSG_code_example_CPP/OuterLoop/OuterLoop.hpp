#ifndef OUTERLOOP_HPP
#define OUTERLOOP_HPP
 
#include "mbed.h"
#include "PidController.hpp"
#include "PosVelFilter.hpp"
 
// This class is an outer loop controller with its own instance of a position velocity filter.
 
class OuterLoop {
public:
    OuterLoop(float interval, int sensor);
    
    // functions for setting up
    void init();
    void update();
    void start();
    void stop();
    
    void runOuterLoop();
    
    // setting and getting variables
    void setCommand(float cmd);
    float getCommand();
    
    float getOutput();
    
    float getPosition();
    float getVelocity();
    
    void setControllerP(float P);
    float getControllerP();
    
    void setControllerI(float I);
    float getControllerI();
    
    void setControllerD(float D);
    float getControllerD();
    
    void setTravelLimit(float limit);
    float getTravelLimit();
    
    void setFilterFrequency(float frequency);
    float getFilterFrequency();
        
    void setDeadband(float deadband);
    float getDeadband();
    bool toggleDeadband(bool toggle);
    
    void setOutputOffset(float offset);
    float getOutputOffset();
    
    void setIHiLimit (float limit); // TZY, 3/1/18 Set saturation limit on controller integral
    void setILoLimit (float limit); // TZY, 3/1/18 Set saturation limit on controller integral
    
    float getPIDErrorTerm();
    float getPIDIntegralTerm();
        
protected:
    PosVelFilter _filter;
    PIDController _pid;
    
    void refreshPVState();
    
    float _SetPoint;
    float _sensorVal;

    // position and velocity in raw units
    float _position;
    float _velocity;

    // setup parameters
    float _Pgain;
    float _Igain;
    float _Dgain;
    float _dt;
    float _filterFrequency;
    float _deadband;
    char _sensor;
    float _offset;
};
 
#endif