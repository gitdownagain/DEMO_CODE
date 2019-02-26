#ifndef POLOLUHBRIDGE_HPP
#define POLOLUHBRIDGE_HPP

#include "mbed.h"

class PololuHBridge
{
public:
    PololuHBridge(PinName pwm, PinName dir, PinName reset);
    
    void run(float cmd);
    void reset();
    void stop();

protected:
    PwmOut _pwm;
    DigitalOut _direction;
    DigitalOut _rst;
    float _p;
    float _clamp(float value, float min, float max);
};

#endif