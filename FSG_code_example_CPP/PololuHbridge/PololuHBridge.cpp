/*******************************************************************************
Author:           Trent Young
Title:            PololuHBridge.cpp
Date:             09/01/2017

Description/Notes:

Controls the PWM signal going to the H-Bridge motor controllers.

By varying the output signal, this changes the voltage that is sent to each
motor, slowing, speeding up, or stopping motors.  

*******************************************************************************/

#include "mbed.h"
#include "PololuHBridge.hpp"


PololuHBridge::PololuHBridge(PinName pwm, PinName dir, PinName reset):
    _pwm(pwm),
    _direction(dir),
    _rst(reset)
{
    _pwm.period_us(50);
    run(0.0);
    //pull Hbridge reset low then high to clear any faults
    _rst = 0;
    _rst = 1;
    _direction = 0;
}

void PololuHBridge::run(float cmd)
{
    _p = _clamp(cmd, -1.0, 1.0);

    //This sets motor direction. Positive should give positive velocity (piston retract)
    if (_p <= 0.0) {
        _direction = 1;
        //the pwm function needs an absolute value
        _p = abs(_p);
    } else if (_p > 0.0) {
        _direction = 0;
    }
    
    //send the dute cycle (percentage represented as 0.0 through +/-1.0)
    _pwm = _p;
    
    return;
}

void PololuHBridge::reset()
{
    _rst = 0;
    _rst = 1;
}

void PololuHBridge::stop()
{
    //stop the motor output (duty cycle 0%)
    _pwm = 0;
}

float PololuHBridge::_clamp(float value, float min, float max)
{
    if(value < min) {
        return min;
    } else if(value > max) {
        return max;
    } else {
        return value;
    }
}