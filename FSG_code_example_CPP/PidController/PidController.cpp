/*******************************************************************************
Author:           Troy Holley
Title:            PidController.cpp
Date:             12/11/2018

Description/Notes:

Generic PID controller used to control the output voltages or signals provided 
to the hardware. Modified this from old code that was causing the integral error
to rise because the PID loop was running even when the hardware was not moving.

*******************************************************************************/

#include "PidController.hpp"
#include "StaticDefs.hpp"

PIDController::PIDController() {
    _error = 0.0;
    _integral = 0.0;
    _previous_error = 0.0;  //new 01/08/19
    _derivative = 0.0;      //new 01/08/19
    _loLimit = 0.0;
    _hiLimit = 0.0;
    _deadbandFlag = false;
    _deadband = 0.0;
    _headingFlag = false;   //default for pressure and depth
}

// https://www.reddit.com/r/Python/comments/1qxp5d/pid_tuning_library/
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
void PIDController::newUpdate(float position, float dt) {

//Troy: checking the math 01/08/19 
    _error = _setPoint - position;      //position = new measurement
    
    // hack to get heading update correct
    if (_headingFlag) {
        if (_error >= 180){
            _error = _error - 360.0;
        }else if(_error <= -180){
            _error = _error + 360.0;
        }
    }
    
    _integral = _integral + (_error * dt);
    
    _derivative = (_error - _previous_error) / dt;
    
    _output = _Pgain*_error + _Igain*_integral + _Dgain*_derivative;
    
    //save the last error value for use in the next derivative calculation
    _previous_error = _error;
}


void PIDController::update(float position, float velocity, float dt) {
    // error update equations

    _error = _setPoint - position;  //position is your measured value, correct 01/07/19
    
    _temp_velocity = velocity;
    
    // hack to get heading update correct
    // need to make sure this makes physical sense
    if (_headingFlag) {
        if (_error >= 180){
            _error = _error - 360.0;
        }else if(_error <= -180){
            _error = _error + 360.0;
        }
    }
    
    //calculate integral error term (adding anti-windup)
    // ADDED k_aw or anti-windup term for bench testing (typically very small but anything from 0 to 0.9999...)
    float AWgain = 0.1;    // AntiWindupGain
    float integral_dot = _Pgain * _Igain * (_error - AWgain * abs(_error) * _integral);

    //calculate integral error term
    //_integral = _integral + (_error*dt);  //original equation
    _integral = _integral + integral_dot * dt;

    // pid controller equation                                          //derivative error had the wrong sign
    _output = _Pgain*_error + _Igain*_integral - _Dgain*velocity ;      //derivative_error = v_setpoint - v_actual * _Dgain

    // limiting on output & integral anti-windup (preventing integral from getting extremely high)
    // Get the current controller output, and stop integrating if it's saturated    
    // one solution, stop integrating when you get to the position limits

    // within deadband on error zeros output
    if (_deadbandFlag) {
        if (abs(_error) < _deadband) {
            _output = 0.0;
        }
    }
}

void PIDController::writeSetPoint(float cmd) {
    _setPoint = cmd;
}

float PIDController::getOutput() {
    return _output;
}

float PIDController::getErrorTerm() {
    return _error;
}

float PIDController::getIntegralTerm() {
    return _integral;
}

float PIDController::getDerivativeTerm() {
    return _derivative;
}

void PIDController::setPgain(float gain) {
    _Pgain = gain;
}

void PIDController::setIgain(float gain) {
    _Igain = gain;
}

void PIDController::setDgain(float gain) {
    _Dgain = gain;
}

void PIDController::toggleDeadBand(bool toggle) {
    _deadbandFlag = toggle;
}
void PIDController::setDeadBand(float deadband) {
    _deadband = deadband;
}

void PIDController::setHeadingFlag(bool heading_flag) {
    _headingFlag = heading_flag;
}

void PIDController::setHiLimit(float high_limit) {
    _hiLimit = high_limit;
}

void PIDController::setLoLimit(float low_limit){
    _loLimit = low_limit;
}

//reset these member variables
void PIDController::resetPidLoop() {
    _error = 0.0;
    _integral = 0.0;
    _previous_error = 0.0;
    _derivative = 0.0;
}