/*******************************************************************************
Author:           Troy Holley
Title:            ServoDriver.cpp
Date:             02/14/2019 (last modified)

Description/Notes:

Servo driver that sends the correct PWM signal to the rudder servo.

Had to create a driver that does not use pwmOut because of the period timer
conflict with the H-bridge motor controllers--the MBED won't allow different PWM
periods on pins initilialized as PWM, so timing is controlled through the main
ticker. 

*******************************************************************************/     

#include "ServoDriver.hpp"

// INNER LOOP ServoDriver has settings, endpoints, direction, neutral point
// initialize default calibration/parameter values
ServoDriver::ServoDriver(PinName digital_pin) : _pwm_out(digital_pin) {
    _period_cnt = 0;
    _center_pwm = 1640.0; // mechanical center trimmed to be 1.5ms (1500us)
    _min_deg = -45.0;     // lower deflection against end stop
    _max_deg =  45.0;     // upper deflection against end stop
    _min_pwm = 2240.0;    // pwm corresponding to _min_deg
    _max_pwm = 1040.0;    // pwm corresponding to _max_deg
    
    _valid_servo_position_pwm = (unsigned int)_center_pwm; // servo position on boot-up
    
    _paused = false;
}

/////////////////////////////////////////////////////////////////////////////
// PWM TIMING SETUP AND SUPPORT FUNCTIONS

// initialize the ticker that will pulse on, start an off ticker, off-ticker pulses off and detaches, rinse and repeat
void ServoDriver::init() {
}

void ServoDriver::runServo() {
    pwm_pulse_on();
}

void ServoDriver::pwm_pulse_off() {
    _pwm_out = 0;                       //changed the pulse on and off to TIMEOUT so you don't have to detach anything
}

void ServoDriver::pwm_pulse_on() {
    if (_paused) {
        pwm_pulse_off_timeout.detach();
    }
    else {
        pwm_pulse_off_timeout.attach_us(callback(this, &ServoDriver::pwm_pulse_off), _valid_servo_position_pwm);  // _valid_servo_position_pwm is the signal in microseconds, e.g. 1500
    }
    _pwm_out = 1;
}

void ServoDriver::pause() {
    //pwm_pulse_on_ticker.detach();     //run a pwm pulse on a digital out pin that has a period of 20 ms
}

void ServoDriver::unpause() {
    //init();
}

// SERVO OUTPUT MAPPING

// this function converts a desired degree location into a PWM command
void ServoDriver::setPosition_deg(float input_deg) {
    // make sure desired angle is within the min and max deflection angles
    _degrees_set_position = servoClamp<float>(input_deg, _min_deg, _max_deg);

    // servo calibration
    // slope is pwm signal over degrees (y = mx + b) using linear fit
    // different slopes for above or below center!
    float slope;
    if (_degrees_set_position > 0)
        slope = (_center_pwm - _min_pwm) / (-_min_deg);

    else if (_degrees_set_position < 0)
        slope = (_max_pwm - _center_pwm) / (_max_deg);

    else
        slope = 0.0;

    // calculate linear servo response (output = slope*input + offset)
    // negative slope needed for correct direction response
    float initial_output = (slope * _degrees_set_position) + _center_pwm;    

    // saturate the pwm output to the min/max pwm limits
    // remember the min/max pwm values may not be (min < max)
    // Basically handles the direction of movement
    if (_max_pwm > _min_pwm)
        _valid_servo_position_pwm = servoClamp<float>(initial_output, _min_pwm, _max_pwm);
        
    else if (_max_pwm < _min_pwm)
        _valid_servo_position_pwm = servoClamp<float>(initial_output, _max_pwm, _min_pwm);
}

//the comparison below is used to swap the rotation of the servo

void ServoDriver::setPWM(float input_pwm) {
    if (_max_pwm > _min_pwm)
        _valid_servo_position_pwm = (unsigned int) servoClamp<float>(input_pwm, _min_pwm, _max_pwm);   //volatile unsigned int
    
    else if (_min_pwm > _max_pwm)
        _valid_servo_position_pwm = (unsigned int) servoClamp<float>(input_pwm, _max_pwm, _min_pwm);   //volatile unsigned int
}

///////////////////////////////////////////////////////////////////////////
// SETTING PARAMETERS
void ServoDriver::setMinPWM(float pwm_input) {
    _min_pwm = pwm_input;
}

void ServoDriver::setMaxPWM(float pwm_input) {
    _max_pwm = pwm_input;
}

void ServoDriver::setCenterPWM(float pwm_input) {
    _center_pwm = pwm_input;
}

void ServoDriver::setMinDeg(float deg) {
    _min_deg = deg;
}

void ServoDriver::setMaxDeg(float deg) {
    _max_deg = deg;
}

///////////////////////////////////////////////////////////////////////////
// READING PARAMETERS
float ServoDriver::getSetPosition_deg() {    //rename to getSetPosition later for consistency
    return _degrees_set_position;
}
    
// returns pwm signal in microseconds, for example: 1580 microseconds
float ServoDriver::getSetPosition_pwm() {
    return (float)_valid_servo_position_pwm;    //servo is unsigned int
}

float ServoDriver::getMinPWM() {
    return _min_pwm;
}

float ServoDriver::getMaxPWM() {
    return _max_pwm;
}

float ServoDriver::getCenterPWM() {
    return _center_pwm;
}

float ServoDriver::getMinDeg() {
    return _min_deg;
}

float ServoDriver::getMaxDeg() {
    return _max_deg;
}