/*******************************************************************************
Author:           Dan Edwards (some modifications by Troy)
Title:            OuterLoop.cpp
Date:             02/28/2019

Description/Notes:

Outer Loop PID controller.

Used to control the outputs that drive the *desired* position of the buoyancy
control engine (BCE) and battery mass mover (BMM). Inputs to these PID loops
are the AHRS/IMU pitch and heading (yaw) data, and depth computed from the
pressure transducer.

Each outer loop is its own instance, created in the StaticDefs file.

The outer loop uses filtered position and velocity data to feed into the PID
loop that controls the output that is trying to get the system to its desired
depth, pitch, and heading (it is attempting to apply a correction to reduce the
error AKA get us to the desired state). 
*******************************************************************************/

#include "mbed.h"
#include "OuterLoop.hpp"
#include "StaticDefs.hpp"

OuterLoop::OuterLoop(float interval, int sensor):
    _filter(),
    _pid()
{
    _Pgain = 0.5;
    _Igain = 0.0;
    _Dgain = 0.1;
    
    _filterFrequency = 2.0;
    _deadband = 0.0;

    _dt = interval;
    _sensor = sensor; // select the sensor ... hacky
    setIHiLimit(3.0);  //give each outerloop instance an integral saturation limit of some default
    setILoLimit(3.0);
    
    if (_sensor == 2) {
        _pid.setHeadingFlag(true);  //used to update the handling for the heading PID control loop
    }
}

void OuterLoop::init() {
    // load gains into pid controller
    setFilterFrequency(_filterFrequency);
    setControllerP(_Pgain);
    setControllerI(_Igain);
    setControllerD(_Dgain);
    
    // setup the controller object
    toggleDeadband(true);
    setDeadband(_deadband);
}

//removed references to the pulse that we are no longer using and the "update" function
//each update the specified sensor is called, in this case we're running at 10 Hz from the main loop ticker
void OuterLoop::runOuterLoop() { 
    // update the position velocity filter
    if (_sensor == 0) {
        //_sensorVal = depth().getDepthFt();    //old method, remove later
        _sensorVal = depth().newGetDepthFt();
    } else if (_sensor == 1) {
        _sensorVal = imu().getPitch();
    } else if (_sensor == 2) {
        _sensorVal = imu().getHeading();
    } else {
        error("\n\r This sensor option does not exist");
    }
    
    // use the sensor reading to update the Position Velocity Filter (PVF)
    _filter.update(_dt, _sensorVal);
    
    // refresh the PVF results and load into class variables
    refreshPVState();
 
    // update the PID controller with latest data
    _pid.update(_position, _velocity, _filter.getDt());
}

void OuterLoop::setCommand(float value) {
    _SetPoint = value;
    _pid.writeSetPoint(_SetPoint);
}
 
float OuterLoop::getCommand() {
    return _SetPoint;
}

float OuterLoop::getOutput() {
    /* PID output + offset to drive the motors to the correct position */
    return _pid.getOutput() + _offset;
}
 
void OuterLoop::refreshPVState() {
    _position = _filter.getPosition();
    _velocity = _filter.getVelocity();
}
 
float OuterLoop::getPosition() {
    return _position;
}
 
float OuterLoop::getVelocity() {
    return _velocity;
}
 
void OuterLoop::setControllerP(float P) {
    _Pgain = P;
    _pid.setPgain(_Pgain);
}
 
float OuterLoop::getControllerP() {
    return _Pgain;
}
 
void OuterLoop::setControllerI(float I) {
    _Igain = I;
    _pid.setIgain(_Igain);
}
 
float OuterLoop::getControllerI() {
    return _Igain;
}
 
void OuterLoop::setControllerD(float D) {
    _Dgain = D;
    _pid.setDgain(_Dgain);
}
 
float OuterLoop::getControllerD() {
    return _Dgain;
}

//uses position velocity filter class
void OuterLoop::setFilterFrequency(float frequency) {
    _filterFrequency = frequency;
    _filter.writeWn(frequency);   
}

float OuterLoop::getFilterFrequency() {
    return _filterFrequency;               //new 7/11/18
}
 
void OuterLoop::setDeadband(float deadband) {
    _deadband = deadband;
    _pid.setDeadBand(_deadband);
}

float OuterLoop::getDeadband() {
    return _deadband;
}
 
bool OuterLoop::toggleDeadband(bool toggle) {
    _pid.toggleDeadBand(toggle);
    return toggle;
}

void OuterLoop::setOutputOffset(float offset) {
    _offset = offset;
}
 
float OuterLoop::getOutputOffset() {
    return _offset;
}

//not used yet
void OuterLoop::setIHiLimit (float limit){
    _pid.setHiLimit(limit);
}

void OuterLoop::setILoLimit (float limit){
    _pid.setLoLimit(limit);    
}

float OuterLoop::getPIDErrorTerm() {
    return _pid.getErrorTerm();
}

float OuterLoop::getPIDIntegralTerm(){
    return _pid.getIntegralTerm();
}