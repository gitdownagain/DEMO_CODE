/*******************************************************************************
Author:           Troy Holley
Title:            IMU.cpp
Date:             08/01/2018

Description/Notes:

Linear Actuator code modified from Young's code.

This code moves the motors that control the depth (buoyancy control engine)
 and pitch (battery mass mover) by separate instantiations of the class.

*******************************************************************************/

#include "mbed.h"
#include "LinearActuator.hpp"
#include "StaticDefs.hpp"
#include "ConfigFile.h"
 
// this is where the variables that can be set are set when the object is created
LinearActuator::LinearActuator(float interval, PinName pwm, PinName dir, PinName reset, PinName limit, int adc_ch):
    _motor(pwm, dir, reset),
    _filter(),
    _pid(),
    _pulse(),
    _limitSwitch(limit)
{
    // attach the address of the switchPressed function on the falling edge of the switch (when the value is zero)
    _limitSwitch.fall(callback(this, &LinearActuator::switchPressed));
    
    _Pgain = 0.10;
    _Igain = 0.0;
    _Dgain = 0.0;
    
    _filterFrequency = 1.0;
 
    _adc_channel = adc_ch;
 
    _dt = interval;
 
    _init = true;
    _paused = false;
    
    _slope = 498.729/4096;  //this value should be correct for our current string pots using .625" diameter and 12 bit ADC (hardcoded in config as 0.12176)
    _deadband = 0.5;
 
    _pid_high_limit = 0.0;
    _pid_low_limit = 0.0;
}
 
void LinearActuator::init() {
    // initialize and start all of the member objects.
    // The pos-velocity filter for this item needs to be allowed to converge
    // Before turning over control to the motor
    // make sure that adc().init is called in mainline code before calling this function
 
    //load gains into pid controller this should eventually be in a config file
    setFilterFrequency(_filterFrequency);
    setControllerP(_Pgain);
    setControllerI(_Igain);
    setControllerD(_Dgain);
    
    //setup the controller object
    //set deadband and limits
    toggleDeadband(true);
    setDeadband(_deadband);
}

//move the channel selector into the refreshPVState function...12/12/18
void LinearActuator::update() { 
    // update the position velocity filter
    if (_adc_channel == 0) {
        _filter.update(_dt, adc().readCh0());
    } else if (_adc_channel == 1) {
        _filter.update(_dt, adc().readCh1());   //delta_t and counts updated in PosVelFilter
    } else {
        error("\n\r This ADC channel does not exist");
    }
 
    // refresh the filter results and load into class variables
    refreshPVState();
 
    // update the PID controller with latest data
    //_pid.update(_position_mm, _velocity_mms, _filter.getDt());    //commented out 01/08/2019
    
    //this currently runs all the time? 01/15/19, huge integrator errors
    _pid.newUpdate(_position_mm, _filter.getDt());
    
 
    if (_init){
        //The initialization phase is active
        //dont run the motors until the velocity stabilizes
        if (abs(_velocity_mms)<0.1) {
            //we must be converged and can now release the motor controls
            _init = false;
        }
    } 

    else if (_paused) {
        //if you get here, the pause function has stopped the motor
        //the only way out is for a function call to unpause the motor
        //this case also keeps the controller disconnected in the event that
        //homing is happening
        return;
    }
 
    else {
        // User or software allowed to move motors when the limit switch is not pressed
        // User or software allowed to move motors when limit switch pressed and piston moving AWAY from switch
        // The output of the motor will be positive.
        
        //Note: When limit switch is pressed, the switch value becomes zero
        if ((_limitSwitch.read() == 0) && (_pid.getOutput()< 0)) {
            //dont run
            return;
        }
        
        // clamp the PID output to the motor to -1.0 to 1.0
        if (_pid.getOutput() > 1.0)
            _motor.run(1.0);
        else if (_pid.getOutput() < -1.0)
            _motor.run(-1.0);
        else 
            _motor.run(_pid.getOutput());
    }
}

// using main loop ticker (attached at 0.01 intervals)
void LinearActuator::runLinearActuator() {
    _init = true;
}
 
void LinearActuator::pause() {
    //this allows the controller to keep running while turning off the motor output
    _motor.stop();
    //paused flag causes controller output not to be piped to the motor
    _paused = true;
    
    //new, reset the integrator
    _pid.resetPidLoop();
}
 
void LinearActuator::unpause() {
    //this resumes motor operation
    _paused = false;
}
 
void LinearActuator::refreshPVState() {
    //The position and velocity are basically filtered "counts" using the PVF
    _position = _filter.getPosition();
    _velocity = _filter.getVelocity();
 
    _position_mm = counts_to_dist(_position);
    _velocity_mms = counts_to_velocity(_velocity);
}
 
// setting and getting variables
void LinearActuator::setPosition_mm(float dist) {
    _SetPoint_mm = clamp<float>(dist, 0.0, _extendLimit);  //this is another spot that prevents the requested set point from going out of range, this template function is defined in the controller header file fyi
 
    _pid.writeSetPoint(_SetPoint_mm);
}

float LinearActuator::getSetPosition_mm() {
    return _SetPoint_mm;
}
 
float LinearActuator::getPosition_mm() {
    return _position_mm;
}
 
float LinearActuator::getPosition_counts() {
    return _position;
}
 
float LinearActuator::getVelocity_mms() {
    return _velocity_mms;
}
 
void LinearActuator::setControllerP(float P) {
    _Pgain = P;
    _pid.setPgain(_Pgain);
    return;
}
 
float LinearActuator::getControllerP() {
    return _Pgain;
}
 
void LinearActuator::setControllerI(float I) {
    _Igain = I;
    _pid.setIgain(_Igain);
    return;
}
 
float LinearActuator::getControllerI() {
    return _Igain;
}
 
void LinearActuator::setControllerD(float D) {
    _Dgain = D;
    _pid.setDgain(_Dgain);
    return;
}
 
float LinearActuator::getControllerD() {
    return _Dgain;
}
 
float LinearActuator::getOutput() {
    return _pid.getOutput();
}

void LinearActuator::setZeroCounts(int zero) {
    _zeroCounts = clamp<int>(zero, 0, 4096);
    return;
}
 
int LinearActuator::getZeroCounts() {
    return _zeroCounts;    
}
 
void LinearActuator::setTravelLimit(float limit) {
    _extendLimit = limit;
    return;
}
 
float LinearActuator::getTravelLimit() {
    return _extendLimit;   
}
 
void LinearActuator::setPotSlope(float slope) {
    _slope = slope;
    return;
}
 
float LinearActuator::getPotSlope() {
    return _slope;    
}

//////////////////////////////////////////////////////////////////////////////// 
float LinearActuator::counts_to_dist(int count) {
    float conv = _slope*(count-_zeroCounts);   
    return conv;
}
////////////////////////////////////////////////////////////////////////////////
 
void LinearActuator::setFilterFrequency(float frequency) {
    _filterFrequency = frequency;
    _filter.writeWn(frequency);   
}

float LinearActuator::getFilterFrequency() {
    return _filterFrequency;               //new 7/11/18
}
 
int LinearActuator::dist_to_counts(float dist) {
    float conv = (dist/_slope)+_zeroCounts;
    return (int) conv;
}
 
float LinearActuator::counts_to_velocity(int count) {
    float conv = count*_slope;      //Does this make sense? 01/08/19 
    return conv;                    //Furthermore does it make sense to use a filtered velocity count?
}

//Stop motor immediately when limit switch pressed.
void LinearActuator::switchPressed() {
    _motor.stop();
}
 
void LinearActuator::homePiston() {
    //system is already active, input readings should be valid
    
    // This sends the motor on a kamakaze mission toward the limit switch
    // The interrupt should catch and stop it, and the piston is now at home
    // position
    
    //unpause the motor (activate it)
    unpause();
    
    _motor.run(-0.5);
    
    xbee().printf("HOMING SEQUENCE ENGAGED. Press \"X\" to exit!\n\r");
    
    while (1) {
        //trap the program here while we wait for the limit switch to be triggered
        //when it does, the limit interrupt will stop the motors
        //if (_limit) {
            
        //unnecessarily convoluted before: 12/12/2018 (reading closed switch that is N.O.)
        if (_limitSwitch.read() == 0) {
            xbee().printf("\r\nHit limit switch\r\n");
            //the switch has been pressed
            if (abs(_filter.getVelocity()) < 0.1) {
                //this is here to make sure the adc filter is not jittering around
                //we are probably stable enough to take a zero here
                
                _zeroCounts = _filter.getPosition() + 20; //get position of motors
                
                //Added 50 counts for some margin of error
                
                // This can be used for troubleshooting
                xbee().printf("\n\rzero_counts: %4i     \n\r" , _zeroCounts);
                
                //pause the motor (deactivate it)
                pause();
                
                break;  //end while loop
            }
        } //end of limit switch if statement
        
        if (xbee().readable()) {
            char user_input = xbee().getc();
            
            if (user_input == 'x' or user_input == 'X') {
                xbee().printf("EXIT! HOMING NOT COMPLETE!\n\r");
                break;  //end while loop
            }
            
            else if (user_input == 'c' or user_input == 'C') {
                xbee().printf("Current counts: %d\n\r", _filter.getPosition());
            }
        }
    }
}
 
bool LinearActuator::getHardwareSwitchStatus() {
    return _limitSwitch.read();
}
 
 void LinearActuator::setDeadband(float deadband) {
    _deadband = deadband;
    _pid.setDeadBand(_deadband);
    return;    
}

float LinearActuator::getDeadband() {
    return _deadband;
}
 
bool LinearActuator::toggleDeadband(bool toggle) {
    _pid.toggleDeadBand(toggle);
    return toggle;
}

void LinearActuator::setPIDHighLimit(float high_limit) {
    _pid_high_limit = high_limit;
    _pid.setHiLimit(_pid_high_limit);
}

void LinearActuator::setPIDLowLimit(float low_limit) {
    _pid_low_limit = low_limit;
    _pid.setLoLimit(_pid_low_limit);                   //default at zero, or the switch retracted
}

//need so see some PID parameter values
float LinearActuator::getPIDErrorTerm() {
    return _pid.getErrorTerm();
}
float LinearActuator::getPIDIntegralTerm(){
    return _pid.getIntegralTerm();
}
float LinearActuator::getPIDDerivativeTerm(){
    return _pid.getDerivativeTerm();
}