#ifndef LINEARACTUATOR_HPP
#define LINEARACTUATOR_HPP
 
#include "mbed.h"
#include "PololuHBridge.hpp"
#include "PidController.hpp"
#include "PosVelFilter.hpp"
 
//Dependencies
//This Class requires adc readings to sense the position of the piston
//This is a resource that ends up being shared among other classes in the vehicle
//for this reason it makes sense for it to be its own entity that is started in
//the main line code
 
class LinearActuator {
public:
    LinearActuator(float interval, PinName pwm, PinName dir, PinName reset, PinName limit, int adc_ch);
    
    // functions for setting up
    void init();
    void update();
    // start and stop were removed, legacy functionality (one ticker runs clocks now)
    void pause();
    void unpause();  
    
    void runLinearActuator();       //new 03/12/2018
    
    void refreshPVState();
    
    // setting and getting variables
    void setPosition_mm(float dist);
    float getSetPosition_mm();
    
    float getPosition_mm();
    float getPosition_counts();
    float getVelocity_mms();
    
    void setControllerP(float P);
    float getControllerP();
    
    void setControllerI(float I);
    float getControllerI();
    
    void setControllerD(float D);
    float getControllerD();
    
    void setZeroCounts(int zero);
    int getZeroCounts();
    
    void setTravelLimit(float limit);
    float getTravelLimit();
    
    void setPotSlope(float slope);
    float getPotSlope();
    
    void homePiston();
    bool getSwitch();       //get state of limit switch
    
    float getOutput();
    
    void setFilterFrequency(float frequency);
    float getFilterFrequency(); // 7/11/18
    
    void setDeadband(float deadband);
    float getDeadband();
    bool toggleDeadband(bool toggle);
    
    void setPIDHighLimit(float high_limit);
    void setPIDLowLimit(float low_limit);
    
    bool getHardwareSwitchStatus(); //new
    
    //new for debugging integral problems 12/18/2018
    //need so see some PID parameter values
    float getPIDErrorTerm();
    float getPIDIntegralTerm();
    float getPIDDerivativeTerm();
    
protected:
    PololuHBridge _motor;
    PosVelFilter _filter;
    PIDController _pid;
    Ticker _pulse;
    InterruptIn _limitSwitch;
    
    void switchPressed();               //checking functionality
    void _switchReleased();
    void _calculateSensorSlope();
    
    bool _init;
    bool _paused;
    
    int _adc_channel;
    
    float _filterFrequency;
    
    float _dt;
    
    float _SetPoint_mm;
    
    // position and velocity in counts (PVF runs on counts)
    float _position;
    float _velocity;

    // position and velocity converted to mm and mm/s
    float _position_mm;
    float _velocity_mms;
    
    // linear actuator servo PID gains
    float _Pgain;
    float _Igain;
    float _Dgain;
    
    float _deadband;
    
    int _zeroCounts; //gets assigned by homing function. can also be stored in config
    float _extendLimit; //config variable, limits the extension of the piston, this is same as datum for normal operation,
    
    float _slope;
    int dist_to_counts(float dist);
    float counts_to_dist(int count);
    float counts_to_velocity(int count);
    
    float _pid_high_limit;
    float _pid_low_limit;
};


template <typename T>
T clamp(T value, T min, T max)
{
    if(value < min) {
        return min;
    } else if(value > max) {
        return max;
    } else {
        return value;
    }
};

#endif