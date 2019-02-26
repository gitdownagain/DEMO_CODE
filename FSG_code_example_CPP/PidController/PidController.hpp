#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "mbed.h"

class PIDController {
public:
    PIDController();
    
    void newUpdate(float position, float dt);
    void update(float position, float velocity, float dt);
    float getOutput();
    
    //new 12/18/2018
    float getErrorTerm();
    float getIntegralTerm();
    float getDerivativeTerm();
    
    void setPgain(float gain);
    void setIgain(float gain);
    void setDgain(float gain);
    
    void writeSetPoint(float cmd);
    
    void setHiLimit(float high_limit);
    void setLoLimit(float low_limit);
    
    void toggleDeadBand(bool toggle);
    void setDeadBand(float deadband);
    
    void setHeadingFlag(bool heading_flag);
    
    //this is used to reset the errors to zero because nothing is moving
    void resetPidLoop();
    //every time you pause and stop the motor you need to reset the PID loop
    
protected:
    float _setPoint;
    float _error;
    float _integral;
    float _output;
    
    //new 01/08/19
    float _previous_error;
    float _derivative;

    float _Pgain;
    float _Dgain;
    float _Igain;
    float _hiLimit; //these parameters clamp the allowable output
    float _loLimit; //these parameters clamp the allowable output
    float _deadband;
    bool _deadbandFlag;
    bool _headingFlag;
    
    float _temp_velocity;
};

#endif