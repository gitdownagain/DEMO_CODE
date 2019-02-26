#ifndef SERVODRIVER_HPP
#define SERVODRIVER_HPP
 
#include "mbed.h"
 
class ServoDriver {
public:
    ServoDriver(PinName pwm);
    
    void endPoints(float lower_endpoint, float upper_endpoint);
    void neutralPosition(float input_neutral_position);
    void setPosition_deg(float input_position);
    
    void setPWM(float input_pwm);     // 8/2/2018
    
    float slope();

    void setMinPWM(float pwm_input);
    void setMaxPWM(float pwm_input);
    void setCenterPWM(float pwm_input);
    void setMinDeg(float deg);
    void setMaxDeg(float deg);
    
    float getSetPosition_deg();
    float getSetPosition_pwm();
    
    void init();
    
    void pwm_pulse_off();
    void pwm_pulse_on();
    void pause();
    void unpause();
    
    void runServo();
    
    float getMinPWM();
    float getMaxPWM();
    float getCenterPWM();
    float getMinDeg();
    float getMaxDeg();
    
private:
    float _min_pwm;
    float _max_pwm;
    float _center_pwm;
    float _min_deg;
    float _max_deg;
    
    volatile bool _paused;
    
    float _degrees_set_position;
    
    volatile unsigned int _valid_servo_position_pwm;      //signal in microseconds, for example 1580 microseconds (close to center of servo)
    volatile unsigned int _period_cnt;
    
    Ticker pwm_pulse_on_ticker;
    Ticker pwm_pulse_off_ticker;
    Timeout pwm_pulse_off_timeout;
    
    DigitalOut _pwm_out;    //cannot be volatile
};

template <typename T>
T servoClamp(T value, T min, T max)
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