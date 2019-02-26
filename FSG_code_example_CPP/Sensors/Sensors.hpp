#ifndef SENSORS_HPP
#define SENSORS_HPP
 
#include "mbed.h"

class Sensors {
public:
    Sensors();
    
    float getInternalPressurePSI();
    float getVoltageInput();
    float getCurrentInput();
    float getAltimeterChannelReadings();
    float getBceCurrent();
    float getBmmCurrent();

private:
    float _reference_voltage;
};
 
#endif /* GUI_HPP */