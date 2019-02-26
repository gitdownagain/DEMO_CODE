#ifndef OMEGAPX209_HPP
#define OMEGAPX209_HPP

/*
This class wraps an Omega pressure transducer.
Author: Matthew, October 24th, 2013
Modified: Dan, 2017-10-30
*/

#include "mbed.h"

#define water_density_kg_m3 1000    // fresh water density [kg/m^3] (1029 for sea water)
#define grav_m_s2 9.80665           // gravitational constant [m/s^2]

#define m2ft 3.28084                // convert m to ft
#define psi2Pa 6894.76              // convert psi to Pa

#define OVERSAMPLE 20               // number of oversamples            //try a 100 maybe according to stearns?

class omegaPX209 {
public:
    omegaPX209(PinName pin);
    
    //new array in here
    int _adc_array[20];
    
    void init();
    void tare();                   // tares reading to ambient pressure

    float getPsi();                 // returns pressure [psi]
    float getDepthFt();             // returns water depth [ft]
    
    float newGetPsi();
    float newGetDepthFt();

    void setZero(float zeroPsi);    // lets user set a different ambient pressure [psi] ... tare, effectively
    float getZero();                // returns the internal ambient pressure [psi]
    
    int readADCCounts();          //06/07/2018 check the outputs
    float readVoltage();            //voltage reading?
    float getRawPSI();
    float getZeroPSI();             //need to check this number

private:
    AnalogIn _adc;

    float _psi;                     // pressure [psi]
    float _zeroPsi;                 // atmospheric pressure at sea level [psi]
    float _adcVoltage;              // voltage of mbed ADC system [V]
    float _fullscale;               // maximum pressure of the sensor [psi]
    float _psi_per_volt_cal;                     // psi per volt calibration [psi/V]
    float _PSI_reading;
    
    float _adc_multiplier;          //using one constant for doing the adc math
};

#endif