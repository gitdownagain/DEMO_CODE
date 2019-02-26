/*******************************************************************************
Author:           Troy Holley
Title:            omegaPX209.cpp
Date:             01/08/2018

Description/Notes:

Reads the Omega pressure transducer.

Modified from Kelly/Newton code.

Fixed some errors, included the header guards, adjusted for 3.3v ADC.

*******************************************************************************/

#include "mbed.h"
#include "omegaPX209.hpp"
#include "StaticDefs.hpp"       //pins and other hardware (new)

//print to both serial ports using this macro
#define serialPrint(fmt, ...) pc().printf(fmt, ##__VA_ARGS__);xbee().printf(fmt, ##__VA_ARGS__)

omegaPX209::omegaPX209(PinName pin): _adc(pin){    
    _psi = 14.7;                    // pressure [psi]
    _zeroPsi = 14.7;                // atmospheric pressure at sea level [psi]
    _adcVoltage = 3.3;              // Troy: I'm not sure this is the right name for this multiplier... [V] (was 3.3 before)
    //_fullscale = 50;                // value of sensor at full scale (*confirm with Stearns*) [psi]
    _fullscale = 500;
    _psi_per_volt_cal = _fullscale/5.0;          // psi per volt calibration [psi/V]
    _PSI_reading = 0;
    
    //add constant for multiplying against the adc counts reading
    // 3.3 volts * (500 psi / 5.0 volts) / 4095 counts = psi / counts
    //_adc_multiplier = 4.99 * (500/5.0) / 4095.0;
    _adc_multiplier = 3.3 * (500/5.0) / 4095.0;
    
    
    //initialize this array with zeroes, try different methods later
    for (int a = 0; a < 20; a++) {
        _adc_array[a] = 0;
    } 
}

// nothing to initialize, but you can call this function if it makes you feel better.
void omegaPX209::init() {
}

// lets user set a different ambient pressure [psi]
void omegaPX209::setZero(float zeroPsi) {
    _zeroPsi = zeroPsi;    
}

// returns the internal ambient pressure [psi]
float omegaPX209::getZero() {
    return _zeroPsi;
}

// reads from ADC system and does math for converting to psi
float omegaPX209::getPsi() {
    // filter by over-sampling
    float psi_sum_in_counts = 0;
    
    _PSI_reading = 0;   //reset each time
    
    //take 20 samples to average below
    for (int i = 0; i < OVERSAMPLE; i++) {         // Oversample = 20
        psi_sum_in_counts += adc().readCh4();
    }   
    
    // use over-sampled psi reading
    _psi =  (psi_sum_in_counts / OVERSAMPLE) * _adc_multiplier;   // check this math!!!

    return _psi;
}

float omegaPX209::newGetPsi() {
    //array is constantly updated each time this is called
    float adc_array_average = 0;

    //get a new PSI reading
    float new_psi_reading = adc().readCh4();
    
    //each time you read the pressure transducer, put that value into a shifting array (almost like queue)
    for(int i = 19; i >= 0; i --) {
        _adc_array[i+1] = _adc_array[i]; //move all elements to the right (20th element gone)
    }
    
    //add the new reading to the beginning of the array
    _adc_array[0] = new_psi_reading;
    
    //do the math
    //serialPrint("ARRAY: ");
    
    for (int x = 0; x < 20; x++) {
        //serialPrint("%d, ", _adc_array[x]); //DEBUG
        adc_array_average += _adc_array[x];
    }
    
    //serialPrint("\n");
    
    adc_array_average = adc_array_average/20;   //sum / oversample of 20
    
    //NEW PSI VALUE
    return adc_array_average * _adc_multiplier;   
}

// ch1_filt = ch1_filt + (ch1_raw - ch1_filt)/CH1OVERSAMPLE;

// reads the ADC system and returns depth in feet

#define m2ft 3.28084                // convert m to ft
#define psi2Pa 6894.76              // convert psi to Pa

float omegaPX209::getDepthFt() {
    float psi = getPsi() - _zeroPsi; // read the sensor and remove atmospheric bias
    float Pa = psi2Pa * psi; // convert psi to Pascals
    float depth_m = Pa / (water_density_kg_m3 * grav_m_s2); // convert Pa to fluid depth in meters
    float depth_ft = m2ft * depth_m; // convert meters to feet

    return depth_ft;
}

float omegaPX209::newGetDepthFt() {
    float psi = newGetPsi() - _zeroPsi; // read the sensor and remove atmospheric bias
    float Pa = psi2Pa * psi; // convert psi to Pascals
    float depth_m = Pa / (water_density_kg_m3 * grav_m_s2); // convert Pa to fluid depth in meters
    float depth_ft = m2ft * depth_m; // convert meters to feet

    return depth_ft;
}

// call this if you want to tare to zero
void omegaPX209::tare() {
    setZero(getPsi());
}

// 06/06/2018

int omegaPX209::readADCCounts() {
    //return _adc.read();
    return adc().readCh4();
}

float omegaPX209::readVoltage() {
    float pressure_voltage = adc().readCh4()/4095.0 * _adcVoltage;
    return pressure_voltage;
}

float omegaPX209::getRawPSI() {
    return _PSI_reading;
}

float omegaPX209::getZeroPSI() {
    return _zeroPsi;
}

// check depth calc via http://docs.bluerobotics.com/calc/pressure-depth/


//check scope on power supply and device (pressure transducer)


// QUEUE

// https://os.mbed.com/handbook/RTOS#queue

// https://os.mbed.com/users/dreschpe/code/mbed-rtos/docs/9780f8334bf5/structosEvent.html
