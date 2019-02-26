/*******************************************************************************
Author:           Troy Holley
Title:            Sensors.cpp
Date:             11/01/2018 (last modified)

Description/Notes:

Newer sensors were added here including the internal pressure transducer, 
PCB current sensor, PCB voltage sensor, and altimeter.

The altimeter needs to be checked because the limited documentation does not 
clearly explain if the voltage reading corresponds to 100 meters or 200 meters
or what specific depth is expected.

*******************************************************************************/

#include "Sensors.hpp"
#include "StaticDefs.hpp"

Sensors::Sensors() {
    //_reference_voltage = 5.0;   //check this against actual v_ref
    _reference_voltage = 3.3;   //check this against actual v_ref 01/15/19
}  
  
    // extrapolated from graph if V_s = 5.0
    // https://www.nxp.com/docs/en/data-sheet/MPXA6115A.pdf   
float Sensors::getInternalPressurePSI() {
    return ( ( 22.029 * ( _reference_voltage * adc().readCh5() / 4095.0 ) + 10.884 ) * 0.145038 ); // Press_Xducer (on-board)
}

float Sensors::getVoltageInput() {
    return ( adc().readCh6() / 4095.0 * _reference_voltage * 11.0 );
}

float Sensors::getCurrentInput() {
    return ( adc().readCh7() / 4095.0 * _reference_voltage );
}

//currently using BCE CS line for this data
float Sensors::getAltimeterChannelReadings() {
    return adc().readCh2();             //channel 2 (third channel) from the schematic
}

float Sensors::getBceCurrent() {
    //pololu reads 2.5 volts at zero current so it can do +/- 30 amps current reading
    return (((adc().readCh2()/4095.0) * _reference_voltage) - 2.5);
}

float Sensors::getBmmCurrent() {
    //pololu reads 2.5 volts at zero current so it can do +/- 30 amps current reading
    return (((adc().readCh3()/4095.0) * _reference_voltage) - 2.5);
}

// channel readings based on PCB 2.4