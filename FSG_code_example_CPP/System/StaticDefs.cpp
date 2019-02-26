/*******************************************************************************
Author:           Troy Holley
Title:            StaticDefs.cpp
Date:             Modified 12/19/2018

Description/Notes:

This sets up the electronics, sensors, file system, etc.

This uses the "construct on use" idiom to ensure that all of the objects are 
constructed once, correctly, & avoids the "static initialization order fiasco".
*******************************************************************************/

#include "StaticDefs.hpp"

Timer & systemTime() {
    static Timer s;
    return s;
}

// need to remove the pulse, not used
Ticker & pulse() {
    static Ticker pulse;
    return pulse;   
}
    
MODSERIAL & pc() {
    static MODSERIAL pc(USBTX, USBRX);
    return pc;
}

MODSERIAL & xbee() {
    static MODSERIAL xb(p9, p10);        //XBee tx, rx pins
    //static MODSERIAL xb(USBTX, USBRX);        //XBee tx, rx pins
    return xb;
}

LocalFileSystem & local() {
    static LocalFileSystem local("local");
    return local;    
} 

//SDFileSystem & sd_card() {
//    static SDFileSystem sd_card(p11, p12, p13, p14, "sd");     //SDFileSystem sd_card(MOSI, MISO, SCK, CS, "sd");
//    return sd_card;
//}

SpiADC & adc() {
    static SpiADC adc(p5,p6,p7,p8,LED2);
    return adc;
}

LinearActuator & bce() {        // pwm,dir,res,swt
    static LinearActuator bce(0.01,p22,p15,p16,p17,0); //interval , pwm, dir, reset, limit switch, adc channel (confirmed)
    return bce;
}

LinearActuator & batt() {        // pwm,dir,res,swt
    static LinearActuator batt(0.01,p21,p20,p19,p18,1); //interval , pwm, dir, reset, limit switch, adc channel (confirmed)
    return batt;       
}

ServoDriver & rudder() {
    static ServoDriver rudder(p26);     //current rudder pin on the latest drawing 06/11/2018
    return rudder;
}
    
//*************Need to adjust class**************
omegaPX209 & depth() {
    static omegaPX209 depth(p19);   // pin
    return depth;
}

IMU & imu() {
    static IMU imu(p28, p27);       // tx, rx pin
    return imu;    
}

OuterLoop & depthLoop() {
    static OuterLoop depthLoop(0.1, 0); // interval, sensor type
    return depthLoop;
}

OuterLoop & pitchLoop() {
    static OuterLoop pitchLoop(0.1, 1); // interval, sensor type
    return pitchLoop;
}

OuterLoop & headingLoop() {
    static OuterLoop headingLoop(0.1, 2); // interval, sensor type
    return headingLoop;
}

StateMachine & stateMachine() {
    static StateMachine stateMachine;
    return stateMachine;
}

ConfigFileIO & configFileIO() {
    static ConfigFileIO configFileIO;
    return configFileIO;
}

SequenceController & sequenceController() {
    static SequenceController sequenceController;
    return sequenceController;
}

Sensors & sensors() {
    static Sensors sensors;
    return sensors;
}

MbedLogger & mbedLogger() {
    static MbedLogger mbedLogger("/local/");        //local file system
    return mbedLogger;
}

MbedLogger & sdLogger() {
    static MbedLogger sdLogger("/sd/");
    return sdLogger;
}

DigitalOut & led1() {
    static DigitalOut led1(LED1);
    return led1;
}

DigitalOut & led2() {
    static DigitalOut led2(LED2);
    return led2;
}

DigitalOut & led3() {
    static DigitalOut led3(LED3);
    return led3;
}

DigitalOut & led4() {
    static DigitalOut led4(LED4);
    return led4;
}

Gui & gui() {
    static Gui pythonGUI;
    return pythonGUI;
}