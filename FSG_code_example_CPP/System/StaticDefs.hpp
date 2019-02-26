#ifndef AUTOPILOTSTATICDEFS_H_
#define AUTOPILOTSTATICDEFS_H_

#include "mbed.h"
#include "MODSERIAL.h"
#include "SpiAdc.hpp"
#include "LinearActuator.hpp"
#include "IMU.h"
#include "omegaPX209.hpp"
#include "PosVelFilter.hpp"
#include "OuterLoop.hpp"
#include "StateMachine.hpp"

#include "ConfigFileIO.hpp"
#include "SequenceController.hpp"
#include "MbedLogger.hpp"
#include "SDFileSystem.h"
#include "ServoDriver.hpp"
#include "Gui.hpp"
#include "Sensors.hpp"

//Declare static global variables using 'construct on use' idiom to ensure they are always constructed correctly
// and avoid "static initialization order fiasco".

Timer                       &   systemTime();
Ticker                      &   pulse();

MODSERIAL                   &   pc();
MODSERIAL                   &   xbee();

LocalFileSystem             &   local();

SpiADC                      &   adc();
LinearActuator              &   bce();
LinearActuator              &   batt();

omegaPX209                  &   depth();
OuterLoop                   &   depthLoop();

IMU                         &   imu();
OuterLoop                   &   pitchLoop();

StateMachine                &   stateMachine();

MbedLogger                  &   mbedLogger();       //internal memory log files

//SDFileSystem                &   sd_card();          //SD card file system

Sensors                     &   sensors();

MbedLogger                  &   sdLogger();         //sd log files

ConfigFileIO                &   configFileIO();

SequenceController          &   sequenceController();

//servo driver
ServoDriver & rudder();     //new 06/06/2018
OuterLoop                   &   headingLoop();  //change 06/08/2018

// leds for debugging, global for use in any function
DigitalOut                  &   led1();
DigitalOut                  &   led2();
DigitalOut                  &   led3();
DigitalOut                  &   led4();

Gui                         &   gui();

#endif