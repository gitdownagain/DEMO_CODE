/*******************************************************************************
Author:           Troy Holley
Title:            SpiAdc.cpp
Date:             12/19/2018

Description/Notes:

Handles the off-board 8-channel analog to digital converter MCP3208.

Channel readings correspond to the 12-bit voltage readings, range 0 to 4095.

*******************************************************************************/

#include "SpiAdc.hpp"

SpiADC::SpiADC(PinName mosi, PinName miso, PinName sclk, PinName csel, PinName led) :
    _spi(mosi, miso, sclk), // mosi, miso, sclk
    adcLed(led), // status led
    cs(csel) // chip select
{
}

void SpiADC::initialize() {
    //set up the spi bus and frequency
    _spi.format(12,0);
    _spi.frequency(1000000);
 
    //chip select high puts ADC in standby
    cs = 1;
    
    //zero the initial ch0 and ch1 oversampled readings    
    ch0_filt = 0;
    ch1_filt = 0;
    ch2_filt = 0;
    ch3_filt = 0;
    ch4_filt = 0;
    ch5_filt = 0;
    ch6_filt = 0;
    ch7_filt = 0;

    //led on to say hello
    adcLed = 0; //debug turned off
}

// start an interupt driven trigger of the external ADC
void SpiADC::start() {
    interval.attach_us(this, &SpiADC::update, 10000);  //this should be a 100 Hz sample rate
}

// stop the interupt driven trigger
void SpiADC::stop() {
    interval.detach();
}

void SpiADC::update() {
    //flash the LED
    //adcLed = !adcLed;

    //chip select low starts data conversion
    cs = 0;
       
    //the next thing is the input data word
    //it is 4 bits and looks like this
    // | start | single/diff | odd/sign | MSB first/LSB first |
    // if you want single ended on channel 0 MSB first then input 0xD
    // if you want single ended on channel 1 MSB first then input 0xF

    // NOTE: The SPI write command writes to the SPI Slave AND returns the response

    // get channel 0
    unsigned int byte = _spi.write((0x18)<<2);
    //send a dummy byte to receive the data
    unsigned int byte1 = _spi.write(0x0);
    ch0_raw = byte1;
    ch0_filt  += (ch0_raw - ch0_filt)/CH0OVERSAMPLE;

    cs = 1;
    cs = 0;

    byte = _spi.write((0x19)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch1_raw = byte1;
    ch1_filt += (ch1_raw - ch1_filt)/CH1OVERSAMPLE;

    //switch chip select back to high
    cs = 1;
    cs = 0;

    byte = _spi.write((0x1A)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch2_raw = byte1;
    ch2_filt += (ch2_raw - ch2_filt)/CH2OVERSAMPLE;

    //switch chip select back to high
    cs = 1;
    cs = 0;

    byte = _spi.write((0x1B)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch3_raw = byte1;
    ch3_filt += (ch3_raw - ch3_filt)/CH3OVERSAMPLE;

    //switch chip select back to high
    cs = 1;

    cs = 0;

    byte = _spi.write((0x1C)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch4_raw = byte1;
    ch4_filt += (ch4_raw - ch4_filt)/CH4OVERSAMPLE;

    //switch chip select back to high
    cs = 1;

    cs = 0;

    byte = _spi.write((0x1D)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch5_raw = byte1;
    ch5_filt += (ch5_raw - ch5_filt)/CH5OVERSAMPLE;

    //switch chip select back to high
    cs = 1;
    
    cs = 0;

    byte = _spi.write((0x1E)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch6_raw = byte1;
    ch6_filt += (ch6_raw - ch6_filt)/CH6OVERSAMPLE;

    //switch chip select back to high
    cs = 1;
    
    cs = 0;

    byte = _spi.write((0x1F)<<2);
    //send a dummy byte to receive the data
    byte1 = _spi.write(0x0);
    ch7_raw = byte1;
    ch7_filt += (ch7_raw - ch7_filt)/CH7OVERSAMPLE;

    //switch chip select back to high
    cs = 1;

    return ;
}

int SpiADC::readCh0() {
    return ch0_filt;
}

int SpiADC::readCh1() {
    return ch1_filt;
}

int SpiADC::readCh2() {
    return ch2_filt;
}

int SpiADC::readCh3() {
    return ch3_filt;
}

int SpiADC::readCh4() {
    return ch4_filt;
}

int SpiADC::readCh5() {
    return ch5_filt;
}

int SpiADC::readCh6() {
    return ch6_filt;
}

int SpiADC::readCh7() {
    return ch7_filt;
}

int SpiADC::readRawCh0() {
    return ch0_raw;
}

int SpiADC::readRawCh1() {
    return ch1_raw;
}

int SpiADC::readRawCh2() {
    return ch2_raw;
}

int SpiADC::readRawCh3() {
    return ch3_raw;
}

int SpiADC::readRawCh4() {
    return ch4_raw;
}

int SpiADC::readRawCh5() {
    return ch5_raw;
}

int SpiADC::readRawCh6() {
    return ch6_raw;
}

int SpiADC::readRawCh7() {
    return ch7_raw;
}
