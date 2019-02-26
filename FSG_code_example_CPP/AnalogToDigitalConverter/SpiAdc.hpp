#ifndef SPIADC_H
#define SPIADC_H
#include "mbed.h"

#define CH0OVERSAMPLE 10
#define CH1OVERSAMPLE 10
#define CH2OVERSAMPLE 10
#define CH3OVERSAMPLE 10
#define CH4OVERSAMPLE 10
#define CH5OVERSAMPLE 10
#define CH6OVERSAMPLE 10
#define CH7OVERSAMPLE 10

class SpiADC{
public:
    SpiADC(PinName mosi, PinName miso, PinName sclk, PinName csel, PinName led);
    void initialize();
    void update();
    void start();
    void stop();
    
    int readCh0();
    int readCh1();
    int readCh2();
    int readCh3();
    int readCh4();
    int readCh5();
    int readCh6();
    int readCh7();
    
    int readRawCh0();
    int readRawCh1();
    int readRawCh2();
    int readRawCh3();
    int readRawCh4();
    int readRawCh5();
    int readRawCh6();
    int readRawCh7();
    
protected:
    SPI _spi;
    Ticker interval;
    DigitalOut adcLed;
    DigitalOut cs;

    int ch0_raw;
    int ch1_raw;
    int ch2_raw;
    int ch3_raw;
    int ch4_raw;
    int ch5_raw;
    int ch6_raw;
    int ch7_raw;
    
    int ch0_filt;
    int ch1_filt;
    int ch2_filt;
    int ch3_filt;
    int ch4_filt;
    int ch5_filt;
    int ch6_filt;
    int ch7_filt;
};



#endif 