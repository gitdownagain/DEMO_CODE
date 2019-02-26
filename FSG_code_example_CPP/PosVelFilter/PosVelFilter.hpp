#ifndef POSVELFILTER_H
#define POSVELFILTER_H

#include "mbed.h"

class PosVelFilter
{
public:
    PosVelFilter();
    
    void update(float deltaT, float counts);
    
    void init();
    
    float getPosition();
    float getVelocity();
    float getDt();
    
    void writeWn(float wn);
    
protected:
    float x1;
    float x2;
    float x2_dot;
    float x1_dot;
    float w_n; 
    
    float dt;
    float position;
    float velocity;
};

#endif