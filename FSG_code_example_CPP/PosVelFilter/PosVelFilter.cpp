/*******************************************************************************
Author:           Kahn?
Title:            PosVelFilter.cpp
Date:             11/28/2017 (last modified)

Description/Notes:

Position and velocity filter used to reduce noise on incoming signals.

Not clear if this is required for the outer loops because of the slower reaction
time of this vehicle.  Benefits of filtering requires more testing.

*******************************************************************************/

#include "PosVelFilter.hpp"

PosVelFilter::PosVelFilter() {
    x1 = 0; // pseudo position state
    x2 = 0; // pseudo velocity state
    
    w_n = 1.0; // natural frequency of the filter bigger increases frequency response
}

//run the pos-vel estimate filter
void PosVelFilter::update(float deltaT, float counts) {
    dt = deltaT;

    x1_dot = x2;
    x2_dot = (-2.0*w_n*x2) - (w_n*w_n)*x1 + (w_n*w_n)*counts;

    position = x1;
    velocity = x2;

    x1 += x1_dot*dt;
    x2 += x2_dot*dt;
}

float PosVelFilter::getPosition() {
    return position;
}

float PosVelFilter::getVelocity() {
    return velocity;
}

float PosVelFilter::getDt() {
    return dt;
}

void PosVelFilter::writeWn(float wn) {
    w_n = wn;
}