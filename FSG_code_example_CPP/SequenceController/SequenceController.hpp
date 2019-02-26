#include "mbed.h"
#include "ConfigFile.h"
#include <string>            //mbed doesn't use cstring

#ifndef SEQUENCECONTROLLER_HPP
#define SEQUENCECONTROLLER_HPP

//struct for saving the data
struct sequenceStruct {
    string title;
    int state;      //for the current StateMachine, states are ID-ed with enumeration
    float timeout;
    float depth;
    float pitch;
};

class SequenceController {
public:
    SequenceController();
    
    sequenceStruct sequenceStructLoaded[256];
    
    void loadSequence();
    
    sequenceStruct process(string input_string);
    
    void sequenceFunction();    //process the sequence
    
    void setState(int input_state);           //manually set the state of the system
    int getState();
    void setTimeout(float input_timeout);
    void setDepthCommand(float input_depth_command);
    void setPitchCommand(float input_pitch_command);
    
    int getSequenceState();
    
private:
    char _neutral_sequence;
    char _dive_cycle;
    char _exit;
    
    int _number_of_sequences;
    int _sequence_counter;
    
    Ticker sequenceTicker;          //Ticker for the sequenceFunction
    
    int _current_state;
};
#endif
