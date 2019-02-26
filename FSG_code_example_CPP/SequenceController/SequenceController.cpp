/*******************************************************************************
Author:           Troy Holley
Title:            SequenceController.cpp
Date:             11/28/2017 (last modified)

Description/Notes:

Sequence controller loads a sequence.txt file that controls how the vehicle
dives in a multi-dive sequence.  I.E. tell the vehicle to dive for 60 seconds to
a depth of 50 meters at a pitch angle of 30 degrees.

*******************************************************************************/  

#include "SequenceController.hpp"
#include "StaticDefs.hpp"

SequenceController::SequenceController() {
    _sequence_counter = 0;
}

void SequenceController::loadSequence() {
    xbee().printf("\n\rLoading Dive Sequence File:");
    
    ConfigFile read_sequence_cfg;
    char value[256];   
    
    //read configuration file stored on MBED
    if (!read_sequence_cfg.read("/local/sequence.txt")) {
        xbee().printf("\n\rERROR:Failure to read sequence.txt file.");
    }
    else {        
        /* Read values from the file until you reach an "exit" character" */
        //up to 256 items in the sequence
        for (int i = 0; i < 256; i++) {                  //works
            /* convert INT to string */
            char buf[256];
            sprintf(buf, "%d", i);                  //searching for 0,1,2,3...
            /* convert INT to string */
        
            if (read_sequence_cfg.getValue(buf, &value[0], sizeof(value))) {
                xbee().printf("\n\rsequence %d = %s",i,value);
                
                sequenceStructLoaded[i] = process(value); //create the structs using process(string randomstring)
            }
            
            if (sequenceStructLoaded[i].title == "exit") {
                _number_of_sequences = i;   //before the exit
                break;
            }
        }
    }   //end of successful read
}

sequenceStruct SequenceController::process(string randomstring) {
    //this is the struct that is loaded from the config variables
    
    sequenceStruct loadStruct; //local struct
    
    /* CONVERT STRING TO CHAR ARRAY */
    const char *cstr = randomstring.c_str();
    /* CONVERT STRING TO CHAR ARRAY */
    
    /* DIVE */
    //this can only be in the first position
    if ((signed int) randomstring.find("dive") != -1) {
        loadStruct.title = "dive";
        loadStruct.state = MULTI_DIVE;      //NEW: separate state handles multiple dives
    }
    /* DIVE */
    
    /* PITCH */
    if ((signed int) randomstring.find("neutral") != -1) {
        loadStruct.title = "neutral";
        xbee().printf("\n\rLOAD neutral. %d", randomstring.find("neutral"));
        loadStruct.state = FIND_NEUTRAL;
    }
    /* PITCH */
    
    /* EXIT */
    if ((signed int) randomstring.find("exit") != -1) {
        loadStruct.title = "exit";
        xbee().printf("\n\rReminder. Exit command is state FLOAT_BROADCAST\n\r");
        loadStruct.state = FLOAT_BROADCAST; //this is the new exit condition of the dive-rise sequence (11/4/17)
    }
    /* EXIT */
    
    /* DEPTH TO FLOAT */
    if ((signed int) randomstring.find("depth") != -1) {
        if (randomstring.find("neutral") || randomstring.find("dive")) {
            int depth_pos = randomstring.find("depth") + 6;     //11 in example literally "depth="
            char depth_array[256] = {0};    //clear memory
            int depth_counter = 0;
            for (int i = depth_pos; i < randomstring.length(); i++) {
                if (cstr[i] == ',') 
                    break;
                else if (cstr[i] == ';') 
                    break;
                else {
                    depth_array[depth_counter] = cstr[i]; 
                    depth_counter++;
                }
            }
            loadStruct.depth = atof(depth_array);
        }
    }    
    /* DEPTH TO FLOAT */
    
    /* PITCH TO FLOAT */
    if ((signed int) randomstring.find("pitch") != -1) {
        if (randomstring.find("neutral") || randomstring.find("dive")) {
            int pitch_pos = randomstring.find("pitch") + 6;     //11 in example
            char pitch_array[256] = {0};    //clear memory
            int pitch_counter = 0;
            for (int i = pitch_pos; i < randomstring.length(); i++) {
                if (cstr[i] == ',') 
                    break;
                else if (cstr[i] == ';') 
                    break;
                else {
                    pitch_array[pitch_counter] = cstr[i]; 
                    pitch_counter++;
                }
            }
            loadStruct.pitch = atof(pitch_array);
        }
    }
    /* PITCH TO FLOAT */
    
    /* PAUSE */
    if ((signed int) randomstring.find("pause") != -1) {
        loadStruct.title = "pause";
    }
    /* PAUSE */
    
    /* TIME TO FLOAT */
    if ((signed int) randomstring.find("timeout") != -1) { 
           
        int time_pos = randomstring.find("timeout") + 8;    //position of timeout + "timeout=" so 8
        char time_array[256] = {0};
        int time_counter = 0;
        for (int i = time_pos; i < randomstring.length(); i++) {
            //xbee().printf("time string cstr[i] = %c\n\r", cstr[i]); //debug
            
            if (cstr[i] == ',') 
                break;
            else if (cstr[i] == ';') 
                break;
            else {
                //xbee().printf("time string cstr[i] = %c\n\r", cstr[i]); //debug
                time_array[time_counter] = cstr[i]; 
                time_counter++;
            }
        }
        loadStruct.timeout = atof(time_array);
    }
    /* TIME TO FLOAT */  
    
//    /* EXIT */
//    if (randomstring.find("exit") != 0) {
//        loadStruct.title = "exit";
//        xbee().printf("\n\rEXIT.");
//    }
//    /* EXIT */
    
    return loadStruct;  //each iteration this returns a completed struct
}

void SequenceController::sequenceFunction() {
    //xbee().printf("sequenceFunction\n\r");    //debug (verified it is working correctly)
    
    int check_current_state = stateMachine().getState();
    xbee().printf("State Machine State: %d\n\r", check_current_state);
        
    if (stateMachine().getState() == SIT_IDLE) {
        //system starts idle
        //set the state machine to the current sequence in the array
        //example, set to "dive" and set pitch and depth and timeout
        
        _current_state = sequenceStructLoaded[_sequence_counter].state;
        xbee().printf("_current_state: %d\n\r", _current_state);
        xbee().printf("_sequence_counter: %d\n\r", _sequence_counter);
        xbee().printf("_number_of_sequences: %d\n\r", _number_of_sequences);
        
        stateMachine().setState(_current_state);
        stateMachine().setDepthCommand(sequenceStructLoaded[_sequence_counter].depth);
        stateMachine().setPitchCommand(sequenceStructLoaded[_sequence_counter].pitch);
        stateMachine().setTimeout(sequenceStructLoaded[_sequence_counter].timeout);
    
        if (_sequence_counter == _number_of_sequences-1)    //end when you finish all of the sequences       
            sequenceTicker.detach();
            
        _sequence_counter++;        //exit ticker when counter complete
    }   
}

int SequenceController::getSequenceState() {
    return _current_state;
}