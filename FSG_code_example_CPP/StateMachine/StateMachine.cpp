/*******************************************************************************
Author:           Troy Holley
Title:            StateMachine.cpp
Date:             02/14/2019 (last modified)

Description/Notes:

Main access to hardware.  Control the different dive modes from here using a
finite state machine that locks hardware into a pre-defined state with a timeout
to prevent the vehicle from being lost below the water.

User can change PID loops and hardware limits in "debug menu."

User can also manually move the motors through the "manual tuning" menu.

There is still some debug code used to check the output from the PCB.

*******************************************************************************/


#include "StateMachine.hpp"
#include "StaticDefs.hpp"

//print to both serial ports using this macro
#define serialPrint(fmt, ...) pc().printf(fmt, ##__VA_ARGS__);xbee().printf(fmt, ##__VA_ARGS__)

 
StateMachine::StateMachine() {
    _timeout = 20;            // generic timeout for every state, seconds
    
    _pitchTolerance = 5.0;     // pitch angle tolerance for FLOAT_LEVEL state
 
    _bceFloatPosition = bce().getTravelLimit();      // bce position for "float" states                  (max travel limit for BCE is 320 mm)
    _battFloatPosition = batt().getTravelLimit();    // batt position tail high for "broadcast" state    (max travel limit for battery is 75 mm)
 
    _depth_command = 2.0;                        // user keyboard depth (default)
    _pitch_command = -20.0;                      // user keyboard pitch (default)
    _heading_command = 0.0;
    
    //new commands
    _BCE_dive_offset = 0.0;     //starting at the limits
    _BMM_dive_offset = 0.0;
    //new commands
    
    _neutral_timer        = 0;                  //timer used in FIND_NEUTRAL sub-FSM
////////////////////////////// 
    _state = SIT_IDLE;                          // select starting state here
    _isTimeoutRunning = false;                  // default timer to not running
    _isSubStateTimerRunning = false;            // default timer to not running
/////////////////////////////    
    _multi_dive_counter = 0;
    
    _depth_KP = depthLoop().getControllerP();  // load current depth value
    _depth_KI = depthLoop().getControllerI();  // load current depth value
    _depth_KD = depthLoop().getControllerD();  // load current depth value
    
    _pitch_KP = pitchLoop().getControllerP();  // load current pitch value
    _pitch_KI = pitchLoop().getControllerI();  // load current pitch value
    _pitch_KD = pitchLoop().getControllerD();  // load current pitch value
    
    _neutral_bce_pos_mm = depthLoop().getOutputOffset(); //load current neutral buoyancy position offset
    _neutral_batt_pos_mm = pitchLoop().getOutputOffset(); //load current neutral buoyancy position offset
    
    _state_array_counter = 1;                   //used to iterate through and record states
    _substate_array_counter = 0;                //used to iterate through and record substates

    _state_array[0] = SIT_IDLE;  //system starts in the SIT_IDLE state, record this
    
    _substate = NEUTRAL_SINKING;                //start sub-FSM in NEUTRAL_SINKING
    _previous_substate = -1;                    //to start sub-FSM
    _previous_state = -1;                       //for tracking FSM states
    
    _max_recorded_depth_neutral = -99;          //float to record max depth
    _max_recorded_depth_dive = -99;             //float to record max depth
    
    _max_recorded_auto_neutral_depth = -99;
    
    _debug_menu_on = false;                     //toggle between debug and simple menu screens
    
    //new file stuff
    _pitch_filter_freq = pitchLoop().getFilterFrequency();
    _pitch_deadband = pitchLoop().getDeadband();
    
    _depth_filter_freq = depthLoop().getFilterFrequency();
    _depth_deadband = depthLoop().getDeadband();
}
 
//Finite State Machine (FSM)
int StateMachine::runStateMachine() {
    // finite state machine ... each state has at least one exit criteria
    switch (_state) {
    case SIT_IDLE :
    case KEYBOARD :
        // there actually is no timeout for SIT_IDLE, but this enables some one-shot actions
        if (!_isTimeoutRunning) {
            //tare pressure sensor
            //depth().tare(); // tares to ambient (do on surface)   12/18/2018 removed
            
            if (_debug_menu_on)
                printDebugMenu();
            else
                printSimpleMenu();
            serialPrint("\r\n\nstate: SIT_IDLE\r\n");
            _isTimeoutRunning = true; 
 
            // what is active?
            bce().pause();
            batt().pause();
                     
            //reset sub FSM
            _isSubStateTimerRunning = false;
        }
        
        // how exit?
        keyboard(); // keyboard function will change the state if needed
        break;
        
    case CHECK_TUNING :                 // state used to check the tuning of the pressure vessel
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: CHECK_TUNING\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();    //this is now active
            batt().unpause();   //this is now active
 
            // what are the commands? (DRIVE THE MOTORS "DIRECTLY")
            bce().setPosition_mm(_neutral_bce_pos_mm);              //this variable is loaded from the file at initialization
            batt().setPosition_mm(_neutral_batt_pos_mm);            //this variable is loaded from the file at initialization
            
            // getSetPosition_mm is the commanded position in the LinearActuator class
            
            serialPrint("CHECK_TUNING: BCE cmd: %3.1f (BCE current position: %3.1f)\r\n", bce().getSetPosition_mm(), bce().getPosition_mm());
            serialPrint("CHECK_TUNING: BATT cmd: %3.1f (BATT current position: %3.1f)\r\n", batt().getSetPosition_mm(), bce().getPosition_mm());
        }
    
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("CHECK_TUNING: timed out!\r\n");
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        //WHAT IS ACTIVE?
        // the inner loop position controls are maintaining the positions of the linear actuators
        
        //print status to screen continuously
        serialPrint("CHECK_TUNING: BCE_position: %0.1f, BATT_position: %0.1f (BCE_cmd: %0.1f, BATT_cmd: %0.1f)(depth: %0.1f ft,pitch: %0.1f deg,heading: %0.1f)     [%0.1f sec]\r",bce().getPosition_mm(),batt().getPosition_mm(),bce().getSetPosition_mm(),batt().getSetPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition(),imu().getHeading(),_fsm_timer.read());
        
        break;
 
    case EMERGENCY_CLIMB :
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: EMERGENCY_CLIMB\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands?
            bce().setPosition_mm(bce().getTravelLimit());
            batt().setPosition_mm(10.0);    //pull nose up (0.0 was sketchy)    
        }
        
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("EC: timed out\r\n");
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        else if (depthLoop().getPosition() < 2.0) { //if the depth is greater than 0.2 feet, go to float broadcast
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        //WHAT IS ACTIVE?
        //print status to screen continuously
        serialPrint("EC: depth: %3.1f, pitch: %0.1f deg [BCE:%0.1f (cmd: %0.1f) BMM:%0.1f (cmd: %0.1f)] [%0.1f sec]\r",depthLoop().getPosition(),pitchLoop().getPosition(),bce().getPosition_mm(), bce().getSetPosition_mm(),batt().getPosition_mm(), batt().getSetPosition_mm(),_fsm_timer.read());
        
        break;
 
    case FIND_NEUTRAL :
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: FIND_NEUTRAL\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true;
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
            
            //start with a small offset from MANUAL neutral positions on the BCE only, BMM was pitching too much
            float bce_find_neutral_mm = _neutral_bce_pos_mm + 10.0;
            //float batt_find_neutral_mm = _neutral_batt_pos_mm + 10.0;
            
            bce().setPosition_mm(bce_find_neutral_mm);
            batt().setPosition_mm(_neutral_batt_pos_mm);    //set battery to the same neutral position
            
            //first iteration goes into Neutral Finding Sub-FSM 
            //set the first state of the FSM, and start the sub-FSM
            _substate = NEUTRAL_SINKING;        //first state in neutral sub-FSM is the pressure vessel sinking
            _previous_substate = -1;
            
            //save this state to the array
            _substate_array[_substate_array_counter] = NEUTRAL_SINKING;  //save to state array
            _substate_array_counter++;     
                   
            runNeutralStateMachine(); 
        }
 
        // how exit? (exit with the timer, if timer still running continue processing sub FSM)
        if (_fsm_timer > _timeout) {
            serialPrint("FN: timed out [time: %0.1f sec]\r\n", _fsm_timer.read());
            _state = EMERGENCY_CLIMB;         //new behavior (if this times out it emergency surfaces)
            _fsm_timer.reset();
            _isTimeoutRunning = false;
            
            //record this to the NEUTRAL sub-FSM tracker
            _substate_array[_substate_array_counter] = EMERGENCY_CLIMB;  //save to state array
            _substate_array_counter++;
        }
        
        //what is active? (neutral finding sub-function runs until completion)        
        //check if substate returned exit state, if so stop running the sub-FSM
        else if (runNeutralStateMachine() == NEUTRAL_EXIT) { 
            //if successful, FIND_NEUTRAL then goes to RISE
            serialPrint("*************************************** FIND_NEUTRAL sequence complete.  Rising.\r\n\n");
            _state = RISE;
            _isTimeoutRunning = false;
        }
        
        break;   
        
    case DIVE :
        // start local state timer and init any other one-shot actions
               
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: DIVE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands?
            depthLoop().setCommand(_depth_command);
            pitchLoop().setCommand(_pitch_command);
            
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            
            serialPrint("DIVE: depth cmd: %3.1f\r\n",depthLoop().getCommand());
            serialPrint("DIVE: pitch cmd: %3.1f\r\n",pitchLoop().getCommand());
            serialPrint("DIVE: heading cmd: %3.1f\r\n",headingLoop().getCommand());
            
            //reset max dive depth
            _max_recorded_depth_dive = -99;            //float to record max depth
        }
 
        // how exit?
        if (_fsm_timer.read() > _timeout) {
            serialPrint("DIVE: timed out\r\n\n");
            _state = RISE; //new behavior 11/17/2017
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        else if (depthLoop().getPosition() > depthLoop().getCommand() - 0.5) { // including offset for low momentum approaches
            serialPrint("DIVE: actual depth: %3.1f (cmd: %3.1f)\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            _state = RISE;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
 
        // WHAT IS ACTIVE?
        serialPrint("DIVE: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg [cmd:%6.1f], heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),pitchLoop().getCommand(),imu().getHeading(),_fsm_timer.read());
        bce().setPosition_mm(depthLoop().getOutput());  //constantly checking the Outer Loop output to move the motors
        batt().setPosition_mm(pitchLoop().getOutput());
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
        
        if (depthLoop().getPosition() > _max_recorded_depth_dive) {  //debug
            _max_recorded_depth_dive = depthLoop().getPosition();    //new max depth recorded
        }
        
        break;
    
    case RISE :
        // start local state timer and init any other one-shot actions
        
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: RISE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands?
            depthLoop().setCommand(-1.0);           //make sure to get towards the surface (saw issues at LASR pool)
            pitchLoop().setCommand(-_pitch_command);
            
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            
            serialPrint("RISE: depth cmd: %3.1f\r\n",depthLoop().getCommand());
            serialPrint("RISE: pitch cmd: %3.1f\r\n",pitchLoop().getCommand());
            serialPrint("RISE: heading cmd: %3.1f\r\n",headingLoop().getCommand());
        }
 
        // how exit?
        if (_fsm_timer.read() > _timeout) {
            serialPrint("RISE: timed out\r\n");
            _state = EMERGENCY_CLIMB;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        //modified from (depthLoop().getPosition() < depthLoop().getCommand() + 0.5) 
        //did not work correctly in bench test (stuck in rise state)
        else if (depthLoop().getPosition() < 0.5) {
            serialPrint("RISE: actual depth: %3.1f (cmd: %3.1f)\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
 
        // WHAT IS ACTIVE?
        serialPrint("RISE: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg [cmd:%6.1f], heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),pitchLoop().getCommand(),imu().getHeading(),_fsm_timer.read());
        bce().setPosition_mm(depthLoop().getOutput());  //constantly checking the Outer Loop output to move the motors
        batt().setPosition_mm(pitchLoop().getOutput());
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
         
        break;
        
// NEW DIVE AND RISE SEQUENCES 
    case POSITION_DIVE :               
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: POSITION DIVE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
            rudder().unpause();
 
            // what are the commands? (using inner loops except for heading outer loop)
            // These actions happen ONCE in the POSITION_DIVE sequence
            batt().setPosition_mm(_neutral_batt_pos_mm + _BMM_dive_offset);
            bce().setPosition_mm(_neutral_bce_pos_mm - _BCE_dive_offset);
            
            //DEPTH COMMAND
            depthLoop().setCommand(_depth_command);
                        
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            
            serialPrint("POS DIVE: BATT cmd: %3.1f\r\n",batt().getSetPosition_mm());  //get the actual commanded position
            serialPrint("POS DIVE: BCE cmd: %3.1f\r\n",bce().getSetPosition_mm());    //get the actual commanded position
            serialPrint("POS DIVE: heading cmd: %3.1f\r\n",headingLoop().getCommand());
            
            //reset max dive depth
            _max_recorded_depth_dive = -99;            //float to record max depth
        }
 
        // how exit?
        // timer runs out goes to POSITION_RISE
        if (_fsm_timer.read() > _timeout) {
            serialPrint("POS DIVE timed out\r\n\n");
            _state = POSITION_RISE; //new behavior 11/17/2017
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        // when you reach the dive threshold, surface
        else if (depthLoop().getPosition() > depthLoop().getCommand() - 0.5) { // including offset for low momentum approaches
            serialPrint("POS DIVE: actual depth: %3.1f (cmd: %3.1f)\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            _state = POSITION_RISE;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
 
        // what is active?
        serialPrint("POS DIVE: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg, heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),imu().getHeading(),_fsm_timer.read());
        
        if (depthLoop().getPosition() > _max_recorded_depth_dive) {
            _max_recorded_depth_dive = depthLoop().getPosition();    //new max depth recorded when it is larger than previous values
        }
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
        
        break;
    
    case POSITION_RISE :
        // start local state timer and init any other one-shot actions
        
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: POSITION RISE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands? (using inner loops except for heading outer loop)            
            batt().setPosition_mm(_neutral_batt_pos_mm - _BMM_dive_offset);          //reversing the BCE and BATT positions
            bce().setPosition_mm(_neutral_bce_pos_mm + _BCE_dive_offset);            //reversing the BCE and BATT positions
            
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            
            serialPrint("POS RISE: BATT cmd: %3.1f\r\n",batt().getSetPosition_mm());  //get the actual commanded position
            serialPrint("POS RISE: BCE cmd: %3.1f\r\n",bce().getSetPosition_mm());    //get the actual commanded position
            serialPrint("POS RISE: heading cmd: %3.1f\r\n",headingLoop().getCommand());
        }
 
        // how exit?
        if (_fsm_timer.read() > _timeout) {
            serialPrint("POS RISE: timed out\r\n");
            _state = EMERGENCY_CLIMB;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        else if (depthLoop().getPosition() < 0.5) {
            serialPrint("POS RISE: actual depth: %3.1f (cmd: %3.1f)\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
 
        // what is active?
        serialPrint("POS RISE: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg, heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),imu().getHeading(),_fsm_timer.read());
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
         
        break;
// NEW DIVE AND RISE SEQUENCES 
    
    case FLOAT_LEVEL :
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: FLOAT_LEVEL\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands?
            bce().setPosition_mm(_bceFloatPosition);
            pitchLoop().setCommand(0.0);
        }
        
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("FL: timed out\r\n");
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        else if (fabs(imu().getPitch() - pitchLoop().getCommand()) < fabs(_pitchTolerance)) {         //current tolerance is 5 degrees
            serialPrint("FL: pitch: %3.1f mm, set pos: %3.1f mm, deadband: %3.1f mm\r\n",imu().getPitch(), pitchLoop().getCommand(), _pitchTolerance);
            _state = FLOAT_BROADCAST;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        // what is active?
        serialPrint("FL: pitchLoop output: %3.1f, batt pos: %3.1f, piston pos: %3.1f [%0.1f sec]\r", pitchLoop().getOutput(), batt().getPosition_mm(), bce().getPosition_mm(), _fsm_timer.read());
        batt().setPosition_mm(pitchLoop().getOutput());
        
        break;
    
    case FLOAT_BROADCAST :
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: FLOAT_BROADCAST\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            // what are the commands?
            bce().setPosition_mm(_bceFloatPosition);        // old 320.0, new 360 12/18/2018
            batt().setPosition_mm(_battFloatPosition);      // old 73.0, new 50 12/18/2018
            
            //set rudder to center
            rudder().setPosition_deg(0.0);  //set rudder to center, zero degrees
        }
        
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("FB: timed out\r\n");
            _state = SIT_IDLE;
            _fsm_timer.reset();
            
            //stop recording data
            //mbedLogger().closeFile();
            
            _isTimeoutRunning = false;
        }
        
        //fix on float_broadcast to account for BCE stopping early in current hardware
        //friction in BCE is screwing up the return to float position
        //still working on integral function
        else if ( (fabs(bce().getPosition_mm() - bce().getSetPosition_mm()) < 5.0 ) and
                  (fabs(batt().getPosition_mm() - batt().getSetPosition_mm()) < batt().getDeadband()) ) {
            serialPrint("FB: position: %3.1f mm, set pos: %3.1f mm, deadband: %3.1f mm\r\n",bce().getPosition_mm(), bce().getSetPosition_mm(), bce().getDeadband());
            _state = SIT_IDLE;
            _fsm_timer.reset();
            
            //stop recording data
            //mbedLogger().closeFile();
            
            _isTimeoutRunning = false;
        }
        
        // what is active?
        serialPrint("FB: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg, heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),imu().getHeading(),_fsm_timer.read());
        
        break;
        
    case MULTI_DIVE :
        // start local state timer and init any other one-shot actions        
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: MULTI-DIVE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
                
            // what needs to be started?
            bce().unpause();
            batt().unpause();
            
            //retrieve commands from structs (loaded from sequence.cfg file)
            float sequence_depth_command = currentStateStruct.depth;
            float sequence_pitch_command = currentStateStruct.pitch;
    
            // what are the commands?            
            depthLoop().setCommand(sequence_depth_command);
            pitchLoop().setCommand(sequence_pitch_command);
            
            
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            serialPrint("MULTI-DIVE: depth cmd: %3.1f ft, pitch cmd: %3.1f deg\r\n",depthLoop().getCommand(), pitchLoop().getCommand());
            
            //no max depth recording right now
        }
        
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("\r\n\nMULTI-DIVE: timed out [time: %0.1f]\r\n\n", _fsm_timer.read());
            _state = MULTI_RISE; //new behavior 11/17/2017
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        else if (depthLoop().getPosition() > depthLoop().getCommand()) {
            serialPrint("MULTI-DIVE: depth: %3.1f, cmd: %3.1f\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            _state = MULTI_RISE;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        // WHAT IS ACTIVE?
        serialPrint("MD: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg [cmd:%6.1f], heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),pitchLoop().getCommand(),imu().getHeading(),_fsm_timer.read());
        bce().setPosition_mm(depthLoop().getOutput());
        batt().setPosition_mm(pitchLoop().getOutput());
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
        
        break;
    
    case MULTI_RISE :
        // start local state timer and init any other one-shot actions
        if (!_isTimeoutRunning) {
            serialPrint("\r\n\nstate: MULTI-RISE\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
            
            // what needs to be started?
            bce().unpause();
            batt().unpause();
 
            //NEW: retrieve depth and pitch commands from config file struct
            // concept is to load this each time the multi-dive restarts
            stateMachine().getDiveSequence();
            
            //retrieve just pitch command from struct
            float sequence_pitch_command = currentStateStruct.pitch;
 
            // what are the commands? (send back to 0.5 feet, not surface) // 11/21/2017
            depthLoop().setCommand(0.5);
            pitchLoop().setCommand(-sequence_pitch_command);            
            
            headingLoop().setCommand(_heading_command);     //ACTIVE HEADING (mimic of dive and rise code)
            serialPrint("MULTI-RISE: depth cmd: 0.0 ft, pitch cmd: %3.1f deg\r\n",depthLoop().getCommand(), pitchLoop().getCommand());
        }
        
        // how exit?
        if (_fsm_timer > _timeout) {
            serialPrint("MULTI-RISE: timed out [time: %0.1f]\r\n\n", _fsm_timer.read());
            _state = EMERGENCY_CLIMB;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
            
            //reset multi-dive sequence to start
            _multi_dive_counter = 0;
            
//            //Reload the dive sequence on exit
//            sequenceController().loadSequence();
        }
        else if (depthLoop().getPosition() < 0.5) { // depth is less than 0.5 (zero is surface level)
            serialPrint("MULTI-RISE: depth: %3.1f, cmd: %3.1f\r\n", depthLoop().getPosition(), depthLoop().getCommand());
            
            //going to next state            
            _isTimeoutRunning = false;
            
            //successful dive-rise sequence CONTINUES the multi-dive sequence
            _multi_dive_counter++;
            
            //UPDATE THE SEQUENCE DATA HERE
            stateMachine().getDiveSequence();
            
            //check if this is the end of the dive sequence
            //CHECK BEFORE ANYTHING ELSE that you have reached the "exit" state (FLOAT_BROADCAST)
            if (currentStateStruct.state == FLOAT_BROADCAST) {
//                //Reload the dive sequence on exit
//                sequenceController().loadSequence();
            
                _state = FLOAT_BROADCAST;
            }
            
            else 
                _state = MULTI_DIVE;    //Note: need to test if this else statement is necessary
            
            //have to stop this with the _multi_dive_counter variable!
        }
        
        // WHAT IS ACTIVE?
        serialPrint("MR: BcePos (cmd):%6.1f mm(%0.1f), BattPos:%6.1f mm(%0.1f), RUD_deg_cmd: %5.1f <<current depth:%6.1f ft [cmd:%6.1f]), pitch:%6.1f deg [cmd:%6.1f], heading_imu:%6.1f deg>>[%0.2f sec]                                         \r", bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),depthLoop().getCommand(),pitchLoop().getPosition(),pitchLoop().getCommand(),imu().getHeading(),_fsm_timer.read());
        bce().setPosition_mm(depthLoop().getOutput());  //constantly checking the Outer Loop output to move the motors
        batt().setPosition_mm(pitchLoop().getOutput()); 
        
        // ACTIVE RUDDER CONTROL
        rudder().setPosition_deg(headingLoop().getOutput());
        
        break; 
        
    case RX_SEQUENCE :
        serialPrint("state: RX_SEQUENCE\r\n");
    
        if (!_isTimeoutRunning) {
            serialPrint("RX_SEQUENCE _isTimeoutRunning\r\n");
            _fsm_timer.reset(); // timer goes back to zero
            _fsm_timer.start(); // background timer starts running
            _isTimeoutRunning = true; 
        }
        
        if (_fsm_timer.read() > _timeout) {
            serialPrint("RX_SEQUENCE: timed out!\r\n");
            _state = SIT_IDLE;
            _fsm_timer.reset();
            _isTimeoutRunning = false;
        }
        
        // what is active?
        serialPrint("Receive sequence active?\r\n");
        
        break;
    
    default :
        serialPrint("DEBUG: SIT_IDLE\r\n");
        _state = SIT_IDLE;
    }
    
    //save the state to print to user
    if (_previous_state != _state) {
        _state_array[_state_array_counter] = _state;  //save to state array
        _state_array_counter++;
        
        _previous_state = _state;
    }
    
    return _state;
}   /* end of runStateMachine */
 
// output the keyboard menu for user's reference
void StateMachine::printSimpleMenu() {   
    serialPrint("\r\r\n\nSIMPLE KEYBOARD MENU (01/16/2019):\r\r\n");        //make sure depth sensor tares itself on startup
    serialPrint(" Neutral Positions BCE: %0.1f BMM: %0.1f (LIMIT: BCE: %0.1f BATT: %0.1f)\r\n\n", _neutral_bce_pos_mm, _neutral_batt_pos_mm, bce().getTravelLimit(),batt().getTravelLimit());
    
    serialPrint("  V to POSITION DIVE (initiate motor position-based dive cycle)\r\n");
    serialPrint("  J to float level\r\n");
    serialPrint("  B to float at broadcast pitch\r\n");
    serialPrint("  E to initiate emergency climb\r\n");
    serialPrint("  P to print the current log file.\r\n");
    serialPrint("  G to transmit MBED log file\r\n");
    serialPrint("  I to receive multi-dive sequence file\r\n");
    serialPrint("  ~ to erase mbed log file. (clear before logging more than a few runs)\r\n");
    
    serialPrint("Q to TYPE in the BMM offset: %0.1f (BMM Dive POS: %0.1f, Rise POS: %0.1f) (positive input offset = pitch down)\r\n",_BMM_dive_offset, _neutral_batt_pos_mm + _BMM_dive_offset, _neutral_batt_pos_mm - _BMM_dive_offset);  
    serialPrint("A to TYPE in the BCE offset: %0.1f (BCE Dive POS: %0.1f, Rise POS: %0.1f) (positive input offset = dive)\r\n",_BCE_dive_offset, _neutral_bce_pos_mm - _BCE_dive_offset, _neutral_bce_pos_mm + _BCE_dive_offset);
    serialPrint("W to TYPE in the heading command: %0.1f deg (imu heading: %0.1f)\r\n",_heading_command,imu().getHeading());
    serialPrint("S to TYPE in depth setpoint: %0.1f (Current depth: %0.1f ft)\r\n",_depth_command, depthLoop().getPosition());
    serialPrint("T to TYPE in the timeout (default is 60 seconds): %d s\r\n",_timeout);    
    
    serialPrint("  C See sensor readings (and max recorded depth of dive & neutral sequences)\r\n");
    serialPrint("  8 STREAM SENSOR STATUS (and channel readings)\r\n");    
    serialPrint("  ? to reset mbed\r\n");
    serialPrint("  * (asterisk) to go to DEBUG keyboard menu\r\n");
}

void StateMachine::printDebugMenu() {
    serialPrint("\r\r\n\nDEBUG KEYBOARD MENU (01/16/2019):\r\r\n");
    serialPrint("  Y to go into CHECK NEUTRAL TUNING (This is on a timer! Uses NEUTRAL positions!)\r\n");
    serialPrint("  N to find neutral\r\n");
    serialPrint("  M to initiate multi-dive cycle\r\n");
    serialPrint("  D to initiate dive cycle\r\n");
    serialPrint("  R to initiate rise\r\n");
    serialPrint("  J to float level\r\n");
    serialPrint("  B to float at broadcast pitch\r\n");
    serialPrint("  E to initiate emergency climb\r\n");
    serialPrint("  '}' to HOME the BCE (5 second delay)\r\n");
    serialPrint("  '|' to HOME the BMM (5 second delay)\r\n");
    serialPrint("  Z to show FSM and sub-FSM states.\r\n");
    serialPrint("  P to print the current log file.\r\n");
    serialPrint("  X to print the list of log files.\r\n");
    serialPrint("  I to receive data.\r\n");
    serialPrint("  G to transmit MBED log file (60 second timeout)\r\n");
    serialPrint("  ~ to erase mbed log file. (clear before logging more than a few runs)\r\n");
    serialPrint("; or : to TYPE in the BCE neutral position: %0.1f << curr pos: %0.1f >>\r\n", _neutral_bce_pos_mm,bce().getPosition_mm());
    serialPrint("[ or { to TYPE in the BMM neutral position: %0.1f << curr pos: %0.1f >>\r\n", _neutral_batt_pos_mm,batt().getPosition_mm());
    serialPrint("Q to TYPE in pitch setpoint: %0.1f (Current IMU pitch: %0.1f deg)\r\n",_pitch_command,imu().getPitch());
    serialPrint("A to TYPE in depth setpoint: %0.1f (Current depth: %0.1f ft)\r\n",_depth_command, depthLoop().getPosition());
    serialPrint("W to TYPE in the heading command: %0.1f deg (imu heading: %0.1f)\r\n",_heading_command,imu().getHeading());
    serialPrint("T to enter in the timeout (default is 60 seconds): %d s\r\n",_timeout);

    serialPrint("  1 BCE PID sub-menu (type in inputs)\r\n");
    serialPrint("  2 BATT PID sub-menu (type in inputs)\r\n");
    serialPrint("  3 Depth PID sub-menu (type in inputs)\r\n");
    serialPrint("  4 Pitch PID sub-menu (type in inputs)\r\n");
    serialPrint("  5 Rudder (servo) sub-menu\r\n");
    serialPrint("  6 HEADING PID sub-menu (type in inputs)\r\n");
    serialPrint("  7 MANUAL_TUNING sub-menu (does not have a timer!)  *** MOTORS ARE ACTIVE *** (bce 200, bmm 40, rudder 1640)\r\n");
    serialPrint("  8 STREAM SENSOR STATUS (and channel readings)\r\n");
    
    serialPrint(" C See sensor readings (and max recorded depth of dive & neutral sequences)\r\n");
    serialPrint(" ? to reset mbed\r\n");
    serialPrint(" * (asterisk) to go to SIMPLE keyboard menu\r\n");
}
 
//FIND_NEUTRAL sub-Finite State Machine (sub-FSM)
// Note: the sub-FSM only moves the pistons once at the start of each timer loop
//  (timer completes, moves piston, timer completes, moves piston, etc)
int StateMachine::runNeutralStateMachine() {                
    switch (_substate) {
        case NEUTRAL_SINKING :
            //start the 10 second timer
            if (!_isSubStateTimerRunning) {                
                _neutral_timer = _fsm_timer.read() + 5; //record the time when this block is first entered and add 5 seconds
                
                serialPrint("\r\n\nNEUTRAL_SINKING: Next retraction at %0.1f sec [current time: %0.1f] (pitch: %0.1f) (BCE getSetPosition: %0.1f)\r\n", _neutral_timer, _fsm_timer.read(), pitchLoop().getPosition(), bce().getSetPosition_mm());
                
                // what are the commands? (BCE linear actuator active, no BMM or pitch movement)
                bce().setPosition_mm(bce().getSetPosition_mm() - 2.5);
                
                serialPrint("NEUTRAL_SINKING: Retracting piston 2.5 mm [BCE CMD : %0.1f] (pitch: %0.1f)\r\n", bce().getSetPosition_mm(), pitchLoop().getPosition());
                
                _isSubStateTimerRunning = true;    //disable this block after one iteration
            }
 
            // how exit?
            //once reached the travel limit, no need to keep trying, so exit
            if (bce().getPosition_mm() <= 0) {
                serialPrint("\r\nDEBUG: BCE current position is %0.1f mm (NEXT SUBSTATE NEUTRAL EXIT)\r\n", bce().getPosition_mm());
                _substate = NEUTRAL_EXIT;
                _isSubStateTimerRunning = false; // reset the sub state timer
            }
            //Troy: Pressure vessel went beyond set depth limit, goes to next state
            //once deeper than the commanded setpoint...
            else if (depthLoop().getPosition() > _depth_command) {
                _substate = NEUTRAL_SLOWLY_RISE; // next state
                _isSubStateTimerRunning = false; //reset the sub state timer
            }
 
            // what is active?
            //once the 10 second timer is complete, reset the timeout so the state one-shot entry will move the setpoint
            if (_fsm_timer.read() >= _neutral_timer) {
                serialPrint("\r\n\n NEUTRAL_SINKING TIMER COMPLETE! [current time: %0.1f]\r\n", _fsm_timer.read());
                
                _isSubStateTimerRunning = false; // reset the sub state timer to do one-shot actions again
            }
            
            // what is active? (only the buoyancy engine moved every 5 seconds at start)
            serialPrint("BCE current pos: %0.1f mm (BCE setpoint: %0.1f mm) (current depth: %0.1f ft)\r", bce().getPosition_mm(),bce().getSetPosition_mm(),depthLoop().getPosition()); //debug
            
            //the BCE moves every 5 seconds. No BMM or rudder movement.
            
            break;
            
        case NEUTRAL_SLOWLY_RISE:
            if (!_isSubStateTimerRunning) {                                
                _neutral_timer = _fsm_timer.read()+ 5; //record the time when this block is first entered and add 5 seconds
                
                serialPrint("\r\n\nNEUTRAL_SLOWLY_RISE: Next extension at %0.1f sec) [current time: %0.1f]\r\n",_neutral_timer,_fsm_timer.read());
                
                // what are the commands?
                //move piston at start of sequence (default: extend 2.0 mm)
                //Pressure vessel should slowly rise
                bce().setPosition_mm(bce().getSetPosition_mm() + 2.0);  //no depth command, only motor position
                
                //Troy: I commented out this command, we're finding pitch in the next state.
                // it's okay to run the pitch outer loop now since we've already found pitch level in the previous state
                //pitchLoop().setCommand(0.0);
                
                serialPrint("NEUTRAL_SLOWLY_RISE: Extending BCE piston 2.0 mm [BCE CMD : %0.1f] (pitch: %0.1f)\r\n", bce().getSetPosition_mm(), pitchLoop().getPosition());

                _isSubStateTimerRunning = true;    //disable this block after one iteration
            }
            
            // how exit?
            //once at full travel limit (setPosition) and haven't yet risen, time to give up and exit
            if (bce().getSetPosition_mm() >= bce().getTravelLimit()) {
                _substate = NEUTRAL_EXIT;     
                _isSubStateTimerRunning = false; // reset the sub state timer
            }
            //Troy: Depth rate will go negative as the pressure vessel starts rising
            //depth rate or sink rate < 0 ft/s, go to the next substate the next iteration
            else if (depthLoop().getVelocity() < 0) { //less than zero ft/s
                serialPrint("\r\n\nNEUTRAL_SLOWLY_RISE: Sink Rate < 0 ft/s [time: %0.1f]\r\n", _fsm_timer.read());
                _substate = NEUTRAL_CHECK_PITCH;
                _isSubStateTimerRunning = false; // reset the sub state timer
            }
            
            // what is active?
            //once 5 second timer complete, reset the timeout so the state one-shot entry will move the setpoint
            if (_fsm_timer.read() >= _neutral_timer) {
                serialPrint("\r\n\n NEUTRAL_SLOWLY_RISE TIMER COMPLETE! [timer: %0.1f]\r\n", _fsm_timer.read());
   
                _isSubStateTimerRunning = false; // reset the sub state timer to do one-shot actions again
            }
                        
            // what is active? (only the buoyancy engine moved every 5 seconds)
            serialPrint("depthLoop getOutput: %0.1f\r", depthLoop().getOutput()); //debug
            
            break;   
                
        case NEUTRAL_CHECK_PITCH : // fall thru to next state is desired
            // start local state timer and init any other one-shot actions
            
            if (!_isSubStateTimerRunning) {                    
                _neutral_timer = _fsm_timer.read() + 10; // record time when this block is entered and add several seconds
                serialPrint("\r\nNEUTRAL_CHECK_PITCH: Next move in %0.1f sec \r\n",_neutral_timer - _fsm_timer.read());
                
                // what are the commands? (default: retract or extend 0.5 mm)
                if (pitchLoop().getPosition() > 2) { // nose is high (extend batteries)
                    batt().setPosition_mm(batt().getSetPosition_mm() + 0.5); // move battery forward (using setpoint from linear actuator)
                    serialPrint("\r\nNeutral Check Pitch: moving battery FWD in 0.5 mm increments\r\n\n");
                }
                else if (pitchLoop().getPosition() < -2) { // nose is low (retract batteries)
                    batt().setPosition_mm(batt().getSetPosition_mm() - 0.5); // move battery aft (using setpoint from linear actuator)
                    serialPrint("\r\nNeutral Check Pitch: moving battery AFT in 0.5 mm increments\r\n\n");
                }

                _isSubStateTimerRunning = true;    //disable this block after one iteration
            }
 
            // how exit?            
            //pitch angle and pitch rate within small tolerance
            //benchtop tests confirm angle needs to be around 2 degrees
            if ((fabs(pitchLoop().getPosition()) < 2.0) and (fabs(pitchLoop().getVelocity()) < 5.0)) { 
                serialPrint("Debug: Found Level (NEUTRAL_CHECK_PITCH or NEUTRAL_FIRST_PITCH)\r\n");    //debug
                // found level, but don't need to save anything this time
                
                if (depthLoop().getPosition() > _max_recorded_depth_neutral) {  //debug
                    _max_recorded_depth_neutral = depthLoop().getPosition();    //new max depth recorded
                }
                
                // found level and at depth too, so save it all now               
                if (_substate == NEUTRAL_CHECK_PITCH) {
                    //save positions locally
                    _neutral_batt_pos_mm = batt().getPosition_mm();
                    _neutral_bce_pos_mm = bce().getPosition_mm();
                    
                    //set the neutral positions in each outer loop
                    depthLoop().setOutputOffset(_neutral_bce_pos_mm);
                    pitchLoop().setOutputOffset(_neutral_batt_pos_mm);
                    
                    // save into the depth.txt and pitch.txt files
       
                    configFileIO().savePitchData(_pitch_KP, _pitch_KI, _pitch_KD, _neutral_batt_pos_mm, _pitch_filter_freq, _pitch_deadband); //P,I,D,batt zeroOffset
                    configFileIO().saveDepthData(_depth_KP, _depth_KI, _depth_KD, _neutral_bce_pos_mm, _depth_filter_freq, _depth_deadband); //P,I,D, bce zeroOffset

                    serialPrint("\r\n\n>>> Saving Positions: BCE: %0.1f mm, BATT: %0.1f <<<\r\n\n",_neutral_bce_pos_mm,_neutral_batt_pos_mm);
                    
                    _substate = NEUTRAL_EXIT;
                    _isSubStateTimerRunning = false; // reset the sub state timer to do one-shot actions again
                }
                
                else {
                    serialPrint("\r\nDid not find NEUTRAL_CHECK_PITCH or NEUTRAL_FIRST_PITCH, how did I get here?!\r\n");
                    _substate = NEUTRAL_EXIT;
                }
            }
            
            // what is active?
            //once timer complete, reset the timeout so the state one-shot entry will move the setpoint
            if (_fsm_timer.read() >= _neutral_timer) {
                serialPrint("\r\n\nlevel timer COMPLETE!");
                serialPrint("\r\n\n (BATT POS: %0.1f) moving 1 mm [timer: %0.1f]\r\n", batt().getPosition_mm(), _fsm_timer.read());
                _isSubStateTimerRunning = false; // reset the sub state timer to do one-shot actions again
            }

            break;
             
        //this state could be removed, it is only used as a transition but is needed to stop entering this function
        case NEUTRAL_EXIT :
            serialPrint("substate: NEUTRAL_EXIT\r\n");            
            break;
            
        default :
            serialPrint("how did we get to substate: default?\r\n"); //debug
            //a default within the sub-state machine
            _substate = NEUTRAL_EXIT;            
            break;
    }
    
    // reset the sub-FSM if needed (useful if you need to redo the neutral-finding sequence)
    if (_substate == NEUTRAL_EXIT) {
        serialPrint("********************************  EXITING sub-FSM! *******************************\r\n\n");

        //reset internal sub-state back to first entry conditions (first state is immediately sinking)
        _substate = NEUTRAL_SINKING;
        _isSubStateTimerRunning = false; // reset the sub state timer
        
        //record sub-states to view after sequence
        _substate_array[_substate_array_counter] = NEUTRAL_EXIT;  //save exit to state array
        _substate_array_counter++;
        
        //reset _previous_substate on exit (has to be done in FIND_NEUTRAL if emergency exit)
        _previous_substate = -1;

        //NEUTRAL_EXIT state is used to tell the greater FSM that this sub-FSM has completed
        return NEUTRAL_EXIT; // message to calling function we just exited
    }
    else {
        //record sub-states to view after sequence (when changed)
        if (_previous_substate != _substate) {
            _substate_array[_substate_array_counter] = _substate;  //save current state to state array
            _substate_array_counter++;
            
            //record the current substate for comparison 
            _previous_substate = _substate;
        }       
        
        return _substate; // message to calling function of what sub-state it's in
    }
}
 
/*  keyboard runs independently of the state machine, handling one key at a time
    keyboard updates the desired _keyboard_state that is used in the state machine
    and only allows input when the state is "idle" */

// NEW KEYBOARD FUNCTION 12/20/2018
void StateMachine::keyboard() {   
    if (_state == SIT_IDLE || _state == KEYBOARD) {
        if (xbee().readable()) {
            keyboardInput(xbee().getc());
        }
        
        else if (pc().readable()) {
            keyboardInput(pc().getc());
        }
    }
}
        

void StateMachine::keyboardInput(char user_input) {
    int _keyboard_state = 0;
    
    //the key should come from the keyboard() function
    
    //record that the keyboard was used
    _state_array[_state_array_counter] = KEYBOARD;
    _state_array_counter++;
    
    // keyboard has to reset timer each time it's used
    _isTimeoutRunning = false;
    
    // check command against desired control buttons

/***************************** COMMON COMMANDS *****************************/ 
    if (user_input == 'W') {
        serialPrint(">> Please enter the heading (deg).\r\n");
        _heading_command = getFloatUserInput();
    }
    
    else if (user_input == 'U') {
        serialPrint("(U) TRANSMIT MULTIPLE PACKETS \n\r");
                    
        mbedLogger().transmitMultiplePackets();
    }
    
    else if (user_input == 'I') {
        serialPrint("(I) Receive Multi-Dive Sequence! \n\r");
        mbedLogger().receiveSequenceFile();    //receive sequence.txt files
    }
    
    else if (user_input == '8') {
        keyboard_menu_STREAM_STATUS();
    }
    
    else if (user_input == '9') {
        keyboard_menu_DEBUG_PID();
    }
                
    else if (user_input == '?') {
        serialPrint("\n\n\n>>> Resetting MBED <<<\n\n\n");
        wait(0.5);
        mbed_reset();
    }
    
    else if (user_input == 'T') {
        serialPrint("Please enter the timeout (timer) value below: \n\r");
        _timeout = fabs(getFloatUserInput());
    }
    
    else if (user_input == '~') {
        serialPrint("MBED LOG FILE MENU!\r\n");
        stateMachine().logFileMenu();
            
        //serialPrint("ERASING MBED LOG FILE\r\n");   //legacy method
        //mbedLogger().eraseFile();
    }
    
    else if (user_input == 'C' or user_input == 'c') {
            
        serialPrint("\r\n\nCURRENT STATUS AND PARAMETERS:\r\n");
        
        serialPrint("raw BCE pos: %d \r\n",adc().readCh0());
        serialPrint("raw BMM pos: %d \r\n",adc().readCh1());
        serialPrint("raw BCE current sense: %d (BCE current: %0.2f amps) \r\n",adc().readCh2(), sensors().getBceCurrent());
        serialPrint("raw BMM current sense: %d (BMM current: %0.2f amps)\r\n",adc().readCh3(), sensors().getBmmCurrent());
        serialPrint("raw depth pressure: %d \r\n",adc().readCh4());
        serialPrint("raw vessel pressure %d (internal psi: %0.1f)\r\n", adc().readCh5(),sensors().getInternalPressurePSI());
        //serialPrint("raw vessel pressure: %d \r\n",adc().readCh5());
        serialPrint("raw board voltage: %d (%0.1f volts)\r\n",adc().readCh6(),sensors().getVoltageInput());
        serialPrint("raw board current: %d (%0.3f amps)\r\n",adc().readCh7(), sensors().getCurrentInput());
        serialPrint("calc vessel pressure: %f (counts: %d) \r\n",sensors().getInternalPressurePSI(),adc().readCh5());
        // End of ADC Test
        
        serialPrint("depth: %3.1f ft\r\n",depthLoop().getPosition());
        serialPrint("pitch: %3.1f deg\r\n",imu().getPitch());
        serialPrint("bce().getPosition_mm(): %3.1f\r\n",bce().getPosition_mm());
        serialPrint("bce().getSetPosition_mm(): %3.1f\r\n",bce().getSetPosition_mm());
        serialPrint("batt().getPosition_mm(): %3.1f\r\n",batt().getPosition_mm());
        serialPrint("batt().getSetPosition_mm(): %3.1f\r\n",batt().getSetPosition_mm());
        serialPrint("depthLoop().getCommand(): %3.1f\r\n",depthLoop().getCommand());
        serialPrint("pitchLoop().getCommand(): %3.1f\r\n",pitchLoop().getCommand());
        
        serialPrint("\r\nNeutral Buoyancy Positions: bce: %0.1f, batt: %0.1f\r\n",_neutral_bce_pos_mm,_neutral_batt_pos_mm);
        serialPrint("depthLoop().getOutputOffset(): %0.1f\r\n",depthLoop().getOutputOffset());
        serialPrint("pitchLoop().getOutputOffset(): %0.1f\r\n",pitchLoop().getOutputOffset());
        serialPrint("Max recorded depth: neutral: %0.1f, dive: %0.1f, auto_neutral_depth: %0.1f\r\n\n",_max_recorded_depth_neutral, _max_recorded_depth_dive, _max_recorded_auto_neutral_depth);
        
        serialPrint("\r\n");
        serialPrint("BCE      P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", bce().getControllerP(), bce().getControllerI(), bce().getControllerD(), bce().getZeroCounts(), bce().getTravelLimit(), bce().getPotSlope(), bce().getFilterFrequency(), bce().getDeadband());
        serialPrint("BMM      P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", batt().getControllerP(), batt().getControllerI(), batt().getControllerD(), batt().getZeroCounts(), batt().getTravelLimit(), batt().getPotSlope(), batt().getFilterFrequency(), batt().getDeadband());
        serialPrint("RUD_SER  min_pwm:%6.1f, center_pwm:%6.1f, max_pwm:%6.1f (min_deg:%6.1f max_deg:%6.1f)\r\n",rudder().getMinPWM(),rudder().getCenterPWM(),rudder().getMaxPWM(),rudder().getMinDeg(),rudder().getMaxDeg());
        serialPrint("depth    P:%6.3f, I:%6.3f, D:%6.3f, output offset: %6.1f mm,  filter_freq: %0.1f, deadband: %0.1f \r\n", depthLoop().getControllerP(), depthLoop().getControllerI(), depthLoop().getControllerD(), depthLoop().getOutputOffset(),depthLoop().getFilterFrequency(),depthLoop().getDeadband());
        serialPrint("pitch    P:%6.3f, I:%6.3f, D:%6.3f, output offset: %6.1f mm,  filter_freq: %0.1f, deadband: %0.1f \r\n", pitchLoop().getControllerP(), pitchLoop().getControllerI(), pitchLoop().getControllerD(), pitchLoop().getOutputOffset(),pitchLoop().getFilterFrequency(),pitchLoop().getDeadband());
        serialPrint("heading  P:%6.3f, I:%6.3f, D:%6.3f, output offset: %6.1f deg, filter_freq: %0.1f, deadband: %0.1f \r\n", headingLoop().getControllerP(), headingLoop().getControllerI(), headingLoop().getControllerD(), headingLoop().getOutputOffset(),headingLoop().getFilterFrequency(),headingLoop().getDeadband());
    }
    
/***************************** COMMON COMMANDS *****************************/
    
/***************************** DEBUG MENU *****************************/             
    if (_debug_menu_on) {            
        if (user_input == 'D') {
            _keyboard_state = DIVE;
        }
        
        else if (user_input == '}') {
            serialPrint("HOMING the BCE (5 second delay)\r\n");
            wait(5);
            bce().homePiston();
        }
        
        else if (user_input == '|') {
            serialPrint("HOMING the BMM (5 second delay)\r\n");
            wait(5);
            batt().homePiston();
        }
        
        else if (user_input == 'N') {
            _keyboard_state = FIND_NEUTRAL;
        }
        else if (user_input == 'M') {
            //currently does not run if there is no file.
            
            //need to add method to Sequence Controller that returns -1 
            //   or some check that insures you cannot run the dive sequence without a file
            
            //load sequence from file
            _multi_dive_counter = 0;
            sequenceController().loadSequence();
            wait(1);    //test if this resets the sequence
            
            stateMachine().getDiveSequence();               //get first sequence on keyboard press
            _keyboard_state = currentStateStruct.state;
            
            serialPrint("Starting Dive Sequence Controller! (state: %d)\r\n", _keyboard_state);  //neutral sequence and dive cycles
        }
        else if (user_input == 'R') {
            _keyboard_state = RISE;
        }
        else if (user_input == 'J') {
            _keyboard_state = FLOAT_LEVEL;
        }
        else if (user_input == 'B') {
            _keyboard_state = FLOAT_BROADCAST;
        }
        else if (user_input == 'E') {
            _keyboard_state = EMERGENCY_CLIMB;
        }
        
        else if (user_input == 'Y') {
            _keyboard_state = CHECK_TUNING;
        }
        
        // some debug tools below
        else if (user_input == 'P') {
            //Print current SD card log file
            //printCurrentSdLog();
            mbedLogger().printCurrentLogFile();        //print the current log file to the screen
        }
        else if (user_input == 'X') {
            mbedLogger().printMbedDirectory();        //print all log files to the screen
        }
        else if (user_input == 'Z') {
            serialPrint("FSG FSM States: \r\n");
            string string_state;
            
            for (int i = 0; i < _state_array_counter; i++) {
                if (_state_array[i] == SIT_IDLE)
                    string_state = "SIT_IDLE              <END>";
                else if (_state_array[i] == FIND_NEUTRAL)
                    string_state = "FIND_NEUTRAL";
                else if (_state_array[i] == DIVE)
                    string_state = "DIVE";
                else if (_state_array[i] == RISE)
                    string_state = "RISE";
                else if (_state_array[i] == FLOAT_LEVEL)
                    string_state = "FLOAT_LEVEL";
                else if (_state_array[i] == FLOAT_BROADCAST)
                    string_state = "FLOAT_BROADCAST";          
                else if (_state_array[i] == EMERGENCY_CLIMB)
                    string_state = "EMERGENCY_CLIMB";
                else if (_state_array[i] == MULTI_DIVE)
                    string_state = "MULTI_DIVE";
                else if (_state_array[i] == MULTI_RISE)
                    string_state = "MULTI_RISE";
                else if (_state_array[i] == KEYBOARD)
                    string_state = "KEYBOARD";                    
                serialPrint("State #%d: %d (%s)\r\n", i, _state_array[i], string_state.c_str());
            }
            
            serialPrint("\r\nNeutral sub-FSM States: \r\n");
            string string_substate;
            
            for (int i = 0; i < _substate_array_counter; i++) {
                if (_substate_array[i] == NEUTRAL_SINKING)
                    string_substate = "NEUTRAL_SINKING";
                else if (_substate_array[i] == NEUTRAL_SLOWLY_RISE)
                    string_substate = "NEUTRAL_SLOWLY_RISE";
                else if (_substate_array[i] == NEUTRAL_CHECK_PITCH)
                    string_substate = "NEUTRAL_CHECK_PITCH";
                else if (_substate_array[i] == NEUTRAL_EXIT)
                    string_substate = "NEUTRAL_EXIT                  <--   ";
                else if (_substate_array[i] == EMERGENCY_CLIMB)
                    string_substate = " -- > EMERGENCY_CLIMB  <-- ";                
                serialPrint("Neutral Substate #%d: %d (%s)\r\n", i, _state_array[i], string_substate.c_str());
            }
            serialPrint("\r\n");  //make space between printouts
        }    
        //BATTERY/PITCH
        else if (user_input == '[' or user_input == '{') {
            serialPrint("Please TYPE in the new BATT neutral position.\n\r");
            _neutral_batt_pos_mm = getFloatUserInput();
            pitchLoop().setOutputOffset(_neutral_batt_pos_mm); // decrease the batt neutral setpoint
            serialPrint("Adjusting batt neutral position. new offset: %0.1f\r\n",pitchLoop().getOutputOffset());
            // save neutral pitch value to config file
            configFileIO().savePitchData(_pitch_KP, _pitch_KI, _pitch_KD, _neutral_batt_pos_mm, _pitch_filter_freq, _pitch_deadband); //P,I,D,batt zeroOffset
        }
        
        //BCE/DEPTH
        else if (user_input == ';' or user_input == ':') {
            serialPrint("Please TYPE in the new BCE neutral position.\n\r");
            _neutral_bce_pos_mm = getFloatUserInput();
            depthLoop().setOutputOffset(_neutral_bce_pos_mm); // decrease the bce neutral setpoint
            serialPrint("Adjusting bce neutral position. new offset: %0.1f\r\n",depthLoop().getOutputOffset());
            // save neutral depth value to config file
            configFileIO().saveDepthData(_depth_KP, _depth_KI, _depth_KD, _neutral_bce_pos_mm, _depth_filter_freq, _depth_deadband);
        }
 
// change settings
        //heading is in the common controls        
        else if (user_input == 'Q') {
            serialPrint(">> Please enter the desired PITCH (deg).\r\n");
            _pitch_command = getFloatUserInput();
        }
        else if (user_input == 'A') {
            serialPrint(">> Please enter the desired DEPTH (ft).\r\n");
            _depth_command = getFloatUserInput();
        }
        
        else if (user_input == '5') {
            keyboard_menu_RUDDER_SERVO_settings();
        }
        
        else if (user_input == '6') {
            keyboard_menu_HEADING_PID_settings();
        }  
        
        // go to tuning sub-menu
        else if (user_input == '7') {
            keyboard_menu_MANUAL_TUNING();
        }
        
        // go to sub-menus for the PID gains (this is blocking)
        else if (user_input == '1') {
            keyboard_menu_BCE_PID_settings();
        }
        else if (user_input == '2') {
            keyboard_menu_BATT_PID_settings();
        }
        else if (user_input == '3') {
            keyboard_menu_DEPTH_PID_settings();
        }
        else if (user_input == '4') {
            keyboard_menu_PITCH_PID_settings();
        }
                 
        else if (user_input == '*') {
            serialPrint("SWITCHING TO SIMPLE MENU!\r\n"); 
            wait(1);
            _debug_menu_on = false;
        }
    }   //end of debug menu
/***************************** DEBUG MENU *****************************/
        
/***************************** SIMPLE MENU *****************************/
    else {     
        if (user_input == 'V') {
            _keyboard_state = POSITION_DIVE;
        }
        else if (user_input == 'N') {
            _keyboard_state = FIND_NEUTRAL;
        }
        else if (user_input == 'J') {
            _keyboard_state = FLOAT_LEVEL;
        }
        else if (user_input == 'B') {
            _keyboard_state = FLOAT_BROADCAST;
        }
        else if (user_input == 'E') {
            _keyboard_state = EMERGENCY_CLIMB;
        }
        
        // some debug tools below
        else if (user_input == 'P') {
            //Print current SD card log file
            //printCurrentSdLog();
            mbedLogger().printCurrentLogFile();        //print the current log file to the screen
        }
        
//POSITION DIVE COMMANDS
        else if (user_input == 'Q') {
            serialPrint(">> Please enter the desired BMM offset (mm).\r\n");
            _BMM_dive_offset = getFloatUserInput();
        }
        else if (user_input == 'A') {
            serialPrint(">> Please enter the desired BCE offset (mm).\r\n");
            _BCE_dive_offset = getFloatUserInput();
        }
        
        else if (user_input == 'S') {
            serialPrint(">> Please enter the desired DEPTH (ft).\r\n");
            _depth_command = getFloatUserInput();
        }
//POSITION DIVE COMMANDS
        
        else if (user_input == '*') {
            serialPrint("SWITCHING TO DEBUG MENU!\r\n"); 
            _debug_menu_on = true;
            wait(1);
        }
    }
/***************************** SIMPLE MENU *****************************/
    
    //when you read the keyboard successfully, change the state
    _state = _keyboard_state;   //set state at the end of this function
    //serialPrint("\r\n\n ********* KEYBOARD STATE: %d *********\r\n\n", _state);
}

void StateMachine::keyboard_menu_STREAM_STATUS() {
    char STATUS_key;
    
    bool channel_on_off = false;
    bool sensors_on_off = false;
        
    // show the menu
    serialPrint("\r\n8: STATUS DEBUG MENU (EXIT WITH 'X' !)\r\n");
    
    while (1) {
        if (xbee().readable()) {
            STATUS_key = xbee().getc();   //get each keystroke
        }
        
        else {
            wait(0.5);         
            
            if (channel_on_off)
                serialPrint("[FILT/RAW 0(%d,%d),1(%d,%d),2(%d,%d),3(%d,%d),4(%d,%d),5(%d,%d),6(%d,%d),7(%d,%d)]\r",adc().readCh0(),adc().readRawCh0(),adc().readCh1(),adc().readRawCh1(),adc().readCh2(),adc().readRawCh2(),adc().readCh3(),adc().readRawCh3(),adc().readCh4(),adc().readRawCh4(),adc().readCh5(),adc().readRawCh5(),adc().readCh6(),adc().readRawCh6(),adc().readCh7(),adc().readRawCh7()); 
            
            if (sensors_on_off)
                serialPrint("BCE POS: %0.1f (cmd %0.1f) BATT POS: %0.1f (cmd %0.1f) PRESS_psi: %0.2f [depth_ft: %0.2f][depth_loop: %0.2f ft][zeroPSIoffset:%0.2f], PITCH: %0.2f, HEADING: %0.2f, rdr_pwm: %0.1f  <<Switch: BCE(%d) BMM(%d) [DEPTH %f psi] >>\r",bce().getPosition_mm(), bce().getSetPosition_mm(),batt().getPosition_mm(), batt().getSetPosition_mm(),depth().getPsi(),depth().getDepthFt(),depthLoop().getPosition(),depth().getZeroPSI(),imu().getPitch(),imu().getHeading(),rudder().getSetPosition_pwm(),bce().getHardwareSwitchStatus(),batt().getHardwareSwitchStatus(),depth().readVoltage()); 

            continue; // SKIP (didn't get a user input, so keep waiting for it)
        }
    
        // process the keys            
        if (STATUS_key == 'X') {  
            serialPrint("\r\nX: EXITING STATUS DEBUG MENU\r\n");             
            break;  //exit the while loop
        }
        
        else if (STATUS_key == '1') {  
            serialPrint("\r\nX: CHANNEL READINGS\r\n");             
            channel_on_off = !channel_on_off;
        }
        
        else if (STATUS_key == '2') {  
            serialPrint("\r\n2: SENSORS\r\n"); 
            sensors_on_off = !sensors_on_off;            
        }
        
        else {
            serialPrint("\r\nThis key (%c) does nothing here. << 1 for channels. 2 for sensors. >>                               ", STATUS_key);
        }
    }
}

void StateMachine::keyboard_menu_DEBUG_PID() {
    char STATUS_key;    
    serialPrint("\r\n8: DEBUG PID MENU (EXIT WITH 'X' !)\r\n");
    
    while (1) {
        if (xbee().readable()) {
            STATUS_key = xbee().getc();   //get each keystroke
        }
        
        else {
            wait(0.5);         
            
            serialPrint("BCE POS: %0.1f (cmd %0.1f) <<output: %0.2f>> [err(%0.2f),int(%0.2f),der(%0.2f)] \n\r", bce().getPosition_mm(),bce().getSetPosition_mm(),bce().getOutput(),bce().getPIDErrorTerm(),bce().getPIDIntegralTerm(),bce().getPIDDerivativeTerm() );
            serialPrint("BMM POS: %0.1f (cmd %0.1f) <<output: %0.2f>> [err(%0.2f),int(%0.2f),der(%0.2f)] \n\r" , batt().getPosition_mm(),batt().getSetPosition_mm(),batt().getOutput(),batt().getPIDErrorTerm(),batt().getPIDIntegralTerm(),batt().getPIDDerivativeTerm() );

            continue; // SKIP (didn't get a user input, so keep waiting for it)
        }
    
        // process the keys            
        if (STATUS_key == 'X') {  
            serialPrint("\r\nX: EXITING STATUS DEBUG MENU\r\n");             
            break;  //exit the while loop
        }
        
//        else if (STATUS_key == '1') {  
//            serialPrint("\r\nX: CHANNEL READINGS\r\n");             
//            channel_on_off = !channel_on_off;
//        }
//        
//        else if (STATUS_key == '2') {  
//            serialPrint("\r\n2: SENSORS\r\n"); 
//            sensors_on_off = !sensors_on_off;            
//        }
//        
//        else {
//            serialPrint("\r\nThis key (%c) does nothing here. << 1 for channels. 2 for sensors. >>                               ", STATUS_key);
//        }
    }
}

void StateMachine::keyboard_menu_RUDDER_SERVO_settings() {
    //load current parameters from the rudder
    float rudder_min_pwm = rudder().getMinPWM();
    float rudder_max_pwm = rudder().getMaxPWM();
    float rudder_ctr_pwm = rudder().getCenterPWM();
    float rudder_min_deg = rudder().getMinDeg();
    float rudder_max_deg = rudder().getMaxDeg();
    
    char RUDDER_PID_key;
 
    // print the menu
    serialPrint("\r\nRUDDER (servo driver) settings (MENU)");
    serialPrint("\r\nAdjust min_pwm/max_pwm/center_pwm/min_deg/max_deg settings with the following keys: N, M, C, K, L");
    serialPrint("\r\nHit shift + X to exit w/o saving.  Hit shift + S to save.\r\n");
    serialPrint("RUDDER min pwm: %f, max pwm: %f, center pwm: %f, min deg: %f, max deg: %f\r\n", rudder().getMinPWM(), rudder().getMaxPWM(), rudder().getCenterPWM(), rudder().getMinDeg(), rudder().getMaxDeg());
 
    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            RUDDER_PID_key = xbee().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
 
    // handle the user's key input                
        if (RUDDER_PID_key == 'S') { // user wants to save the modified values
            // set global values
            rudder().setMinPWM(rudder_min_pwm);
            rudder().setMaxPWM(rudder_max_pwm);
            rudder().setCenterPWM(rudder_ctr_pwm);
            rudder().setMinDeg(rudder_min_deg);
            rudder().setMaxDeg(rudder_max_deg);
            
            // save rudder servo driver values for inner loop
            configFileIO().saveRudderData(rudder_min_deg, rudder_max_deg, rudder_ctr_pwm, rudder_min_pwm, rudder_max_pwm);
            serialPrint("RUDDER min pwm: %f, max pwm: %f, center pwm: %f, min deg: %f, max deg: %f\r\n", rudder().getMinPWM(), rudder().getMaxPWM(), rudder().getCenterPWM(), rudder().getMinDeg(), rudder().getMaxDeg());
            serialPrint("Adjust min_pwm/max_pwm/center_pwm/min_deg/max_deg settings with the following keys: N, M, C, K, L\r\n");
        }
        else if (RUDDER_PID_key == 'X') {    
            break;  //exit the while loop
        }
            // MIN PWM
        else if (RUDDER_PID_key == 'N') {
            serialPrint(">> Type in rudder_min_pwm with keyboard.\r\n");
            rudder_min_pwm = getFloatUserInput();
        }
    // MAX PWM
        else if (RUDDER_PID_key == 'M') {
            serialPrint(">> Type in rudder_max_pwm with keyboard.\r\n");
            rudder_max_pwm = getFloatUserInput();
        }
    // CENTER PWM
        else if (RUDDER_PID_key == 'C') {
            serialPrint(">> Type in rudder_ctr_pwm with keyboard.\r\n");
            rudder_ctr_pwm = getFloatUserInput();
        }
    // MIN DEG
        else if (RUDDER_PID_key == 'K') {
            serialPrint(">> Type in rudder_min_deg with keyboard.\r\n");
            rudder_min_deg = getFloatUserInput();
        }
    // MAX DEG
        else if (RUDDER_PID_key == 'L') {
            serialPrint(">> Type in rudder_max_deg with keyboard.\r\n");
            rudder_max_deg = getFloatUserInput();
        }       
        else {
            serialPrint("RUDDER SETUP: [%c] This key does nothing here.                           \r", RUDDER_PID_key);
        }
    }
}

void StateMachine::keyboard_menu_COUNTS_STATUS() {
    // DELETE REMOVE  
}

// MODIFIED THIS TO RECORD THE DATA! 02/13/2019
void StateMachine::keyboard_menu_MANUAL_TUNING() {
    char TUNING_key;
        
    //02/11/2019 the positions start where the motors left off
    float _tuning_bce_pos_mm = bce().getPosition_mm();       //changed this to start wherever the motor is positioned
    float _tuning_batt_pos_mm = batt().getPosition_mm();     //changed this to start wherever the motor is positioned
    float _tuning_rudder_pos_deg = 0.0;                         //safe starting position and used if you want to tune by deg
    float _tuning_rudder_pwm = 1640.0;                          //safe starting position and used if you want to tune by PWM
    
    // bce().getTravelLimit()
    // batt().getTravelLimit()
    
    //immediately start at those positions
    bce().setPosition_mm(_tuning_bce_pos_mm);
    batt().setPosition_mm(_tuning_batt_pos_mm);
    rudder().setPosition_deg(_tuning_rudder_pos_deg);
    
    // show the menu
    serialPrint("\r\n7: MANUAL TUNING MENU (EXIT WITH 'X' !) (Pause and Unpause rudder ticker with P and U)\n");
    serialPrint("Tuning Position: BCE: %0.1f(set position BCE: %0.1f)",_tuning_bce_pos_mm,bce().getSetPosition_mm());
    serialPrint("\r\n(Adjust BCE and BATT positions in real-time.  Timeout NOT running! (decrease/increase BCE with A/S, BATT with Q/W, RUDDER with E/R)\r\n");
    serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        
    // what needs to be started? The motors and servos.
    bce().unpause();        //this is now active
    batt().unpause();       //this is now active
    rudder().unpause();     //this is now active
    
    // TURN ON FILE SAVING
    mbedLogger().appendLogFile(MANUAL_TUNING, 1);   //open file (logic in the MbedLogger method itself)
    
    while (1) {
        wait(0.1);
                      
        if (xbee().readable()) {
            TUNING_key = xbee().getc();     //get each keystroke from the XBee connection
            
            //only record on key strokes
            mbedLogger().appendLogFile(MANUAL_TUNING, 1);   //file open, append to it!
        }
        
        else if ( pc().readable() ) {
            TUNING_key = pc().getc();       //get each keystroke from the PC connection
            
            //only record on key strokes
            mbedLogger().appendLogFile(MANUAL_TUNING, 1);   //file open, append to it!
        }
        
        else {              
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // process the keys            
        if (TUNING_key == 'X') {    
            // STOP THE MOTORS BEFORE LEAVING! (Just in case.)
            bce().pause();
            batt().pause();
            rudder().pause();
            
            //right now the rudder is always active................................................hmm
            //deactivate the pin? new/delete?
            
            //CLOSE THE FILE
            mbedLogger().appendLogFile(MANUAL_TUNING, 0);
            wait(1);    //give it time to work before screwing it up!
            serialPrint("\r\nEXITING MANUAL TUNING!\r\n");
            
            break;  //exit the while loop
        }
        
        //Buoyancy Engine
        //LARGE (10 mm) and small (1 mm) movements
        else if (TUNING_key == 'A') {
            _tuning_bce_pos_mm = _tuning_bce_pos_mm - 10.0;
            bce().setPosition_mm(_tuning_bce_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BCE CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",bce().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        else if (TUNING_key == 'a') {
            _tuning_bce_pos_mm = _tuning_bce_pos_mm - 1.0;
            bce().setPosition_mm(_tuning_bce_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BCE CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",bce().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        else if (TUNING_key == 'S') {
            _tuning_bce_pos_mm = _tuning_bce_pos_mm + 10.0;
            bce().setPosition_mm(_tuning_bce_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BCE CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",bce().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        else if (TUNING_key == 's') {
            _tuning_bce_pos_mm = _tuning_bce_pos_mm + 1.0;
            bce().setPosition_mm(_tuning_bce_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BCE CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",bce().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        
        //BATTERY
        else if (TUNING_key == 'Q') {
            _tuning_batt_pos_mm = _tuning_batt_pos_mm - 10.0;
            batt().setPosition_mm(_tuning_batt_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BATT CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",batt().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        else if (TUNING_key == 'q') {
            _tuning_batt_pos_mm = _tuning_batt_pos_mm - 1.0;
            batt().setPosition_mm(_tuning_batt_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BATT CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",batt().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        
        else if (TUNING_key == 'W') {
            _tuning_batt_pos_mm = _tuning_batt_pos_mm + 10.0;
            batt().setPosition_mm(_tuning_batt_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BATT CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",batt().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        else if (TUNING_key == 'w') {
            _tuning_batt_pos_mm = _tuning_batt_pos_mm + 1.0;
            batt().setPosition_mm(_tuning_batt_pos_mm);              //this variable is loaded from the file at initialization
            serialPrint("\r\nMANUAL_TUNING: (BATT CHANGE: %0.1f)\r\n BCE_position: %0.1f, BATT_position: %0.1f (depth: %0.1f ft, pitch: %0.1f deg)\r",batt().getSetPosition_mm(),bce().getPosition_mm(),batt().getPosition_mm(),depthLoop().getPosition(),pitchLoop().getPosition());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }        
        else if (TUNING_key == 'c' or TUNING_key == 'C') {
            serialPrint("\r\nMT: (CUR_POS) BCE:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
            
            //debug 12/12/2018
            //serialPrint("\nDEBUG Output: BCE(%0.3f) and BATT(%0.3f)\n", bce().getOutput(),batt().getOutput()); 
            //debugging 12/18/2018
            serialPrint("\nOutput: BCE(%0.3f) and BATT(%0.3f) [bce: error(%0.3f) int(%0.3f) der(%0.3f)] [batt: error(%0.3f) int(%0.3f) der(%0.3f)]\n", bce().getOutput(), batt().getOutput(), bce().getPIDErrorTerm(), bce().getPIDIntegralTerm(), bce().getPIDDerivativeTerm(), batt().getPIDErrorTerm(), batt().getPIDIntegralTerm(), batt().getPIDDerivativeTerm());
        }
        
        //RUDER
        else if (TUNING_key == 'R' or TUNING_key == 'r') {
            _tuning_rudder_pos_deg = _tuning_rudder_pos_deg - 0.5;
            rudder().setPosition_deg(_tuning_rudder_pos_deg);
            serialPrint("MT: RUDDER CHANGE %0.1f deg [servo pwm: %f, %0.1f deg] (headingLoop heading: % 0.1f deg, IMU heading: %0.1f deg)\r\n", _tuning_rudder_pos_deg, rudder().getSetPosition_pwm(), rudder().getSetPosition_deg(), headingLoop().getPosition(), imu().getHeading());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        
        else if (TUNING_key == 'E' or TUNING_key == 'e') {
            _tuning_rudder_pos_deg = _tuning_rudder_pos_deg + 0.5;
            rudder().setPosition_deg(_tuning_rudder_pos_deg);
            serialPrint("MT: RUDDER CHANGE %0.1f deg [servo pwm: %f, %0.1f deg] (headingLoop heading: % 0.1f deg, IMU heading: %0.1f deg)\r\n", _tuning_rudder_pos_deg, rudder().getSetPosition_pwm(), rudder().getSetPosition_deg(), headingLoop().getPosition(), imu().getHeading());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        
        else if (TUNING_key == '-' or TUNING_key == '_') {
            _tuning_rudder_pwm -= 10.0;
            rudder().setPWM(_tuning_rudder_pwm);
            serialPrint("MT: (-) RUDDER CHANGE %0.1f pwm\n\r", rudder().getSetPosition_pwm());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
            
        else if (TUNING_key == '=' or TUNING_key == '+') {
            _tuning_rudder_pwm += 10.0;
            rudder().setPWM(_tuning_rudder_pwm);
            serialPrint("MT: (+) RUDDER CHANGE %0.1f pwm\n\r", rudder().getSetPosition_pwm());
            serialPrint("\r\nMT: BCE pos:%5.1f (cmd:%5.1f), BMM:%5.1f (cmd:%5.1f), RUDDER:%6.1f (%6.1f deg) (depth: %0.1f ft, pitch: %0.1f deg, headingLoop heading: %0.1f deg, IMU heading: %0.1f deg)                                                  \n\n\r",bce().getPosition_mm(),bce().getSetPosition_mm(),batt().getPosition_mm(),batt().getSetPosition_mm(),rudder().getSetPosition_pwm(),rudder().getSetPosition_deg(),depthLoop().getPosition(),pitchLoop().getPosition(), headingLoop().getPosition(),imu().getHeading());
        }
        
        else {
            serialPrint("\r\nMANUAL_TUNING: [%c] This key does nothing here.                                  \r", TUNING_key);
            serialPrint("\r\n(Adjust BCE and BATT positions in real-time.  Timeout NOT running! (decrease/increase BCE with A/S, BATT with Q/W, RUDDER with E/R)\r\n");
        }            
    }
}

void StateMachine::keyboard_menu_CHANNEL_READINGS() {
    char TUNING_key;
        
    // show the menu
    serialPrint("\r\n8: CHANNEL READINGS (EXIT WITH 'X' !)");
    
    while (1) {
        if (xbee().readable()) {
            TUNING_key = xbee().getc();   //get each keystroke
        }
        
                // process the keys            
        if (TUNING_key == 'X') {    
            // STOP THE MOTORS BEFORE LEAVING! (Just in case.)
            bce().pause();
            batt().pause();
            
            break;  //exit the while loop
        }
        
        else {
            wait(0.5);                    
            serialPrint("0(%d),1(%d),2(%d),6(%d),4(%d),5(%d),6(%d),7(%d)\r\n",adc().readCh0(),adc().readCh1(),adc().readCh2(),adc().readCh3(),adc().readCh4(),adc().readCh5(),adc().readCh6(),adc().readCh7()); 
            continue; // didn't get a user input, so keep waiting for it
        }            
    }
}
 
void StateMachine::keyboard_menu_BCE_PID_settings() {    
    char BCE_PID_key;
    
    // load current values from files
    float bce_KP = bce().getControllerP();
    float bce_KI = bce().getControllerI();
    float bce_KD = bce().getControllerD();
    
    float bce_deadband = bce().getDeadband();
    float bce_frequency = bce().getFilterFrequency();
    int bce_zero_offset = bce().getZeroCounts(); 
    //BCE frequency and deadband are hardcoded!
 
    // show the menu
    serialPrint("\n\rBuoyancy Engine PID gain settings (MENU). ADJUST WITH CARE!");
    serialPrint("\n\rAdjust PID settings with the following keys: P  I  D. Filter = F, deadband = B, zero offset = Z\n\r");
    serialPrint("\n\r(Hit shift + X to exit w/o saving.  Hit shift + S to save.)\n\n\n\r");
    serialPrint("bce      P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", bce().getControllerP(), bce().getControllerI(), bce().getControllerD(), bce().getZeroCounts(), bce().getTravelLimit(), bce().getPotSlope(), bce().getFilterFrequency(), bce().getDeadband());    
    
    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            BCE_PID_key = xbee().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // handle the user's key input
        if (BCE_PID_key == 'S') { // user wants to save these modified values
            // set values
            bce().setControllerP(bce_KP);
            bce().setControllerI(bce_KI);
            bce().setControllerD(bce_KD); 
            
            bce().setDeadband(bce_deadband);
            bce().setFilterFrequency(bce_frequency);
            bce().setZeroCounts(bce_zero_offset); //integer value
 
            // save to "BCE.TXT" file
            //saveBattData(float batt_p_gain, float batt_i_gain, float batt_d_gain, int batt_zeroOffset, float batt_filter_freq, float batt_deadband)
            configFileIO().saveBCEData(bce_KP, bce_KI, bce_KD, bce_zero_offset, bce_frequency, bce_deadband);
            serialPrint("bce      P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", bce().getControllerP(), bce().getControllerI(), bce().getControllerD(), bce().getZeroCounts(), bce().getTravelLimit(), bce().getPotSlope(), bce().getFilterFrequency(), bce().getDeadband());
        }
        else if (BCE_PID_key == 'X') {    
            break;  //exit the while loop
        }
        else if (BCE_PID_key == 'P') {
            serialPrint(">> Type in proportional gain with keyboard.\n\r");
            bce_KP = getFloatUserInput();
        }
        else if (BCE_PID_key == 'I') {
            serialPrint(">> Type in integral gain with keyboard.\n\r");
            bce_KI = getFloatUserInput();
        }
        else if (BCE_PID_key == 'D') {
            serialPrint(">> Type in derivative gain with keyboard.\n\r");
            bce_KD = getFloatUserInput();
        }
        else if (BCE_PID_key == 'F') {
            serialPrint(">> Type in FILTER FREQUENCY with keyboard.\n\r");
            bce_frequency = getFloatUserInput();
        }
        else if (BCE_PID_key == 'B') {
            serialPrint(">> Type in DEADBAND with keyboard.\n\r");
            bce_deadband = getFloatUserInput();
        }
        else if (BCE_PID_key == 'Z') {
            serialPrint(">> Type in zero count offset with keyboard.\n\r");
            bce_zero_offset = (int)getFloatUserInput();
        }
        else {
            serialPrint("\n\rBCE: [%c] This key does nothing here.                                  \r", BCE_PID_key);
        }
    }
}

void StateMachine::keyboard_menu_BATT_PID_settings() {    
    char BMM_PID_key;

    // load current values from files
    float batt_KP = batt().getControllerP();
    float batt_KI = batt().getControllerI();
    float batt_KD = batt().getControllerD();
    
    float batt_deadband = batt().getDeadband();
    float batt_frequency = batt().getFilterFrequency();
    int batt_zero_offset = batt().getZeroCounts(); 
    //BATT frequency and deadband are hardcoded!
 
    // print the menu
    serialPrint("\n\rBattery Motor PID gain settings (MENU)");
    serialPrint("\n\rAdjust PID settings with the following keys: P I D. Filter = F, deadband = B.\n\r");
    serialPrint("\n\r(Hit shift + X to exit w/o saving.  Hit shift + S to save.)\n\n\n\r");
    serialPrint("batt     P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", batt().getControllerP(), batt().getControllerI(), batt().getControllerD(), batt().getZeroCounts(), batt().getTravelLimit(), batt().getPotSlope(), batt().getFilterFrequency(), batt().getDeadband());

    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            BMM_PID_key = xbee().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // handle the user's key input
        if (BMM_PID_key == 'S') { // user wants to save these modified values
            // set values
            batt().setControllerP(batt_KP);
            batt().setControllerI(batt_KI);
            batt().setControllerD(batt_KD);
            
            batt().setDeadband(batt_deadband);
            batt().setFilterFrequency(batt_frequency);
            batt().setZeroCounts(batt_zero_offset); //integer value
 
            // save to "BATT.TXT" file
            //saveBCEData(float bce_p_gain, float bce_i_gain, float bce_d_gain, int bce_zeroOffset, float bce_filter_freq, float bce_deadband)
            configFileIO().saveBattData(batt_KP, batt_KI, batt_KD, batt_zero_offset, batt_frequency, batt_deadband);
            serialPrint("batt     P:%6.3f, I:%6.3f, D:%6.3f,   zero offset: %3i, limit %6.1f mm, slope %0.5f, filter_freq: %0.1f, deadband: %0.1f\r\n", batt().getControllerP(), batt().getControllerI(), batt().getControllerD(), batt().getZeroCounts(), batt().getTravelLimit(), batt().getPotSlope(), batt().getFilterFrequency(), batt().getDeadband());
        }
        else if (BMM_PID_key == 'X') {    
            break;  //exit the while loop
        }
        else if (BMM_PID_key == 'P') {
            serialPrint(">> Type in proportional gain with keyboard.\n\r");
            batt_KP = getFloatUserInput();
        }
        else if (BMM_PID_key == 'I') {
            serialPrint(">> Type in integral gain with keyboard.\n\r");
            batt_KI = getFloatUserInput();
        }
        else if (BMM_PID_key == 'D') {
            serialPrint(">> Type in derivative gain with keyboard.\n\r");
            batt_KD = getFloatUserInput();
        }
        else if (BMM_PID_key == 'F') {
            serialPrint(">> Type in FILTER FREQUENCY with keyboard.\n\r");
            batt_frequency = getFloatUserInput();
        }
        else if (BMM_PID_key == 'B') {
            serialPrint(">> Type in DEADBAND with keyboard.\n\r");
            batt_deadband = getFloatUserInput();
        }
        else if (BMM_PID_key == 'Z') {
            serialPrint(">> Type in zero count offset with keyboard.\n\r");
            batt_zero_offset = (int)getFloatUserInput();
        }
        else {
            serialPrint("\n\rBATT: [%c] This key does nothing here.                                  \r", BMM_PID_key);
        }
    }
}
 
void StateMachine::keyboard_menu_DEPTH_PID_settings() {    
    char DEPTH_PID_key;
    
    float depth_KP = depthLoop().getControllerP();       // load current depth value
    float depth_KI = depthLoop().getControllerI();       // load current depth value
    float depth_KD = depthLoop().getControllerD();       // load current depth value
    
    float depth_freq = depthLoop().getFilterFrequency();
    float depth_deadband = depthLoop().getDeadband();
 
    // print the menu
    serialPrint("\n\rDEPTH (Buoyancy Engine O.L.) PID gain settings (MENU)");
    serialPrint("\n\rAdjust PID settings with the following keys: P I D. Filter = F, deadband = B.\n\r");
    serialPrint("\n\r(Hit shift + X to exit w/o saving.  Hit shift + S to save.)\n\n\n\r");
    serialPrint("DEPTH P: %3.3f, I: %3.3f, D %3.3f, offset: %3.1f mm (filter: %0.2f, deadband: %0.2f)\r\n", depthLoop().getControllerP(), depthLoop().getControllerI(), depthLoop().getControllerD(),depthLoop().getOutputOffset(),depthLoop().getFilterFrequency(),depthLoop().getDeadband());
    
    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            DEPTH_PID_key = xbee().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // handle the user's key input
        if (DEPTH_PID_key == 'S') { // user wants to save these modified values
            // set values
            depthLoop().setControllerP(depth_KP);
            depthLoop().setControllerI(depth_KI);
            depthLoop().setControllerD(depth_KD);
            
            depthLoop().setFilterFrequency(depth_freq);
            depthLoop().setDeadband(depth_deadband);
 
            // save to "DEPTH.TXT" file
            configFileIO().saveDepthData(depth_KP, depth_KI, depth_KD, _neutral_bce_pos_mm, depth_freq, depth_deadband); //P,I,D, bce zeroOffset
            
            serialPrint("DEPTH P: %3.3f, I: %3.3f, D %3.3f, offset: %3.1f mm (filter: %0.2f, deadband: %0.2f)\r\n", depthLoop().getControllerP(), depthLoop().getControllerI(), depthLoop().getControllerD(),depthLoop().getOutputOffset(),depthLoop().getFilterFrequency(),depthLoop().getDeadband());
            
            //set class variables that will be used in find neutral sequence
            _depth_KP = depthLoop().getControllerP();       // load current depth value
            _depth_KI = depthLoop().getControllerI();       // load current depth value
            _depth_KD = depthLoop().getControllerD();       // load current depth value
        }
        else if (DEPTH_PID_key == 'X') {    
            break;  //exit the while loop
        }
        else if (DEPTH_PID_key == 'P') {
            serialPrint(">> Type in proportional gain with keyboard.\n\r");
            depth_KP = getFloatUserInput();
        }
        else if (DEPTH_PID_key == 'I') {
            serialPrint(">> Type in integral gain with keyboard.\n\r");
            depth_KI = getFloatUserInput();
        }
        else if (DEPTH_PID_key == 'D') {
            serialPrint(">> Type in derivative gain with keyboard.\n\r");
            depth_KD = getFloatUserInput();
        }
        else if (DEPTH_PID_key == 'F') {
            serialPrint(">> Type in FILTER FREQUENCY with keyboard.\n\r");
            depth_freq = getFloatUserInput();
        }
        else if (DEPTH_PID_key == 'B') {
            serialPrint(">> Type in DEADBAND with keyboard.\n\r");
            depth_deadband = getFloatUserInput();
        }
        else {
            serialPrint("\n\rDEPTH: [%c] This key does nothing here.                                  \r", DEPTH_PID_key);
        }
    }
}
 
void StateMachine::keyboard_menu_PITCH_PID_settings() {    
    char PITCH_PID_key;
    
    float pitch_KP = pitchLoop().getControllerP();       // load current pitch value
    float pitch_KI = pitchLoop().getControllerI();       // load current pitch value
    float pitch_KD = pitchLoop().getControllerD();       // load current pitch value

    float pitch_freq = pitchLoop().getFilterFrequency();
    float pitch_deadband = pitchLoop().getDeadband();
 
    // print the menu
    serialPrint("\n\rPITCH (Battery Motor O.L.) PID gain settings (MENU)");
    serialPrint("\n\rAdjust PID settings with the following keys: P I D. Filter = F, deadband = B.\n\r");
    serialPrint("\n\r(Hit shift + X to exit w/o saving.  Hit shift + S to save.)\n\n\n\r");
    serialPrint("PITCH P: %3.3f, I: %3.3f, D %3.3f, offset: %3.1f mm (filter: %0.2f, deadband: %0.2f)\r\n", pitchLoop().getControllerP(), pitchLoop().getControllerI(), pitchLoop().getControllerD(), pitchLoop().getOutputOffset(),pitchLoop().getFilterFrequency(),pitchLoop().getDeadband());
    
    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            PITCH_PID_key = xbee().getc();      //12/20/2018 make into one function that you call and return the character
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // handle the user's key input
        if (PITCH_PID_key == 'S') { // user wants to save these modified values
            // set values
            pitchLoop().setControllerP(pitch_KP);
            pitchLoop().setControllerI(pitch_KI);
            pitchLoop().setControllerD(pitch_KD);
            
            pitchLoop().setFilterFrequency(pitch_freq);
            pitchLoop().setDeadband(pitch_deadband);
 
            // save to "PITCH.TXT" file (doesn't modify neutral position)
            configFileIO().savePitchData(pitch_KP, pitch_KI, pitch_KD, _neutral_batt_pos_mm, pitch_freq, pitch_deadband);
            
            serialPrint("PITCH P: %3.3f, I: %3.3f, D %3.3f, zeroOffset: %3.1f mm (filter: %0.2f, deadband: %0.2f)\r\n", pitchLoop().getControllerP(), pitchLoop().getControllerI(), pitchLoop().getControllerD(), pitchLoop().getOutputOffset(),pitchLoop().getFilterFrequency(),pitchLoop().getDeadband());

            _pitch_KP = pitchLoop().getControllerP();       // load current pitch value
            _pitch_KI = pitchLoop().getControllerI();       // load current pitch value
            _pitch_KD = pitchLoop().getControllerD();       // load current pitch value
        }
        else if (PITCH_PID_key == 'X') {    
            break;  //exit the while loop
        }
        else if (PITCH_PID_key == 'P') {
            serialPrint(">> Type in proportional gain with keyboard.\n\r");
            pitch_KP = getFloatUserInput();
        }
        else if (PITCH_PID_key == 'I') {
            serialPrint(">> Type in integral gain with keyboard.\n\r");
            pitch_KI = getFloatUserInput();
        }
        else if (PITCH_PID_key == 'D') {
            serialPrint(">> Type in derivative gain with keyboard.\n\r");
            pitch_KD = getFloatUserInput();
        }
        else if (PITCH_PID_key == 'F') {
            serialPrint(">> Type in FILTER FREQUENCY with keyboard.\n\r");
            pitch_freq = getFloatUserInput();
        }
        else if (PITCH_PID_key == 'B') {
            serialPrint(">> Type in DEADBAND with keyboard.\n\r");
            pitch_deadband = getFloatUserInput();
        }
        else {
            serialPrint("\n\rPITCH: [%c] This key does nothing here.                                  \r", PITCH_PID_key);
        }
    }
}

void StateMachine::keyboard_menu_HEADING_PID_settings() {    
    char HEADING_PID_key;
    
    float heading_KP = headingLoop().getControllerP();
    float heading_KI = headingLoop().getControllerI();
    float heading_KD = headingLoop().getControllerD(); 
       
    float heading_offset_deg = headingLoop().getOutputOffset();
    float heading_freq = headingLoop().getFilterFrequency();
    float heading_deadband = headingLoop().getDeadband();
 
    // print the menu
    serialPrint("\n\rHEADING (rudder outer loop) PID gain settings (MENU)");
    serialPrint("\n\rAdjust PID settings with the following keys: P I D. Filter = F, deadband = B.\n\r");
    serialPrint("\n\r   Adjust zero offset with O (oh).");
    serialPrint("\n\r(Hit shift + X to exit w/o saving.  Hit shift + S to save.\n\r");
    serialPrint("HEADING P: %3.3f, I: %3.3f, D %3.3f, zeroOffset: %3.1f deg (filter: %0.2f, deadband: %0.2f)\r\n\r\n", headingLoop().getControllerP(),headingLoop().getControllerI(),headingLoop().getControllerD(),headingLoop().getOutputOffset(),headingLoop().getFilterFrequency(),headingLoop().getDeadband());
 
    // handle the key presses
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            HEADING_PID_key = xbee().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
 
        // handle the user's key input     
        if (HEADING_PID_key == 'S') { // user wants to save the modified values
            // set global values
            headingLoop().setControllerP(heading_KP);
            headingLoop().setControllerI(heading_KI);
            headingLoop().setControllerD(heading_KD);
            headingLoop().setOutputOffset(heading_offset_deg);
            headingLoop().setFilterFrequency(heading_freq);
            headingLoop().setDeadband(heading_deadband);
 
            // save pitch PID values for outer loop (must save neutral position also)
            configFileIO().saveHeadingData(heading_KP, heading_KI, heading_KD, heading_offset_deg, heading_freq, heading_deadband);    //_neutral_heading_pos_deg);
            serialPrint("HEADING P: %3.3f, I: %3.3f, D %3.3f, zeroOffset: %3.1f deg (filter: %0.2f, deadband: %0.2f)\r\n\r\n", headingLoop().getControllerP(),headingLoop().getControllerI(),headingLoop().getControllerD(),headingLoop().getOutputOffset(),headingLoop().getFilterFrequency(),headingLoop().getDeadband());
        }
        else if (HEADING_PID_key == 'X') {    
            break;  //exit the while loop
        }
        
        else if (HEADING_PID_key == 'P') {
            heading_KP = getFloatUserInput();;
        }
        else if (HEADING_PID_key == 'I') {
            heading_KI = getFloatUserInput();
        }
        else if (HEADING_PID_key == 'D') {
            heading_KD = getFloatUserInput();
        }
        else if (HEADING_PID_key == 'F') {
            serialPrint(">> Type in FILTER FREQUENCY with keyboard.\n\r");
            heading_freq = getFloatUserInput();
        }
        else if (HEADING_PID_key == 'B') {
            serialPrint(">> Type in DEADBAND with keyboard.\n\r");
            heading_deadband = getFloatUserInput();
        }
        else if (HEADING_PID_key == 'O') {
            heading_offset_deg = getFloatUserInput();
        }
        else {
            serialPrint("HEADING SETUP: [%c] This key does nothing here.                           \r", HEADING_PID_key);
        }
    }
}
 
float StateMachine::getDepthCommand() {
    return _depth_command;
}
 
float StateMachine::getPitchCommand() {
    return _pitch_command;
}

float StateMachine::getDepthReading() {
    return _depth_reading;
}
 
float StateMachine::getPitchReading() {
    return _pitch_reading;
}

float StateMachine::getTimerReading() {
    return _timer_reading;
}

void StateMachine::setState(int input_state) {
    _state = input_state;
    
    _isTimeoutRunning = false;  //to start each state you have to reset this
}
 
int StateMachine::getState() {
    return _state;  //return the current state of the system
}
 
void StateMachine::setTimeout(float input_timeout) {
    _timeout = input_timeout;
}
 
void StateMachine::setDepthCommand(float input_depth_command) {
    _depth_command = input_depth_command;
}
 
void StateMachine::setPitchCommand(float input_pitch_command) {
    _pitch_command = input_pitch_command;
}
 
void StateMachine::setNeutralPositions(float batt_pos_mm, float bce_pos_mm) {
    _neutral_batt_pos_mm = batt_pos_mm;
    _neutral_bce_pos_mm = bce_pos_mm;
    
    serialPrint("Neutral Buoyancy Positions: batt: %0.1f, bce: %0.1f\r\n",_neutral_batt_pos_mm,_neutral_bce_pos_mm);
}
 
//process one state at a time
void StateMachine::getDiveSequence() {
    //iterate through this sequence using the FSM
    currentStateStruct.state = sequenceController().sequenceStructLoaded[_multi_dive_counter].state;
    currentStateStruct.timeout = sequenceController().sequenceStructLoaded[_multi_dive_counter].timeout;
    currentStateStruct.depth = sequenceController().sequenceStructLoaded[_multi_dive_counter].depth;
    currentStateStruct.pitch = sequenceController().sequenceStructLoaded[_multi_dive_counter].pitch;
    
    _timeout = currentStateStruct.timeout;  //set timeout before exiting this function
}

void StateMachine::printCurrentSdLog() {
    serialPrint("SD card log work in progress\r\n");
    //might be worth saving the last few logs to the MBED...
}

// 06/06/2018
float StateMachine::getFloatUserInput() {
    float float_conversion = 0.0;
    
    while(1) {
        bool valid_input = false;                   //flag for valid or invalid input
        
        serialPrint("\n\rPlease enter your number below and press ENTER:\r\n");
        char user_string [80];                      //variable to store input as a character array
    
        xbee().scanf("%s", user_string);              //read formatted data from stdin
        serialPrint("\n\n\ruser_string was <%s>\r\n", user_string);
        
        //check through the string for invalid characters (decimal values 43 through 57)
        for (int c = 0; c < strlen(user_string); c++) {
            //serialPrint("character is [%c]\r\n", user_string[c]);   //debug
            if (user_string[c] >= 43 and user_string[c] <= 57) {
                //serialPrint("VALID CHARACTER!\r\n"); //debug
                ;
            }
            else {
                serialPrint("INVALID INPUT!\r\n");
                break;
            }
            
            if (c == (strlen(user_string) - 1)) {
                valid_input = true;
            }
        }
        
        if (valid_input) {
            float_conversion = atof(user_string);
            serialPrint("VALID INPUT!  Your input was: %3.3f (PRESS \"S\" (shift + S) to save!)\r\n", float_conversion);
            break;
        }
    }
    
    return float_conversion;
}

float StateMachine::getTimerValue() {
    return _fsm_timer;    
}

void StateMachine::logFileMenu() {    
    char FILE_MENU_key;
    
    // print the menu
    serialPrint("\n\r>>> LOG FILE MENU. Y = Yes, erase file (and exit).  N = No, keep file (and exit).  P = Print file size. T = Tare depth sensor.<<<\n\r");
    
    // handle the key presses
    // NOTE TO SELF, is there a way to read both serial ports at once?? 02/13/19
    while(1) {
        // get the user's keystroke from either of the two inputs
        if (xbee().readable()) {
            serialPrint("\n\r>>> LOG FILE MENU. Y = Yes, erase file (and exit).  N = No, keep file (and exit).  P = Print file size. T = Tare depth sensor.<<<\n\r");
            FILE_MENU_key = xbee().getc();
        }
        if (pc().readable()) {
            serialPrint("\n\r>>> LOG FILE MENU. Y = Yes, erase file (and exit).  N = No, keep file (and exit).  P = Print file size. T = Tare depth sensor.<<<\n\r");
            FILE_MENU_key = pc().getc();
        }
        else {
            continue; // didn't get a user input, so keep waiting for it
        }
    
        // handle the user's key input
        if (FILE_MENU_key == 'P') { // user wants to save these modified values
            serialPrint("\n\r>> Printing log file size!\n\r");
            wait(2);
            mbedLogger().getFileSize("/local/LOG000.csv");
        }
        else if (FILE_MENU_key == 'Y') {
            serialPrint("\n\r>> Erasing MBED LOG FILE!\n\r");
            wait(2);
            mbedLogger().eraseFile();
            break;
        }
        else if (FILE_MENU_key == 'N') {
            serialPrint("\n\r>> EXITING MENU. Log file intact.\n\r");
            wait(2);
            break;
        }
        else if (FILE_MENU_key == 'T') {
            serialPrint("\n\r>> Taring pressure sensor!\n\r");
            wait(1);
            //TARE depth sensor (pressure transducer)
            depth().tare(); // tares to ambient (do on surface)
            break;
        }
        else {
            serialPrint("\n\r[%c] This key does nothing here.                                  \r", FILE_MENU_key);
        }
    }
}