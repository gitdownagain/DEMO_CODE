#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP
 
#include "mbed.h"
#include <vector>
 
extern "C" void mbed_reset();           // utilized to reset the mbed
 
// main finite state enumerations
enum {
    SIT_IDLE,               // stops both motors, exits after a keyboard input
    CHECK_TUNING,           // runs the system to the positions specified in the files
    FIND_NEUTRAL,           // dives to depth at zero pitch, exits when stable
    DIVE,                   // dives to depth at negative pitch, exits when crossing a defined depth
    RISE,                   // rises to surface at positive pitch, exits when near surface
    POSITION_DIVE,          // NEW POSITION ONLY COMMANDS (inner loop)
    POSITION_RISE,          // NEW POSITION ONLY COMMANDS (inner loop)
    FLOAT_LEVEL,            // bce position to float, pitch loop active at zero, exits when stable near zero pitch
    FLOAT_BROADCAST,        // bce position to float, batt position forward to hold tail up, exits when actuators done
    EMERGENCY_CLIMB,        // bce position to full rise, batt position to full aft, exits when at surface
    MULTI_DIVE,             // multi-dive sequence
    MULTI_RISE,             // multi-rise sequence
    KEYBOARD,               // "state" for tracking only
    TX_MBED_LOG,
    RX_SEQUENCE,
    MANUAL_TUNING
};
 
// find_neutral finite state machine enumerations
enum {
    NEUTRAL_SINKING,        // increment the bce until really start sinking
    NEUTRAL_SLOWLY_RISE,    // once sinking, arrest the sink
    NEUTRAL_CHECK_PITCH,    // find level again, then save the data and exit
    NEUTRAL_EXIT,            // sub-FSM has completed all checks
};

// test idea
 
//struct for saving the data
struct currentSequenceStruct {
    int state;      //for the current StateMachine, states are ID-ed with enumeration
    float timeout;
    float depth;
    float pitch;
};
 
class StateMachine {
public:
    StateMachine();
    
    int runStateMachine();
    
    void printSimpleMenu();  // simple menu
    void printDebugMenu();   // debug menu
    
    void keyboard();
    void keyboardInput(char user_input);
    
    void keyboard_menu_MANUAL_TUNING();
    void keyboard_menu_STREAM_STATUS();
    void keyboard_menu_DEBUG_PID();         //new 01/08/2019
    
    void keyboard_menu_CHANNEL_READINGS();
    void keyboard_menu_POSITION_READINGS();
    void keyboard_menu_RUDDER_SERVO_settings();
    void keyboard_menu_HEADING_PID_settings();
    void keyboard_menu_COUNTS_STATUS();             //remove?
    
    void keyboard_menu_BCE_PID_settings();
    void keyboard_menu_BATT_PID_settings();
    void keyboard_menu_DEPTH_PID_settings();
    void keyboard_menu_PITCH_PID_settings();
    
    float getDepthCommand();
    float getPitchCommand();
    float getDepthReading();
    float getPitchReading();
    float getTimerReading();
    
    int runNeutralStateMachine();           //substate returns the state (which is used in overall FSM)
    
    int getState();
    void setState(int input_state);
    
    void setTimeout(float input_timeout);    
    void setDepthCommand(float input_depth_command);    
    void setPitchCommand(float input_pitch_command);
    
    void setNeutralPositions(float batt_pos_mm, float bce_pos_mm);
    
    void getDiveSequence(); //used in multi-dive sequence with public variables for now
    
    void runActiveNeutralStateMachine();    //new neutral substate returns the state (which is used in overall FSM)
    
    float * getLoggerArray();               //delete soon
    
    void printDirectory();
    void printCurrentSdLog();               //more tricky for SD card, work in progress
    
    void createNewFile();
    
    void transmitData();
    
    float * dataArray();
    
//GUI UPDATE FUNCTIONS
    float getTimerValue();
    
    void logFileMenu();         //instead of immediately erasing log files, NRL Stennis suggests a confirmation 
    
private:
    bool _debug_menu_on;         // default is false to show simple menu, debug allows more tuning and has a lot of keyboard commands

    int _timeout;                // generic timeout for every state, seconds
    float _pitchTolerance;       // pitch angle tolerance for neutral finding exit criteria
    float _bceFloatPosition;     // bce position for "float" states
    float _battFloatPosition;    // batt position for "broadcast" state
    
    float _depth_command;       // user keyboard depth
    float _pitch_command;       // user keyboard pitch
    float _heading_command;     // user keyboard heading
    
    float _depth_reading;       // depth reading (to get the readings at the same time)
    float _pitch_reading;       // pitch reading (to get the readings at the same time)   
    float _timer_reading;       // pitch reading (to get the readings at the same time)
    
    Timer _fsm_timer;               //timing variable used in class
    
    volatile int _state;                 // current state of Finite State Machine (FSM)
    int _previous_state;        // record previous state
    int _sub_state;             // substate on find_neutral function
    int _previous_sub_state;    // previous substate so that what goes into the sub-state is not being changed as it is processed
    float _neutral_timer;  // keep time for rise/sink/level timer incremnets
    
    bool _isTimeoutRunning;
    
    bool _isSubStateTimerRunning;
    
    float _neutral_bce_pos_mm;
    float _neutral_batt_pos_mm;
    
    int _multi_dive_counter;
    
    currentSequenceStruct currentStateStruct;   //type_of_struct struct_name
    
    float _depth_KP;
    float _depth_KI;
    float _depth_KD;
    
    float _pitch_KP;
    float _pitch_KI;
    float _pitch_KD;

    int _state_array[256];                         //used to print out the states
    int _state_array_counter;                       //used to iterate through state records    
    int _substate_array[256];                      //used to print out the sub-states
    int _substate_array_counter;                    //used to iterate through sub-state records
    
    int _substate;
    int _previous_substate;
    
    float _max_recorded_depth_neutral;
    float _max_recorded_depth_dive;
    
    float _neutral_sink_command_mm;                 //defaults for neutral finding sub-FSM
    float _neutral_rise_command_mm;
    float _neutral_pitch_command_mm;
    
    float _max_recorded_auto_neutral_depth;
    
    bool _is_log_timer_running;
    float _log_timer;
    
    float _BCE_dive_offset;                         // NEW COMMANDS FOR POSITION CONTROLLER
    float _BMM_dive_offset;
    
    float getFloatUserInput();
    
    //new
    float _batt_filter_freq;
    float _bce_filter_freq;
    float _pitch_filter_freq;
    float _depth_filter_freq;
    float _heading_filter_freq;
    
    float _batt_deadband;
    float _bce_deadband;
    float _pitch_deadband;
    float _depth_deadband;
    float _heading_deadband;
};
 
#endif