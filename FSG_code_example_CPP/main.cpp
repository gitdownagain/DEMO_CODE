/*******************************************************************************
Author:           Troy Holley
Title:            main.cpp
Date:             02/14/2018 (Last modified)

Description/Notes:

The main function loads all of the system settings and instantiates all of the 
hardware and electronics.

The "system ticker" here controls the timing for all of the electronics and
logging on the system.  This was a modification of previous behavior that was
causing the system to lock up as multiple tickers (interrupts) were causing 
unknown behavior.

This loads configuration files at the start--takes a second or two to load a few
files. Also initiliazes the file systems on the MBED and SD card.

Commented out SD card because we were testing hardware without an SD card at WH.

*******************************************************************************/

#include "mbed.h"
#include "StaticDefs.hpp"

// Macro to output to both serial ports
#define serialPrint(fmt, ...) pc().printf(fmt, ##__VA_ARGS__);xbee().printf(fmt, ##__VA_ARGS__)

Ticker systemTicker;
bool setup_complete = false;
volatile unsigned int bTick = 0;
volatile unsigned int timer_counter = 0;
DigitalOut ssr_cntl(p29);//pin used for switching between on-board pressure transducer and off-board altimeter (just added on last PCB revision)

//B sically this makes sure you're reading the datat in one instance (not while it's changing)
static unsigned int read_ticker(void) {
    unsigned int val = bTick;
    if(val)
        bTick = 0;
    return(val);
}

// Variables used to make sure each iteration of these loops are not conflicting with one another
//used so the compiler does not optimize this variable (load from memory, do not assume state of variable)
volatile bool fsm_loop = false;
volatile bool log_loop = false; 

void loop_trigger() { fsm_loop = true;} // loop trigger (used in while loop)
void log_loop_trigger() { log_loop = true;} // log loop trigger (used in while loop)

// Maintains the state of the system and whether or not the file is opened or closed
static int current_state = 0;     
static bool file_opened = false;

// Run the state machine based on the timing set in the main loop
void FSM() {                    // FSM loop runs at 10 hz
    if(fsm_loop) {
        // led one removed
        fsm_loop = false;       // wait until the loop rate timer fires again
        current_state = stateMachine().runStateMachine();       //running State Machine. Returns 0 if sitting idle or keyboard press (SIT_IDLE state).
    }
}

// Run the logging based on the timing set in the main loop
void log_function() {    
    // log loop runs at 1 hz
    if (log_loop) {
        //when the state machine is not in SIT_IDLE state (or a random keyboard press)
        
        if(current_state != 0) {
            if (!file_opened) {                                 //if the log file is not open, open it
                mbedLogger().appendLogFile(current_state, 0);   //open MBED file once
                //sdLogger().appendLogFile(current_state, 0);     //open SD file once
                           
                file_opened = true;                             //stops it from continuing to open it

                serialPrint(">>>>>>>> Recording. Log file opened. <<<<<<<<\n\r");
            }
            
            //record to Mbed file system   
            
            mbedLogger().appendLogFile(current_state, 1);    //writing data
            //sdLogger().appendLogFile(current_state, 1);    //writing data
        }
        
        //when the current FSM state is zero (SIT_IDLE), close the file
        else {
            //this can only happen once
            if (file_opened) {
                //WRITE ONCE
                mbedLogger().appendLogFile(current_state, 1);   //write the idle state, then close
                //sdLogger().appendLogFile(current_state, 1);     //write the idle state, then close
                
                mbedLogger().appendLogFile(current_state, 0);    //close log file
                //sdLogger().appendLogFile(current_state, 0);    //close log file
                
                file_opened = false;
                
                serialPrint(">>>>>>>> Stopped recording. Log file closed. <<<<<<<<\n\r");
            }
        }
    }   //END OF LOG LOOP8
    
    log_loop = false;   // wait until the loop rate timer fires again
}

//single system timer to run hardware/electronics timing
static void system_timer(void) {
    bTick = 1;
    
    timer_counter++;
    
    //only start these updates when everything is properly setup (through setup function)
    if (setup_complete) {
        if ( (timer_counter % 5) == 0) {    //this runs at 0.005 second intervals (200 Hz)
            adc().update();  //every iteration of this the A/D converter runs   //now this runs at 0.01 second intervals 03/12/2018
        }
        
        if ( (timer_counter % 10) == 0) {   // runs at 100 Hz
            bce().update();      //update() inside LinearActuator class (running at 0.01 second intervals)
            batt().update();
        }
        
        if ( (timer_counter % 20) == 0 ) {    // 0.02 second intervals
            rudder().runServo();
        }
        
        if ( (timer_counter % 50) == 0 ) {    // 0.05 second intervals 
            imu().runIMU();
        }
                
        if ( (timer_counter % 100) == 0) {     // 100,000 microseconds = 0.1 second intervals
            depthLoop().runOuterLoop();
            pitchLoop().runOuterLoop();
            headingLoop().runOuterLoop();
        }
        
        if ( (timer_counter % 1000) == 0) {        // update at 1.0 second intervals
            //gui().updateGUI();
        }
        
        if ( (timer_counter % 30000) == 0) {        // update at 30.0 second intervals
            //pc().printf("XB!\n");
        }
    }
}

void setup() {
    //initialize both serial ports
    pc().baud(115200);
    xbee().baud(115200);

    // start up the system timer (with ticker function above)
 
    // set up and start the adc. This runs on a fixed interval and is interrupt driven
    adc().initialize();
    //one central interrupt is updating the ADC (not using the start function)
    
    // setup and run the rudder(servo) pwm signal (start the ticker)
    //rudder().init();
    serialPrint("Rudder servo initialized!\n\r");
    
    // set up and start the imu. This polls in the background
    imu().initialize();
    //imu().start();
    
    // construct the MBED local file system
    local();
    
    // construct the SD card file system TEST 10/23/18
    //sd_card();
 
    // load config data from files
    configFileIO().load_BATT_config();      // load the battery mass mover parameters from the file "batt.txt"
    configFileIO().load_BCE_config();       // load the buoyancy engine parameters from the file "bce.txt"
    
    configFileIO().load_DEPTH_config();     // load the depth control loop parameters from the file "depth.txt" (contains neutral position)
    configFileIO().load_PITCH_config();     // load the depth control loop parameters from the file "pitch.txt" (contains neutral position)
    
    configFileIO().load_RUDDER_config();    // load the rudder servo inner loop parameters from the file "SERVO.txt"
    configFileIO().load_HEADING_config();   // load the rudder servo outer loop HEADING control parameters from the file "HEADING.txt" (contains neutral position)
 
    // set up the linear actuators.  adc has to be running first.
    bce().setPIDHighLimit(bce().getTravelLimit());     //travel limit of this linear actuator
    bce().init();
    //NEW 01/08
    bce().setPosition_mm(bce().getPosition_mm());
    //NEW 01/08
    //bce().start();  //removed start, it's handled by the interrupt
    bce().runLinearActuator();
    bce().pause(); // start by not moving
 
    batt().setPIDHighLimit(batt().getTravelLimit());    //travel limit of this linear actuator
    batt().init();
    //NEW 01/08
    batt().setPosition_mm(batt().getPosition_mm());
    //NEW 01/08
    //batt().start();//removed start, it's handled by the interrupt
    batt().runLinearActuator(); // _init = true;
    batt().pause(); // start by not moving
 
    // set up the depth, pitch, and rudder outer loop controllers
    depthLoop().init();
    //removed start, it's handled by the interrupt
    depthLoop().setCommand(stateMachine().getDepthCommand());
 
    pitchLoop().init();
    //removed start, it's handled by the interrupt
    pitchLoop().setCommand(stateMachine().getPitchCommand());                   /////// IS THIS THE WRONG WAY TO DO THIS?
    
    headingLoop().init();           
    //removed start, it's handled by the interrupt
    //headingLoop().setCommand(stateMachine().getHeadingCommand());         // FIX LATER
    //heading flag that adjust the PID error is set in the constructor
    
    //systemTicker.attach_us(&system_timer, 10000);         // Interrupt timer running at 0.01 seconds       (slower than original ADC time interval)
    
    
 
    // show that the PID gains are loading from the file
    serialPrint("bce    P:%6.2f, I:%6.2f, D:%6.2f, zero %3i, limit %6.1f mm, slope %0.5f  \r\n", bce().getControllerP(), bce().getControllerI(), bce().getControllerD(), bce().getZeroCounts(), bce().getTravelLimit(), bce().getPotSlope());
    serialPrint("batt   P:%6.2f, I:%6.2f, D:%6.2f, zero %3i, limit %6.1f mm, slope %0.5f  \r\n", batt().getControllerP(), batt().getControllerI(), batt().getControllerD(), batt().getZeroCounts(), batt().getTravelLimit(), batt().getPotSlope());
    serialPrint("rudder min pwm: %6.2f, max pwm: %6.2f, center pwm: %6.2f, min deg: %6.2f, max deg: %6.2f\r\n", rudder().getMinPWM(), rudder().getMaxPWM(), rudder().getCenterPWM(), rudder().getMinDeg(), rudder().getMaxDeg());
    
    serialPrint("depth   P:%6.2f, I:%6.2f, D:%6.2f, offset:%6.1f mm \r\n", depthLoop().getControllerP(), depthLoop().getControllerI(), depthLoop().getControllerD(), depthLoop().getOutputOffset());
    serialPrint("pitch   P:%6.2f, I:%6.2f, D:%6.2f, offset:%6.1f mm \r\n", pitchLoop().getControllerP(), pitchLoop().getControllerI(), pitchLoop().getControllerD(), pitchLoop().getOutputOffset());
    serialPrint("heading P: %3.2f, I: %3.2f, D %3.2f, offset: %3.1f deg (deadband: %0.1f)\r\n", headingLoop().getControllerP(), headingLoop().getControllerI(), headingLoop().getControllerD(), headingLoop().getOutputOffset(), headingLoop().getDeadband());
    
    serialPrint("\n\r");
         
    //load sequence from file
    sequenceController().loadSequence();
    
    //set time of logger (to current or close-to-current time)
    mbedLogger().setLogTime();
    //sdLogger().setLogTime();
    
    //create log files if not present on file system
    mbedLogger().initializeLogFile();
    //sdLogger().initializeLogFile(); // works 10/23/18
    
    //hardcoded p29 to be active for the altimeter
    ssr_cntl.write(0);  // Off-board altimeter on! This appears to be flipped from PCB drawing.
    
    setup_complete = true;    
}

int main() {
    //setup electronics/hardware
    setup();    
    
    unsigned int tNow = 0;  //used for ticker timing in main

    serialPrint("\n\n\r 02/13/2019 FSG Woods Hole Test\n\n\r");
    
    systemTicker.attach_us(&system_timer, 1000);         // Interrupt timer running at 0.001 seconds       (slower than original ADC time interval)
        
    while (1) {        
        if( read_ticker() )                         // read_ticker runs at the speed of 10 kHz (adc timing)
        {
            ++tNow;

            //Note to self: Retest data transmission code.
            //This is currently running at 0.1 second intervals (10 hz) and was working well for data transmission
            if (current_state == TX_MBED_LOG or current_state == RX_SEQUENCE) {
                if ( (tNow % 100) == 0 ) {   // 0.1 second intervals  (100 Hz)
                    fsm_loop = true;
                    FSM();
                }
            }
            
            //NOT TRANSMITTING DATA, NORMAL OPERATIONS
            else {  
            //FSM
                if ( (tNow % 100) == 0 ) {   // 0.1 second intervals
                    fsm_loop = true;
                    FSM();
                    
                    //get commands and update GUI
                    gui().getCommandFSM();
                }        
            //LOGGING     
                if ( (tNow % 1000) == 0 ) {   // 1.0 second intervals                
                    log_loop = true;
                    log_function();
                }
            }
        }
    }
}