/*******************************************************************************
Author:           Troy Holley
Title:            Gui.cpp
Date:             08/01/2018

Description/Notes:

Used to interact with a Python GUI.

Work in progress on GUI packet output.  Disabled for now.

*******************************************************************************/

#include "Gui.hpp"
#include "StaticDefs.hpp"

// 0x FE ED CMD PL CRC1 CRC2

// SIT_IDLE, CHECK_TUNING, FIND_NEUTRAL, DIVE, RISE, FLOAT_LEVEL, FLOAT_BROADCAST, EMERGENCY_CLIMB, MULTI_DIVE, MULTI_RISE, POSITION_DIVE, POSITION_RISE, KEYBOARD, TRANSMIT_LOG, RECEIVE_SEQUENCE, PITCH_TUNER_DEPTH, PITCH_TUNER_RUN, SEND_STATUS

Gui::Gui() {
}

//calculate the crc with an integer array
int Gui::guiCalcCrc1(int *input_array, int array_length) {    
    int crc = 0;
    
    int crc_table[] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    for (int z = 0; z < array_length; z++) {
        crc = (crc_table[(input_array[z] ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;
    }
    
    return crc / 256; //second-to-last byte
}

int Gui::guiCalcCrc2(int *input_array, int array_length) {    
    int crc = 0;
    int crc_table[] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    for (int z = 0; z < array_length; z++) {
        crc = (crc_table[(input_array[z] ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;
    }
    return crc % 256; //second-to-last byte
}

int Gui::calcCrcOneVector(vector <int> crc_packet) {
    std::vector<int>::iterator itr;
    
    //can't initialize the table in the constructor in c++
    int crc_table [256] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    int crc = 0;
    for (itr = crc_packet.begin(); itr < crc_packet.end(); itr++)
        crc = (crc_table[(*itr ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;

    return crc / 256; //second-to-last byte
}

int Gui::calcCrcTwoVector(vector <int> crc_packet) {
    std::vector<int>::iterator itr;
    
    int crc_table [256] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    int crc = 0;
    for (itr = crc_packet.begin(); itr < crc_packet.end(); itr++)
        crc = (crc_table[(*itr ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;

    return crc % 256; //last byte
}

void Gui::transmitDataPacket(vector <int> full_packet) {
    std::vector<int>::iterator itr;
    
    //transmit full packet
    for (itr = full_packet.begin(); itr < full_packet.end(); itr++) {
        xbee().putc(*itr); //send integers over serial port one byte at a time
    }
}

// get command one byte at a time
void Gui::getCommandFSM() {
    static int fsm_command = -1;           
    static int command_state = HEADER_FE;     //state in switch statement
    int incoming_byte = -1;                     //reset each time a character is read
    static int input_packet[4];                 //changed from char in previous iteration 03/28/2018
    static int command_crc_one = 0;            //hold crc values until they're reset with calculations
    static int command_crc_two = 0;
    static int command_packet_size = -1;
    
    if (xbee().readable()) {
        incoming_byte = xbee().getc();
        
        switch(command_state) {
        case HEADER_FE:
            //continue processing
            if (incoming_byte == 254){   //FE
                command_state = HEADER_ED;
                input_packet[0] = 254;
            }
            //did not receive byte 1
            else {
                command_state = HEADER_FE;    //go back to checking the first packet
            }
            break;
                        
        case HEADER_ED:
            if(incoming_byte == 237) {   //ED
                command_state = COMMAND_PACKET;
                input_packet[1] = 237;
            }
            
            //did not get the correct second byte
            else {
                command_state = HEADER_FE;    //go back to checking the first packet
            }
                
            break;
        
        case COMMAND_PACKET:    
            // EMERGENCY_CLIMB, MULTI_DIVE, MULTI_RISE, POSITION_DIVE, POSITION_RISE, KEYBOARD, TRANSMIT_LOG, RECEIVE_SEQUENCE, PITCH_TUNER_DEPTH, PITCH_TUNER_RUN, SEND_STATUS
        
            //note these ENUM values come from the StateMachine
        
            if (incoming_byte == 1)                  
                fsm_command = CHECK_TUNING;
            else if (incoming_byte == 2)
                fsm_command = FIND_NEUTRAL;
            else if (incoming_byte == 3)
                fsm_command = DIVE;
            else if (incoming_byte == 4)
                fsm_command = RISE;
            else if (incoming_byte == 5)
                fsm_command = FLOAT_LEVEL;    
            else if (incoming_byte == 6)
                fsm_command = FLOAT_BROADCAST;
            else if (incoming_byte == 7)
                fsm_command = EMERGENCY_CLIMB;
            else if (incoming_byte == 8)
                fsm_command = MULTI_DIVE;                
            else if (incoming_byte == 9)
                fsm_command = MULTI_RISE;
            else if (incoming_byte == 10)
                fsm_command = POSITION_DIVE;
            else if (incoming_byte == 11)
                fsm_command = POSITION_RISE;
        //SKIP 13    
            else if (incoming_byte == 12)
                fsm_command = TX_MBED_LOG;
            else if (incoming_byte == 13)
                fsm_command = RX_SEQUENCE;
            else {
                command_state = HEADER_FE;    //go back to checking the first packet
            }

            //state
            command_state = FSM_PACKET_SIZE;
            
            input_packet[2] = incoming_byte;
            
            break;

        case FSM_PACKET_SIZE:            
            if (incoming_byte >= 0) {
                command_packet_size = incoming_byte;
            }
            
            command_state = FSM_CRC_ONE;
            
            input_packet[3] = incoming_byte;
            
            break;
            
        case FSM_CRC_ONE:        
            command_crc_one = guiCalcCrc1(input_packet, 4);        //calc CRC 1 from the input packet (size 4)
            
            if (incoming_byte == command_crc_one) {
                command_state = FSM_CRC_TWO;
            }
            
            else
                command_state = HEADER_FE;
            //or state remains the same?
            
            break;
        
        case FSM_CRC_TWO:            
            command_crc_two = guiCalcCrc2(input_packet, 4);        //calc CRC 2 from the input packet (size 4)
            
            //check if CRC TWO is correct (then send full packet)
            if (incoming_byte == command_crc_two) {                
                //set state of statemachine
                stateMachine().setState(fsm_command);                
                xbee().printf("CRC 1 and CRC 2 IS GOOD! fsm_command is %d\n\r", fsm_command);
            }
            
            command_state = HEADER_FE;
            
            break;
        }   /* switch statement complete */
    } /* end of pc readable */
}

//remove class vector?

void Gui::updateGUI() {
    float roll_value = imu().getRoll();
    float pitch_value = imu().getPitch();
    float heading_value = imu().getHeading();
    float depth_value = depthLoop().getPosition(); //filtered depth position
    float timer_value = stateMachine().getTimerValue();
    
    xbee().printf("roll %0.2f / pitch %0.2f / heading %0.2f / depth %0.2f / timer %0.2f\n\r", roll_value, pitch_value, heading_value, depth_value, timer_value);
    
    vector <int> gui_update_packet;
    
    //ROLL PITCH HEADING(YAW) DEPTH TIMER (sending all at once, at one second intervals)
    
    //take float value, convert to unsigned char, reverse and send
    const unsigned char * ptr_roll_value = reinterpret_cast<const unsigned char*>(&roll_value);
    const unsigned char * ptr_pitch_value = reinterpret_cast<const unsigned char*>(&pitch_value);
    const unsigned char * ptr_heading_value = reinterpret_cast<const unsigned char*>(&heading_value);
    const unsigned char * ptr_depth_value = reinterpret_cast<const unsigned char*>(&depth_value);
    const unsigned char * ptr_timer_value = reinterpret_cast<const unsigned char*>(&timer_value);
    //xbee().printf("DEBUG: timer_value is %f or HEX: %x %x %x %x\n\r",timer_value,ptr_timer_value[3],ptr_timer_value[2],ptr_timer_value[1],ptr_timer_value[0]);
    
    // BE AD GUI GUI LENGTH DATA DATA CC CC
    
    gui_update_packet.clear();
    
    //DATA PACKET HEADER
    gui_update_packet.push_back(121);  // y = 0x79
    gui_update_packet.push_back(113);  // q = 0x71
    
    gui_update_packet.push_back(204);  // 0xCC 
    gui_update_packet.push_back(204);  // 0xCC
    
    gui_update_packet.push_back(20);  // 0x14 (length)
    
    gui_update_packet.push_back(ptr_roll_value[3]);
    gui_update_packet.push_back(ptr_roll_value[2]);
    gui_update_packet.push_back(ptr_roll_value[1]);
    gui_update_packet.push_back(ptr_roll_value[0]);
    
    gui_update_packet.push_back(ptr_pitch_value[3]);
    gui_update_packet.push_back(ptr_pitch_value[2]);
    gui_update_packet.push_back(ptr_pitch_value[1]);
    gui_update_packet.push_back(ptr_pitch_value[0]);
    
    gui_update_packet.push_back(ptr_heading_value[3]);
    gui_update_packet.push_back(ptr_heading_value[2]);
    gui_update_packet.push_back(ptr_heading_value[1]);
    gui_update_packet.push_back(ptr_heading_value[0]);
    
    gui_update_packet.push_back(ptr_depth_value[3]);
    gui_update_packet.push_back(ptr_depth_value[2]);
    gui_update_packet.push_back(ptr_depth_value[1]);
    gui_update_packet.push_back(ptr_depth_value[0]);
    
    gui_update_packet.push_back(ptr_timer_value[3]);
    gui_update_packet.push_back(ptr_timer_value[2]);
    gui_update_packet.push_back(ptr_timer_value[1]);
    gui_update_packet.push_back(ptr_timer_value[0]);
    
    //CRC CALCULATION
    int crc_one = calcCrcOneVector(gui_update_packet);
    int crc_two = calcCrcTwoVector(gui_update_packet);
    
    //place the crc bytes into the data packet that is transmitted
    gui_update_packet.push_back(crc_one);
    gui_update_packet.push_back(crc_two);
    
    //transmit the full packet
    transmitDataPacket(gui_update_packet);
}