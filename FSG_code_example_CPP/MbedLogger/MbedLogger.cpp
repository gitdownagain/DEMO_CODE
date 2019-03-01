/*******************************************************************************
Author:           Troy Holley
Title:            MbedLogger.cpp
Date:             12/05/2018

Description/Notes:

Used for logging data to the MBED file system and SD card file system.

Also used to transmit data from the MBED over the XBee radio to a receiver.

And receive dive sequence files from the transmit/receiver program.

*******************************************************************************/

#include "MbedLogger.hpp"
#include "StaticDefs.hpp"

//print to both serial ports using this macro
#define serialPrint(fmt, ...) pc().printf(fmt, ##__VA_ARGS__);xbee().printf(fmt, ##__VA_ARGS__)

  //Timer t;    //used to test time to create packet    //timing debug

MbedLogger::MbedLogger(string file_system_input_string) {
    _file_system_string = file_system_input_string;
    _full_file_path_string = _file_system_string + "LOG000.csv";    //use multiple logs in the future? (after file size is too large)  
    _file_transmission = true;
    _confirmed_packet_number = 0;   //must set this to zero
    _transmit_counter = 0;
    _file_transmission_state = -1;
    _total_number_of_packets = 0;
    _mbed_transmit_loop = false;
    _received_filename = "";
    
    _log_file_line_counter = 0;     //used to set timer in finite state machine based on size of log
    
    //heading string is 254 bytes long, FIXED LENGTH
    _heading_string = "StateStr,St#,TimeSec,DepthCmd,DepthFt,PitchCmd,PitchDeg,RudderPWM,RudderCmdDeg,HeadDeg,bceCmd,bce_mm,battCmd,batt_mm,PitchRateDegSec,depthrate_fps,SystemAmps,SystemVolts,AltChRd,IntPSI,BCE_p,BCi,BCd,BATT_p,BTi,BTd,DEPTH_p,Di,Dd,PITCH_p,Pi,Pd,HEAD_p,Hi,Hd\n";
    _transmit_packet_num = 0;
    _fsm_transmit_complete = false;
    _end_transmit_packet = false;
    _end_sequence_transmission = false;
    _packet_number = 0;     //remove later
}

//this function has to be called for the time to function correctly
void MbedLogger::setLogTime() {
    serialPrint("\n%s log time set.\n\r", _file_system_string.c_str());
    set_time(1551154422);   // Set RTC time to Tuesday, 01 JAN 2019 08:00 AM
}

void MbedLogger::initializeLogFile() {
    string file_name_string = _file_system_string + "LOG000.csv";
    serialPrint("%s file system init\n\r", _file_system_string.c_str());
    
    //try to open this file...
    _fp = fopen(file_name_string.c_str(), "r");
    
    //if the file is empty, create this.
    if (!_fp) {
        _fp = fopen(file_name_string.c_str(), "w");     //write,print,close
        //fprintf(_fp,"state_string,state_ID,timer,depth_cmd,depth_ft,pitch_cmd,pitch_deg,bce_cmd,bce_mm,batt_cmd,batt_mm,pitchRate_degs,depthRate_fps\nempty log file!\n");
        fprintf(_fp,_heading_string.c_str());
        closeLogFile();
    }
    else
        closeLogFile();   //close the opened read file
}

void MbedLogger::closeLogFile() {
    led4() = 1;
    
    if (_fp == NULL){
        serialPrint("MbedLogger: (%s) LOG FILE WAS ALREADY CLOSED!\n\r", _file_system_string.c_str());
    }
    
    else {
        serialPrint("MbedLogger: (%s) CLOSING LOG FILE!\n\r", _file_system_string.c_str());
        
        //close file
        fclose(_fp);
                
        _fp = NULL;     //set pointer to zero
    }    
}

// Open the file for reading or close the file to regain access to the file system
void MbedLogger::appendLogFile(int current_state, int option) {
    //option one means write to file
    
    if (option == 1) {
        if (!_fp) {     //if not present
            _fp = fopen(_full_file_path_string.c_str(), "a");
        }
        
        //record data using the recordData function (takes in the state integer)
        recordData(current_state);
    }
    
    else {
        closeLogFile();
    }
}

// Get the current time from the mbed
int MbedLogger::getSystemTime() {
    time_t seconds = time(NULL);    // Time as seconds since January 1, 1970
    return seconds;
}

void MbedLogger::recordData(int current_state) {
    int data_log_time = mbedLogger().getSystemTime();                 //read the system timer to get unix timestamp
    
    _data_log[0] = depthLoop().getCommand();        //depth command
    _data_log[1] = depthLoop().getPosition();       //depth reading (filtered depth)
    _data_log[2] = pitchLoop().getCommand();        //pitch command
    _data_log[3] = pitchLoop().getPosition();       //pitch reading (filtered pitch)
    _data_log[4] = rudder().getSetPosition_pwm();      //rudder command PWM
    _data_log[5] = rudder().getSetPosition_deg();      //rudder command DEG
    _data_log[6] = headingLoop().getPosition();     //heading reading (filtered heading)
    
    _data_log[7] = bce().getSetPosition_mm();       //BCE command
    _data_log[8] = bce().getPosition_mm();          //BCE reading
    _data_log[9] = batt().getSetPosition_mm();      //Batt command
    _data_log[10] = batt().getPosition_mm();         //Batt reading    
    _data_log[11] = pitchLoop().getVelocity();       // pitchRate_degs (degrees per second)
    _data_log[12] = depthLoop().getVelocity();      // depthRate_fps (feet per second)
    
    _data_log[13] = sensors().getCurrentInput();      // i_in
    _data_log[14] = sensors().getVoltageInput();      // v_in
    _data_log[15] = sensors().getAltimeterChannelReadings();   // Altimeter Channel Readings
    _data_log[16] = sensors().getInternalPressurePSI();   // int_press_PSI
    
    //BCE_p,i,d,freq,deadband
    _data_log[17] = bce().getControllerP();
    _data_log[18] = bce().getControllerI();
    _data_log[19] = bce().getControllerD();
    
    _data_log[20] = batt().getControllerP();
    _data_log[21] = batt().getControllerI();
    _data_log[22] = batt().getControllerD();
    
    _data_log[23] = depthLoop().getControllerP();
    _data_log[24] = depthLoop().getControllerI();
    _data_log[25] = depthLoop().getControllerD();
    
    _data_log[26] = pitchLoop().getControllerP();
    _data_log[27] = pitchLoop().getControllerI();
    _data_log[28] = pitchLoop().getControllerD();
    
    _data_log[29] = headingLoop().getControllerP();
    _data_log[30] = headingLoop().getControllerI();
    _data_log[31] = headingLoop().getControllerD();
    
    string string_state;
    if (current_state == SIT_IDLE)
        string_state = "________SIT_IDLE";
    else if (current_state == FIND_NEUTRAL)
        string_state = "____FIND_NEUTRAL";
    else if (current_state == DIVE)
        string_state = "____________DIVE";
    else if (current_state == RISE)
        string_state = "____________RISE";
    else if (current_state == FLOAT_LEVEL)
        string_state = "_____FLOAT_LEVEL";
    else if (current_state == FLOAT_BROADCAST)
        string_state = "_FLOAT_BROADCAST";          
    else if (current_state == EMERGENCY_CLIMB)
        string_state = "_EMERGENCY_CLIMB";
    else if (current_state == MULTI_DIVE)
        string_state = "______MULTI_DIVE";
    else if (current_state == MULTI_RISE)
        string_state = "______MULTI_RISE";
    else if (current_state == KEYBOARD)
        string_state = "________KEYBOARD";
    else if (current_state == CHECK_TUNING)
        string_state = "____CHECK_TUNING";
    else if (current_state == POSITION_DIVE)
        string_state = "___POSITION_DIVE"; 
    else if (current_state == POSITION_RISE)
        string_state = "___POSITION_RISE";
    else if (current_state == TX_MBED_LOG) 
        string_state = "_____TX_MBED_LOG";
    else if (current_state == RX_SEQUENCE) 
        string_state = "RECEIVE_SEQUENCE";
    else if (current_state == MANUAL_TUNING)    //new 02/13/2019
        string_state = "_____MANUAL_TUNE";   
        
    string blank_space = ""; //to get consistent spacing in the file (had a nonsense char w/o this)
    
    //below this format is used for data transmission, each packet needs to be 254 characters long (not counting newline char)
    
    //verified that this generates the correct line length of 254 using SOLELY an mbed 08/16/2018
    fprintf(_fp, "%16s,%.2d,%10d,%06.1f,%06.1f,%06.1f,%06.1f,%06.0f,%06.0f,%06.1f,%06.1f,%06.1f,%06.1f,%06.1f,%06.1f,%06.1f,%06.3f,%06.2f,%06.0f,%06.2f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f,%06.2f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f,%06.3f\n",
    string_state.c_str(),current_state,data_log_time,
    _data_log[0],_data_log[1],_data_log[2],_data_log[3],_data_log[4],_data_log[5],_data_log[6],_data_log[7],_data_log[8],_data_log[9],_data_log[10],_data_log[11],_data_log[12],_data_log[13],_data_log[14],_data_log[15],
    _data_log[16],_data_log[17],_data_log[18],_data_log[19],_data_log[20],_data_log[21],_data_log[22],_data_log[23],_data_log[24],_data_log[25],_data_log[26],_data_log[27],_data_log[28],_data_log[29],_data_log[30],
    _data_log[31]);

    //each line in the file is 160 characters long text-wise, check this with a file read
}

int MbedLogger::getNumberOfPacketsInCurrentLog() {    
    //takes less than a second to complete, verified 7/24/2018
   
    //open the file
    string file_name_string = _file_system_string + "LOG000.csv";
    _fp = fopen(file_name_string.c_str(), "r");
    
    fseek(_fp, 0L, SEEK_END);
       
    size_t size = ftell(_fp);

    //move the FILE pointer back to the start
    fseek(_fp, 0, SEEK_SET);        // SEEK_SET is the beginning of file
    
    _total_number_of_packets = size/254;
    
    //CLOSE THE FILE
    closeLogFile();
    
    return _total_number_of_packets;
}

//print the files in the MBED directory (useful for debugging)
void MbedLogger::printMbedDirectory() {
    DIR *dir;
    struct dirent   *dp;        //dirent.h is the format of directory entries
    int log_found =0,loop=0;
    long int temp=0;
    
    //char * char_pointer;
    char * numstart;
    //char *numstop;
    
    serialPrint("\n\rPrinting out the directory of device %s\n\r", _file_system_string.c_str());
    
    if ( NULL == (dir   = opendir( _file_system_string.c_str() )) ) {
        serialPrint("MBED directory could not be opened\r\n");
    }                   
    else
    {
        while ( NULL != (dp    = readdir( dir )) )
        {
            if(strncmp(dp->d_name,"LOG",3)==0)
            {
                log_found=1;
                /*numstart = dp->d_name; //Look for third character (do safety)
                numstop = strchr(dp->d_name,'.');
                if(numstart!=NULL&&numstop!=NULL)
                {
                    temp=numstop-numstart;
                }
                else
                    log_found=0; //Something is not right. Ignore
                */
                numstart=dp->d_name+3;
                temp=strtol(numstart,NULL,10);      //add in check to see if this is null (start logs at one)
            }
            else
                log_found=0;
            serialPrint( "%d. %s (log file: %d, %d)\r\n", loop, dp->d_name,log_found,temp); 
            
            loop++;
        }
    }
}

//prints current log file to the screen (terminal)
void MbedLogger::printCurrentLogFile() {
    //open the file for reading
    string file_name_string = _file_system_string + "LOG000.csv";
    
    _log_file_line_counter = 0;

    _fp = fopen(file_name_string.c_str(), "r");
       
    char buffer[500];
    
    //read the file line-by-line and print that to the screen
    serialPrint("\n\rCURRENT MBED LOG FILE /local/Log000.csv:\n\n\r");
    while (!feof(_fp)) {
        // read in the line and make sure it was successful
        if (fgets(buffer,500,_fp) != NULL) {            //stops at new line
            serialPrint("%s\r",buffer);
            _log_file_line_counter++;
        }
    }
    
    //fgets stops when either (n-1) characters are read, the newline character is read,
    // or the end-of-file is reached
    
    //close the file
    closeLogFile();
    serialPrint("\n\rLog file closed. Lines in log file: %d.\n\r", _log_file_line_counter);
}

void MbedLogger::transmitDataPacket() {
    //WRITE the data (in bytes) to the serial port
    for (_it=_data_packet.begin(); _it < _data_packet.end(); _it++) {
        xbee().putc(*_it); //send integers over serial port one byte at a time
    }
}

//transmit log file with fixed length of characters to receiver program
void MbedLogger::transmitPacketNumber(int line_number) {
    int line_size = 254;    //length of lines in the log file, EVERY LINE MUST BE THE SAME LENGTH
    char line_buffer[254]; //line buffer used to read file line by line
        
    fseek(_fp,(line_size+1)*line_number,SEEK_SET);      //fseek must use the +1 to get the newline character
           
    //write over the internal _line_buffer               //start from the beginning and go to this position   
    fread(line_buffer, 1, line_size, _fp);              //read the line that is exactly 160 characters long
    
    //serialPrint("Debug (transmitPacketNumber): line_buffer <<%s>> (line size: %d)\n\r", line_buffer,line_size);
    
    //    createDataPacket requires _packet_number, _total_number_of_packets, _current_line_length (data packet size)
    //    uses char _line_buffer[256] variable to hold characters read from the file
    //    packs this into a vector for transmission                 
    
    //change the internal member variable for packet number, reorg this later
    _packet_number = line_number;   
    
    createDataPacket(line_buffer, line_size);  //create the data packet from the _line_buffer (char array)

    transmitDataPacket();   //transmit the assembled packet
}


// Using vector was faster in testing than passing the arrays
int MbedLogger::calcCrcOne() {
    //can't initialize the table in the constructor in c++
    int crc_table [256] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    int crc = 0;
    for (_it=_data_packet.begin(); _it < _data_packet.end(); _it++)
        crc = (crc_table[(*_it ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;

    return crc / 256; //second-to-last byte
}

// Using vector was faster in testing than passing the arrays
int MbedLogger::calcCrcTwo() {
    int crc_table [256] = {0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448};
    
    int crc = 0;
    for (_it=_data_packet.begin(); _it < _data_packet.end(); _it++)
        crc = (crc_table[(*_it ^ crc) & 0xff] ^ (crc >> 8)) & 0xFFFF;
    
    //serialPrint("DEBUG: calcCrcTwo string length: %d crc: %d\n\r", input_array.length(), crc % 256);

    return crc % 256; //last byte
}

//new 6/27/2018, create data packet
void MbedLogger::createDataPacket(char line_buffer_sent[], int line_length_sent) {     
    // packet is 7565 0001 FFFF EEEE CC DATA DATA DATA ... CRC1 CRC2
    
    //CLEAR: Removes all elements from the vector (which are destroyed), leaving the container with a size of 0.
    _data_packet.clear();
    
    //DATA PACKET HEADER
    _data_packet.push_back(117);                             //0x75
    _data_packet.push_back(101);                             //0x65    
    
    _data_packet.push_back(_packet_number/256);                //current packet number in 0x#### form
    _data_packet.push_back(_packet_number%256);                //current packet number in 0x#### form
    
    _data_packet.push_back(_total_number_of_packets/256);                //total number of packets, 0x#### form
    _data_packet.push_back(_total_number_of_packets%256);                //total number of packets, 0x#### form
    
    _data_packet.push_back(line_length_sent);

    //serialPrint("DEBUG: Current line buffer: %s\n\r", _line_buffer);        //debug

    //DATA FROM LINE READ (read string character by chracter)
    for (int i = 0; i < line_length_sent; i++) {
        _data_packet.push_back(line_buffer_sent[i]);
    }
    
    //CRC CALCULATIONS BELOW
    //character version of calculation messes up on null character 0x00, scrapped and using vector of integers

    int crc_one = calcCrcOne();
    int crc_two = calcCrcTwo();
    
    //place the crc bytes into the data packet that is transmitted
    _data_packet.push_back(crc_one);
    _data_packet.push_back(crc_two);
    
    //serialPrint("debug createDataPacket(char line_buffer_sent[], int line_length_sent)\n\r");
}

void MbedLogger::transmitMultiplePackets() {
    serialPrint("transmitMultiplePackets\n");
    
    static int input_packet;                 //changed from char in previous iteration 03/28/2018
    static int transmit_crc_one = 0;            //hold crc values until they're reset with calculations
    static int transmit_crc_two = 0;
        
    int current_byte = -1;
    
    static int bytes_received = 0;
    
    int req_packet_number = -1;
     
//GET TOTAL NUMBER OF PACKETS!
    getNumberOfPacketsInCurrentLog();
//GET TOTAL NUMBER OF PACKETS!
        
    //open the file
    string file_name_string = _file_system_string + "LOG000.csv";
    _fp = fopen(file_name_string.c_str(), "r");
    
    //DEFAULT STATE
    static int current_state = HEADER_117;
    
    bool active_loop = true;
    
    while (active_loop) {
        //INCOMING BYTE
        current_byte = xbee().getc();
        
        //provide the next byte / state
        
        switch (current_state) {
            case HEADER_117:
                //serialPrint("HEADING 117\n\r");
                if (current_byte == 0x75) { 
                    current_state = HEADER_101;
                }
                
                                
                else if (current_byte == 0x10) {
                    current_state = END_TX_1;
                }
                break;
            case HEADER_101:
                //serialPrint("HEADING 101\n\r");
                if (current_byte == 0x65) { 
                    current_state = PACKET_NO_1;
                }
                break;
            case PACKET_NO_1:
                //serialPrint("PACKET_NO_1\n\r");
                input_packet = current_byte * 256;
                
                current_state = PACKET_NO_2;
                //serialPrint("PACKET # 1 current byte %d\n\r", current_byte);
                break;
            
            case PACKET_NO_2:
                //serialPrint("PACKET_NO_2\n\r");
                input_packet = input_packet + current_byte;
                
                current_state = PACKET_CRC_ONE;
                //serialPrint("PACKET # 2 current byte %d (req packet num: %d)\n\r", current_byte, input_packet);
                break;
                
            case PACKET_CRC_ONE:
                //serialPrint("PACKET_CRC_ONE\n\r");
                current_state = PACKET_CRC_TWO;
                break;
            
            case PACKET_CRC_TWO:
                //serialPrint("PACKET_CRC_TWO\n\r");
                current_state = HEADER_117;
                transmitPacketNumber(input_packet);
                break;
                
            case END_TX_1:
                //serialPrint("END_TX_1\n\r");                
                current_state = END_TX_2;
                break;
            case END_TX_2:
                //serialPrint("END_TX_2\n\r");                
                current_state = HEADER_117;
                active_loop = false;
                break;
            
            default:
                //reset state
                //serialPrint("DEFAULT. HEADER_117\n\r");
                current_state = HEADER_117; //reset here
                break;
        }
    }
    
    //CLOSE THE FILE
    closeLogFile(); 
    serialPrint("08/05/2018 CLOSE THE LOG FILE\n\r");
    
    //RESET THE STATE
    //current_state = HEADER_117;
}

//only do this for the MBED because of the limited file size
//write one line to the file (open to write, this will erase all other data) and close it.
void MbedLogger::eraseFile() {    
    _fp = fopen(_full_file_path_string.c_str(), "w"); // LOG000.csv
    
    fprintf(_fp,_heading_string.c_str());

    closeLogFile();
}


void MbedLogger::setTransmitPacketNumber(int packet_number) {
    _transmit_packet_num = packet_number;
    
    //also needed to reset a boolean flag on the transmit
    _fsm_transmit_complete = false;
    
    _end_transmit_packet = false;
}

void MbedLogger::receiveSequenceFile() {
    //restart each time
    _end_sequence_transmission = false;
    
    serialPrint("Opening Mission file (sequence.txt) for reception.\n\r");
    string filename_string = _file_system_string + "sequence.txt";
    
    //serialPrint("DEBUG: openNewMissionFile: %s\n\r", filename_string.c_str());

    _fp = fopen(filename_string.c_str(), "w");

    //zero will be reserved for the file name, future 
    _confirmed_packet_number = 1;           //in sendReply() function that transmits a reply for incoming data
    
//    int current_packet_number = 1;
//    int last_packet_number = -1;
//    int break_transmission = 0;
    
    int counter = 0;
    
    while(1) {
        wait(0.25);  //runs at 4 Hz
        
        checkForIncomingData();
                
        //serialPrint("\n\rDEBUG: _confirmed_packet_number%d\n\r", _confirmed_packet_number);    
        
        counter++;
        
        sendReply();    //bad name, should call it send request or something
        
        if (_end_sequence_transmission)
            break;
    }
    
    closeLogFile();
}

int MbedLogger::sendReply() {
    //change this method to be more explicit later
    
    //integer vector _data_packet is used here, cleared fist just in case
    
    _data_packet.clear();   //same data packet for transmission
    _data_packet.push_back(117);
    _data_packet.push_back(101);
    
    //_confirmed_packet_number comes from the packet number that is sent from the Python program
    _data_packet.push_back(_confirmed_packet_number / 256);    //packet number only changed when confirmed
    _data_packet.push_back(_confirmed_packet_number % 256);     //split into first and second byte
    
    //compute checksums
    
    int receiver_crc_one = calcCrcOne();
    int receiver_crc_two = calcCrcTwo();
    
    _data_packet.push_back(receiver_crc_one);
    _data_packet.push_back(receiver_crc_two);
    
    //transmit this packet
    for (_it=_data_packet.begin(); _it < _data_packet.end(); _it++) {
        xbee().putc(*_it); //send integers over serial port one byte at a time
    }
    
    //change process methodology later...
    
    return _confirmed_packet_number;
}

int MbedLogger::getFileSize(string filename) {    
    // fixed the const char * errror:
    // https://stackoverflow.com/questions/347949/how-to-convert-a-stdstring-to-const-char-or-char
    const char * char_filename = filename.c_str();  // Returns a pointer to an array that contains a null-terminated sequence of characters (i.e., a C-string) representing the current value of the string object.
    //http://www.cplusplus.com/reference/string/string/c_str/
    
    _fp = fopen(filename.c_str(), "rb");     //open the file for reading as a binary file
    
    fseek(_fp, 0, SEEK_END);                     //SEEK_END is a constant in cstdio (end of the file)    
    unsigned int file_size = ftell(_fp);        //For binary streams, this is the number of bytes from the beginning of the file.
    fseek(_fp, 0, SEEK_SET);                    //SEEK_SET is hte beginning of the file, not sure this is necessary
        
    serialPrint("%s file size is %d bytes.\n\r", filename.c_str(), file_size);
    
    closeLogFile();                                //can probably just close the file pointer and not worry about position

    return file_size;
}

// rewrite this when time permits!

// function checks for incoming data (receiver function) from a Python program that transmits a file
// current limit is a file that has 255 lines of data
bool MbedLogger::checkForIncomingData() {        
    int receive_packet_number;
    int receive_total_number_packets;
    int receive_packet_size;
    
    bool data_transmission_complete = false;
    
    int incoming_byte;
    
    char char_buffer[256] = {};    //create empty buffer
   
    //starting state
    int process_state = HEADER_117;
    
    //variables for processing data below
    int checksum_one = -1;
    int checksum_two = -1;
    
    int i = 5;
    int serial_timeout = 0;
    
    while (xbee().readable() && !data_transmission_complete) {                
        incoming_byte = xbee().getc();    //getc returns an unsigned char cast to an int
        //serialPrint("DEBUG: State 0\n\r");
        
        switch(process_state) {
            case HEADER_117:
                //continue processing
                if (incoming_byte == 117){
                    process_state = HEADER_101;
                    //serialPrint("DEBUG: Case 117\n\r");
                }
                //end transmission
                else if (incoming_byte == 16) {
                    process_state = END_TRANSMISSION;
                    //serialPrint("DEBUG: State 16 (END_TRANSMISSION)\n\r");
                }
                else {
                    process_state = HEADER_117; // ???
                    //serialPrint("DEBUG: State Header 117\n\r");
                }
                break;
            
            case HEADER_101:
                if(incoming_byte == 101) {
                    process_state = PACKET_NUM;
                    //serialPrint("DEBUG: Case 101\n\r");
                }
                break;
                
            case PACKET_NUM:
                receive_packet_number = incoming_byte;
                process_state = TOTAL_NUM_PACKETS;
                //serialPrint("DEBUG: Case PACKET_NUM\n\r");
                break;
            
            case TOTAL_NUM_PACKETS:
                receive_total_number_packets = incoming_byte;
                process_state = PACKET_SIZE;
                //serialPrint("DEBUG: Case TOTAL_NUM_PACKETS\n\r");
                break;
                
            case PACKET_SIZE:      
                receive_packet_size = incoming_byte;
                //serialPrint("DEBUG: Case PACKET_SIZE\n\r");
                
                //write the header stuff to it
                char_buffer[0] = 117;
                char_buffer[1] = 101;
                char_buffer[2] = receive_packet_number;
                char_buffer[3] = receive_total_number_packets;
                char_buffer[4] = receive_packet_size;    
                
                // tests confirmed that packet number is zero, number of packets is 12, packet size is 12
                //serialPrint("char_buffer 2/3/4: %d %d %d\n\r", receive_packet_number,receive_total_number_packets,receive_packet_size);
                       
                //process packet data, future version will append for larger data sizes, 0xFFFF
                
                // IF YOU GET AN INTERRUPTED DATA STREAM YOU NEED TO BREAK OUT OF THE LOOP
                i = 5;
                serial_timeout = 0;
                
                while (true) {
                    if (xbee().readable()) {
                        char_buffer[i] = xbee().getc();   //read all of the data packets
                        i++;
                        
                        serial_timeout = 0; //reset the timeout
                    }
                    else {
                        serial_timeout++;        
                    }
                    
                    // When full data packet is received...
                    if (i >= receive_packet_size+5) {     //cannot do this properly with a for loop                        
                        //get checksum bytes
                        checksum_one = xbee().getc();
                        checksum_two = xbee().getc();
                        
                        //calculate the CRC from the header and data bytes using _data_packet (vector)
                        //found out calculating crc with string was dropping empty or null spaces
                        _data_packet.clear();   //clear it just in case
                        
                        for (int a = 0; a < receive_packet_size+5; a++) {
                            _data_packet.push_back(char_buffer[a]);  //push the character array into the buffer
                        }
                        
                        //calculate the CRC using the vector (strings will cut off null characters)
                        int calc_crc_one = calcCrcOne();
                        int calc_crc_two = calcCrcTwo();

                        //serialPrint("DEBUG: calc crc 1: %d, crc 2: %d\n\r", calc_crc_one, calc_crc_two);
                        
                        // first confirm that the checksum is correct
                        if ((calc_crc_one == checksum_one) and (calc_crc_two == checksum_two)) {    
                        
                            //serialPrint("DEBUG: checksums are good!\n\r");
                            
                            //serialPrint("receive_packet_number %d and _confirmed_packet_number %d\n\r", receive_packet_number, _confirmed_packet_number); //debug
                                                                            
                            // check if the packet that you're receiving (receive_packet_number) has been received already...
        
                            //CHECKSUM CORRECT & packet numbers that are 1 through N packets
                            if (receive_packet_number == _confirmed_packet_number){
                                //save the data (char buffer) to the file if both checksums work...                 
                                
                                // when a packet is received (successfully) send a reply
                                //sendReply();
                                
                                // write correct data to file
                                fprintf(_fp, "%s", char_buffer+5);
                                
                                // even if correct CRC, counter prevents the program from writing the same packet twice
                                _confirmed_packet_number++;
                            }
                            
                            //clear the variables
                            checksum_one = -1;
                            checksum_two = -1;
                        }
                        
                        process_state = HEADER_117;
                        
                        break;
                    }
                    
//                    //counter breaks out of the loop if no data received
//                    if (serial_timeout >= 10000) {
//                        //serialPrint("break serial_timeout %d\n\r", serial_timeout);
//                        break;      
//                    }
                }
                break;   
                
            case END_TRANSMISSION:
                if (xbee().getc() == 16) {
                    //serialPrint("DEBUG: END_TRANSMISSION REACHED: 1. \n\r");
                    
                    if (xbee().getc() == 16) {
                        //serialPrint("DEBUG: END_TRANSMISSION REACHED: 2. \n\r");
                        
                        _end_sequence_transmission = true;
                        //endReceiveData();
                    }
                }
                
                //serialPrint("DEBUG: END_TRANSMISSION REACHED: 5. \n\r");
                
                //process_state = HEADER_117; //don't do this unless the check is wrong
                //serialPrint("END_TRANSMISSION process_state is %d\n\r", process_state);        //should be 5 (debug) 02/06/2018
                data_transmission_complete = true;
                break;            
        }//END OF SWITCH
        
        if (data_transmission_complete) {
            //serialPrint("DEBUG: checkForIncomingData data_transmission_complete \n\r");
            break;  //out of while loop
        }   
    } // while loop
    
    led3() = !led3();
    
    if (data_transmission_complete)
        return false;   //tell state machine class that this is done (not transmitting, false)
    else {
        return true;
    }
}