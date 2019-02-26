#ifndef MBEDLOGGER_HPP
#define MBEDLOGGER_HPP
 
#include "mbed.h"
#include <string>
using namespace std;

#include <vector>
#include <fstream>

//used in switch-case statements for checking if I received the correct packets
enum {
    HEADER_117,
    HEADER_101,
    PACKET_NUM,
    TOTAL_NUM_PACKETS,
    PACKET_SIZE,
    END_TRANSMISSION,
    TRANSMIT_PACKET_1,
    TRANSMIT_PACKET_2,
    PACKET_CRC_ONE,
    PACKET_CRC_TWO,
    RANDOM_CHAR,
    COMMAND_ONE,
    COMMAND_TWO,
    HEADER_1E,
    HEADER_1F,
    PACKET_NO_1,
    PACKET_NO_2,
    END_TX_1,
    END_TX_2
};

class MbedLogger {
public:
    MbedLogger(string file_system_input_string);            //to choose between MBED and SD card
    
    void setLogTime();
    void initializeLogFile();
    void closeLogFile();    //this sets pointer to null and checks if it is closed otherwise
    void appendLogFile(int current_state, int option);     //check if you have orphaned file pointers before this (file should not be open already)
    int getSystemTime();          //parse the time to record to the log file
    void recordData(int current_state); //Save current state of data
    void printMbedDirectory();          //print the MBED directory (future feature)
    void printCurrentLogFile();         //print the current MBED log file
    //void checkForPythonTransmitRequest();
    bool checkForIncomingData();
    int getNumberOfPacketsInCurrentLog();
    void transmitDataPacket();  // Transmit the data packet
    void transmitPacketNumber(int line_or_packet_number);
    void eraseFile();       //erase MBED log file    
    int calcCrcOne();  //used with vector _data_packet, cleaning up later
    int calcCrcTwo();
    void transmitMultiplePackets();  
    void createDataPacket(char line_buffer_sent[], int line_length_sent);
    void setTransmitPacketNumber(int packet_number);
    bool endTransmitPacket();
    void receiveSequenceFile();
    int sendReply();
    int getFileSize(string filename);   //return the file size of the MBED log file
    
private:
    FILE *_fp;              //the file pointer
    
    string _file_system_string;
    string _full_file_path_string;
    
    //check what I need to remove from this
    bool _file_transmission;
    int _confirmed_packet_number;   //must set this to zero
    int _transmit_counter;
    int _file_transmission_state;
    int _total_number_of_packets;
    bool _mbed_transmit_loop;
    string _received_filename;
    int _log_file_line_counter;     //used to set timer in finite state machine based on size of log
    string _heading_string;
    int _transmit_packet_num;
    bool _fsm_transmit_complete;
    bool _end_transmit_packet;
    bool _end_sequence_transmission;
    int _packet_number;             //keep track of packet number for transmitting data
    float _data_log[37];            //for logging all of the data from the outer and inner loops and so on
    vector <int> _data_packet;      //holds the current packet I'm processing
    std::vector<int>::iterator _it; //used to iterate through current data packet
    //check what I need to remove from this !!!!!!!!!!!!!!!!!!
};
 
#endif