#ifndef GUI_HPP
#define GUI_HPP
 
#include "mbed.h"
#include <string>
using namespace std;

#include <algorithm> //for reverse function

#include <vector> //delete?

enum {
    HEADER_FE,
    HEADER_ED,
    COMMAND_PACKET,
    FSM_PACKET_SIZE,
    FSM_CRC_ONE,
    FSM_CRC_TWO
};

class Gui {
public:
    Gui();           //constructor

    int guiCalcCrc1(int *input_array, int array_length);
    int guiCalcCrc2(int *input_array, int array_length);
    
    void getCommandFSM();
    
    void updateGUI();
    
    int calcCrcOneVector(vector <int> crc_packet);
    int calcCrcTwoVector(vector <int> crc_packet);
    void transmitDataPacket(vector <int> crc_packet);
 
private:
    int _crc_table[];
    
    vector <int> _gui_update_packet;
    
    std::vector<int>::iterator _it;
};
 
#endif /* GUI_HPP */