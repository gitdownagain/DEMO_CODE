#ifndef IMU_H_
#define IMU_H_

#include "mbed.h"
#include "MODSERIAL.h"

// for Microstrain's MIPS protocol, try this link, or search on microstrain.com
// http://www.microstrain.com/sites/default/files/3dm-gx5-45_dcp_manual_8500-0064_0.pdf

#define _PI ((float) 3.14159265359)

// state machine states
#define SYNC0 0 // first sync byte state
#define SYNC1 1 // second sync byte state
#define SET   2 // aka descriptor set
#define LEN   3 // payload length
#define PAY   4 // getting the payload data
#define CRC0  5 // crc high byte
#define CRC1  6 // crc low byte

// data set descriptors
#define IMU_DATA_SET                0x80    //decimal 128
#define GPS_DATA_SET                0x81    //decimal 129   (page 83: GPS Data > LLH Position)

// enumerations for the Euler angle packet we care about
#define EULER_CF_LENGTH             14
#define EULER_CF_DESCRIPTOR         0x0C    // page 78 of 3DM-GX3-35 Data Protocol -- Euler Angles (0x80, 0x0C)
#define ROLL_OFFSET                 0
#define PITCH_OFFSET                4
#define YAW_OFFSET                  8

// enumerations for the lat-lon-alt packet we care about
#define LLH_POSITION_LENGTH         44
#define LLH_POSITION_DESCRIPTOR     0x03    // page 83 of 3DM-GX3-35 Data Protocol -- Euler Angles (0x80, 0x0C)
#define LATITUDE_OFFSET             0
#define LONGITUDE_OFFSET            8
#define HEIGHT_MSL_OFFSET           16
#define VALID_FLAG_OFFSET           40

class IMU {
public:

    IMU(PinName Tx, PinName Rx);
    void initialize();
    void update();
    void start();
    void stop();
    
    void runIMU();
    
    float getRoll();
    float getPitch();
    float getHeading();
   
    float getLatitude();
    float getLongitude();
    float getAltitudeMSL();
    
    char byte;  //in order to debug quickly
    
    int packetLength(); //DEBUG
    
    
protected:
    Ticker interval;
    MODSERIAL _rs232;     //don't put this here?
    
    //char byte;
    
    unsigned char state, len, descriptor, i, packet[256];
    unsigned int checksum, crc0, crc1;
    
/*    unsigned char state, len, descriptor, i, crc0, crc1, payload[30];
    unsigned int checksum;*/
    
    float euler[3];
    float lat_lon_height[3];

    void processPayload(char type, char length, unsigned char * payload);
    void processEulerCfPacket(char length, unsigned char * payload);
    void processLatLonHeightPacket(char length, unsigned char * payload);

    unsigned int calcChecksum(unsigned char * mip_packet, char checksum_range);

    float floatFromChar(unsigned char * value);
    double doubleFromChar(unsigned char * value);
};

#endif

// forgot the ifndef, define, endif