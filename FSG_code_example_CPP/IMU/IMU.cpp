/*******************************************************************************
Author:           Troy Holley
Title:            IMU.cpp
Date:             08/01/2018

Description/Notes:

IMU code fixed from Newton's earlier code.

This code reads the AHRS/IMU and outputs the roll, pitch, and heading data.

*******************************************************************************/

#include "IMU.h"

IMU::IMU(PinName Tx, PinName Rx): _rs232(Tx,Rx) {
}

void IMU::initialize() {
    //set up the serial bus and frequency
    _rs232.baud(115200);

    // initialize the processing state machine
    state = SYNC0; 
    
    // initialize to zeros
    euler[0] = 0.0;
    euler[1] = 0.0;
    euler[2] = 0.0;
    
    // initialize to zeros
    lat_lon_height[0] = 0.0;
    lat_lon_height[1] = 0.0;
    lat_lon_height[2] = 0.0;
}

void IMU::runIMU() {
    update();
}

// updated the imu update function with a state machine that doesn't hang if no data is present
void IMU::update() {    
    while (_rs232.readable()) {
        // read a single byte
        byte = _rs232.getc();   

        
        // state machine to process byte-by-byte
        switch (state) {
        case SYNC0 : 
            if (byte == 0x75) {
                packet[0] = byte; // save into the packet
                state = SYNC1;
            }
            break;
            
        case SYNC1 :
            if (byte == 0x65) {
                packet[1] = byte; // save into the packet
                state = SET;
            }
            else {
                state = SYNC0;
            }
            break;
            
        case SET :
            descriptor = byte; // save descriptor set
            packet[2] = byte; // save into the packet
            state = LEN;                
            break;
            
        case LEN :
            len = byte; // save payload length
            packet[3] = byte; // save into the packet
            state = PAY;
            i = 0; // reset payload field length counter            
            
            break;
            
        case PAY :
            if (i < len) { // keep adding until get all the payload length
                packet[4+i] = byte; // add byte to the packet, skipping over the header 4-bytes
                i++; // increment payload counter
            }
            else {
                state = CRC0;
            }
            if (i >= len) { // not an elseif, since we want to escape when i==len
                state = CRC0;
            }
            break;
            
        case CRC0 :
            crc0 = byte; // save the msb of the checksum
            state = CRC1;
            break;
            
        case CRC1 :
            crc1 = byte; // save the lsb of the checksum
            checksum = ((unsigned int)crc0 << 8) + (unsigned int)crc1; // make checksum into a uint16
            
            // Troy: Payload Length byte check (header)
            if (checksum == calcChecksum(packet, len+4)) { // Newton: passed checksum, wahoo!

                // position in the packet, made this more explicit
                int header_descriptor_set_byte = 2;     //this HEADER DESCRIPTOR SET byte is constant among a single packet
                int field_length_position = 4;          //this 1st field length descriptor starts at the fifth element in the packet (page 11)
                int curr_packet_pos = 4;                //the 1st element of the packet starts at the fifth element of the full packet
                int header_payload_length_byte = (int)packet[3];
                int total_field_length = 0;         //this is used to compare and make sure you only process the amount of data given by the payload length byte in the header 
                int full_packet_length = 4 + header_payload_length_byte + 2;   //header(4) + packet payload(#) + checksum(2) (just for clarification)
                
                while (1) {
                    
                    //add the total field length at the beginning (this is keeping track of how many bytes you've processed to compare to the header's paylod length byte
                    total_field_length = total_field_length + packet[field_length_position]; //current packet field length
                    
                    //process payload by passing one payload packet (length, header descriptor, current packet) to the function (descriptor = Descriptor Set byte)
                    processPayload(packet[field_length_position], packet[header_descriptor_set_byte], &packet[curr_packet_pos]); // process the payload part of the packet, starting at byte 4
                                        
                    //add the length
                    //keep moving the pointer after processing
                    //shift these two positions based on the field length byte value; e.g. packet of size 14 moves it 14 to the right
                    field_length_position = field_length_position + packet[field_length_position];
                    curr_packet_pos = curr_packet_pos + packet[field_length_position];
                    
                    if (total_field_length >= header_payload_length_byte) //break out next time
                        break;
                }
            }
            state = SYNC0; // reset to SYNC0 state
            break;

        default :
            state = SYNC0;
        }
    }
    return;
}

// page 78 of 3DM-GX3-35 Data Protocol -- Euler Angles (0x80, 0x0C)
void IMU::processPayload(char field_length, char header_descriptor, unsigned char * payload) {
    
    //make sure payload is at least two bytes to see the descriptor
    if (field_length >= 2) {
    
        //process payload based on the FIELD DESCRIPTOR byte (not header descriptor byte)
        if (header_descriptor == IMU_DATA_SET) {
        
            if (payload[1] == EULER_CF_DESCRIPTOR)            // find an euler CF field descriptor   (0x0C)
                processEulerCfPacket(field_length, payload);
        }            
        else if (header_descriptor == GPS_DATA_SET) {
            if (payload[1] == LLH_POSITION_DESCRIPTOR)     // find a lat-lon-alt field descriptor incorrect?  GPS Data > LLH Position (0x81, 0x03) page 83
                processLatLonHeightPacket(field_length, payload);
        }
    }   //end of length checking if-then statement
}

//Function below will convert a 14-byte packet (length,descriptor,data) into Euler Angles (float data type)
void IMU::processEulerCfPacket(char field_length, unsigned char * payload) {
    if (field_length >= EULER_CF_LENGTH) { // make sure correct field length
        if (payload[0] == EULER_CF_LENGTH) { // make sure field length is as expected
            euler[0] = floatFromChar(&payload[ROLL_OFFSET+2])*180/_PI;  // roll Euler angle convert in degrees
            euler[1] = floatFromChar(&payload[PITCH_OFFSET+2])*180/_PI; // pitch Euler angle convert in degrees
            euler[2] = floatFromChar(&payload[YAW_OFFSET+2])*180/_PI;   // yaw Euler angle convert in degrees
        }
    }
}

//Function below will convert a 14-byte packet (length,descriptor,data) into GPS coordinates (fload data type)
//verified this is the same between the 3DM-GX3-35 and the 3DM-GX3-45
void IMU::processLatLonHeightPacket(char field_length, unsigned char * payload) {
    if (field_length >= LLH_POSITION_LENGTH) { // make sure correct field length
        if (payload[0] == LLH_POSITION_LENGTH) { // make sure field length is as expected
            lat_lon_height[0] = floatFromChar(&payload[LATITUDE_OFFSET+2]);   // latitude in decimal degrees
            lat_lon_height[1] = floatFromChar(&payload[LONGITUDE_OFFSET+2]);  // longitude in decimal degrees
            lat_lon_height[2] = floatFromChar(&payload[HEIGHT_MSL_OFFSET+2]); // altitude above mean sea level in meters
        }
    }
}

float IMU::floatFromChar(unsigned char * value) {
    unsigned char temp[4];
    temp[0] = value[3];
    temp[1] = value[2];
    temp[2] = value[1];
    temp[3] = value[0];
    return *(float *) temp;
}

double IMU::doubleFromChar(unsigned char * value) {
    unsigned char temp[8];
    temp[0] = value[7];
    temp[1] = value[6];
    temp[2] = value[5];
    temp[3] = value[4];
    temp[4] = value[3];
    temp[5] = value[2];
    temp[6] = value[1];
    temp[7] = value[0];
    return *(double *) temp;
}

float IMU::getRoll() {
    return euler[0];
}

float IMU::getPitch() {
    return euler[1];
}

float IMU::getHeading() {
    return euler[2];    //
}

float IMU::getLatitude() {
    return lat_lon_height[0];
}

float IMU::getLongitude() {
    return lat_lon_height[1];
}

float IMU::getAltitudeMSL() {
    return lat_lon_height[2];
}

unsigned int IMU::calcChecksum(unsigned char * mip_packet, char checksum_range) {
    unsigned char checksum_byte1 = 0;
    unsigned char checksum_byte2 = 0;
    unsigned int myChecksum = 0;
    
    for (int i=0; i<checksum_range; i++) {
        checksum_byte1 += mip_packet[i];
        checksum_byte2 += checksum_byte1;
    }
    myChecksum = ((unsigned int)checksum_byte1 << 8) + (unsigned int)checksum_byte2;
    return myChecksum;
}

int IMU::packetLength() {
    return len;
}