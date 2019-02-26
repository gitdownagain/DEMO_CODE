#include "mbed.h"
#include "ConfigFile.h"

#ifndef CONFIGFILEIO_HPP
#define CONFIGFILEIO_HPP

class ConfigFileIO {
public:
    ConfigFileIO();
    
    //modified this to save frequency and deadband 7/11/2018
    
    void saveBattData(float batt_p_gain, float batt_i_gain, float batt_d_gain, int batt_zeroOffset, float batt_filter_freq, float batt_deadband); //modified this because zero offsets are integers
    void savePitchData(float pitch_p_gain, float pitch_i_gain, float pitch_d_gain, float pitch_zeroOffset, float pitch_filter_freq, float pitch_deadband);
    
    void saveBCEData(float bce_p_gain, float bce_i_gain, float bce_d_gain, int bce_zeroOffset, float bce_filter_freq, float bce_deadband);  //modified this because zero offsets are integers
    void saveDepthData(float depth_p_gain, float depth_i_gain, float depth_d_gain, float depth_zeroOffset, float depth_filter_freq, float depth_deadband);
    
    void saveRudderData(float setMinDeg, float setMaxDeg, float setCenterPWM, float setMinPWM, float setMaxPWM);
    void saveHeadingData(float heading_p_gain, float heading_i_gain, float heading_d_gain, float heading_zeroOffset, float heading_filter_freq, float heading_deadband);
    
    //ConfigFunctions
    int load_BCE_config();
    int load_BATT_config();
    int load_DEPTH_config();
    int load_PITCH_config();
    int load_HEADING_config();      //heading outer loop of rudder servo
    int load_RUDDER_config();       //rudder servo
    int load_script();

private: 
    float _neutral_batt_pos_mm;
    float _neutral_bce_pos_mm;
};
#endif
