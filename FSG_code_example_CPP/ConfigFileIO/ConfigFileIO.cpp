/*******************************************************************************
Author:           Troy Holley
Title:            ConfigFileIO.cpp
Date:             12/19/2018

Description/Notes:

Configuration file input and output.

This allows a user to load different configuration files, set parameters, and
rewrite those configuration files through user menus when parameters are changed.
*******************************************************************************/

#include "ConfigFileIO.hpp"
#include "StaticDefs.hpp"

ConfigFileIO::ConfigFileIO() {
}    

void ConfigFileIO::saveBattData(float batt_p_gain, float batt_i_gain, float batt_d_gain, int batt_zeroOffset, float batt_filter_freq, float batt_deadband) {       
    ConfigFile write_Batt_txt;   //write to the config file
    
    char PGain_buffer[128];
    sprintf(PGain_buffer,"# configuration file for battery mover parameters\n\n#Gains\nPGain");
    
    char string_pgain[128];  
    sprintf(string_pgain, "%f", batt_p_gain);
    write_Batt_txt.setValue(PGain_buffer, string_pgain);
    
    char string_igain[128];  
    sprintf(string_igain, "%f", batt_i_gain);
    write_Batt_txt.setValue("IGain", string_igain);
    
    char string_dgain[128];  
    sprintf(string_dgain, "%f", batt_d_gain);
    write_Batt_txt.setValue("DGain", string_dgain);
      
    char string_zeroCounts[128];  
    sprintf(string_zeroCounts, "%d", batt_zeroOffset);      
    write_Batt_txt.setValue("\n#string pot parameters\nzeroCounts", string_zeroCounts);
    
    //modified this from hard-coded values to set to whatever is in the file on startup
    char string_batt_travel_limit[128];
    sprintf(string_batt_travel_limit, "%f", batt().getTravelLimit());
    write_Batt_txt.setValue("PistonTravelLimit", string_batt_travel_limit);
    
    write_Batt_txt.setValue("slope", "0.12176");
    
    char string_filter_freq[128];  
    sprintf(string_filter_freq, "%f", batt_filter_freq);
    write_Batt_txt.setValue("filterWn", string_filter_freq);
    
    char string_deadband[128];  
    sprintf(string_deadband, "%f", batt_deadband);
    write_Batt_txt.setValue("deadband", string_deadband);

    //SAVE THE DATA!
    xbee().printf("Saving BATTERY MOVER PID data!");
    
    if (!write_Batt_txt.write("/local/batt.txt")) {
        xbee().printf("\n\rERROR: (SAVE)Failure to write batt.txt file.");
    }
    else {
        xbee().printf("\n\rFile batt.txt successful written.\n\r");
    }  
}

void ConfigFileIO::savePitchData(float pitch_p_gain, float pitch_i_gain, float pitch_d_gain, float pitch_zeroOffset, float pitch_filter_freq, float pitch_deadband) {   
    ConfigFile write_pitch_txt;   //write to the config file
    
    char PGain_buffer[128];
    sprintf(PGain_buffer,"# pitch outer loop parameters\n\n#Gains\nPGain");
    
    char string_pgain[128];  
    sprintf(string_pgain, "%f", pitch_p_gain);
    write_pitch_txt.setValue(PGain_buffer, string_pgain);
    
    char string_igain[128];  
    sprintf(string_igain, "%f", pitch_i_gain);
    write_pitch_txt.setValue("IGain", string_igain);
    
    char string_dgain[128];  
    sprintf(string_dgain, "%f", pitch_d_gain);
    write_pitch_txt.setValue("DGain", string_dgain);
    
    char string_filter_freq[128];  
    sprintf(string_filter_freq, "%f", pitch_filter_freq);       
    write_pitch_txt.setValue("\n#Pitch sensor filter parameters\nfilterWn", string_filter_freq);
    
    char string_deadband[128];
    sprintf(string_deadband, "%f", pitch_deadband);
    write_pitch_txt.setValue("deadband", string_deadband);
    
    char string_zeroOffset[128];
    sprintf(string_zeroOffset, "%f", pitch_zeroOffset);
    //bce setting was 41 mm during LASR experiments
    write_pitch_txt.setValue("\n#Offset for neutral (default: 41)\nzeroOffset", string_zeroOffset);
    
    //SAVE THE DATA!
    xbee().printf("Saving Buoyancy Engine Neutral Buoyancy Positions!");
    
    if (!write_pitch_txt.write("/local/pitch.txt")) {
        xbee().printf("\n\rERROR: (SAVE)Failure to write depth.txt file.");
    }
    else {
        xbee().printf("\n\rFile pitch.txt successful written.\n\r");
    } 
}

void ConfigFileIO::saveBCEData(float bce_p_gain, float bce_i_gain, float bce_d_gain, int bce_zeroOffset, float bce_filter_freq, float bce_deadband) {       
    ConfigFile write_BCE_txt;   //write to the config file
    
    char PGain_buffer[128];
    sprintf(PGain_buffer,"# configuration file for BCE parameters\n\n#Gains\nPGain");
    
    char string_pgain[128];  
    sprintf(string_pgain, "%f", bce_p_gain);
    write_BCE_txt.setValue(PGain_buffer, string_pgain);
    
    char string_igain[128];  
    sprintf(string_igain, "%f", bce_i_gain);
    write_BCE_txt.setValue("IGain", string_igain);
    
    char string_dgain[128];  
    sprintf(string_dgain, "%f", bce_d_gain);
    write_BCE_txt.setValue("DGain", string_dgain);
    
    char string_zeroCounts[128];
    sprintf(string_zeroCounts, "%d", bce_zeroOffset);
    write_BCE_txt.setValue("\n#string pot parameters\nzeroCounts", string_zeroCounts);
    
    //modified this from hard-coded values to set to whatever is in the file on startup
    char string_bce_travel_limit[128];
    sprintf(string_bce_travel_limit, "%f", bce().getTravelLimit());
    write_BCE_txt.setValue("PistonTravelLimit", string_bce_travel_limit);
    
    write_BCE_txt.setValue("slope", "0.12176");
    
    char string_filter_freq[128];  
    sprintf(string_filter_freq, "%f", bce_filter_freq);
    write_BCE_txt.setValue("filterWn", string_filter_freq);
    
    char string_deadband[128];  
    sprintf(string_deadband, "%f", bce_deadband);
    write_BCE_txt.setValue("deadband", string_deadband);

    //SAVE THE DATA!
    xbee().printf("Saving BCE PID data!");
    
    if (!write_BCE_txt.write("/local/bce.txt")) {
        xbee().printf("\n\rERROR: (SAVE)Failure to write bce.txt file.");
    }
    else {
        xbee().printf("\n\rFile bce.txt successful written.\n\r");
    } 
}

void ConfigFileIO::saveDepthData(float depth_p_gain, float depth_i_gain, float depth_d_gain, float depth_zeroOffset, float depth_filter_freq, float depth_deadband) {       
    ConfigFile write_depth_txt;   //write to the config file
    
    char PGain_buffer[128];
    sprintf(PGain_buffer,"# depth outer loop parameters\n\n#Gains\nPGain");
    
    char string_pgain[128];  
    sprintf(string_pgain, "%f", depth_p_gain);
    write_depth_txt.setValue(PGain_buffer, string_pgain);
    
    char string_igain[128];  
    sprintf(string_igain, "%f", depth_i_gain);
    write_depth_txt.setValue("IGain", string_igain);
    
    char string_dgain[128];  
    sprintf(string_dgain, "%f", depth_d_gain);
    write_depth_txt.setValue("DGain", string_dgain);
    
    char string_filter_freq[128];
    sprintf(string_filter_freq, "%f", depth_filter_freq);
    write_depth_txt.setValue("\n#Depth sensor filter parameters\nfilterWn", string_filter_freq);
    
    char string_deadband[128];
    sprintf(string_deadband, "%f", depth_deadband);
    write_depth_txt.setValue("deadband", string_deadband);
    
    char string_zeroOffset[128];
    sprintf(string_zeroOffset, "%f", depth_zeroOffset);
    //bce setting was 240 mm during LASR experiments
    write_depth_txt.setValue("\n#Offset for neutral (default: 240)\nzeroOffset", string_zeroOffset);
    
    //SAVE THE DATA!
    xbee().printf("Saving Buoyancy Engine Neutral Buoyancy Positions!");
    
    if (!write_depth_txt.write("/local/depth.txt")) {
        xbee().printf("\n\rERROR: (SAVE)Failure to write depth.txt file.");
    }
    else {
        xbee().printf("\n\rFile depth.txt successful written.\n\r");
    } 
}

//write rudder (servo driver) file rudder.txt
void ConfigFileIO::saveRudderData(float setMinDeg, float setMaxDeg, float setCenterPWM, float setMinPWM, float setMaxPWM) {
    ConfigFile rudder_txt;
    
    char header[128];
    sprintf(header,"# rudder (servo) inner loop (heading outer loop uses this)");
    
    char string_min_deg[128];  
    sprintf(string_min_deg, "%f", setMinDeg);
    rudder_txt.setValue("setMinDeg", string_min_deg);
    
    char string_max_deg[128];  
    sprintf(string_max_deg, "%f", setMaxDeg);
    rudder_txt.setValue("setMaxDeg", string_max_deg);
    
    char string_ctr_pwm[128];  
    sprintf(string_ctr_pwm, "%f", setCenterPWM);
    rudder_txt.setValue("setCenterPWM", string_ctr_pwm);
    
    char string_min_pwm[128];  
    sprintf(string_min_pwm, "%f", setMinPWM);
    rudder_txt.setValue("setMinPWM", string_min_pwm);
    
    char string_max_pwm[128];  
    sprintf(string_max_pwm, "%f", setMaxPWM);
    rudder_txt.setValue("setMaxPWM", string_max_pwm);

    //SAVE THE DATA!
    xbee().printf("Saving RUDDER DATA!");
    
    if (!rudder_txt.write("/local/rudder.txt")) {
        xbee().printf("\n\rERROR: (SAVE)Failure to write rudder.txt file.");
    }
    else {
        xbee().printf("\n\rFile rudder.txt successful written.\n\r");
    } 
}

int ConfigFileIO::load_BCE_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/bce.txt")) {
            error("File Read Error");
    }
    char value[BUFSIZ];
 
    if (cfg.getValue("PGain", &value[0] , sizeof(value))) {
        bce().setControllerP(atof(value));
        count++;
    }
    if (cfg.getValue("IGain", &value[0] ,sizeof(value))) {
        bce().setControllerI(atof(value));
        count++;
    }
    if (cfg.getValue("DGain", &value[0] , sizeof(value))) {
        bce().setControllerD(atof(value));
        count++;
    }
    if (cfg.getValue("zeroCounts", &value[0],sizeof(value))) {
        bce().setZeroCounts(atoi(value));
        count++;
    }
    if (cfg.getValue("PistonTravelLimit", &value[0], sizeof(value))) {
        bce().setTravelLimit(atof(value));
        count++;
    }
    if (cfg.getValue("slope", &value[0], sizeof(value))) {
        bce().setPotSlope(atof(value));
        count++;
    }
    if (cfg.getValue("filterWn", &value[0], sizeof(value))) {
        bce().setFilterFrequency(atof(value));
        count++;
    }
    if (cfg.getValue("deadband", &value[0], sizeof(value))) {
        bce().setDeadband(atof(value));
        count++;
    }
    
    return count;     
}

//write to heading.txt
void ConfigFileIO::saveHeadingData(float heading_p_gain, float heading_i_gain, float heading_d_gain, float heading_zeroOffset, float heading_filter_freq, float heading_deadband) {
    ConfigFile heading_txt;
    
    char PGain_buffer[128];
    sprintf(PGain_buffer,"# HEADING (rudder outer loop) parameters\n\n#Gains\nPGain");
    
    //convert input numbers into text and save them to text file
    char string_pgain[128];  
    sprintf(string_pgain, "%f", heading_p_gain);
    heading_txt.setValue(PGain_buffer, string_pgain);
    
    char string_igain[128];  
    sprintf(string_igain, "%f", heading_i_gain);
    heading_txt.setValue("IGain", string_igain);
    
    char string_dgain[128];  
    sprintf(string_dgain, "%f", heading_d_gain);
    heading_txt.setValue("DGain", string_dgain);

    char string_heading_freq[128];
    sprintf(string_heading_freq, "%f", heading_filter_freq);
    heading_txt.setValue("\n# HEADING sensor filter parameters\nfilterWn", string_heading_freq);
    
    char string_heading_db[128];
    sprintf(string_heading_db, "%f", heading_deadband);
    heading_txt.setValue("deadband",string_heading_db);
    
    char string_zeroOffset[128];
    sprintf(string_zeroOffset, "%f", heading_zeroOffset);
    heading_txt.setValue("\n#HEADING offset\nzeroOffset", string_zeroOffset);
    
    //SAVE THE DATA!
    xbee().printf("(ConfigFileIO) Saving HEADING parameters!");
    
    if (!heading_txt.write("/local/heading.txt")) {
        xbee().printf("\n\rERROR: (SAVE) Failure to write heading.txt file.");
    }
    else {
        xbee().printf("\n\rFile heading.txt successful written.\n\r");
    } 
}

int ConfigFileIO::load_BATT_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/batt.txt")) {
            error("BATT File Read Error");
    }
    char value[BUFSIZ];
 
    
    if (cfg.getValue("PGain", &value[0] , sizeof(value))) {
        batt().setControllerP(atof(value));
        count++;
    }
    if (cfg.getValue("IGain", &value[0] ,sizeof(value))) {
        batt().setControllerI(atof(value));
        count++;
    }
    if (cfg.getValue("DGain", &value[0] , sizeof(value))) {
        batt().setControllerD(atof(value));
        count++;
    }
    if (cfg.getValue("zeroCounts", &value[0],sizeof(value))) {
        batt().setZeroCounts(atoi(value));
        count++;
    }
    if (cfg.getValue("PistonTravelLimit", &value[0], sizeof(value))) {
        batt().setTravelLimit(atof(value));
        count++;
    }
    if (cfg.getValue("slope", &value[0], sizeof(value))) {
        batt().setPotSlope(atof(value));
        count++;
    }
    if (cfg.getValue("filterWn", &value[0], sizeof(value))) {
        batt().setFilterFrequency(atof(value));
        count++;
    }
    if (cfg.getValue("deadband", &value[0], sizeof(value))) {
        batt().setDeadband(atof(value));
        count++;
    }
    
    return count;     
}

int ConfigFileIO::load_DEPTH_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/depth.txt")) {
            error("DEPTH File Read Error");
    }
    char value[BUFSIZ];
    
    if (cfg.getValue("PGain", &value[0] , sizeof(value))) {
        depthLoop().setControllerP(atof(value));
        count++;
    }
    if (cfg.getValue("IGain", &value[0] ,sizeof(value))) {
        depthLoop().setControllerI(atof(value));
        count++;
    }
    if (cfg.getValue("DGain", &value[0] , sizeof(value))) {
        depthLoop().setControllerD(atof(value));
        count++;
    }
    if (cfg.getValue("filterWn", &value[0], sizeof(value))) {
        depthLoop().setFilterFrequency(atof(value));
        count++;
    }
    if (cfg.getValue("deadband", &value[0], sizeof(value))) {
        depthLoop().setDeadband(atof(value));
        count++;
    }
    if (cfg.getValue("zeroOffset", &value[0], sizeof(value))) {
        depthLoop().setOutputOffset(atof(value));
        count++;
    }    
    return count;
}

int ConfigFileIO::load_PITCH_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/pitch.txt")){
            error("PITCH File Read Error");
    }
    char value[BUFSIZ];
    
    if (cfg.getValue("PGain", &value[0] , sizeof(value))) {
        pitchLoop().setControllerP(atof(value));
        count++;
    }
    if (cfg.getValue("IGain", &value[0] ,sizeof(value))) {
        pitchLoop().setControllerI(atof(value));
        count++;
    }
    if (cfg.getValue("DGain", &value[0] , sizeof(value))) {
        pitchLoop().setControllerD(atof(value));
        count++;
    }
    if (cfg.getValue("filterWn", &value[0], sizeof(value))) {
        pitchLoop().setFilterFrequency(atof(value));
        count++;
    }
    if (cfg.getValue("deadband", &value[0], sizeof(value))) {
        pitchLoop().setDeadband(atof(value));
        count++;
    }
    
    if (cfg.getValue("zeroOffset", &value[0], sizeof(value))) {
        pitchLoop().setOutputOffset(atof(value));
        count++;
    }     
    return count;
}

int ConfigFileIO::load_HEADING_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/heading.txt")){
            error("HEADING File Read Error");
    }
    char value[BUFSIZ];
    
    if (cfg.getValue("PGain", &value[0] , sizeof(value))) {
        headingLoop().setControllerP(atof(value));
        count++;
    }
    if (cfg.getValue("IGain", &value[0] ,sizeof(value))) {
        headingLoop().setControllerI(atof(value));
        count++;
    }
    if (cfg.getValue("DGain", &value[0] , sizeof(value))) {
        headingLoop().setControllerD(atof(value));
        count++;
    }
    if (cfg.getValue("filterWn", &value[0], sizeof(value))) {
        headingLoop().setFilterFrequency(atof(value));
        count++;
    }
    if (cfg.getValue("deadband", &value[0], sizeof(value))) {
        headingLoop().setDeadband(atof(value));
        count++;
    }
    
    if (cfg.getValue("zeroOffset", &value[0], sizeof(value))) {
        headingLoop().setOutputOffset(atof(value));
        count++;
    }     
    return count;
}

int ConfigFileIO::load_RUDDER_config() {
    ConfigFile cfg;
    int count = 0;
    if (!cfg.read("/local/rudder.txt")){
            error("RUDDER File Read Error");
    }
    char value[BUFSIZ];
    
    //float values below
    if (cfg.getValue("setMinDeg", &value[0] , sizeof(value))) {
        rudder().setMinDeg(atof(value));
        count++;
    }
    if (cfg.getValue("setMaxDeg", &value[0] ,sizeof(value))) {
        rudder().setMaxDeg(atof(value));
        count++;
    }
    
    //integer values below
    if (cfg.getValue("setCenterPWM", &value[0] , sizeof(value))) {
        rudder().setCenterPWM(atof(value));
        count++;
    }
    if (cfg.getValue("setMinPWM", &value[0], sizeof(value))) {
        rudder().setMinPWM(atof(value));
        count++;
    }
    if (cfg.getValue("setMaxPWM", &value[0], sizeof(value))) {
        rudder().setMaxPWM(atof(value));
        count++;
    }
    return count;
}