#!/usr/bin/env python
'''
Author:           Troy Holley
Title:            troy_rosbag_to_csv_xlsx.py
Date:             Modified 02/21/2019
Version:          0.6
Description:      This script reads bag files and converts them to CSV and XLSX files.
Python Version:   2.7.6
System:           Odroid ARM processor using Linux Ubuntu 14.04
Notes:            Tried a bunch of methods to speed up Excel portion.
                  Found out I could write wholesale CSV files to a sheet using a Pandas dataframe.
                  This made the code speed up from 20 minutes to a few minutes for large 90+ MB bag files.
'''

import rosbag
import csv
import sys            # for system arguments
import time
import string
import os             # for file management make directory (listdirs and makedirs)
#import xlsxwriter     # for writing files to the 

import pandas as pd

# verify correct input arguments: 1 or 2 by checking sys.argv (contains the list of command-line arguments)
if (len(sys.argv) > 2):
    print "invalid number of arguments:   " + str(len(sys.argv))
    print "should be 2: 'bag2csv.py' and 'bagName'"
    print "or one argument  : 'bag2csv.py'"
    sys.exit(1)

# process one bag file
elif (len(sys.argv) == 2):
    bag_filename = sys.argv[1]
    print "reading one bagfile: " + bag_filename
    
else:
    print "bad argument(s): " + str(sys.argv)    #shouldnt really come up
    sys.exit(1)
    
current_dir = os.path.dirname(os.path.realpath(__file__))
new_directory = "2019-02-19-17-20-33_2"

# Decorator for timing these functions
# Prints function name and time to complete
def timerDecorator(a_func):
    def wrapTheFunction():
        start_time = time.time()
        a_func()
        end_time = time.time()
        print "Function {} took {:.2f} seconds to complete.".format(a_func.__name__,(end_time-start_time))
    # You MUST return the function
    return wrapTheFunction

@timerDecorator  
def bagFileToCsv():
    global new_directory     
    print "reading bagfile: " + bag_filename
    
    #access ROS bag, this takes a few seconds to load
    bag = rosbag.Bag(bag_filename)
    bag_data = bag.read_messages()
    
    orig_bag_filename = bag.filename
 
     #create a new directory
    new_directory = string.rstrip(orig_bag_filename, ".bag")
    try:    #else already exists
        os.makedirs(new_directory) # makes it in the same folder as the python file
    except:
        pass 
 
    #get list of topics from the bag
    # this part is super slow, needs to be sped up
    topic_list = []
    for topic, msg, t in bag_data:
        if topic not in topic_list:
            topic_list.append(topic)
 
    # DEBUG
    print "DEBUG - topic_list: ", topic_list # confirmed

    # PROCESS EACH TOPIC (And process as a sheet in the excel file)
    for topic_name in topic_list:
        # empty the dictionary for each topic
        topic_data_dict = {}
    
        #Create a new CSV file for each topic
        # make something that Excel can read later on (single quote)
        filename = new_directory + '/' + string.replace(topic_name, '/', '_\'_') + '.csv'
        # need a good separator for the subtopics
         
        with open(filename, 'w+') as csvfile:
            # WRITER = Return a writer object responsible for converting the user's data into delimited strings on the given file-like object.
            filewriter = csv.writer(csvfile, delimiter = ',')
             
            first_iteration_for_header = True    #allows header row
            
            #EXCEL ROW COUNTER (first row = 0 is the header)
            row_count = 1
            
            # Read each subtopic at each time interval    
            for subtopic, msg, rb_time in bag.read_messages(topic_name):            
                
                # make sure the message is formatted as a string
                msg_string = str(msg)
                
                # convert the message into a list of strings
                msg_list = string.split(msg_string, '\n')
                               
                # process the message list into matching pairs of data
                # for example one item in the msg_list is 'header: <empty>' OR '  seq: 180468'
                # these need to be split apart...again
                
                data_pair_list = []     # list of data transformed
                
                for orginal_data in msg_list:
                    temp_pair = string.split(orginal_data, ':') # split the data again item by item in msg_list
                    #print "DEBUG - temp_pair:", temp_pair
                    
                    for i in range(len(temp_pair)):    #should be 0 to 1
                        temp_pair[i] = string.strip(temp_pair[i])
                    data_pair_list.append(temp_pair)
                    
                # write the header using the first item/key from each pair
                if first_iteration_for_header:    # header
                    headers = ["rosbagTimestamp"]    #first column header
                    for pair in data_pair_list:
                        headers.append(pair[0])
                        
                    filewriter.writerow(headers)
                    first_iteration_for_header = False
                    
                
                # go through the pairs of data, append each value to the values list
                # first column uses rosbag timestamp
                values = [str(t)]
                for pair in data_pair_list:
                    if len(pair) > 1:
                        values.append(pair[1])
                
                # write each row to its CSV file
                filewriter.writerow(values)
                
    # close the bag file when complete
    bag.close()

@timerDecorator
def csvToXlsx():        
    sub_dir = str(current_dir + '/' + new_directory + '/')
    #print "DEBUG - sub_dir", sub_dir
    
    # Create the Excel spreadsheet file
    writer = pd.ExcelWriter(sub_dir + '/' + new_directory + '.xlsx')
    
    # get all of the CSV files in the directory
    csv_file_list = []
    for a_file in os.listdir(sub_dir):
        #"DEBUG - a_file", a_file
        if a_file.endswith(".csv"):
            csv_file_list.append(a_file)
            
    # use csv_file_list to process each file
    for csv_file in csv_file_list:
        # SHEET NAME
        cur_sheet_name = csv_file[:-4]
        #cur_sheet_name = cur_sheet_name.replace("_slash_","_\'_") # REMOVE
        cur_sheet_name = csv_file[:31] # first 31 lines

        # READ CSV FILES
        # skipping "bad lines" because some don't match header
        # The bag files are have data that doesn't match headers on some lines
        df = pd.read_csv(sub_dir + csv_file, error_bad_lines=False)

        # write dataframe to excel sheet
        df.to_excel(writer,sheet_name=cur_sheet_name,index = False)
    
    # save full Excel workbook
    writer.save()

if __name__ == "__main__":
    bagFileToCsv()
    csvToXlsx()
    
    


# The bag parses data per time interval, basically in columns of data
# example of data read in a msg_string:

# DEBUG msg_string header:
#   seq: 180468
#   stamp:
#     secs: 1550596833
#     nsecs: 299152553
#   frame_id: ''
# Insolation: 944.941345215
# Azimuth_deg: 180.524490356
# Elevation_deg: 38.9996490479
# Zenith_deg: 51.0003509521
# Sunrise: 11.7251577377
# Sunset: 22.4337768555
# Daylight: 10.7086191177
# Declination_deg: 11.5590333939
# Global_Horizontal_Insolation_Wm2: 594.666381836
# Eta_attitude: 0.336743712425
# Tas: 18.1958656311
# Wind_West: 0.0125937378034
# Wind_South: 0.0174806509167
# GroundSpeed: 18.4236507416
# GroundTrack: 1.16811394691
# HeadWind: -0.0184368547052
# Rpm: 4332.0
# Netto_Variometer_Signal: 0.230280727148
# Specific_Energy_State: 240294.671875
# Specific_Energy_Rate: 0.0
# Specific_Energy_Acceleration: 0.0
# PV_expected_W: 119.343177795
# Wind_Cov0: 0.0551566928625
# Wind_Cov1: 0.172179743648
# Wind_Cov2: 0.0992919355631
