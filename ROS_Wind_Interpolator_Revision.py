#!/usr/bin/env python
"""
   Created on: November 28, 2017
       Author: vldobr
   Updated on: March 11, 2019
       Author: Troy Holley

The script implements the COAMPS data interpolation service
To run the service: roslaunch action_nodes gpp_interpolate_server.launch
                        or
                    rosrun action_nodes gpp_interpolate_service_test_server.py

To send a request for service use the corresponding client part

"""

import numpy as np
import glob
import os  

from datetime import datetime
import pytz

import rospy
import rospkg

import hybrid_tiger_msgs.srv
import hybrid_tiger_msgs.msg
from geometry_msgs.msg import Vector3

from WIND.wind import WIND

# colors
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'
WHO_TXT = '(INTERPOLATION SERVICE)'

CYAN_DEBUG = '\033[36m' # NEW

# constants
sec2hours = 3600
RAD2DEG = 180/np.pi


class HDF5interp(object):
    # CLASS COUNTER
    _class_counter = 0

    def handle_interpolate_request(self, request):
        """This function interpolates HDF5 data
        Arguments:
            request: hybrid_tiger_msgs/InterpolateWeatherDataRequest message

        Returns:
            response: hybrid_tiger_msgs/InterpolateWeatherDataResponse message
        """

        ''' create an instance of the response message '''
        response = hybrid_tiger_msgs.srv.InterpolateWeatherDataResponse()
        len_resp = len(request.interpolation_points)

        ''' DEBUG the request id '''
        REQ_TEXT = str(request.request_id)
        rospy.loginfo('DEBUG - request_id: [%s]', (CYAN_DEBUG + REQ_TEXT + OKGREEN) )
        self.service_request_id = request.request_id
        
        ''' update wind data'''
        self.getHDF()

        ''' initialize empty placeholders '''
        response_data = []
        lat_q = np.array([])
        long_q = np.array([])
        alt_q = np.array([])
        time_q = np.array([])

        # Determine if the system is running the regular wind data generator or a wind_sim data generator
        if self.service_request_id == 55 and hasattr(self, 'wind_sim'):
            self.wind_sim.setWRun(1)
            
            for point in request.interpolation_points:
                lat_q = np.append(lat_q,point.vector.x*RAD2DEG)
                long_q = np.append(long_q, point.vector.y*RAD2DEG)
                alt_q = np.append(alt_q, point.vector.z)
                time_ref2seed = np.float((datetime.fromtimestamp(point.header.stamp.to_sec(), pytz.utc) -
                             self.wind_sim.wind_seedtime).total_seconds() / 3600.)
                time_q = np.append(time_q,time_ref2seed)

            ''' do the interpolation over all points in one step '''
            [UU, VV, WW, J, SLX, WWstar, PBLzht, RelHum, WFLX ] = self.wind_sim.coamps_wind_service(lat_q, long_q, alt_q, time_q)

            rospy.loginfo('< WIND SIM > %s interpolation step is done', (OKGREEN + WHO_TXT))
            
        # if the HDF file update is successful
        # Troy: Is this correct?  Check for wind when we know it exists in the init? Seems...strange
        # Vlad checks to see if this attribute exists, which seems like a roundabout way to do this
        elif hasattr(self, 'wind'):
            # update wind scale in case the HDF was updated
            self.wind.setWRun(1)
            #  parse the request
            for point in request.interpolation_points:
                lat_q = np.append(lat_q,point.vector.x*RAD2DEG)
                long_q = np.append(long_q, point.vector.y*RAD2DEG)
                alt_q = np.append(alt_q, point.vector.z)
                time_ref2seed = np.float((datetime.fromtimestamp(point.header.stamp.to_sec(), pytz.utc) -
                             self.wind.wind_seedtime).total_seconds() / 3600.)
                time_q = np.append(time_q,time_ref2seed)

            ''' do the interpolation over all points in one step '''
            [UU, VV, WW, J, SLX, WWstar, PBLzht, RelHum, WFLX ] = self.wind.coamps_wind_service(lat_q, long_q, alt_q, time_q)

            rospy.loginfo('< WIND_SIM >%s interpolation step is done', (OKGREEN + WHO_TXT))
        else:
            # if the update of HDF5 fails => send all zeros
            UU = np.zeros(len_resp)
            VV = np.zeros(len_resp)
            WW = np.zeros(len_resp)
            SLX = np.zeros(len_resp)
            WWstar = np.zeros(len_resp)
            PBLzht = np.zeros(len_resp)
            RelHum = np.zeros(len_resp)
            WFLX = np.zeros(len_resp)
            J = np.zeros([8, 4, len_resp])

            rospy.loginfo('%s failed to update HDF5 file, responding with all zeros', (FAIL + WHO_TXT))

        # build the ext_response in WxDataVector message format
        index = 0
        for point in request.interpolation_points:
            ''' re-package the response into HT format '''
            idx_response = hybrid_tiger_msgs.msg.WxData()
            # at each given WP
            idx_response.wp.wp_lla = point.vector
            idx_response.unix_time = request.interpolation_points[index].header.stamp.to_sec()
            # COAMPS insolation
            idx_response.insolation = SLX[index]

            idx_response.insolation_gradient.d_dx = J[3,0,index]
            idx_response.insolation_gradient.d_dy = J[3,1,index]
            idx_response.insolation_gradient.d_dz = J[3,2,index]
            idx_response.insolation_gradient.d_dt = J[3,3,index]

            # COAMPS tswflx
            idx_response.tswflx = WFLX[index]

            idx_response.tswflx_gradient.d_dx = J[7,0,index]
            idx_response.tswflx_gradient.d_dy = J[7,1,index]
            idx_response.tswflx_gradient.d_dz = J[7,2,index]
            idx_response.tswflx_gradient.d_dt = J[7,3,index]

            # COAMPS predicted wind vector averaged over segment (m/s)
            idx_response.mean_wind.x = VV[index]
            idx_response.mean_wind.y = UU[index]
            idx_response.mean_wind.z = WW[index]

            idx_response.wind_gradient.d_dx = self.convert2Vector3(J[0:3, 0, index])
            idx_response.wind_gradient.d_dy = self.convert2Vector3(J[0:3, 1, index])
            idx_response.wind_gradient.d_dz = self.convert2Vector3(J[0:3, 2, index])
            idx_response.wind_gradient.d_dt = self.convert2Vector3(J[0:3, 3, index])

            # Convective updraft scale velocity reported by COAMPS, averaged over the
            # flight segment (m/s)
            idx_response.w_star = WWstar[index]

            idx_response.w_star_gradient.d_dx = J[4, 0, index]
            idx_response.w_star_gradient.d_dy = J[4, 1, index]
            idx_response.w_star_gradient.d_dz = J[4, 2, index]
            idx_response.w_star_gradient.d_dt = J[4, 3, index]

            # COAMPS reported BL height averaged over segment (m)
            idx_response.boundary_layer_height = PBLzht[index]

            idx_response.boundary_layer_height_gradient.d_dx = J[5, 0, index]
            idx_response.boundary_layer_height_gradient.d_dy = J[5, 1, index]
            idx_response.boundary_layer_height_gradient.d_dz = J[5, 2, index]
            idx_response.boundary_layer_height_gradient.d_dt = J[5, 3, index]

            # relative humidity
            idx_response.relative_humidity = RelHum[index]

            idx_response.relative_humidity_gradient.d_dx = J[6, 0, index]
            idx_response.relative_humidity_gradient.d_dy = J[6, 1, index]
            idx_response.relative_humidity_gradient.d_dz = J[6, 2, index]
            idx_response.relative_humidity_gradient.d_dt = J[6, 3, index]

            ''' and build our response vector '''
            response_data.append(idx_response)
            index = index + 1

        # give it a timestamp so the bag is happy
        response.weather_data.header.stamp = rospy.Time.now()

        # and throw our response data in
        response.weather_data.data = response_data

        rospy.loginfo('%s WxDataVector message is formed', (OKGREEN + WHO_TXT))

        return response

    def convert2Vector3 (self, data):
        # helper for Vector3 type
        tmpVector = Vector3()
        tmpVector.x = data[0]
        tmpVector.y = data[1]
        tmpVector.z = data[2]
        return tmpVector

    def __init__(self, name):
        """ initializes  wind object and verifies if a new HDF5 file has been added to /src/resources """
        # NEW 03/12/2019
        #self.wind_sim = None # Troy: these should be INITIALIZED here not in the method
                
        self._action_name = name
        '''  initialize the service provider=> a name, a type, and a function handle'''
        self.service_request_id = None
        ''' NEW used to store request_id from InterpolateWeatherData.srv, has to come before services '''
        self._provider = rospy.Service( 'interpolate_weather_data',
            hybrid_tiger_msgs.srv.InterpolateWeatherData, self.handle_interpolate_request)
        ''' supply an HDF file '''
        self.getHDF()
        ''' shift the counter to skip the 'zero' step in interpolation '''
### GOOFY WAY OF DOING THIS...
        try:
            self.wind.setWRun (1)   # Vlad's method is to create this object in the getHDF method. There must be a better way to do this.
        except:
            pass
        try:
            self.wind_sim.setWRun (1) # Vlad's method is to create this object in the getHDF method. There must be a better way to do this.
        except:
            pass
        
        ''' NEW WIND FILE FROM Class WIND '''
        
        HDF5interp._class_counter += 1
        
        return

    def does_file_exist(self, path):
        return any([os.path.isfile(os.path.join(path,i)) for i in os.listdir(path) if i.endswith('.hdf5')])

    def get_old_file(self, curr_dir):
        ''' Find an older COAMPS file if a request comes in from the piccolo_wx_generator '''
        hdf5_files_list = [a_file for a_file in os.listdir(curr_dir) if a_file.endswith(".hdf5")]

        # Vlad's code checks for this already, but just in case...
        if len(hdf5_files_list) == 0:
            print "DEBUG - No COAMPS FILES FOUND!"
            return None

        elif len(hdf5_files_list) == 1:
            print "DEBUG - returning 1 file (From one file.):", hdf5_files_list[0]
            return hdf5_files_list[0]

        # The following logic is necessary to check the filenames (not file times)
        else:
            # 2 or more files. Get a file that is at least 24 hours old (if possible).
            old_file_to_return = None
            for hours_to_check in [24,16,18,12,6,0]:
                for index,a_file in enumerate(hdf5_files_list):
                    #print a_file[15:25]
                    current_file_datetime = datetime.datetime.strptime(a_file[15:25], '%Y%m%d%H')
                    newest_file_datetime = datetime.datetime.strptime(hdf5_files_list[-1][15:25], '%Y%m%d%H')
                    
                    # separated for clarification
                    # Files within 4 days (96 hours)
                    if current_file_datetime >= newest_file_datetime - datetime.timedelta(hours=96) and (current_file_datetime + datetime.timedelta(hours=hours_to_check)) <= newest_file_datetime:

                            # RETURN THE CORRECT FILE
                            old_file_to_return = hdf5_files_list[index]
                            time_diff = newest_file_datetime - current_file_datetime
                            print "DEBUG - File to return ({} hours difference)".format(time_diff)
                            return old_file_to_return

                # if there are no files within this pattern (within the 24/18/12/6 through 96 hour margin), use the second newest file
                if old_file_to_return == None:
                    old_file_to_return = hdf5_files_list[-2]
                    print "DEBUG - Using second latest file:", datetime.datetime.strptime(old_file_to_return[15:25], '%Y%m%d%H')
                    return old_file_to_return
                
                print "DEBUG - File to return:", datetime.datetime.strptime(old_file_to_return[15:25], '%Y%m%d%H')
                return old_file_to_return
    
    def getHDF(self):
        ''' Instantiate wind object using COAMPS file in the path relative to /src'''
        prj_path = os.path.join(rospkg.RosPack().get_path('gpp_resources'), 'resources/')
        rospy.loginfo('%s at dir_path %s',( OKGREEN + WHO_TXT), prj_path)
        
        rospy.loginfo('\033[95m DEBUG - prj_path is [%s] \033[92m', prj_path)

        # check if there is at least one HDF5 file before proceeding
        ''' Detect the Latest HDF5 file by using os and glob '''
        if self.does_file_exist(prj_path):
            if self.service_request_id == 55:  # piccolo_wx_generator_id
                #rospy.loginfo('\033[95m DEBUG - Processing Service Request from piccolo_wx_generator node \033[92m')
                ''' DETECT AN OLD COAMPS FILE '''
                old_coamps = self.get_old_file(prj_path)
                rospy.loginfo('>>>> OLD COAMPS %s pointing to coamps file %s',( OKGREEN + WHO_TXT), old_coamps)
    
                ''' Format the filename '''
                # len_old_coamps = len(old_coamps)
                old_coamps = old_coamps[:-5]       # got rid of weird method here
    
                if not hasattr(self, 'wind_sim'):
                    ''' initialize first time'''
                    rospy.loginfo(WHO_TXT + ' Initializing wind_sim using file [%s]', old_coamps)
                    
                    self.wind_sim = WIND (prj_path + old_coamps, np.array([0, 0, 0]) )
    
                    ''' set the wind origin at the point closest to the desired area '''
                    self.wind_sim.set_windOrig()
    
                    rospy.loginfo('%s is initialized with HDF file %s',
                                  (OKGREEN + WHO_TXT), prj_path + old_coamps)
    
                elif self.wind_sim.coamps_file != (prj_path + old_coamps):
                    ''' update wind data and then commanded airspeed and take off time'''
                    self.wind_sim.unpackHDF5file(prj_path + old_coamps)
                    #  update interpolant function as the grid changes across versions of HDFile
                    self.wind_sim.get_lambda0()
                    self.wind_sim.build_linearND_ll()
    
                    rospy.loginfo('%s Interpolation service has new HDF file %s',
                                  (OKGREEN + WHO_TXT),prj_path + old_coamps)

            else:
                # ORIGINAL WIND GENERATOR METHOD FROM VLAD
                newest_coamps = max(glob.iglob(prj_path + '*.hdf5'), key=os.path.getctime)
                rospy.loginfo('%s pointing to coamps file %s',( OKGREEN + WHO_TXT), newest_coamps)
    
                ''' Format the filename '''
                lnewest_coamps = len(newest_coamps)
                newest_coamps = newest_coamps[len(prj_path):lnewest_coamps - 5]
    
                if not hasattr(self, 'wind'):
                    ''' initialize first time'''
                    rospy.loginfo(WHO_TXT + ' Initializing wind_sim using file [%s]', newest_coamps)
                    
                    self.wind = WIND (prj_path + newest_coamps, np.array([0, 0, 0]) )
    
                    ''' set the wind origin at the point closest to the desired area '''
                    self.wind.set_windOrig()
    
                    rospy.loginfo('%s is initialized with HDF file %s',
                                  (OKGREEN + WHO_TXT), prj_path + newest_coamps)
    
                elif self.wind.coamps_file != (prj_path + newest_coamps):
                    ''' update wind data and then commanded airspeed and take off time'''
                    self.wind.unpackHDF5file(prj_path + newest_coamps)
                    #  update interpolant function as the grid changes across versions of HDFile
                    self.wind.get_lambda0()
                    self.wind.build_linearND_ll()
    
                    rospy.loginfo('%s Interpolation service has new HDF file %s',
                                  (OKGREEN + WHO_TXT),prj_path + newest_coamps)
        else:
            # Added logic to process correct attribute (wind vs wind_sim)
            if self.service_request_id == 55:
                rospy.loginfo('%s failed to find an HDF file at %s', (FAIL + WHO_TXT), prj_path)
                                #  delete wind attribute
                if hasattr(self, 'wind_sim'):
                    del self.wind_sim
            else:
                rospy.loginfo('%s failed to find an HDF file at %s', (FAIL + WHO_TXT), prj_path)
                #  delete wind attribute
                if hasattr(self, 'wind'):
                    del self.wind


if __name__ == "__main__":
    # init a node
    rospy.init_node('weather_interpolation_service')
    try:
        service = HDF5interp(rospy.get_name())
        rospy.loginfo('%s is up and running', WHO_TXT) #  interpolation service is up and running
        rospy.spin()
    except AttributeError:
        rospy.loginfo('%s  failed to start because HDF5 file is not supplied', (FAIL + WHO_TXT))
