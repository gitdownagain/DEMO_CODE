'''
Author:           Troy Holley
Title:            FSG_plot_and_save_to_excel.py
Date:             Modified 02/08/2019
Version:          0.6
Description:      This script reads bag files and converts them to CSV and XLSX files.
Python Version:   2.7.13
System:           Windows 7 64-bit
Notes:            Graph data to a PyQT gui from the LOG000.csv log files and save the data as an excel file also.
'''

# create listbox from list

import csv        # read csv
import xlsxwriter # create XLSX excel files for data analysis, may do something different later

## EXCEL CELLS
from string import ascii_uppercase # for ascii A to Z and AA to ZZ

## FILE NAME
import datetime
from dateutil.tz import tzlocal

# GRAPH GUI 
from PyQt4 import QtGui
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar

import sys  # do we need the arguments?

## TEST NUMPY
import numpy                # array, vstack, etc.


## PLOT GRAPHS
import matplotlib.pyplot as plt

class PrettyWidget(QtGui.QWidget):
    """Print each column by passing the y-list of data to it"""
    
    def __init__(self, control_list,x_list,y_list, data_list):    # control list is NOT optional
        super(PrettyWidget, self).__init__()

        self.x_list = x_list
        self.y_list = y_list
        self.data_list = data_list

        # GRAPH DATA
        self.graph_index = 0 # used to traverse the graph properly
        self.graph_list = [tuple( [ control_list[x],control_list[x+1] ] ) for x, value in enumerate(control_list) if x < len(control_list)-1]

        list_header = ['st_ID#', 'timer', 'depth_cmd', 'depth_ft', 'pitch_cmd', 'pitch_deg', 'rud_deg', 'h_deg', 'bce_cmd', 'bce_mm', 'batt_cmd', 'batt_mm', 'pitchRate_degs', 'depthRate_fps', 'sys_amps', 'sys_volts', 'int_PSI']
        y_axis_unit_list = ['string','sec','feet','feet','deg','deg','deg','(heading)deg','mm (bce)','mm (cmd)','mm (batt)','deg/sec','feet/sec','amps','volts','(internal) psi']        
        self.y_axis_units = dict(zip(list_header, y_axis_unit_list))

        self.initUI() # convenient function to create your display

    def initUI(self):
        self.setGeometry(600,300,1000,600)
        self.center()
        self.setWindowTitle('FSG Log File Graphs')
        # NOTE TO SELF: The above code is just setting things up

        grid = QtGui.QGridLayout()
        self.setLayout(grid)
        # PyQt4 supports a grid layout, for example calculator buttons

        btn1 = QtGui.QPushButton('Plot 1', self)
        btn1.resize(btn1.sizeHint())    # gives an automatic/recommended size for the button
        btn1.clicked.connect(self.plot1)
        grid.addWidget(btn1,2,0)

        btn2 = QtGui.QPushButton('Plot 2', self)
        btn2.resize(btn1.sizeHint())    # gives an automatic/recommended size for the button
        btn2.clicked.connect(self.plot2)
        grid.addWidget(btn2,2,1)

        # https://stackoverflow.com/questions/332289/how-do-you-change-the-size-of-figures-drawn-with-matplotlib
        self.figure = plt.figure( figsize=(15,5) ) # is this inches?
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        grid.addWidget(self.canvas,1,0,1,2)     # The canvas is a general purpose widget, which is typically used to display and edit graphs and other drawings.
        grid.addWidget(self.toolbar,0,0,1,2)
        # NOTE TO SELF: Still just adding buttons a nd stuff, NOTHING DISPLAYED!

        self.comboBoxGraph = QtGui.QComboBox(self)
        # self.comboBoxGraph.addItem('COLUMN TO GRAPH')
        # add to the combo box
        for item in self.data_list:
            self.comboBoxGraph.addItem(item)
        grid.addWidget(self.comboBoxGraph,3,0)     # bottom left position
        
        self.show()


    def plot1(self):
        cb_text1 = str(self.comboBoxGraph.currentText())

        # when you hit this function the index changes
        self.graph_index -= 1 # used to traverse the graph properly
        self.graph_index = self.graph_index % len(self.graph_list)  # used to traverse the graph properly


        print("graph index", self.graph_index)

        front = self.graph_list[self.graph_index][0]
        back = self.graph_list[self.graph_index][1]
        
        plt.cla()
        plt.xlabel('SECONDS') 
        plt.ylabel(self.y_axis_units[cb_text1]) # pull y-axis labels from dictionary
        
        ax = self.figure.add_subplot(111)

        x = self.x_list[front:back]

        #y = self.y_list[front:back]
        y = self.data_list[cb_text1][front:back] # subset of the dictionary data

        ax.plot(x,y, 'b.-')

        title = "Time (sec) VS " + cb_text1 + " (Work in Progress)"
        ax.set_title(title)
        self.canvas.draw()


    def plot2(self):
        cb_text2 = str(self.comboBoxGraph.currentText())

        self.graph_index += 1    # for readability
        self.graph_index = self.graph_index % len(self.graph_list)  # used to traverse the graph properly
        
        print("graph index", self.graph_index)
        print(self.graph_list[self.graph_index])

        front = self.graph_list[self.graph_index][0]
        back = self.graph_list[self.graph_index][1]

        ## clear the plot and regraph the new stuff
        plt.cla()
        plt.xlabel('SECONDS') 
        plt.ylabel(self.y_axis_units[cb_text2]) # pull y-axis labels from dictionary

        ax2 = self.figure.add_subplot(111)
        x = self.x_list[front:back]
        #y = self.y_list[front:back]
        y = self.data_list[cb_text2][front:back] # subset of the dictionary data
        
        ax2.plot(x,y, 'r.-')

        #title = "Time (sec) vs depth (FT)"        
        title = "Time (sec) VS " + cb_text2 + " (Work in Progress)"
        ax2.set_title(title)
        self.canvas.draw()

    def center(self):
        qr = self.frameGeometry()                                   # geometry of the widget relative to its parent including any window frame
        center_point = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(center_point)
        self.move(qr.topLeft())
        # DOUBLE CHECK WHAT THIS IS DOING!

class graphCsv(object):
    def __init__(self):        
        #lists filled from CSV file
        self.heading_strings = []

        self.graph_dict = {}
        self.graph_list = []

        self.state_ID = []
        self.timer_sec = []
        self.depth_cmd = []
        self.depth_ft = []
        self.pitch_cmd = []
        self.pitch_deg = []
        self.rud_deg = []
        self.h_deg = []
        self.bce_cmd = []
        self.bce_mm = []
        self.batt_cmd = []
        self.batt_mm = []
        self.ptc_rate = []
        self.dep_rate = []
        self.sys_amps = []
        self.sys_volts = []
        self.int_PSI = []

        self.positions_to_plot = [2] #positions from 1 to n, heading is 1
        self.row_index = 0

        self.numpy_array = numpy.empty((1,1))    

    def getDataFromCsv(self):
        with open('LOG000.csv', 'rb') as csvfile:
            print "DEBUG: file open!"
            
            spamreader = csv.reader(csvfile, delimiter= ',')

            #print spamreader # DEBUG

            for row in spamreader:
                #print row # print all the rows (DEBUG)

                # READ HEADING
                if self.row_index == 0:
                    # set this to the first list of strings
                    self.heading_strings = row
                else:
                    # add ALL data to a list you will graph
                    self.state_ID.append(int(row[1]))
                    self.timer_sec.append(float(row[2]))
                    self.depth_cmd.append(float(row[3]))
                    self.depth_ft.append(float(row[4]))
                    self.pitch_cmd.append(float(row[5]))
                    self.pitch_deg.append(float(row[6]))
                    self.rud_deg.append(float(row[7]))
                    self.h_deg.append(float(row[8]))
                    self.bce_cmd.append(float(row[9]))
                    self.bce_mm.append(float(row[10]))
                    self.batt_cmd.append(float(row[11]))
                    self.batt_mm.append(float(row[12]))
                    self.ptc_rate.append(float(row[13])) #rename these in the file
                    self.dep_rate.append(float(row[14])) #rename these in the file
                    self.sys_amps.append(float(row[15]))
                    self.sys_volts.append(float(row[16]))
                    self.int_PSI.append(float(row[17]))
                
                # check field 2 in the row to find the SIT_IDLE state
                if row[1] == "00":
                    #self.graphFollowingData()
                    self.positions_to_plot.append(self.row_index)
                    #print row

                #APPEND INDEX AT THE END
                self.row_index = self.row_index + 1

            # MAKE A DICTIONARY
            self.graph_dict['st_ID#'] = self.state_ID
            self.graph_dict['timer'] = self.timer_sec
            self.graph_dict['depth_cmd'] = self.depth_cmd
            self.graph_dict['depth_ft'] = self.depth_ft
            self.graph_dict['pitch_cmd'] = self.pitch_cmd
            self.graph_dict['pitch_deg'] = self.pitch_deg
            self.graph_dict['rud_deg'] = self.rud_deg
            self.graph_dict['h_deg'] = self.h_deg
            self.graph_dict['bce_cmd'] = self.bce_cmd
            self.graph_dict['bce_mm'] = self.bce_mm
            self.graph_dict['batt_cmd'] = self.batt_cmd
            self.graph_dict['batt_mm'] = self.batt_mm
            self.graph_dict['pitchRate_degs'] = self.ptc_rate
            self.graph_dict['depthRate_fps'] = self.dep_rate
            self.graph_dict['sys_amps'] = self.sys_amps
            self.graph_dict['sys_volts'] = self.sys_volts
            self.graph_dict['int_PSI'] = self.int_PSI

    ## CALL BY NAME!
            #print("DEBUG: THE DICTIONARY")
            #print(self.graph_dict['int_PSI'])

            ## make list of lists
            self.graph_list = [self.state_ID, self.timer_sec,self.depth_cmd,self.depth_ft,self.pitch_cmd,self.pitch_deg,self.rud_deg,self.h_deg,self.bce_cmd,self.bce_mm,self.batt_cmd,self.batt_mm,self.ptc_rate,self.dep_rate,self.sys_amps,self.sys_volts,self.int_PSI]
            
            # CHECK THE DATA
            #print "Positions to plot"
            #print self.positions_to_plot
            #print self.heading_strings
            #print timer_sec

#####            # SAVE TO NUMPY ARRAY

        # APPEND TO EACH ROW WITH VSTACK
##        self.numpy_array = numpy.array(self.state_ID)
##
##        self.numpy_array = numpy.vstack( [self.numpy_array,numpy.array(self.timer_sec)] )
##
##        print("self.numpy_array", self.numpy_array)
        
##        self.state_ID = []
##        self.timer_sec = []
##        self.depth_cmd = []
##        self.depth_ft = []
##        self.pitch_cmd = []
##        self.pitch_deg = []
##        self.rud_deg = []
##        self.h_deg = []
##        self.bce_cmd = []
##        self.bce_mm = []
##        self.batt_cmd = []
##        self.batt_mm = []
##        self.ptc_rate = []
##        self.dep_rate = []
##        self.sys_amps = []
##        self.sys_volts = []
##        self.int_PSI = []

        # print(self.state_ID) #DEBUG

    def createSingleTestPlot(self):
        print "Create Test Plot"

        print len(self.positions_to_plot)

        test_x_plot = []
        test_y_plot = []
        
        #for x in range(len(self.positions_to_plot)-1):
        # enumerate gets the full range
        for index,value in enumerate(self.positions_to_plot):
            # prevent the index out of range error
            if index > (len(self.positions_to_plot) - 2):
                break

            print index

            front = self.positions_to_plot[index]
            back = self.positions_to_plot[index+1] 

        test_x_plot = self.timer_sec[front:back]
        test_y_plot = self.depth_ft[front:back]

        #print test_x_plot  #DEBUG
            
        plt.plot(test_x_plot, test_y_plot) # x and y
        plt.show()

        # https://stackoverflow.com/questions/45140085/generate-multiple-plots-with-for-loop-display-output-in-matplotlib-subplots

        ###########################################################################################
        ###########################################################################################
        ###########################################################################################

    def createMultipleTestPlots(self):
        test_x_plot = []
        test_y_plot = []

        for index,value in enumerate(self.positions_to_plot):
            # prevent the index out of range error
            if index > (len(self.positions_to_plot) - 2):
                break

            print index

            front = self.positions_to_plot[index]
            back = self.positions_to_plot[index+1]


        ## NEED TO FIX THIS
        first = self.positions_to_plot[0]
        second = self.positions_to_plot[1]

        test_x_plot1 = self.timer_sec[first:second]
        test_y_plot1 = self.timer_sec[first:second]
        ## NEED TO FIX THIS

        test_x_plot2 = self.timer_sec[front:back]
        test_y_plot2 = self.depth_ft[front:back]

        plt.subplot(2, 1, 1)
        plt.plot(test_x_plot1, test_y_plot1, 'o-')
        plt.title('A tale of 2 subplots')
        plt.ylabel('Damped oscillation')

        plt.subplot(2, 1, 2)
        plt.plot(test_x_plot2, test_y_plot2, '.-')
        plt.xlabel('time (s)')
        plt.ylabel('Undamped')

        plt.show()


        ###########################################################################################
        ###########################################################################################
        ###########################################################################################

    def createXlsxFile(self):
        print self.heading_strings
        # CREATE THE FILE
        # Get the current date/time with the timezone.
        now = datetime.datetime.now(tzlocal())

        # save the formatted time
        current_time = now.strftime("%Y-%m-%d %H_%M")

        current_time_file = current_time + '.xlsx' 
        workbook = xlsxwriter.Workbook(current_time_file)
        
        #add sheet
        worksheet = workbook.add_worksheet()

        # bold format
        bold = workbook.add_format({'bold': 1})

        #create reference data for worksheet
        # start with a row from A1

        worksheet.write_row('A1',self.heading_strings, bold) # bold cause why not
        
        #worksheet.write_row('A1', headings, bold) # bold cause why not
        worksheet.write_column('B2', self.state_ID)
        worksheet.write_column('C2', self.timer_sec)
        worksheet.write_column('D2', self.depth_cmd)
        worksheet.write_column('E2', self.depth_ft)
        worksheet.write_column('F2', self.pitch_cmd)
        worksheet.write_column('G2', self.pitch_deg)
        worksheet.write_column('H2', self.rud_deg)
        worksheet.write_column('I2', self.h_deg)
        worksheet.write_column('J2', self.bce_cmd)
        worksheet.write_column('K2', self.bce_mm)
        worksheet.write_column('L2', self.batt_cmd)
        worksheet.write_column('M2', self.batt_mm)
        worksheet.write_column('N2', self.ptc_rate)
        worksheet.write_column('O2', self.dep_rate)
        worksheet.write_column('P2', self.sys_amps)
        worksheet.write_column('Q2', self.sys_volts)
        worksheet.write_column('R2', self.int_PSI)

        # print(self.state_ID) # DEBUG to see the list of data

        # CREATE A SCATTER CHART with connected straight lines and markers
        chart1 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart2 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart3 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart4 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart5 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart6 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart7 = workbook.add_chart({'type': 'scatter',
                'subtype': 'straight_with_markers'})
        chart8 = workbook.add_chart({'type': 'scatter',
                    'subtype': 'straight_with_markers'})
        
        # NEED TO FIGURE OUT HOW TO READ SERIES

        # can add multiple series of data to a chart
        #for x in len(self.positions_to_plot) - 1:

        # FIRST SERIES

        end_row_1 = self.positions_to_plot[1]-1

        #DEPTH
        chart1.add_series({
            'name':         ['Sheet1',0,3],
            'categories':   ['Sheet1',1,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',1,3,end_row_1,3], #row, column, row, column # depth_cmd
        })

        chart1.add_series({
            'name':         ['Sheet1',0,4],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,4,end_row_1,4], #row, column, row, column # depth_ft
            'y2_axis':      1,
        })

        #PITCH
        chart2.add_series({
            'name':         ['Sheet1',0,5],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,5,end_row_1,5], #row, column, row, column # pitch_cmd
        })

        chart2.add_series({
            'name':         ['Sheet1',0,6],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,6,end_row_1,6], #row, column, row, column # pitch_deg
            'y2_axis':      1,
        })

        #HEADING
        chart3.add_series({
            'name':         ['Sheet1',0,7],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,7,end_row_1,7], #row, column, row, column # rud_deg
            'y2_axis':      1,
        })

        chart3.add_series({
            'name':         ['Sheet1',0,8],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',1,8,end_row_1,8], #row, column, row, column # h_deg
        })

        # DEPTH VS DEPTH RATE
        chart4.add_series({
            'name':         ['Sheet1',0,4],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,4,end_row_1,4], #row, column, row, column # depth_ft
        })

        chart4.add_series({
            'name':         ['Sheet1',0,14],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,14,end_row_1,14], #row, column, row, column # depth_rate_fps
            'y2_axis':      1,
        })


        #HEADING vs DEPTH_FT
        chart5.add_series({
            'name':         ['Sheet1',0,4],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,4,end_row_1,4], #row, column, row, column # depth_ft
            'y2_axis':      1,
        })

        chart5.add_series({
            'name':         ['Sheet1',0,8],
            'categories':   ['Sheet1',2,2,end_row_1,2], #row, column, row, column
            'values':       ['Sheet1',2,8,end_row_1,8], #row, column, row, column # h_deg
        })



        # FIRST SERIES OF CHARTS
        chart1.set_size({'x_scale': 2, 'y_scale': 2})
        chart2.set_size({'x_scale': 2, 'y_scale': 2})
        chart3.set_size({'x_scale': 2, 'y_scale': 2})
        chart4.set_size({'x_scale': 2, 'y_scale': 2})
        chart5.set_size({'x_scale': 2, 'y_scale': 2})
        #chart6.set_size({'x_scale': 2, 'y_scale': 2})
        #chart7.set_size({'x_scale': 2, 'y_scale': 2})
        #chart8.set_size({'x_scale': 2, 'y_scale': 2})
    
        # Insert the chart into the worksheet (with an offset).
        worksheet.insert_chart( 'D10', chart1, {'x_offset': 0, 'y_offset': 0})
        worksheet.insert_chart( 'D50', chart2, {'x_offset': 0, 'y_offset': 0})
        worksheet.insert_chart( 'D90', chart3, {'x_offset': 0, 'y_offset': 0})
        worksheet.insert_chart('D130', chart4, {'x_offset': 0, 'y_offset': 0})
        worksheet.insert_chart('D170', chart5, {'x_offset': 0, 'y_offset': 0})
        #worksheet.insert_chart('D210', chart6, {'x_offset': 0, 'y_offset': 0})
        #worksheet.insert_chart('D250', chart7, {'x_offset': 0, 'y_offset': 0})
        #worksheet.insert_chart('D290', chart8, {'x_offset': 0, 'y_offset': 0})


### A TO Z, AA TO ZZ ###
        excel_abc = []

        for one in ascii_uppercase:
            excel_abc.append(one)

        for one in ascii_uppercase:
            for two in ascii_uppercase:
                excel_abc.append(one+two)
### A TO Z, AA TO ZZ ###
        chart_list = []

        # the for loop limits the data from the second sit_idle to the last (will not try to graph last + something)
        # list is 0,1,2...n

        chart_index = 0
        
        for row_sit_idle in range(1,len(self.positions_to_plot)-1):
            # 8 charts for each set
            for x in range(8):
                # left and right axis graphing
                chart_list.append(workbook.add_chart({'type': 'scatter',
                    'subtype': 'straight_with_markers'}))

            #XLSXWRITER
            # print( workbook.add_chart({'type': 'scatter','subtype': 'sraight_with_markers'}) )

            # PARAMETERS USED TO SEPARATE DATA INTO DIFFERENT SERIES
            start_row = self.positions_to_plot[row_sit_idle]    # this is based on the position of sit idle
            end_row = self.positions_to_plot[row_sit_idle + 1]

            #print("DEBUG: start_row:", start_row)
            #print("DEBUG:   end_row:", end_row)

            #DEPTH
            chart_list[0 + chart_index].add_series({
                'name':         ['Sheet1',0,3],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,3,end_row,3], #row, column, row, column # depth_cmd
            })

            chart_list[0 + chart_index].add_series({
                'name':         ['Sheet1',0,4],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,4,end_row,4], #row, column, row, column # depth_ft
                'y2_axis':      1,
            })

            #PITCH
            chart_list[1 + chart_index].add_series({
                'name':         ['Sheet1',0,5],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,5,end_row,5], #row, column, row, column # pitch_cmd
            })

            chart_list[1 + chart_index].add_series({
                'name':         ['Sheet1',0,6],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,6,end_row,6], #row, column, row, column # pitch_deg
                'y2_axis':      1,
            })

            #HEADING
            chart_list[2 + chart_index].add_series({
                'name':         ['Sheet1',0,7],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,7,end_row,7], #row, column, row, column # pitch_cmd
            })

            chart_list[2 + chart_index].add_series({
                'name':         ['Sheet1',0,8],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,8,end_row,8], #row, column, row, column # pitch_deg
                'y2_axis':      1,
            })

            # DEPTH VS DEPTH RATE
            chart_list[3 + chart_index].add_series({
            'name':         ['Sheet1',0,4],
            'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
            'values':       ['Sheet1',start_row,4,end_row,4], #row, column, row, column # depth_ft
            })

            chart_list[3 + chart_index].add_series({
                'name':         ['Sheet1',0,14],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,14,end_row,14], #row, column, row, column # depth_rate_fps
                'y2_axis':      1,
            })

            #HEADING vs DEPTH_FT
            chart_list[4 + chart_index].add_series({
                'name':         ['Sheet1',0,4],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,4,end_row,4], #row, column, row, column # depth_ft
                'y2_axis':      1,
            })

            chart_list[4 + chart_index].add_series({
                'name':         ['Sheet1',0,8],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,8,end_row,8], #row, column, row, column # h_deg
                'y2_axis':      1,
            })

            #bce cmd/mm pos J/K or 9/10
            chart_list[5 + chart_index].add_series({
                'name':         ['Sheet1',0,8],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,8,end_row,8], #row, column, row, column # h_deg
            })
            
            chart_list[5 + chart_index].add_series({
                'name':         ['Sheet1',0,9],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,9,end_row,9], #row, column, row, column # bce_cmd
                'y2_axis':      1,
            })
            
            chart_list[5 + chart_index].add_series({
                'name':         ['Sheet1',0,10],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,10,end_row,10], #row, column, row, column # bce_mm
                'y2_axis':      1,
            })

            #bce cmd/mm pos J/K or 9/10
            chart_list[6 + chart_index].add_series({
                'name':         ['Sheet1',0,8],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,8,end_row,8], #row, column, row, column # h_deg
            })
            
            chart_list[6 + chart_index].add_series({
                'name':         ['Sheet1',0,11],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,11,end_row,11], #row, column, row, column # batt_cmd
                'y2_axis':      1,
            })
            
            chart_list[6 + chart_index].add_series({
                'name':         ['Sheet1',0,12],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,12,end_row,12], #row, column, row, column # batt_mm
                'y2_axis':      1,
            })

            ### HEADING vs BATT vs BCE
            chart_list[7 + chart_index].add_series({
                'name':         ['Sheet1',0,8],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,8,end_row,8], #row, column, row, column # h_deg
            })
            
            chart_list[7 + chart_index].add_series({
                'name':         ['Sheet1',0,12],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,12,end_row,12], #row, column, row, column # batt_mm
                'y2_axis':      1,
            })
            
            chart_list[7 + chart_index].add_series({
                'name':         ['Sheet1',0,10],
                'categories':   ['Sheet1',start_row,2,end_row,2], #row, column, row, column
                'values':       ['Sheet1',start_row,10,end_row,10], #row, column, row, column # bce_mm
                'y2_axis':      1,
            })

        ## add to chart with the position of things with letters in excel_abc list
            ##D4, D24
            ## excel_abc[3] = D

            # THIS CREATES LETTERS, FOR EXAMPL "X290"
            cell_1 = excel_abc[3 + 20 * row_sit_idle] + '10'                  # example 3 + 20, 3 + 40
            cell_2 = excel_abc[3 + 20 * row_sit_idle] + '50'
            cell_3 = excel_abc[3 + 20 * row_sit_idle] + '90'
            cell_4 = excel_abc[3 + 20 * row_sit_idle] + '130'
            cell_5 = excel_abc[3 + 20 * row_sit_idle] + '170'
            cell_6 = excel_abc[3 + 20 * row_sit_idle] + '210'
            cell_7 = excel_abc[3 + 20 * row_sit_idle] + '250'
            cell_8 = excel_abc[3 + 20 * row_sit_idle] + '290'

            # EXCEL CHART STYLE chart style is 2: scatter chart with straight lines and markers
            chart_list[0 + chart_index].set_style(1)
            chart_list[1 + chart_index].set_style(2)
            chart_list[2 + chart_index].set_style(3)
            chart_list[3 + chart_index].set_style(12)
            # chart_list[4 + chart_index].set_style(12)
            # chart_list[5 + chart_index].set_style(12)
            # chart_list[6 + chart_index].set_style(12)

            # EXCEL CHART SIZE
            chart_list[0 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[1 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[2 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[3 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[4 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[5 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[6 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})
            chart_list[7 + chart_index].set_size({'x_scale': 2, 'y_scale': 2})

            #ADDING THE TITLE AND AXIS LABELS
            chart_list[4 + chart_index].set_title({'name': 'Heading (deg) vs Depth (ft)'})
            chart_list[4 + chart_index].set_x_axis({'name': 'time (sec) (UTC)', })
            chart_list[4 + chart_index].set_y_axis({'name': 'heading (deg)'})
            chart_list[4 + chart_index].set_y2_axis({'name': 'depth (ft)'})

            chart_list[5 + chart_index].set_title({'name': 'Heading (deg) vs BCE'})
            chart_list[5 + chart_index].set_x_axis({'name': 'time (sec) (UTC)', })
            chart_list[5 + chart_index].set_y_axis({'name': 'bce (cmd)'})
            chart_list[5 + chart_index].set_y2_axis({'name': 'bce (mm)'})

            chart_list[6 + chart_index].set_title({'name': 'Heading (deg) vs BATT/BMM'})
            chart_list[6 + chart_index].set_x_axis({'name': 'time (sec) (UTC)', })
            chart_list[6 + chart_index].set_y_axis({'name': 'batt (cmd)'})
            chart_list[6 + chart_index].set_y2_axis({'name': 'batt (mm)'})

            chart_list[7 + chart_index].set_title({'name': 'Heading (deg) vs BATT BCE POS (mm)'})
            chart_list[7 + chart_index].set_x_axis({'name': 'time (sec) (UTC)', })
            chart_list[7 + chart_index].set_y_axis({'name': 'batt (mm)'})
            chart_list[7 + chart_index].set_y2_axis({'name': 'bce (mm)'})

            # INSERT CELLS
            worksheet.insert_chart(cell_1, chart_list[0 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_2, chart_list[1 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_3, chart_list[2 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_4, chart_list[3 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_5, chart_list[4 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_6, chart_list[5 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_7, chart_list[6 + chart_index], {'x_offset': 0, 'y_offset': 0})
            worksheet.insert_chart(cell_8, chart_list[7 + chart_index], {'x_offset': 0, 'y_offset': 0})
            
            # ITERATE TO THE NEXT SET OF CHARTS!
            chart_index = chart_index + 8

        # CLOSE YOUR WORKBOOK (not in for loop)
        workbook.close()

    def graphInGui(self):
        #print("TEST: Positions to plot")
        #print(self.positions_to_plot)

        #print("SUBSET")
        #print(self.timer_sec[self.positions_to_plot[-2]:self.positions_to_plot[-1]])
        #self.depth_cmd[self.positions_to_plot[0]:self.positions_to_plot[1]]
        #self.depth_ft[self.positions_to_plot[0]:self.positions_to_plot[1]]

        app = QtGui.QApplication(sys.argv)
        w = PrettyWidget(self.positions_to_plot, self.timer_sec, self.depth_ft, self.graph_dict)
        # why does it pop up without exec below?
        app.exec_()
        

# RUN IT
def main():
    test_csv = graphCsv()
    test_csv.getDataFromCsv()
    test_csv.createXlsxFile()

    test_csv.graphInGui()
    
if __name__ == "__main__":
    main()
