import time
import sys
from PyQt4 import QtGui, QtCore

from PyQt4.QtCore import QThread

from serial_ports_list_01_31_2018 import serial_ports_list              #code to find active serial ports
from receive_file_from_mbed_02_12_2018 import ReceiveFromSerialFSG      #import serial receive class
from transmit_file_to_mbed_02_08_18 import TransmitToFSG                #import serial transmit class

# THIS MUST RUN FIRST TO GET THE SERIAL PORT LIST
serial_port_checker = serial_ports_list()

serial_receive = ReceiveFromSerialFSG()

#filename = 'C:/Python27/Python_2_7_Troy/sequence.cfg' # KEPT FOR REFERENCE
            
serial_transmit = TransmitToFSG()

class Window(QtGui.QMainWindow):

    def __init__(self):    
        super(Window, self).__init__()  #super(subclass, instance).method(args)
        self.setGeometry(50,50,400,300)
        self.setWindowTitle("DSG transmit/receive GUI!")
        self.setWindowIcon(QtGui.QIcon('red_sun.png')) #icon works

        self.serial_label = QtGui.QLabel("<Program status>", self) #must set the parent widget (self/this class!)
        #self.serial_label.setStyleSheet('background-color: yellow; color: red') #multiple settings
        self.serial_label.resize(400,30)
        self.serial_label.move(25, 125)

        transmit_btn = QtGui.QPushButton("Transmit", self)
        transmit_btn.clicked.connect(self.transmit_button)
        transmit_btn.resize(50,50)
        transmit_btn.move(25,50)

        receive_btn = QtGui.QPushButton("Receive", self)
        receive_btn.clicked.connect(self.receive_button)
        receive_btn.resize(50,50)
        receive_btn.move(100,50)

        #this will exit the current instant of the application
        #btn.clicked.connect(self.close_application)
        quit_btn = QtGui.QPushButton("Quit", self) #button text
        quit_btn.setStyleSheet('background-color: red') # how the heck do you change the color?
        quit_btn.clicked.connect(self.close_application)
        quit_btn.resize(50,25)
        quit_btn.move(300,250)   #same thing, starts from top left at 0,0

        # INITIAL VALUES #
        self._serial_port = 'COM1' #give this a value at initialization
        self._file_name = ""
        # INITIAL VALUES #

        # COMBO BOX for comm/serial ports #
        comboBox = QtGui.QComboBox(self) #reference this class      #combo box is a drop down / droplist / dropbox
        print serial_port_checker.list_of_ports

        #gotta use the length of the list to add these to the combo box drop-down menu
        for x in range(len(serial_port_checker.list_of_ports)):
            comboBox.addItem(serial_port_checker.list_of_ports[x])                 #put items into combo box...
            pass

        comboBox.resize(225, 25)    #resize the combo box
        comboBox.move(25, 250)
        comboBox.activated[str].connect(self.select_serial_port)    #connect it to serial port finder (when active it does this)
        
        
        # COMBO BOX #

        #NEED FOR THINGS FOR EACH ACTION
        extractAction = QtGui.QAction("&Quit program", self)
        extractAction.setShortcut("Ctrl+Q")               #literally set a shortcut
        extractAction.setStatusTip("Leave The App!")
        extractAction.triggered.connect(self.close_application)

        # OPEN A FILE WITH THIS #
        openTransmitFile = QtGui.QAction('&Open Transmit File', self) #text that shows up in the menu
        openTransmitFile.setShortcut("Ctrl+O")                      # KEYBOARD SHORTCUT
        openTransmitFile.setStatusTip("Open File TROY!")            #this text shows up in the bottom status bar
        openTransmitFile.triggered.connect(self.open_transmit_file)           #button connects it to the function fileOpen

        self.statusBar()

        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('&File')        #MENU BAR with item that says "File"
        fileMenu.addAction(openTransmitFile)        #get a file for transmission (I really don't need to "open" it perse, just get file name)
        fileMenu.addAction(extractAction)           #each action you add here

        ## PROGRESS BAR ##
        self.progress_bar_label = QtGui.QLabel("Transmit/Receive Progress", self)
        self.progress_bar_label.setGeometry(200, 80, 250, 20) #fix dimensions
        self.progress_bar_label.move(45, 180)
        self.progress_bar = QtGui.QProgressBar(self)
        self.progress_bar.setGeometry(200, 80, 250, 20)
        self.progress_bar.move(0,200)
        self.progress_bar.setStyleSheet('background-color: red') # how the heck do you change the color?
        ## PROGRESS BAR ##        
        
        self.show() #show all this GUI stuff

    def update_progress_bar(self, new_progress_value):        
        if (new_progress_value >= 100):
            self.progress_bar.setValue(100)  #if over 100, this will just stay at 100
        else:
            self.progress_bar.setValue(new_progress_value)  #this function will take in an integer and update the progress bar

    def open_transmit_file(self):
        name = QtGui.QFileDialog.getOpenFileName(self, 'Open File')       #built in function to create open file window
        file = open(name, 'r')  #open file for reading

        self._file_name = file.name
        print("FILENAME IS %s" % self._file_name)
        self.serial_label.setText(self._file_name)                #PROGRAM LABEL TEXT


    #FUNCTION TAKES THE SERIAL PORT STRING
    def select_serial_port(self, input_serial_port):            #send the combo box string to this
        print(input_serial_port)

        comm_port_string = ""; string_start = False; string_end = False

        comm_port_string = ""
        
        for x in input_serial_port:
            if (x == '('):
                string_start = True
                continue

            if (x == ')'):
                string_end = True
                break #break the for loop

            if (string_start):
                comm_port_string += x

        print comm_port_string
        
        self._serial_port = comm_port_string

    def transmit_button(self):
        self.serial_label.setText("Transmitting data to MBED")

        if (str(self._serial_port) == 'COM1'):
            print("Please choose a serial port.")

        elif(self._file_name == ""):      # blank file name
            print("Please choose a file!.")

            #use warning later
            popup_message_box = QtGui.QMessageBox()
            popup_message_box.setText("Please choose a file!")
            popup_message_box.exec_()
            

        else:
            # CREATE TRANSMITTER THREAD QTHREAD #
            self.mbedTransmitterThread = MbedTransmitter(self._file_name,self._serial_port)
            # RUN IT #
            self.mbedTransmitterThread.start()

            ### PROGRESS BAR ###
            self.transmitterProgressThread = TransmitterProgressBar()
            self.transmitterProgressThread.start()
            self.connect(self.transmitterProgressThread, QtCore.SIGNAL("transmitterProgress(int)"), self.update_progress_bar) #this will update the progress bar using the signal from the receiver thread
    
    def receive_button(self):
        self.serial_label.setText("Receiving data from MBED")

        if (str(self._serial_port) == 'COM1'):
            print("Please choose a serial port.")

        else:
            # RECEIVER PROGRESS BAR THREAD #
            self.receiverThread = ReceiverProgressBar() #thread class
            self.receiverThread.start() # START RECEIVER THREAD (connect after)
            self.connect(self.receiverThread, QtCore.SIGNAL("receiverProgress(int)"), self.update_progress_bar) #this will update the progress bar using the signal from the receiver thread
            # RECEIVER PROGRESS BAR THREAD #
            
            ##### MBED RECEIVER THREAD #####
            self.mbedReceiverThread = MbedReceiver(self._serial_port)
            self.mbedReceiverThread.start()                                                # CONNECT FIRST????
            ##### MBED RECEIVER THREAD #####

    def close_application(self):
        print("SYSTEM EXIT!")
        sys.exit()

        # Exit from Python. This is implemented by raising the SystemExit exception,
        # so cleanup actions specified by finally clauses of try statements are honored, and it is possible to intercept the exit attempt at an outer level.

class TransmitterProgressBar(QThread):                              #class CHILD(FATHER)
    def __init__(self, parent=None):                                #for PyQt you use the parent on initialization
        super(TransmitterProgressBar, self).__init__(parent)        #initialize it      # super(child, self).__init__()

    def run(self):
        self.emit(QtCore.SIGNAL("transmitterProgress(int)"), 0)
        
        while True:
            time.sleep(0.5)
            
            progress_value = serial_transmit.transmit_progress      # TRANSMITTER PROGRESS

            if (progress_value >= 100):
                self.emit(QtCore.SIGNAL("transmitterProgress(int)"), 100) #SIGNAL NAME AND DATA BEING PASSED (in this case an integer value)
                break
            else:
                self.emit(QtCore.SIGNAL("transmitterProgress(int)"), progress_value) #SIGNAL NAME AND DATA BEING PASSED (in this case an integer value)

    def stop(self):
        self.terminate()


class ReceiverProgressBar(QThread):                                 #class CHILD(FATHER)
    def __init__(self, parent=None):                                #for PyQt you use the parent on initialization
        super(ReceiverProgressBar, self).__init__(parent)           #initialize it      # super(child, self).__init__()

    def run(self):
        self.emit(QtCore.SIGNAL("receiverProgress(int)"), 0)
        
        while True:            
            time.sleep(0.5)
            receiver_progress_value = serial_receive.download_progress

            # EMIT a signal to the main thread that updates the progress bar!  Do not update it here or you will crash the program!
            if (receiver_progress_value >= 100):
                self.emit(QtCore.SIGNAL("receiverProgress(int)"), 100) #SIGNAL NAME AND DATA BEING PASSED (in this case an integer value)
                break
            else:
                self.emit(QtCore.SIGNAL("receiverProgress(int)"), receiver_progress_value) #SIGNAL NAME AND DATA BEING PASSED (in this case an integer value)

    def stop(self):
        self.terminate()

class MbedReceiver(QThread):                                #class CHILD(FATHER)
    def __init__(self, GUI_serial_port, parent=None):                        #for PyQt you use the parent on initialization
        super(MbedReceiver, self).__init__(parent)        #initialize it      # super(child, self).__init__()

        # RECEIVE THE SERIAL PORT FROM THE CALLING FUNCTION #
        self._serial_port = str(GUI_serial_port)

        # SET THE SERIAL PORT IN THE CLASS! #
        serial_receive.setSerialPort(self._serial_port)
        
    def run(self):
        # START DATA RECEPTION #
        serial_receive.getCurrentLog()
        

    def stop(self):
        self.terminate()

class MbedTransmitter(QThread):                                #class CHILD(FATHER)
    def __init__(self, GUI_filename, GUI_serial_port, parent=None):                        #for PyQt you use the parent on initialization
        super(MbedTransmitter, self).__init__(parent)        #initialize it      # super(child, self).__init__()

        #SET THE FILE NAME IN THE CLASS! #
        serial_transmit.setTransmitFileName(str(GUI_filename))       #these are classes before converted to string, not sure why
        
        # SET THE SERIAL PORT IN THE CLASS! #
        serial_transmit.setSerialPort(str(GUI_serial_port))         #these are classes before converted to string, not sure why
        
    def run(self):
        # RUN THE THREAD HERE
        serial_transmit.openserial()
        serial_transmit.transmitFile() #transmit file and close serial port

    def stop(self):
        self.terminate()

# CODE OUTSIDE OF THE CLASS #
try:
    app = QtGui.QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())
except (KeyboardInterrupt, SystemExit):
    print("System Exit Exception!")
    app.exit()



# more modifications
# https://srinikom.github.io/pyside-docs/PySide/QtGui/QMessageBox.html
