from __future__ import print_function

import serial
import time

class TransmitToFSG(object):
    # the crc list is always going to stay the same no matter the instance of the class
    CRCTABLE = [0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448]

    _ser = serial.Serial()  #why is this needed for the import?
    
    def __init__(self, input_filename="", input_port='COM25'):     #get a Serial instance and configure/open it later #blank file
        print("Serial port object created.")
        self._ser = serial.Serial()
        self._ser.port = input_port
        self._ser.baudrate = 57600
        self._ser.timeout = 5

        self.filename = input_filename

        self.transmit_progress = 0.0

        self._write_string = ""
        self._data_packet_list = []

        self._number_of_packets_in_file = 1

    def setSerialPort(self, new_serial_port):
        self._ser.port = new_serial_port

    def setTransmitFileName(self, transmit_filename):
        self.filename = transmit_filename

    def openserial(self):
        try:
            self._ser.open()
            #print("%s open!" % self._ser.port)
            return True

        except Exception:
            print("Python: >>> Could not connect to %s <<<" % self._ser.port)
            return False

    def close(self):
        try:
            self._ser.close()
            print("Python: Serial port closed properly!")
        except Exception:
            print("Python: Nothing to close....")
            pass

    def crccalc(self,input_string):
        crc=0;
        for x in range(0,len(input_string)):
            crc = (self.CRCTABLE[(ord(input_string[x])^crc) & 0xff]^(crc >> 8))&0xFFFF
        return crc

    def calc_crc_1(self,input_string):
        calculated_crc_1 = self.crccalc(input_string) / 256
        return calculated_crc_1

    def calc_crc_2(self,input_string):
        calculated_crc_2 = self.crccalc(input_string) % 256
        return calculated_crc_2

    def readDataPacket(self, packet_length):        
        self._write_string = ""                               #EMPTY STRING
       
        for x in range(packet_length):
            current_input = ord(self._ser.read())

            #WRITE THE CHARACTERS TO THE STRING
            self._write_string += chr(current_input)
     
            #DOUBLE CHECK THIS ON-SCREEN
            print(chr(current_input), end='')           #chr converts to ascii (string of one character in ascii)

        return self._write_string


    def receiveData(self, check_number):
        # rewritten from the function in serial_read_class_01_08_2018.py
        # serial port will already be confirmed open
        # CHECK FOR REPLIES HERE! return true/false 01/16/2018
        # no case statements in python...

        #print("receiveData...")

        found_reply_data = False

        received_int_117 = False
        received_int_101 = False

        current_received_packet_number = -1

        #while data is in the serial port buffer

        buffer_size = self._ser.in_waiting
        #print("receiveData serial port buffer %d" % buffer_size)
        
        # READ ALL OF THE DATA #
        serial_buffer = []  # empty list
        
        while buffer_size:
            current_byte = self._ser.read()

            #print(current_byte, end='')

            serial_buffer.append(current_byte)

            buffer_size = buffer_size - 1

        #### PROCESS THE DATA ####
        #print(serial_buffer)        #verified that it comes in as a set of strings for each character sent over serial

        if len(serial_buffer) < 6:       # packet is 6 bytes
            #print("BREAK")
            return False

        # use enumerate to iterate over the list in Python

        header_one = ''
        header_two = ''
        packet_number_first_byte = ''
        packet_number_second_byte = ''
        crc_byte_one = ''
        crc_byte_two = ''
        
        for index, value in enumerate(serial_buffer):
            if (received_int_117 and received_int_101):
                packet_number_first_byte = value
                packet_number_second_byte = serial_buffer[index+1]
                crc_byte_one = serial_buffer[index+2]
                crc_byte_two = serial_buffer[index+3]

                received_int_117 = False
                received_int_101 = False

                #DEBUG THE NUMBERS...
                #print(">>> CHECK THIS: %d %d %d %d" % (ord(packet_number_first_byte),ord(packet_number_second_byte),ord(crc_byte_one),ord(crc_byte_two)))
                                    
                current_received_packet_number = ord(packet_number_first_byte) * 256 + ord(packet_number_second_byte)

                #print("current_received_packet_number %s" % current_received_packet_number)

                #print("FOUND FULL PACKET, crc 1: %s" % crc_byte_one)
                #print("FOUND FULL PACKET, crc 2: %s" % crc_byte_two)
 
                break #BREAK WHILE LOOP
            
            if (ord(value) == 117 and received_int_117 == False):
                header_one = value
                received_int_117 = True
                #print("header_one %s" % value)
                
            if (ord(value) == 101 and received_int_117 == True):
                header_two = value
                received_int_101 = True
                #print("header_two %s" % value)

        #first check if the checksum is correct
        checksum_string = "%c%c%c%c" % (header_one,header_two,packet_number_first_byte,packet_number_second_byte)

        crc_1 = self.calc_crc_1(checksum_string)
        crc_2 = self.calc_crc_2(checksum_string)

        #print(">>> crc %d %d" % (crc_1, crc_2))

        if (crc_1 == ord(crc_byte_one) and crc_2 == ord(crc_byte_two)):
            print("CHECKSUMS GOOD!")
            return True



## UNNECESSARY

##            #check if you're on the correct packet
##            if (check_number == current_received_packet_number):
##                print("***************************************** received request for packet %d %d " % (check_number, current_received_packet_number))
##                return True     #you found the reply data
##            else:
##                print("WTF WTF WTF WTF WTF WTF WTF WTF WTF received request for packet %d %d" % (check_number, current_received_packet_number))
##                return False
        
        else:
            return False    #stop program here

        #### PROCESS THE DATA (end of receiveData function) ####


    # https://stackoverflow.com/questions/16208206/confused-by-python-file-mode-w
    def recordLog(self, input_list):
        # NEXT CREATE FILE BASED ON DATE AND TIME
        log_filename = time.strftime("Log_%Y_%m_%d_time_%H_%M.csv", time.localtime())
        
        f = open(log_filename, "w")      #create/overwrite file if it exists

        # use for loop to write each list item to the file
        #awesome, https://stackoverflow.com/questions/899103/writing-a-list-to-a-file-with-python
        for item in input_list:
            f.write("%s" % item)    #write each item as a string line by line

        f.close()   #close the file when you're done

    def serialExitCommand(self):
        print("serialExitCommand sent")    # command that tells MBED/FSG that transmission is complete
        time.sleep(0.1)                         #was 1 second             
        self._ser.write(chr(16))                #0x10    DLE
        self._ser.write(chr(16))                #0x10    DLE
        self._ser.write(chr(16))                #0x10    DLE
        self._ser.write(chr(16))                #0x10    DLE

    def file_len(self, fname):
        with open(fname) as f:
            for i, l in enumerate(f):
                pass
        return i + 1

    def transmitFile(self):
        #get number of lines in the file first
        packet_number = 0
        number_of_lines_or_packets = self.file_len(self.filename)

        print(" number_of_lines_or_packets %d" % number_of_lines_or_packets)

        #SEND RECEIVE COMMAND to MBED, currently "I"
        self._ser.write("I")
        time.sleep(0.5)

        # MBED immediately starts transmitting #
        #SEND RECEIVE COMMAND to MBED, currently "I"

        # open the file with "with" method, which automatically raises an exception if it exits
        # incorrectly
        # http://preshing.com/20110920/the-python-with-statement-by-example/

        transmit_complete = False
        
        with open(self.filename) as fileobj:
            for line in fileobj:                    #iterate through the file line by line
                
                packet_number = packet_number + 1
                
                length_test_string = len(line)

                while True:
                    #SLOW IT DOWN
                    time.sleep(0.1)
                    
                    #calculate transmit progress number (just for progress bar)
                    self.transmit_progress = 100 * packet_number / (number_of_lines_or_packets * 1.0) #python 2.7 defaults to integer division, force floating point here
                    #print(">>> TRANSMIT PROGRESS: %d" % self.transmit_progress)

                    #create packet
                    transmit_string = "%c%c%c%c%c%s" % (117,101,packet_number,number_of_lines_or_packets,length_test_string,line)

                    #add checksum to packet
                    full_packet = transmit_string + "%c%c" % (self.calc_crc_1(transmit_string), self.calc_crc_2(transmit_string))

                    #print("FULL PACKET: %s" % full_packet) #DEBUG

                    # write full packet to serial port
                    # TRANSMIT ONE TIME! (need time to check before sending same packet again)
                    if not transmit_complete: 
                        self._ser.write(full_packet)
                        print("FULL PACKET: %s" % full_packet) #DEBUG
                        transmit_complete = True

                    #CHECK FOR RECEIPT OF DATA AND BREAK IF REPLY RECEIVED
                    if (self.receiveData(packet_number)):
                        transmit_complete = False
                        break
                    else:
                        pass
                       #print("MBED has not sent the correct reply...") #DEBUG

        print("one second before serial exit command")
        time.sleep(1)
        self.serialExitCommand() #command that tells MBED/FSG that transmission is complete

        #check for data on the serial port before exiting

        while self._ser.in_waiting:
            print(self._ser.read(),end='')

        time.sleep(0.5)
        self.close()    #close serial port

################################# CODE BELOW #################################
################################# CODE BELOW #################################

if __name__ == '__main__':

    filename = 'C:/Python27/Python_2_7_Troy/sequence.cfg'   #change this at some point 1/22/2018
    
    test_serial = TransmitToFSG()
    test_serial.setTransmitFileName(filename)
    test_serial.setSerialPort('COM16')   #com7 laptop, com25 desktop, com 16 desktop xbee for FSG 900 mhz
    #test_serial.setSerialPort('COM25')
    test_serial.openserial()

    test_serial.transmitFile() #transmit file and close serial port
