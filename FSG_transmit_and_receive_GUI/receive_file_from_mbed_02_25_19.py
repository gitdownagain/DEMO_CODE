from __future__ import print_function

import serial
import time

DEMO = True

class ReceiveFromSerialFSG(object):
    # the crc list is always going to stay the same no matter the instance of the class
    CRCTABLE = [0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448]
    
    def __init__(self, input_port='COM25'):     #get a Serial instance and configure/open it later        
        print("Python: Serial port object created.")
        self._ser = serial.Serial()
        self._ser.port = input_port
        self._ser.baudrate = 115200
        self._ser.timeout = 5

        self.download_progress = 0

        self._write_string = ""
        self._data_packet_list = []

        self._number_of_packets_in_file = 0

        self.finished_processing = False

        # TIMER
        self.t0 = 0
        self.t1 = 0

    def setSerialPort(self, new_serial_port):
        self._ser.port = new_serial_port

    def openserial(self):
        try:
            self._ser.open()
            print("Python: %s open!" % self._ser.port)
            return True

        except serial.SerialException:
            print("Python: ************ Could not connect to %s" % self._ser.port)
            return False

    def close(self):
        try:
            self._ser.close()
            print("Python: Serial port closed?")
        except Exception:
            print("Python: Nothing to close....")
            pass

    def endTransmitRequest(self):
        print("endTransmitRequest: ENDING MBED TRANSMIT")
        #send this stream over the serial port

        end_transmission_string = "%c%c%c%c" % ( chr(16), chr(16), chr(16), chr(16) )
        self._ser.write(end_transmission_string) # 0x010 0x10 0x10 0x10

        print(end_transmission_string)  # TEST

    def crccalc(self,input_string):
        crc=0;
        for x in range(0,len(input_string)):
            crc = (self.CRCTABLE[(ord(input_string[x])^crc) & 0xff]^(crc >> 8))&0xFFFF

            #print("DEBUG: crcrcalc? for loop %d (crc: %d)" % (x,crc))
        return crc

    def calc_crc_1(self,input_string):
        calculated_crc_1 = self.crccalc(input_string) / 256
        return calculated_crc_1

    def calc_crc_2(self,input_string):
        calculated_crc_2 = self.crccalc(input_string) % 256
        return calculated_crc_2

    def sendRequest(self, packet_number):
        ### assemble a packet with CRC ###
        send_byte_three = packet_number / 256
        send_byte_four = packet_number % 256
        
        send_packet_1 = chr(send_byte_three)
        send_packet_2 = chr(send_byte_four)

        crc_1_string = "%c%c%c%c" % (117,101,send_byte_three,send_byte_four)
        crc_2_string = "%c%c%c%c" % (117,101,send_byte_three,send_byte_four)

        calculated_crc_one = self.calc_crc_1(crc_1_string) # integer
        calculated_crc_two = self.calc_crc_2(crc_2_string) # integer

        send_request_string = "%c%c%c%c%c%c" % (117,101,send_packet_1,send_packet_2,chr(calculated_crc_one),chr(calculated_crc_two))
        self._ser.write(send_request_string)              
        print("send_request_string:",send_request_string)
        
        #print("sendRequest: %d %d %d %d (#%d) (Python)" % (75,65,send_byte_three,send_byte_four, packet_number)) #DEBUG        

    def recordLog(self, input_list):
        # NEXT CREATE FILE BASED ON DATE AND TIME
        log_filename = time.strftime("Log_%Y_%m_%d_time_%H_%M.csv", time.localtime())
        
        f = open(log_filename, "w")      #create/overwrite file if it exists

        # use for loop to write each list item to the file
        #awesome, https://stackoverflow.com/questions/899103/writing-a-list-to-a-file-with-python
        for item in input_list:
            f.write("%s" % item)    #write each item as a string line by line

        f.close()   #close the file when you're done

    def receiveData(self, check_number):
        print("Python: receiveData")

    #variables
        read_byte = ""
        read_data_string = ""
        self._write_string = ""

        byte_one = ""
        byte_two = ""
        byte_three = ""
        byte_four = ""
        byte_five = ""
        byte_six = ""
        byte_packet_length = ""
        header_string = ""
        position = 0
        found_117 = False
        found_101 = False


        serial_buffer = self._ser.in_waiting
                
        while (serial_buffer):               # Get the number of bytes in the input buffer
            read_data_string += self._ser.read()
            serial_buffer = serial_buffer - 1 

        string_length = len(read_data_string) #works, verified numerous times

        print("DEBUG - string_length: %d" % string_length)
        print("DEBUG - read_data_string: [%s]\n" % read_data_string)

        #Series of checks to make sure data was received
        # Packet: 75 65 NN NN TT TT LL CC CC  (smallest packet, no data, is size 9)
        # Packet: 75 65 NN NN TT TT LL DD CC CC  (smallest packet, should be over size 10)

        if (string_length < 10):
            return False

        # if the first two bytes are not "u and e"
        if (ord(read_data_string[0]) == 117 and ord(read_data_string[1]) == 101):
            print("found u and e")
        else:
            return False

        print("receiveData DEBUG (size: %d): <<%s>>" % (string_length, read_data_string))
        # debug shows that even on USB the packets sent over the serial port are dropped half the time
        # practically just receiving 1/3 packets

        if (self.finished_processing):
            return True
        else:
            ### PROCESS DATA LIST, sort of like a switch/case statement that does not exist in python ###

            for index, element in enumerate(read_data_string):
                if (found_101):
                    #print("process next three bytes")
                    #process next three bytes#
                    try: 
                        byte_three = read_data_string[index]              #current packet number byte 1
                        byte_four = read_data_string[index+1]             #current packet number byte 2
                        byte_five = read_data_string[index+2]             #total number of packets byte 1
                        byte_six = read_data_string[index+3]              #total number of packets byte 2
                        byte_packet_length = read_data_string[index+4]     #length of data stream                # STOPPED HERE BEFORE
                    except Exception:
                        return False    #2/14 had an error happen just now at 6:05 pm

                    #print("DEBUG (int): %d %d %d %d %d %d %d" % (ord(byte_one),ord(byte_two),ord(byte_three),ord(byte_four),ord(byte_five),ord(byte_six),ord(byte_packet_length)))

                    try:
                        current_packet_num = ord(byte_three)*256 + ord(byte_four)   #this keeps getting blank strings, not sure why
                    except Exception:
                        return False        # something screwed up
                

                    #print("current_packet_num %d" % current_packet_num)

                    if (ord(byte_packet_length) == 0):
                        #print("byte_packet_length")
                        return False
                        # should break here

                    if (ord(byte_packet_length) > string_length):
                        print("ord(byte_packet_length) > string_length")
                        return False
                        # should break here too (what should the actual number be?)

                    header_string = "%c%c%c%c%c%c%c" % (byte_one, byte_two, byte_three, byte_four, byte_five, byte_six, byte_packet_length)
                        
                    break #END FOR LOOP
                
                if (ord(element) == 117):
                    byte_one = element               
                    found_117 = True
                    #print("117") #debug
                    
                if (found_117 and ord(element) == 101):
                    byte_two = element
                    found_101 = True
                    found_117 = False   #toggle this to make sure these don't get entered again
                    #print("101") #debug

            print("HEADER_STRING: %s" % header_string)
                    
        ### OUTSIDE OF FOR LOOP... ###
            data_packet_string = ""

            crc_1_end = read_data_string[-2:-1] #second to last character
            crc_2_end = read_data_string[-1:]   #last character

            ### WRITE DATA PACKET STRING, start from beyond the header ###
            for x in range(7, len(read_data_string)-2):
                data_packet_string += read_data_string[x]

            ### COMBINE HEADER AND DATA PACKET STRING ###
            crc_string = header_string + data_packet_string
            #print("crc_string <%s>" % crc_string)               #DEBUG

    ### CALCULATE CRC ###
            calculated_crc_1 = self.calc_crc_1(crc_string)
            calculated_crc_2 = self.calc_crc_2(crc_string)
            #print("CRC-1: %d, CRC-2: %d" % (calculated_crc_1, calculated_crc_2)) #DEBUG
            print("CALC CRC-1: %d, CRC-1: %d" % (calculated_crc_1, ord(crc_1_end))) #DEBUG
            print("CALC CRC-2: %d, CRC-2: %d" % (calculated_crc_2, ord(crc_2_end))) #DEBUG

            # AFTER CHECKSUM, MAKE SURE YOU HAVE THE CORRECT PACKET #
            #print("Python: byte_three %d" % ord(byte_three))
            #print("Python: byte_four %d" % ord(byte_four))

            try:
                current_packet_number_processed = ord(byte_three) * 256 + ord(byte_four)            # find out how this is getting past the other checks
            except Exception:
                return False    # this screwed up by getting a blank "" string
            
            #print("Python: current_packet_number_processed %d" % current_packet_number_processed)

            #print("Python: number of packets %d" % self._number_of_packets_in_file)

            ### CHECK THE CHECKSUM ###
            if ((ord(crc_1_end) == calculated_crc_1) and (ord(crc_2_end) == calculated_crc_2)):
                print("  DEBUG: Current checksum good!")
                self._write_string = data_packet_string.rstrip() +"\n"  #remove trailing new line character #NO CLUE

                # GET NUMBER OF PACKETS BACK TO USE IN CALLING FUNCTION
                self._number_of_packets_in_file = ord(byte_five) * 256 + ord(byte_six)
                print("Total # of packets: %d" % self._number_of_packets_in_file)
                return True
            
            else:
                print("  DEBUG: Checksum bad?")
                return False

    def getFirstPacket(self):   # this packet gives total number of packets
        while True:
            
            # GET FIRST PACKET #
            self.sendRequest(1)
            time.sleep(0.5) #need a small delay to fill buffer
            if(self.receiveData(1)):
                break #received valid data

        # when loop completes, append to the csv file
        self._data_packet_list.append(self._write_string)
            

    def getData(self):
        x = 1
        valid_data = False  #initial variables
        
        #self.getFirstPacket() # total number of packets
        
        while True:
            self.sendRequest(x) # SEND REQUEST, WAIT FOR REPLY
            time.sleep(0.25)     #best timing so far  
            
            # IF TRUE, WRITE THE STRING
            valid_data = self.receiveData(x)

            print("DEBUG - receiveData ???\n", valid_data)
            
            if (valid_data):
                #APPEND STRING TO PACKET (IF TRUE)

                #print("DEBUG: <%s>" % self._write_string) #DEBUG to see what is being written
                self._data_packet_list.append(self._write_string)
                    
                #NEW, update your progress... (packet / total packets)
                self.download_progress = 100 * (1.0 * x / self._number_of_packets_in_file)

                print("(getData) PROGRESS: %d%% (packet number #%d)" % (self.download_progress, x))

                #IF YOU RECEIVE DATA, RECORD IT TO A LIST, INCREMENT COUNTER
                x = x+1

                # CHECK TO MAKE SURE YOU HAVE RECEIVED ALL OF THE PACKETS! Basically send request higher than number of packets needed            
                if (x > self._number_of_packets_in_file):
                #if (self.finished_processing):
                    print("<><> COMPLETED PROCESSING %d" %x)
                    break

    def getCurrentLog(self):
        self.t0 = time.time()    # get current time in seconds (USED TO TIME HOW LONG THIS TAKES TO COMPLETE)
        
        parse_open_string = ""      #empty string to check this
        
        self.openserial()
        time.sleep(1)

        self._ser.write("c")
        time.sleep(1)
        self._ser.flush()

        print("Python: (getCurrentLog) Sending MBED transmit command ('U')")
        self._ser.write("U")
        time.sleep(3)           #wait a few seconds to try and get data

        ### PROCESS PACKETS (GET DATA) ###
        self.getData()

        ### RECORD DATA TO FILE
        self.recordLog(self._data_packet_list)

        ### END TRANSMISSION WITH HEX 16 16 16 16 command, DEC 10 10 10 10)
        self.endTransmitRequest()
        
        #CLOSE THE SERIAL PORT
        print("Python: CLOSING SERIAL PORT!")
        self._ser.close()

        #on success, tell you how much time this took
        self.t1 = time.time()

        print("getCurrentLog: time to complete was %d seconds" % (self.t1-self.t0))

## CODE TO TEST BELOW ##
if __name__ == '__main__':
    test_serial = ReceiveFromSerialFSG()
    test_serial.setSerialPort('COM8')
    test_serial.getCurrentLog()
