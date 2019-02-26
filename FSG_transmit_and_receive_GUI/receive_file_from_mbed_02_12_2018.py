from __future__ import print_function

import serial
import time

class ReceiveFromSerialFSG(object):
    # the crc list is always going to stay the same no matter the instance of the class
    CRCTABLE = [0, 49345, 49537, 320, 49921, 960, 640, 49729, 50689, 1728, 1920, 51009, 1280, 50625, 50305,  1088, 52225,  3264,  3456, 52545,  3840, 53185, 52865,  3648,  2560, 51905, 52097,  2880, 51457,  2496,  2176, 51265, 55297,  6336,  6528, 55617,  6912, 56257, 55937,  6720,  7680, 57025, 57217,  8000, 56577,  7616,  7296, 56385,  5120, 54465, 54657,  5440, 55041,  6080,  5760, 54849, 53761,  4800,  4992, 54081,  4352, 53697, 53377,  4160, 61441, 12480, 12672, 61761, 13056, 62401, 62081, 12864, 13824, 63169, 63361, 14144, 62721, 13760, 13440, 62529, 15360, 64705, 64897, 15680, 65281, 16320, 16000, 65089, 64001, 15040, 15232, 64321, 14592, 63937, 63617, 14400, 10240, 59585, 59777, 10560, 60161, 11200, 10880, 59969, 60929, 11968, 12160, 61249, 11520, 60865, 60545, 11328, 58369,  9408,  9600, 58689,  9984, 59329, 59009,  9792,  8704, 58049, 58241,  9024, 57601,  8640,  8320, 57409, 40961, 24768, 24960, 41281, 25344, 41921, 41601, 25152, 26112, 42689, 42881, 26432, 42241, 26048, 25728, 42049, 27648, 44225, 44417, 27968, 44801, 28608, 28288, 44609, 43521, 27328, 27520, 43841, 26880, 43457, 43137, 26688, 30720, 47297, 47489, 31040, 47873, 31680, 31360, 47681, 48641, 32448, 32640, 48961, 32000, 48577, 48257, 31808, 46081, 29888, 30080, 46401, 30464, 47041, 46721, 30272, 29184, 45761, 45953, 29504, 45313, 29120, 28800, 45121, 20480, 37057, 37249, 20800, 37633, 21440, 21120, 37441, 38401, 22208, 22400, 38721, 21760, 38337, 38017, 21568, 39937, 23744, 23936, 40257, 24320, 40897, 40577, 24128, 23040, 39617, 39809, 23360, 39169, 22976, 22656, 38977, 34817, 18624, 18816, 35137, 19200, 35777, 35457, 19008, 19968, 36545, 36737, 20288, 36097, 19904, 19584, 35905, 17408, 33985, 34177, 17728, 34561, 18368, 18048, 34369, 33281, 17088, 17280, 33601, 16640, 33217, 32897, 16448]
    
    def __init__(self, input_port='COM25'):     #get a Serial instance and configure/open it later        
        print("Python: Serial port object created.")
        self._ser = serial.Serial()
        self._ser.port = input_port
        self._ser.baudrate = 57600
        self._ser.timeout = 5

        self.download_progress = 0

        self._write_string = ""
        self._data_packet_list = []

        self._number_of_packets_in_file = 0

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
        send_byte_three = packet_number / 256
        send_byte_four = packet_number % 256
        
        send_packet_1 = chr(send_byte_three)
        send_packet_2 = chr(send_byte_four)
        
        # YOU SHOULD ALREADY BE CONNECTED TO THE COM PORT
        # MUST USE CHR
        #print("send request 0")
                  
        self._ser.write(chr(117))              # 0x75 
        self._ser.write(chr(101))              # 0x65            
        self._ser.write(send_packet_1)
        self._ser.write(send_packet_2)

# ADD THE CHECKSUM! 2/9/2018

        print("sendRequest: %d %d %d %d (#%d) (the packet that was sent from Python)" % (117,101,send_byte_three,send_byte_four, packet_number)) #DEBUG
        time.sleep(0.1)

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
        #variables
        read_string = ""
        read_byte = ""
        read_data_byte_list = []

        serial_buffer = self._ser.in_waiting
        
        while (serial_buffer):               # Get the number of bytes in the input buffer
            read_byte = self._ser.read()
            read_data_byte_list.append(read_byte)   # append the string bytes to a list
            #read_string += str(read_byte)          # debugging

            time.sleep(0.001) #this while loop needs a small delay to process data correctly!    # THIS WORKED WELL WITH previous packet
            serial_buffer = serial_buffer - 1

        list_length = len(read_data_byte_list) #works, verified numerous times

        #print("LIST_LENGTH %d" % list_length)

        if (list_length <= 9):
            #print(" ************************ list_length was less than nine ************************")
            return False

        ### PROCESS DATA LIST ###
        ### SORTA SWITCH/CASE LIKE ### (does not exist in Python)

        found_117 = False
        found_101 = False

        byte_one = ""
        byte_two = ""
        byte_three = ""
        byte_four = ""
        byte_five = ""
        byte_six = ""
        byte_packet_length = ""

        position = 0
        for x in read_data_byte_list:            
            if (found_101):
                #process next three bytes#
                byte_three = read_data_byte_list[position]              #current packet number byte 1
                byte_four = read_data_byte_list[position+1]             #current packet number byte 2
                byte_five = read_data_byte_list[position+2]             #total number of packets byte 1
                byte_six = read_data_byte_list[position+3]              #total number of packets byte 2
                byte_packet_length = read_data_byte_list[position+4]     #length of data stream                # STOPPED HERE BEFORE

                #print("DEBUG (int): %d %d %d %d %d %d %d" % (ord(byte_one),ord(byte_two),ord(byte_three),ord(byte_four),ord(byte_five),ord(byte_six),ord(byte_packet_length)))

                current_packet_num = ord(byte_three)*256 + ord(byte_four)

                if (ord(byte_packet_length) == 0):
                    return False
                    # should break here

                if (ord(byte_packet_length) > list_length):
                    return False
                    # should break here too (what should the actual number be?)

                header_string = "%c%c%c%c%c%c%c" % (byte_one, byte_two, byte_three, byte_four, byte_five, byte_six, byte_packet_length)
                #print("HEADER STRING: %s (length: %d)" % (header_string, len(header_string)))
# HERE

                break #END FOR LOOP
            
            if (ord(x) == 117):
                byte_one = x               
                found_117 = True
                
            if (found_117 and ord(x) == 101):
                byte_two = x
                found_101 = True
                found_117 = False   #toggle this to make sure these don't get entered again

            position += 1

        # OUTSIDE OF FOR LOOP...
        position = position + 5    # move this position tracker five over (to account for the five more bytes you read after the header 75 65)

        data_packet_string = ""

        #print("ord(byte_five) %d"% ord(byte_five))

        for y in range(ord(byte_packet_length)):
            data_packet_string += read_data_byte_list[position]
            position = position+1
            #print("position %d" % position)
        #print(">>>>> (DEBUG) data packet string: <<%s>>" % data_packet_string)

        # READ CRC CHARACTERS
        crc_input_one = read_data_byte_list[position]
        #print("crc one %s (%d)" % (crc_input_one, ord(crc_input_one)))
        crc_input_two = read_data_byte_list[position+1]
        #print("crc two %s (%d)" % (crc_input_two, ord(crc_input_two)))

        crc_string = header_string + data_packet_string
        #print("crc_string %s" % crc_string)

        calculated_crc_1 = self.calc_crc_1(crc_string)
        calculated_crc_2 = self.calc_crc_2(crc_string)

        #print("calc and real one %d %d" % (calculated_crc_1, ord(crc_input_one)))
        #print("calc and real two %d %d" % (calculated_crc_2, ord(crc_input_two)))

        # CLEAR YOUR DATA PACKET STRING
        self._write_string = ""

        # CHECK THE CHECKSUM #
        if ((ord(crc_input_one) == calculated_crc_1) and (ord(crc_input_two) == calculated_crc_2)):
            #print("  DEBUG: Current checksum good!")
            self._write_string = data_packet_string.rstrip() +"\n"  #remove trailing new line character #NO CLUE
        else:
            #print("  DEBUG: Checksum bad?")
            return False

        # AFTER CHECKSUM, MAKE SURE YOU HAVE THE CORRECT PACKET #
        current_packet_number_processed = current_packet_number = ord(byte_three) * 256 + ord(byte_four)
        print("Python: current_packet_number_processed %d" % current_packet_number_processed)
        
        if (check_number != current_packet_number_processed):
            print("************* check_number != current_packet_number_processed (NO DATA IS SENT, FALSE CONDITION")
            return False
        else:
            return True


    def getData(self):
        x = 1

        serial_buffer = 0
        counter = 0
        
        while True:
            self.sendRequest(x)
            time.sleep(0.4) #worked at one second (did not work at 0.2, again did not work at 0.2)

##            serial_buffer = self._ser.in_waiting
##
##            print("serial_buffer %d" % serial_buffer)
##
##            while serial_buffer:                  # DEBUG
##                counter = counter + 1
##                print(self._ser.read(), end='')
##                serial_buffer = serial_buffer - 1 #decrement
##
##            print("counter is %d" % counter)
##            counter = 0

            
            if (self.receiveData(x)):                 
                #APPEND STRING TO PACKET (IF TRUE)

                #print("DEBUG: <%s>" % self._write_string) #DEBUG to see what is being written
                self._data_packet_list.append(self._write_string)
                    
                #NEW, update your progress... (packet / total packets)
                self.download_progress = 100 * (1.0 * x / self._number_of_packets_in_file)

                print("PROGRESS: %d%% (packet number #%d)" % (self.download_progress, x))

                #IF YOU RECEIVE DATA, RECORD IT TO A LIST, INCREMENT COUNTER
                x = x+1

            if (x > self._number_of_packets_in_file):
                print("<><> COMPLETED PROCESSING %d" %x)
                break
                pass

    def getCurrentLog(self):
        parse_open_file = ""
        
        self.openserial()
        time.sleep(1)

        print("Python: (getCurrentLog) Sending MBED transmit command ('O')")
        self._ser.write("O")
        time.sleep(1)

        serial_buffer = self._ser.in_waiting
        print("Python: what is left in buffer? %d" % serial_buffer)

        #FIX THIS IDEA.  Right now you read the first reply, which gives you number of lines in the file (number of packets)
        while (serial_buffer):
            read_byte = self._ser.read()
            parse_open_file += read_byte        
            print(read_byte,end='')
            serial_buffer = serial_buffer - 1       # HAVE TO DECREMENT THIS...

        # PARSE STRING
        position_to_start = parse_open_file.find("are")
        position_to_end = parse_open_file.find("lines")
            
        string_number_of_packets = ""
        for x in parse_open_file[position_to_start:position_to_end]:
            try:
                int(x)
                string_number_of_packets += x #if it's a string number, put it into this string
            except Exception:
                pass

        #print("position_to_start: %d, position_to_end %d" % (position_to_start, position_to_end)) # debug

        print("Python: NUMBER OF PACKETS <<%d>>" % int(string_number_of_packets))

        # set the number of packets based on this string
        self._number_of_packets_in_file = int(string_number_of_packets)

        if (self._number_of_packets_in_file > 0):
            ### PROCESS PACKETS (GET DATA) ###
            self.getData()

            ### RECORD DATA TO FILE
            self.recordLog(self._data_packet_list)
                    
            #CLOSE THE SERIAL PORT
            print("Python: CLOSING SERIAL PORT!")
            self._ser.close()
        else:
            print("(getCurrentLog) No data processed.  Number of packets was zero.")
            self._ser.close()









if __name__ == '__main__':
    test_serial = ReceiveFromSerialFSG()

    test_serial.setSerialPort('COM16')

    test_serial.getCurrentLog()

##    try:
##        test_serial.getCurrentLog()
##    except serial.SerialException:
##        print("EXCEPTION: Could not open the serial port!")
##
