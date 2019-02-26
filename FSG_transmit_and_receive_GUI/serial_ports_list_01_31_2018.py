##from serial.tools import list_ports
##what = list_ports.comports()
##
##print(what)
##
##my_port_name = list(list_ports.grep("0483:5740"))[0][0]
##
##print(my_port_name)

import serial.tools.list_ports

class serial_ports_list(object):
    # produce a list of all serial ports. The list contains a tuple with the port number, 
    # description and hardware address

    def __init__(self):
        self.ports = list(serial.tools.list_ports.comports()) #the list of ports with a load of data

        self.list_of_ports = []
        self.list_of_comms = []

        self.getPortInfo()

    def getPortInfo(self):
        for port_no, description, address in self.ports:
            ascii_description = description.encode('ascii', 'ignore')
            
            self.list_of_ports.append(ascii_description)   #encode as ASCII not unicode

            #read the parenthesis
            self.list_of_comms.append(self.read_between_parenthesis(ascii_description))

    def read_between_parenthesis(self, input_string):
        string_start = False
        string_end = False

        comm_port_string = ""
    
        for x in input_string:
            if (x == '('):
                string_start = True
                continue

            if (x == ')'):
                string_end = True
                break #break the for loop

            if (string_start):
                comm_port_string += x

        return comm_port_string
      
## CODE BELOW ##
## CODE BELOW ##

if __name__ == '__main__':
    check_serial_ports = serial_ports_list()

    print(check_serial_ports.list_of_ports)
    print(check_serial_ports.list_of_comms)    
    
#https://stackoverflow.com/questions/32004317/how-to-automatically-get-port-no-of-a-hardware-in-python-serial-communcation
