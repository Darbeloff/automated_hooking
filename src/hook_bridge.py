#!/usr/bin/env python2

import numpy as np
import serial
import time

class HookBridge:
    """
    This class interfaces with the hook-mounted arduino over serial, recieving updates about the orientation and acceleration of the board. This class can also command the nano to open and close the retainer
    """

    # ARDUINO_PORT = 'ttyUSB4'
    ARDUINO_PORT = 'COM6'
    BAUDRATE = 115200

    state = {
        '\{acceleration\}': [],
        '\{gryo\}': [],
        '\{orientation\}': []
    }

    def __init__(self, debug=False):
        self.debug = debug
        
        self.port = serial.Serial(ARDUINO_PORT, BAUDRATE)

        while True:
            self.receive_serial()
            time.sleep(100)
    
    def set_retainer(self, open):
        """ Write to the serial port with characters indicating whether to open or close
        """
        print("command:" + open)
        if open:
            self.port.write(b'o')
        else:
            self.port.write(b'c')

    def receive_serial(self, open):
        in_line = self.port.readline()
        
        print(in_line)

        if in_line == None: return

        for header in self.state.keys():
            print(header)
            if in_line.contains(header):
                in_line = in_line.replace(header, '') # remove header string
                self.state[header] = in_line[1,-1].split(',') # clean brackets and split on comma 
                return

        


        

    def on_close(self):
        self.port.close()


if __name__ == '__main__':
    HookBridge(debug=True)

    while(True):
        pass