#!/usr/bin/env python2

import numpy as np
import serial
import time

class HookBridge:
    """
    This class interfaces with the hook-mounted arduino over serial, receiving updates about the orientation and acceleration of the board. This class can also command the nano to open and close the retainer
    """

    # ARDUINO_PORT = 'ttyUSB4'
    ARDUINO_PORT = 'COM6'
    BAUDRATE = 115200

    state = {
        'acceleration': [],
        'gyro': [],
        'orientation': []
    }

    def __init__(self, debug=False):
        self.debug = debug
        
        self.port = serial.Serial(self.ARDUINO_PORT, self.BAUDRATE)

        self.set_retainer(True)
        while True:
            self.receive_serial()
            time.sleep(0.005)

            print(self.state)
    
    def set_retainer(self, open):
        """ Write to the serial port with characters indicating whether to open or close
        """
        # print("command:" + str(open))
        if open:
            self.port.write(b'o')
        else:
            self.port.write(b'c')

    def receive_serial(self):
        in_line = self.port.readline().decode('utf-8')

        if in_line == None:
            print("None")
            return

        for header in self.state.keys():
            if header in in_line:
                in_line = in_line.replace(header, '') # remove header string
                self.state[header] = in_line[3:-3].split(',') # clean brackets and split on comma 
                return

    

    def __del__(self):
        """ Destructor ensures serial port is properly closed
        """
        self.port.close()


if __name__ == '__main__':
    HookBridge(debug=True)

    while(True):
        pass