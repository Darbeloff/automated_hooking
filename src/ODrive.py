#!/usr/bin/python3
"""
Interface from Python to ODrive
Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019

Edited by Rachel Hoffman-Bice and Jerry Ng, January 2020
Edited by Cormac O'Neill, August 2021
Edited by Quinn Bowers, July 2022
"""

import odrive
from odrive.enums import *
from threading import Thread
import numpy as np

import time



in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604 

#208637853548
#2061377C3548

#In this python script, there is skeleton code for how you may use the class at some point.
#There will be additional updates to this driver to make it easier to use for you in the future.

class ODrive:
    MAX_VEL = 100000
    CPR2RAD = (2*np.pi/400000)

    def __init__(self,*inputs):

        self.odrv, self.axis = self.connect_all(inputs)
        self.printErrorStates()

        # print("Setting gains to default")
        self.set_gains(axis_num = 0)
        self.set_gains(axis_num = 1)

    def connect_all(self, serials):
        # Connects to odrives of specified serial ids
        drives = [None]*len(serials)
        
        def _connect(i):
            def _thread(): 
                serial = serials[i]
                print("Finding odrive: " + serial + "...")

                drive = odrive.find_any(serial_number = serial)
                drives[i] = drive
                print("Found odrive!")

            return _thread

        # start threads
        threads = [Thread(target=_connect(i)) for i in range(len(serials))]
        for thread in threads:
            thread.start()
        # wait for all to complete
        for thread in threads:
            thread.join()

        axis = np.ravel([[d.axis0, d.axis1] for d in drives])
        
        return drives,axis

    #--------------------------- INIT FUNCTIONS -----------------------------
    def startup_init(self):
        print('Initializing encoder calibration sequence')
        for axis in self.axis:
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.initflag=1

    def full_init(self,reset = True):
        for drive in self.drives:
            drive.config.brake_resistance = 0.5
            drive.save_configuration()
        
        self.set_gains(range(len(self.axis)))

        for axis in self.axis:
            if reset:
                axis.motor.config.pre_calibrated = False
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(10)

                axis.motor.config.pole_pairs = 4
                axis.controller.config.vel_limit = 200000 

                axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
                axis.encoder.config.cpr = 4000
                axis.encoder.config.use_index = True
                axis.encoder.config.zero_count_on_find_idx = True
                axis.encoder.config.pre_calibrated = False

                #motor calibration current
                axis.motor.config.calibration_current = 4
                axis.motor.config.resistance_calib_max_voltage = 12

            axis.motor.config.pre_calibrated=True
            axis.config.startup_encoder_index_search = True
            axis.config.startup_encoder_offset_calibration = True

        print('Calibration completed')
        self.printErrorStates()
    

    def set_gains(self,axis_num,kpp = 10.0,kvp = 0.000005,kvi = 0.0001):
        self.axis[axis_num].requested_state=AXIS_STATE_IDLE
        self.axis[axis_num].controller.config.pos_gain = kpp
        self.axis[axis_num].controller.config.vel_gain = kvp
        self.axis[axis_num].controller.config.vel_integrator_gain = kvi
        time.sleep(1)

    def reboot(self):
        #Reboot and reconnect function
        self.odrv.reboot()
        time.sleep(5)
        connect_all()
        print('Rebooted ')

    def erase_and_reboot(self):
        #Erase the configuration of the system and reboots
        print('erasing config')
        self.odrv.erase_configuration()
        print('reboot')
        self.odrv.reboot()


    #--------------------------- CONTROL FUNCTIONS --------------------------

    # update or remove
    def trajMoveCnt(self, axis_num, posDesired = 10000, velDesired = 25000, accDesired = 50000):
        #Move to a position with a specified trajectory
        
        self.axis[axis_num].trap_traj.config.vel_limit = velDesired 
        self.axis[axis_num].trap_traj.config.accel_limit = accDesired 
        self.axis[axis_num].trap_traj.config.decel_limit = accDesired
        self.axis[axis_num].controller.move_to_pos(posDesired)
        

    def set_position(self, ids, positions):
        for id,position in zip(ids, positions):
            self.axis[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis[id].controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
            self.axis[id].controller.pos_setpoint=position

    def set_velocity(self, ids, velocities):
        for id,velocity in zip(ids, velocities):
            self.axis[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis[id].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
            self.axis[id].controller.vel_setpoint = velocity
        
    def set_effort(self, ids, efforts):
        for id,effort in zip(ids, effort):
            self.axis[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis[id].controller.config.control_mode=CTRL_MODE_TORQUE_CONTROL
            self.axis[id].controller.torque_setpoint = effort
    
    #--------------------------- SENSOR FUNCTIONS --------------------------
    def get_encoder_count(self,num): # this should be removes
        return self.axis[num].encoder

    def get_position(self):
        return np.array([axis.encoder.pos_estimate for axis in self.axis])
    def get_velocity(self):
        return np.array([axis.encoder.vel_estimate for axis in self.axis])
    def get_effort(self):
        # return [axis.motor.current_control.Id_measured for axis in self.axis]
        return np.array([axis.motor.current_control.Id_setpoint for axis in self.axis]) * Nm2A


    #-------------------- ERROR CHECKING PRINT FUNCTIONS --------------------
    def print_controllers(self):
        for i in self.axis:
            print(i.controller)

    def print_encoders(self):
        for i in self.axis:
            print(i.encoder)

    def printErrorStates(self):
        for i in self.axis:
            print(' axis error:',hex(i.error))
            print(' motor error:',hex(i.motor.error))
            print(' encoder error:',hex(i.encoder.error))

    def printPos(self):
        for i in self.axis:
            print(' pos_estimate: ', i.encoder.pos_estimate)
            print(' count_in_cpr: ', i.encoder.count_in_cpr)
            print(' shadow_count: ', i.encoder.shadow_count)


    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()