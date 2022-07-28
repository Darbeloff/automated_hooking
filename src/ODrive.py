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

import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb
import matplotlib.pyplot as plt
import numpy as np


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
        #Connects to odrives of specified serial ids
        # TODO: parallelize
        drives = [None]*len(serials)
        axis = [None]*2*len(serials)
        
        def _connect(i):
            def _thread(): 
                serial = serials[i]
                print("Finding odrive: " + serial + "...")

                drive = odrive.find_any(serial_number = serial)
                drive[i] = drive
                print("Found odrive!")

                axis[2*i] = drive.axis0
                axis[2*i + 1] = drive.axis1

            return _thread

        threads = [Thread(target=_connect(i)) for i in range(len(serials))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        
        return drives,axis

    #--------------------------- INIT FUNCTIONS -----------------------------
    def startup_init(self):
        print('Initializing encoder calibration sequence')
        for i in self.axis:
            i.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            i.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            i.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            i.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.initflag=1

    def full_init(self,reset = True):
        if(reset):
            for i in range(0,2):
                print(i)
                self.odrv.config.brake_resistance = 0.5
                self.axis[i].motor.config.pre_calibrated = False

                #pole pairs
                self.axis[i].motor.config.pole_pairs = 4
                self.axis[i].controller.config.vel_limit = 200000 

                self.axis[i].motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
                self.axis[i].encoder.config.cpr = 4000
                self.axis[i].encoder.config.use_index = True
                self.axis[i].encoder.config.zero_count_on_find_idx = True
                self.axis[i].encoder.config.pre_calibrated = False

                #motor calibration current
                self.axis[i].motor.config.calibration_current = 4
                self.axis[i].motor.config.resistance_calib_max_voltage = 12

                time.sleep(1)
                self.axis[i].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(10)
        for i in range(0,2):
            time.sleep(1)
            self.printErrorStates()
            self.axis[i].requested_state = AXIS_STATE_IDLE
            self.axis[i].motor.config.pre_calibrated=True

            self.axis[i].config.startup_encoder_index_search = True
            self.axis[i].config.startup_encoder_offset_calibration = True
            self.axis[i].controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
            self.printErrorStates()
            kP_des = 2
            kD_des = 0.0002
            self.axis[i].controller.config.pos_gain = kP_des 
            self.axis[i].controller.config.vel_gain = kD_des
            self.axis[i].controller.config.vel_integrator_gain = 0.001
            self.axis[i].controller.pos_setpoint = 0
            time.sleep(1)

            self.odrv.save_configuration()
            print('Calibration completed')
            self.printErrorStates()

    def make_perm(self):
        self.odrv.save_configuration()

    def set_gains(self,axis_num,kpp = 0.3,kvp = 0.0002,kvi = 0.0001):
        self.axis[axis_num].requested_state=AXIS_STATE_IDLE
        self.axis[axis_num].controller.config.pos_gain = kpp
        self.axis[axis_num].controller.config.vel_gain = kvp
        self.axis[axis_num].controller.config.vel_integrator_gain = kvi
        time.sleep(1)

    def set_closed_loop_state(self,axis_num):
        self.axis[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

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
    def trajMoveCnt(self, axis_num, posDesired = 10000, velDesired = 25000, accDesired = 50000):
        #Move to a position with a specified trajectory
        
        self.axis[axis_num].trap_traj.config.vel_limit = velDesired 
        self.axis[axis_num].trap_traj.config.accel_limit = accDesired 
        self.axis[axis_num].trap_traj.config.decel_limit = accDesired
        self.axis[axis_num].controller.move_to_pos(posDesired)

    def PosMove(self,pos_setpt, axis_num):
        self.axis[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis[axis_num].controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
        self.axis[axis_num].controller.pos_setpoint=pos_setpt

    def VelMove(self,vel_setpt, axis_num):
        #100000 = quarter rev per second
        self.axis[axis_num].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis[axis_num].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
        self.axis[axis_num].controller.vel_setpoint = vel_setpt
    
    #--------------------------- SENSOR FUNCTIONS --------------------------
    def get_encoder_count(self,num):
        return self.axis[num].encoder

    def get_current(self,axis_num):
        return self.axis[axis_num].motor.current_control.Iq_measured


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