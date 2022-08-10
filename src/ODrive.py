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


class ODrive:
    MAX_VEL = 100000
    CPR2RAD = (2*np.pi/400000)

    def __init__(self,*inputs):
        self.drives, self.axes = self.connect_all(inputs)
        
        # self.printErrorStates()
        # print("Setting gains to default")
        self.set_gains()

    # def __del__(self):
    #     for axis in self.axes:
    #         axis.requested_state = AXIS_STATE_IDLE

    def connect_all(self, serials):
        """
        Connects to odrives of specified serial ids. Returns both the drives and the attached axes
        """
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

        axes = np.ravel([[d.axis0, d.axis1] for d in drives])
        # remove axes that are not attached to motors
        # axes = [axis for axis in axes if not axis.motor.error]
        
        
        return drives,axes

    #--------------------------- INIT FUNCTIONS -----------------------------
    def startup_init(self):
        """
        Do calibration
        NOTE: untested 7/29/22
        """
        print('Initializing encoder calibration sequence')
        def _calibrate(axis):
            def _thread():
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(1)
                axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
                time.sleep(10)
                axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
                time.sleep(10)
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(1)

            return _thread

        # for axis in self.axes:
        #     axis.requested_state = AXIS_STATE_IDLE
        #     time.sleep(1)
        #     axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
        #     time.sleep(10)
        #     axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        #     time.sleep(10)
        #     axis.requested_state = AXIS_STATE_IDLE
        #     time.sleep(1)
        threads = [Thread(target=_calibrate(axis)) for axis in self.axes]
        for thread in threads:
            thread.start()
        
        # wait for all to complete
        for thread in threads:
            thread.join()
        
        self.initflag=1

    def full_init(self, reset=True):
        """
        Do calibration, set various control constants and save the configuration
        NOTE: untested 7/29/22
        """
        for drive in self.drives:
            drive.config.brake_resistance = 0.5
            drive.save_configuration()
        
        self.set_gains()

        def _calibrate(axis):
            def _thread():
                # if reset:
                axis.motor.config.pre_calibrated = False

                axis.motor.config.pole_pairs = 4
                axis.controller.config.vel_limit = 200000 

                axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
                axis.encoder.config.cpr = 4000
                axis.encoder.config.use_index = True
                axis.encoder.config.zero_count_on_find_idx = True
                
                
                #motor calibration current
                axis.motor.config.calibration_current = 4
                axis.motor.config.resistance_calib_max_voltage = 12


                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(5)
                
                # axis.motor.config.pre_calibrated=True
                # axis.config.startup_encoder_index_search = True
                # axis.config.startup_encoder_offset_calibration = True

            return _thread

        threads = [Thread(target=_calibrate(axis)) for axis in self.axes]
        for thread in threads:
            thread.start()
            time.sleep(1)
        
        # wait for all to complete
        for thread in threads:
            thread.join()
        
        print('Calibration completed')
    
    
    def set_gains(self,kpp = 10.0,kvp = 0.000005,kvi = 0.0001):
        """
        Set the odrive control constants
        """
        for axis in self.axes:
            axis.requested_state=AXIS_STATE_IDLE
            axis.controller.config.pos_gain = kpp
            axis.controller.config.vel_gain = kvp
            axis.controller.config.vel_integrator_gain = kvi
        time.sleep(1)

    def reboot(self):
        """
        Reboot and reconnect function
        NOTE: untested 7/29/22
        """
        for drive in self.drives:
            drive.reboot()
        time.sleep(5)
        self.connect_all()
        print('Rebooted ')

    def erase_and_reboot(self):
        """
        Erase the configuration of the system and reboots
        NOTE: untested 7/29/22
        """
        print('erasing config')
        for drive in self.drives:
            drive.erase_configuration()
        self.reboot()


    #--------------------------- CONTROL FUNCTIONS --------------------------

    # update or remove
    def trajMoveCnt(self, axis_num, posDesired = 10000, velDesired = 25000, accDesired = 50000):
        """
        Move to a position with a specified trajectory
        NOTE: untested 7/29/22
        """
        
        self.axes[axis_num].trap_traj.config.vel_limit = velDesired 
        self.axes[axis_num].trap_traj.config.accel_limit = accDesired 
        self.axes[axis_num].trap_traj.config.decel_limit = accDesired
        self.axes[axis_num].controller.move_to_pos(posDesired)
        
    def set_position_all(self, position):
        self.set_position(range(len(self.axes)), [position]*len(self.axes))
    def set_position(self, ids, positions):
        for id,position in zip(ids, positions):
            self.axes[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axes[id].controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
            # odrv0.axis0.controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
            self.axes[id].controller.pos_setpoint=position

    def set_velocity_all(self, velocity):
        self.set_velocity(range(len(self.axes)), [velocity]*len(self.axes))
    def set_velocity(self, ids, velocities):
        for id,velocity in zip(ids, velocities):
            self.axes[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axes[id].controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
            self.axes[id].controller.vel_setpoint = velocity
        
    def set_effort_all(self, effort):
        self.set_effort(range(len(self.axes)), [effort]*len(self.axes))
    def set_effort(self, ids, efforts):
        for id,effort in zip(ids, efforts):
            self.axes[id].requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axes[id].controller.config.control_mode=CTRL_MODE_CURRENT_CONTROL
            self.axes[id].controller.current_setpoint = effort
    
    #--------------------------- SENSOR FUNCTIONS --------------------------
    def get_encoder_count(self,num): # this should be removed
        return self.axes[num].encoder

    def get_position(self):
        return np.array([axis.encoder.pos_estimate for axis in self.axes])
    def get_velocity(self):
        return np.array([axis.encoder.vel_estimate for axis in self.axes])
    def get_effort(self):
        return [axis.motor.current_control.Id_measured for axis in self.axes]
        # return np.array([axis.motor.current_control.Iq_setpoint for axis in self.axes]) * Nm2A


    #-------------------- ERROR CHECKING PRINT FUNCTIONS --------------------
    def print_controllers(self):
        for i in self.axes:
            print(i.controller)

    def print_encoders(self):
        for i in self.axes:
            print(i.encoder)

    def printErrorStates(self):
        for i in self.axes:
            print(' axis error:',hex(i.error))
            print(' motor error:',hex(i.motor.error))
            print(' encoder error:',hex(i.encoder.error))

    def printPos(self):
        for i in self.axes:
            print(' pos_estimate: ', i.encoder.pos_estimate)
            print(' count_in_cpr: ', i.encoder.count_in_cpr)
            print(' shadow_count: ', i.encoder.shadow_count)


    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()