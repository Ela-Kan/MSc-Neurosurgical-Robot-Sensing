# -*- coding: utf-8 -*-

import numpy as np
import helper_functions as hf

class CalibrateSystemGravity():
    
    def __init__(self, measured_FTs, measured_accelerations, accelerometer_aligment_angles) -> None:
        """
        Used for calibrating the system.

        Parameters
        ----------
        measured_FTs : list of lists (each element contains list of size = [6 x 1])
           List of calibration measured force and torque measurements in the following form:
                [Fx, Fy, Fz, 𝜏x, 𝜏y, 𝜏z]'.
                
        measured_acceleration : list of lists (each element contains list of size = [1 x 3])
           List of calibration calculated accelerations from the accelerometer
        
        accelerometer_aligment_angles : list of lists
            n measurements of the roll, pitch and yaw measurements for the accelerometer vs the sensor
            during calibration [n x [roll, pitch, yaw]]
            
        Returns
        -------
        None

        """
        
        # Define the global variables to be used throughout the implementations detailed.
        self.measured_FTs = measured_FTs
        self.measured_accelerations = measured_accelerations
        self.accelerometer_aligment_angles = accelerometer_aligment_angles
        self.force_interation = 0
   
        return None

    def calcSensorAccelAlignment(self):
        """
        Uses a least squares and binary search algorithm to find the three rotation alignment angles between
        the sensor and accelerometer frames.

            
        Returns
        -------
        current_accelerometer_aligment_angles = the alignment between the accelerometer and sensor frame
        """
        current_accelerometer_aligment_angles = [0,0,0]

        return current_accelerometer_aligment_angles
    
    def calcGravityCompensationConstant(self):
        p = len(self.measured_FTs) # the number of calibration measurements taken
        sum_of_forces = 0
        sum_of_accels = 0
        for i in p: # for each calibration measurement
            current_force = np.absolute(self.measured_FTs[i][0:3])
            sum_of_forces += current_force/p
            current_acceleration = self.measured_accelerations[i][0:3]
            current_alignment = hf.calcRotAlignment(self.accelerometer_aligment_angles[i][:])
            sum_of_accels += np.absolute((current_alignment @ current_acceleration) / p)
            
        K1 = sum_of_forces/sum_of_accels
        
        return K1
    
    