"""
Code to extrapolate the force at the tip of the instrument, as determined from the sensor measurement. This assumes that the
accelerometer, sensor and tool are all collinear. Therefore, future iterations of the code should look to include the
translation of the sensor and accelerometer.
Based upon: DOI:10.1002/rcs.1737

"""
import numpy as np
import CalibrateSystemGravity as csg
import helper_functions as hf

class SensorMeasurement():

    def __init__(self, current_measured_FT, current_measured_acceleration = [0,0,0], current_accelerometer_aligment_angles = [0,0,0], gravity_compensation = False, EE_consideration = False) -> None:
        """
        Initialises instance of a sensor measurment object.

        Parameters
        ----------
        current_measured_FT : np.array (size = [6 x 1])
            Live measured force and torque measurements in the following form:
                [Fx, Fy, Fz, 𝜏x, 𝜏y, 𝜏z]'.
                
        current_measured_acceleration : np.array (size = [1 x 3])
            Live measured acceleration from the accelerometer. Set this to [0,0,0] if there is no gravity compensation.
        
        current_accelerometer_aligment_angles :  np.array (size = [3 x 1])
            measurements of the roll, pitch and yaw measurements for the accelerometer vs the sensor
            during calibration 
            
        sensor_bias, accelerometer_bias : float
            bias errors found for both sensor and accelerometer by zeroing the sensors

        gravity_compensation : bool
            default value is False. Whether the system should account for gravity compensation

        EE_consideration : bool
            default value is False. Whether the end effector transformation matrix should be considered

        Returns
        -------
        None

        """
        
        # Define the global variables to be used throughout the implementations detailed.
        self.mass = 0 # mass attached to the sensor
        self.body_instrument_wrench = 0 # the wrench due to the interactions between the body and the tool. Assuming this to be zero for now but can adjust this constant.
        self.sensor_forces = np.array(current_measured_FT[0:3]) # the current Fx, Fy, Fz measurement from the sensor
        self.sensor_forces= np.append(self.sensor_forces,1)
        self.sensor_torques = np.array(current_measured_FT[3:]) # the current torques measured from the sensor: 𝜏x, 𝜏y, 𝜏z
        self.sensor_torques= np.append(self.sensor_torques,1)
        self.current_measured_acceleration = current_measured_acceleration # current acceleration from the accelerometer.
        self.current_accelerometer_aligment_angles = current_accelerometer_aligment_angles # alignment between the accelerometer and the sensor frames found via calibration
        self.sensor_bias = np.zeros((1,4)) # bias found in sensor, LabView should allow auto-biasing
        self.accelerometer_bias = 0 # bias found in accelerometer
        self.gravity_compensation = gravity_compensation # whether to apply gravity compensation
        self.EE_consideration = EE_consideration # whether to apply end effector consideration
        self.force_interaction = 0  # reaction force calculated by the sensor and the environment

   
        return None
    

    
    def calcEndEffectorForce(self, K1 = 0):
        """
        Calculates the tooltip interaction force including gravitational compensation.

        Parameters
        ----------
        K1 : np.array
            Calibrated gravitational compensation constant. If gravitational compensation is not desired, set to zero
                
        Returns
        -------
        None

        """
        R1_0 = hf.calcRotAlignment(roll = 90, pitch = 0, yaw = 0) # angles between handle/shaft frame and sensor frame determined from CAD. Rotate +X counterclockwise about Z by 90 (+Y of sensor is +X of our frame)
       
       # transformation matrix tranforming from the tool (sensor) frame to the handle based on CAD 
        T1_0 = np.zeros((4,4)) 
        T1_0[:3, :3] = R1_0
        T1_0[:3, 3] = [0,-0.0124,0] # sensor displaced -12.4mm below shaft
        T1_0[3,3] = 1

        # find the accelerometer transformations
        if self.gravity_compensation == True:
            R_accel = hf.calcRotAlignment(self.current_accelerometer_aligment_angles[0], self.current_accelerometer_aligment_angles[1], self.current_accelerometer_aligment_angles[2]) # rotation matrix tranforming from the accelerometer to the sensor frame
            # convert R_accel to 4x4 homogeneous transformation matrix (i.e. accelerometer reference frame)
            T_accel = np.zeros((4,4))
            T_accel[:3, :3] = R_accel
            T_accel[3,3] = 1

        # if the end effector transformation should be considered (i.e. the length of the shaft). Here T1_0 becomes the reference between the sensor and end effector
        if self.EE_consideration == True:
            T_ee = np.identity(4) # no change in rotation between axes
            T_ee[2,3] = 0.17 # end effector is 0.205m from the sensor. (Z direction)
            T1_0 =  T_ee @ T1_0
            
        

       # if gravity compensation is required 
        if self.gravity_compensation == True:
            self.force_interaction =  T1_0 @ ((self.sensor_forces-self.sensor_bias) - self.mass*K1 @ (T_accel) @ (self.measured_acceleration - self.accelerometer_bias)) - self.body_instrument_wrench
        else:
            self.force_interaction =  T1_0 @ (self.sensor_forces-self.sensor_bias).reshape((4,1))

        current_tooltip_force = self.force_interaction

        return current_tooltip_force


def SensorMeasurementWrapper():
    success = False
    currentSensorMeasurement = SensorMeasurement(current_measured_FT, EE_consideration = True) 
    print(currentSensorMeasurement.calcEndEffectorForce()) # print the force measured, future iterations will include a gui
    success = True
    return success # returns success to labview



if __name__ == "__main__":
    
    """Need to fill in the inputs here"""
     # calibrate the system, this is in the case of an accelerometer
    #calibration = csg.CalibrateSystemGravity(measured_FTs, measured_accelerations, accelerometer_aligment_angles) # these should be arrays of calibration test values
    #current_accelerometer_aligment_angles = calibration.calcSensorAccelAlignment()
    #K1 = calibration.calcGravityCompensationConstant() 
    
    # calculate current sensor measurement
    current_measured_FT = np.array([2,6,3,4,5,6])
    currentSensorMeasurement = SensorMeasurement(current_measured_FT, EE_consideration = True) 
    current_tooltip_force = currentSensorMeasurement.calcEndEffectorForce()
    print(current_tooltip_force)