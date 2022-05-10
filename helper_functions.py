# -*- coding: utf-8 -*-

import numpy as np

# helper functions
def calcRotAlignment(roll, pitch, yaw):
    """
    Calculates the amalgamated rotation matrix for each plane and returns
    a single 3D matrix to describe the rotations in each plane. IN DEGREES

    Parameters
    ----------
    roll : float
        Angle of roll
    pitch : float
        Angle of pitch
    yaw : float
        Angle of yaw

    Returns
    -------
    R : np.array (size = 3 x 3)
        3 x 3 rotation matrix representing the rotation of the given in all 3 directions

    """
    
    
    # Save input variables as single character to make matrices clearer
    a = roll
    B = pitch
    y = yaw
    
    Rx = np.array([[1, 0, 0], [0, np.cos(y), -np.sin(y)], [0, np.sin(y), np.cos(y)]])
    Ry = np.array([[np.cos(B), 0, np.sin(B)], [0, 1, 0], [-np.sin(B), 0, np.cos(B)]])
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    
    R = Rx @ Ry @ Rz 

    return R