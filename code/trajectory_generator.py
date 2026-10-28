import os
import sys
import time

import numpy as np
from scipy.interpolate import CubicSpline

import constants
from pal.products.qarm import QArm

directory_path = os.path.dirname(constants.path_to_interface)
if directory_path not in sys.path:
    sys.path.append(directory_path)

from QArm_functions import QArm_Lab_interface # type: ignore


x_dist = 0.4
trajectory_points = [
    # [X, Y, Z]
    [x_dist, 0, 0.5], # Start
    [x_dist, -0.4, 0.3], 
    [x_dist, 0, 0.1], # opposite side
    [x_dist, 0.4, 0.3], 
    [x_dist, 0, 0.5] # Start (needed for periodicity)
]

t = np.array([0, 4, 8, 12, 16]) # Time stamps for each knot point

QArm_Interface = QArm_Lab_interface()

# Find starting joint positions
start_phi = QArm_Interface.inverse_kinematics(trajectory_points[0], 0, np.array([0, 0, 0, 0]))[1]

mode = "-1"
while(mode != '0' and mode != '1'):
    mode = input("Enter 1 for real hardware, 0 for simulation: ")

with QArm(hardware=int(mode), readMode=0) as myArm:

    # Initilize cubic spline
    Kp = 0.5
    spline = CubicSpline(t, trajectory_points, axis=0, bc_type='periodic')
    spline_vel = spline.derivative()

    QArm_Interface.attach_QArm(myArm)
    QArm_Interface.write_to_arm(start_phi) # move arm to starting position
    QArm_Interface.close_gripper()
    time.sleep(1)

    #TODO: Implement trajectory generation as detailed in Exercise 5 and Concept Review
    
    # Use the following helper functions:
    # - QArm_Interface.write_to_arm(joint_positions)
    # - phi = QArm_Interface.read_from_arm()
    # - J_inv = QArm_Interface.Inv_Jacobian(phi)
    # - p4, _ = QArm_Interface.forward_kinematics(phi)
    #
    # - spline(t) to find position at a certain time
    # - spline_vel(t) to find velocity at a certain time
    #
    # - time.time() gives current time

    time.sleep(4)