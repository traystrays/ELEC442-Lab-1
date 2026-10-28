import numpy as np

L_1 = 0.1400
L_2 = 0.3500
L_3 = 0.0500
L_4 = 0.2500
L_5 = 0.1500
BETA = np.arctan(L_3/L_2)

LAMBDA_1 = L_1
LAMBDA_2 = np.sqrt(L_2**2 + L_3**2)
LAMBDA_3 = L_4 + L_5

def forward_kin(phi):
    '''
    Implement a forward kinematics function for the QArm

    Input:
        phi: the joint angles in radians, in order of base, shoulder, elbow, wrist

    Output:
        location: the end effector coordinate system origin position in the base {0} coordinate system
        rotation: rotation matrix from the end effector {4} coordinate system to the base {0} coordinate system
    '''

    phi = np.array(phi, dtype=float, copy=True)

    # == your code here ==

    # Replace these two definitions
    location = None
    rotation = None

    # == end of your code ==

    return location, rotation



if __name__ == '__main__':

    # Function input is radians

    phi = np.radians(np.array([0, 0, 0, 0]))
    location, rotation = forward_kin(phi)
    print('Input 1 -> location:', location, 'rotation:', rotation)
    
    phi = np.radians(np.array([30, 55, -60, 0]))
    location, rotation = forward_kin(phi)
    print('Input 2 -> location:', location, 'rotation:', rotation)