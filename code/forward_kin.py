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

def dh_matrix(dh_params: np.array):
    '''
    Creates a dh_matrix based on dh_params.

    dh_params input must be a numpy array of shape (1,4)
    '''

    if dh_params.shape != (1, 4):
        raise ValueError("dh_params must be a numpy array of shape (1,4)")

    theta, d, a, alpha = dh_params[0]

    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])

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

    # i-1_T_i, transformation matrix from frame {i} to frame {i-1}
    # Find 0_T_4


    dh_params = np.array([[phi[0], LAMBDA_1, 0, - np.pi/2],
                           [phi[1] - np.pi/2 + BETA, 0, LAMBDA_2, 0],
                           [phi[2] - BETA, 0, 0, -np.pi/2],
                           [phi[3], LAMBDA_3, 0, 0]])

    T = np.eye(4)

    for row in dh_params:
        i_T_i1 = dh_matrix(row.reshape(1, 4))
        T = np.dot(T, i_T_i1)

    # Replace these two definitions
    location = T[:3, 3]
    rotation = T[:3, :3]

    # removes very small values close to zero
    location[np.isclose(location, 0, atol=1e-10)] = 0.0
    rotation[np.isclose(rotation, 0, atol=1e-10)] = 0.0

    # round to 5 decimal places
    location = np.round(location, 3)
    rotation = np.round(rotation, 3)

    # == end of your code ==

    return location, rotation



if __name__ == '__main__':

    # Function input is radians

    phi = np.radians(np.array([-90, 73.5, -75, 0]))
    location, rotation = forward_kin(phi)
    print('Input 1 -> location:', location, 'rotation:', rotation)
    
    phi = np.radians(np.array([90, -85, 75, 90]))
    location, rotation = forward_kin(phi)
    print('Input 2 -> location:', location, 'rotation:', rotation)