import numpy as np
from math import sqrt,pi
#import tf

scaling = False

# Implements Kabsch algorithm - best fit.
# Supports scaling (umeyama)
# Compares well to SA results for the same data.
# Input:
#     Nominal  A Nx3 matrix of points
#     Measured B Nx3 matrix of points
# Returns s,R,t
# s = scale B to A
# R = 3x3 rotation matrix (B to A)
# t = 3x1 translation vector (B to A)

def rotz(theta):
    rot = np.matrix([[np.cos(theta), -np.sin(theta)],
                 [np.sin(theta),  np.cos(theta)]])
    return rot

def rotz3(Z_angle):
    rot = np.matrix([[np.cos(Z_angle), -np.sin(Z_angle),   0.0],
                 [np.sin(Z_angle),  np.cos(Z_angle),   0.0],
                 [0.0,      0.0,        1.0]])
    return rot
def rot2quat(R):
    
    M1 = R
    #get the real part of the quaternion first
    w = np.math.sqrt(abs(float(1)+M1[0,0]+M1[1,1]+M1[2,2]))*0.5
    x = (M1[2,1]-M1[1,2])/(4*w)
    y = (M1[0,2]-M1[2,0])/(4*w)
    z = (M1[1,0]-M1[0,1])/(4*w)
    qr_base_quat = (x,y,z,w)
    
    return qr_base_quat

def rotMat2quatern(R):
    # this function can transform the rotation matrix into quatern
    q = np.zeros(4)
    K = np.zeros([4, 4])
    K[0, 0] = 1 / 3 * (R[0, 0] - R[1, 1] - R[2, 2])
    K[0, 1] = 1 / 3 * (R[1, 0] + R[0, 1])
    K[0, 2] = 1 / 3 * (R[2, 0] + R[0, 2])
    K[0, 3] = 1 / 3 * (R[1, 2] - R[2, 1])
    K[1, 0] = 1 / 3 * (R[1, 0] + R[0, 1])
    K[1, 1] = 1 / 3 * (R[1, 1] - R[0, 0] - R[2, 2])
    K[1, 2] = 1 / 3 * (R[2, 1] + R[1, 2])
    K[1, 3] = 1 / 3 * (R[2, 0] - R[0, 2])
    K[2, 0] = 1 / 3 * (R[2, 0] + R[0, 2])
    K[2, 1] = 1 / 3 * (R[2, 1] + R[1, 2])
    K[2, 2] = 1 / 3 * (R[2, 2] - R[0, 0] - R[1, 1])
    K[2, 3] = 1 / 3 * (R[0, 1] - R[1, 0])
    K[3, 0] = 1 / 3 * (R[1, 2] - R[2, 1])
    K[3, 1] = 1 / 3 * (R[2, 0] - R[0, 2])
    K[3, 2] = 1 / 3 * (R[0, 1] - R[1, 0])
    K[3, 3] = 1 / 3 * (R[0, 0] + R[1, 1] + R[2, 2])
    # print(R)
    # print("***********")
    # print(K)
    D, V = np.linalg.eig(K)
    # print(K)
    pp = 0
    for i in range(1, 4):
        if(D[i] > D[pp]):
            pp = i
    # print(D[pp])
    # print(D)
    q = V[:, pp]
    q = np.array([q[1], q[2], q[3], q[0]])
    return q

def rigid_transform_3D(A, B, scale):
    assert len(A) == len(B)

    N = A.shape[0];  # total points
    dim = A.shape[1]

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # center the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    if scale:
        H = np.transpose(BB) * AA / N
    else:
        H = np.transpose(BB) * AA

    U, S, Vt = np.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print "Reflection detected"
        Vt[dim-1, :] *= -1
        R = Vt.T * U.T
        
    print(R)

    if scale:
        varA = np.var(A, axis=0).sum()
        c = 1 / (1 / varA * np.sum(S))  # scale factor
        t = -R * (centroid_B.T * c) + centroid_A.T
    else:
        c = 1
        t = -R * B[0].T + A[0].T
        
        
    #Calculate Quatanions
    
    #qr_base_quat = rotMat2quatern(R)
    qr_base_quat=rot2quat(R)
    

    qr_base_quat = qr_base_quat/np.linalg.norm(qr_base_quat)

    return t, qr_base_quat




if __name__ == '__main__':
    


    
    # Test
    A = np.matrix([[-3.489348347550435,-3.01,0],
                   [-5.45,-3.1,0]])

    B = np.matrix([[-3.08,0.0,0],
                   [-3.08,1.95,0]])
    
    
    A = np.matrix([[3,-3,0],
                   [1,4.0,0]]) 
    # B = rotz(1.52)*A+np.matrix([[3,-3]])
   
    W = np.matrix([[-3.4221571874321994, -2.8740788777105073, 0],
                   [-6.177316558199416, 3.105796647211759, 0]])
    Q = np.matrix([[2.67, 3.23, 0],
                  [-3.08, 0.0, 0]])




    
    # recover the transformation
    # ret_t, ret_R = rigid_transform_3D((A)*rotz3(-pi/2)+np.matrix([[3,3,0]]),A,1)
    ret_t, ret_R = rigid_transform_3D(W,Q,1)
    #s, ret_R, ret_t = umeyama(A, B)



    print "Points A"
    print A
    print ""

    print "Points B"
    print B
    print ""

    print "Rotation"
    print ret_R
    print ""

    print "Translation"
    print ret_t
    print ""

