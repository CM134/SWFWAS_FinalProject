#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 28 22:14:37 2020

@author: christian
"""
# kabsch algorithm
import numpy as np





def make_tf(r1,r2,q1,q2):



    r = np.asarray([r1,r2])
    q = np.asarray([q1,q2])
    
    # Applying kabsch to find frame
    r0 = np.mean(r,0)
    q0 = np.mean(q,0)
    
    rc = r-r0
    qc = q-q0

    # 2. compute covariance matrix
    
    C = np.dot(np.transpose(qc), rc)
    
    # 3. calculate optimal rotation matirx
    
    V, S, Wt = np.linalg.svd(C)
    
    
    # Create Rotation matrix U
    rot = np.dot(V, Wt)
    
    d = np.linalg.det(rot)
    
    if d< 0.0:
        Wt[2,:] *= -1
        R = Wt.T * V.T
    
    trans = r0 - np.dot(q0,rot)
    print(trans)
    #Calculate Quatanions
    M1 = rot
    #get the real part of the quaternion first
    w = np.math.sqrt(float(1)+M1[0,0]+M1[1,1]+M1[2,2])*0.5
    x = (M1[2,1]-M1[1,2])/(4*w)
    y = (M1[0,2]-M1[2,0])/(4*w)
    z = (M1[1,0]-M1[0,1])/(4*w)
    
    #print(x)
    #print(y)
    #print(z)
    #print(w)
    
   
    
    #print(trans)
    
    
    
    qr_base_trans = (trans[0],trans[1],trans[2])
    qr_base_quat = (x,y,z,w)
    
    return qr_base_trans, qr_base_quat


if __name__ == "__main__":
    r1 = [-4.2, 3.1, 0]
    r2 = [-6.12, 3.1, 0]
    q1 = [-3.08, 1.95, 0]
    q2 = [-3.08, 0.0, 0]
    
    qr_base_trans, qr_base_quat = make_tf(r1,r2,q1,q2)
	
			
			
    		
			

    r1 = [-6.745,0.44,0]
    r2 = [-4.7,2,0]
    q1 = [0.1,3.5,0]
    q2 = [3.02,1.15,0]
    
    r1 = [1,1,0]
    r2 = [1,1,0]
    q1 = [-1,2,0]
    q2 = [-1,2,0]




    tran,quat = make_tf(r1,r2,q1,q2)


