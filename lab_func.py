#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.104
	L7 = 0.085
	L8 = 0.092

	xe = 0.0535
	ze = 0.059

	xzero = 0.15
	yzero = 0.15
	zzero = 0.01

	mx = -xzero+L3+L5+L7
	my = yzero+L2-L4+L6+L8+ze
	mz = zzero+L1+xe

	M = np.array([[0, -1, 0, mx],
				  [0, 0, -1, my],
				  [1, 0, 0, mz],
				  [0, 0, 0, 1]])

	# w1 = np.array([0, 0, 1])
	# w2 = np.array([0, 1, 0])
	# w3 = np.array([1, 0, 0])
	# w4
	Q1 = np.array([-xzero, yzero, zzero+L1])
	Q2 = np.array([-xzero, yzero+L2, zzero+L1])
	Q3 = np.array([-xzero+L3, yzero+L2, zzero+L1])
	# Q4 = [-xzero+L3, yzero+L2-L4, zzero+L1]
	Q4 = np.array([-xzero+L3+L5, yzero+L2-L4, zzero+L1])
	Q5 = np.array([-xzero+L3+L5, yzero+L2-L4+L6, zzero+L1])
	Q6 = np.array([-xzero+L3+L5+L7, yzero+L2-L4+L6, zzero+L1])

	w1 = np.array([0, 0, 1])
	w2 = np.array([0, 1, 0])
	w3 = np.array([0, 1, 0])
	w4 = np.array([0, 1, 0])
	w5 = np.array([1, 0, 0])
	w6 = np.array([0, 1, 0])

	v1 = np.cross(-w1, Q1)
	v2 = np.cross(-w2, Q2)
	v3 = np.cross(-w3, Q3)
	v4 = np.cross(-w4, Q4)
	v5 = np.cross(-w5, Q5)
	v6 = np.cross(-w6, Q6)

	print("v1: " + str(v1))
	print("w1: " + str(w1))
	print("Q1: " + str(Q1))

	S1 = np.concatenate([w1, v1])
	S2 = np.concatenate([w2, v2])
	S3 = np.concatenate([w3, v3])
	S4 = np.concatenate([w4, v4])
	S5 = np.concatenate([w5, v5])
	S6 = np.concatenate([w6, v6])

	S = np.column_stack([S1, S2, S3, S4, S5, S6])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	theta = [theta1, theta2, theta3, theta4, theta5, theta6]
	# T = [ [1.0, 0.0, 0.0], \
    #   [0.0, 1.0, 0.0], \
    #   [0.0, 0.0, 1.0] ]
	T = np.eye(4)

	M, S = Get_MS()
	print("S matrix:")
	print(S)
	print("\n")
	# for i in range(6):
	# 	w = S[i][:3]
	# 	v = S[i][3:]
	# 	s_skew = np.array([[0, -w[2], w[1],v[0]], [w[2], 0, -w[0],v[1]], [-w[1], w[0], 0,v[2]], [0, 0, 0, 0]])
	# 	T = T @ expm(s_skew * theta[i])

	for i in range(6):
		w = S[:,i][:3]
		v = S[:,i][3:]

		w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])

		s_skew = np.zeros((4, 4))
		s_skew[:3, :3] = w_hat
		s_skew[:3, 3] = v

		print(f"t{i}", str(expm(s_skew * theta[i])) + "\n")
		print(f"skew{i}", str(s_skew) + "\n")
		T = T @ expm(s_skew * theta[i])
	# ==============================================================#

	T_06 = T @ M  
	print("T_06", str(T_06) + "\n\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.104
	L7 = 0.085
	L8 = 0.092
	L9 = 0.0535
	L10 = 0.059

	xzero = 0.15
	yzero = 0.15
	zzero = 0.01
	# 1
	xgrip = xWgrip + xzero
	ygrip = yWgrip - yzero
	zgrip = zWgrip - zzero
	print("xgrip: " + str(xgrip))
	print("ygrip: " + str(ygrip))	
	print("zgrip: " + str(zgrip))
	# 2
	xcen = xgrip - L9 * np.cos(np.deg2rad(yaw_WgripDegree))
	ycen = ygrip - L9 * np.sin(np.deg2rad(yaw_WgripDegree))
	zcen = zgrip 
	print("xcen: " + str(xcen))
	print("ycen: " + str(ycen))
	print("zcen: " + str(zcen))
	# 3
	# theta1 = math.atan2(ycen, xcen)
	a = math.atan2(ycen, xcen)
	# LI
	# c = math.atan2(L3+L5+L7,L2)
	# theta1 = a+c-np.deg2rad(90)
	d = L2 - L4 + L6 
	r = np.sqrt(xcen**2 + ycen**2)
	theta1 = a - math.asin(d / r)
	print("a: " + str(np.rad2deg(a)))
	# print("c: " + str(np.rad2deg(c)))

	# 4 
	theta6 = theta1 + np.deg2rad(90) - np.deg2rad(yaw_WgripDegree)
	# 5
	z3end = zcen + L10 + L8
	# LI
	# x3end = np.cos(theta1) * (L3 + L5)
	# y3end = np.sin(theta1) * (L3 + L5)
	x3end = xcen + d*sin(theta1)
	y3end = ycen - d*cos(theta1)
	# 6
	X = x3end
	Z = z3end - L1
	R = np.sqrt(X**2 + Z**2)
	theta2 = -math.atan2(Z,X) - math.acos((R**2 + L3**2 - L5**2)/(2*L3*R))
	theta4 = math.atan2(L3* np.sin(theta2)+Z, X-L3*np.cos(theta2))
	theta3 = -theta4 + -theta2
	theta5 = np.deg2rad(-90)

	print("theta1: " + str(np.rad2deg(theta1)))
	print("theta2: " + str(np.rad2deg(theta2)))
	print("theta3: " + str(np.rad2deg(theta3)))
	print("theta4: " + str(np.rad2deg(theta4)))
	print("theta5: " + str(np.rad2deg(theta5)))
	print("theta6: " + str(np.rad2deg(theta6)))

	# raise error
	# raise NotImplementedError("Inverse kinematics function not implemented yet.")

	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
