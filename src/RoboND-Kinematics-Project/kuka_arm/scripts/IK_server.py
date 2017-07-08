#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np


def build_trans_matrix(alpha, a, q, d):
    return Matrix([[cos(q), -sin(q), 0, a], 
    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d], 
    [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
    [0, 0, 0, 1]])


def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    return R_x
    
def rot_y(q):              
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    return R_y

def rot_z(q):    
    R_z = Matrix([[cos(q), -sin(q), 0],
                 [sin(q), cos(q), 0],
                 [0, 0, 1]])
    return R_z              


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Define DH param symbols

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        a01 = 0
        a12 = 0.35
        a23 = 1.25
        a34 = -0.054
        a45 = 0
        a56 = 0
        a67 = 0

        # DH Parameters
        dh = {alpha0: 0, a0: a01, d1: 0.75,
              alpha1: -pi/2, a1: a12, d2: 0, q2: q2 - np.pi/2,
              alpha2: 0, a2: a23, d3: 0,
              alpha3: -pi/2, a3: a34, d4: 1.50,
              alpha4: pi/2, a4: a45, d5: 0, 
              alpha5: -pi/2, a5: a56, d6: 0, 
              alpha6: 0, a6: a67, d7: 0.303, q7: 0}


        # Define Modified DH Transformation matrix
        # Create individual transformation matrices

        T0_1 = build_trans_matrix(alpha0, a0, q1, d1)
        T0_1 = T0_1.subs(dh)

        T1_2 = build_trans_matrix(alpha1, a1, q2, d2)
        T1_2 = T1_2.subs(dh)

        T2_3 = build_trans_matrix(alpha2, a2, q3, d3)
        T2_3 = T2_3.subs(dh)

        T3_4 = build_trans_matrix(alpha3, a3, q4, d4)
        T3_4 = T3_4.subs(dh)

        T4_5 = build_trans_matrix(alpha4, a4, q5, d5)
        T4_5 = T4_5.subs(dh)

        T5_6 = build_trans_matrix(alpha5, a5, q6, d6)
        T5_6 = T5_6.subs(dh)

        T6_7 = build_trans_matrix(alpha6, a6, q7, d7)
        T6_7 = T6_7.subs(dh)

        T0_2 = (T0_1 * T1_2)

        T0_3 = (T0_2 * T2_3)

        T0_4 = (T0_3 * T3_4)

        T0_5 = (T0_4 * T4_5)

        T0_6 = (T0_5 * T5_6)

        T0_7 = (T0_6 * T6_7)

        R_corr = rot_z(pi) * rot_y(-pi/2)

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            R_rpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll)
            R0_6 = R_rpy * R_corr

            # 0.303 is length of end effector
            Wc = Matrix([[px], [py], [pz]]) - (0.303) * R0_6.col(2)
            theta1 = atan2(Wc[1], Wc[0])

            L2_3 = 1.25
            L3_5 = 1.5

            JX0_2 = 0.35 / cos(theta1) # X axis of joint 2 wrt joint 0, We also rotate this axis 
            JZ0_2 = 0.75 # Z axis of joint 2 wrt joint 0

            # Distance between joint 2 and joint 5 / wrist center
            L2_5 = sqrt((Wc[0] - JX0_2)**2 + Wc[1]**2 + (Wc[2] - JZ0_2)**2)

            # 180 - Angle between links connecting Joint2-3 and joint3-5
            # Add the offset and add correction for the knee bend
            D = (L2_5**2 - L2_3**2 - L3_5**2) / (2 * L2_3 * L3_5)
            D = 1. if D > 1 else -1. if D < -1 else D # Precaution agains imaginary numbers
            theta3 = atan2(0.054, 1.5) - np.pi/2. + acos(D)

            # Angle between links connecting Joint2-3 and joint2-5
            # Add the offset, i.e. angle between XY plant of joint 2 and Wrist center 
            # and add correction for the knee bend
            D = (-L3_5**2 + L2_3**2 + L2_5**2) / (2 * L2_3 * L2_5)
            D = 1. if D > 1 else -1. if D < -1 else D
            theta2 = atan2(sqrt(1. - D**2), D) + atan2((Wc[2]-JZ0_2), sqrt((Wc[0]-JX0_2)**2 + (Wc[1])**2))
            theta2 = np.pi / 2. - theta2

            # Calculate theta4-6, 
            R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})[0:3, 0:3]
            R3_6 = R0_3.T * R_rpy * R_corr.T
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])            
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            print ['wc', Wc, 'p:', [px, py, pz], 'q:', [theta1, theta2, theta3, theta4, theta5, theta6]]
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
