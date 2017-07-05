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
import timeit


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        rospy.loginfo("Pose: %s", req.poses[0])
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
            a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
            q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
            
            # Joint angle symbols
            theta1, theta2, theta3, theta4, theta5, theta6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            print("before dh param dictionary")
      
            # Modified DH params
            s = {alpha0:     0, a0:      0, d1:  0.75, 
                 alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
                 alpha2:     0, a2:   1.25, d3:     0,
                 alpha3: -pi/2, a3: -0.054, d4:  1.50,
                 alpha4:  pi/2, a4:      0, d5:     0,
                 alpha5: -pi/2, a5:      0, d6:     0,
                 alpha6:     0, a6:      0, d7: 0.303,         q7: 0}
            
            # Define Modified DH Transformation matrix
            # defined in function dh_transform
            
            # Create individual transformation matrices
            print("computing all the nice transforms")
            T0_1 = dh_transform(q1, alpha0, a0, d1)
            T1_2 = dh_transform(q2, alpha1, a1, d2)
            T2_3 = dh_transform(q3, alpha2, a2, d3)
            T3_4 = dh_transform(q4, alpha3, a3, d4)
            T4_5 = dh_transform(q5, alpha4, a4, d5)
            T5_6 = dh_transform(q6, alpha5, a5, d6)
            T6_G = dh_transform(q7, alpha6, a6, d7)

            print("subs")
            T0_1 = T0_1.subs(s)
            T1_2 = T1_2.subs(s)
            T2_3 = T2_3.subs(s)
            T3_4 = T3_4.subs(s)
            T4_5 = T4_5.subs(s)
            T5_6 = T5_6.subs(s)
            T6_G = T6_G.subs(s)

            print("simplifying")
            start_time = timeit.default_timer()
            T0_2 = T0_1 * T1_2
            T0_3 = T0_2 * T2_3
            T0_4 = T0_3 * T3_4
            T0_5 = T0_4 * T4_5
            T0_6 = T0_5 * T5_6
            T0_G = T0_6 * T6_G
            end_time = timeit.default_timer() - start_time
            print("simplifying took {0}s".format(end_time))

            print("correction matrix simplify")
            R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                          [sin(pi),  cos(pi), 0, 0],
                          [      0,        0, 1, 0],
                          [      0,        0, 0, 1]])
            R_y = Matrix([[ cos(-pi/2),        0, sin(-pi/2), 0],
                          [          0,        1,          0, 0],
                          [-sin(-pi/2),        0, cos(-pi/2), 0],
                          [          0,        0,          0, 1]])
            R_corr = simplify(R_z * R_y)

            print("total matrix simplify")
            start_time = timeit.default_timer()
            T0_total = simplify(T0_G * R_corr)
            end_time = timeit.default_timer() - start_time
            print("simplifying total matrix took {0}s".format(end_time))
            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            print("getting the end effector stuff")
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            rospy.loginfo("values: ({0},{1},{2}) ({3},{4},{5})".format(pz,py,pz,roll,pitch,yaw))
            # Calculate joint angles using Geometric IK method
            print("gonna calculate the IK")
            
            R_roll = rot_x(roll)
            R_pitch = rot_y(pitch)
            R_yaw = rot_z(yaw)            
            Rrpy = R_roll * R_pitch * R_yaw
            n = Rrpy.col(2)

            d67 = s[d6] + s[d7]
            wx = px - (d67) * n[0]
            wy = py - (d67) * n[1]
            wz = pz - (d67) * n[2]
            print("Got wrist center ({0},{1},{2})".format(wx, wy, wz))

            theta1 = atan2(wy, wx)
            r = sqrt(wy**2 + wx**2)
            dia = sqrt(r**2 + (wz - s[d1])**2)
            ang1 = atan2(wz - s[d1], r)
            ang2 = arg_law_of_cosine(s[a2], s[a1], dia)
            ang3 = arg_law_of_cosine(dia, s[a1], s[a2])
            theta2 = ang2 + ang1
            theta3 = ang3 - pi
            print("Got first three angles ({0},{1},{2})".format(theta1, theta2, theta3))

            T0_3s = T0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            T0_3s.col_del(-1)
            T0_3s.row_del(-1)
            R3_6 = T0_3s.T * Rrpy
            print("Got the final 3_6 matrix {0}".format(R3_6))
            
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def arg_law_of_cosine(c, a, b):
    res = (c**2 - a**2 - b**2) / (-2*(a*b))
    return acos(res)

def dh_transform(q, alpha, a, d):
    return Matrix([[           cos(q),           -sin(q),           0,             a],
                   [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                   [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                0,                 0,           0,             1]])

def rot_x(q1):
    return Matrix([[ 1,              0,        0],
                   [ 0,        cos(q1), -sin(q1)],
                   [ 0,        sin(q1),  cos(q1)]])

def rot_y(q2):
    return Matrix([[ cos(q2),        0,  sin(q2)],
                   [       0,        1,        0],
                   [-sin(q2),        0,  cos(q2)]])

def rot_z(q3):
    return Matrix([[ cos(q3), -sin(q3),        0],
                   [ sin(q3),  cos(q3),        0],
                   [ 0,              0,        1]])

def make_homogeneous(R, t):
    return R.row_join(t).col_join(Matrix([[0, 0, 0, 1]]))
    
def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    rospy.loginfo("Ready to receive an IK request")
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
