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


            start_time = timeit.default_timer()
            # Create individual transformation matrices
            print("computing all the nice transforms")

            R0_1 = dh_rotation(q1, alpha0, a0, d1)
            R1_2 = dh_rotation(q2, alpha1, a1, d2)
            R2_3 = dh_rotation(q3, alpha2, a2, d3)
            # R3_4 = dh_rotation(q4, alpha3, a3, d4)
            # R4_5 = dh_rotation(q5, alpha4, a4, d5)
            # R5_6 = dh_rotation(q6, alpha5, a5, d6)
            # R6_G = dh_rotation(q7, alpha6, a6, d7)
            
            # T0_1 = dh_transform(q1, alpha0, a0, d1)
            # T1_2 = dh_transform(q2, alpha1, a1, d2)
            # T2_3 = dh_transform(q3, alpha2, a2, d3)
            # T3_4 = dh_transform(q4, alpha3, a3, d4)
            # T4_5 = dh_transform(q5, alpha4, a4, d5)
            # T5_6 = dh_transform(q6, alpha5, a5, d6)
            # T6_G = dh_transform(q7, alpha6, a6, d7)

            print("subs")
            R0_1 = R0_1.subs(s)
            R1_2 = R1_2.subs(s)
            R2_3 = R2_3.subs(s)
            # R3_4 = R3_4.subs(s)
            # R4_5 = R4_5.subs(s)
            # R5_6 = R5_6.subs(s)
            # R6_G = R6_G.subs(s)
            
            # T0_1 = T0_1.subs(s)
            # T1_2 = T1_2.subs(s)
            # T2_3 = T2_3.subs(s)
            # T3_4 = T3_4.subs(s)
            # T4_5 = T4_5.subs(s)
            # T5_6 = T5_6.subs(s)
            # T6_G = T6_G.subs(s)

            print("simplifying")
            R0_2 = R0_1 * R1_2
            R0_3 = simplify(R0_2 * R2_3)

            # R3_5 = R3_4 * R4_5
            # R3_6 = R3_5 * R5_6
            # R3_G = R3_6 * R6_G
            
            # T0_2 = (T0_1 * T1_2)
            # T0_3 = (T0_2 * T2_3)
            # T0_4 = (T0_3 * T3_4)
            # T0_5 = (T0_4 * T4_5)
            # T0_6 = (T0_5 * T5_6)
            # T0_G = simplify(T0_6 * T6_G)

            print("correction matrix simplify")
            # R_z = rot_z(pi)
            # R_y = rot_y(-pi/2)
            # R_corr = simplify(R_z * R_y)

            # R_z2 = make_homogeneous(rot_z(pi), Matrix([[0],[0],[0]]))
            # R_y2 = make_homogeneous(rot_y(-pi/2), Matrix([[0],[0],[0]]))
            # R_corr2 = simplify(R_z2 * R_y2)

            print("total matrix simplify")

            # R3_total = simplify(R3_G * R_corr)
            # T0_total = simplify(T0_G * R_corr2)
            end_time = timeit.default_timer() - start_time
            print("simplifying total matrix took {0}s".format(end_time))

            print("numerically evaling forward trans")
            # print("T0_1 = {0}".format(T0_1.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_2 = {0}".format(T0_2.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_3 = {0}".format(T0_3.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_4 = {0}".format(T0_4.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_5 = {0}".format(T0_5.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_6 = {0}".format(T0_6.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # print("T0_G = {0}".format(T0_G.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})))
            # A = T0_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0})
            # print("T0_total = {0},{1}".format(A, get_euler_angles(A)))
            
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

            d67 = d6 + d7
            wx = px - (d67) * n[0]
            wy = py - (d67) * n[1]
            wz = pz - (d67) * n[2]
            print("Got wrist center ({0},{1},{2})".format(wx, wy, wz))

            theta1 = atan2(wy, wx)
            r = sqrt(wy**2 + wx**2) - a1
            vert_d = (wz - a3 - d1)
            dia = sqrt(r**2 + vert_d**2)
            ang1 = atan2(vert_d, r)
            ang2 = arg_law_of_cosine(d4, a2, dia)
            ang3 = arg_law_of_cosine(dia, a2, d4)
            theta2 = ang2 + ang1
            theta3 = ang3 - pi
            #print("what? ({0},{1},{2},{3},{4}".format(r.evalf(subs=s), dia.evalf(subs=s), ang1.evalf(subs=s), ang2.evalf(subs=s), ang3.evalf(subs=s)))
            #print("Got first three angles ({0},{1},{2})".format(theta1.evalf(subs=s), theta2.evalf(subs=s), theta3.evalf(subs=s)))


            start_time0 = timeit.default_timer()
            theta1 = theta1.subs(s)
            theta2 = theta2.subs(s)
            theta3 = theta3.subs(s)
            R0_3s = R0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            R3_6 = simplify(R0_3s.T * Rrpy)
            end_time0 = timeit.default_timer() - start_time0
            print("simplifying r3_6 took {0}s".format(end_time0))

            # T0_3s = T0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            # T0_3s.col_del(-1)
            # T0_3s.row_del(-1)
            # R3_6 = T0_3s.T * Rrpy

            #print("Got the final 3_6 matrix {0}".format(R3_6.evalf(subs=s)))
            # print("Got the final 3_6 matrix {0}\nCompare vs {1}".format(R3_6.evalf(subs=s),R3_G.evalf(subs=s)))
            start_time0 = timeit.default_timer()
            theta4, theta5, theta6 = get_euler_angles_from_homogeneous(R3_6)
            end_time0 = timeit.default_timer() - start_time0
            print("simplifying theta4,5,6 took {0}s".format(end_time0))

           # print("about to return the values ({0},{1},{2},{3},{4},{5})".format(theta1.evalf(subs=s), theta2.evalf(subs=s), theta3.evalf(subs=s), theta4.evalf(subs=s), theta5.evalf(subs=s), theta6.evalf(subs=s)))
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1.evalf(), theta2.evalf(), theta3.evalf(), theta4.evalf(), theta5.evalf(), theta6.evalf()]

            end_time = timeit.default_timer() - start_time
            print("simplifying took {0}s".format(end_time))
            print("result {0}".format(joint_trajectory_point.positions))

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

def dh_rotation(q, alpha, a, d):
    return Matrix([[           cos(q),           -sin(q),           0],
                   [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha)],
                   [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha)]])

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

def get_euler_angles(T):
    pitch = atan2(-T[2,0], sqrt(T[0,0]**2 + T[1,0]**2))
    yaw = atan2(T[1,0],T[0,0])
    roll = atan2(T[2,1],T[2,2])
    return (roll, pitch, yaw)

def get_euler_angles_from_homogeneous(T):
    gamma = atan2(-T[1,1],T[1,0])
    alpha = atan2(T[0,2],-T[2,2])
    beta = atan2(sqrt(T[1,0]**2 + T[1,1]**2),T[1,2])
    return (alpha, beta, gamma)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    rospy.loginfo("Ready to receive an IK request")
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
