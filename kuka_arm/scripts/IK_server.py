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
import pickle
import os.path

def handle_calculate_IK(req, check_error = True):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            start_time0 = timeit.default_timer()
            
            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
            a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
            q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
            
            # Joint angle symbols
            theta1, theta2, theta3, theta4, theta5, theta6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
      
            # Modified DH params
            dh = {alpha0:     0, a0:      0, d1:  0.75, 
                  alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
                  alpha2:     0, a2:   1.25, d3:     0,
                  alpha3: -pi/2, a3: -0.054, d4:  1.50,
                  alpha4:  pi/2, a4:      0, d5:     0,
                  alpha5: -pi/2, a5:      0, d6:     0,
                  alpha6:     0, a6:      0, d7: 0.303,         q7: 0}
            
            # Define Modified DH Transformation matrix
            # defined in function dh_transform


            # Create individual transformation matrices
            R0_1 = dh_rotation(q1, alpha0, a0, d1)
            R1_2 = dh_rotation(q2, alpha1, a1, d2)
            R2_3 = dh_rotation(q3, alpha2, a2, d3)

            R0_1 = R0_1.subs(dh)
            R1_2 = R1_2.subs(dh)
            R2_3 = R2_3.subs(dh)
            

            R0_2 = R0_1 * R1_2
            R0_3 = R0_2 * R2_3

            R_z = rot_z(pi)
            R_y = rot_y(-pi/2)
            R_corr = simplify(R_z * R_y)

            R_z_r = rot_y(pi/2)
            R_y_r = rot_z(-pi)
            R_corr_rev = simplify(R_z_r * R_y_r)
            
            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            rospy.loginfo("values: ({0},{1},{2}) ({3},{4},{5})".format(px,py,pz,roll,pitch,yaw))

            # Calculate joint angles using Geometric IK method
            # Compute wrist joint location by moving along the dh z-axis of the gripper (need apply a correction first)
            R_roll = rot_x(roll)
            R_pitch = rot_y(pitch)
            R_yaw = rot_z(yaw)
            # instrinsic rotation from euler angles
            Rrpy = R_yaw * R_pitch * R_roll
            Rrpy_cor = Rrpy * R_corr_rev

            wx = px - dh[d7] * Rrpy_cor[0,2]
            wy = py - dh[d7] * Rrpy_cor[1,2]
            wz = pz - dh[d7] * Rrpy_cor[2,2]

            # Compute problem of position using law of cosine
            p = sqrt(wy**2 + wx**2) - dh[a1]
            f = sqrt(dh[d4]**2 + dh[a3]**2)
            o = (wz - dh[d1])
            s = sqrt(p**2 + o**2)
            ang1 = atan2(o, p)
            ang2 = arg_law_of_cosine(f, dh[a2], s, 0)
            ang3 = arg_law_of_cosine(s, dh[a2], f, pi)

            theta1 = atan2(wy, wx)
            theta2 = pi/2 - ang2 - ang1
            theta3 = pi/2 - ang3 + atan2(dh[a3], dh[d4])
            
            # Compute problem of wrist orientation using the roll,pitch,yaw matrix constructed from euler rotations
            R0_3s = R0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            R3_G = R0_3s.T * Rrpy

            theta4, theta5, theta6 = get_euler_angles_from_homogeneous(R3_G)


            # Populate response for the IK request
	    joint_trajectory_point.positions = [theta1.evalf(), theta2.evalf(), theta3.evalf(), theta4.evalf(), theta5.evalf(), theta6.evalf()]

            # Perfom an optional error computation
            if check_error:
                res_fwd = get_forward_kinematics(*(joint_trajectory_point.positions))
                error([px,py,pz,roll,pitch,yaw], res_fwd)
                
            print("result {0}".format(*(joint_trajectory_point.positions)))
            print("took {0}s. Done: {1}/{2}".format(timeit.default_timer() - start_time0, (x+1), len(req.poses)))
            
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def error(pos1, pos2):
    error = sqrt(sum([(x - y)**2 for (x,y) in zip(pos1,pos2)])).evalf()
    print("input: {0} vs output: {1}, with error = {2}".format(pos1, pos2, error))
    
def arg_law_of_cosine(c, a, b, fallback):
    # check if input isnt a triangle (out of workspace point)
    if c > a + b or a > c + b or b > c + a:
        print("Out of envelope {0},{1},{2}".format(c,a,b))
        return fallback
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

def print_homogeneous_transform():
    a, b, c = symbols("a,b,c")
    px, py, pz = symbols("px,py,pz")
    R_roll = rot_x(a)
    R_pitch = rot_y(b)
    R_yaw = rot_z(c)
    # instrinsic rotation from euler angles
    Rrpy = R_yaw * R_pitch * R_roll
    pprint(make_homogeneous(Rrpy, Matrix([[px],[py],[pz]])))

    q, alpha, a, d = symbols("q,alpha,a,d")
    T = dh_transform(q, alpha, a, d)
    pprint(T)


def get_forward_kinematics(j1,j2,j3,j4,j5,j6):

    dh_filename = 'total_dh.pickle'
    
    d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
    a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
    q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")

    s = {alpha0:     0, a0:      0, d1:  0.75, 
         alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
         alpha2:     0, a2:   1.25, d3:     0,
         alpha3: -pi/2, a3: -0.054, d4:  1.50,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303,         q7: 0}

    # forward kinematics take long to compute so we store the matrix
    if os.path.isfile(dh_filename):
        with open(dh_filename, 'rb') as inf:
            T0_total = pickle.loads(inf.read())
            A = T0_total.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})
            r, p, y = get_euler_angles(A)
            return A[0,3], A[1,3], A[2,3], r, p, y
    
    T0_1 = dh_transform(q1, alpha0, a0, d1)
    T1_2 = dh_transform(q2, alpha1, a1, d2)
    T2_3 = dh_transform(q3, alpha2, a2, d3)
    T3_4 = dh_transform(q4, alpha3, a3, d4)
    T4_5 = dh_transform(q5, alpha4, a4, d5)
    T5_6 = dh_transform(q6, alpha5, a5, d6)
    T6_G = dh_transform(q7, alpha6, a6, d7)

    T0_1 = T0_1.subs(s)
    T1_2 = T1_2.subs(s)
    T2_3 = T2_3.subs(s)
    T3_4 = T3_4.subs(s)
    T4_5 = T4_5.subs(s)
    T5_6 = T5_6.subs(s)
    T6_G = T6_G.subs(s)

    # pprint(T0_1)
    # pprint(T1_2)
    # pprint(T2_3)
    # pprint(T3_4)
    # pprint(T4_5)
    # pprint(T5_6)
    # pprint(T6_G)
    
    T0_2 = (T0_1 * T1_2)
    T0_3 = (T0_2 * T2_3)
    T0_4 = (T0_3 * T3_4)
    T0_5 = (T0_4 * T4_5)
    T0_6 = (T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    R_z = make_homogeneous(rot_z(pi), Matrix([[0],[0],[0]]))
    R_y = make_homogeneous(rot_y(-pi/2), Matrix([[0],[0],[0]]))
    R_corr = simplify(R_z * R_y)

    T0_total = simplify(T0_G * R_corr)

    with open(dh_filename, 'wb') as outf:
        outf.write(pickle.dumps(T0_total))

    # print("T0_1 = {0}".format(T0_1.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_2 = {0}".format(T0_2.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_3 = {0}".format(T0_3.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_4 = {0}".format(T0_4.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_5 = {0}".format(T0_5.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_6 = {0}".format(T0_6.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    # print("T0_G = {0}".format(T0_G.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})))
    A = T0_total.evalf(subs={q1: j1, q2: j2, q3: j3, q4: j4, q5: j5, q6: j6})
    print("T0_total = {0},{1}".format(A, get_euler_angles(A)))
    r, p, y = get_euler_angles(A)
    return A[0,3], A[1,3], A[2,3], r, p, y 

def get_euler_angles(T):
    pitch = atan2(-T[2,0], sqrt(T[0,0]**2 + T[1,0]**2))
    yaw = atan2(T[1,0],T[0,0])
    roll = atan2(T[2,1],T[2,2])
    return (roll, pitch, yaw)

def get_euler_angles_from_homogeneous(T):
    gamma = atan2(T[1,1],T[1,2])
    alpha = atan2(T[2,0],-T[0,0])
    beta = atan2(sqrt(T[1,2]**2 + T[1,1]**2),T[1,0])
    return (alpha, beta, gamma)

def IK_server():
    #print_homogeneous_transform()
    #get_forward_kinematics(0,0,0,0,0,0)
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    rospy.loginfo("Ready to receive an IK request")
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
