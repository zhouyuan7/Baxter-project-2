#!/usr/bin/env python
import sys
import tf
import math
import rospy
import time
import struct
import numpy as np
import baxter_interface
import baxter_pykdl
 
from std_msgs.msg import Header
from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import (Point,PoseStamped,Pose,Quaternion)
 
real = np.array([])
 
def callback(data):
    global real
    real = np.array(data)
 
def HomogeneousSolution():
    iJ = kdl.jacobian_pseudo_inverse
    J = kdl.jacobian
    return np.dot((np.identity(6) - np.dot(iJ, J)), np.gradient(np.sqrt(np.absolute(np.dot(J, J.transpose())))))
 
def QuaternionToEuler(quaternion):
    q = quaternion
 
    rotation = np.array([[1-2*(q[2]*q[2]+q[3]*q[3]),2*(q[1]*q[2]-q[0]*q[3]),2*(q[1]*q[3]+q[0]*q[2]),0],
                         [2*(q[1]*q[2]+q[0]*q[3]),1-2*(q[1]*q[1]+q[3]*q[3]),2*(q[2]*q[3]-q[0]*q[1]),0],
                         [2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]),0],
                         [0,0,0,1]])
    r31 = 2*(q[1]*q[3]-q[0]*q[2])
    r11 = 1-2*(q[2]*q[2]+q[3]*q[3])
    r21 = 2*(q[1]*q[2]+q[0]*q[3])
    r32 = 2*(q[2]*q[3]+q[0]*q[1])
    r33 = 1-2*(q[1]*q[1]+q[2]*q[2])
 
    theta2 = np.arctan2(-r31,np.sqrt(r11*r11 + r21*r21))
    theta1 = np.arctan2(np.divide(r21,np.cos(theta2)),np.divide(r11,np.cos(theta2)))
    theta3 = np.arctan2(np.divide(r32,np.cos(theta2)),np.divide(r33,np.cos(theta2)))
    euler = np.array([theta1,theta2,theta3])
    return euler
 
def jacobian_inverse():
    X_sub = 0.858318633316
    Y_sub = -0.0443270839514
    Z_sub = 0.18510778609
    U = 1
    right_arm = baxter_interface.limb.Limb("right")
    right_joint_names = right_arm.joint_names()
    K = 0.1
    right = baxter_interface.Limb('right').endpoint_pose()
    position = right['position']
    quaternion = right['orientation']
    quaternion = np.array([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
    XE = np.array([position[0],position[1],position[2],euler[0],euler[1],euler[2]])
    print('XE',XE)
 
    XT = np.array([X_sub, Y_sub, (Z_sub + 0.05), -0.785745835105283, -0.1115831008789236, -3.1365214718905774])
 
 
    GU = np.array([[K*(XE[0]-XT[0])],
                   [K*(XE[1]-XT[1])],
                   [K*(XE[2]-XT[2])],
                   [K*(XE[3]-XT[3])],
                   [K*(XE[4]-XT[4])],
                   [K*(XE[5]-XT[5])]])
    '''
    GU = np.array([[1],
                   [-1],
                   [1],
                   [0],
                   [0],
                   [0]])
    '''
    L0 = 0.27035
    L1 = 0.069
    Lh = 0.37082
    L4 = 0.37429
 
    C2 = math.cos(right_arm.joint_angle('right_s1'))
    C24 = math.cos(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
    S1 = math.sin(right_arm.joint_angle('right_s0'))
    S2 = math.sin(right_arm.joint_angle('right_s1'))
    S24 = math.sin(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
    C1 = math.cos(right_arm.joint_angle('right_s0'))
    C5 = math.cos(right_arm.joint_angle('right_w0'))
    S5 = math.sin(right_arm.joint_angle('right_w0'))
    C6 = math.cos(right_arm.joint_angle('right_w1'))
    S6 = math.sin(right_arm.joint_angle('right_w1'))
 
 
    jacobian = np.array([[-(L1+Lh*C2+L4*C24)*S1, -(Lh*S2+L4*S24)*C1, -L4*C1*S24, 0, 0, 0],
                         [(L1+Lh*C2+L4*C24)*C1, (Lh*S2+L4*S24)*S1, L4*S1*S24, 0, 0, 0],
                         [0, -(Lh*C2+L4*C24), -L4*C24, 0, 0, 0],
                         [0, -S1, -S1, C1*C24, -S1*C5+C1*S24*S5, C1*C24*C6-(S1*S5+C1*S24*C5)*S6],
                         [0, C1, C1, S1*C24, C1*C5+S1*S24*S5, S1*C24*C6+(C1*S5-S1*S24*C5)*S6],
                         [1, 0, 0, -S24, C24*S5, -(S24*C6+C24*C5*S6)]])
 
    inversejacobian = np.linalg.inv(jacobian)
 
    print('inversejacobian', inversejacobian)
 
    JointVelocity = np.dot(inversejacobian, GU)
 
    print('JointVelocity', JointVelocity)
 
 
    cmd = {'right_s0': JointVelocity[0], 'right_s1': JointVelocity[1], 'right_e0': 0, 'right_e1': JointVelocity[2], 'right_w0': JointVelocity[3], 'right_w1': JointVelocity[4], 'right_w2': JointVelocity[5]}
 
    #cmd = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}
 
    print('cmd',cmd)
 
    right_arm.set_joint_velocities(cmd)
 
    print('E0', right_arm.joint_angle('right_e0'))
 
def extend_jacobian():
    kdl = baxter_pykdl.baxter_kinematics('right')
    right_arm = baxter_interface.limb.Limb("right")
 
    Jacobian = kdl.jacobian()
 
    print('Jacobian',Jacobian)
 
    L2 = 0.36435
    L3 = 0.069
    L4 = 0.37429
    Lh = math.sqrt(pow(L2,2)+pow(L3,2))
 
    S2 = math.sin(right_arm.joint_angle('right_s1'))
    C2 = math.cos(right_arm.joint_angle('right_s1'))
    S24 = math.sin(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
    C24 = math.cos(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
 
    JacobianInverse = np.linalg.inv(np.array([[Jacobian[0,0],Jacobian[0,1],Jacobian[0,2],Jacobian[0,3],Jacobian[0,4],Jacobian[0,5],Jacobian[0,6]],
                                              [Jacobian[1,0],Jacobian[1,1],Jacobian[1,2],Jacobian[1,3],Jacobian[1,4],Jacobian[1,5],Jacobian[1,6]],
                                              [Jacobian[2,0],Jacobian[2,1],Jacobian[2,2],Jacobian[2,3],Jacobian[2,4],Jacobian[2,5],Jacobian[2,6]],
                                              [Jacobian[3,0],Jacobian[3,1],Jacobian[3,2],Jacobian[3,3],Jacobian[3,4],Jacobian[3,5],Jacobian[3,6]],
                                              [Jacobian[4,0],Jacobian[4,1],Jacobian[4,2],Jacobian[4,3],Jacobian[4,4],Jacobian[4,5],Jacobian[4,6]],
                                              [Jacobian[5,0],Jacobian[5,1],Jacobian[5,2],Jacobian[5,3],Jacobian[5,4],Jacobian[5,5],Jacobian[5,6]],
                                              [0,0,-Lh*S2-L4*S24,0,-L4*S24,0,0]]))
    return JacobianInverse
 
def extend_inverse_process():
    kdl = baxter_pykdl.baxter_kinematics('right')
    X_sub = 0.858318633316
    Y_sub = -0.0443270839514
    Z_sub = 0.18510778609
    right_arm = baxter_interface.limb.Limb("right")
    right_joint_names = right_arm.joint_names()
    K = 0.1
    right = baxter_interface.Limb('right').endpoint_pose()
    position = right['position']
    quaternion = right['orientation']
    quaternion = np.array([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
    XE = np.array([position[0],position[1],position[2],euler[0],euler[1],euler[2]])
    print('XE',XE)
 
    XT = np.array([X_sub, Y_sub, (Z_sub + 0.05), -0.785745835105283, -0.1115831008789236, -3.1365214718905774])
    '''
    GU = np.array([[K*(XE[0]-XT[0])],
                   [K*(XE[1]-XT[1])],
                   [K*(XE[2]-XT[2])],
                   [K*(XE[3]-XT[3])],
                   [K*(XE[4]-XT[4])],
                   [K*(XE[5]-XT[5])]])
    '''
    GU = np.array([[0],
                   [0],
                   [0],
                   [0],
                   [0],
                   [0]])
 
    DX = XE
 
    #Homo = HomogeneousSolution
 
    #JacobianInverse = kdl.jacobian_pseudo_inverse()
 
    Jacobian = kdl.jacobian()
 
    print('Jacobian',Jacobian)
 
    L2 = 0.36435
    L3 = 0.069
    L4 = 0.37429
    Lh = math.sqrt(pow(L2,2)+pow(L3,2))
 
    S2 = math.sin(right_arm.joint_angle('right_s1'))
    C2 = math.cos(right_arm.joint_angle('right_s1'))
    S24 = math.sin(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
    C24 = math.cos(right_arm.joint_angle('right_s1')+right_arm.joint_angle('right_e1'))
 
    JacobianInverse = np.linalg.inv(np.array([[Jacobian[0,0],Jacobian[0,1],Jacobian[0,2],Jacobian[0,3],Jacobian[0,4],Jacobian[0,5],Jacobian[0,6]],
                                              [Jacobian[1,0],Jacobian[1,1],Jacobian[1,2],Jacobian[1,3],Jacobian[1,4],Jacobian[1,5],Jacobian[1,6]],
                                              [Jacobian[2,0],Jacobian[2,1],Jacobian[2,2],Jacobian[2,3],Jacobian[2,4],Jacobian[2,5],Jacobian[2,6]],
                                              [Jacobian[3,0],Jacobian[3,1],Jacobian[3,2],Jacobian[3,3],Jacobian[3,4],Jacobian[3,5],Jacobian[3,6]],
                                              [Jacobian[4,0],Jacobian[4,1],Jacobian[4,2],Jacobian[4,3],Jacobian[4,4],Jacobian[4,5],Jacobian[4,6]],
                                              [Jacobian[5,0],Jacobian[5,1],Jacobian[5,2],Jacobian[5,3],Jacobian[5,4],Jacobian[5,5],Jacobian[5,6]],
                                              [0,0,-Lh*S2-L4*S24,0,-L4*S24,0,0]]))
 
    print('JacobianInverse', JacobianInverse)
 
    JointVelocity = np.dot(JacobianInverse, GU)
 
    print('JointVelocity', JointVelocity)
 
    cmd = {'right_s0': JointVelocity[0], 'right_s1': JointVelocity[1], 'right_e0': JointVelocity[2], 'right_e1': JointVelocity[3], 'right_w0': JointVelocity[4], 'right_w1': JointVelocity[5], 'right_w2': JointVelocity[6]}
 
    #cmd = {'right_s0': 0.5, 'right_s1': -0.1, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0.5, 'right_w1': 0, 'right_w2': 0}
 
    print('cmd',cmd)
 
    right_arm.set_joint_velocities(cmd)
 
def simplejacobian():
    U = 1
    right_arm = baxter_interface.limb.Limb("right")
    #right_arm.move_to_neutral()
    jointpositions = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}
    right_arm.move_to_joint_positions(jointpositions)
    while U > 0.021:
        X_sub = 0.858318633316
        #X_sub = 0.70
        Y_sub = -0.0443270839514
        Z_sub = 0.18510778609
        K = 1
        right = baxter_interface.Limb('right').endpoint_pose()
        position = right['position']
        XE = np.array([position[0],position[1],position[2]])
        print('XE',XE)
 
        XT = np.array([X_sub, Y_sub, (Z_sub + 0.05)])
 
 
        GU = np.array([[K*(XE[0]-XT[0])],
                       [K*(XE[1]-XT[1])],
                       [K*(XE[2]-XT[2])]])
        '''
        GU = np.array([[0],
                       [-0.005],
                       [0]])
 
        GU = np.array([[K*(XE[0]-XT[0])],
                       [K*(XE[1]-XT[1])],
                       [K*(XE[2]-XT[2])],
                       [0],
                       [0],
                       [0],
                       [0]])
 
        GU = np.array([[0],
                       [0],
                       [1],
                       [0],
                       [0],
                       [0],
                       [0]])
        '''
 
 
        U = pow(XE[0]-XT[0],2)+pow(XE[1]-XT[1],2)+pow(XE[2]-XT[2],2)
        print('U',U)
 
        S1 = math.sin(right_arm.joint_angle('right_s0'))
        C1 = math.cos(right_arm.joint_angle('right_s0'))
        S2 = math.sin(right_arm.joint_angle('right_s1'))
        C2 = math.cos(right_arm.joint_angle('right_s1'))
        S3 = math.sin(right_arm.joint_angle('right_e0'))
        C3 = math.cos(right_arm.joint_angle('right_e0'))
        S4 = math.sin(right_arm.joint_angle('right_e1'))
        C4 = math.cos(right_arm.joint_angle('right_e1'))
        S5 = math.sin(right_arm.joint_angle('right_w0'))
        C5 = math.cos(right_arm.joint_angle('right_w0'))
        S6 = math.sin(right_arm.joint_angle('right_w1'))
        C6 = math.cos(right_arm.joint_angle('right_w1'))
 
        L0 = 0.27035
        L1 = 0.069
        L2 = 0.36435
        L3 = 0.069
        L4 = 0.37429
        L5 = 0.01
        L6 = 0.3683
        L = 0.278
        h = 0.064
        H = 1.104
 
 
        X = L1*C1+L2*C1*C2-L3*(S1*S3+C1*S2*C3)-L4*((S1*S3+C1*S2*C3)*S4-C1*C2*C4)-L5*((S1*C3-C1*S2*S3)*S5+((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C5)
        X1 = -L1*S1 - L2*S1*C2 - L3*(C1*S3-S1*S2*C3) - L4*((C1*S3 - S1*S2*C3)*S4 + S1*C2*C4) - L5*((C1*C3 + S1*S2*S3)*S5 + ((C1*S3 - S1*S2*C2)*C4 -S1*C2*S4)*C5)
        X2 = -L2*C1*S2 -L3*C1*C2*C3 -L4*(C1*C2*C3*S4 +C1*S2*C4) - L5*((-C1*C2*S3*S5) + ((-S1*C2*C3*C4+S1*S2*S4)*C5))
        X3 = -L3*(S1*C3-C1*S2*S3)-L4*(S1*C3-C1*S2*S3)*S4-L5*((-S1*S3-C1*S2*C3)*S5+((S1*C3-C1*S2*S3)*C4)*C5)
        X4 = -L4*((S1*S3+C1*S2*C3)*C4+C1*C2*S4)-L5*(-(S1*S3+C1*S2*C3)*S4+C1*C2*C4)*C5
        X5 = -L5*((S1*C3-C1*S2*S3)*C5-((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*S5)
 
        Y = L1*S1+L2*S1*C2+L3*(C1*S3-S1*S2*C3)+L4*((C1*S3-S1*S2*C3)*S4+S1*C2*C4)+L5*((C1*C3+S1*S2*S3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)
        Y1 = L1*C1+L2*C1*C2+L3*(-S1*S3-C1*S2*C3)+L4*((-S1*S3-C1*S2*C3)*S4+C1*C2*C4)+L5*((-S1*C3+C1*S2*S3)*S5+((-S1*S3-C1*S2*C3)*C4-C1*C2*S4)*C5)
        Y2 = -L2*S1*S2+L3*(-C1*C2*C3)+L4*(-S1*C2*C3*S4-S1*S2*C4)+L5*(S1*C2*S3*S5+(-S1*C2*C3*C4+S1*S2*S4)*C5)
        Y3 = L3*(C1*C3+S1*S2*S3)+L4*((C1*C3+S1+S3+S3)*S4)+L5*((-C1*S3+S1*S2*C3)*S5+(C1*C3+S1*S2*S3)*C4*C5)
        Y4 = L4*((C1*S3-S1*S2*C3)*C4-S1*C2*S4)+L5*((-(C1*S3-S1*S2*C3)*S4-S1*C2*C4)*C5)
        Y5 = L5*((C1*C3+S1*S2*S3)*C5-((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*S5)
 
        Z = -L2*S2-L3*C2*C3-L4*(S2*C4+C2*C3*S4)+L5*((S2*S4-C2*C3*C4)*C5+C2*S3*S5)
        Z1 = 0
        Z2 = -L2*C2+L3*S2*C3-L4*(C2*C4-S2*C3*S4)+L5*((C2*S4+S2*C3*C4)*C5-S2*S3*S5)
        Z3 = L3*C2*S3+L4*C2*S3*S4+L5*(C2*S3*C4*C5+C2*C3*S5)
        Z4 = -L4*(-S2*S4+C2*C3*C4)+L5*(S2*C4+C2*C3*S4)*C5
        Z5 = L5*(-(S2*S4-C2*C3*C4)*S5+C2*S3*C5)
 
        Xadd = L6*0.7071*(-((S1*S3+C1*S2*C3)*S4-C1*C2*C4)*C6-((S1*C3-C1*S2*C3)*S5+((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C5)*S6-((C1*S3-S1*S2*C3)*S4+S1*C2*C4)*C6-((C1*C3+S1*S2*S3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*S6)
        Xa1 = L6*0.7071*(-((C1*S3-S1*S2*C3)*S4+S1*C2*C4)*C6-((C1*C3+S1*S2*C3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*S6-((-S1*S3-C1*S2*C3)*S4+C1*C2*C4)*C6-((-S1*C3+C1*S2*S3)*S5+((-S1*S3-C1*S2*C3)*C4-C1*C2*S4)*C5)*S6)
        Xa2 = L6*0.7071*(-((-S1*S2*C3)*S4+C1*S2*C4)*C6-((-C1*C2*C3)*S5+((C1*C2*C3)*C4-C1*S2*S4)*C5)*S6-((-S1*C2*C3)*S4-S1*S2*C4)*C6-((S1*C2*S3)*S5+((-S1*C2*C3)*C4+S1*S2*S4)*C5)*S6)
        Xa3 = L6*0.7071*(-((S1*C3-C1*S2*S3)*S4)*C6-((-S1*S3+C1*S2*S3)*S5+((S1*C3-C1*S2*S3)*C4)*C5)*S6-((C1*C3+S1*S2*S3)*S4)*C6-((-C1*S3+S1*S2*C3)*S5+((C1*C3+S1*S2*S3)*C4)*C5)*S6)
        Xa4 = L6*0.7071*(-((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C6-((-1*(S1*S3+C1*S2*C3)*S4+C1*C2*C4)*C5)*S6-((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C6-((-1*(C1*S3-S1*S2*C3)*S4-S1*C2*C4)*C5)*S6)
        Xa5 = L6*0.7071*(-((S1*C3-C1*S2*C3)*C5-((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*S5)*S6-((C1*C3+S1*S2*S3)*C5-((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*S5)*S6)
        Xa6 = L6*0.7071*(((S1*S3+C1*S2*C3)*S4-C1*C2*C4)*S6+((S1*C3-C1*S2*C3)*S5+((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C5)*C6+((C1*S3-S1*S2*C3)*S4+S1*C2*C4)*S6-((C1*C3+S1*S2*S3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*C6)
 
        Yadd = L6*0.7071*(-((S1*S3+C1*S2*C3)*S4-C1*C2*C4)*C6-((S1*C3-C1*S2*C3)*S5+((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C5)*S6+((C1*S3-S1*S2*C3)*S4+S1*C2*C4)*C6+((C1*C3+S1*S2*S3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*S6)
        Ya1 = L6*0.7071*(-((C1*S3-S1*S2*C3)*S4+S1*C2*C4)*C6-((C1*C3+S1*S2*C3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*S6+((-S1*S3-C1*S2*C3)*S4+C1*C2*C4)*C6+((-S1*C3+C1*S2*S3)*S5+((-S1*S3-C1*S2*C3)*C4-C1*C2*S4)*C5)*S6)
        Ya2 = L6*0.7071*(-((C1*C2*C3)*S4+C1*S2*C4)*C6-((-C1*C2*C3)*S5+((C1*C2*C3)*C4-C1*S2*S4)*C5)*S6+((-S1*C2*C3)*S4-S1*C2*C4)*C6+((S1*C2*S3)*S5+((-S1*C2*C3)*C4+S1*S2*S4)*C5)*S6)
        Ya3 = L6*0.7071*(-((S1*C3-C1*S2*S3)*S4)*C6-((-S1*S3+C1*S2*S3)*S5+((S1*C3-C1*S2*S3)*C4)*C5)*S6+((C1*C3+S1*S2*S3)*S4)*C6+((-C1*S3+S1*S2*C3)*S5+((C1*C3+S1*S2*S3)*C4)*C5)*S6)
        Ya4 = L6*0.7071*(-((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C6-((-1*(S1*S3+C1*S2*C3)*S4+C1*C2*C4)*C5)*S6+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C6+((-1*(C1*S3-S1*S2*C3)*S4-S1*C2*C4)*C5)*S6)
        Ya5 = L6*0.7071*(-((S1*C3-C1*S2*C3)*C5-((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*S5)*S6+((C1*C3+S1*S2*S3)*C5-((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*S5)*S6)
        Ya6 = L6*0.7071*(((S1*S3+C1*S2*C3)*S4-C1*C2*C4)*S6-((S1*C3-C1*S2*C3)*S5+((S1*S3+C1*S2*C3)*C4+C1*C2*S4)*C5)*C6+(-1*(C1*S3-S1*S2*C3)*S4+S1*C2*C4)*S6+((C1*C3+S1*S2*S3)*S5+((C1*S3-S1*S2*C3)*C4-S1*C2*S4)*C5)*C6)
 
        Zadd = -L6*(-(S2*C4+C2*C3*S4)*C6+((S2*S4-C2*C3*C4)*C5+C2*S3*S5)*S6)
        Za1 = 0
        Za2 = -L6*(-(C2*C4-S2*C3*S4)*C6+((C2*S4+S2*C3*C4)*C5-S2*S3*S5)*S6)
        Za3 = -L6*((C2*S3*S4)*C6+((C2*S3*C4)*C5+C2*C3*S5)*S6)
        Za4 = -L6*(-(-S2*S4+C2*C3*C4)*C6+((S2*C4+C2*C3*S4)*C5)*S6)
        Za5 = -L6*((-1*(S2*S4-C2*C3*C4)*S5+C2*S3*C5)*S6)
        Za6 = -L6*((S2*C4+C2*C3*S4)*S6+((S2*S4-C2*C3*C4)*C5+C2*S3*S5)*C6)
 
        Xnew = Xadd-0.7071*(X-Y)-L
        Ynew = Yadd-0.7071*(X+Y)-h
        Znew = Zadd+Z+L0+H
 
        jacobian = np.array([[Xa1-0.7071*(X1-Y1),Xa2-0.7071*(X2-Y2),Xa3-0.7071*(X3-Y3),Xa4-0.7071*(X4-Y4),Xa5-0.7071*(X5-Y5),Xa6],
                             [Ya1-0.7071*(X1+Y1),Ya2-0.7071*(X2+Y2),Ya3-0.7071*(X3+Y3),Ya4-0.7071*(X4+Y4),Ya5-0.7071*(X5+Y5),Ya6],
                             [Za1+Z1,Za2+Z2,Za3+Z3,Za4+Z4,Za5+Z5,Za6]])
        print('jacobian',jacobian)
        #pseudo_inverse
        #inversejacobian = baxter_pykdl.baxter_kinematics('right').jacobian_pseudo_inverse()
 
        #simple transformation jacobian
        inversejacobian = np.dot(np.matrix.transpose(jacobian), np.linalg.inv(np.dot(jacobian, np.matrix.transpose(jacobian))))
 
        #extend jacobain
        #inversejacobian = extend_jacobian()
 
        print('inversejacobian', inversejacobian)
 
        JointVelocity = np.dot(inversejacobian, GU)
 
        print('JointVelocity', JointVelocity)
 
 
        cmd = {'right_s0': JointVelocity[0], 'right_s1': JointVelocity[1], 'right_e0': JointVelocity[2], 'right_e1': JointVelocity[3], 'right_w0': JointVelocity[4], 'right_w1': JointVelocity[5], 'right_w2': 0}
 
        #cmd = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}
 
        print('cmd',cmd)
 
        right_arm.set_joint_velocities(cmd)
        print('joint_angles',right_arm.joint_angles())
    right_arm.exit_control_mode()
    print('Exit the velocity mode')
 
def rightendpose():
    right = baxter_interface.Limb('right').endpoint_pose()
    position = right['position']
    quaternion = right['orientation']
    quaternion = np.array([quaternion[3],quaternion[0],quaternion[1],quaternion[2]])
    euler = tf.transformations.euler_from_quaternion(quaternion, axes='sxyz')
    return(np.array([position[0],position[1],position[2],euler[0],euler[1],euler[2]]))
 
def positioncontrol(TargetPosition):
    kdl = baxter_pykdl.baxter_kinematics('right')
    right_arm = baxter_interface.limb.Limb("right")
    right_arm.set_joint_position_speed(0.5)
    #right_arm.move_to_neutral()
    #jointpositions = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}
    #right_arm.move_to_joint_positions(jointpositions)
    rate = rospy.Rate(10)
 
    XT = np.array([TargetPosition[0], TargetPosition[1], (TargetPosition[2] + 0.05), -0.785745835105283, -0.1115831008789236, -3.1365214718905774])
    XE = rightendpose()
 
    GU = np.array([[-(XE[0]-XT[0])/abs(XE[2]-XT[2])],
                   [-1.5*(XE[1]-XT[1])/abs(XE[2]-XT[2])],
                   [-1],
                   [0],
                   [0],
                   [0]])
    '''
    GU = np.array([[0],
                   [0],
                   [0.1],
                   [0],
                   [0],
                   [0]])
    '''
 
    timeout = time.time() + 2
    # For tost 2 table 6
    # Y coffection 1.3 for table
    while True:
        print('-------------')
        #XE = rightendpose()
        U = pow(XE[0]-XT[0],2)+pow(XE[1]-XT[1],2)+pow(XE[2]-XT[2],2)
        joint_angles = right_arm.joint_angles()
        JacobianInverse = kdl.jacobian_pseudo_inverse()
        print('JacobianInverse', JacobianInverse)
        delta = 0.01*np.dot(JacobianInverse, GU)
        deltajointanglesdictionary = {'right_s0': delta[0], 'right_s1': delta[1], 'right_e0': delta[2], 'right_e1': delta[3], 'right_w0': delta[4], 'right_w1': delta[5], 'right_w2': delta[6]}
        print('joint_angles',joint_angles)
        print('adding',deltajointanglesdictionary)
        #joint_angles = dict(Counter(joint_angles)+Counter(deltajointanglesdictionary))
        #joint_angles = dict(joint_angles.items()+deltajointanglesdictionary.items())
 
        joint_angles['right_s0'] += delta[0]
        joint_angles['right_s1'] += delta[1]
        joint_angles['right_e0'] += delta[2]
        joint_angles['right_e1'] += delta[3]
        joint_angles['right_w0'] += delta[4]
        joint_angles['right_w1'] += delta[5]
        joint_angles['right_w2'] += delta[6]
 
        print('joint_angles',joint_angles)
 
        right_arm.set_joint_positions(joint_angles,raw=False)
        rate.sleep()
 
        if time.time() > timeout:
            break
 
def ik_test():
    right = baxter_interface.Limb('right').endpoint_pose()
    position = right['position']
    quaternion = right['orientation']
    ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                ),
                orientation=Quaternion(
                    x=quaternion[0],
                    y=quaternion[1],
                    z=quaternion[2],
                    w=quaternion[3],
                ),
            ),
        ),
    }
 
    ikreq.pose_stamp.append(poses['right'])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
 
    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    joint_angles = {'right_s0': limb_joints['right_s0'], 'right_s1': limb_joints['right_s1'], 'right_w0': limb_joints['right_w0'], 'right_w1': limb_joints['right_w1'], 'right_w2': limb_joints['right_w2'], 'right_e0': limb_joints['right_e0'], 'right_e1': limb_joints['right_e1']}
 
    return joint_angles
 
def TestVision(TargetPosition):
    right_arm = baxter_interface.Limb('right')
    ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=TargetPosition[0],
                    y=TargetPosition[1],
                    z=TargetPosition[2],
                ),
                orientation=Quaternion(
                    x=-0.377481491084,
                    y=0.924451532113,
                    z=0.0193834993405,
                    w=0.0193834993405,
                ),
            ),
        ),
    }
 
    ikreq.pose_stamp.append(poses['right'])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
 
    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
 
    joint_angles = {'right_s0': limb_joints['right_s0'], 'right_s1': limb_joints['right_s1'], 'right_w0': limb_joints['right_w0'], 'right_w1': limb_joints['right_w1'], 'right_w2': limb_joints['right_w2'], 'right_e0': limb_joints['right_e0'], 'right_e1': limb_joints['right_e1']}
 
    print('Start moving')
    right_arm.move_to_joint_positions(joint_angles)
    print('finished')
 
 
def main(args):
    rospy.init_node('Motion', anonymous=True)
    RightGripper = baxter_interface.Gripper('right')
    rospy.Subscriber('/coordinate_real', Point, callback)
    print(real)
 
    data = np.array([0.724036966537,-0.0552060635377,0.144624583263])
    #RightGripper.open()
    positioncontrol(data)
    #simplejacobian()
    #TestVision(data)
    RightGripper.close()
 
if __name__ == '__main__':
    main(sys.argv)
