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
    right_arm.set_joint_position_speed(0.8)
    #right_arm.move_to_neutral()
    #jointpositions = {'right_s0': 0, 'right_s1': 0, 'right_e0': 0, 'right_e1': 0, 'right_w0': 0, 'right_w1': 0, 'right_w2': 0}
    #right_arm.move_to_joint_positions(jointpositions)
    rate = rospy.Rate(10)
 
    XT = np.array([TargetPosition[0], TargetPosition[1], (TargetPosition[2] + 0.05), -0.785745835105283, -0.1115831008789236, -3.1365214718905774])
    XE = rightendpose()
 
    GU = np.array([[-(XE[0]-XT[0])/abs(XE[2]-XT[2])],
                   [-(XE[1]-XT[1])/abs(XE[2]-XT[2])],
                   [-0.8],
                   [0],
                   [0],
                   [0]])
 
    timeout = time.time() + 3
    # on the tost 3 seconds X coffiecent 1 Y coffiecent 1
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
 
 
def main(args):
    rospy.init_node('Motion', anonymous=True)
    RightGripper = baxter_interface.Gripper('right')
    rospy.Subscriber('/coordinate_real', Point, callback)
    print(real)
 
    data = np.array([0.74159038956,-0.0730693560262,0.12202538316])
    positioncontrol(data)
 
if __name__ == '__main__':
    main(sys.argv)
