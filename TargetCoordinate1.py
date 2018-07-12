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
 
 
 
def main(args):
    rospy.init_node('Test', anonymous=True)
 
    right_arm = baxter_interface.limb.Limb("right")
    left_arm = baxter_interface.limb.Limb("left")
    #name: ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    #position: [-0.20862138715241627, 1.5075196192943903, -0.24658741165258025, -1.3222914391572267, -0.7148350471546028, 1.386718632248414, -0.09242234247009617, 0.5434126941084079, 1.1577719996565161, -0.1349903093339164, -1.1604564660353156, 0.56105347316909, 1.621034197597911, -0.02914563496982286]
 
    #rightjointpositions = {'right_s0': -0.1349903093339164, 'right_s1': -1.1604564660353156, 'right_e0': 0.5434126941084079, 'right_e1': 1.1577719996565161, 'right_w0': 0.56105347316909, 'right_w1': 1.621034197597911, 'right_w2': -0.02914563496982286}
    rightjointpositionstost = {'right_s0': -0.15569904997036949, 'right_s1': -1.2866263858388909, 'right_e0': 0.4563592843959106, 'right_e1': 1.4342720366728618, 'right_w0':0 , 'right_w1': 1.5984079809766012, 'right_w2': -0.08551942892461181}
    #0.6661311571392409


    #leftjointpositions = {'left_s0': -0.24658741165258025, 'left_s1': -1.3222914391572267, 'left_e0': -0.20862138715241627, 'left_e1': 1.5075196192943903, 'left_w0': -0.7148350471546028, 'left_w1': 1.386718632248414, 'left_w2': -0.09242234247009617}
    leftjointpositionstost = {'left_s0': -0.12655341500054662, 'left_s1': -1.37137882436956, 'left_e0': -0.13690778531877318, 'left_e1': 1.5121215616580466, 'left_w0': -0.8114758367913839, 'left_w1': 1.6110633224766557, 'left_w2': 0.010354370318226542}






    rightjointpositionstable = {'right_s0': -0.14687866044002837, 'right_s1': -1.349903093339164, 'right_e0': 0.46096122675956686, 'right_e1': 1.5075196192943903, 'right_w0':0.5426457037144651 , 'right_w1': 1.4891118498397653, 'right_w2': -0.0732475826215285}
    #w0 = 0.5426457037144651


    leftjointpositionstable = {'left_s0': -0.12310195822780445, 'left_s1': -1.3556555212937345, 'left_e0': -0.1449611844551716, 'left_e1': 1.5738642883704346, 'left_w0': -0.6929758209272356, 'left_w1': 1.4733885467639398, 'left_w2': -0.009587379924283835}
    right_arm.move_to_joint_positions(rightjointpositionstost)
    left_arm.move_to_joint_positions(leftjointpositionstost)
 
if __name__ == '__main__':
    main(sys.argv)


