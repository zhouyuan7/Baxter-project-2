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
    
    right_arm.move_to_neutral()
    left_arm.move_to_neutral()
 
if __name__ == '__main__':
    main(sys.argv)


