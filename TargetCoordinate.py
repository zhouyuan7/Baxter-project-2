#!/usr/bin/env python
import rospy
import baxter_interface
import math
import sys
import tf
import numpy as np
from geometry_msgs.msg import (Pose2D, Point)
 
'''
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800
'''
 
L_sub = []
R_sub = []

IL = np.array([[402.645782714, 0.0, 641.443999011, 0.0],
               [0.0, 402.645782714, 369.204392796, 0.0],
               [0.0, 0.0, 1.0, 0.0]])
'''
 
IL = np.array([[402.645782714, 0.0, 641.443999011, 0.0],
               [0.0, -402.645782714, -369.204392796, 0.0],
               [0.0, 0.0, 1.0, 0.0]])
'''
IR = np.array([[405.605299893, 0.0, 653.2303814899, 0.0],
               [0.0, 405.605299893, 357.669514705, 0.0],
               [0.0, 0.0, 1.0, 0.0]])
'''
 
IR = np.array([[405.605299893, 0.0, 653.2303814899, 0.0],
               [0.0, -405.605299893, -357.669514705, 0.0],
               [0.0, 0.0, 1.0, 0.0]])
'''
KL = np.array([0.0155944702647, -0.0505694105742, 0.000976841736257, -0.00106528361331, 0.0129621223372])
 
KR = np.array([0.0195748525532, -0.0538296329161, 0.000369008213963, -0.000577152591598, 0.0137315607859])
 
 
def L_sub(data):
    global L_sub
    L_sub=np.array([[data.x],[data.y]])
 
def R_sub(data):
    global R_sub
    R_sub=np.array([[data.x],[data.y]])
 
def undistort(x, y, I, K):
    x = (x - I[0,0])/I[0,2]
    y = (y - I[1,1])/I[1,2]
 
    r2 = x*x + y*y
 
    """Radial distorsion"""
    xUD = x * (1 + K[0]*r2 + K[1]*r2*r2 + K[4]*r2*r2*r2)
    yUD = y * (1 + K[0]*r2 + K[1]*r2*r2 + K[4]*r2*r2*r2)
 
    """Tangential distortion"""
    xUD = xUD + (2*K[2]*x*y + K[3]*(r2 + 2*x*x))
    yUD = yUD + (2*K[3]*x*y + K[2]*(r2 + 2*x*x))
 
    xUD = xUD * I[0,2] + I[0,0]
    yUD = yUD * I[1,2] + I[1,1]
 
    return xUD, yUD
 
def projection(position, quaternion, I):
    '''
    rotation = tf.transformations.quaternion_matrix(quaternion)
    rotation = np.array([[rotation[0,0], rotation[0,1], rotation[0,2]],
                         [rotation[1,0], rotation[1,1], rotation[1,2]],
                         [rotation[2,0], rotation[2,1], rotation[2,2]]])
    '''
    q = quaternion
    '''
    rotation = np.array([[1-2*(q[2]*q[2]+q[3]*q[3]),2*(q[1]*q[2]-q[0]*q[3]),2*(q[1]*q[3]+q[0]*q[2])],
                         [-2*(q[1]*q[2]+q[0]*q[3]),-1*(1-2*(q[1]*q[1]+q[3]*q[3])),-2*(q[2]*q[3]-q[0]*q[1])],
                         [2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])]])
    '''
    rotation = np.array([[1-2*(q[2]*q[2]+q[3]*q[3]),2*(q[1]*q[2]-q[0]*q[3]),2*(q[1]*q[3]+q[0]*q[2])],
                         [2*(q[1]*q[2]+q[0]*q[3]),1-2*(q[1]*q[1]+q[3]*q[3]),2*(q[2]*q[3]-q[0]*q[1])],
                         [2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])]])
    #rotation = np.transpose(rotation)
    
     
    POSITION = np.dot(-1*rotation, position)
 
    E = np.array([[rotation[0,0], rotation[0,1], rotation[0,2], POSITION[0]],
                  [rotation[1,0], rotation[1,1], rotation[1,2], POSITION[1]],
                  [rotation[2,0], rotation[2,1], rotation[2,2], POSITION[2]],
                  [0, 0, 0, 1]])
    P = np.dot(I, E)
    #quaternion = [quaternion[1],quaternion[2],quaternion[3],quaternion[0]]
    #E = tf.fromTranslationRotation(position,quaternion)
    #P = np.dot(I, E)
    return P
 
def triangulation_linear_test():
    PL = projection_matrix_left()
    PR = projection_matrix_right()
    X_true=[0.6113066337386801, -0.0011079176139375331, 0.11206701705119819];
    xl=project(PL,X_true);
    print('TestXL_sub', xl)
    xr=project(PR,X_true);
    print('TestXR_sub', xr)
    X_estimated=triangulation_linear(PL,PR,xl,xr)
    print X_true, X_estimated
 
def project(P,X):
    x=np.dot(P, [X[0],X[1],X[2],1])
    x=[x[0]/x[2],x[1]/x[2]]
    return x
 
def projectr(X):
    """Project a 3-D point X to right camera frame"""
    limb = baxter_interface.Limb('right').endpoint_pose()
    position = limb['position']
    quaternion = limb['orientation']
    quaternion = [quaternion[3],quaternion[0],quaternion[1],quaternion[2]]
    PR = projection(position, quaternion, IR)
    x=project(PR,X)
    return x
 
def projectl(X):
    """Project a 3-D point X to left camera frame"""
    limb = baxter_interface.Limb('left').endpoint_pose()
    position = limb['position']
    quaternion = limb['orientation']
    quaternion = [quaternion[3],quaternion[0],quaternion[1],quaternion[2]]
    PL = projection(position, quaternion, IL)
    x=project(PL,X)
    return x
 
def projection_matrix_left():
    left = baxter_interface.Limb('left').endpoint_pose()
    print('left',left)
    positionl = left['position']
    quaternionl = left['orientation']
    quaternionl = [quaternionl[3],quaternionl[0],quaternionl[1],quaternionl[2]]
    PL = projection(positionl, quaternionl, IL)
    return PL
 
def projection_matrix_right():
    right = baxter_interface.Limb('right').endpoint_pose()
    print('right',right)
    positionr = right['position']
    quaternionr = right['orientation']
    quaternionr = [quaternionr[3],quaternionr[0],quaternionr[1],quaternionr[2]]
    PR = projection(positionr, quaternionr, IR)
    return PR
 
#def xTriangulated=triangulation_linear(IL,IR,PL_sub,PR_sub):
def triangulation_linear(IL,IR,L_sub, R_sub):
    XL_sub = L_sub[0]
    YL_sub = L_sub[1]
    XR_sub = R_sub[0]
    YR_sub = R_sub[1]
    #XL_sub, YL_sub = undistort(XL_sub, YL_sub, IL, KL)
    #XR_sub, YR_sub = undistort(XR_sub, YR_sub, IR, KR)
    PL = projection_matrix_left()
    #print('PL', PL)
    #print('ER', ER)
    PR = projection_matrix_right()
    #print('PR', PR)
    #pixelL = np.array([XL_sub,YL_sub])
    #print('pixelL', pixelL)
    #pixelR = np.array([XR_sub,YR_sub])
    #print('pixelR', pixelR)
    D = np.array([[XL_sub*PL[2, :] - PL[0, :]],
                  [YL_sub*PL[2, :] - PL[1, :]],
                  [XR_sub*PR[2, :] - PR[0, :]],
                  [YR_sub*PR[2, :] - PR[1, :]]])
 
    D = np.array([[D[0,0,0], D[0,0,1], D[0,0,2], D[0,0,3]],
                  [D[1,0,0], D[1,0,1], D[1,0,2], D[1,0,3]],
                  [D[2,0,0], D[2,0,1], D[2,0,2], D[2,0,3]],
                  [D[3,0,0], D[3,0,1], D[3,0,2], D[3,0,3]]])
    #print('______')
    print('D', D)
    #d = np.diag(np.reciprocal(np.amax(np.absolute(D), axis=1)))
    #print('d', d)
    #Q = D.transpose()*D
    #print('Q', Q)
    #D = np.dot(D, d)
 
    u, s, vh = np.linalg.svd(D, full_matrices=True)
    #print('u', u)
 
    print('s', s)
 
    print('vh', vh)
 
    XY = np.array(vh[3,0:4])
    #XY = np.dot(d,XY)
    XY = [XY[0]/XY[3],XY[1]/XY[3],XY[2]/XY[3]]
    #XY = [XY[0],XY[1],XY[2]]
    return XY
 
def opticoordinate():
    leftjointangles = baxter_interface.Limb('left').joint_angles()
    leftjointangles['left_w2'] = leftjointangles['left_w2'] + 0.78539816339
    baxter_interface.Limb('left').set_joint_positions(leftjointangles, raw=False)
 
def main(args):
    rospy.init_node('Coordinate', anonymous=True)
    point_coo = rospy.Publisher('/coordinate_real', Point, latch=True, queue_size=10)
    backprojectl = rospy.Publisher('/backprojectl', Pose2D, latch=True, queue_size=10)
    backprojectr = rospy.Publisher('/backprojectr', Pose2D, latch=True, queue_size=10)
    rospy.Subscriber('/Pcoordinate_left', Pose2D, L_sub)
    rospy.Subscriber('/Pcoordinate_right', Pose2D, R_sub)
    while not rospy.is_shutdown():
        PL = projection_matrix_left()
        PR = projection_matrix_right()
        XY = triangulation_linear(IL,IR, L_sub, R_sub)
        print('real target coordinate',XY)
        print('left pixel coordinate', L_sub)
        pixellb = projectl(XY)
        #pixellb = [pixellb[0],pixellb[1]]
        global backprojectl
        backprojectl.publish(pixellb[0], pixellb[1], 0)
        Deltal = np.power(pixellb[0]-L_sub[0],2)+np.power(pixellb[1]-L_sub[1],2)
        print('back projection left', pixellb)
        print('right pixel coordinate', R_sub)
        pixelrb = projectr(XY)
        #pixelrb = [pixelrb[0],pixelrb[1]]
        global backprojectr
        backprojectr.publish(pixelrb[0], pixelrb[1], 0)
        Deltar = np.power(pixelrb[0]-R_sub[0],2)+np.power(pixelrb[1]-R_sub[1],2)
        print('back projection right', pixelrb)
        print('translation1', XY)
        global point_coo
        point_coo.publish(XY[0],XY[1],XY[2])
        print('TEST_________')
        triangulation_linear_test()
        print('_____________')
        print('_____________')
        print('_____________')
        print('_____________')
'''
        if Deltal < 1800 and Deltar < 1800:
            print('Optimizaiton finished')
            global point_coo
            point_coo.publish(XY[0],XY[1],XY[2])
        else:
            opticoordinate()
            print('Optimize the oritnetation')
'''
 
if __name__ == '__main__':
    main(sys.argv)
