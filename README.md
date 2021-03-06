# Baxter-project-2

<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/table_own.gif"/>
</p>
<p align="center">
'position-velocity' control
</p>
<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/hand_baxter.gif"/>
</p>
<p align="center">
Position-inverse Kinematic Control
</p>

## Introduction

This is my second Baxter project and my first solo Baxter project. This is also the course project of ME 740 Vision, Robotics and Planning in Boston University. The propose of this project is to estimate a spatial position (translation) of a tennis ball and to use a position-velocity movement strategy generated by myself(partly) to let the gripper approaches to it.

This project can also be classified into a visual servoing control method. Compared with my first Baxter project, this is an off-line spatial target position estimation and position-velocity Jacobian control. In detail, this project can be divided into vision and motion parts.

### Vision

For vision part, the pattern recognition is the same as the first project using color separation method. However, the process of transforming the location information from camera pixel frame to Baxter coordinate is different. In this project i use both two arms' hand cameras and linear triangulation with single value decomposition method combining camera intrinsic matrix to estimate the target location in Baxter coordinate system. 

![alt text](https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/baxter_vision.png)
The above image is the spatial position estiamtion. The red points are the pattern recognition using color separation and the blue points are the back projecton check bsaed on our estimated spatial coordinate. The result is very good!

### Motion

For motion part, using plain language, now our Baxter opens its 'eyes' and can 'see' something. We want to use its arm to move to it. based on my thought, there are three methods: position-inverse kinematic control, velocity control and 'position-velocity' control.

#### Position-inverse Kinematic Control

Given target work space coordinate. Go through inverse kinematic process to get the relate configurations (joint angles). The core of this method is the inverse kinematic solver. Due to the project time limit, I cannot do any better than the default Baxter solver. One of the demo videos shows the performance of this method. 

#### Pure Velocity control

With a potential function based on the start point and estimated goal point, using analysis Jacobian matrix, this method does not need to solve inverse kinematic of the robot arm chain only forward kinematic needed. The way I first want to choose but faced an issue which is the gravity of the robot arm (Zero-Gravity mode).


#### Position-velocity control
For this project, I generate a control strategy between the above two levels of control. This control is actually still a position control because the output data still go through the moving joint angle API not joint velocity, but with a particular trajectory. The following one-line algorithm show the strategy.

<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/algorithm.png"/>
</p>

The whole movement process is divided into many tiny time steps. In each time step the difference of the joint angles between now and next time step is given by the Moore-Penrose pseudo-inverse of Jacobian times a given workspace trajectory velocity.

However, Jacobian of the Baxter arm chain is still needed. I try to do it  by myself using Baxter D-H parameter table, but due to time limit, I use Baxter provided python method (baxter_pykdl.baxter_kinematics(‘right’).jacobian) to compute the jacobian. 


<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/table_baxter.gif"/>
</p>
<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/hand_own.gif"/>
</p>
<p align="center">
<img src="https://github.com/zhouyuan7/Baxter-project-2/blob/master/source/simulation.gif"/>
</p>
