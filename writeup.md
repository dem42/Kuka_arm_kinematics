## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

### Writeup 

The pick & place project involved figuring out the closed form solution of the inverse kinematics for the Kuka210 robot arm. The goal of the inverse kinematics problem is to compute the joint variables of the kuka arm using only the the position and orientation of the end-effector reference frame. These six parameters are passed to the `calculate_ik` ROS service by the `Moveit!` planner.

To be able to implement the inverse kinematics solver, I first needed to construct individual forward kinematics transformation matrices using Denavit-Hartenberg (DH) parametrization. I did this with the help of the lecture videos to place the origins of the reference frames for each joints. Then I double checked the values of the DH parameters `d, a, alpha, theta` using the xarco urdf file. In the urdf file the joint reference frames do not match the ones of ones build using the DH parameterization, but the values of the DH parameters can be computed simply by adding up the values from the respective frames in the urdf. An example of how I evaluated the urdf file to get the joint parameters is in section Kinematic Analysis.

After, constructing the DH parametrization and getting the individual transformation matrices for each joint, the other task was to construct a homogeneous transform between the base link and the end-effector using just the end-effector pose. This can be done since the end-effector reference frame can be considered a rigid body in 3D space and thus we can use Euler angles to compute the rotation using the Euler angle formula for intrinsic rotation:

$$R_{roll,pitch,yaw} = R_{yaw} * R_{pitch} * R_{roll}$$ 

The matrix of the transform can be found at the bottom of point #2 of section Kinematic analysis. 

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To go from joints defined in urdf to the values of the DH parameters I needed to add up the values found in the urdf file. 
For example `joint_1` of urdf has `<origin xyz="0 0 0.33" rpy="0 0 0"/>` and `joint_2` of urdf has `<origin xyz="0.35 0 0.42" rpy="0 0 0"/>`. The DH parametrization `dh_joint_1` has its origin at the same `z` distance as urdf `join_2` so therefore the `d1` value of `dh_joint_1` is the sum of the `z` offsets and is thus `0.33 + 0.42 = 0.75`. 

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The modified DH parameters for the Kuka210 arm are:

Joint | $$\alpha_{i-1}$$ | $$a_{i-1}$$ | $$d_i$$ | $$\theta_i$$ |
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | 0
2 | $$-\pi/2$$ | 0.35 | 0 | $$\theta_2 -\pi/2$$
3 | 0 | 1.25 | 0 | 0
4 | $$-\pi/2$$ | -0.054 | 1.50 | 0
5 | $$\pi/2$$ | 0 | 0 | 0
6 | $$-\pi/2$$ | 0 | 0 | 0
G | 0 | 0 | 0.303 | 0


The individual DH transformation matrices per each joint are:
```
⎡cos(q₁)  -sin(q₁)  0   0  ⎤
⎢                          ⎥
⎢sin(q₁)  cos(q₁)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1  0.75⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦
```
```
⎡sin(q₂)  cos(q₂)   0  0.35⎤
⎢                          ⎥
⎢   0        0      1   0  ⎥
⎢                          ⎥
⎢cos(q₂)  -sin(q₂)  0   0  ⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦
```
```
⎡cos(q₃)  -sin(q₃)  0  1.25⎤
⎢                          ⎥
⎢sin(q₃)  cos(q₃)   0   0  ⎥
⎢                          ⎥
⎢   0        0      1   0  ⎥
⎢                          ⎥
⎣   0        0      0   1  ⎦
```
```
⎡cos(q₄)   -sin(q₄)  0  -0.054⎤
⎢                             ⎥
⎢   0         0      1   1.5  ⎥
⎢                             ⎥
⎢-sin(q₄)  -cos(q₄)  0    0   ⎥
⎢                             ⎥
⎣   0         0      0    1   ⎦
```
```
⎡cos(q₅)  -sin(q₅)  0   0⎤
⎢                        ⎥
⎢   0        0      -1  0⎥
⎢                        ⎥
⎢sin(q₅)  cos(q₅)   0   0⎥
⎢                        ⎥
⎣   0        0      0   1⎦
```
```
⎡cos(q₆)   -sin(q₆)  0  0⎤
⎢                        ⎥
⎢   0         0      1  0⎥
⎢                        ⎥
⎢-sin(q₆)  -cos(q₆)  0  0⎥
⎢                        ⎥
⎣   0         0      0  1⎦
```
With the following being the transformation between joint 6 and the gripper reference frame (without a correction applied)
```
⎡1  0  0    0  ⎤
⎢              ⎥
⎢0  1  0    0  ⎥
⎢              ⎥
⎢0  0  1  0.303⎥
⎢              ⎥
⎣0  0  0    1  ⎦
```

Given an end-effector reference frame origin at `(px,py,pz)` with respect to the base link reference frame and an orientation of the end-effector reference frame of roll,pitch,yaw being `(a,b,c)`, then the total homogeneous transform between the base link and the end-effector is given by the following matrix:
```
⎡cos(b)⋅cos(c)  sin(a)⋅sin(b)⋅cos(c) - sin(c)⋅cos(a)  sin(a)⋅sin(c) + sin(b)⋅cos(a)⋅cos(c)     px⎤
⎢                                                                                             ⎥
⎢sin(c)⋅cos(b)  sin(a)⋅sin(b)⋅sin(c) + cos(a)⋅cos(c)  -sin(a)⋅cos(c) + sin(b)⋅sin(c)⋅cos(a)    py⎥
⎢                                                                                             ⎥
⎢   -sin(b)                sin(a)⋅cos(b)                          cos(a)⋅cos(b)              pz⎥
⎢                                                                                             ⎥
⎣      0                         0                                      0                   1 ⎦
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image! 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]