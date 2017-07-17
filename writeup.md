## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/robot1-1.jpg
[image2]: ./misc_images/robot2-1.jpg
[image3]: ./misc_images/triangles0-1.jpg
[image4]: ./misc_images/triangles1-1.jpg
[image5]: ./misc_images/equations-1.jpg
[image6]: ./misc_images/equations2-1.jpg
[image7]: ./misc_images/equations3-1.jpg

### Writeup 

The pick & place project involved figuring out the closed form solution of the inverse kinematics for the Kuka210 robot arm. The goal of the inverse kinematics problem is to compute the joint variables of the kuka arm using only the the position and orientation of the end-effector reference frame. These parameters are passed to the `calculate_ik` ROS service by the `Moveit!` planner.

To be able to implement the inverse kinematics solver, I first needed to construct individual forward kinematics transformation matrices using the Denavit-Hartenberg (DH) parametrization. I did this with the help of the lecture videos to place the origins of the reference frames for each joint. Then I double checked the values of the DH parameters `d, a, alpha, theta` using the xarco urdf file. In the urdf file the joint reference frames do not match the ones obtained using the DH parameterization, but the values of the DH parameters can be computed simply by adding up the values from the respective frames in the urdf. An example of how I evaluated the urdf file to get the joint parameters is in point #1 of section Kinematic Analysis.

After, constructing the DH parametrization and getting the individual transformation matrices for each joint, the other task was to construct a homogeneous transform between the base link and the end-effector using just the end-effector pose. This can be done since the end-effector reference frame can be considered a rigid body in 3D space and thus we can use Euler angles to compute the rotation using the Euler angle formula for intrinsic rotation. The matrix of the transform can be found at the bottom of point #2 of section Kinematic analysis. 

The next step was to consider that a correction is needed because in my DH parameterization, the end-effector reference frame is not aligned with the reference frame in the urdf file. So I needed two correction rotations so that my reference frames align otherwise my end-effector z-axis would be where urdf expects the x-axis.

Finally, with the closed form solution of the inverse kinematics, the problem can be broken down into inverse position and inverse orientation sub-problems. In inverse position we compute the joint angles of the first three joints using the location of the wrist center (which is coincident with the origin of DH reference frames 4,5,6). I computed these joint values using law of cosine. 

The the inverse position is computed by computing a transformation matrix between base link and link three using the first three joints. The transpose of this transformation matrix can than be multiplied with the homogeneous matrix obtained using the end-effector pose. The result of this multiplication is gives us the transformation matrix between link three and the gripper frame but without variables. So we can match it with a DH matrix with variables for joint angles four, five and six. 

The in-depth form of these inverse position and inverse orientation computations is in point #3 of section Kinematic analysis.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To go from joints defined in urdf to the values of the DH parameters I needed to add up the values found in the urdf file.

For example `joint_1` of urdf has `<origin xyz="0 0 0.33" rpy="0 0 0"/>` and `joint_2` of urdf has `<origin xyz="0.35 0 0.42" rpy="0 0 0"/>`. The DH parametrization `dh_joint_1` has its origin at the same `z` distance as urdf `join_2` so therefore the `d1` value of `dh_joint_1` is the sum of the `z` offsets and is thus `0.33 + 0.42 = 0.75`. 

The $$\alpha$$ twist angles are obtained by checking if the reference frame axes of joints $$i-1$$ and $$i$$ are perpendicular or not. If they are then the $$\alpha$$ is $$\pi/2$$ either negative or positive based on the direction of rotation.

The reference frames that I chose for the DH parameterization of the Kuka210 arm are shown in the image below:
![alt text][image1]

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

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I computed the individual DH transformation matrices between joint $$i-1$$ and joint $$i$$ by pluging the parameters $$\alpha_{i-1}, a_{i-1}, d_i, \theta_i$$ into the DH transformation matrix from the lecture:
```
⎡   cos(q)         -sin(q)        0         a   ⎤
⎢                                               ⎥
⎢sin(q)⋅cos(α)  cos(α)⋅cos(q)  -sin(α)  -d⋅sin(α)⎥
⎢                                               ⎥
⎢sin(α)⋅sin(q)  sin(α)⋅cos(q)  cos(α)   d⋅cos(α) ⎥
⎢                                               ⎥
⎣      0              0           0         1   ⎦
```

The individual DH transformation matrices I got per each joint are:
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

This matrix was computed using

$$R_{roll,pitch,yaw} = R_{yaw} * R_{pitch} * R_{roll}$$ 

where the $$R$$ matrices are simple rotations around the z,y and x axis. I then concatenated the rotation matrix with the origin offset vector.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

My computation of the inverse position starts with computing the wrist center. For this I use the homogeneous transform computed from the end-effector position. However, this transformation needs a correction because the DH reference frame of the end effector has a different orienation. So I apply this correction and then compute the wrist location as:

$$W = P_{end-effector} - d_{7} * R_{roll,pitch,yaw} * R_{correction} * [0, 0, 1]$$

Using the wrist I can compute $$\theta_1$$ easily just by projecting $$W_{z}$$ to the x-y plane. Then $$theta_2$$ and $$theta_3$$ I compute using the law of cosines. I noticed that there is a triangle spanned by the joint origins two and three and the wrist joint. I can compute the angles of this triangle and they will be within $$(0,180)$$ degrees. Next I can compute the signed angle $$\gamma_1$$ using `atan2` and that gives me the formulas for $$theta_2$$ and $$theta_3$$ as follows:

The following image shows how I obtain the projections of the triangles to compute the joint angles
![alt text][image3]

The equations I use to compute the first three joint angles are given in the below image:
![alt text][image5]

Using the previous intermediate values I compute the second and thrid joint angle as follows, using the fast that the base, 0 angle configuration is 90 degrees in both cases:
![alt text][image6]

In the next image I show an example of the triangle in the default configuration with all the angles set to 0 and an example where there are multiple solutions. By always adding $$\gamma1$$ and $$\gamma2$$ I implicitly always choose the solution with the arm going up.
![alt text][image4]

The computation of inverse orientation was explained in the lectures, so I just follow that technique of equating the total, corrected DH rotation with the rotation obtained using the end-effector pose because they must match. Then I can just multiply from the left using the inverse of the rotation $R0_3$ (or the transpose since this is a rotation matrix). The result is a matrix which I can solve by extracing the euler angles using the $$atan2$$ trick where I note that $$tan(a) = sin(a) / cos(a)$$.

$$R3_6 = R0_3^{T} * R_{roll,pitch,yaw}$$

The computation of the last three joint angles is done by matching the LHS to the RHS of the previous equation. The LHS is given by a matrix with variables and the RHS is a matrix of just numbers. The following image shows the LHS matrix and the computation of the three angles from this matrix:
![alt text][image7]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

In my implementation I noticed that the speed of the computation was a big issue. Initially, using full homogenenous transformations and computing all of the matrices, I noticed that a single inverse computation would take several minutes at best. So I decided to heavily optimize the code and drop any unnecessary matrix multiplications. In the end I only need DH rotations for joints one, two and three, so I avoid computing anything else. This and also not using symbols for the inverse position computation allowed me to reduce the runtime of a single inverse kinematics computation to around half a second.

In the final code I added the optional parameter `check_error` to `handle_calculate_IK`. If this is set to `true` then the code will also compute the forward kinematics after it finishes the inverse kinematics and compute the distance between the two six-component parameter vectors. It performs quickly because it uses a stored DH transformation matrix, which I serialize using the library `pickle`. I noticed that my errors are extremely tiny, on the magnitude of `1e-15` so I decided not to graph them. I assume that either my computation is almost perfect or I made a big error somewhere :D

The Kuka arm performs very well. I have not seen it fail to pick and place. Sometimes, however **the kuka robot rotates its wrist a lot without moving**. I believe this is because I just compute any angles that give the correct end-effector position, however there may be another correct solution which is closer to the current angle configuration and which wouldn't require so much rotating. A potential improvement could be to try to find a solution which requires as little rotating as possible.

Another possible problem with my solution is that I don't not impose constraints on joint variables. So it could happen that my solution suggests a joint angle that cannot be reached. I have not seen it happen, but I think it's very much possible.
