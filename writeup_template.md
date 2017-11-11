## Project: Kinematics Pick & Place
[//]: # (Image References)

[image1]: ./misc_images/dh.jpg
[image2]: ./misc_images/equation.jpg
[image3]: ./misc_images/matrix.jpg
[image4]: ./misc_images/position1.jpg
[image5]: ./misc_images/position2.jpg

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The following is a picture showing the Denavit-Hartenberg parameters for the Kuka KR210 robot.

![alt text][image1]

Here is a table of the DH values that were taken from the kr210.urdf.xacro file. 

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

I'll discuss how I got these values. Let's take alpha(1) for example. alpha in general is the twist angle between Z(i-1) and Z(i) around 
X(i-1) in a right hand sense. alpha(1) is obtained by rotating Z(1) to Z(2) along X(1) which 
is -pi/2. alpha(3), alpha(4) and alpha(5) are derived the same way.

a(i-1) is the distance from Z(i-1) to Z(i) measured along X(i-1). For a1, the distance between
Z(1) and Z(2) along X(1) is just the distance in the urdf file of joint 1 to joint 2 along X which
is 0.35. a(2) and a(3) are derived similarly.

Theta is the angle between X(i-1) to X(i) measured about Z(i) in a right hand sense.
For theta(1), theta(3), theta(4), theta(5) and theta(6) this is just a variable quantity. Theta(2) is a bit tricky. It has an offset of -pi/2 because that's the angle you would rotate X(1) to X(2)
along Z(2). theta(7) is 0 because that line in the DH table just explains the translation from joint 6
to the end-effector. There is no revolute joint 7.


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint.

The forward kinematics problem is the composition of homogeneous transforms starting with the base link
to the end effector. Each individual transformation about a joint is simplified down
to the matrix returned by TF_Matrix. Each individual tranformation is the cumulative effect 
of applying 4 individual transforms, 2 rotations and 2 translations. 

```
DH_Table = {alpha0:        0, a0:      0, d1:  0.75, q1: q1,
            alpha1:    -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
            alpha2:        0, a2:   1.25, d3:     0, q3: q3,
            alpha3:    -pi/2, a3: -0.054, d4:  1.50, q4: q4,
            alpha4:     pi/2, a4:      0, d5:     0, q5: q5,
            alpha5:    -pi/2, a5:      0, d6:     0, q6: q6,
            alpha6:        0, a6:      0, d7: 0.303, q7: 0}

def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                 [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                 [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                 [                0,                 0,           0,             1]])
    return TF

T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_G = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G 

```
#### In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
The complete homogeneous transform between the base_link and the gripper_link using just
the end-effector pose is the following:

![alt text][image2]

In code this is:
```
px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [req.poses[x].orientation.x, req.poses[x].orientation.y,
     req.poses[x].orientation.z, req.poses[x].orientation.w])

def CorrectedRotationMatrix(r, p, y, cp, cy):
    R_x = Matrix([[ 1,              0,          0],
                  [ 0,         cos(r),    -sin(r)],
                  [ 0,         sin(r),     cos(r)]])
              
    R_y = Matrix([[cos(p),          0,     sin(p)],
                  [0,               1,          0],
                  [-sin(p),         0,     cos(p)]])

    R_z = Matrix([[ cos(y),   -sin(y),          0],
                  [ sin(y),    cos(y),          0],
                  [ 0,              0,          1]])

    C_y = Matrix([[cos(cp),         0,    sin(cp)],
                  [0,               1,          0],
                  [-sin(cp),        0,    cos(cp)]])

    C_z = Matrix([[ cos(cy), -sin(cy),          0],
                  [ sin(cy),  cos(cy),          0],
                  [ 0,              0,          1]])
    return R_z * R_y * R_x * C_z * C_y

T0_G = CorrectedRotationMatrix(roll, pitch, yaw, -pi/2, pi)
T0_G = T0_G.row_join(Matrix([[px], [py], [pz]]))
T0_G = T0_G.col_join(Matrix([[0, 0, 0, 1]])) 
```
Note the corrected matrix is post multiplied to the rotation matrix. This is because
the frame from the DH table did not match the orientation of the default value in gazebo.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Position:
First we find the orthonormal vector representing the Z axis in the local coordinate frame of the'
end-effector's orientation and translate backward from the end-effector by the length 
of the sum of d6 and the end-effector length. This gives us wc (wrist center). 

![alt text][image4]

![alt text][image5]

##### Inverse Orientation:

First we find the symbolic matrix between T3_4 and T5_6 which is the following:

![alt text][image3]

Next we look at the relationships between the entries to come up with equations for theta4, theta5 and
theta6. Assuming the symbolic matrix is named R3_6:

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0]) 
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

I implemented the techniques described above in IK_server.py. I noticed that some of theta solutions were 2pi instead
of just 0, which made the revolute joint needlessly rotate all the way around. If I had more time, I would 
check to see if theta was greater than 2*pi and then subtract that out. My program did do a relatively good job
of the pick and place passing the minimum specifications of 8 out of 10. Sometimes it would get stuck with joint
collisions. If I had more time I would look into handling the joint collisions better.

Also, if I had more time I would like to try different ways of solving for theta such as the Jacobian Inverse 
Technique or other heuristic methods.



