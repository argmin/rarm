# Project: Pick and Place
#### Report
---

### Summary
The goal of the project was to introduce the kinematics and demonstrate it's application to plan a robots joint movements, in our case the was `KUKA KR 210`.

We begin by understanding the structure of the arm, basically it's joints & links, the DoF, constraints, and build a representation that minimally describes the variables required to move the arm at any given location in it's reachable space.


### Forward Kinematics

In this section we describe forward kinematics of of the arm, the idea here is to describe the geometry such that when you control the joint angles it would describe the position of each joint and the end effector. 

Lets begin by handing labels to the joints and links, joints `1 to 6 + end effector`, and links `1 to 6 + base & gripper link`. We fix the frame fo references at each joints, in addtion to a joint `O_0` from which the base link orininates. The next step is to assign DH parameters, the values of which were extracted from the lessons and the urdf file.

#### DH parameter

```python
dh = {alpha0: 0, a0: a01, d1: 0.75,
     alpha1: -pi/2, a1: a12, d2: 0, q2: q2 - pi/2,
     alpha2: 0, a2: a23, d3: 0,
     alpha3: -pi/2, a3: a34, d4: 1.50,
     alpha4: pi/2, a4: a45, d5: 0,
     alpha5: -pi/2, a5: a56, d6: 0,
     alpha6: 0, a6: a67, d7: 0.303, q7: 0}

```

Here's the image that would give an idea of frame of reference atached to the joints
![Frame of Reference](https://github.com/argmin/rarm/blob/03f11b0a9f68f49f9f425c982794fabf343af98c/for.jpg)


The next step is to build a homogeneous trantion matrix from the base to the end effector, each of which would describe the position and orientation of the joint wrt to origin in terms of DH parameters as shown in the example.

```python
T0_2 =

⎡                     -sin(q₁)⋅sin(q₂)⋅cos(α₁) + cos(q₁)⋅cos(q₂)                                            -sin(q₁)⋅cos(α₁)⋅cos(q₂) - sin(q₂)⋅cos(q₁)                                    sin(α₁)⋅sin(q₁)                                  
⎢                                                                                                                                                                                                                                          
⎢-sin(α₀)⋅sin(α₁)⋅sin(q₂) + sin(q₁)⋅cos(α₀)⋅cos(q₂) + sin(q₂)⋅cos(α₀)⋅cos(α₁)⋅cos(q₁)  -sin(α₀)⋅sin(α₁)⋅cos(q₂) - sin(q₁)⋅sin(q₂)⋅cos(α₀) + cos(α₀)⋅cos(α₁)⋅cos(q₁)⋅cos(q₂)  -sin(α₀)⋅cos(α₁) - sin(α₁)⋅cos(α₀)⋅cos(q₁)  a₁⋅sin(q₁)⋅cos(α₀)
⎢                                                                                                                                                                                                                                          
⎢sin(α₀)⋅sin(q₁)⋅cos(q₂) + sin(α₀)⋅sin(q₂)⋅cos(α₁)⋅cos(q₁) + sin(α₁)⋅sin(q₂)⋅cos(α₀)   -sin(α₀)⋅sin(q₁)⋅sin(q₂) + sin(α₀)⋅cos(α₁)⋅cos(q₁)⋅cos(q₂) + sin(α₁)⋅cos(α₀)⋅cos(q₂)  -sin(α₀)⋅sin(α₁)⋅cos(q₁) + cos(α₀)⋅cos(α₁)  a₁⋅sin(α₀)⋅sin(q₁)
⎢                                                                                                                                                                                                                                          
⎣                                         0                                                                                     0                                                                0                                         

    a₀ + a₁⋅cos(q₁) + d₂⋅sin(α₁)⋅sin(q₁)                       ⎤
                                                               ⎥
 - d₁⋅sin(α₀) - d₂⋅sin(α₀)⋅cos(α₁) - d₂⋅sin(α₁)⋅cos(α₀)⋅cos(q₁)⎥
                                                               ⎥
 + d₁⋅cos(α₀) - d₂⋅sin(α₀)⋅sin(α₁)⋅cos(q₁) + d₂⋅cos(α₀)⋅cos(α₁)⎥
                                                               ⎥
                      1                                        ⎦


```

`T0_2` is obtained by multiplication of transition matrix from joint 0 to 1 (`T0_1`) and joint 1 to 2 (`T1_2`)

This logic is extended to find the transition matrix from base link's origin to the end effector (`T0_7`). Now given the `theta's` we can compute the orientation and position of the end effector.

### Inverse kinematics

In this section we how to compute the values of `theta` given the expected position of the end effector, in other words we are revese engineering the forward kinemtics transition matrices to get the control values, which in our case are `theta 1 to 6`. In this project MoveIt describes the path / trajectory of the point and we need to compute the thetas such then end effector follows that trajectory.

#### Theta 1
Calculation of theta 1 is just projecting the wrist center on XY plane of the origin of joint 1 / base link.

#### Theta 2
First we need to form a triangle between join 2, 3, & end effector say 5, and compute the distance between joint 2 & 5, by moving the frame of reference from origin of base link to origin of joint 2.

```python
L2_5 = sqrt((Wc[0] - JX0_2)**2 + (Wc[1])**2 + (Wc[2] - JZ0_2)**2)
```
Next we find the angle between the line joining joint 2-3 and 2-5 (wrist center) with the use of cosine rule, add it to the angle formed by line joining joint 2 - 5 and plane of joint 2 and adjust of knee bend.

```python
D = (-L3_5**2 + L2_3**2 + L2_5**2) / (2 * L2_3 * L2_5)
D = 1. if (D > 1 or D < -1) else D
theta2 = atan2(sqrt(1. - D*D), D) + atan2((Wc[2]-JZ0_2), sqrt((Wc[0]-JX0_2)**2 + (Wc[1])**2))
theta2 = np.pi / 2. - theta2

```
#### Theta 3
We first need to find the angle between line from joint 3-5 and line from extending from joint 2 - 3, and make adjustment for the offset for joint 4 in Z direction and the knee bend.

```python
D = (L2_5**2 - L2_3**2 - L3_5**2) / (2 * L2_3 * L3_5)
D = 1. if (D > 1 or D < -1) else D # Precaution agains imaginary numbers
theta3 = atan2(0.054, 1.5) - np.pi/2. + acos(D)

``` 

#### Theta 4-6
To compute theta4-6 we need to first find the orientation matrix from `R3_6`, when theta1-3 are applied to the transition matrix `T0_3`


```python
T3_6 =
⎡-sin(q₄)⋅sin(q₆) + cos(q₄)⋅cos(q₅)⋅cos(q₆)  -sin(q₄)⋅cos(q₆) - sin(q₆)⋅cos(q₄)⋅cos(q₅)  -sin(q₅)⋅cos(q₄)  -0.054⎤
⎢                                                                                                                ⎥
⎢             sin(q₅)⋅cos(q₆)                             -sin(q₅)⋅sin(q₆)                   cos(q₅)        1.5  ⎥
⎢                                                                                                                ⎥
⎢-sin(q₄)⋅cos(q₅)⋅cos(q₆) - sin(q₆)⋅cos(q₄)  sin(q₄)⋅sin(q₆)⋅cos(q₅) - cos(q₄)⋅cos(q₆)   sin(q₄)⋅sin(q₅)     0   ⎥
⎢                                                                                                                ⎥
⎣                    0                                           0                              0            1   ⎦

```

This provides a base to compute theta4-6

```python

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

```


### Drawbacks and Improvements
* The end effector follows the trajectory plotted by `MoveIt`, however it fails at the very end as it over extends the arm because of theta2 and 3. I haven't been able to figure out the reason yet.
* I do see imaginary numbers sometimes, the cause eludes me, for now I have a check to prevent it from happening.
* Inverse kinematics althogh very interesting is not particularly scalable way, perhaps a better way could be learning to follow a trajectory ex: imitation learning.


### Credits
Thanks to everyone in the Udacity Robotics ND Slack community, who helped me understand the concepts and clear the confusion.


