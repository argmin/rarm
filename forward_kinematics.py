from sympy import symbols, cos, sin, pi, simplify, pprint
from sympy.matrices import Matrix
from mpmath import atan2, sqrt, acos
import numpy as np

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

a01 = 0
a12 = 0.35
a23 = 1.25
a34 = -0.054
a45 = 0
a56 = 0
a67 = 0

# DH Parameters
dh = {alpha0: 0, a0: a01, d1: 0.75, 
     alpha1: -pi/2, a1: a12, d2: 0, q2: q2 - pi/2, 
     alpha2: 0, a2: a23, d3: 0,
     alpha3: -pi/2, a3: a34, d4: 1.50,
     alpha4: pi/2, a4: a45, d5: 0,
     alpha5: -pi/2, a5: a56, d6: 0,
     alpha6: 0, a6: a67, d7: 0.303, q7: 0}


def build_trans_matrix(alpha, a, q, d):
    return Matrix([[cos(q), -sin(q), 0, a],
    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
    [sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
    [0, 0, 0, 1]])

# Forward Kinematics



T0_1 = build_trans_matrix(alpha0, a0, q1, d1)
T0_1 = T0_1.subs(dh)

T1_2 = build_trans_matrix(alpha1, a1, q2, d2)
T1_2 = T1_2.subs(dh)

T2_3 = build_trans_matrix(alpha2, a2, q3, d3)
T2_3 = T2_3.subs(dh)

T3_4 = build_trans_matrix(alpha3, a3, q4, d4)
T3_4 = T3_4.subs(dh)

T4_5 = build_trans_matrix(alpha4, a4, q5, d5)
T4_5 = T4_5.subs(dh)

T5_6 = build_trans_matrix(alpha5, a5, q6, d6)
T5_6 = T5_6.subs(dh)

T6_g = build_trans_matrix(alpha6, a6, q7, d7)
T6_g = T6_g.subs(dh)


#T0_2 = simplify(build_trans_matrix(alpha0, a0, q1, d1) * build_trans_matrix(alpha1, a1, q2, d2))
#pprint(T0_2)
'''
T0_3 = simplify(T0_2 * T2_3)

T0_4 = simplify(T0_3 * T3_4)

T0_5 = simplify(T0_4 * T4_5)

T0_6 = simplify(T0_5 * T5_6)

T0_g = simplify(T0_6 * T6_g)
'''

T0_2 = (T0_1 * T1_2)

T0_3 = (T0_2 * T2_3)

T0_4 = (T0_3 * T3_4)

T0_5 = (T0_4 * T4_5)

T0_6 = (T0_5 * T5_6)

T0_g = (T0_6 * T6_g)

R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
              [sin(pi), cos(pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],
              [0, 1, 0, 0],
              [-sin(-pi/2), 0, cos(-pi/2), 0],
              [0,0,0,1]])

R_corr = (R_z * R_y)

T0_total = (T0_g * R_corr)


ctrl = {q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0} 

print("T0_1=", T0_1.evalf(subs=ctrl))
print("T0_2=", T0_2.evalf(subs=ctrl))
print("T0_3=", T0_3.evalf(subs=ctrl))
print("T0_4=", T0_4.evalf(subs=ctrl))
print("T0_5=", T0_5.evalf(subs=ctrl))
print("T0_6=", T0_6.evalf(subs=ctrl))
print("T0_g=", T0_g.evalf(subs=ctrl))
print("T_total=", T0_total.evalf(subs=ctrl))

T_eval = T0_total.evalf(subs=ctrl)
Wc = (T_eval.col(3) - (0.303 + 0) * T_eval.col(2))[:3]

print T_eval.col(2)


# Inverse kinematics workbook

# Values posted by RoboND slack community
#Wc = Matrix([[0.165865000000000], [0.229042000000000], [2.58480000000000], [1.00000000000000]])
#Wc = Matrix([[-0.51301794000116], [2.49961331185438], [1.60002874806474], [1.00000000000000]])

def rot_x(q):
    R_x = Matrix([[1, 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q), cos(q)]])
    return R_x

def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1, 0],
                  [-sin(q), 0, cos(q)]])
    return R_y

def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                 [sin(q), cos(q), 0],
                 [0, 0, 1]])
    return R_z

# Values posted by RoboND slack community
px, py, pz = -0.209973197041, 2.49999627205, 1.6000302304
r, p, y = -0.000442585913517, 0.000232783125596, 0.000765804329612

R_corr = rot_z(pi) * rot_y(-pi/2)

R_rpy = rot_z(y) * rot_y(p) * rot_x(r)

R0_6 = R_rpy * R_corr

print('R_rpy: ', R_rpy)

Wc = Matrix([[px], [py], [pz]]) - (0.303) * R0_6.col(2)
print('Wc:', Wc)

theta1 = atan2(Wc[1], Wc[0])
print('q1=', theta1)

L2_3 = 1.25
L3_5 = sqrt(0.96**2 + 0.054**2) + 0.54 

JX0_2 = 0.35 / cos(theta1)
JZ0_2 = 0.75

L2_5 = sqrt((Wc[0] - JX0_2)**2 + (Wc[1])**2 + (Wc[2] - JZ0_2)**2)

D = (-L3_5**2 + L2_3**2 + L2_5**2) / (2 * L2_3 * L2_5)
D = 1 if (D > 1 or D < -1) else D
theta2 = atan2(sqrt(1 - D**2), D) + atan2((Wc[2]-JZ0_2), sqrt((Wc[0]-JX0_2)**2 + Wc[1]**2))
theta2 = (np.pi / 2. - theta2)
print('q2=', theta2)

D = (L2_5**2 - L2_3**2 - L3_5**2) / (2 * L2_3 * L3_5)
D = 1 if (D > 1 or D < -1) else D
theta3 = atan2(0.054, 1.5) - np.pi/ 2. + acos(D)
#theta3 = -(theta3 - atan2(0.054, 1.5) - np.pi/2.)

print('q3=', theta3)

#R3_6 = simplify(T3_4 * T4_5 * T5_6)
#pprint(R3_6)

# Values posted by RoboND slack community
#theta1, theta2, theta3 = 0.94403337696, -0.983765211672, 0.0827262605206
#theta1, theta2, theta3 = 1.7732, 0.6055, -0.5228

R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})[0:3, 0:3]
R3_6 = R0_3.T * R_rpy * R_corr.T
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[1,0]**2 + R3_6[1,1]**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])

print [theta4, theta5, theta6] 
 
