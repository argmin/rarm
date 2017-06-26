from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

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

T0_2 = simplify(T0_1 * T1_2)

T0_3 = simplify(T0_2 * T2_3)

T0_4 = simplify(T0_3 * T3_4)

T0_5 = simplify(T0_4 * T4_5)

T0_6 = simplify(T0_5 * T5_6)

T0_g = simplify(T0_6 * T6_g)


R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
              [sin(pi), cos(pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],
              [0, 1, 0, 0],
              [-sin(-pi/2), 0, cos(-pi/2), 0],
              [0,0,0,1]])

R_corr = simplify(R_z * R_y)

T0_total = simplify(T0_g * R_corr)


ctrl = {q1: -2, q2: 1, q3: 0, q4: 0, q5: 0, q6: 0} 

print("T0_1=", T0_1.evalf(subs=ctrl))
print("T0_2=", T0_2.evalf(subs=ctrl))
print("T0_3=", T0_3.evalf(subs=ctrl))
print("T0_4=", T0_4.evalf(subs=ctrl))
print("T0_5=", T0_5.evalf(subs=ctrl))
print("T0_6=", T0_6.evalf(subs=ctrl))
print("T0_g=", T0_g.evalf(subs=ctrl))
print("T_total=", T0_total.evalf(subs=ctrl))

T_eval = T0_total.evalf(subs=ctrl)
print(T_eval.col(3) - (0 + 0) * T_eval.col(2))
