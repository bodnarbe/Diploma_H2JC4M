from sympy import symbols, Matrix
import numpy as np
import math

# Szimbolikus változók létrehozása
l_1, l_2, l_2_x, l_2_z, l_3, l_3_x, l_4, l_5, l_6 = symbols('l_1 l_2 l_2_x l_2_z l_3 l_3_x l_4 l_5 l_6')

alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6, alpha_7, alpha_8, alpha_9 = symbols('alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6 alpha_7 alpha_8 alpha_9')
a_1, a_2, a_3, a_4, a_5, a_6, a_7, a_8, a_9 = symbols('a_1 a_2 a_3 a_4 a_5 a_6 a_7 a_8 a_9')
theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7, theta_8, theta_9 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7 theta_8 theta_9')
d_1, d_2, d_3, d_4, d_5, d_6, d_7, d_8, d_9 = symbols('d_1 d_2 d_3 d_4 d_5 d_6 d_7 d_8 d_9')
S, C = symbols('S C')

# (alpha,a,theta,d)


a, b, c, d, e, f, g, h, i = symbols('a b c d e f g h i')

# Szimbolikus mátrixok létrehozása
A = Matrix([[a, b, c],
            [d, e, f],
            [g, h, i]])

B = Matrix([[1, 2, 3],
            [4, 5, 6],
            [7, 8, 9]])

class Forward_kinematics:
    def __init__(self, name):
        self.name = name
        self.t_0_1 = [0,0,theta_1,d_1]
        self.t_1_2 = [0,a_2,0,d_2]
        self.t_2_3 = [0,a_3,0,d_3]
        self.t_3_4 = [0,0,theta_4,0]
        self.t_4_5 = [alpha_5,0,theta_5,0]
        self.t_5_6 = [0,0,0,d_6]
        self.t_6_7 = [alpha_7,0,theta_7,0]
        self.t_7_8 = [0,a_8,theta_8,0]
        self.t_8_9 = [alpha_9,0,theta_9,d_9]

        # t_0_1 = [0,0,(joint_angles[0]),l_1]
        # t_1_2 = [0, l_2_x + l_2 * self.S(joint_angles[1]), 0, - l_2 * self.C(joint_angles[1]) + l_2_z]
        # t_2_3 = [0, l_3_x + l_3 * self.C(joint_angles[3]), 0, l_3 * self.S(joint_angles[3])]
        # t_3_4 = [0, 0, -1 * math.pi / 2, 0]
        # t_4_5 = [-1 * math.pi / 2, 0, joint_angles[4], 0]
        # t_5_6 = [0, 0, 0, l_4]
        # t_6_7 = [math.pi / 2, 0, joint_angles[5], 0]
        # t_7_8 = [0, l_5, -1 * math.pi / 2, 0]
        # t_8_9 = [-1 * math.pi / 2, 0, joint_angles[6], l_6]

        # make it print better
        np.set_printoptions(suppress=True)

        # modified DH parameters: alpha a theta d
        # types: revolute=1, prismatic=2 (not implemented yet)
        dh = [
                {'alpha': 0,  'a': 0, 'theta': 0, 'd': 0, 'type': 1},
                {'alpha': pi/2, 'a': 52, 'theta': 0, 'd': 0, 'type': 1},
                {'alpha': 0, 'a': 89, 'theta': 0, 'd': 0, 'type': 1},
                {'alpha': 0, 'a': 90, 'theta': 0, 'd': 0, 'type': 1},
                {'alpha': 0, 'a': 95, 'theta': 0, 'd': 0, 'type': 1}
            ]

kc = KinematicChain.from_parameters(dh)

# forward kinematics
angles = np.deg2rad([-45.00, 77.41, -98.15, -69.27, 0])
t = kc.forward(angles)
print(f">> {t}")

# Eredmény kiíratása
fw=Forward_kinematics("v1")
t_0_1=fw.dh_transform(fw.t_0_1)
t_1_2=fw.dh_transform(fw.t_1_2)
t_2_3=fw.dh_transform(fw.t_2_3)
t_3_4=fw.dh_transform(fw.t_3_4)
t_4_5=fw.dh_transform(fw.t_4_5)
t_5_6=fw.dh_transform(fw.t_5_6)
t_6_7=fw.dh_transform(fw.t_6_7)
t_7_8=fw.dh_transform(fw.t_7_8)
t_8_9=fw.dh_transform(fw.t_8_9)
print(t_0_1*t_1_2)