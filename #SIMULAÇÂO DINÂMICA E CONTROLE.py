#SIMULAÇÂO DINÂMICA E CONTROLE
import numpy as np
from roboticstoolbox import ET, ERobot, Link
import sympy

#a2 = 0.2
#a3 = 0.2

q0, q1, q2 = sympy.symbols("q0, q1, q2")

a2, a3, r1, r2, r3, m1, m2, m3, g = sympy.symbols("a2, a3, r1, r2, r3, m1, m2, m3, g")

link0 = Link(ET.tz(qlim=[0, 0.170]), r = [0, 0, q0/2])
link1 = Link(ET.tz(0.300) * ET.Rz(qlim=[-np.pi, np.pi]), r = [0, 0, 0.15], m = m1, I = [0, 0, 0])
link2 = Link(ET.tx(a2) * ET.tx(a3) * ET.Rx(qlim=[-np.pi, np.pi]), r = [0.4, 0, 0], m = m2, I = [0, 0, 0])
link3 = Link(ET.Ry(np.pi/6) * ET.tz(-0.100) * ET.tx(0.100) * ET.tz(0.100), r = [0.05, 0, 0.05], m = m3, I = [0, 0, 0])

cookbot = ERobot([link0, link1, link2, link3], name="Cookbot")
print(cookbot)
print(cookbot.dynamics())

#definindo as variáveis simbólicas para as juntas
q = sympy.symbols("q:3")
qd = sympy.symbols("qd:3")
qdd = sympy.symbols("qdd:3")

#torque das juntas em funçao de q, qd e qdd
tau = cookbot.rne(q, qd, qdd, gravity = [0, 0, g], symbolic=True)
#print("O torque das juntas é dado por: ", tau)

cookbot.qn = (0,0,0)

zero = np.zeros(3)
Q = cookbot.rne(cookbot.qn, zero, zero)
