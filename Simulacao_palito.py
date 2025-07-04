#importar bibliotecas necessárias
from roboticstoolbox import ET, ERobot
import numpy as np
from spatialmath import SE3
import sympy as sp

#definindo o braço do robô

#parâmetros do braço
a2 = 600 
a3 = 200

#estrutura do braço
e = ET.tz(qlim=[0, 170]) * ET.tz(300) * ET.Rz(qlim=[-np.pi, np.pi]) * ET.tx(a2) * ET.tx(a3) * ET.Rx(qlim=[-np.pi, np.pi]) * ET.tz(-100) * ET.tx(100) * ET.tz(100)
tool = ET.tx(100) * ET.tz(100)
#estrutura do end-effector
cookbot = ERobot(e, tool = tool, name = 'Cookbot')

#posição de teste
pos_test = (148.03351258, 1.57079511, 1.96325879)

#testes
print("Número de juntas: ", cookbot.n)
print("Estrutura do robô:", cookbot)
cookbot.teach(pos_test, block=True)