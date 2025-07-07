#importar bibliotecas necessárias
from roboticstoolbox import ET, ERobot, xplot
import numpy as np
from spatialmath import SE3
import sympy as sp

#FUNÇÃO PARA LIMPAR RESULTADOS
import numpy as np

def clear_results(valores, tol=1e-5):
    valores = np.array(valores)
    valores[np.abs(valores) < tol] = 0
    return valores

#definindo o braço do robô

#parâmetros do braço
a2 = 0.6 
a3 = 0.2

#estrutura do braço
e = ET.tz(qlim=[0, 0.170]) * ET.tz(0.300) * ET.Rz(qlim=[-np.pi, np.pi]) * ET.tx(a2) * ET.tx(a3) * ET.Rx(qlim=[-np.pi, np.pi]) * ET.Ry(np.pi/4) * ET.tz(-0.100) * ET.tx(0.100) * ET.tz(0.100)

#estrutura do end-effector
cookbot = ERobot(e, name = "cookbot")

#posição de teste
pos_test = (0.16, 1.57079633, -3.14159265)

#testes
print("Número de juntas: ", cookbot.n)
print("Estrutura do robô:", cookbot)
cookbot.teach(pos_test, block=True)

#CINEMÁTICA INVERSA NUMÉRICA

#supondo um valor de posição final e inicial
pos_inicial = (0.900, 0, 0.300)
pos_inicial = SE3(pos_inicial)

pos_final = (0, 0.900, 0.460)
pos_final = SE3(pos_final)

#cinemática inversa numérica
sol1 = cookbot.ikine_LM(pos_inicial)
sol2 = cookbot.ikine_LM(pos_final)
print("A solução numérica da cinemática inversa para posição inicial é:")
print(clear_results(sol1.q))
print("A solução numérica da cinemática inversa para posição final é:")
print(clear_results(sol2.q))


#TRAJETÓRIAS

#intervalo de tempo
from roboticstoolbox import mtraj, quintic


t = np.arange(0, 3, 0.04) 

#traj1 = mtraj(quintic, sol1.q, sol2.q, t)
#cookbot.plot(traj1.q)


from roboticstoolbox import mstraj

#INTERVALOS DE TEMPO
t1 = np.arange(0, 5, 0.04)

#TRAJETÓRIAS
#batatas serão despejadas na bandeja
POS0 = cookbot.fkine([0, 0, 0])
print("A posição inicial do end-effector é:", POS0.t)
#posição 1: levar as batatas até acima da fritadeira
POS1 = cookbot.fkine([0.17, np.pi/2, 0])
print("A posição  do end-effector na posição 1 é:", POS1.t)
#posição 2: levar as batatas até dentro da fritadeira
POS2 = cookbot.fkine([0, np.pi/2, 0])
print("A posição  do end-effector na posição 2 é:", POS2.t)
#posição 3: retirar as batatas da fritadeira
POS3 = POS1
print("A posição  do end-effector na posição 3 é:", POS3.t)
#posição 4: levar as batatas até a bandeja de despejo
POS4 = cookbot.fkine([0.17, np.pi, 0])
print("A posição  do end-effector na posição 4 é:", POS4.t)
#posição 5: despejar as batatas na bandeja
POS5 = cookbot.fkine([0.17, np.pi, np.pi])
print("A posição  do end-effector na posição 5 é:", POS5.t)


#Crinado um vetor com as posições
viapoints = np.array([
    [0, 0, 0],                       # POS0
    [0.17, np.pi/2, 0],              # POS1
    [0, np.pi/2, 0],                 # POS2
    [0.17, np.pi/2, 0],              # POS3
    [0.17, np.pi, 0],                # POS4
    [0.17, np.pi, np.pi],            # POS5
])

traj_final = mstraj(
    viapoints=viapoints, 
    dt=0.04, 
    tacc=0.2, 
    qdmax=1.0
)

cookbot.plot(traj_final.q)

