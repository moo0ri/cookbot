#SIMULAÇÂO NUMÉRICA DINÂMICA E CONTROLE
#SIMULAÇÂO DINÂMICA E CONTROLE
import numpy as np
from roboticstoolbox import ET, ERobot, Link, mstraj
from spatialmath import SE3
import sympy

# Definindo os parâmetros do robô Cookbot
a2 = 0.6
a3 = 0.2
m0 = 0.1
m1 = 0.5
m2 = 0.2
m3 = 0.3
g = 9.81

# Definindo o manipulador robótico
link0 = Link(ET.tz(qlim=[0, 0.170]), r = [0, 0, 0.085], m = m0, I = [0, 0, 0])
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
#tau = cookbot.rne(q, qd, qdd, gravity = [0, 0, g], symbolic=True)
#print("O torque das juntas é dado por: ", tau)

cookbot.qn = (0,0,0)

zero = np.zeros(3)

#Torque necessário para manter o robô na posição nominal contra a ação da gravidade
Q = cookbot.gravload(cookbot.qn)
print("O torque das juntas na posição nominal é dado por: ", Q)


# TORQUE NA TRAJETÓRIA


#TRAJETÓRIAS
#batatas serão despejadas na bandeja
POS0 = cookbot.fkine([0, 0, 0])
#posição 1: batatas acima da fritadeira
POS1 = cookbot.fkine([0.17, np.pi/2, 0])
#posição 2: batatas  dentro da fritadeira
POS2 = cookbot.fkine([0, np.pi/2, 0])
#posição 3: retirar as batatas da fritadeira
POS3 = POS1
#posição 4: levar as batatas até a bandeja de despejo
POS4 = cookbot.fkine([0.17, np.pi, 0])
#posição 5: despejar as batatas na bandeja
POS5 = cookbot.fkine([0.17, np.pi, np.pi])


#Crinado um vetor com as posições
viapoints = np.array([
    [0, 0, 0],                       # POS0
    [0.17, np.pi/2, 0],              # POS1
    [0, np.pi/2, 0],                 # POS2
    [0.17, np.pi/2, 0],              # POS3
    [0.17, np.pi, 0],                # POS4
    [0.17, np.pi, np.pi],            # POS5
])

#vetor com os tempos de cada segmento
tsegment = np.array([5,         # Tempo para ir de POS0 a POS1                    
                     8,         # Tempo para ir de POS1 a POS2  
                     8,         # Tempo para ir de POS2 a POS3
                     6,         # Tempo para ir de POS3 a POS4
                     5])        # Tempo para ir de POS4 a POS5

traj_final = mstraj(
    viapoints=viapoints, 
    dt=0.04, 
    tacc=2, 
    tsegment = tsegment
)

#VELOCIDADE E ACELERAÇÃO DAS JUNTAS


q = traj_final.q
t = traj_final.t

# Derivada numérica para obter velocidade
qd = np.gradient(q, t, axis=0)

# Derivada numérica para obter aceleração
qdd = np.gradient(qd, t, axis=0)

import matplotlib.pyplot as plt
import numpy as np


# Derivada numérica para obter velocidades e acelerações
qd = np.gradient(q, t, axis=0)
qdd = np.gradient(qd, t, axis=0)

Qtraj = cookbot.rne(q, qd, qdd, gravity=[0, 0, g], symbolic=False)
print(Qtraj.shape)

t = np.arange(Qtraj.shape[0])  # vetor de tempo (amostras)

run_plots = input("Deseja rodar os plots de torque? (s/n): ").strip().lower()
if run_plots == 's':
    # Junta 1
    plt.figure(figsize=(8, 3))
    plt.plot(t, Qtraj[:, 0], label='Torque Junta 1')
    plt.xlabel('Amostra')
    plt.ylabel('Torque [Nm]')
    plt.title('Torque da Junta 1 ao Longo do Tempo')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Junta 2
    plt.figure(figsize=(8, 3))
    plt.plot(t, Qtraj[:, 1], label='Torque Junta 2', color='orange')
    plt.xlabel('Amostra')
    plt.ylabel('Torque [Nm]')
    plt.title('Torque da Junta 2 ao Longo do Tempo')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Junta 3
    plt.figure(figsize=(8, 3))
    plt.plot(t, Qtraj[:, 2], label='Torque Junta 3', color='green')
    plt.xlabel('Amostra')
    plt.ylabel('Torque [Nm]')
    plt.title('Torque da Junta 3 ao Longo do Tempo')
    plt.grid(True)
    plt.tight_layout()
    plt.show()
else:
    print("Plots de torque não executados.")




run_gravplot = input("Deseja rodar o plot da carga gravitacional? (s/n): ").strip().lower()

if run_gravplot == 's':
    # Cálculo de carga gravitacional
    N = 100
    Q1, Q2 = np.meshgrid(np.linspace(-np.pi, np.pi, N),
                         np.linspace(-np.pi, np.pi, N))
    G1, G2 = np.zeros((N, N)), np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            g = cookbot.gravload(np.array([0, Q1[i, j], Q2[i, j]]))
            G1[i, j] = g[1]  # shoulder gravity load
            G2[i, j] = g[2]  # elbow gravity load
    plt.axes(projection="3d").plot_surface(Q1, Q2, G1)
    plt.title('Torque Gravitacional da Shoulder (Junta 2)')
    plt.xlabel('Q1 (rad)')
    plt.ylabel('Q2 (rad)')
    plt.show()
else:
    print("Plot de carga gravitacional não executado.")

print("Inércia do robô na posição nominal:")
print(cookbot.inertia(cookbot.qn))

qd_teste = np.array([0.1, 0.1, 0.1])  # Velocidade de teste
print("Coriolis do robô na posição nominal:")
print(cookbot.coriolis(cookbot.qn, qd_teste))