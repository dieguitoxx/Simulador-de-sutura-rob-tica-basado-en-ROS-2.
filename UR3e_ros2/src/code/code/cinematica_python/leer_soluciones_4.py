import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DaVinciPreciseIK:
    def __init__(self):
        self.dh_params = [
            {'theta': 0,       'alpha': np.pi/2,  'a': 0,      'd': 0},
            {'theta': np.pi/2, 'alpha': np.pi/2,  'a': 0,      'd': 0},
            {'theta': 0,       'alpha': 0,        'a': 0,      'd': 0.01},
            {'theta': np.pi,   'alpha': np.pi/2,  'a': 0,      'd': 0},
            {'theta': np.pi/2, 'alpha': -np.pi/2, 'a': 0.0091, 'd': 0},
            {'theta': np.pi/2, 'alpha': np.pi/2,  'a': 0.0102, 'd': 0}
        ]

        self.base_to_dh0 = np.array([
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])

    def dh_transform(self, theta, alpha, a, d):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, q):
        T = self.base_to_dh0.copy()
        positions = [T[:3, 3]]  # Almacena la posición inicial (base)
        
        for i in range(6):
            params = self.dh_params[i].copy()
            if i == 2:
                params['d'] = q[i]  
            else:
                params['theta'] += q[i]
            T = T @ self.dh_transform(**params)
            positions.append(T[:3, 3])  # Guardar posición de la articulación
        
        return np.array(positions)  # Devuelve todas las posiciones en una matriz

# Cargar los datos de las articulaciones
file_path = "herida/matrizm_q_vec_ik_1.npy"
matrix = np.load(file_path)

# Crear el objeto del robot
robot = DaVinciPreciseIK()

# Configurar la gráfica 3D
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Graficar cada 10 muestras del robot
for i in range(0, len(matrix), 10):  # Tomar una muestra cada 10
    q = matrix[i]
    positions = robot.forward_kinematics(q)
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '-o', markersize=5)

# Configurar etiquetas y título
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Posición cartesiana de las articulaciones del robot Da Vinci")

plt.show()
