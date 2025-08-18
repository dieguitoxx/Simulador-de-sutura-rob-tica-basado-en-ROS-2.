# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
#import numpy as np




# importar socket para comunicarse con el robot
import socket





# define a function to convert rotation matrix to quaternion
def rotm2quat(r):
    # compute the trace of the rotation matrix
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    # check the cases for different traces
    if tr > 0:
        # compute the components of the quaternion
        qw = np.sqrt(1 + tr) / 2
        qx = (r[2, 1] - r[1, 2]) / (4 * qw)
        qy = (r[0, 2] - r[2, 0]) / (4 * qw)
        qz = (r[1, 0] - r[0, 1]) / (4 * qw)
    elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
        # compute the components of the quaternion
        qx = np.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) / 2
        qw = (r[2, 1] - r[1, 2]) / (4 * qx)
        qy = (r[0, 1] + r[1, 0]) / (4 * qx)
        qz = (r[0, 2] + r[2, 0]) / (4 * qx)
    elif r[1, 1] > r[2, 2]:
        # compute the components of the quaternion
        qy = np.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) / 2
        qw = (r[0, 2] - r[2, 0]) / (4 * qy)
        qx = (r[0, 1] + r[1, 0]) / (4 * qy)
        qz = (r[1, 2] + r[2, 1]) / (4 * qy)
    else:
        # compute the components of the quaternion
        qz = np.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) / 2
        qw = (r[1, 0] - r[0, 1]) / (4 * qz)
        qx = (r[0, 2] + r[2, 0]) / (4 * qz)
        qy = (r[1, 2] + r[2, 1]) / (4 * qz)
    # return the quaternion as a numpy array
    return np.array([qw, qx, qy, qz])


# define a function to convert quaternion to rotation matrix
def quat2rotm(q):
    # normalize the quaternion
    q = q / np.linalg.norm(q)
    # unpack the components
    qw, qx, qy, qz = q
    # compute the elements of the rotation matrix
    r11 = 1 - 2*qy**2 - 2*qz**2
    r12 = 2*qx*qy - 2*qz*qw
    r13 = 2*qx*qz + 2*qy*qw
    r21 = 2*qx*qy + 2*qz*qw
    r22 = 1 - 2*qx**2 - 2*qz**2
    r23 = 2*qy*qz - 2*qx*qw
    r31 = 2*qx*qz - 2*qy*qw
    r32 = 2*qy*qz + 2*qx*qw
    r33 = 1 - 2*qx**2 - 2*qy**2
    # return the rotation matrix as a numpy array
    return np.array([[r11, r12, r13],
                     [r21, r22, r23],
                     [r31, r32, r33]])






# Definir la función endotro_fwd_kinematics
def endotro_fwd_kinematicspy(joint_positions):
    # Tabla de parámetros DH
    gripper_jaw_length = 0.0001
    DH = np.zeros((6, 4))
    
    #  a      d    alfa   theta
    DH[0, :] = [0, 0, 1.5708, 0]
    DH[1, :] = [0, 0, 1.5708, 1.5708]
    DH[2, :] = [0, -0.0156, 0, 0]
    DH[3, :] = [0, 0, 1.5708, 3.14159]
    DH[4, :] = [0.0091, 0, -1.5708, 1.5708]
    DH[5, :] = [0, 0, 1.5708, 1.5708]

    # Número de articulaciones
    num_joints = DH.shape[0]

    # Inicializar la matriz de transformación homogénea
    T = np.eye(4)
    T1 = np.eye(4)
    T2 = np.eye(4)
    

    # Inicializar la lista para almacenar las matrices de transformación
    T_all = []

    T0 = np.array([[0, 1, 0, 0],
                   [0, 0,-1, 0],
                   [-1, 0, 0, 0],
                   [0, 0, 0, 1]])

    # Posición del agarre de la aguja
    a = 0
    alpha = 0
    d = gripper_jaw_length
    theta = -math.pi/2
    # Calcular la matriz de transformación homogénea para la articulación i
    Tp = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                   [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                   [0, math.sin(alpha), math.cos(alpha), d],
                   [0, 0, 0, 1]])

    # Calcular la matriz de transformación homogénea
    for i in range(num_joints):
        a = DH[i, 0]

        alpha = DH[i, 2]

        if i == 2: # como la tercera es prismática 
            d = joint_positions[i] + DH[i, 1]
            theta = DH[i, 3]
        else:
            d = DH[i, 1]
            theta = joint_positions[i] + DH[i, 3]
        # Calcular la matriz de transformación homogénea para la articulación i
        A = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                      [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                      [0, math.sin(alpha), math.cos(alpha), d],
                      [0, 0 ,0 ,1]])
        
        # Multiplicar la matriz de transformación homogénea acumulada por la matriz A
        #T = T @ A
        
        T = np.matmul(T, A)
        
        # Almacenar la matriz de transformación acumulada en la lista y se multiplica por las transformaciones necesarias para el endowrist y el robot
        
        T1= np.matmul(T, Tp)
        T2=np.matmul(T0, T1)       
        
        T_all.append(T2)

    # Devolver la lista con las matrices de transformación para cada articulación
    return T_all





def md_circulo_v7(p1, p2, rd):
    
    

    # Calcular el punto intermedio pc3
    pc3 = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2)

    # Calcular la elevación en el eje z
    elevacion_z = math.sqrt(rd**2 - (rd/2)**2)

    # Calcular el punto c que es el centro del círculo
    c = (pc3[0], pc3[1], pc3[2] + elevacion_z)
    pc0=c

 #   print("p1  ",p1 ,"   p2 " ,p2 ,"   pc3  ",pc3 ," pc0  ",pc0 ,)



    # Etiquetar el punto c
  #  ax.text(c[0], c[1], c[2], 'C')

    # Calcular la normal al plano formado por p1, p2 y c
    n = np.cross(p2 - p1, c - p1)

    # Normalizar la normal
    n = n / np.linalg.norm(n)



    # Definir el ángulo de rotación del círculo en radianes
    theta = np.linspace(0, 2 * np.pi, 100)

    # Definir el vector de rotación del círculo en el plano perpendicular a la normal
    r = np.array([-n[1], n[0], 0])

    # Normalizar el vector de rotación
    r = r / np.linalg.norm(r)

    # Calcular los puntos del círculo usando la fórmula de Rodrigues para la rotación alrededor de un eje arbitrario
    cx = c[
        0] + rd * (np.cos(theta) * r[
        0] + np.sin(theta) * (n[
        2] * r[
        1] - n[
        1] * r[
        2]) + (1 - np.cos(theta)) * n[
        0] * (n[
        0] * r[
        0] + n[
        1] * r[
        1] + n[
        2] * r[
        2]))
    cy = c[
        1] + rd * (np.cos(theta) * r[
        1] + np.sin(theta) * (n[
        0] * r[
        2] - n[
        2] * r[
        0]) + (1 - np.cos(theta)) * n[
        1] * (n[
        0] * r[
        0] + n[
        1] * r[
        1] + n[
        2] * r[
        2]))
    cz = c[
        2] + rd * (np.cos(theta) * r[
        2] + np.sin(theta) * (n[
        1] * r[
        0] - n[
        0] * r[
        1]) + (1 - np.cos(theta)) * n[
        2] * (n[
        0] * r[
        0] + n[
        1] * r[
        1] + n[
        2] * r[
        2]))

    # Dibujar el círculo
    #ax.plot(cx, cy, cz, 'r-')

    v = np.column_stack((cx, cy, cz))

    # Mostrar el vector
  #  print(v)





    longitud = len(v)

    # Calcular el índice de corte para dividir en mitades
    indice_corte = longitud // 2

    # Separar la matriz en mitad superior e inferior
    mitad_superior = v[:indice_corte]

    indice_corte2=indice_corte
    mitad_inferior = v[indice_corte2:]

    # Mostrar las partes separadas
 #   print("Mitad Superior:")
   # print(mitad_superior)

  #  print("Mitad Inferior:")
    #print(mitad_inferior)
# Crear el gráfico 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Dibujar los puntos p1 y p2
    ax.scatter(p1[0], p1[1], p1[2], 'bo')
    ax.scatter(p2[0], p2[1], p2[2], 'bo')

    # Etiquetar los puntos p1 y p2
    ax.text(p1[0], p1[1], p1[2], 'P1')
    ax.text(p2[0], p2[1], p2[2], 'P2')

    # Dibujar el punto c
    ax.scatter(c[0], c[1], c[2], 'ro')

    # Etiquetar el punto c
    ax.text(c[0], c[1], c[2], 'C')


    # Obtener las coordenadas x, y, z de la matriz
    x1 = mitad_inferior[:, 0]
    y1 = mitad_inferior[:, 1]
    z1 = mitad_inferior[:, 2]


    # Dibujar el círculo
    ax.plot(x1, y1, z1, 'r-')



    # Obtener las coordenadas x, y, z de la matriz
    x11 = mitad_superior[:, 0]
    y11 = mitad_superior[:, 1]
    z11 = mitad_superior[:, 2]


    # Dibujar el círculo
    ax.plot(x11, y11, z11, 'b-')
    #Dibujar el punto c



    
    
    M_tray = []

    Tr0 = np.array([[0.0000, 1.0000, 0.0000, 0.0000],
                    [0.0000, 0.0000, -1.0000, 0.0000],
                    [-1.0000, 0.0000, 0.0000, 0.00],
                    [0, 0, 0, 1.0000]])




    thetaii=np.pi/2

    Tp_1=np.array([[0.0000, 0.0000, 0.0000, 0.0000],
                    [0.0000, 0.0000, 0.0000, 0.0000],
                    [0.0000, 0.0000, 0.0000, 0.00],
                    [0, 0, 0, 1.0000]])

    for i in range(len(mitad_inferior)):
        
        
        Tp_1[0:3, 3] = mitad_inferior[i].T
        

        pos_perimetro =  mitad_inferior[i].T
        pc01 = pc0

        # Traslada el sistema de coordenadas para que p0 sea el origen
        p2a = pos_perimetro - pc0

        # Calcula el ángulo con la línea horizontal. Usamos np.arctan2 para obtener un 
        # ángulo en el rango de -pi a pi.
        angulo = np.arctan2(p2a[0], p2a[2])

        # Normaliza el ángulo para estar en el rango de 0 a 360 grados.
        # if angulo < 0:
        #     angulo = angulo + np.pi

        theta = thetaii + angulo

        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta), 0],
            [0, 1, 0, 0],
            [-np.sin(theta), 0, np.cos(theta), 0],
            [0, 0, 0, 1]
        ])

        # Multiplicar la matriz TP_actual por la matriz de rotación
        TP_rotada = np.matmul(R_y, Tr0)

        Tp_1[0:3, 0:3] = TP_rotada[0:3, 0:3]

                 # Extrae las coordenadas x, y, y z de los puntos
        x_vals = [pc01[0], pos_perimetro[0]]
        y_vals = [pc01[1], pos_perimetro[1]]
        z_vals = [pc01[2], pos_perimetro[2]]

        # Dibuja una línea entre los dos puntos
        ax.plot(x_vals, y_vals, z_vals, color='b') # elige el color que prefieras

         
        
        M_tray.append(Tp_1.copy())
        
      
    
    fig.savefig('trayectoria_circular.png')
    
        # Mostrar la figura
   # plt.show()
        


    return M_tray


def fc(a, p0, q1, q2):
    return p0 + np.cos(a)[:, np.newaxis] * (q1 - p0) + np.sin(a)[:, np.newaxis] * (q2 - p0)



def angleFromPoint(p0, p, q1, q2):
    # Obtener el ngulo del crculo para el punto 'p'
    comp = lambda a, b: np.dot(a, b) / np.linalg.norm(b)
    ang = np.arctan2(comp(p - p0, q2 - p0), comp(p - p0, q1 - p0))
    return ang



def path_Arc(fc, a, b, p0, q1, q2):
    # Calcular arco de crculo entre los ngulos 'a' y 'b' para la funcin de cculo 'fc'
    while a > b:
        a = a - 2 * np.pi  # asegurarse de siempre ir de 'a' a 'b'

    aa = np.linspace(a, b, 100)
    c = fc(aa, p0, q1, q2)

    return c


def getCentre(p1, p2, p3):
    # Obtener el centro del crculo definido por los puntos 3D 'p1', 'p2' y 'p3'
    v1 = p2 - p1
    v2 = p3 - p1

    v11 = np.dot(v1, v1)
    v22 = np.dot(v2, v2)
    v12 = np.dot(v1, v2)

    b = 1 / (2 * (v11 * v22 - v12**2))
    k1 = b * v22 * (v11 - v12)
    k2 = b * v11 * (v22 - v12)

    p0 = p1 + k1 * v1 + k2 * v2
    return p0, np.argmax(np.isnan(p0))


def getNormal(p0, p1, p2, p3):
    # Calcular todas las normales en caso de que dos puntos sean colineales con el centro
    n12 = np.cross((p1 - p0), (p2 - p0))
    n23 = np.cross((p3 - p0), (p2 - p0))
    n13 = np.cross((p3 - p0), (p1 - p0))

    n = np.column_stack((n12, n23, n13))
    n = n / np.sign(n[0])
    idx = np.where(~np.isnan(n).all(axis=0))[0]
    n = n[:, idx[0]]
    n0 = n / np.linalg.norm(n)
    return n0


# p1 = [0.02, 0.01, 0.1]
# p2 = [0.02, 0.015, 0.1]
# Rd = 0.08

# M_tray = md_circulo_v2(p1, p2, Rd)
# print(M_tray)

# #_graficar

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Obtener las coordenadas x, y, z de los puntos
# x = [m[0, 3] for m in M_tray]
# y = [m[1, 3] for m in M_tray]
# z = [m[2, 3] for m in M_tray]

# # Crear una figura 3D
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Graficar los puntos 3D
# ax.scatter(x, y, z, c='r', marker='o')


# # Agregar los puntos p1 y p2 en verde
# ax.scatter(p1[0], p1[1], p1[2], c='g', marker='o')
# ax.scatter(p2[0], p2[1], p2[2], c='g', marker='o')



# # Configurar etiquetas de los ejes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Mostrar la figura
# plt.show()
