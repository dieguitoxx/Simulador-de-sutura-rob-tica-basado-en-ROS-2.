# -*- coding: utf-8 -*-
import time
import copy
import json

from scipy.interpolate import interp1d
import funciones_system as fs
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation
import subprocess
import re
import os

import pdb

from scipy.spatial.transform import Rotation as R
import pickle
import math 
import datetime

import numpy as np
from scipy.optimize import minimize




class DaVinciPreciseIK:
    def __init__(self):
        # Parámetros D-H exactos de la Tabla 7.1 (corregidos)
        self.dh_params = [
            {'theta': 0,       'alpha': np.pi/2,  'a': 0,      'd': 0},      # Joint 1
            {'theta': np.pi/2, 'alpha': np.pi/2,  'a': 0,      'd': 0},      # Joint 2
            {'theta': 0,       'alpha': 0,        'a': 0,      'd': 0.01},   # Joint 3 (d3 inicial)
            {'theta': np.pi,   'alpha': np.pi/2,  'a': 0,      'd': 0},      # Joint 4
            {'theta': np.pi/2, 'alpha': -np.pi/2, 'a': 0.0091, 'd': 0},      # Joint 5
            {'theta': np.pi/2, 'alpha': np.pi/2,  'a': 0.0102, 'd': 0}       # Joint 6
        ]

        self.joint_limits = [
            (-1, 1), (-0.7, 0.7), (0.01, 0.23),
            (-2.25, 2.25), (-1.57, 1.57), (-1.39, 1.39)
        ]

        # Matriz de transformación base a DH0 verificada
        self.base_to_dh0 = np.array([
            [0, 1, 0, 0],
            [0, 0, -1, 0],
            [-1, 0, 0, 0],
            [0, 0, 0, 1]
        ])

    def dh_transform(self, theta, alpha, a, d):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, q):
        T = self.base_to_dh0.copy()
        for i in range(6):
            params = self.dh_params[i].copy()
            if i == 2:  # Joint prismático
                params['d'] = q[i]  # d3 es directamente el valor de la articulación
            else:
                params['theta'] += q[i]

            T = T @ self.dh_transform(**params)
        return T

    def inverse_kinematics(self, target_pose, initial_guess=None, tol=1e-6):
        """Versión mejorada con múltiples intentos y ajustes finos"""
        def objective(x):
            current_pose = self.forward_kinematics(x)
            pos_error = np.linalg.norm(target_pose[:3,3] - current_pose[:3,3])

            # Error de orientación usando cuaterniones
            R_current = current_pose[:3,:3]
            R_target = target_pose[:3,:3]
            error_rot = 0.5 * np.trace(np.eye(3) - R_target.T @ R_current)

            return pos_error + 0.5 * error_rot  # Ponderación ajustada

        # Múltiples intentos con inicializaciones aleatorias
        best_solution = None
        min_error = float('inf')

        for _ in range(10):  # 10 intentos con semillas diferentes
            if initial_guess is None:
                x0 = np.array([np.random.uniform(low, high) for (low, high) in self.joint_limits])
            else:
                x0 = initial_guess + np.random.normal(0, 0.05, 6)  # Ruido gaussiano

            result = minimize(
                objective,
                x0,
                method='SLSQP',
                bounds=self.joint_limits,
                options={'maxiter': 500, 'ftol': tol}
            )

            if result.success and result.fun < min_error:
                best_solution = result.x
                min_error = result.fun

        # Ajuste fino con Levenberg-Marquardt
        if best_solution is not None:
            result = minimize(
                objective,
                best_solution,
                method='L-BFGS-B',
                bounds=self.joint_limits,
                options={'maxiter': 1000, 'ftol': 1e-8}
            )
            if result.success:
                return result.x

        return None









def iniciar():    
    print('Ejecutando..1.')
    maquina = MaquinaEstados()
    maquina.ejecutar()
    print('saliendo del programa   usar ctrl +c   .')
  #  rospy.spin()



def reubicar_robot(x,y,z):
    
       # Guardar las variables en un archivo
    with open('posiciones_xyzaa.pickle', 'wb') as archivo:
        pickle.dump((x, y, z), archivo)


    time.sleep(0.4)

    comando = ['python3 mr_posicionar6.py']
   # print(comando)
    try:
    # Ejecutar el comando y obtener la salida
        salida = subprocess.check_output(comando,  shell=True)

    # Imprimir la salida del programa 'suma'
        print(salida)
    except subprocess.CalledProcessError as e:
        print("Error possicionando':", e.output) 
     
        


def interpolate_matrix2(matrix, num_inter_points):
    # convertir la matriz a un array de numpy
    matrix = np.array(matrix)

    # obtener el número de filas
    num_rows = matrix.shape[0]

    # generar un nuevo conjunto de índices para la interpolación
    new_index = np.linspace(0, num_rows-1, num_inter_points*(num_rows-1)+1)

    # inicializar la matriz interpolada
    interp_matrix = np.zeros((len(new_index), 6))

    # para cada columna en la matriz
    for i in range(6):
        # crear una función de interpolación para la columna actual
        f = interp1d(np.arange(num_rows), matrix[:, i])

        # usar la función de interpolación para generar los nuevos puntos
        interp_matrix[:, i] = f(new_index)

    return interp_matrix


def convert_matrix(matrix_str):
    # Dividir la cadena en filas
    rows = matrix_str.split(';')
    
    # Convertir cada fila en una lista de floats
    matrix = np.array([list(map(float, row.split())) for row in rows])
    
    # Formatear la salida con notación científica
    np.set_printoptions(precision=8, suppress=False)
    
    return matrix



def calcular_ik(M_tray_endo_sale2,q_initr):

    ik_solver = DaVinciPreciseIK()
    q_vec_ik=[] 

#  for i in range(0, cant_rutas, 1):
    Ta = M_tray_endo_sale2 #[i]
    Ta = np.array(Ta)                    
    Ta_redondeada = np.round(Ta, 4)
    
    #pdb.set_trace()       # funcion para interrumpir y debugar
    
    T_target0 = '; '.join(' '.join(str(num) for num in fila) for fila in Ta_redondeada)

    
 #   print("antes de calcular",T_target0 )
    
    
        
        
        
  #  matrix_str = "-0.1315 0.9913 0.0 -0.0126; 0.0 0.0 -1.0 -0.0694; -0.9913 -0.1315 0.0 -0.1294; 0.0 0.0 0.0 1.0"
    converted_matrix = convert_matrix(T_target0)
 #   print(converted_matrix)
        
        
    
    #comando = ['./ik_endo_v2 '+'"'+matriz_cadena+'"']
    
    
    q_sol = ik_solver.inverse_kinematics(converted_matrix, q_initr)
    q_vec_ik_n=q_sol
    #comando = ['python3 mr_posicionar6.py']
    
    
    return q_vec_ik_n





class MaquinaEstados:
    def __init__(self):
        self.estados = ["Estado 1", "Estado 2", "Estado 3", "Estado 4", "Estado 5", "Estado 6"]
        self.estado_actual = 0

    def preguntar_usuario(self, pregunta):
        respuesta = input(pregunta)
        
        return respuesta.lower()
  

    
    

    def ejecutar(self):
    

        


    
        while True:
            estado_actual = self.estados[self.estado_actual]
        #    estado_actual2=estado_actual-1
            print("TE ENCUENTRAS EN EL ESTADO  ", estado_actual)
            
            if self.estado_actual == 0:
                print("Iniciando")
    
                print("estas en el estado 0")
                print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio")       


                                # Cargar las variables desde el archivo
                with open('datos.pickle', 'rb') as archivo:
                    angulo_inicialpinza, angulo_incremento,radio_aguja,velocidadr,velocidad_robot = pickle.load(archivo)
                index_puntadas=1

          
            # Conecctar con el sistema generador de puntos     


               
            elif self.estado_actual == 1:
            
                  print("Analizando heridas", )          
           



          
                
            elif self.estado_actual == 2:

                print("estas en el estado calcular trayectoria de aguja  ESPERA", )          
           


        

        
                #translation: 
                px= -0.13566123914606618
                py1= -0.22705395707400128
                pz= 0.40037829653455154
                #rotation: 
                rx= -0.003700219368508587
                ry= 0.9652759095765896
                rz= -0.26117893822394445
                rw= 0.003780078825329629





                #____________**** cambiar___________________________________________________

                rotMatrix = fs.quat2rotm([rw, rx, ry, rz])

                Ts_inicial3 = np.eye(4)
                Ts_inicial3[:3, :3] = rotMatrix
                Ts_inicial3[0, 3] = px
                Ts_inicial3[1, 3] = py1
                Ts_inicial3[2, 3] = pz

                print("posicion actual del robot",Ts_inicial3)
                
  

                Ts_inicial = Ts_inicial3     
                        
                print("posicion de trabajo",Ts_inicial)
                        
                        
                
                #se requieren transformaciones para que el fulcro y la aguja esten alineados para las correctas soluciones ****
                

                Dis_j6_fulcro=0.435  # dis de la base al fulcro
                dis_j5_ppinza=0.535   #long de pinza


                Tpq = np.array([[1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 1, Dis_j6_fulcro],
                                [0, 0, 0, 1]])
                Tpq2 = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0],
                                 [0, 0, 1, dis_j5_ppinza],
                                 [0, 0, 0, 1]])

                Tpfulcro = np.dot(Ts_inicial, Tpq)
                Tpinza = np.dot(Ts_inicial, Tpq2)

                PEe21ii = Tpfulcro[:3, 3]
                PEe2122 = Tpinza[:3, 3]

                PEe21 = -PEe21ii + PEe2122

      
                ppz = PEe21[2]  # Python usa indexación base 0, así que el tercer elemento es el índice 2

                if ppz > -0.05:
                    print('Inclinar más el robot')

                elif ppz < -0.13:
                    print('El robot está muy inclinado')

                elif -0.13 < ppz < -0.05:
                    pdif = ppz + 0.15
                    p1 = np.array([PEe21[0]-0.00410, PEe21[1]+0.00301, PEe21[2]-pdif])
                    p2 = np.array([PEe21[0]+0.0061, PEe21[1]+0.0030, PEe21[2]-pdif])


                dict_puntadas = {}

                puntada = []
                
                
                sub_z=0.009
                
                sub_y=-0.008
                
                p_au=[0,sub_y,sub_z]
                puntada_p1 = [a + b for a, b in zip(p1, p_au)]

    
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)





                i=1
                dict_puntadas['puntada_'+str(i)] = puntada

                
                            # Realiza la suma término a término utilizando zip y una comprensión de listas
                puntada = []


                sub_y=sub_y-0.008
                p_au=[0,sub_y,sub_z]
                puntada_p1 = [a + b for a, b in zip(p1, p_au)]

    
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=2
                dict_puntadas['puntada_'+str(i)] = puntada
                
                
                puntada = []
                

                sub_y=sub_y-0.008
                p_au=[0,sub_y,sub_z]
 
                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
      
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=3
                dict_puntadas['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)



                sub_y=sub_y-0.008
                p_au=[0,sub_y,sub_z]
 

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                puntada = []               
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=4
                dict_puntadas['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)
                


 
                sub_y=sub_y-0.008
                p_au=[0,sub_y,sub_z]
 

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                puntada = []               
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=5
                dict_puntadas['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)
     
 
#_______________________PARA CALIBRACION

                dict_puntadas_c = {}

                puntada = []
                
                
                puntada.append(p1)
                puntada.append(p2)
                i=1
                dict_puntadas_c['puntada_'+str(i)] = puntada

                
                            # Realiza la suma término a término utilizando zip y una comprensión de listas
                puntada = []

                sub_z=0
                
                p_au=[0,-0.005,sub_z]
                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                p_au=[0,-0.005,sub_z]
    
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=2
                dict_puntadas_c['puntada_'+str(i)] = puntada
                
                
                puntada = []
                

                p_au=[0.005,-0.005,sub_z]

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                p_au=[0.005,-0.005,sub_z]
    
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=3
                dict_puntadas_c['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)



                p_au=[0.005,-0.010,sub_z]

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                
                p_au=[0.005,-0.010,sub_z]
                
                
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                puntada = []               
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=4
                dict_puntadas_c['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)
                


                p_au=[-0.005,-0.010,sub_z]

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                puntada = []               
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=5
                dict_puntadas_c['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)
     

                p_au=[0,0.005,sub_z]

                puntada_p1 = [a + b for a, b in zip(p1, p_au)]
                puntada_p2 = [a + b for a, b in zip(p2, p_au)]
                puntada = []               
                puntada.append(puntada_p1)
                puntada.append(puntada_p2)
                i=6
                dict_puntadas_c['puntada_'+str(i)] = puntada
                print(" indice",i, " ", puntada_p1,"   ",puntada_p1)




#_________________________



 
                

                dict_rutas = {}
                dic_q_ik={}
                # Puntos deseados
                M_tray_endo2 = []

                # parte 2
                print('calculo de tratectoria..2.')

#___desde camara______


                longitud_dict = len(dict_puntadas)
    
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')





              #  m_q_vec_ik = m_q_vec_ik1[::-1]
                with open('data1.pickle', 'rb') as archivo:
                 data_cam= pickle.load(archivo)
                print("datos recibidos", data_cam)


                #_________

                # La transformación requerida
                transformed_data = []
                dict_puntadasr = {}
#                for index in range(1, len(data_cam)-2, 2):  # Recorremos la lista con pasos de 2 en 2
                for index in range(1, len(data_cam), 2):  # Recorremos la lista con pasos de 2 en 2
                    sublista1 = data_cam[index]
                    sublista2 = data_cam[index+1]
      


                    ccc1 = np.array(sublista1)/10000
                    ccc2 = np.array(sublista2)/10000
                    ccc2[2] += 0.001
                    ccc1[2] += 0.001  # mas grande mas arriba

                    ccc2[0] += 0.035  # mayor se mueve el robot adelante  sobre la herida mueve en y de grafica
                    ccc1[0] += 0.035



                    ccc2[1] += -0.002   # mas grande hacia el lado del borde
                    ccc1[1] += -0.002



                    p_cam1 = np.array([ccc1[1], -ccc1[0], -ccc1[2]])
                    p_cam2 = np.array([ccc2[1], -ccc2[0], -ccc2[2]])
                    arr_trasl = np.array([-0.0098, 0.0858, 0.0340])

                    p_robot1 = p_cam1 - arr_trasl
                    p_robot2 = p_cam2 - arr_trasl

                    transformed_data.append(p_robot1)
                    transformed_data.append(p_robot2)

                    # Crear y llenar el diccionario con las parejas de puntos
                    puntada = []
                    puntada.append(p_robot1)
                    puntada.append(p_robot2)
                    dict_puntadasr['puntada_'+str(index//2+1)] = puntada

       


                longitud_dict = len(dict_puntadasr)

                posicionespp=[]
                for i in range(1, longitud_dict+1):
                    print(i)
                    clave ='puntada_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    p11=dict_puntadasr[clave][0]
                    p22=dict_puntadasr[clave][1]

                    
                    p11 = [round(num, 6) for num in p11]
                    p22 = [round(num, 6) for num in p22]

                    print("reales p11", p11, " p2 " ,p22)

                    ax.scatter(*p11, c='g', marker='o')  # Dibuja el punto p11
                    ax.scatter(*p22, c='k', marker='o')  # Dibuja el punto p22
                    
                    posicionespp.append(p11)
                    posicionespp.append(p22)

                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')


                plt.show()

 
                # Convierte la lista de posiciones en una matriz numpy
                posicionespp = np.array(posicionespp)

      

                directory = "herida"

                # Comprueba si la carpeta ya existe, si no es así, crea una nueva
                if not os.path.exists(directory):
                    os.makedirs(directory)

                # Obtiene la fecha y hora actuales
                now = datetime.datetime.now()

                # Crea el nombre del archivo con la fecha y hora actuales
#                filename = 'posiciones_{}.txt'.format(now.strftime("%Y-%m-%d_%H-%M-%S"))

                filename = 'posiciones_.txt'


                # Crea la ruta del archivo uniendo el nombre de la carpeta y el nombre del archivo
                file_path = os.path.join(directory, filename)

                # Guarda la matriz en un archivo de texto
                np.savetxt(file_path, posicionespp)







                longitud_dict = len(dict_puntadas)



                posicionespp=[]
                for i in range(1, longitud_dict+1):
                    print(i)
                    clave ='puntada_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    p11=dict_puntadas[clave][0]
                    p22=dict_puntadas[clave][1]
                    
                    p11 = [round(num, 6) for num in p11]
                    p22 = [round(num, 6) for num in p22]

                    print("virtuales p11", p11, " p2 ",p22)


                    ax.scatter(*p11, c='r', marker='o')  # Dibuja el punto p11
                    ax.scatter(*p22, c='b', marker='o')  # Dibuja el punto p22
                    
                    posicionespp.append(p11)
                    posicionespp.append(p22)

  

                               
                
                    
                dict_puntadas_pp = dict_puntadasr    # dict_puntadas es la de calibracion
                
                
                # adicionar r para los puntos que llegan de afuera
                longitud_dict = len(dict_puntadas_pp)
                 
                
                
                for i in range(1, longitud_dict+1):
                    print(i)
                    clave ='puntada_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    p11=dict_puntadas_pp[clave][0]
                    p22=dict_puntadas_pp[clave][1]
                    
                    
                    
                    p11 = [round(num, 6) for num in p11]
                    p22 = [round(num, 6) for num in p22]
                    
                    print("puntada 1 p1:",p11 )
                    print("puntada 1 p2:", p22)
                    
                    p11 = np.array(p11)
                    p22 = np.array(p22) 
                    
                    Rd = radio_aguja  #0.0125
                    M_tray_endo2_1 = fs.md_circulo_v7(p11, p22, Rd)
                    # ** ojo esta version calcula la trayectoria en ciclos diferentes hay que
   #                 cant_rutas = len(M_tray_endo2_1)
                    dict_rutas['rutas_'+str(i)] = M_tray_endo2_1
                    
                    
                
                
                
                longitud_dict_rutas = len(dict_rutas)
                print("La longitud del diccionario es:", longitud_dict_rutas)
                               
                
               # print("cantidad de tray salieron",cant_rutas)
                print('Ejecutando..4. tray')
                print("estas en el estado 2 calcular trayectoria de aguja", )
                print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio") 
                  
        

             
            elif self.estado_actual == 3:
            
                print("estas en el estado Calculando ik endo")
                
# _______________estado 3  calculo cinematica inversa


                q_vec_ik1   = np.array([])

                i = 1  # O cualquier valor que desees para el archivo

                for i in range(1, longitud_dict_rutas+1):
   
                    print("el ciclo principali",i)
      
                    clave ='rutas_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    M_tray_endo_sale=dict_rutas[clave]   # aqui hay que adicionar que el robot suba y baje ***
                    ln_M_tray_endo_sale=len(M_tray_endo_sale)
                    print("cantidad de rutas",ln_M_tray_endo_sale)               
                   # print("matrices______________",M_tray_endo_sale)                    
                    with open('matrices_Sal.pickle', 'wb') as archivo:
                        pickle.dump((M_tray_endo_sale), archivo)


        
                    m_q_vec_ik1a = np.array([])      
                    q_init = [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]
                    for i2 in range(1, ln_M_tray_endo_sale):
                    
     #                   print("el ciclo i2",i2)
                        M_tray_endo_sale_r=M_tray_endo_sale[i2]
                 #       print("matriz para calcular ik ",M_tray_endo_sale_r)


                     
                        q_vec_ik1=calcular_ik(M_tray_endo_sale_r,q_init)
                        

              #          pdb.set_trace()    

                        q_vec_ik1 = np.array(q_vec_ik1)
                                    
                        q_init=q_vec_ik1

                        # Apilar verticalmente el array a la matriz de resultados
                        if m_q_vec_ik1a.size == 0:  # Si la matriz está vacía, asignar el primer array
                            m_q_vec_ik1a = q_vec_ik1
         
                            
                        elif q_vec_ik1.size != 0:  # Si no está vacío, apilar el array debajo de la matriz
                            m_q_vec_ik1a = np.vstack((m_q_vec_ik1a, q_vec_ik1))
                            
                  #  print("sol ik encontradas   ", len(m_q_vec_ik1a), "estas",m_q_vec_ik1a)
                    
                    
                    #sitio para aumentar la densidad de puntos intermedios
                                    

                    # Crear una copia de la matriz original para no modificar la original
                    m_q_vec_ik1a_aux1 = np.copy(m_q_vec_ik1a)

                    # Modificar la primera columna si el valor es > 1
                    # Crear una máscara booleana para las filas que cumplen con la condición
                    mask = m_q_vec_ik1a_aux1[:,3] <= 0.5

                    # Utilizar la máscara para seleccionar solo las filas que queremos mantener
                    m_q_vec_ik1a_aux1 = m_q_vec_ik1a_aux1[mask]
        
                    # Esta es la matriz original
                    matrix = m_q_vec_ik1a_aux1
               #     segment_2_au = interpolate_matrix2(matrix, num_inter_points=8)  # el num representa la cantidad de puntos intemedios



                    # Calcular los índices de los segmentos
                    total_elements = len(matrix)
                    index_33 = int(total_elements * 0.35)
                    index_40 = int(total_elements * 0.55)



                    # Dividir la matriz en segmentos
                    segment_1 = matrix[:index_33+1]
                    segment_2 = matrix[index_33:index_33+index_40]
                    segment_3 = matrix[index_33+index_40:]
                                  

                    # # Llamada a la función  aumentar la densidad de puntos intemedios
                    segment_1_au = interpolate_matrix2(segment_1, num_inter_points=18)  # el num representa la cantidad de puntos intemedios

                    # Llamada a la función  aumentar la densidad de puntos intemedios
                    segment_2_au = interpolate_matrix2(segment_2, num_inter_points=10)  # el num representa la cantidad de puntos intemedios
                    # # Llamada a la función  aumentar la densidad de puntos intemedios
                    # segment_3_au = interpolate_matrix2(segment_3, num_inter_points=7)  # el num representa la cantidad de puntos intemedios
                    # # Unir los segmentos
# #                    m_q_vec_ik1a_aumentado = np.concatenate((segment_1_au, segment_2_au, segment_3_au), axis=0)
                    m_q_vec_ik1a_aumentado = np.concatenate((segment_1_au, segment_2_au), axis=0)
              #      m_q_vec_ik1a_aumentado = segment_2_au

   
                    directory = "herida"

                    # Comprueba si la carpeta ya existe, si no es así, crea una nueva
                    if not os.path.exists(directory):
                        os.makedirs(directory)

                    # Concatena el nombre del archivo con la variable 'i'
                    filename = 'matrizm_q_vec_ik_{}.npy'.format(i)

                    # Crea la ruta del archivo uniendo el nombre de la carpeta y el nombre del archivo
                    file_path = os.path.join(directory, filename)

                    # Guarda la matriz en el archivo
                    np.save(file_path, m_q_vec_ik1a_aumentado)
                    print("guardando 555555855")









                    dic_q_ik['vect_q_ik_'+str(i)] = m_q_vec_ik1a_aumentado       
                        
                    
                              
                print("estas en el estado 3 Calculando ik endo")
                    
                print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio") 
                
  
            

            accion = self.preguntar_usuario("Ingresa 'p' para pasar al siguiente estado, 'a' para regresar al estado anterior, 'r' para volver al mismo estado, e0 estado 0 y 'x' para salir: ")
          

            if accion == "p":
                self.estado_actual = (self.estado_actual + 1) % len(self.estados)
            elif accion == "a":
                self.estado_actual = (self.estado_actual - 1) % len(self.estados)
            elif accion == "r":
                print("repitiendo estado")
                index_puntadas=index_puntadas                
                self.estado_actual = (self.estado_actual) % len(self.estados)
                pass
                
            elif accion == "w":
                index_puntadas=index_puntadas-1
                self.estado_actual 

                
            elif accion == "x":
                print("Saliendo del programa...")
                break
            elif accion == "e0":
                self.estado_actual = 0
            else:
                print("Acción inválida. Ingresa 'p', 'a', 'r', 'x' o 'e0'.")






    
if __name__ == '__main__':
    #rospy.init_node('system_py')
    iniciar()

