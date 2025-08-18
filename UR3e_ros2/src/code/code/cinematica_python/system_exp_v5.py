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





def iniciar():    
    print('Ejecutando..1.')
    maquina = MaquinaEstados()
    maquina.ejecutar()
    print('saliendo del programa   usar ctrl +c   .')
#    rospy.spin()



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





def hacer_posicion_ini(m_efector,m_q_vec_ik,despl):

    # desplazamiento permite mover el robot en el eje y

    # Ahora la matriz está almacenada en la variable "m_q_vec_ik_cargada"
    #print(m_efector)
  #  print(" fila 5 completa")
    poseArrayMsg = PoseArray()
    poseArrayMsg_ini = PoseArray()
    poseArrayMsg_inv = PoseArray()
    poseArrayMsg_fin_arriba = PoseArray()
    poseArrayMsg_ini_arriba = PoseArray()
        


    cont_q=len(m_efector)
#________________
    

    for i in range(1, cont_q + 1):
        posES = Pose()

        Tur3 = m_efector[i - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
    #    print(" matrix ")   
      
   #     print(Tur3)
        
        
   #     print(" arreglo con datos")
        # Imprimir el arreglo
                
        
        posES.position.x = Tur3[0, 3]
        posES.position.y = Tur3[1, 3]  #+despl
        posES.position.z = Tur3[2, 3]

        rotm = Tur3[:3, :3]
        quat = fs.rotm2quat(rotm)

        posES.orientation.x = quat[1]
        posES.orientation.y = quat[2]
        posES.orientation.z = quat[3]
        posES.orientation.w = quat[0]

        poseArrayMsg.poses.append(posES)
        
        if i == 1:
         TiniT = Tur3.copy()
   
  #  time.sleep(8)   

  #  pdb.set_trace()       # funcion para interrumpir y debugar
  
 

    last_pose = poseArrayMsg.poses[-1]
  
    last_pose.position.y=    last_pose.position.y+0.030
 
    poseArrayMsg_fin_arriba.poses.append(last_pose)
    pub_traj.publish(poseArrayMsg_fin_arriba)  # Publish the PoseArray message
    
    



def hacer_puntada_virtual(m_efector,m_q_vec_ik,despl):

    # desplazamiento permite mover el robot en el eje y

    # Ahora la matriz está almacenada en la variable "m_q_vec_ik_cargada"
    #print(m_efector)
  #  print(" fila 5 completa")
    poseArrayMsg = PoseArray()
    poseArrayMsg_ini = PoseArray()
    poseArrayMsg_inv = PoseArray()
    poseArrayMsg_fin_arriba = PoseArray()
    poseArrayMsg_ini_arriba = PoseArray()
        


    cont_q=len(m_efector)
#________________
    

    for i in range(1, cont_q + 1):
        posES = Pose()

        Tur3 = m_efector[i - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
    #    print(" matrix ")   
      
   #     print(Tur3)
        
        
   #     print(" arreglo con datos")
        # Imprimir el arreglo
                
        
        posES.position.x = Tur3[0, 3]
        posES.position.y = Tur3[1, 3]  #+despl
        posES.position.z = Tur3[2, 3]

        rotm = Tur3[:3, :3]
        quat = fs.rotm2quat(rotm)

        posES.orientation.x = quat[1]
        posES.orientation.y = quat[2]
        posES.orientation.z = quat[3]
        posES.orientation.w = quat[0]

        poseArrayMsg.poses.append(posES)
        
        if i == 1:
         TiniT = Tur3.copy()
   
  #  time.sleep(8)   

  #  pdb.set_trace()       # funcion para interrumpir y debugar
  
  
    poseArrayMsg_inv = copy.deepcopy(poseArrayMsg)
    poseArrayMsg_inv.poses.reverse()  
      

    
    first_pose = poseArrayMsg_inv.poses[0]
    first_pose_arr = copy.deepcopy(first_pose)
    first_pose_arr.position.z=    first_pose_arr.position.z+0.006
   
    t_sincronia=5
    
    poseArrayMsg_ini.poses.append(first_pose)
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)

 #   pdb.set_trace()       # funcion para interrumpir y debugar

    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(8)
   
   
    pub_traj.publish(poseArrayMsg_ini)  # Publish the PoseArray message
    time.sleep(8)
    
 
    m_q_vec_ik_in=m_q_vec_ik[::-1]  # movinmiento de regresar por donde ingresa

    m_efector_inv=m_efector[::-1]  # movinmiento de regresar por donde ingresa
    
    publicar_virtual(m_q_vec_ik_in,m_efector_inv,1)
        
    tiempo_eje_robot=8
    time.sleep(4)

    pub_traj.publish(poseArrayMsg_inv)  # Publish the PoseArray message
    time.sleep(t_sincronia)  
    publicar_virtual(m_q_vec_ik_in,m_efector_inv,2)

    time.sleep(5)

 #   time.sleep(9.42)
    pub_traj.publish(poseArrayMsg)  # Publish the PoseArray message
    time.sleep(5)
    publicar_virtual(m_q_vec_ik,m_efector,2)
    
 
    time.sleep(7)

   
    last_pose = poseArrayMsg.poses[-1]

    last_pose.position.z=    last_pose.position.z+0.006
 
    poseArrayMsg_fin_arriba.poses.append(last_pose)
    pub_traj.publish(poseArrayMsg_fin_arriba)  # Publish the PoseArray message
    time.sleep(5)
    
   


def hacer_puntada_alin(m_efector,m_q_vec_ik,despl):

    # desplazamiento permite mover el robot en el eje y

    # Ahora la matriz está almacenada en la variable "m_q_vec_ik_cargada"
    #print(m_efector)
  #  print(" fila 5 completa")
    poseArrayMsg = PoseArray()
    poseArrayMsg_ini = PoseArray()
    poseArrayMsg_inv = PoseArray()
    poseArrayMsg_fin_arriba = PoseArray()
    poseArrayMsg_ini_arriba = PoseArray()
        


    cont_q=len(m_efector)
#________________
    

    for i in range(1, cont_q + 1):
        posES = Pose()

        Tur3 = m_efector[i - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
    #    print(" matrix ")   
      
   #     print(Tur3)
        
        
   #     print(" arreglo con datos")
        # Imprimir el arreglo
                
        
        posES.position.x = Tur3[0, 3]
        posES.position.y = Tur3[1, 3]  #+despl
        posES.position.z = Tur3[2, 3]

        rotm = Tur3[:3, :3]
        quat = fs.rotm2quat(rotm)

        posES.orientation.x = quat[1]
        posES.orientation.y = quat[2]
        posES.orientation.z = quat[3]
        posES.orientation.w = quat[0]

        poseArrayMsg.poses.append(posES)
        
        if i == 1:
         TiniT = Tur3.copy()
   
  #  time.sleep(8)   

  #  pdb.set_trace()       # funcion para interrumpir y debugar
  
  
    poseArrayMsg_inv = copy.deepcopy(poseArrayMsg)
    poseArrayMsg_inv.poses.reverse()  
      

    
    first_pose = poseArrayMsg_inv.poses[0]
    first_pose_arr = copy.deepcopy(first_pose)
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)
    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(7)

    first_pose_arr.position.y=    first_pose_arr.position.y-0.005
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)
    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(7)


    first_pose_arr.position.y=    first_pose_arr.position.y-0.01
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)
    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(7)


    first_pose_arr.position.y=    first_pose_arr.position.y-0.015
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)
    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(7)

    
     


def hacer_puntada(m_efector,m_q_vec_ik,despl):

    # desplazamiento permite mover el robot en el eje y

    # Ahora la matriz está almacenada en la variable "m_q_vec_ik_cargada"
    #print(m_efector)
  #  print(" fila 5 completa")
    poseArrayMsg = PoseArray()
    poseArrayMsg_ini = PoseArray()
    poseArrayMsg_inv = PoseArray()
    poseArrayMsg_fin_arriba = PoseArray()
    poseArrayMsg_ini_arriba = PoseArray()
        


    cont_q=len(m_efector)
#________________
    

    for i in range(1, cont_q + 1):
        posES = Pose()

        Tur3 = m_efector[i - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
    #    print(" matrix ")   
      
   #     print(Tur3)
        
        
   #     print(" arreglo con datos")
        # Imprimir el arreglo
                
        
        posES.position.x = Tur3[0, 3]
        posES.position.y = Tur3[1, 3]  #+despl
        posES.position.z = Tur3[2, 3]

        rotm = Tur3[:3, :3]
        quat = fs.rotm2quat(rotm)

        posES.orientation.x = quat[1]
        posES.orientation.y = quat[2]
        posES.orientation.z = quat[3]
        posES.orientation.w = quat[0]

        poseArrayMsg.poses.append(posES)
        
        if i == 1:
         TiniT = Tur3.copy()
   
  #  time.sleep(8)   

  #  pdb.set_trace()       # funcion para interrumpir y debugar
  
  
    poseArrayMsg_inv = copy.deepcopy(poseArrayMsg)
    poseArrayMsg_inv.poses.reverse()  
      

    
    first_pose = poseArrayMsg_inv.poses[0]
    first_pose_arr = copy.deepcopy(first_pose)
    first_pose_arr.position.z=    first_pose_arr.position.z+0.009
   
    t_sincronia=5
    
    poseArrayMsg_ini.poses.append(first_pose)
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)

 #   pdb.set_trace()       # funcion para interrumpir y debugar

    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    time.sleep(8)
   
   
    pub_traj.publish(poseArrayMsg_ini)  # Publish the PoseArray message
    time.sleep(8)
    
 
    m_q_vec_ik_in=m_q_vec_ik[::-1]  # movinmiento de regresar por donde ingresa

    m_efector_inv=m_efector[::-1]  # movinmiento de regresar por donde ingresa
    
    publicar_endo(m_q_vec_ik_in,m_efector_inv,1)
        
    tiempo_eje_robot=8
    time.sleep(4)

    pub_traj.publish(poseArrayMsg_inv)  # Publish the PoseArray message
    time.sleep(t_sincronia)  
    publicar_endo(m_q_vec_ik_in,m_efector_inv,2)

    time.sleep(10)

 #   time.sleep(9.42)
    pub_traj.publish(poseArrayMsg)  # Publish the PoseArray message
    time.sleep(5)
    publicar_endo(m_q_vec_ik,m_efector,2)
    
 
    time.sleep(5)

   
    last_pose = poseArrayMsg.poses[-1]

    last_pose.position.z=    last_pose.position.z+0.009
 
    poseArrayMsg_fin_arriba.poses.append(last_pose)
    pub_traj.publish(poseArrayMsg_fin_arriba)  # Publish the PoseArray message

      




def hacer_puntada_calib(m_efector,m_q_vec_ik,despl):

    # desplazamiento permite mover el robot en el eje y

    # Ahora la matriz está almacenada en la variable "m_q_vec_ik_cargada"
    #print(m_efector)
  #  print(" fila 5 completa")
    poseArrayMsg = PoseArray()
    poseArrayMsg_ini = PoseArray()
    poseArrayMsg_inv = PoseArray()
    poseArrayMsg_fin_arriba = PoseArray()
    poseArrayMsg_ini_arriba = PoseArray()
        


    cont_q=len(m_efector)
#________________
    

    for i in range(1, cont_q + 1):
        posES = Pose()

        Tur3 = m_efector[i - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
    #    print(" matrix ")   
      
   #     print(Tur3)
        
        
   #     print(" arreglo con datos")
        # Imprimir el arreglo
                
        
        posES.position.x = Tur3[0, 3]
        posES.position.y = Tur3[1, 3]  #+despl
        posES.position.z = Tur3[2, 3]

        rotm = Tur3[:3, :3]
        quat = fs.rotm2quat(rotm)

        posES.orientation.x = quat[1]
        posES.orientation.y = quat[2]
        posES.orientation.z = quat[3]
        posES.orientation.w = quat[0]

        poseArrayMsg.poses.append(posES)
        
        if i == 1:
         TiniT = Tur3.copy()
   
  #  time.sleep(8)   

  #  pdb.set_trace()       # funcion para interrumpir y debugar
  
  
    poseArrayMsg_inv = copy.deepcopy(poseArrayMsg)
    poseArrayMsg_inv.poses.reverse()  
      

    
    first_pose = poseArrayMsg_inv.poses[0]
    first_pose_arr = copy.deepcopy(first_pose)
    first_pose_arr.position.z=    first_pose_arr.position.z+0.006
    first_pose_arr.position.y=    first_pose_arr.position.y-0.015

   
    t_sincronia=5
    
    poseArrayMsg_ini.poses.append(first_pose)
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)

 #   pdb.set_trace()       # funcion para interrumpir y debugar

    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    
    first_pose_arr.position.z=    first_pose_arr.position.z+0.008


   
    t_sincronia=5
    
    poseArrayMsg_ini.poses.append(first_pose)
    poseArrayMsg_ini_arriba.poses.append(first_pose_arr)

 #   pdb.set_trace()       # funcion para interrumpir y debugar

    pub_traj.publish(poseArrayMsg_ini_arriba)  # Publish the PoseArray message
    
  
   
   
    




def publicar_endo(m_q_vec_ik,m_efector2,modo):




  #  m_q_vec_ik = m_q_vec_ik1[::-1]
    with open('datos.pickle', 'rb') as archivo:
     angulo_inicialpinza, angulo_incremento,radio_aguja,velocidadr, velocidad_robot= pickle.load(archivo)


  #  print("publicar")


 #  rospy.init_node('publicador', anonymous=True) #inicia el nodo publicador
    # rate = rospy.Rate(0.5) # 0.5 Hz
    
    # pub_endo = rospy.Publisher('/endowrist', JointState, queue_size=10)
    # roll = rospy.Publisher('/EndoWrist/servo1/target', Int16, queue_size=10) #crea el publicador en el topic hola_mundo y tipo de mensaje int16
    # yaw = rospy.Publisher('/EndoWrist/servo2/target', Int16, queue_size=10)
    # pitch_2 = rospy.Publisher('/EndoWrist/servo4/target', Int16, queue_size=10)
    # pitch_1 = rospy.Publisher('/EndoWrist/servo3/target', Int16, queue_size=10)


    # pub_endoMsg = JointState()

    # pub_endoMsg.header.frame_id = 'base_link'
    # pub_endoMsg.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']

    # pub_endo = rospy.Publisher('/endowrist', JointState, queue_size=10)

   # rospy.loginfo("Estoy publicando en el topic hola_mundo") #escribo en log
            # Definir el rango de posiciones del servo en microsegundos
    servo_min = 1000 # 0 grados
    servo_max = 2000 # 180 grados

    # Definir el rango de ángulos en radianes
    angle_min = -2.35619 # -135 grados
    angle_max = 2.35619 # 135 grados
    
    
     
    def interpolate_matrix(matrix, num_inter_points=10):
        # convertir la matriz a un array de numpy
        matrix = np.array(matrix)

        # obtener el número de filas
        num_rows = matrix.shape[0]

        # generar un nuevo conjunto de índices para la interpolación
        new_index = np.linspace(0, num_rows-1, num_inter_points*(num_rows-1)+1)

        # inicializar la matriz interpolada
        interp_matrix = np.zeros((len(new_index), 3))

        # para cada columna en la matriz
        for i in range(3):
            # crear una función de interpolación para la columna actual
            f = interp1d(np.arange(num_rows), matrix[:, i])

            # usar la función de interpolación para generar los nuevos puntos
            interp_matrix[:, i] = f(new_index)

        return interp_matrix

       
        
    
    def convert_range(tstatus):
        # Define tus rangos
        input_start = 1984
        input_end = 9600
        output_start = 500
        output_end = 2480

        # Calcula la proporción de escalado
        scale = float(output_end - output_start) / float(input_end - input_start)

        # Aplica la conversión de rango
        output2 = output_start + ((tstatus - input_start) * scale)



        return output2

    
    
    def get_status():
    
    
             



            # Ejecutar el comando y capturar la salida
            output = subprocess.check_output("./maestro-linux/UscCmd --status", shell=True)

            # Dividir la salida en líneas y eliminar el encabezado
            sequence = output.split('\n')[1:-2]



            start_index = None
            end_index = None

            # Buscar el índice de inicio y fin del contenido deseado
            for i, line in enumerate(sequence):
                if '#' in line:
                    start_index = i
                if 'error' in line:
                    end_index = i
                    break

            # Extraer el contenido entre los índices encontrados
            content = sequence[start_index:end_index]



            # Imprimir el contenido
            i=1
            matrix = []
            for line in content:
                i=i+1
              

                data_string =line # " 0    5854       0       0    5854\n 1    5919       0       0    5919\n 2    5760       0       0    5760\n 3     287       0       0     283\n 4     352       0       0     352\n 5     362       0       0     362\n"

                # Dividir el string en líneas
                lines = data_string.split('\n')

                # Crear una matriz para almacenar los valores numéricos


                # Leer cada línea y almacenar los valores numéricos en la matriz
                for line2 in lines:
                    if line2.strip():
                        values = line2.split()
                        numeric_values = [int(value) for value in values]
                        matrix.append(numeric_values)

      #      print("Imprimir la matriz")
            numpy_matrix = np.array(matrix)

            col4=numpy_matrix[:,4]
            
            
            
            
            lcol4=len(col4)


            mcol4=[]
            for i2 in range(lcol4):

                pos_m2=convert_range(int(col4[i2]))
                mcol4.append(int(pos_m2))

            
            return mcol4
           
                
   



    def compensar_th3(x_deseado,th3d):
      


        # Definimos los datos de la matriz.
        matrix = [
            [500, 642.00],
            [610, 721.50],
            [720, 801.00],
            [830, 880.50],
            [940, 960.00],
            [1050, 1039.50],
            [1160, 1119.00],
            [1270, 1198.50],
            [1380, 1278.00],
            [1490, 1357.50],
            [1600, 1437.00],
            [1710, 1516.50],
            [1820, 1596.00],
            [1930, 1675.50],
            [2040, 1755.00],
            [2150, 1834.50],
            [2260, 1914.00],
            [2370, 1993.50],
            [2480, 2073.00]
        ]

        # Separamos los datos en dos listas: x_vals para la primera columna y y_vals para la segunda.
        x_vals, y_vals = zip(*matrix)

        def obtener_valor_intermedio(x_deseado):
            # Usamos la función de interpolación para encontrar y_deseado.
            y_deseado = np.interp(x_deseado, x_vals, y_vals)
            return y_deseado
         

        # Obtenemos y mostramos el resultado
        y_deseado = obtener_valor_intermedio(x_deseado)
        th3centro=y_deseado
      #  th3p=0

        if th3d >= 1490:
            dif = th3d - 1490
            th3p = th3centro + dif
            
            
            if th3p > 2480:
                th3p = 2480

            
        elif th3d < 1490:
            dif = 1490 - th3d
            th3p = th3centro - dif

            if th3p < 490:
                th3p = 490

            
        # elif th3p > (th3centro+1000):

            # th3p = th3centro +1000


            
        # elif th3p < (th3centro-1000):

            # th3p = th3centro -1000   



        return th3p



    def th_to_val_join_th2(th):
        x1, y1 = 1.57, 500
        x2, y2 = -1.57, 2480

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2480), 500)
        
        val11=int(val_join)-60

        return val11



    def th_to_val_join_th3(th):
        x1, y1 = 1.57, 585.25
        x2, y2 = -1.57, 2260.25

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2480), 500)
        
        val11=int(val_join)+242

        return val11





    def th_to_val_join_roll(th):
        x1, y1 = -5.1051, 500
        x2, y2 = 0, 2480

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2400), 736)
        
        val11=int(val_join)

        return val11


      
    m_q_vec_ik_456 = m_q_vec_ik[:,3:6]

        

    tiem_ini=0.08
    frac=70  # saltos
                          
   # print("Punto de interrupción")
        # pdb.set_trace()       # funcion para interrumpir y debugar
    lista2 = [0, 0,0]
    if modo==1:
        
        sublista_fila2 = m_q_vec_ik_456[1,:]
        fila = [a + b for a, b in zip(sublista_fila2, lista2)]
        # Imprime el resultado
      
        a1 = fila[0]
        a1 = th_to_val_join_roll(a1)

        a2 = fila[1]
        a2 = th_to_val_join_th2(a2)
        a3 = fila[2]
        a3 = th_to_val_join_th3(a3)
        
        pactual=get_status()
        pm1=pactual[0]      
        pm2=pactual[1]      
        pm3=pactual[2]              
        

        dm1=((a1-pm1)/frac)
        dm2=((a2-pm2)/frac)
        dm3=((a3-pm3)/frac)
        
        pdm1=pm1
        pdm2=pm2
        pdm3=pm3
        

        for i in range(1, frac+1):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.
        
            pdm1=pdm1+dm1
            pdm2=pdm2+dm2
            pdm3=pdm3+dm3



            th3pp=compensar_th3(pdm2,pdm3)    




            yaw.publish(pdm2)
            pitch_1.publish(th3pp)     
           

   #         if 736 < pdm1 < 1900:
            roll.publish(pdm1)
            
            
            time.sleep(tiem_ini)


        

        
        # a = fila[3])
        # pitch_2.publish(a)
        mandi=3.14  # en el simulador se esconde la mordaza que no se usa
        posiciones =fila#  offset para calinrar el mov del mo
        
        mandi_iz=posiciones[2]
        mandi_de=-posiciones[2]
        
        # velocidad_endows=[0.0, 0.0, 0.0, 0.0]
        # pub_endoMsg.header.seq = 1
        # pub_endoMsg.header.stamp = rospy.Time.now()
        # pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
        # pub_endoMsg.velocity = velocidad_endows
        # pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
        # # Publicar el mensaje en el tópico '/endowrist'
        # pub_endo.publish(pub_endoMsg)
   #     time.sleep(0.1)

        print("pos inicial")
   #     pdb.set_trace()    

    elif modo == 2:        

                    
            len_m_efector2=len(m_q_vec_ik)
            print("soluc",len_m_efector2)
            len_m_ef22=len(m_efector2)
       #     pdb.set_trace()       # funcion para interrumpir y debugar          
            tiempo=0
            for i in range(len_m_efector2):
                

    #            velocidadr=0.02
         
                if i>1:
                
                    T_a  = m_efector2[i]      
                    T_a2  = m_efector2[i-1]      
                    posxy1=T_a[:3, 3]
                    posxy2=T_a2[:3, 3]                    
                    
      #              print("1    ",posxy1,"       2      ",posxy2)
                
                    dx = posxy1[0]-posxy2[0]  
                    dy = posxy1[1]-posxy2[1]  
                    dz = posxy1[2]-posxy2[2]  
                    
                    
                    
                    Distancia = math.sqrt((dx**2) + (dy**2) + (dz**2))
         #           print(Distancia)
                    if Distancia>0:
                        tiempo = (Distancia/(velocidadr))*1
                        
       #                 print("tiempo calculado",tiempo)
                    else:
                        tiempo = 0.11
         #               print("soluc dis")
                    retardo=tiempo  
                else:
                    retardo=0.1   # tiempo de retardo sirve para sincronizar
     #               print(" lanza endo  ", time.time())
                
              #  retardo2=0.01
                
         
       #         print("vel   ", velocidadr,"tiempo  ", retardo)



                
              #  retardo=0.1
                cpuntos=m_q_vec_ik_456.shape[0]
                sublista_fila2 = m_q_vec_ik_456[i, :]
                

                # Realiza la suma término a término utilizando zip y una comprensión de listas
                fila = [a + b for a, b in zip(sublista_fila2, lista2)]
                # Imprime el resultado
    #            print(fila)
        #        fila = m_q_vec_ik[i]
           #     print("ciclo",i, " conte",fila)
                a1 = fila[0]
                a1 = th_to_val_join_roll(a1) #+100  # ajuste para penetracion **
                a2 = fila[1]
                a2 = th_to_val_join_th2(a2)
                a3 = fila[2]
                a3 = th_to_val_join_th3(a3)
           #     pactual=get_status()


                th3pp=compensar_th3(a2,a3)    


     #           pdb.set_trace()       # funcion para interrumpir y debugar
  

    
                roll.publish(a1)
                yaw.publish(a2)
                pitch_1.publish(th3pp)
                
                inicio = time.time()               
                
                posiciones =fila#  offset para calinrar el mov del mo
        
                mandi_iz=-posiciones[2]

        #        mandi_iz=0
                               
                mandi_de=3.14 #posiciones[2]
                
                velocidad_endows=[0.0, 0.0, 0.0, 0.0]
                pub_endoMsg.header.seq = 1
                pub_endoMsg.header.stamp = rospy.Time.now()
                pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
 #               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

                pub_endoMsg.velocity = velocidad_endows
                pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
                # Publicar el mensaje en el tópico '/endowrist'
                pub_endo.publish(pub_endoMsg)
            
                
                        


            
                fin = time.time()
                tiem_tar=fin - inicio
      #          print("La funcin tard segundos en ejecutarse.",tiem_tar)                
                
           #     time.sleep(0.2)
                       
                time.sleep(retardo)
          

    elif modo == 3:
        # este modo es para dejar el robot en la posicion inicial siempre
        


        a1 = 1450
        a2 = 1450
        a3 = 1450

        
        pactual=get_status()
        pm1=pactual[0]      
        pm2=pactual[1]      
        pm3=pactual[2]              
        
        dm1=((a1-pm1)/frac)
        dm2=((a2-pm2)/frac)
        dm3=((a3-pm3)/frac)
        
        pdm1=pm1
        pdm2=pm2
        pdm3=pm3
        
        
        for i in range(1, frac+1):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.
        
            pdm1=pdm1+dm1
            pdm2=pdm2+dm2
            pdm3=pdm3+dm3
         #   roll.publish(pdm1)
            yaw.publish(pdm2)
            pitch_1.publish(pdm3)    

            roll.publish(pdm1)
       #     time.sleep(0.05)
        
        # a = fila[3])
        # pitch_2.publish(a)
            mandi=3.14  # en el simulador se esconde la mordaza que no se usa

      
        
        # a = fila[3])
        # pitch_2.publish(a)
            mandi=3.14  # en el simulador se esconde la mordaza que no se usa
          
            velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            pub_endoMsg.header.seq = 1
            pub_endoMsg.header.stamp = rospy.Time.now()
            pub_endoMsg.position = [0, 0,0 , mandi]
            pub_endoMsg.velocity = velocidad_endows
            pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # Publicar el mensaje en el tópico '/endowrist'
            pub_endo.publish(pub_endoMsg)
            

            time.sleep(tiem_ini)



    elif modo == 4:
        # modo de sincronizacion roll y yau
        

        # Generar una secuencia desde 0 hasta pi/2 con pasos de pi/24
      #  sequence = np.arange(0, np.pi/3 + np.pi/24, np.pi/24)
        sequence = np.arange(0, np.pi/4 + np.pi/24, np.pi/24)


        # Redondear a 4 decimales
        sequence = np.round(sequence, 4)
        sequence=sequence* -1
        # Imprimir la secuencia
        print(sequence)

        tiem_ini=0.7

        le_sec=len(sequence)
        print(le_sec)
        

       
        for i in range(0,le_sec):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.




            por_roll=sequence[i]
  #          print(i)

      #      a1 = th_to_val_join_roll(por_roll)
    
            a3 = por_roll
            a3 = th_to_val_join_th3(a3)
 
            print("cilo  ", i, " deseado th3",a3)

            #roll.publish(a1)
 #           yaw.publish(a2)
 
            a2=1430
            th3pp=compensar_th3(a2,a3)    
     #       print(th3pp)


     #       roll.publish(a1)
       #     yaw.publish(a2)
            pitch_1.publish(th3pp)
            





 
            posiciones =[0,0,por_roll]#  offset para calinrar el mov del mo
             
            mandi_iz=-posiciones[2]
                           
            mandi_de=3.14 #posiciones[2]
            
            velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            pub_endoMsg.header.seq = 1
            pub_endoMsg.header.stamp = rospy.Time.now()
            pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
#               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

            pub_endoMsg.velocity = velocidad_endows
            pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # Publicar el mensaje en el tópico '/endowrist'
            pub_endo.publish(pub_endoMsg)
            time.sleep(tiem_ini)

        time.sleep(15)
        print("ggggggggggggggggg")        
        
        sequence_in=sequence[::-1]

        le_sec=len(sequence_in)
        
       
        for i in range(0,le_sec):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.

            por_roll=sequence_in[i]


      #      a1 = th_to_val_join_roll(por_roll)
    
            a3 = por_roll
            a3 = th_to_val_join_th3(a3)
     
   #         print(" deseado",a3)
            print("cilo  ", i, " deseado th3",a3)
            #roll.publish(a1)
 #           yaw.publish(a2)
 
            a2=1430
            th3pp=compensar_th3(a2,a3)    
    #        print("enviado" ,th3pp)

     #       roll.publish(a1)
       #     yaw.publish(a2)
            pitch_1.publish(th3pp)
            # if i==le_sec-2:
               # pdb.set_trace()       # funcion para interrumpir y debugar





     
            posiciones =[0,0,por_roll]#  offset para calinrar el mov del mo
             
            mandi_iz=-posiciones[2]
                           
                           
            mandi_de=3.14 #posiciones[2]
            
            velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            pub_endoMsg.header.seq = 1
            pub_endoMsg.header.stamp = rospy.Time.now()
            pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
#               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

            pub_endoMsg.velocity = velocidad_endows
            pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # Publicar el mensaje en el tópico '/endowrist'
            pub_endo.publish(pub_endoMsg)
            time.sleep(tiem_ini)








    else:
        # Aquí va el código que se ejecuta si ninguna de las condiciones anteriores es verdadera
        pass








def publicar_virtual(m_q_vec_ik,m_efector2,modo):




  #  m_q_vec_ik = m_q_vec_ik1[::-1]
    with open('datos.pickle', 'rb') as archivo:
     angulo_inicialpinza, angulo_incremento,radio_aguja,velocidadr, velocidad_robot= pickle.load(archivo)


  #  print("publicar")


 #  rospy.init_node('publicador', anonymous=True) #inicia el nodo publicador
    # rate = rospy.Rate(0.5) # 0.5 Hz
    
    # pub_endo = rospy.Publisher('/endowrist', JointState, queue_size=10)
    # roll = rospy.Publisher('/EndoWrist/servo1/target', Int16, queue_size=10) #crea el publicador en el topic hola_mundo y tipo de mensaje int16
    # yaw = rospy.Publisher('/EndoWrist/servo2/target', Int16, queue_size=10)
    # pitch_2 = rospy.Publisher('/EndoWrist/servo4/target', Int16, queue_size=10)
    # pitch_1 = rospy.Publisher('/EndoWrist/servo3/target', Int16, queue_size=10)


    # pub_endoMsg = JointState()

    # pub_endoMsg.header.frame_id = 'base_link'
    # pub_endoMsg.name = ['shaft', 'wrist', 'jaw_dx', 'jaw_sx']

    # pub_endo = rospy.Publisher('/endowrist', JointState, queue_size=10)

   # rospy.loginfo("Estoy publicando en el topic hola_mundo") #escribo en log
            # Definir el rango de posiciones del servo en microsegundos
    servo_min = 1000 # 0 grados
    servo_max = 2000 # 180 grados

    # Definir el rango de ángulos en radianes
    angle_min = -2.35619 # -135 grados
    angle_max = 2.35619 # 135 grados
    
    
     
    def interpolate_matrix(matrix, num_inter_points=10):
        # convertir la matriz a un array de numpy
        matrix = np.array(matrix)

        # obtener el número de filas
        num_rows = matrix.shape[0]

        # generar un nuevo conjunto de índices para la interpolación
        new_index = np.linspace(0, num_rows-1, num_inter_points*(num_rows-1)+1)

        # inicializar la matriz interpolada
        interp_matrix = np.zeros((len(new_index), 3))

        # para cada columna en la matriz
        for i in range(3):
            # crear una función de interpolación para la columna actual
            f = interp1d(np.arange(num_rows), matrix[:, i])

            # usar la función de interpolación para generar los nuevos puntos
            interp_matrix[:, i] = f(new_index)

        return interp_matrix

       
        
    
    def convert_range(tstatus):
        # Define tus rangos
        input_start = 1984
        input_end = 9600
        output_start = 500
        output_end = 2480

        # Calcula la proporción de escalado
        scale = float(output_end - output_start) / float(input_end - input_start)

        # Aplica la conversión de rango
        output2 = output_start + ((tstatus - input_start) * scale)



        return output2

    
    



    def compensar_th3(x_deseado,th3d):
      


        # Definimos los datos de la matriz.
        matrix = [
            [500, 642.00],
            [610, 721.50],
            [720, 801.00],
            [830, 880.50],
            [940, 960.00],
            [1050, 1039.50],
            [1160, 1119.00],
            [1270, 1198.50],
            [1380, 1278.00],
            [1490, 1357.50],
            [1600, 1437.00],
            [1710, 1516.50],
            [1820, 1596.00],
            [1930, 1675.50],
            [2040, 1755.00],
            [2150, 1834.50],
            [2260, 1914.00],
            [2370, 1993.50],
            [2480, 2073.00]
        ]

        # Separamos los datos en dos listas: x_vals para la primera columna y y_vals para la segunda.
        x_vals, y_vals = zip(*matrix)

        def obtener_valor_intermedio(x_deseado):
            # Usamos la función de interpolación para encontrar y_deseado.
            y_deseado = np.interp(x_deseado, x_vals, y_vals)
            return y_deseado
         

        # Obtenemos y mostramos el resultado
        y_deseado = obtener_valor_intermedio(x_deseado)
        th3centro=y_deseado
      #  th3p=0

        if th3d >= 1490:
            dif = th3d - 1490
            th3p = th3centro + dif
            
            
            if th3p > 2480:
                th3p = 2480

            
        elif th3d < 1490:
            dif = 1490 - th3d
            th3p = th3centro - dif

            if th3p < 490:
                th3p = 490

            
        # elif th3p > (th3centro+1000):

            # th3p = th3centro +1000


            
        # elif th3p < (th3centro-1000):

            # th3p = th3centro -1000   



        return th3p



    def th_to_val_join_th2(th):
        x1, y1 = 1.57, 500
        x2, y2 = -1.57, 2480

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2480), 500)
        
        val11=int(val_join)-60

        return val11



    def th_to_val_join_th3(th):
        x1, y1 = 1.57, 585.25
        x2, y2 = -1.57, 2260.25

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2480), 500)
        
        val11=int(val_join)+242

        return val11





    def th_to_val_join_roll(th):
        x1, y1 = -5.1051, 500
        x2, y2 = 0, 2480

        # Calcula la pendiente
        m = (y2 - y1) / (x2 - x1)

        # Calcula b
        b = y1 - m * x1

        # Calcula y (val_join)
        val_join = m * th + b

        # Asegúrate de que val_join está dentro del rango permitido
        val_join = max(min(val_join, 2400), 736)
        
        val11=int(val_join)

        return val11


      
    m_q_vec_ik_456 = m_q_vec_ik[:,3:6]

        

    tiem_ini=0.08
    frac=70  # saltos
                          
   # print("Punto de interrupción")
        # pdb.set_trace()       # funcion para interrumpir y debugar
    lista2 = [0, 0,0]
    if modo==1:
        
        sublista_fila2 = m_q_vec_ik_456[1,:]
        fila = [a + b for a, b in zip(sublista_fila2, lista2)]
        # Imprime el resultado
      
        

        
        # a = fila[3])
        # pitch_2.publish(a)
        mandi=3.14  # en el simulador se esconde la mordaza que no se usa
        posiciones =fila#  offset para calinrar el mov del mo
        
        mandi_iz=posiciones[2]
        mandi_de=-posiciones[2]
        
        velocidad_endows=[0.0, 0.0, 0.0, 0.0]
        pub_endoMsg.header.seq = 1
        pub_endoMsg.header.stamp = rospy.Time.now()
        pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
        pub_endoMsg.velocity = velocidad_endows
        pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
        # Publicar el mensaje en el tópico '/endowrist'
        pub_endo.publish(pub_endoMsg)
   #     time.sleep(0.1)

        print("pos inicial")
   #     pdb.set_trace()    

    elif modo == 2:        

                    
            len_m_efector2=len(m_q_vec_ik)
            print("soluc",len_m_efector2)
            len_m_ef22=len(m_efector2)
       #     pdb.set_trace()       # funcion para interrumpir y debugar          
            tiempo=0
            for i in range(len_m_efector2):
                

    #            velocidadr=0.02
         
                if i>1:
                
                    T_a  = m_efector2[i]      
                    T_a2  = m_efector2[i-1]      
                    posxy1=T_a[:3, 3]
                    posxy2=T_a2[:3, 3]                    
                    
      #              print("1    ",posxy1,"       2      ",posxy2)
                
                    dx = posxy1[0]-posxy2[0]  
                    dy = posxy1[1]-posxy2[1]  
                    dz = posxy1[2]-posxy2[2]  
                    
                    
                    
                    Distancia = math.sqrt((dx**2) + (dy**2) + (dz**2))
         #           print(Distancia)
                    if Distancia>0:
                        tiempo = (Distancia/(velocidadr))*1
                        
       #                 print("tiempo calculado",tiempo)
                    else:
                        tiempo = 0.11
         #               print("soluc dis")
                    retardo=tiempo  
                else:
                    retardo=0.1   # tiempo de retardo sirve para sincronizar
     #               print(" lanza endo  ", time.time())
                
              #  retardo2=0.01
                
         
       #         print("vel   ", velocidadr,"tiempo  ", retardo)



                
              #  retardo=0.1
                cpuntos=m_q_vec_ik_456.shape[0]
                sublista_fila2 = m_q_vec_ik_456[i, :]
                

                # Realiza la suma término a término utilizando zip y una comprensión de listas
                fila = [a + b for a, b in zip(sublista_fila2, lista2)]
                # Imprime el resultado
                posiciones =fila#  offset para calinrar el mov del mo
        
                mandi_iz=-posiciones[2]

        #        mandi_iz=0
                               
                mandi_de=3.14 #posiciones[2]
                
                velocidad_endows=[0.0, 0.0, 0.0, 0.0]
                pub_endoMsg.header.seq = 1
                pub_endoMsg.header.stamp = rospy.Time.now()
                pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
 #               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

                pub_endoMsg.velocity = velocidad_endows
                pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
                # Publicar el mensaje en el tópico '/endowrist'
                pub_endo.publish(pub_endoMsg)
            
                
                        
           #     time.sleep(0.2)
                       
                time.sleep(retardo)
          

    elif modo == 3:
        # este modo es para dejar el robot en la posicion inicial siempre
        


        a1 = 1450
        a2 = 1450
        a3 = 1450

     
        
        
        for i in range(1, frac+1):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.
        
   
        
        # a = fila[3])
        # pitch_2.publish(a)
            mandi=3.14  # en el simulador se esconde la mordaza que no se usa

      
        
        # a = fila[3])
        # pitch_2.publish(a)
            mandi=3.14  # en el simulador se esconde la mordaza que no se usa
          
            # velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            # pub_endoMsg.header.seq = 1
            # pub_endoMsg.header.stamp = rospy.Time.now()
            # pub_endoMsg.position = [0, 0,0 , mandi]
            # pub_endoMsg.velocity = velocidad_endows
            # pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # # Publicar el mensaje en el tópico '/endowrist'
            # pub_endo.publish(pub_endoMsg)
            

            time.sleep(tiem_ini)



    elif modo == 4:
        # modo de sincronizacion roll y yau
        

        # Generar una secuencia desde 0 hasta pi/2 con pasos de pi/24
      #  sequence = np.arange(0, np.pi/3 + np.pi/24, np.pi/24)
        sequence = np.arange(0, np.pi/4 + np.pi/24, np.pi/24)


        # Redondear a 4 decimales
        sequence = np.round(sequence, 4)
        sequence=sequence* -1
        # Imprimir la secuencia
        print(sequence)

        tiem_ini=0.7

        le_sec=len(sequence)
        print(le_sec)
        

       
        for i in range(0,le_sec):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.




            por_roll=sequence[i]
  #          print(i)

      #      a1 = th_to_val_join_roll(por_roll)
    
            a3 = por_roll
            a3 = th_to_val_join_th3(a3)
 
            print("cilo  ", i, " deseado th3",a3)

            #roll.publish(a1)
 #           yaw.publish(a2)
 
            a2=1430
            th3pp=compensar_th3(a2,a3)    
     #       print(th3pp)


     #       roll.publish(a1)
       #     yaw.publish(a2)
            pitch_1.publish(th3pp)
            





 
            posiciones =[0,0,por_roll]#  offset para calinrar el mov del mo
             
            mandi_iz=-posiciones[2]
                           
            mandi_de=3.14 #posiciones[2]
            
            # velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            # pub_endoMsg.header.seq = 1
            # pub_endoMsg.header.stamp = rospy.Time.now()
            # pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
# #               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

            # pub_endoMsg.velocity = velocidad_endows
            # pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # # Publicar el mensaje en el tópico '/endowrist'
            # pub_endo.publish(pub_endoMsg)
            # time.sleep(tiem_ini)

        time.sleep(15)
    
        
        sequence_in=sequence[::-1]

        le_sec=len(sequence_in)
        
       
        for i in range(0,le_sec):  # Nota que 'range' es inclusivo en el inicio y exclusivo en el fin, así que debes usar 21 para incluir el 20.

            por_roll=sequence_in[i]


      #      a1 = th_to_val_join_roll(por_roll)
    
            a3 = por_roll
            a3 = th_to_val_join_th3(a3)
     
   #         print(" deseado",a3)
            print("cilo  ", i, " deseado th3",a3)
            #roll.publish(a1)
 #           yaw.publish(a2)
 
            a2=1430
            th3pp=compensar_th3(a2,a3)    
    #        print("enviado" ,th3pp)

     #       roll.publish(a1)
       #     yaw.publish(a2)
            pitch_1.publish(th3pp)
            # if i==le_sec-2:
               # pdb.set_trace()       # funcion para interrumpir y debugar





     
            posiciones =[0,0,por_roll]#  offset para calinrar el mov del mo
             
            mandi_iz=-posiciones[2]
                           
                           
            mandi_de=3.14 #posiciones[2]
            
            # velocidad_endows=[0.0, 0.0, 0.0, 0.0]
            # pub_endoMsg.header.seq = 1
            # pub_endoMsg.header.stamp = rospy.Time.now()
            # pub_endoMsg.position = [posiciones[0], posiciones[1],mandi_iz , mandi_de]
# #               pub_endoMsg.position = [0, 0,mandi_iz , mandi_de]

            # pub_endoMsg.velocity = velocidad_endows
            # pub_endoMsg.effort = [0.0, 0.0, 0.0, 0.0]
            # # Publicar el mensaje en el tópico '/endowrist'
            # pub_endo.publish(pub_endoMsg)
            # time.sleep(tiem_ini)








    else:
        # Aquí va el código que se ejecuta si ninguna de las condiciones anteriores es verdadera
        pass








def calcular_ik(M_tray_endo_sale2):

    q_vec_ik=[] 

#  for i in range(0, cant_rutas, 1):
    Ta = M_tray_endo_sale2 #[i]
    Ta = np.array(Ta)                    
    Ta_redondeada = np.round(Ta, 4)
    
    #pdb.set_trace()       # funcion para interrumpir y debugar
    
    matriz_cadena = '; '.join(' '.join(str(num) for num in fila) for fila in Ta_redondeada)
 #   matriz_cadena = '; '.join(' '.join(map(str, fila)) for fila in Ta)
    
    
    
    comando = ['./ik_endo_v2 '+'"'+matriz_cadena+'"']
    print(comando)
    try:
    # Ejecutar el comando y obtener la salida
        salida = subprocess.check_output(comando,  shell=True)

    # Imprimir la salida del programa 'suma'
        print(salida)
    except subprocess.CalledProcessError as e:
        print("Error al ejecutar 'ik endo ':", e.output) 
         
    matches = re.findall(r"q_vec_ik: (-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", salida)

    if matches:
        q_vec_ik = matches[0]
        # Resto del código que utiliza q_vec_ik
             # Convertir los valores a números y crear un array de numpy
    
     #   print("vector salida IK ++++ ",q_vec_ik)
        
        q_vec_ik = np.array([float(x) for x in q_vec_ik])
        
    
                        
  #  time.sleep(0.002)

   # pdb.set_trace()       # 

    return q_vec_ik







class MaquinaEstados:
    def __init__(self):
        self.estados = ["Estado 1", "Estado 2", "Estado 3", "Estado 4", "Estado 5", "Estado 6"]
        self.estado_actual = 0

    def preguntar_usuario(self, pregunta):
        respuesta = input(pregunta)
        return respuesta.lower()
  

    
        
    def mover_robot_lineal(self,x,y,z):   # esta funcion mueve el robot hasta una posicion deseada
    


        # Guardar las variables en un archivo
        with open('posiciones_xyz.pickle', 'wb') as archivo:
            pickle.dump((x, y, z), archivo)



        time.sleep(0.1)

        comando = ['python3 mr_posicionar6.py']
        print(comando)
        try:
        # Ejecutar el comando y obtener la salida
            salida = subprocess.check_output(comando,  shell=True)

        # Imprimir la salida del programa 'suma'
            print(salida)
        except subprocess.CalledProcessError as e:
            print("Error possicionando':", e.output) 
            
            
            
    
    

    def ejecutar(self):
    
    
        #sub1 = rospy.Subscriber('/tf', TFMessage, callback_function1)
        #sub2 = rospy.Subscriber('/joint_states', JointState, callback_function)
        fig = plt.figure()
        

        


    
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
           


                # # esta seccion pide la posicion actual del robot
                # subscriber = rospy.Subscriber(topic_name, TFMessage)

                # # Esperar a que se reciban los mensajes con la posicin actual
                # rospy.sleep(2)  # Ajusta el tiempo de espera segn sea necesario

                # # Obtener la ltima posicin y orientacin del robot
                # latest_msg = rospy.wait_for_message(topic_name, TFMessage)

                # # Extraer los componentes x, y, z y qx, qy, qz, qw de la posicin y orientacin
                # position = latest_msg.transforms[0].transform.translation
                # orientation = latest_msg.transforms[0].transform.rotation

                # px = position.x
                # py1 = position.y
                # pz = position.z
                # rx = orientation.x
                # ry = orientation.y
                # rz = orientation.z
                # rw = orientation.w


                # # Crear un diccionario con las variables
                # data = {
                    # "px": px,
                    # "py1": py1,
                    # "pz": pz,
                    # "rx": rx,
                    # "ry": ry,
                    # "rz": rz,
                    # "rw": rw
                # }

                # # Escribir el diccionario en un archivo JSON
                # with open("data.json", "w") as outfile:
                    # json.dump(data, outfile)

        
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
                
                # Ts_inicial = np.array([[-0.9999, -0.0132, 0.0003, -0.1326],
                        # [-0.0081, 0.5956, -0.8032, -0.1238],
                        # [0.0104, -0.8032, -0.5957, 0.3839],
                        # [0, 0, 0, 1.0000]])        


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


                    # A los datos pares sumarle 0.006 en la segunda posición
                  #  if index % 2 != 0:
                        #ccc1[1] -= 0.001
                    #    ccc2[1] -= 0.015

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

                # Convertir la lista transformada a un array de numpy
                # transformed_data = np.array(transformed_data)

                # # Graficar los datos
                # for i in range(transformed_data.shape[0]):
                    # # Si el índice es par (incluyendo el cero), pintamos en verde.
                    # color = 'green' if i % 2 != 0 else 'blue'  # O el color que normalmente se usa.
                    # ax.scatter(transformed_data[i, 0], transformed_data[i, 1], transformed_data[i, 2], color=color)






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
 
 
                # Convierte la lista de posiciones en una matriz numpy
                posicionespp = np.array(posicionespp)

                # # Guarda la matriz en un archivo de texto
                # # Obtiene la fecha y hora actuales
                # now = datetime.datetime.now()

                # # Crea el nombre del archivo con la fecha y hora actuales
                # filename = 'posiciones_{}.txt'.format(now.strftime("%Y-%m-%d_%H-%M-%S"))

                # # Guarda la matriz en un archivo de texto
                # np.savetxt(filename, posicionespp)



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

                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

                plt.show()

  

                               
                
                    
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



                for i in range(1, longitud_dict_rutas+1):
   
                    print("el ciclo principali",i)
      
                    clave ='rutas_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    M_tray_endo_sale=dict_rutas[clave]   # aqui hay que adicionar que el robot suba y baje ***
                    ln_M_tray_endo_sale=len(M_tray_endo_sale)
                    print("cantidad de rutas",ln_M_tray_endo_sale)               
                    
                         
                    m_q_vec_ik1a = np.array([])                    
                    for i2 in range(1, ln_M_tray_endo_sale):
                    
     #                   print("el ciclo i2",i2)
                        M_tray_endo_sale_r=M_tray_endo_sale[i2]
                 #       print("matriz para calcular ik ",M_tray_endo_sale_r)

                        q_vec_ik1=calcular_ik(M_tray_endo_sale_r)
          

              #          pdb.set_trace()    

                        q_vec_ik1 = np.array(q_vec_ik1)



                        # Apilar verticalmente el array a la matriz de resultados
                        if m_q_vec_ik1a.size == 0:  # Si la matriz está vacía, asignar el primer array
                            m_q_vec_ik1a = q_vec_ik1
         
                            
                        elif q_vec_ik1.size != 0:  # Si no está vacío, apilar el array debajo de la matriz
                            m_q_vec_ik1a = np.vstack((m_q_vec_ik1a, q_vec_ik1))
                            
                    print("sol ik encontradas   ", len(m_q_vec_ik1a), "estas",m_q_vec_ik1a)
                    
                    
                    #sitio para aumentar la densidad de puntos intermedios
                                    

                    # Crear una copia de la matriz original para no modificar la original
                    m_q_vec_ik1a_aux1 = np.copy(m_q_vec_ik1a)

                    # Modificar la primera columna si el valor es > 1
                    # Crear una máscara booleana para las filas que cumplen con la condición
                    mask = m_q_vec_ik1a_aux1[:,3] <= 0.5

                    # Utilizar la máscara para seleccionar solo las filas que queremos mantener
                    m_q_vec_ik1a_aux1 = m_q_vec_ik1a_aux1[mask]
                    # Modificar la última columna si el valor es > 0
                    #m_q_vec_ik1a_aux1[m_q_vec_ik1a_aux1[:,-1]>0,-1] *= -1


                    # Llamada a la función  aumentar la densidad de puntos intemedios
             #       m_q_vec_ik1a_aumentado = interpolate_matrix2(m_q_vec_ik1a_aux1, num_inter_points=5)  # el num representa la cantidad de puntos intemedios



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

   
           
                   # pdb.set_trace()       # funcion para interrumpir y debugar  ==============================================
                    #______________
                    # if(i==1):
                        # np.save("matrizm_q_vec_ik_original.npy", m_q_vec_ik1a_aux1)
                        # np.save("matrizm_q_vec_ik.npy", m_q_vec_ik1a_aumentado)


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










                    dic_q_ik['vect_q_ik_'+str(i)] = m_q_vec_ik1a_aumentado       
                        
                    
                              
                print("estas en el estado 3 Calculando ik endo")
                    
                print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio") 
                
            elif self.estado_actual == 4:

                #parte 4


                tras = Tpfulcro.copy()

                
#                print(pos_op)
#______________
                print('Ejecutando..6.')
                # parte 5

                #import rospy
                #from geometry_msgs.msg import PoseArray, Pose

                dic_m_efector = {}  

                #rospy.init_node('your_node_name')

                m_posiciones_efect = np.array([])
                
                lon_dic_q_ik=len(dic_q_ik)
                print("lon vector q ik  ", lon_dic_q_ik)        

                array_poses=[]
                array_tini=[]

                
                for i in range(1, lon_dic_q_ik+1):
             
                    clave ='vect_q_ik_'+str(i)

                    print("Clave: diccionario  ", clave)
                    
                    m_q_vec_ik=dic_q_ik[clave]
                #    m_q_vec_ik1=len(lista_m_q_vec_ik)
               #     print("contenido de clave  :::::::::::::::::::::::::::::::::::::::::::::  ", m_q_vec_ik)
                
                    print("contenido de clave  :::::::::::::::::::::::::::::::::::::::::::::  ")
     

           #     pdb.set_trace()       # funcion para interrumpir y debugar  ==============================================
            
                
                
                #++++
                    poseArrayMsg= PoseArray()
                    M_efector = []   
                    m_T_ento = []   
                    T_ento=[]
                    T_ento_muneca=[]
                    T_ento_muneca3=[]
                    T_ento_base=[]

                    m_T_ento_muneca=[]
                    m_T_ento_muneca3=[]
                    m_T_ento_base=[]

                    
                    len_m_q_vec_ik=len(m_q_vec_ik)
                    for i3 in range(1, len_m_q_vec_ik + 1):
                        posES = Pose()
                        print("cilco i3   ",i3)

                        q_vec_ik3 = m_q_vec_ik[i3 - 1]   # puedes agregar un desplazamiento a las posiciones articulares del endoscopio
                        
                        # **
                        T = fs.endotro_fwd_kinematicspy(q_vec_ik3)
                        matrices_homogeneas = T
                        num_matrices = len(matrices_homogeneas)
            #****    esta distancia al fulcro no es clara
                        Tpq = [[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, -dis_j5_ppinza],
                               [0, 0, 0, 1]]
                        Tur3 = np.dot(T[2], Tpq)
                        Tur3[0, 3] += tras[0, 3]
                        Tur3[1, 3] += tras[1, 3]
                        Tur3[2, 3] += tras[2, 3]

                        T_ento = T[5].copy()

                        T_ento[0, 3] += tras[0, 3]
                        T_ento[1, 3] += tras[1, 3]
                        T_ento[2, 3] += tras[2, 3]

                        T_ento_muneca = T[2].copy()

                        T_ento_muneca[0, 3] += tras[0, 3]
                        T_ento_muneca[1, 3] += tras[1, 3]
                        T_ento_muneca[2, 3] += tras[2, 3]


                        T_ento_muneca3 = T[3].copy()

                        T_ento_muneca3[0, 3] += tras[0, 3]
                        T_ento_muneca3[1, 3] += tras[1, 3]
                        T_ento_muneca3[2, 3] += tras[2, 3]



                        T_ento_base = T[0].copy()

                        T_ento_base[0, 3] += tras[0, 3]
                        T_ento_base[1, 3] += tras[1, 3]
                        T_ento_base[2, 3] += tras[2, 3]




                        posES.position.x = Tur3[0, 3]
                        posES.position.y = Tur3[1, 3]
                        posES.position.z = Tur3[2, 3]

                        rotm = Tur3[:3, :3]
                        quat = fs.rotm2quat(rotm)

                        posES.orientation.x = quat[1]
                        posES.orientation.y = quat[2]
                        posES.orientation.z = quat[3]
                        posES.orientation.w = quat[0]

                        poseArrayMsg.poses.append(posES)
                        
                        M_efector.append(Tur3.copy())  # *t_flip
                        m_T_ento.append(T_ento.copy())  # *t_flip
                        m_T_ento_muneca.append(T_ento_muneca.copy())  # *t_flip
                        m_T_ento_muneca3.append(T_ento_muneca3.copy())  # *t_flip
                        m_T_ento_base.append(T_ento_base.copy())  # *t_flip
                                                
                        # #para graficar la posicion del terminal del ur3
                        # posiciones_efect = np.array([Tur3[0, 3], Tur3[1, 3], Tur3[2, 3]])
                        
                        # posiciones_efect = np.reshape(posiciones_efect, (1, 3))
                        
                        # if m_posiciones_efect.size == 0: # Si la matriz está vacía, asignar el primer array
                            # m_posiciones_efect = posiciones_efect
                        # else: # Si no, apilar el array debajo de la matriz
                            # m_posiciones_efect = np.vstack((m_posiciones_efect, posiciones_efect))                           
                        
                        if i3 == 1:
                         TiniT = Tur3.copy()

                         
                    print("adicionando poses   ",i)                        
                    array_poses.append(poseArrayMsg)
                    array_tini.append(TiniT)
                    dic_m_efector['m_efector_'+str(i)] = M_efector   
                    
                    
                    
                    
                    
                    
                    if i == 1:                    
                        np.save("m_efector.npy", M_efector)
                        np.save("m_T_ento.npy", m_T_ento)
                        np.save("m_T_ento_muneca.npy", m_T_ento_muneca)
                        np.save("m_T_ento_muneca3.npy", m_T_ento_muneca3)
                        np.save("m_T_ento_base.npy", m_T_ento_base)
                           
                    
                    


            #        pdb.set_trace()       # funcion para interrumpir y debugar  ==============================================
            

                print("estas en el estado 4 ")
                print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio") 

                
                num_puntadas=len(array_poses)

               

       
                # print("Punto de interrupción")
                # pdb.set_trace()       # funcion para interrumpir y debugar
            
                index_puntadas=1   
            elif self.estado_actual == 5:
            
         #       habilitar un contador para repetir el mismo estado pero otra puntada
                # primero se ubica por encima


                M_efector2 = []   
                poseArrayMsg2= PoseArray()
                m_q_vec_ik1=[]
                m_q_vec_ik=[]

                if index_puntadas <= num_puntadas: 


                    clave ='m_efector_'+str(index_puntadas)
                    print("Clave: diccionario  ", clave)
                    m_efector=dic_m_efector[clave]



                    clave ='vect_q_ik_'+str(index_puntadas)
                    print("Clave: diccionario  ", clave)
                    m_q_vec_ik=dic_q_ik[clave]




                    desplazamiento=0
#                   hacer_puntada_virtual(m_efector,m_q_vec_ik,desplazamiento)


                    hacer_puntada(m_efector,m_q_vec_ik,desplazamiento)
                   

   #                 hacer_puntada_alin(m_efector,m_q_vec_ik,desplazamiento)
                   
                   

 
 #                   hacer_puntada_calib(m_efector,m_q_vec_ik,desplazamiento)
                    
                    
                    # time.sleep(5)
                    # hacer_posicion_ini(m_efector,m_q_vec_ik,desplazamiento)
                   






                 #   print("llamando a publicar")
                          
                    print("puntada #  ",index_puntadas," de / ",num_puntadas)
                    print("PRESONA r PARA PASAR A LA SIGUIENTE PUNTADA")
                    print("PRESONA w PARA repetir esta  PUNTADA")
                    print("presiona x para Salir")                     
                    index_puntadas=index_puntadas+1
                                       
                else:
                    print("se terminaron las puntadas")
                    print("presiona x para Salir")                     
                    print("estas en el estado 5 ")
                    print("e1 analisis heridas, e3  tray aguja, e3 ik, e4  tray robot, e5 mover r p, e0 inicio")                   
                    
                    
      








                

                
                # usar la funcion de enviar info  publicar_motor2.py
            
            
            
            

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
  #  rospy.init_node('system_py')
    iniciar()

