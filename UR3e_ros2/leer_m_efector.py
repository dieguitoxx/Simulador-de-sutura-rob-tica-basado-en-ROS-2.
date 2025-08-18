import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def mostrar_npy(archivo):
    try:
        datos = np.load(archivo)
        print("Contenido de", archivo, ":\n", datos)
        graficar_matrices(datos)
    except Exception as e:
        print("Error al cargar el archivo:", e)

def graficar_matrices(matrices):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    added_labels = set()
    for i in range(0, len(matrices), 10):  # Mostrar cada 10 saltos
        matriz = matrices[i]
        origen = matriz[:3, 3]
        x_axis = matriz[:3, 0] * 0.05
        y_axis = matriz[:3, 1] * 0.05
        z_axis = matriz[:3, 2] * 0.05
        
        if 'X' not in added_labels:
            ax.quiver(*origen, *x_axis, color='r', label='X')
            added_labels.add('X')
        else:
            ax.quiver(*origen, *x_axis, color='r')
        
        if 'Y' not in added_labels:
            ax.quiver(*origen, *y_axis, color='g', label='Y')
            added_labels.add('Y')
        else:
            ax.quiver(*origen, *y_axis, color='g')
        
        if 'Z' not in added_labels:
            ax.quiver(*origen, *z_axis, color='b', label='Z')
            added_labels.add('Z')
        else:
            ax.quiver(*origen, *z_axis, color='b')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

# Nombre del archivo
archivo_npy = "m_efector.npy"
mostrar_npy(archivo_npy)