import numpy as np
import matplotlib.pyplot as plt

# Cargar la matriz desde el archivo .npy
file_path = "herida/matrizm_q_vec_ik_1.npy"
matrix = np.load(file_path)

# Crear el eje X (índices de las filas)
x = np.arange(matrix.shape[0])

# Crear la gráfica
plt.figure(figsize=(10, 6))
for i in range(matrix.shape[1]):
    plt.plot(x, matrix[:, i], label=f'Columna {i+1}')

# Configurar etiquetas y título
plt.xlabel("Índice de fila")
plt.ylabel("Valor")
plt.title("Gráfica de las 6 columnas de datos")
plt.legend()
plt.grid()

# Mostrar la gráfica
plt.show()
