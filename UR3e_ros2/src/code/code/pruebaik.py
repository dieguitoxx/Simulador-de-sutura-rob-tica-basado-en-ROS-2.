# -*- coding: utf-8 -*-
import numpy as np
import re
import ikfastpy

# Posición del UR3 en Unity (en metros)
robot_position_unity = np.array([-0.55, 0.188, -0.465])  

# Orientaciones candidatas (matrices de rotación 3x3)
orientations = [
    np.eye(3),  # Identidad
    np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]),  # Hacia abajo
    np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]),  # Lateral
    np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])   # Frontal
]

def load_suture_points(filename):
    """Carga puntos de sutura desde archivo exportado por Unity."""
    points = []
    with open(filename, 'r') as f:
        lines = f.readlines()

    entrada = None
    salida = None
    for line in lines:
        if "Entrada" in line:
            entrada = tuple(map(float, re.findall(r"[-\d.]+", line)))
        elif "Salida" in line:
            salida = tuple(map(float, re.findall(r"[-\d.]+", line)))
            if entrada and salida:
                points.append(entrada)
                points.append(salida)
                entrada, salida = None, None
    return points

def convert_unity_to_robot(points, robot_pos):
    """Convierte coordenadas globales de Unity a coordenadas relativas al robot."""
    points_robot = []
    for p in points:
        p_rel = np.array(p) - robot_pos
        points_robot.append(p_rel)
    return points_robot

def test_ik(points_robot):
    """Prueba IK para cada punto y orientación."""
    ur_kin = ikfastpy.PyKinematics()
    n_joints = ur_kin.getDOF()

    for idx, p in enumerate(points_robot):
        print(f"\n🔹 Punto {idx+1} relativo al robot: {p}")
        found_any = False
        for o_idx, R in enumerate(orientations):
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = R
            pose_matrix[:3, 3] = p

            pose_3x4_flat = pose_matrix[:3, :].flatten().tolist()
            try:
                joint_configs = ur_kin.inverse(pose_3x4_flat)
            except Exception as e:
                print(f"Orientación {o_idx+1}: error {e}")
                continue

            if joint_configs:
                found_any = True
                n_solutions = len(joint_configs) // n_joints
                print(f"✅ Orientación {o_idx+1}: {n_solutions} soluciones")
            else:
                print(f"❌ Orientación {o_idx+1}: sin solución")

        if not found_any:
            print("⚠ Ninguna orientación tiene solución para este punto.")

if __name__ == "__main__":
    puntos_unity = load_suture_points("puntos_sutura.txt")
    puntos_robot = convert_unity_to_robot(puntos_unity, robot_position_unity)
    test_ik(puntos_robot)
