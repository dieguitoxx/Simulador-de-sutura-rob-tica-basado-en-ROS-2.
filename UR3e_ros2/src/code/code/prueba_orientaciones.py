# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R
import ikfastpy

def unity_to_robot_coords(points_global, pos_robot, rot_robot_euler, scale_factor=1.0):
    """Convierte puntos desde coordenadas globales Unity a coordenadas del robot (base_link)."""
    pos_robot_m = np.array(pos_robot) * scale_factor
    rot_robot = R.from_euler('xyz', rot_robot_euler, degrees=True).as_matrix()

    T_world_robot = np.eye(4)
    T_world_robot[:3, :3] = rot_robot
    T_world_robot[:3, 3] = pos_robot_m

    T_robot_world = np.linalg.inv(T_world_robot)

    points_robot = []
    for p in points_global:
        p_m = np.array(p) * scale_factor
        p_hom = np.append(p_m, 1)
        p_robot = T_robot_world @ p_hom
        points_robot.append(p_robot[:3])

    return points_robot


def test_ik(points_robot):
    """Prueba la cinemática inversa para cada punto con orientaciones múltiples."""
    ur_kin = ikfastpy.PyKinematics()
    n_joints = ur_kin.getDOF()

    orientations = [
        np.eye(3),  # identidad
        np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]]),  # hacia abajo
        np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]),  # lateral
        np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])   # frontal
    ]

    for i, (x, y, z) in enumerate(points_robot):
        found_solution = False
        print(f"\n🔍 Punto {i+1}: {x:.4f}, {y:.4f}, {z:.4f}")
        for idx, R_mat in enumerate(orientations):
            pose_matrix = np.eye(4)
            pose_matrix[:3, :3] = R_mat
            pose_matrix[0, 3] = x
            pose_matrix[1, 3] = y
            pose_matrix[2, 3] = z

            pose_3x4_flat = pose_matrix[:3, :].flatten().tolist()
            joint_configs = ur_kin.inverse(pose_3x4_flat)

            if joint_configs:
                n_solutions = len(joint_configs) // n_joints
                solutions = np.array(joint_configs).reshape(n_solutions, n_joints)
                print(f"  ✅ Orientación {idx+1}: {n_solutions} soluciones")
                for sol in solutions:
                    print(f"    {np.round(sol, 4)}")
                found_solution = True
            else:
                print(f"  ❌ Orientación {idx+1}: Sin solución")

        if not found_solution:
            print("⚠ Ninguna orientación tiene solución para este punto.")


# ================================
# CONFIGURACIÓN
# ================================

# Posición del prefab del robot en Unity
pos_robot = (-0.55, 0.188, -0.465)  # en Unity

# Rotación del robot en Unity (Euler XYZ en grados)
rot_robot_euler = (0, 0, 0)

# Lista de puntos de sutura en coordenadas globales Unity
points_global = [
    (-1.0321, 0.3922, -0.2366),
    (-1.0168, 0.3922, -0.2494),
    (-1.0450, 0.3933, -0.2495),
    (-1.0296, 0.3933, -0.2623),
    (-1.0620, 0.3945, -0.2611),
    (-1.0467, 0.3945, -0.2740)
]

# FACTOR DE ESCALA:
#   - Si Unity está en metros: 1.0
#   - Si Unity está en mm: 0.001
scale_factor = 1.0

# ================================
# PROCESO
# ================================
print("📌 Transformando puntos de Unity a coordenadas del robot...")
points_robot = unity_to_robot_coords(points_global, pos_robot, rot_robot_euler, scale_factor)

print("\n📌 Probando IK para cada punto...")
test_ik(points_robot)
