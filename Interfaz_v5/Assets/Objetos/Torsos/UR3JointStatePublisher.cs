using UnityEngine;
using System.IO;

public class UR3JointStateSaver : MonoBehaviour
{
    [Header("Referencias de Articulaciones")]
    public Transform joint1;
    public Transform joint2;
    public Transform joint3;
    public Transform joint4;
    public Transform joint5;
    public Transform joint6;

    [Header("Ángulos en grados (ajustar en el inspector)")]
    [Range(-180f, 180f)] public float joint1Angle;
    [Range(-180f, 180f)] public float joint2Angle;
    [Range(-180f, 180f)] public float joint3Angle;
    [Range(-180f, 180f)] public float joint4Angle;
    [Range(-180f, 180f)] public float joint5Angle;
    [Range(-180f, 180f)] public float joint6Angle;

    [Header("Ruta de guardado (carpeta de proyecto)")]
    public string fileName = "referencia_unity.txt";

    void Update()
    {
        // Mueve visualmente las articulaciones en Unity
        if (joint1) joint1.localRotation = Quaternion.Euler(0, joint1Angle, 0);
        if (joint2) joint2.localRotation = Quaternion.Euler(joint2Angle, 0, 0);
        if (joint3) joint3.localRotation = Quaternion.Euler(joint3Angle, 0, 0);
        if (joint4) joint4.localRotation = Quaternion.Euler(joint4Angle, 0, 0);
        if (joint5) joint5.localRotation = Quaternion.Euler(0, joint5Angle, 0);
        if (joint6) joint6.localRotation = Quaternion.Euler(joint6Angle, 0, 0);
    }

    [ContextMenu("Guardar referencia en .txt")]
    public void SaveJointAngles()
    {
        // Convertir a radianes
        float[] jointAnglesRad = new float[]
        {
            joint1Angle * Mathf.Deg2Rad,
            joint2Angle * Mathf.Deg2Rad,
            joint3Angle * Mathf.Deg2Rad,
            joint4Angle * Mathf.Deg2Rad,
            joint5Angle * Mathf.Deg2Rad,
            joint6Angle * Mathf.Deg2Rad
        };

        // Crear string
        string data = string.Join(",", jointAnglesRad);

        // Guardar en archivo
        string path = Path.Combine(Application.dataPath, fileName);
        File.WriteAllText(path, data);

        Debug.Log($"Referencia guardada en: {path}");
        Debug.Log($"Valores (rad): {data}");
    }
}

