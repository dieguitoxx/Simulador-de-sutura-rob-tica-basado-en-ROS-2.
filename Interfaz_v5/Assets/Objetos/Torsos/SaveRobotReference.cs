using UnityEngine;
using System.IO;

public class SaveRobotReference : MonoBehaviour
{
    public Transform joint1, joint2, joint3, joint4, joint5, joint6;
    public string fileName = "unity_reference_matrix.txt";

    void SaveReference()
    {
        // 1️⃣ Obtener la pose final del efector
        Transform endEffector = joint6; // último eslabón de tu UR3
        Vector3 position = endEffector.position; // en metros
        Quaternion rotation = endEffector.rotation;

        // 2️⃣ Convertir la rotación en matriz 3x3
        Matrix4x4 rotMatrix = Matrix4x4.Rotate(rotation);

        // 3️⃣ Construir la matriz homogénea 4x4
        Matrix4x4 homog = Matrix4x4.identity;
        homog.SetColumn(0, rotMatrix.GetColumn(0));
        homog.SetColumn(1, rotMatrix.GetColumn(1));
        homog.SetColumn(2, rotMatrix.GetColumn(2));
        homog.SetColumn(3, new Vector4(position.x, position.y, position.z, 1));

        // 4️⃣ Guardar en un archivo .txt en formato legible
        string path = Path.Combine(Application.dataPath, fileName);
        using (StreamWriter writer = new StreamWriter(path))
        {
            for (int i = 0; i < 4; i++)
            {
                writer.WriteLine($"{homog[i,0]} {homog[i,1]} {homog[i,2]} {homog[i,3]}");
            }
        }

        Debug.Log($"✅ Matriz de referencia guardada en: {path}");
    }

    // Llama esto desde un botón en Unity
    public void SaveReferenceFromButton()
    {
        SaveReference();
    }
}

