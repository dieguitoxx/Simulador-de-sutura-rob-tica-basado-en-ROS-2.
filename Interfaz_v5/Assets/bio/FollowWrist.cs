using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowWrist : MonoBehaviour
{
    public Transform esferaTransform;
    public float alturaCilindro; // Altura del cilindro (ajustar según tus necesidades)

    void Update()
    {
        // Obtener la posición y rotación del cilindro
        Vector3 posicionCilindro = transform.position;
        Quaternion rotacionCilindro = transform.rotation;

        // Obtener la punta del cilindro (ajustar según la escala del cilindro)
        Vector3 puntaCilindro = posicionCilindro + transform.right * alturaCilindro;

        // Aplicar la posición y rotación a la esfera
        esferaTransform.position = puntaCilindro;
        esferaTransform.rotation = rotacionCilindro;
    }
}

