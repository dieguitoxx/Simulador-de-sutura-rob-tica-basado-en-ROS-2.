using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoverEsferaHaciaObjeto : MonoBehaviour
{
    public Transform objetivo; // El objeto "Jaw_Dx"
    public float velocidad = 5f; // Velocidad de movimiento de la esfera

    void Update()
    {
        // Comprueba si el objeto "Jaw_Dx" existe
        if (objetivo != null)
        {
            // Mueve la esfera hacia la posición del objeto "Jaw_Dx"
            transform.position = Vector3.MoveTowards(transform.position, objetivo.position, velocidad * Time.deltaTime);
        }
    }
}

