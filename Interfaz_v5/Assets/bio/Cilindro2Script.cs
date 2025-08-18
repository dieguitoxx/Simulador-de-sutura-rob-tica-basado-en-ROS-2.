using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cilindro2Script : MonoBehaviour
{
    public GameObject cilindro1; // Asigna el cilindro1 desde el Inspector
    public float ajusteAbajo = 0.1f; // Ajuste según sea necesario
    public float distanciaDerecha = 0.0f; // Ajuste la distancia hacia la derecha desde el Inspector
    public float distanciaProfundidad = 0.0f; // Ajuste la distancia en profundidad desde el Inspector

    void Update()
    {
        // Obtiene la posición del cilindro1 y ajusta la posición del cilindro2
        Vector3 offset = cilindro1.transform.up * (cilindro1.transform.localScale.y / 2 + transform.localScale.y / 2);
        Vector3 offset2 = cilindro1.transform.up * cilindro1.transform.localScale.y * ajusteAbajo; // Ajuste hacia abajo
        Vector3 offsetDerecha = cilindro1.transform.right * distanciaDerecha; // Ajuste hacia la derecha
        Vector3 offsetProfundidad = cilindro1.transform.forward * distanciaProfundidad; // Ajuste en profundidad

        transform.position = cilindro1.transform.position + offset - offset2 + offsetDerecha + offsetProfundidad;

        // Obtiene la rotación horizontal del cilindro1 y aplica la misma rotación al cilindro2
        transform.rotation = Quaternion.Euler(cilindro1.transform.eulerAngles.y, 90f, 90f);
    }
}


