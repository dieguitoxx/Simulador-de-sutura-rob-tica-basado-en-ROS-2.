using System.Collections;
using System.Collections.Generic;
 using UnityEngine;

public class movimiento1: MonoBehaviour
{
    public float velocidad = 20f; // Puedes ajustar la velocidad según tus necesidades.
    //public float distanciaMaximaX = 20f; // Ajusta según sea necesario

    public GameObject bloque3; // Asigna el bloque 1 desde el inspector.
    public GameObject bloque4; // Asigna el bloque 2 desde el inspector.
    public GameObject bloque5; // Asigna el bloque 3 desde el inspector.
    public GameObject bloque6; // Asigna el bloque 4 desde el inspector.
    public GameObject bloque7; // Asigna el bloque 1 desde el inspector.
    public GameObject bloque8; // Asigna el bloque 2 desde el inspector.
    public GameObject bloque9; // Asigna el bloque 3 desde el inspector.
    public GameObject bloque10; // Asigna el bloque 4 desde el inspector.

    private Vector3 posicionInicialBloque3; // Para almacenar la posición inicial del bloque 3
    private bool bloque3EnMovimiento = false; // Para rastrear si el bloque 3 está en movimiento

    private Vector3 posicionInicialBloque4; // Para almacenar la posición inicial del bloque 4
    private bool bloque4EnMovimiento = false; // Para rastrear si el bloque 4 está en movimiento

    private Vector3 posicionInicialBloque5; // Para almacenar la posición inicial del bloque 3
    private bool bloque5EnMovimiento = false; // Para rastrear si el bloque 3 está en movimiento

    private Vector3 posicionInicialBloque6; // Para almacenar la posición inicial del bloque 4
    private bool bloque6EnMovimiento = false; // Para rastrear si el bloque 4 está en movimiento

    private Vector3 posicionInicialBloque7; // Para almacenar la posición inicial del bloque 3
    private bool bloque7EnMovimiento = false; // Para rastrear si el bloque 3 está en movimiento

    private Vector3 posicionInicialBloque8; // Para almacenar la posición inicial del bloque 4
    private bool bloque8EnMovimiento = false; // Para rastrear si el bloque 4 está en movimiento

    private Vector3 posicionInicialBloque9; // Para almacenar la posición inicial del bloque 3
    private bool bloque9EnMovimiento = false; // Para rastrear si el bloque 3 está en movimiento

    private Vector3 posicionInicialBloque10; // Para almacenar la posición inicial del bloque 4
    private bool bloque10EnMovimiento = false; // Para rastrear si el bloque 4 está en movimiento

    void Start()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque3 = bloque3.transform.position;
        // Guardar la posición inicial del bloque 4
        posicionInicialBloque4 = bloque4.transform.position;
    }

    void Update()
    {
        // Verificar si el bloque 3 se ha movido 0.26 unidades
        if (bloque3EnMovimiento)
        {
            if (Mathf.Abs(bloque3.transform.position.x - posicionInicialBloque3.x) >= 0.003f)
            {
                // Detener el bloque 3
                bloque3.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque3EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }
        // Verificar si el bloque 4 se ha movido 6 unidades
        if (bloque4EnMovimiento)
        {
            if (Mathf.Abs(bloque4.transform.position.x - posicionInicialBloque4.x) >= 0.003f)
            {
                // Detener el bloque 3
                bloque4.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque4EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }
        // Verificar si el bloque 5 se ha movido 6 unidades
        if (bloque5EnMovimiento)
        {
            if (Mathf.Abs(bloque5.transform.position.x - posicionInicialBloque5.x) >= 0.003f)
            {
                // Detener el bloque 3
                bloque5.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque5EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }
        // Verificar si el bloque 6 se ha movido 6 unidades
        if (bloque6EnMovimiento)
        {
            if (Mathf.Abs(bloque6.transform.position.x - posicionInicialBloque6.x) >= 0.003f)
            {
                // Detener el bloque 3
                bloque6.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque6EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }
         // Verificar si el bloque 7 se ha movido 6 unidades
        if (bloque7EnMovimiento)
        {
            if (Mathf.Abs(bloque7.transform.position.x - posicionInicialBloque7.x) >= 0.002f)
            {
                // Detener el bloque 3
                bloque7.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque7EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }
                // Verificar si el bloque 8 se ha movido la distancia deseada
        if (bloque8EnMovimiento)
        {
            if (Mathf.Abs(bloque8.transform.position.x - posicionInicialBloque8.x) >= 0.002f)
            {
                // Detener el bloque 8
                bloque8.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque8EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }

        // Verificar si el bloque 9 se ha movido la distancia deseada
        if (bloque9EnMovimiento)
        {
            if (Mathf.Abs(bloque9.transform.position.x - posicionInicialBloque9.x) >= 0.002f)
            {
                // Detener el bloque 9
                bloque9.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque9EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }

        // Verificar si el bloque 10 se ha movido la distancia deseada
        if (bloque10EnMovimiento)
        {
            if (Mathf.Abs(bloque10.transform.position.x - posicionInicialBloque10.x) >= 0.002f)
            {
                // Detener el bloque 10
                bloque10.GetComponent<Rigidbody>().velocity = Vector3.zero;
                bloque10EnMovimiento = false; // Indica que el bloque ya no está en movimiento
            }
        }

    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Bloque1")) // Asegúrate de que no haya un punto y coma aquí
        {
            MoverBloque3();
            MoverBloque7();
        }else if (other.CompareTag("Bloque2")) // Asegúrate de que no haya un punto y coma aquí
        {
            MoverBloque4();
            MoverBloque8();
        }else if (other.CompareTag("Bloque3")) // Asegúrate de que no haya un punto y coma aquí
        {
            MoverBloque5();
            MoverBloque9();
        }else if (other.CompareTag("Bloque4")) // Asegúrate de que no haya un punto y coma aquí
        {
            MoverBloque6();
            MoverBloque10();
        }

    }

    void MoverBloque3()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque3 = bloque3.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque3 = bloque3.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque3.velocity = new Vector3(-1f, 0f, 0f) * (0.05f);

        bloque3EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque4()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque4 = bloque4.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque4 = bloque4.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque4.velocity = new Vector3(-1f, 0f, 0f) * 0.05f;

        bloque4EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque5()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque5 = bloque5.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque5 = bloque5.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque5.velocity = new Vector3(-1f, 0f, 0f) * 0.05f;

        bloque5EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque6()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque6 = bloque6.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque6 = bloque6.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque6.velocity = new Vector3(-1f, 0f, 0f) * 0.05f;

        bloque6EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque7()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque7 = bloque7.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque7 = bloque7.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque7.velocity = new Vector3(1f, 0f, 0f) * (0.03f);

        bloque7EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque8()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque8 = bloque8.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque8 = bloque8.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque8.velocity = new Vector3(1f, 0f, 0f) *(0.03f) ;

        bloque8EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque9()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque9 = bloque9.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque9 = bloque9.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque9.velocity = new Vector3(1f, 0f, 0f) * (0.03f);

        bloque9EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
    void MoverBloque10()
    {
        // Guardar la posición inicial del bloque 3
        posicionInicialBloque10 = bloque10.transform.position;

        // Obtener el componente Rigidbody del bloque 3
        Rigidbody rbBloque10 = bloque10.GetComponent<Rigidbody>();

        // Aplicar el movimiento
        rbBloque10.velocity = new Vector3(1f, 0f, 0f) * (0.03f);

        bloque10EnMovimiento = true; // Indica que el bloque 3 ha comenzado a moverse
    }
}
