using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CilindroScript : MonoBehaviour
{
    public bool tieneAguja = false;
    public GameObject agujaContainer;
    private Vector3 posicionInicialAguja;
    private Quaternion rotacionInicialAguja;

    private void Start()
    {
        // Al inicio, obtiene los datos iniciales de la aguja si est� presente
        ObtenerDatosInicialesAguja();
    }

    private void OnTriggerStay(Collider other)
    {
        // Verificar si se presiona la barra espaciadora y si no tiene la aguja
        if (Input.GetKeyDown(KeyCode.Space) && !tieneAguja && other.CompareTag("Aguja"))
        {
            AgarrarAguja(other.gameObject);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        SoltarAguja();
    }

    private void Update()
    {
        // Verificar si se presiona la tecla 'C'
        if (Input.GetKeyDown(KeyCode.C))
        {
            // Mover la aguja a su posici�n y rotaci�n inicial desplazada en Y
            if (tieneAguja && agujaContainer != null)
            {
                // Obtener la aguja como GameObject antes de destruir el contenedor
                GameObject aguja = null;
                if (agujaContainer.transform.childCount > 0)
                {
                    aguja = agujaContainer.transform.GetChild(0).gameObject;

                    // Limpiar la referencia al contenedor
                    aguja.transform.parent = null;

                    // Restaurar la posici�n y rotaci�n inicial
                    aguja.transform.position = posicionInicialAguja + new Vector3(0f, 0f, 0.01142f)+new Vector3(-0.01196285f,0f,0f);
                    aguja.transform.rotation = rotacionInicialAguja;
                }

                // Destruir el contenedor despu�s de liberar la aguja
                Destroy(agujaContainer);
                tieneAguja = false;
            }

            // Ambos cilindros tienen la variable tieneAguja en false
            CilindroScript[] cilindros = FindObjectsOfType<CilindroScript>();
            foreach (CilindroScript cilindro in cilindros)
            {
                cilindro.tieneAguja = false;
            }
        }
    }

    private void AgarrarAguja(GameObject aguja)
    {
        tieneAguja = true;

        if (agujaContainer == null)
        {
            agujaContainer = new GameObject("AgujaContainer");
            agujaContainer.transform.parent = transform;
            agujaContainer.transform.localPosition = Vector3.zero;
        }

        // Obtener el punto de contacto en el cilindro
        Vector3 puntoDeContacto = aguja.transform.position;

        // Ajustar la posici�n local de la aguja en relaci�n con el punto de contacto
        aguja.transform.parent = agujaContainer.transform;
        aguja.transform.localPosition = agujaContainer.transform.InverseTransformPoint(puntoDeContacto);
    }

    private void SoltarAguja()
    {
        if (tieneAguja && agujaContainer != null)
        {
            // Obtener la aguja como GameObject antes de destruir el contenedor
            GameObject aguja = null;
            if (agujaContainer.transform.childCount > 0)
            {
                aguja = agujaContainer.transform.GetChild(0).gameObject;
                // Limpiar la referencia al contenedor
                aguja.transform.parent = null;
            }

            // Destruir el contenedor despu�s de liberar la aguja
            Destroy(agujaContainer);
            tieneAguja = false;
        }
    }

    private void ObtenerDatosInicialesAguja()
    {
        // Buscar el objeto aguja en la escena
        GameObject aguja = GameObject.FindGameObjectWithTag("Aguja");

        // Si se encuentra la aguja, obtener y almacenar los datos iniciales
        if (aguja != null)
        {
            posicionInicialAguja = aguja.transform.position;
            rotacionInicialAguja = aguja.transform.rotation;

            // Depuraci�n: Imprimir la posici�n y rotaci�n inicial
            Debug.Log("Posici�n Inicial de la Aguja: " + posicionInicialAguja.ToString("F6"));
     	Debug.Log("Rotaci�n Inicial de la Aguja: " + rotacionInicialAguja.ToString("F6"));

        }
        else
        {
            // Depuraci�n: Imprimir un mensaje si no se encuentra la aguja
            Debug.LogError("�Error! No se encontr� el objeto de la aguja en la escena.");
        }
    }
}

