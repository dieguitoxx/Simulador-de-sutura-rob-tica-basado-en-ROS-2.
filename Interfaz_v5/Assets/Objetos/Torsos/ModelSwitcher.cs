using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ModelSwitcher : MonoBehaviour
{
    public GameObject[] models; // Array para los modelos
    private int currentModelIndex = 0; // Índice del modelo activo

    void Start()
    {
        // Asegurarse de que solo el primer modelo esté activo al inicio
        UpdateModelVisibility();
    }

    void Update()
    {
        // Cambiar modelo con las teclas numéricas 1, 2, 3 y 4
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            currentModelIndex = 0;
            UpdateModelVisibility();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            currentModelIndex = 1;
            UpdateModelVisibility();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            currentModelIndex = 2;
            UpdateModelVisibility();
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            currentModelIndex = 3;
            UpdateModelVisibility();
        }
    }

    void UpdateModelVisibility()
    {
        // Activar solo el modelo correspondiente al índice actual
        for (int i = 0; i < models.Length; i++)
        {
            models[i].SetActive(i == currentModelIndex);
        }
    }
}
