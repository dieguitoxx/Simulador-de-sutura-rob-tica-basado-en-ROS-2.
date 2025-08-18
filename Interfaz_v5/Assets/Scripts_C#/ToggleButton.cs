using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class ToggleButton : MonoBehaviour
{
    public Button conectarButton;
    public Button desconectarButton;
    public TMP_Text estadoText;
    
    private string ultimoEstado; // Para rastrear cambios

    void Start()
    {
        if (conectarButton == null || desconectarButton == null || estadoText == null)
        {
            Debug.LogError("Faltan asignar componentes en el inspector!");
            return;
        }

        // Guardar el estado inicial
        ultimoEstado = estadoText.text;
        ActualizarUI();
    }

    void Update()
    {
        // Verificar si el texto ha cambiado
        if (estadoText.text != ultimoEstado)
        {
            ultimoEstado = estadoText.text;
            ActualizarUI();
        }
    }

    void ActualizarUI()
    {
        // Usamos Contains para mayor flexibilidad en el texto
        bool mostrarConectar = estadoText.text.Contains("Desconectado") || 
                              estadoText.text.Contains("fallida") || 
                              estadoText.text.Contains("Falló");
        
        bool mostrarDesconectar = estadoText.text.Contains("Conectado") && 
                                !estadoText.text.Contains("Conectando");

        conectarButton.gameObject.SetActive(mostrarConectar);
        desconectarButton.gameObject.SetActive(mostrarDesconectar);
    }
}
