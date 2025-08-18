using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MenuController : MonoBehaviour
{
    [Header("Menus")]
    public GameObject Panel_Izq;
    public GameObject Panel_Der;
    public GameObject PanelConexion;
    public GameObject PanelHerida;
    public GameObject PanelTrayectoria;
    public GameObject PanelTrayectoria2;

    [Header("Buttons")]
    public Button BotonIzq;
    public Button BotonDer;
    public Button BotonCerrarIzq;
    public Button BotonCerrarDer;
    public Button BotonConexion;
    public Button BotonAtrasC;
    public Button BotonHerida;
    public Button BotonAtrasH;
    public Button BotonTrayectoria;
    public Button BotonAtrasT;
    
    private void Start()
    {
        // Iniciar con el menú izquierdo abierto y derecho cerrado
        Panel_Izq.SetActive(true);
        Panel_Der.SetActive(false);
        BotonIzq.gameObject.SetActive(false);
        
        // Configurar listeners de los botones
        BotonIzq.onClick.AddListener(ToggleLeftMenu);
        BotonDer.onClick.AddListener(ToggleRightMenu);
        BotonCerrarIzq.onClick.AddListener(CloseLeftMenu);
        BotonCerrarDer.onClick.AddListener(CloseRightMenu);
        BotonConexion.onClick.AddListener(AbrirPanelConexion);
        BotonAtrasC.onClick.AddListener(CerrarPanelConexion);
        BotonHerida.onClick.AddListener(AbrirPanelHerida);
        BotonAtrasH.onClick.AddListener(CerrarPanelHerida);
        BotonTrayectoria.onClick.AddListener(AbrirPanelTrayectoria);
        BotonAtrasT.onClick.AddListener(CerrarPanelTrayectoria);
    }

    public void ToggleLeftMenu()
    {
        Panel_Izq.SetActive(!Panel_Izq.activeSelf);
        BotonIzq.gameObject.SetActive(false);
        BotonDer.gameObject.SetActive(true);
        // Cerrar el menú derecho si está abierto
        if(Panel_Der.activeSelf) 
            Panel_Der.SetActive(false);
    }

    public void ToggleRightMenu()
    {
        Panel_Der.SetActive(!Panel_Der.activeSelf);
        BotonDer.gameObject.SetActive(false);
        if(PanelHerida.activeSelf)
        	BotonIzq.gameObject.SetActive(false);
        else
        	BotonIzq.gameObject.SetActive(true);
        // Cerrar el menú izquierdo si está abierto
        if(Panel_Izq.activeSelf) 
            Panel_Izq.SetActive(false);
    }

    public void CloseLeftMenu()
    {
        Panel_Izq.SetActive(false);
        BotonIzq.gameObject.SetActive(true);
    }

    public void CloseRightMenu()
    {
        Panel_Der.SetActive(false);
        BotonDer.gameObject.SetActive(true);
    }
    // Abre el panel de conexión y cierra el principal
    public void AbrirPanelConexion()
    {
        if(Panel_Izq != null) Panel_Izq.SetActive(false);
        if(PanelConexion != null) PanelConexion.SetActive(true);
        BotonDer.gameObject.SetActive(false);
    }

    // Cierra el panel de conexión y vuelve al principal
    public void CerrarPanelConexion()
    {
        if(PanelConexion != null) PanelConexion.SetActive(false);
        if(Panel_Izq != null) Panel_Izq.SetActive(true);
        BotonDer.gameObject.SetActive(true);
    }
    // Abre el panel de herida
    public void AbrirPanelHerida()
    {
        if(Panel_Izq != null) Panel_Izq.SetActive(false);
        if(PanelHerida != null) PanelHerida.SetActive(true);
    }

    // Cierra el panel de herida
    public void CerrarPanelHerida()
    {
        if(PanelHerida != null) PanelHerida.SetActive(false);
        if(Panel_Izq != null) Panel_Izq.SetActive(true);
    }
    // Abre el panel de Trayectoria
    public void AbrirPanelTrayectoria()
    {
        if(Panel_Izq != null) Panel_Izq.SetActive(false);
        if(PanelTrayectoria != null) PanelTrayectoria.SetActive(true);
        if(PanelTrayectoria2 != null) PanelTrayectoria2.SetActive(true);
        BotonDer.gameObject.SetActive(false);
    }

    // Cierra el panel Trayectoria
    public void CerrarPanelTrayectoria()
    {
        if(PanelTrayectoria != null) PanelTrayectoria.SetActive(false);
        if(PanelTrayectoria2 != null) PanelTrayectoria2.SetActive(false);
        if(Panel_Izq != null) Panel_Izq.SetActive(true);
        BotonDer.gameObject.SetActive(true);
    }   
}
