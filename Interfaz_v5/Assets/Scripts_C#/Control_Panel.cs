/*******************
Autores:    Juan David Ruiz (juandavidrf@unicauca.edu.co)
            Sebastian Montenegro (exlogam@unicauca.edu.co)
*******************/

/********************* Librerias ********************/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Control_Panel : MonoBehaviour
{
    /**************** Variables ******************/
    //boolenos del estado de visibilidad los panesls
    [HideInInspector] public bool P_Principal;
    [HideInInspector] public bool P_Conexion;     
    [HideInInspector] public bool P_Generador;
    [HideInInspector] public bool P_Camara;
   

    //Objetos Panels
    public GameObject PanelMain;
    public GameObject PanelCon;
    public GameObject PanelGen;
    public GameObject PanelCam;



    /**************** Funciones ******************/
    void Start()                            //Inicacializa los panels (Solo el prinicpal visible)
    {
        P_Principal = true;
        P_Conexion = false;     
        P_Generador = false;
        P_Camara = false;
    }

    public void Conexion()                  //Activa y desactiva visibilidad del panel conexion
    {
        P_Conexion = !P_Conexion;
        P_Generador = false;
        P_Camara = false;
        CambioPanel();
    }

   


    public void GeneradorP()                     //Activa y desactiva visibilidad del panel de Generacion de trayectoria
    {
        P_Conexion = false;         
        P_Generador = !P_Generador;
        P_Camara = false;
        CambioPanel();
    }

    public void Cam()                     //Activa y desactiva visibilidad del panel de Generacion de trayectoria
    {
        P_Conexion = false;         
        P_Generador = false;
        P_Camara = !P_Camara;
        CambioPanel();
    }
  
    public void CambioPanel()               //Cambia la visibilidad de todos los paneles
    {
        PanelCon.SetActive(P_Conexion);
        PanelGen.SetActive(P_Generador);
        PanelCam.SetActive(P_Camara);
    }

    public void OcultarPanel()              //Cambia la visbilidad del panel principal
    {
        P_Principal = !P_Principal;
        PanelMain.SetActive(P_Principal);
    }
}
