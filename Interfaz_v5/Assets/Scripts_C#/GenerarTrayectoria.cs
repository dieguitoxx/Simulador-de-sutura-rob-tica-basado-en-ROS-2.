using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GenerarTrayectoria : MonoBehaviour
{
    public string mov1FileName;

    void Start()
    {
        Button btn = GetComponent<Button>();
        btn.onClick.AddListener(OnClick);
    }

    void OnClick()
    {
        // Crea una instancia de la clase mov1 desde el archivo especificado.
        mov1 movimiento = gameObject.AddComponent(System.Type.GetType(mov1FileName)) as mov1;

        // Llama al método Start en la instancia creada.
        movimiento.Start();
    }
}
