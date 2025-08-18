using UnityEngine;

public class DuplicateAndRename : MonoBehaviour
{
    public GameObject objectToDuplicate;  // Arrastra aquí en el Editor de Unity el objeto que quieres duplicar

    void Start()
    {
        // Duplicar el objeto
        GameObject duplicateObject = Instantiate(objectToDuplicate);

        // Cambiar el nombre del objeto duplicado
        duplicateObject.name = objectToDuplicate.name + "_2";

        // Renombrar todos los hijos
        RenameChildren(duplicateObject.transform, "_2");
    }

    // Función recursiva para renombrar todos los hijos del objeto
    void RenameChildren(Transform parentTransform, string suffix)
    {
        foreach (Transform child in parentTransform)
        {
            child.name += suffix;
            RenameChildren(child, suffix);
        }
    }
}

