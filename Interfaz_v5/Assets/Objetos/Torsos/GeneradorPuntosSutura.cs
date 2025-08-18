using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;
using System.IO;

public class GeneradorPuntosSutura : MonoBehaviour
{
    [Header("Configuración")]
    public float distance = 1f;
    public Color laserColor = Color.green;
    public float pointSize = 0.1f;
    public int maxSuturePairs = 3; // Límite de pares de puntos

    [Header("Referencias")]
    public LayerMask suturableLayer;
    public RawImage targetRawImage;
    public Camera sutureCamera;

    private bool sutureMode = false;
    private List<Vector3> suturePoints = new List<Vector3>();
    private GameObject laserPoint;
    private GameObject secondPreviewPoint;
    private GameObject pointsContainer;
    private float currentRotation = 0f;
    private int currentPairCount = 0;

    void Start()
    {
        pointsContainer = new GameObject("SuturePoints");

        laserPoint = CreateSphere(laserColor);
        laserPoint.name = "LaserPoint";
        laserPoint.SetActive(false);

        secondPreviewPoint = CreateSphere(Color.yellow);
        secondPreviewPoint.name = "PreviewPoint";
        secondPreviewPoint.SetActive(false);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.P))
        {
            sutureMode = !sutureMode;
            laserPoint.SetActive(sutureMode);
            secondPreviewPoint.SetActive(sutureMode);
            Debug.Log("Modo sutura: " + (sutureMode ? "Activado" : "Desactivado"));
        }

        if (Input.GetKeyDown(KeyCode.B))
        {
            ClearSuturePoints();
            Debug.Log("Puntos borrados.");
        }

        if (Input.GetKeyDown(KeyCode.S))
        {
            SavePointsToFile();
            Debug.Log("Puntos guardados en archivo.");
        }

        if (Input.GetKeyDown(KeyCode.UpArrow))
        {
            distance += 0.01f;
            Debug.Log("Distancia aumentada: " + distance.ToString("F2"));
        }

        if (Input.GetKeyDown(KeyCode.DownArrow))
        {
            distance = Mathf.Max(0.01f, distance - 0.01f);
            Debug.Log("Distancia reducida: " + distance.ToString("F2"));
        }

        if (Input.GetKeyDown(KeyCode.LeftArrow))
        {
            currentRotation -= 10f;
            Debug.Log("Rotación: " + currentRotation);
        }

        if (Input.GetKeyDown(KeyCode.RightArrow))
        {
            currentRotation += 10f;
            Debug.Log("Rotación: " + currentRotation);
        }

        if (sutureMode && currentPairCount < maxSuturePairs)
        {
            HandleSuturePlacement();
        }
        else if (currentPairCount >= maxSuturePairs)
        {
            laserPoint.SetActive(false);
            secondPreviewPoint.SetActive(false);
        }
    }

    void HandleSuturePlacement()
    {
        if (!IsMouseOverRawImage())
            return;

        Vector2 localPoint;
        RectTransformUtility.ScreenPointToLocalPointInRectangle(
            targetRawImage.rectTransform, 
            Input.mousePosition, 
            null, 
            out localPoint);

        Rect rect = targetRawImage.rectTransform.rect;
        Vector2 normalizedPoint = new Vector2(
            (localPoint.x + rect.width * 0.5f) / rect.width,
            (localPoint.y + rect.height * 0.5f) / rect.height);

        Ray ray = sutureCamera.ViewportPointToRay(normalizedPoint);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, Mathf.Infinity, suturableLayer))
        {
            laserPoint.SetActive(true);
            laserPoint.transform.position = hit.point;

            Vector3 direction = Quaternion.Euler(0, currentRotation, 0) * Vector3.right;
            Vector3 secondPos = hit.point + direction.normalized * distance;

            secondPreviewPoint.SetActive(true);
            secondPreviewPoint.transform.position = secondPos;

            if (Input.GetMouseButtonDown(0))
            {
                CreateSuturePoint(hit.point, secondPos);
                currentPairCount++;
                Debug.Log($"Par de puntos {currentPairCount}/{maxSuturePairs} creado");
            }
        }
        else
        {
            laserPoint.SetActive(false);
            secondPreviewPoint.SetActive(false);
        }
    }

    void CreateSuturePoint(Vector3 position1, Vector3 position2)
    {
        // Punto de entrada (verde)
        GameObject point1 = CreateSphere(Color.green);
        point1.transform.parent = pointsContainer.transform;
        point1.transform.position = position1;
        suturePoints.Add(position1);

        // Punto de salida (rojo)
        GameObject point2 = CreateSphere(Color.red);
        point2.transform.parent = pointsContainer.transform;
        point2.transform.position = position2;
        suturePoints.Add(position2);

        // Conectar con una línea
        /*GameObject line = new GameObject("SutureLine");
        line.transform.parent = pointsContainer.transform;
        LineRenderer lr = line.AddComponent<LineRenderer>();
        lr.material = new Material(Shader.Find("Standard"));
        lr.startColor = Color.blue;
        lr.endColor = Color.blue;
        lr.startWidth = 0.02f;
        lr.endWidth = 0.02f;
        lr.SetPositions(new Vector3[] { position1, position2 });*/
    }

    void SavePointsToFile()
    {
        string path = Application.dataPath + "/SuturePoints.txt";
        StreamWriter writer = new StreamWriter(path, false);

        writer.WriteLine("Coordenadas de puntos de sutura");
        writer.WriteLine($"Fecha: {System.DateTime.Now}");
        writer.WriteLine($"Total pares: {currentPairCount}");
        writer.WriteLine("----------------------------");

        for (int i = 0; i < suturePoints.Count; i += 2)
        {
            int pairNumber = (i / 2) + 1;
            writer.WriteLine($"Par #{pairNumber}:");
            writer.WriteLine($"  Entrada: {suturePoints[i].ToString("F4")}");
            writer.WriteLine($"  Salida: {suturePoints[i+1].ToString("F4")}");
            writer.WriteLine($"  Distancia: {Vector3.Distance(suturePoints[i], suturePoints[i+1]).ToString("F4")}");
            writer.WriteLine();
        }

        writer.Close();
        Debug.Log($"Archivo guardado en: {path}");
    }

    bool IsMouseOverRawImage()
    {
        Vector2 localMousePosition = targetRawImage.rectTransform.InverseTransformPoint(Input.mousePosition);
        return targetRawImage.rectTransform.rect.Contains(localMousePosition);
    }

    GameObject CreateSphere(Color color)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.localScale = Vector3.one * pointSize;

        Material material = new Material(Shader.Find("Standard"));
        material.color = color;
        sphere.GetComponent<Renderer>().material = material;

        Destroy(sphere.GetComponent<Collider>());

        return sphere;
    }

    public Vector3[] GetSuturePoints()
    {
        return suturePoints.ToArray();
    }

    public void ClearSuturePoints()
    {
        suturePoints.Clear();
        currentPairCount = 0;
        foreach (Transform child in pointsContainer.transform)
        {
            Destroy(child.gameObject);
        }
    }
}
