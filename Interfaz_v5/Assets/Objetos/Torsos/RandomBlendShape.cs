using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomBlendShape : MonoBehaviour
{
    private SkinnedMeshRenderer skinnedMeshRenderer;

    public int blendShapeIndex = 0;
    public float minValue = 0f;
    public float maxValue = 100f;

    // Rango excluido
    private float excludedMin = 50f;
    private float excludedMax = 76f;

    void Start()
    {
        skinnedMeshRenderer = GetComponent<SkinnedMeshRenderer>();

        if (skinnedMeshRenderer == null)
        {
            Debug.LogError("No se encontró un componente SkinnedMeshRenderer en el GameObject.");
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.O))
        {
            if (skinnedMeshRenderer != null && blendShapeIndex < skinnedMeshRenderer.sharedMesh.blendShapeCount)
            {
                float randomValue = GetRandomExcludingRange(minValue, maxValue, excludedMin, excludedMax);
                skinnedMeshRenderer.SetBlendShapeWeight(blendShapeIndex, randomValue);
                Debug.Log($"Blend Shape {blendShapeIndex} ajustado a: {randomValue}");
            }
            else
            {
                Debug.LogWarning("El índice del Blend Shape es inválido o el SkinnedMeshRenderer no tiene Blend Shapes.");
            }
        }
    }

    float GetRandomExcludingRange(float min, float max, float excludeMin, float excludeMax)
    {
        if (excludeMin <= min || excludeMax >= max)
        {
            Debug.LogWarning("Rango de exclusión está fuera del rango general. Se ignorará.");
            return Random.Range(min, max);
        }

        float totalValidLength = (excludeMin - min) + (max - excludeMax);
        float rand = Random.Range(0f, totalValidLength);

        if (rand < (excludeMin - min))
        {
            return min + rand;
        }
        else
        {
            return excludeMax + (rand - (excludeMin - min));
        }
    }
}
