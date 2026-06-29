using UnityEngine;
using System.Collections.Generic;
using System.IO;

public static class MeshLoader
{
    public static GameObject[] LoadAllMeshes(string resourcePath, Transform parentTransform)
    {
        Debug.Log("Loading all meshes from: " + resourcePath);

        // Check if directory exists
        string fullPath = Path.Combine(Application.dataPath, "Resources", resourcePath);
        Debug.Log("Full path: " + fullPath);
        if (!Directory.Exists(fullPath))
        {
            Debug.LogError($"Resource path does not exist: {fullPath}");
            return new GameObject[0];
        }

        // If parentTransform is empty, create a new GameObject called "Reflecting_Obstacles"
        if (parentTransform == null)
        {
            GameObject reflectingObstaclesParent = new GameObject("Reflecting_Obstacles");
            parentTransform = reflectingObstaclesParent.transform;
        }

        // Find all mesh assets in the specified resource path
        Debug.Log("Attempting to load meshes...");
        Mesh[] meshes = Resources.LoadAll<Mesh>(resourcePath);
        Debug.Log($"Loaded {meshes.Length} meshes.");

        List<GameObject> meshObjects = new List<GameObject>();

        if (meshes.Length == 0)
        {
            Debug.LogError("No meshes found at: " + resourcePath);
            return meshObjects.ToArray();
        }

        foreach (Mesh mesh in meshes)
        {
            if (mesh != null)
            {
                Debug.Log($"Processing mesh: {mesh.name}");
                GameObject meshObject = new GameObject(mesh.name);
                meshObject.transform.SetParent(parentTransform);
                MeshFilter meshFilter = meshObject.AddComponent<MeshFilter>();
                meshObject.transform.localScale = new Vector3(-1, 1, 1);
                MeshRenderer meshRenderer = meshObject.AddComponent<MeshRenderer>();
                meshRenderer.material = new Material(Shader.Find("Standard"));
                meshFilter.mesh = mesh;

                // Add a MeshCollider to the mesh object
                MeshCollider meshCollider = meshObject.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = mesh;

                meshObjects.Add(meshObject);
            }
            else
            {
                Debug.LogError("Failed to load a mesh from " + resourcePath);
            }
        }

        Debug.Log($"Loaded {meshObjects.Count} meshes from {resourcePath}");
        return meshObjects.ToArray();
    }
}
