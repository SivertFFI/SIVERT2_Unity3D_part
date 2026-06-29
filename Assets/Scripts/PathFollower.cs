/*
Copyright (c) <2025> <Aleksei Fedorov, aleksei.fedorov@eit.lth.se; Nikita Lyamin, nikita.lyamin@volvocars.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEditor;

public class PathFollower : MonoBehaviour
{
    public GameObject entityPrefab; // Assign the prefab in the inspector
    public Vector3Int axesMapping = new Vector3Int(0, 2, 1); // Mapping indices for x, y, z axes (0 = x, 1 = y, 2 = z)
    public Vector3 offset = new Vector3(0f, 0f, 0f);
    public Vector3Int multiplier = new Vector3Int(1, 1, 1);
    private string directoryPath; // Directory path is set in Awake()
    private GameObject entitiesParent; // Parent GameObject for all entities

    private int totalEntities;
    private int entitiesCompleted;

    void Awake()
    {
        Scene currentScene = SceneManager.GetActiveScene();
        directoryPath = "Assets/" + currentScene.name + "/paths";
        string filePath = System.IO.Path.Combine(Environment.CurrentDirectory, directoryPath);

        Debug.Log($"Path for PathFollower: {filePath}");

        // Ensure there is a parent GameObject for all entities
        entitiesParent = GameObject.Find("Entities") ?? new GameObject("Entities");

        // Load all CSV files and start path following
        LoadAndFollowPaths(filePath);
    }

    void LoadAndFollowPaths(string directory)
    {
        string[] files = Directory.GetFiles(directory, "*.csv");
        totalEntities = files.Length;
        entitiesCompleted = 0;

        foreach (var file in files)
        {
            StartCoroutine(FollowPath(file));
        }
    }

    IEnumerator FollowPath(string filePath)
    {
        List<Vector3> positions = new List<Vector3>();
        List<float> times = new List<float>();

        string[] lines = File.ReadAllLines(filePath);
        long lastTime = 0;

        foreach (string line in lines)
        {
            try
            {
                string[] parts = line.Split(',');

                if (parts.Length < 5)
                {
                    continue;
                }

                long unixTime = long.Parse(parts[0], CultureInfo.InvariantCulture); // Time in nanoseconds
                //float timeInSeconds = unixTime / 1e9f;

                if (lastTime == 0)
                {
                    lastTime = unixTime - Convert.ToInt64(0.1f * 1e9f); // Subtract 0.1 seconds to avoid instant teleportation
                }

                float deltaTime = (float)((unixTime - lastTime) / 1e9); // Delta time in seconds
                lastTime = unixTime;

                Vector3 position = new Vector3(
                    (float.Parse(parts[2 + axesMapping.x], CultureInfo.InvariantCulture) + offset.x)*multiplier.x,
                    (float.Parse(parts[2 + axesMapping.y], CultureInfo.InvariantCulture) + offset.y)*multiplier.y,
                    (float.Parse(parts[2 + axesMapping.z], CultureInfo.InvariantCulture) + offset.z)*multiplier.z
                );

                if (IsValidVector3(position))
                {
                    positions.Add(position);
                    times.Add(deltaTime);
                }
            }
            catch (Exception)
            {
                continue;
            }
        }

        if (positions.Count == 0)
        {
            yield break;
        }

        GameObject prefabToUse = entityPrefab != null ? entityPrefab : CreateTemporaryRedSphere();

        // Assuming positions is an array or list of Vector3
        Vector3 currentPosition = positions[0];
        Vector3 nextPosition = positions[1]; // Assuming there is at least one more position
        
        // Calculate the direction vector
        Vector3 direction = (nextPosition - currentPosition).normalized;
        
        // Create a rotation that points in the direction of the calculated vector
        Quaternion rotation = Quaternion.LookRotation(direction);
        
        // Instantiate the prefab with the calculated rotation
        GameObject entity = Instantiate(prefabToUse, currentPosition, rotation, entitiesParent.transform);

        entity.name = Path.GetFileNameWithoutExtension(filePath);
        entity.tag = "UE";

        if (entity.transform.Find("Antenna") == null)
        {
            // Add Antenna child object
            GameObject antenna = new GameObject("Antenna");
            antenna.transform.SetParent(entity.transform);
            antenna.transform.localPosition = new Vector3(0, 1, 0);
            antenna.tag = "Antenna";
        }

        int nextPointIndex = 1;
        float startTime = Time.time;
        Vector3 startPosition = positions[0];

        while (nextPointIndex < positions.Count)
        {
            float fracJourney = (Time.time - startTime) / times[nextPointIndex - 1];
            entity.transform.position = Vector3.Lerp(startPosition, positions[nextPointIndex], fracJourney);

            // Calculate the direction to the next point
            direction = positions[nextPointIndex] - entity.transform.position;
            if (direction != Vector3.zero)
            {
                // Update the rotation to look in the direction of movement
                entity.transform.rotation = Quaternion.LookRotation(direction.normalized);
            }

            if (fracJourney >= 1)
            {
                startPosition = positions[nextPointIndex];
                startTime = Time.time;
                nextPointIndex++;
            }
            yield return null;
        }

        OnEntityCompleted();

        if (prefabToUse != entityPrefab)
        {
            Destroy(entity);
        }
    }

    private void OnEntityCompleted()
    {
        entitiesCompleted++;
        Debug.Log($"Entity completed. Total completed: {entitiesCompleted}/{totalEntities}");

        if (entitiesCompleted >= totalEntities)
        {
            TerminateGame();
        }
    }

    private void TerminateGame()
    {
        Debug.Log("All entities have reached their destinations. Terminating the game.");
        #if UNITY_EDITOR
        EditorApplication.isPlaying = false; // Stop play mode in the Unity Editor
        #else
        Application.Quit(); // Quit the application
        #endif
    }

    private GameObject CreateTemporaryRedSphere()
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.localScale = Vector3.one * 0.2f; // Set default size
        Renderer rend = sphere.GetComponent<Renderer>();
        rend.material.color = Color.red; // Set color to red
        return sphere;
    }

    private bool IsValidVector3(Vector3 vector)
    {
        return !float.IsNaN(vector.x) && !float.IsNaN(vector.y) && !float.IsNaN(vector.z);
    }
}
