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
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using Unity.Collections;

public static class ObservationPointSelector
{
    private const float defaultDistance = 10f;

    public static NativeArray<Vector3> SelectObservationPoints(string directoryPath, float distance, GameObject[] visualObstacles, Vector3Int axesMapping, GameObject[] additionalOPs, string parentName = "ObservationPoints", string saveFileName = "ObservationPoints.json")
    {
        Debug.Log("Starting observation point selection process...");

        // Ensure there is a parent GameObject for observation points
        GameObject parentObject = GameObject.Find(parentName) ?? new GameObject(parentName);

        // Check if the save file exists
        string saveFilePath = Path.Combine(directoryPath, saveFileName);
        NativeArray<Vector3> observationPointsArray;

        if (File.Exists(saveFilePath))
        {
            Debug.Log($"Save file found at {saveFilePath}. Loading observation points from file...");
            observationPointsArray = LoadObservationPoints(saveFilePath);
        }
        else
        {
            List<Vector3> allPoints = new List<Vector3>();
            string[] files = Directory.GetFiles(directoryPath, "*.csv");

            // Collect points from all CSV files
            Debug.Log($"Found {files.Length} CSV files in directory {directoryPath}.");
            foreach (var file in files)
            {
                Debug.Log($"Loading points from {file}...");
                List<Vector3> points = LoadPointsFromCSV(file, axesMapping);
                allPoints.AddRange(points);
            }

            // Select points with appropriate distance
            Debug.Log("Selecting points with appropriate distance...");
            List<Vector3> selectedPoints = SelectPointsWithDistance(allPoints, distance);

            // Filter points based on visibility
            Debug.Log("Filtering points based on visibility...");
            List<Vector3> visiblePoints = new List<Vector3>();
            for (int i = 0; i < selectedPoints.Count; i++)
            {
                bool visible = true;
                for (int j = 0; j < selectedPoints.Count; j++)
                {
                    if (i != j && !ArePointsVisible(selectedPoints[i], selectedPoints[j], visualObstacles))
                    {
                        visible = false;
                        break;
                    }
                }
                if (visible)
                {
                    visiblePoints.Add(selectedPoints[i]);
                }
            }


            foreach (GameObject additionalOP in additionalOPs){
                visiblePoints.Add(additionalOP.transform.position);
            }

            Debug.Log($"Selected {visiblePoints.Count} visible points out of {selectedPoints.Count}.");
            observationPointsArray = new NativeArray<Vector3>(visiblePoints.ToArray(), Allocator.Persistent);

            // Save the points to a file
            Debug.Log($"Saving observation points to {saveFilePath}...");
            SaveObservationPoints(observationPointsArray, saveFilePath);
        }



        Debug.Log("Observation point selection process completed.");
        CreateWhiteCubes(observationPointsArray, parentObject.transform);
        return observationPointsArray;
    }

    private static List<Vector3> LoadPointsFromCSV(string filePath, Vector3Int axesMapping)
    {
        List<Vector3> positions = new List<Vector3>();
        string[] lines = File.ReadAllLines(filePath);

        foreach (string line in lines)
        {
            string[] parts = line.Split(',');
            if (parts.Length < 3) continue;

            try
            {
                Vector3 position = new Vector3(
                    float.Parse(parts[2 + axesMapping.x], CultureInfo.InvariantCulture),
                    float.Parse(parts[2 + axesMapping.y], CultureInfo.InvariantCulture),
                    float.Parse(parts[2 + axesMapping.z], CultureInfo.InvariantCulture)
                );

                if (IsValidVector3(position))
                {
                    positions.Add(position);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Failed to parse line in {filePath}: {line}. Error: {ex.Message}");
                continue;
            }
        }

        Debug.Log($"Loaded {positions.Count} valid points from {filePath}.");
        return positions;
    }

    private static List<Vector3> SelectPointsWithDistance(List<Vector3> points, float distance)
    {
        List<Vector3> selectedPoints = new List<Vector3>();
        for (int i = 0; i < points.Count; i++)
        {
            bool tooClose = false;
            foreach (Vector3 selectedPoint in selectedPoints)
            {
                if (Vector3.Distance(points[i], selectedPoint) < distance)
                {
                    tooClose = true;
                    break;
                }
            }
            if (!tooClose)
            {
                selectedPoints.Add(points[i]);
            }
        }

        Debug.Log($"Selected {selectedPoints.Count} points with a minimum distance of {distance}.");
        return selectedPoints;
    }

    private static bool ArePointsVisible(Vector3 pointA, Vector3 pointB, GameObject[] visualObstacles)
    {
        Ray ray = new Ray(pointA, pointB - pointA);
        float distance = Vector3.Distance(pointA, pointB);

        foreach (GameObject obstacle in visualObstacles)
        {
            if (obstacle.GetComponent<Collider>().Raycast(ray, out RaycastHit hit, distance))
            {
                return false;
            }
        }
        return true;
    }

    private static void SaveObservationPoints(NativeArray<Vector3> observationPoints, string filePath)
    {
        List<Vector3> positions = new List<Vector3>(observationPoints);
        string json = JsonUtility.ToJson(new Vector3ArrayWrapper { positions = positions.ToArray() });
        File.WriteAllText(filePath, json);
        Debug.Log($"Saved {observationPoints.Length} observation points to {filePath}.");
    }

    private static NativeArray<Vector3> LoadObservationPoints(string filePath)
    {
        string json = File.ReadAllText(filePath);
        Vector3ArrayWrapper wrapper = JsonUtility.FromJson<Vector3ArrayWrapper>(json);
        Debug.Log($"Loaded {wrapper.positions.Length} observation points from {filePath}.");
        return new NativeArray<Vector3>(wrapper.positions, Allocator.Persistent);
    }

    private static bool IsValidVector3(Vector3 vector)
    {
        return !float.IsNaN(vector.x) && !float.IsNaN(vector.y) && !float.IsNaN(vector.z);
    }

    private static void CreateWhiteCubes(NativeArray<Vector3> points, Transform parentTransform)
    {
        foreach (Vector3 point in points)
        {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            cube.transform.position = point;
            cube.transform.localScale = Vector3.one * 0.2f;
            Renderer rend = cube.GetComponent<Renderer>();
            rend.material.color = Color.white;
            cube.transform.SetParent(parentTransform);
            Debug.Log($"Created white cube at position {point}.");
        }
    }

    [Serializable]
    private class Vector3ArrayWrapper
    {
        public Vector3[] positions;
    }
}
