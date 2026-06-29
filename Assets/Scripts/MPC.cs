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
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System;
using System.Linq;

public static class MPC
{ 
    public struct ScatterPoint
    {
        public float3 Point;
        public float3 Normal;
        public float Attentuation; 

        public ScatterPoint(Vector3 point, Vector3 normal, float attenuation)
        {
            this.Point = point;
            this.Normal = normal;
            this.Attentuation = attenuation;
        }
    }
    public static (NativeArray<ScatterPoint>, NativeArray<ScatterPoint>, NativeArray<ScatterPoint>) CreateMPCFromPointCloud(
        string pathPointCloud,
        int skipEveryNthPoint,
        Vector3Int axesMapping,
        float chiW1,
        float chiW2,
        float chiW3,
        float2 attenuation1,
        float2 attenuation2,
        float2 attenuation3,
        NativeArray<Vector3> observationPoints,
        float distance1,
        float distance2,
        float distance3,
        float offset,
        GameObject[] reflectingObstacles)
    {
        List<Vector3> points = new List<Vector3>();
        List<Vector3> normals = new List<Vector3>();

        LoadPointCloud(pathPointCloud, skipEveryNthPoint, axesMapping, points, normals);

        NativeArray<Vector3> positionsArray = new NativeArray<Vector3>(points.ToArray(), Allocator.TempJob);
        NativeArray<Vector3> normalsArray = new NativeArray<Vector3>(normals.ToArray(), Allocator.TempJob);

        NativeArray<ScatterPoint> MPC1 = SelectScatterPoints(positionsArray, normalsArray, chiW1, attenuation1, observationPoints, distance1, offset, reflectingObstacles);
        NativeArray<ScatterPoint> MPC2 = SelectScatterPoints(positionsArray, normalsArray, chiW2, attenuation2, observationPoints, distance2, offset, reflectingObstacles);
        NativeArray<ScatterPoint> MPC3 = SelectScatterPoints(positionsArray, normalsArray, chiW3, attenuation3, observationPoints, distance3, offset, reflectingObstacles);

        positionsArray.Dispose();
        normalsArray.Dispose();

        return (MPC1, MPC2, MPC3);
    }

    public static void LoadPointCloud(string pathPointCloud, int skipEveryNPoints, Vector3Int axesMapping, List<Vector3> points, List<Vector3> normals)
    {
        string[] pointCloudLines = File.ReadAllLines(pathPointCloud);

        for (int i = 1; i < pointCloudLines.Length; i += skipEveryNPoints) // Start at 1 to skip header
        {
            string line = pointCloudLines[i];

            try
            {
                string[] parts = line.Split(new char[] { ',' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 6)
                {
                    Vector3 position = new Vector3(
                        float.Parse(parts[axesMapping.x], CultureInfo.InvariantCulture),
                        float.Parse(parts[axesMapping.y], CultureInfo.InvariantCulture),
                        float.Parse(parts[axesMapping.z], CultureInfo.InvariantCulture)
                    );
                    points.Add(position);

                    Vector3 normal = new Vector3(
                        float.Parse(parts[axesMapping.x + 3], CultureInfo.InvariantCulture),
                        float.Parse(parts[axesMapping.y + 3], CultureInfo.InvariantCulture),
                        float.Parse(parts[axesMapping.z + 3], CultureInfo.InvariantCulture)
                    );
                    normals.Add(normal);
                }
            }
            catch (Exception ex)
            {
                Debug.LogWarning($"Failed to parse line {i}: {ex.Message}");
                continue;
            }
        }
    }

    private static NativeArray<ScatterPoint> SelectScatterPoints(
        NativeArray<Vector3> positionsArray,
        NativeArray<Vector3> normalsArray,
        float chi,
        float2 attenuation,
        NativeArray<Vector3> observationPoints,
        float distance,
        float offset,
        GameObject[] reflectingObstacles)
    {
        int numMpc = Mathf.FloorToInt(positionsArray.Length * chi);
        NativeArray<ScatterPoint> scatterPoints = new NativeArray<ScatterPoint>(numMpc, Allocator.Persistent);

        int[] selectedCandidatePoints = GetUniqueRandomNumbers(0, positionsArray.Length, numMpc);

        NativeArray<Vector3> possiblePoints = new NativeArray<Vector3>(numMpc, Allocator.TempJob);
        NativeArray<Vector3> possibleNormals = new NativeArray<Vector3>(numMpc, Allocator.TempJob);

        for (int i = 0; i < numMpc; i++)
        {
            int index = selectedCandidatePoints[i];
            possiblePoints[i] = positionsArray[index];
            possibleNormals[i] = normalsArray[index];
        }

        if (observationPoints.Length == 0)
        {
            for (int i = 0; i < numMpc; i++)
            {
                scatterPoints[i] = new ScatterPoint(possiblePoints[i], possibleNormals[i], UnityEngine.Random.Range(attenuation.x, attenuation.y));
            }
        }
        else
        {
            // Move the points along their normals, flipping normals if necessary
            for (int i = 0; i < numMpc; i++)
            {
                float minDistSq = float.MaxValue;
                Vector3 nearestObsPoint = Vector3.zero;
                for (int j = 0; j < observationPoints.Length; j++)
                {
                    float distSq = (possiblePoints[i] - observationPoints[j]).sqrMagnitude;
                    if (distSq < minDistSq)
                    {
                        minDistSq = distSq;
                        nearestObsPoint = observationPoints[j];
                    }
                }

                // Check if the normal is pointing away from the observation point
                Vector3 directionToObs = (nearestObsPoint - possiblePoints[i]).normalized;
                if (Vector3.Dot(possibleNormals[i], directionToObs) < 0)
                {
                    possibleNormals[i] = -possibleNormals[i];
                }

                // Move the point along the normal by the offset
                possiblePoints[i] = possiblePoints[i] + possibleNormals[i] * offset;
            }

            scatterPoints = FilterScatterPoints(possiblePoints, possibleNormals, observationPoints, distance, attenuation, reflectingObstacles);
        }

        possiblePoints.Dispose();
        possibleNormals.Dispose();

        return scatterPoints;
    }


    private static NativeArray<ScatterPoint> FilterScatterPoints(
        NativeArray<Vector3> possiblePoints,
        NativeArray<Vector3> possibleNormals,
        NativeArray<Vector3> observationPoints,
        float distance,
        float2 attenuation,
        GameObject[] reflectingObstacles)
    {
        int numMpc = possiblePoints.Length;
        int numObs = observationPoints.Length;

        NativeArray<RaycastCommand> commandsNativeArrayMpc = new NativeArray<RaycastCommand>(numMpc * numObs, Allocator.TempJob);
        NativeArray<RaycastHit> resultsNativeArrayMpc = new NativeArray<RaycastHit>(numMpc * numObs, Allocator.TempJob);
        NativeList<ScatterPoint> resultList = new NativeList<ScatterPoint>(Allocator.TempJob);

        // Parallel job for generating raycast commands
        ParallelRayCastingData parallelRayCastingDataMpc = new ParallelRayCastingData
        {
            MPC_Array = possiblePoints,
            OBS_Array = observationPoints,
            commands = commandsNativeArrayMpc,
        };
        JobHandle jobHandleRayCastingDataMpc = parallelRayCastingDataMpc.Schedule(numMpc * numObs, 64);
        jobHandleRayCastingDataMpc.Complete();

        // Raycast and identify hits
        JobHandle rayCastJobMpc = RaycastCommand.ScheduleBatch(commandsNativeArrayMpc, resultsNativeArrayMpc, 5, jobHandleRayCastingDataMpc);
        rayCastJobMpc.Complete();

        for (int i = 0; i < numMpc; i++)
        {
            for (int j = 0; j < numObs; j++)
            {
                if (resultsNativeArrayMpc[i + j * numMpc].distance == 0)
                {
                    resultList.Add(new ScatterPoint(possiblePoints[i], possibleNormals[i], UnityEngine.Random.Range(attenuation.x, attenuation.y)));
                    break;
                }
            }
        }

        NativeArray<ScatterPoint> finalResults = resultList.ToArray(Allocator.Persistent);

        commandsNativeArrayMpc.Dispose();
        resultsNativeArrayMpc.Dispose();
        resultList.Dispose();

        return finalResults;
    }

    private static int[] GetUniqueRandomNumbers(int min, int max, int count)
    {
        HashSet<int> uniqueNumbers = new HashSet<int>();
        while (uniqueNumbers.Count < count)
        {
            uniqueNumbers.Add(UnityEngine.Random.Range(min, max));
        }
        return uniqueNumbers.ToArray();
    }

    public static NativeArray<ScatterPoint> spawnScatterPoints(
        int pathOrder,
        GameObject[] spawningPlanes,
        GameObject[] reflectingObstacles,
        float chi,
        float distance,
        float2 attenuation,
        NativeArray<Unity.Mathematics.Random> rndNums,
        NativeArray<Vector3> observationPoints
    )
    {
        // Get spawning area
        float totalSpawningArea;
        UnityEngine.Vector3 _center, _size;
        (_center, _size, totalSpawningArea) = GetSpawningArea(spawningPlanes[0]);

        // Calculate number of MPCs
        int numMpc = Mathf.FloorToInt(totalSpawningArea * chi);

        NativeArray<Vector3> possiblePoints = new NativeArray<Vector3>(numMpc, Allocator.TempJob);
        
        // Spawn MPCs uniformly
        PlaneMpcSpawner spawner = new PlaneMpcSpawner
        {
            Center = _center,
            Size = _size,
            MaxHight = 30f,
            Rngs = rndNums,
            Array = possiblePoints,
        };
        JobHandle jobHandler = spawner.Schedule(numMpc, 1);
        jobHandler.Complete();

        // Filter out MPCs based on observation points
        using (NativeArray<RaycastCommand> commandsNativeArrayMpc = new NativeArray<RaycastCommand>(numMpc * observationPoints.Length, Allocator.TempJob))
        using (NativeArray<RaycastHit> resultsNativeArrayMpc = new NativeArray<RaycastHit>(numMpc * observationPoints.Length, Allocator.TempJob))
        {
            // Parallel job for generating raycast commands
            ParallelRayCastingData parallelRayCastingDataMpc = new ParallelRayCastingData
            {
                MPC_Array = possiblePoints,
                OBS_Array = observationPoints,
                commands = commandsNativeArrayMpc,
            };
            JobHandle jobHandleRayCastingDataMpc = parallelRayCastingDataMpc.Schedule(possiblePoints.Length * observationPoints.Length, 64);
            jobHandleRayCastingDataMpc.Complete();

            // Raycast and identify hits
            JobHandle rayCastJobMpc = RaycastCommand.ScheduleBatch(commandsNativeArrayMpc, resultsNativeArrayMpc, 5, jobHandleRayCastingDataMpc);
            rayCastJobMpc.Complete();
                
            int pointsCount = 0;
            for (int i = 0; i < possiblePoints.Length; i++)
            {
                if (resultsNativeArrayMpc[i].distance == 0)
                {
                    pointsCount++;
                }
            }
            Debug.Log("pointsCount: " + pointsCount);

            if (pathOrder == 1)
            {
                // Combine _reflectingObstacles and _spawningPlanes into a single array
                reflectingObstacles = reflectingObstacles.Concat(spawningPlanes).ToArray();
            }
                
                
            using NativeList<ScatterPoint> resultList = new NativeList<ScatterPoint>(Allocator.TempJob);

            foreach (GameObject reflectingObstacle in reflectingObstacles)
            {
                Renderer renderer = reflectingObstacle.GetComponent<Renderer>();
                Bounds bounds = renderer.bounds;

                        

                if (reflectingObstacle.name == "SpawningCube")
                {
                    bounds.Expand(1.0f);

                    Debug.Log("MPCSpawningPlane");
                    int inBoundsCount = 0;
                    for (int i = 0; i < possiblePoints.Length; i++)
                    {
                        Vector3 point = possiblePoints[i];
                        if (bounds.Contains(point))
                        {
                            inBoundsCount++;
                        }
                    }
                    Debug.Log("inBoundsCount: " + inBoundsCount);
                }
                else
                {
                    bounds.Expand(distance);
                }

                for (int i = 0; i < possiblePoints.Length; i++)
                {
                    Vector3 point = possiblePoints[i];
                    if (bounds.Contains(point))
                    {
                        bool isVisible = false;
                        for (int obsPointIndex = 0; obsPointIndex < observationPoints.Length; obsPointIndex++)
                        {
                            if (resultsNativeArrayMpc[i + obsPointIndex * possiblePoints.Length].distance == 0)
                            {
                                isVisible = true;
                                break;
                            }
                        }
                        if (isVisible)
                        {
                            (float GPTDistnance, float3 nearestNormalGPT) = NearestNormalFinderGPT(reflectingObstacle, point);
                            if (GPTDistnance <= distance)
                            {
                                resultList.Add(new ScatterPoint(point, nearestNormalGPT,
                                    UnityEngine.Random.Range(attenuation.x, attenuation.y)));
                            }
                        }
                    }
                }
            }
            possiblePoints.Dispose();
                    
            return resultList.ToArray(Allocator.Persistent);

            
        }
    }

    static void DrawBounds(Bounds b, float delay = 0)
    {
        // bottom
        var p1 = new Vector3(b.min.x, b.min.y, b.min.z);
        var p2 = new Vector3(b.max.x, b.min.y, b.min.z);
        var p3 = new Vector3(b.max.x, b.min.y, b.max.z);
        var p4 = new Vector3(b.min.x, b.min.y, b.max.z);

        Debug.DrawLine(p1, p2, Color.blue, delay);
        Debug.DrawLine(p2, p3, Color.red, delay);
        Debug.DrawLine(p3, p4, Color.yellow, delay);
        Debug.DrawLine(p4, p1, Color.magenta, delay);

        // top
        var p5 = new Vector3(b.min.x, b.max.y, b.min.z);
        var p6 = new Vector3(b.max.x, b.max.y, b.min.z);
        var p7 = new Vector3(b.max.x, b.max.y, b.max.z);
        var p8 = new Vector3(b.min.x, b.max.y, b.max.z);

        Debug.DrawLine(p5, p6, Color.blue, delay);
        Debug.DrawLine(p6, p7, Color.red, delay);
        Debug.DrawLine(p7, p8, Color.yellow, delay);
        Debug.DrawLine(p8, p5, Color.magenta, delay);

        // sides
        Debug.DrawLine(p1, p5, Color.white, delay);
        Debug.DrawLine(p2, p6, Color.gray, delay);
        Debug.DrawLine(p3, p7, Color.green, delay);
        Debug.DrawLine(p4, p8, Color.cyan, delay);
    }

    private static (float minDistance, float3 nearestNormal) NearestNormalFinderGPT(GameObject targetGameObject, float3 point)
    {
        Mesh mesh = targetGameObject.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices; // Keep as Vector3 to use mesh data directly
        int[] triangles = mesh.triangles;
        int trianglesCount = triangles.Length / 3;

        float minDistance = float.MaxValue;
        float3 nearestNormal = float3.zero;

        Matrix4x4 localToWorldMatrix = targetGameObject.transform.localToWorldMatrix;

        for (int i = 0; i < trianglesCount; i++)
        {
            // Directly convert Vector3 to float3 using the float3 constructor
            float3 vertexA = localToWorldMatrix.MultiplyPoint3x4(vertices[triangles[i * 3]]);
            float3 vertexB = localToWorldMatrix.MultiplyPoint3x4(vertices[triangles[i * 3 + 1]]);
            float3 vertexC = localToWorldMatrix.MultiplyPoint3x4(vertices[triangles[i * 3 + 2]]);

            float3 closestPoint = ClosestPointOnTriangleToPoint(point, vertexA, vertexB, vertexC); // Assume this function is adapted to use float3
            float distanceSquared = math.distancesq(point, closestPoint); // Correctly calculate squared distance using math.distancesq

            if (distanceSquared < minDistance)
            {
                minDistance = distanceSquared;
                nearestNormal = math.normalize(math.cross(vertexB - vertexA, vertexC - vertexA)); // Use math.cross and math.normalize for float3
            }
        }

        return (math.sqrt(minDistance), nearestNormal);
    }

    private static (float minDistance, Vector3 nearestNormal) NearestNormalFinder(GameObject targetGameObject, Vector3 point)
    {
        // Get mesh and attributes from target GameObject
        Mesh mesh = targetGameObject.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3[] normal = mesh.normals;
        int[] triangles = mesh.triangles;
        int trianglesCount = triangles.Length / 3;

        // Initialize variables to store the minimum distance and nearest normal
        float minDistance = float.MaxValue;
        Vector3 minClosestPoint = Vector3.zero;
        Vector3 nearestNormal = Vector3.zero;

        // Array to hold the vertices transformed into world space
        Vector3[] transformedVertices = new Vector3[vertices.Length];

        // Transform the local-space vertex coordinates to world-space
        for (int i = 0; i < vertices.Length; i++)
        {
            transformedVertices[i] = targetGameObject.transform.TransformPoint(vertices[i]);
        }

        // Object to lock shared resource while updating minDistance and nearestNormal in a multi-threaded context
        object lockObject = new object();

        // Parallelized loop to go through each triangle in the mesh
        Parallel.For(0, trianglesCount, i =>
        {
            // Extract vertices of the i-th triangle
            Vector3 vertexA = transformedVertices[triangles[i * 3]];
            Vector3 vertexB = transformedVertices[triangles[i * 3 + 1]];
            Vector3 vertexC = transformedVertices[triangles[i * 3 + 2]];

            if (vertexA.y + vertexB.y + vertexC.y != 0f)
            {
                // Compute closest point on the i-th triangle to the target point
                Vector3 closestPoint = ClosestPointOnTriangleToPoint(point, vertexA, vertexB, vertexC);

                // Compute the squared distance from the closest point to the target point
                float distanceSquared = (point - closestPoint).magnitude;

                // Check if this distance is less than the currently stored minimum distance
                if (distanceSquared < minDistance)
                {
                    lock (lockObject) // Lock to safely update shared resources
                    {
                        // Double-check the condition after acquiring the lock
                        if (distanceSquared < minDistance)
                        {
                            minClosestPoint = closestPoint;
                            minDistance = distanceSquared;

                            // Compute the normal of the triangle and normalize it
                            nearestNormal = Vector3.Cross(vertexB - vertexA, vertexC - vertexA).normalized;
                        }
                    }
                }
            }
        });

        return (minDistance, nearestNormal);
    }

    // This function computes the point on the triangle ABC that is closest to point P
    private static Vector3 ClosestPointOnTriangleToPoint(Vector3 P, Vector3 A, Vector3 B, Vector3 C)
    {
        // Compute normal vector of the triangle
        Vector3 normal = Vector3.Cross(B - A, C - A).normalized;

        // Project P onto the plane of the triangle
        Vector3 projectedP = P - Vector3.Dot(P - A, normal) * normal;

        // Check if projectedP lies inside the triangle. If so, it's the closest point
        if (IsPointInTriangle(projectedP, A, B, C))
        {
            return projectedP;
        }

        // If projected P is outside the triangle, then the closest point is on one of the triangle's edges
        Vector3 edge1Closest = ClosestPointOnLineSegmentToPoint(A, B, projectedP);
        Vector3 edge2Closest = ClosestPointOnLineSegmentToPoint(B, C, projectedP);
        Vector3 edge3Closest = ClosestPointOnLineSegmentToPoint(C, A, projectedP);

        float d1 = Vector3.Distance(edge1Closest, projectedP);
        float d2 = Vector3.Distance(edge2Closest, projectedP);
        float d3 = Vector3.Distance(edge3Closest, projectedP);

        if (d1 < d2 && d1 < d3)
        {
            return edge1Closest;
        }
        else if (d2 < d1 && d2 < d3)
        {
            return edge2Closest;
        }
        else
        {
            return edge3Closest;
        }
    }

    // Check if point P is inside the triangle ABC
    private static bool IsPointInTriangle(Vector3 P, Vector3 A, Vector3 B, Vector3 C)
    {
        Vector3 u = B - A;
        Vector3 v = C - A;
        Vector3 w = P - A;

        Vector3 vCrossW = Vector3.Cross(v, w);
        Vector3 vCrossU = Vector3.Cross(v, u);

        if (Vector3.Dot(vCrossW, vCrossU) < 0)
        {
            return false;
        }

        Vector3 uCrossW = Vector3.Cross(u, w);
        Vector3 uCrossV = Vector3.Cross(u, v);

        if (Vector3.Dot(uCrossW, uCrossV) < 0)
        {
            return false;
        }

        float denominator = uCrossV.magnitude;
        float r = vCrossW.magnitude / denominator;
        float t = uCrossW.magnitude / denominator;

        return (r <= 1 && t <= 1 && r + t <= 1);
    }

    private static Vector3 ClosestPointOnLineSegmentToPoint(Vector3 A, Vector3 B, Vector3 P)
    {
        // Vector from A to B
        Vector3 AB = B - A;

        // Vector from A to P
        Vector3 AP = P - A;

        // The parameter t along the line AB that represents the point that lies closest to P
        float t = Vector3.Dot(AP, AB) / Vector3.Dot(AB, AB);

        // If t is within [0,1], the point lies within the line segment AB
        if (t < 0f)
        {
            return A;
        }
        if (t > 1f)
        {
            return B;
        }

        // Calculate the point that is closest to P on the line segment AB
        Vector3 ClosestPoint = A + t * AB;

        return ClosestPoint;
    }

    private static (Vector3, Vector3, float) GetSpawningArea(GameObject spawningPlane)
    {
        Transform planeTransform = spawningPlane.GetComponent<Transform>();
        Renderer planeRenderer = spawningPlane.GetComponent<Renderer>();

        Bounds bounds = planeRenderer.bounds;
        Vector3 size = bounds.size;

        float area = size.x * size.z;
        Vector3 _center = planeTransform.position;
        Vector3 _size = size;

        return (_center, _size, area);
    }

    [BurstCompile]
    private struct PlaneMpcSpawner : IJobParallelFor
    {
        public Vector3 Center;
        public Vector3 Size;
        public float MaxHight;

        [NativeDisableContainerSafetyRestriction]
        public NativeArray<Unity.Mathematics.Random> Rngs;

        [NativeSetThreadIndex] readonly int _threadId;

        [WriteOnly] public NativeArray<Vector3> Array;

        public void Execute(int index)
        {
            var rng = Rngs[_threadId];

            Array[index] = new Vector3(
                rng.NextFloat(Center.x - Size.x / 2f, Center.x + Size.x / 2f),
                Center.y + rng.NextFloat(0f, MaxHight),
                rng.NextFloat(Center.z - Size.z / 2f, Center.z + Size.z / 2f)
            );

            Rngs[_threadId] = rng;
        }
    }

    [BurstCompile]
    private struct ParallelRayCastingData : IJobParallelFor
    {
        [Unity.Collections.ReadOnly] public NativeArray<Vector3> MPC_Array;
        [Unity.Collections.ReadOnly] public NativeArray<Vector3> OBS_Array;

        [WriteOnly] public NativeArray<RaycastCommand> commands;
        public void Execute(int index)
        {
            int i_obs = Mathf.FloorToInt(index / MPC_Array.Length);
            int i_mpc = index - i_obs * MPC_Array.Length;

            Vector3 temp_direction = MPC_Array[i_mpc] - OBS_Array[i_obs];
            commands[index] = new RaycastCommand(OBS_Array[i_obs], temp_direction.normalized, QueryParameters.Default, temp_direction.magnitude);
        }
    }
}
