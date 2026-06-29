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
using System.Diagnostics;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Entities;
using Debug = UnityEngine.Debug;
using Complex = System.Numerics.Complex;

public struct LookUpTable
{
    public NativeArray<float> Distances;
    public NativeArray<int> StartIndecies;

    public void Dispose()
    {
        this.Distances.Dispose();
        this.StartIndecies.Dispose();
    }
}

public class LookUpTableGenrator
{
    public NativeArray<float> CalculateDistanceMatrix(NativeList<MPC.ScatterPoint> points, float distance)
    {
        float t_start = Time.realtimeSinceStartup;

        int numPoints = points.Length;
        NativeArray<float> flatDistanceMatrix = new NativeArray<float>(numPoints * numPoints, Allocator.Persistent);

        // Total number of unique pairs
        int numUniquePairs = numPoints * (numPoints - 1) / 2;
        NativeArray<Vector2Int> mpcPairs = new NativeArray<Vector2Int>(numUniquePairs, Allocator.TempJob);
        int pairsIndex = 0;
        for (int i = 0; i < numPoints; i++)
        {
            for (int j = i + 1; j < numPoints; j++)
            {
                mpcPairs[pairsIndex] = new Vector2Int(i, j);
                pairsIndex += 1;
            }
        }

        // Prepare raycast commands
        NativeArray<RaycastCommand> raycastCommands = new NativeArray<RaycastCommand>(numUniquePairs, Allocator.TempJob);
        
        // Create and schedule the job
        PrepareForRaycastJob prepareJob = new PrepareForRaycastJob
        {
            Points = points,
            MpcPairs = mpcPairs,
            RaycastCommands = raycastCommands   
        };
        
        JobHandle prepareHandle = prepareJob.Schedule(numUniquePairs, 64);
        prepareHandle.Complete();
        
        
        // Perform the raycasting
        NativeArray<RaycastHit> raycastHits = new NativeArray<RaycastHit>(raycastCommands.Length, Allocator.TempJob);
        JobHandle raycastHandle = RaycastCommand.ScheduleBatch(raycastCommands, raycastHits, 64, prepareHandle);
        raycastHandle.Complete();

        float t_rc = Time.realtimeSinceStartup;

        // Initialize the flat distance matrix with -1f (or any invalid distance value)
        for (int i = 0; i < flatDistanceMatrix.Length; i++)
        {
            flatDistanceMatrix[i] = -1f;
        }

        
        // Populate the distance matrix
        for (int index = 0; index < numUniquePairs; index++)
        {
            
            int i = mpcPairs[index].x;
            int j = mpcPairs[index].y;
            
            
            RaycastHit hit = raycastHits[index];
            
            if (hit.collider == null)
            {
                Vector3 inorm = points[i].Normal;
                Vector3 jnorm = points[j].Normal;
                
                Vector3 iMpc = points[i].Point;
                Vector3 jMpc = points[j].Point;
                Vector3 ijDirection = jMpc - iMpc;
                float iDirection = Vector3.Dot(inorm, ijDirection.normalized);
                float jDirection = Vector3.Dot(jnorm, -ijDirection.normalized);
               
                if (iDirection > 0 && jDirection > 0 && Vector3.Dot(inorm, jnorm) < 0f && raycastCommands[index].distance <= distance)
                    {
                        flatDistanceMatrix[i * numPoints + j] = raycastCommands[index].distance;
                        flatDistanceMatrix[j * numPoints + i] = raycastCommands[index].distance; // Since it's symmetric
                    }
            }
        }
        
        raycastCommands.Dispose();
        raycastHits.Dispose();
        mpcPairs.Dispose();

        //Debug.Log("LUC generation for " + numPoints + " MPCs took: " + ((Time.realtimeSinceStartup - t_start) * 1000f) +
                  //" ms, where ray casting took: " + ((t_rc - t_start) * 1000f) + " ms");

        return flatDistanceMatrix;
    }
    
    [BurstCompile]
    [System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential)]
    public struct PrepareForRaycastJob : IJobParallelFor
    {
        [ReadOnly] public NativeList<MPC.ScatterPoint> Points;
        [ReadOnly] public NativeArray<Vector2Int> MpcPairs;
        [WriteOnly] public NativeArray<RaycastCommand> RaycastCommands;
        
        public void Execute(int index)
        {
            int numPoints = Points.Length;

            int i = numPoints - 2 - Mathf.FloorToInt(Mathf.Sqrt(-8 * index + 4 * numPoints * (numPoints - 1) - 7) / 2.0f - 0.5f);
            int j= index + i + 1 - numPoints * (numPoints - 1) / 2 + (numPoints - i) * ((numPoints - i) - 1) / 2;
            
            Vector3 origin = Points[i].Point;
            Vector3 target = Points[j].Point;
            Vector3 direction = target - origin;
            float distance = direction.magnitude;
            
            RaycastCommands[index] = new RaycastCommand(origin, direction.normalized, QueryParameters.Default, distance);
        }
    }
}