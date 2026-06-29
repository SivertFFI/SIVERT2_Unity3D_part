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
using System.Diagnostics;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using UnityEngine.Serialization;



public class MPCFiltering
{
    [BurstCompile]
    public struct ParallelVisbilityDetection : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Vector3> Entities;
        [ReadOnly] public NativeArray<MPC.ScatterPoint> MPCs;

        [WriteOnly] public NativeArray<RaycastCommand> RayCastcommands;
        
        public void Execute(int index)
        {
            int entityIndex = Mathf.FloorToInt(index / MPCs.Length);
            int pointIndex = index % MPCs.Length;

            Vector3 enitityPosition = Entities[entityIndex];
            Vector3 mpcPosition = MPCs[pointIndex].Point;

            Vector3 direction = mpcPosition - enitityPosition;
            RayCastcommands[index] = new RaycastCommand(enitityPosition, direction.normalized, QueryParameters.Default, direction.magnitude);
        }
    }

    [BurstCompile]
    public struct FindMPCsWithinRadiusParallelJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RaycastHit> RaycastHits;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommands;
        [ReadOnly] public float Radius;
        
        [WriteOnly] public NativeArray<int> PointMarkers; // 1 if within radius, 0 otherwise
        
        public void Execute(int index)
        {
            if (RaycastHits[index].distance == 0 && RaycastCommands[index].distance <= Radius)
            {
                PointMarkers[index] = 1;
            }
            else
            {
                PointMarkers[index] = 0;
            }
        }
    }

    public NativeArray<NativeList<int>> CreateEntityToMPCMapping(NativeArray<int> pointMarkers, int numEntities,
        int numPoints)
    {
        NativeArray<NativeList<int>> mapping = new NativeArray<NativeList<int>>(numEntities, Allocator.Temp);

        for (int i = 0; i < numEntities; i++)
        {
            NativeList<int> pointList = new NativeList<int>(Allocator.Temp);

            for (int j = 0; j < numPoints; j++)
            {
                if (pointMarkers[i * numPoints + j] == 1)
                {
                    pointList.Add(j);
                }
            }

            mapping[i] = pointList;
        }

        return mapping;
    }

    public struct FlattenedResult
    {
        public NativeArray<int> MpcIndices;
        public NativeArray<int> StartIndicesForObjects;
        
        public void Dispose()
        {
            MpcIndices.Dispose();
            StartIndicesForObjects.Dispose();
        }
    }

    public FlattenedResult FindAllVisibleMpCsWithinRadius(NativeArray<Vector3> entities,
        NativeArray<MPC.ScatterPoint> pointsWithNormal, float radius)
    {
        int totalLength = entities.Length * pointsWithNormal.Length;
        NativeArray<int> pointMarkers = new NativeArray<int>(totalLength, Allocator.TempJob);

        NativeArray<RaycastCommand> raycastCommandsVisibility =
            new NativeArray<RaycastCommand>(totalLength, Allocator.TempJob);
        NativeArray<RaycastHit> raycastHitVisibility = new NativeArray<RaycastHit>(totalLength, Allocator.TempJob);

        ParallelVisbilityDetection visibilityDetection = new ParallelVisbilityDetection
        {
            Entities = entities,
            MPCs = pointsWithNormal,

            RayCastcommands = raycastCommandsVisibility,
        };

        // Create all commands in parallel
        JobHandle loSDetectionHandle = visibilityDetection.Schedule(totalLength, 64);

        // Run ray-cast commands
        JobHandle raycastJobVisibility = RaycastCommand.ScheduleBatch(raycastCommandsVisibility, raycastHitVisibility, 64, loSDetectionHandle);
        raycastJobVisibility.Complete();

        
        FindMPCsWithinRadiusParallelJob hitAndRadiusJob = new FindMPCsWithinRadiusParallelJob
        {
            RaycastHits = raycastHitVisibility,
            RaycastCommands = raycastCommandsVisibility,
            PointMarkers = pointMarkers,
            Radius = radius,
        };
        
        JobHandle handle = hitAndRadiusJob.Schedule(totalLength, 64);
        handle.Complete();
        
        int totalCount = 0;
        for (int i = 0; i < totalLength; i++)
        {
            if (raycastHitVisibility[i].collider is null && (raycastCommandsVisibility[i].distance <= radius))
            {
                pointMarkers[i] = 1;
                totalCount++;
            }
            else
            {
                pointMarkers[i] = 0;
            }
        }

        NativeArray<int> flattenedArray = new NativeArray<int>(totalCount, Allocator.TempJob);
        NativeArray<int> startIndices = new NativeArray<int>(entities.Length, Allocator.TempJob);

        int flattenedIndex = 0;

        for (int i = 0; i < entities.Length; i++)
        {
            startIndices[i] = flattenedIndex;

            for (int j = i * pointsWithNormal.Length; j < (i + 1) * pointsWithNormal.Length; j++)
            {
                if (pointMarkers[j] == 1)
                {
                    flattenedArray[flattenedIndex++] = j % pointsWithNormal.Length;
                }
            }
        }

        // Dispose of the temporary NativeArray used for point markers.
        pointMarkers.Dispose();
        raycastHitVisibility.Dispose();
        raycastCommandsVisibility.Dispose();
        
        
        return new FlattenedResult
        {
            MpcIndices = flattenedArray,
            StartIndicesForObjects = startIndices
        };
        
    }

    #region Check if above square
    public struct AboveSquareCheckJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Vector3> ObjectCoordinates;
        [ReadOnly] public NativeArray<MPC.ScatterPoint> Mpcs; // Input positions
        [ReadOnly] public float Distance;
            
        public NativeArray<bool> Results; // Output results
        
        public void Execute(int i)
        {
            int objectIdx = Mathf.FloorToInt(i/Mpcs.Length);
            int mpcIdx = i % Mpcs.Length;

            float minX = ObjectCoordinates[objectIdx].x - Distance / 2.0f;
            float maxX = ObjectCoordinates[objectIdx].x + Distance / 2.0f;
            float minZ = ObjectCoordinates[objectIdx].z - Distance / 2.0f;
            float maxZ = ObjectCoordinates[objectIdx].z + Distance / 2.0f;
                
            // Check if the position is within the bounds of the square
            Results[i] = Mpcs[i].Point.x > minX &&
                         Mpcs[i].Point.x < maxX &&
                         Mpcs[i].Point.z > minZ &&
                         Mpcs[i].Point.z < maxZ;
        }
    }

    public FlattenedResult AboveSquareFilter(NativeArray<Vector3> objectCoordinates, NativeArray<MPC.ScatterPoint> mpcs, float distance)
    {
        NativeArray<bool> resultParallel = new NativeArray<bool>(objectCoordinates.Length * mpcs.Length, Allocator.TempJob);

        AboveSquareCheckJob aboveSquareCheckJob = new AboveSquareCheckJob
        {
            ObjectCoordinates = objectCoordinates,
            Mpcs = mpcs,
            Distance = distance,
            Results = resultParallel
        };
        
        JobHandle handle = aboveSquareCheckJob.Schedule(objectCoordinates.Length * mpcs.Length, 64);
        handle.Complete();

        NativeList<int> mpcIndices = new NativeList<int>(Allocator.TempJob);
        NativeList<int> startIndicesForObjects = new NativeList<int>(Allocator.TempJob);

        int prevObjectIdx = -1;
        int objectIdx = -1;
        int mpcIdx = -1;
        int idx = 0;
        
        for (int i = 0; i < resultParallel.Length; i++)
        {
            if (resultParallel[i])
            {
                objectIdx = Mathf.FloorToInt(i/mpcs.Length);
                mpcIdx = i % mpcs.Length;
                
                mpcIndices.Add(mpcIdx);
                
                if (prevObjectIdx != objectIdx)
                {
                    startIndicesForObjects.Add(idx);
                }
                
                prevObjectIdx = objectIdx;
                idx += 1;
            }
        }

        resultParallel.Dispose();
        
        return new FlattenedResult
        {
            MpcIndices = mpcIndices.ToArray(Allocator.TempJob), 
            StartIndicesForObjects = startIndicesForObjects.ToArray(Allocator.TempJob)
        };
    }

    #endregion
    
}

