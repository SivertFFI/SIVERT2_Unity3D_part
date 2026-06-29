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
using System.Diagnostics;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Debug = UnityEngine.Debug;
using Complex = System.Numerics.Complex;
using System.Runtime.InteropServices;


public struct LUT2
{
    public int InMpcId;
    public int OutMpcId;
    public float Distance;

    public LUT2(int i, int j, float d)
    {
        this.InMpcId = i;
        this.OutMpcId = j;
        this.Distance = d;
    }

}

public struct MPCFloat
{
    public float3 Point;
    public float3 Normal;
    public float Attenuation;
    public MPCFloat(float3 point, float3 normal, float attenuation)
    {
        this.Point = point;
        this.Normal = normal;
        this.Attenuation = attenuation;
    }
}

//Nested structs cannot be used in BurstCompile
public struct Lut2Dictionary
{
    public int InMpcId;
    public int OutMpcId;
    public float Distance;
    // it's better to use float3 for high-performing calculation
    public float3 ImageOfNextMpc;
    public float3 ImageOfThisMpc;
    public float Attenuation;
    
    // Constructor for initialization
    public Lut2Dictionary(int i, int j, float d, float3 imageOfNextMpc, float3 imageOfThisMpc, float attenuation)
    {
        this.InMpcId = i;
        this.OutMpcId = j;
        this.Distance = d;
        this.ImageOfNextMpc = imageOfNextMpc;
        this.ImageOfThisMpc = imageOfThisMpc;
        this.Attenuation = attenuation;
    }
    
}


public struct LUT3
{
    public int InMpcId;
    public int MidMpcId;
    public int OutMpcId;
    
    public float Distance;
    
    // Path3 : InMpc -> MidMpc -> OutMpc; we need to track the image of MidMpc in relation to InMpc and OutMpc
    public float3 ImOfMidMpcIn;
    public float3 ImOfMidMpcOut;

    public float Attenuation;

    public LUT3(int i, int j, int k, float d, float3 imOfMidMpcIn, float3 imOfMidMpcOut, float attenuation)
    {
        this.InMpcId = i;
        this.MidMpcId = j;
        this.OutMpcId = k;
        
        this.Distance = d;

        this.ImOfMidMpcIn = imOfMidMpcIn;
        this.ImOfMidMpcOut = imOfMidMpcOut;

        this.Attenuation = attenuation;
    }

}

public struct LUT2Comparer : IComparer<LUT2>
{
    public int Compare(LUT2 x, LUT2 y)
    {
        return x.InMpcId.CompareTo(y.InMpcId);
    }
}

public struct LUT3Comparer : IComparer<LUT3>
{
    public int Compare(LUT3 x, LUT3 y)
    {
        return x.InMpcId.CompareTo(y.InMpcId);
    }
}

public class LUTDictionary
{
    public (int, NativeArray<Vector2Int>, NativeArray<Lut2Dictionary>) CalculateLUT2(NativeArray<MPCFloat> mpcs, float distance, float attenuationThreshold, float lambdaOvert4PI)
    {
        
        NativeList<LUT2> lut = new NativeList<LUT2>(Allocator.TempJob);
        float visibilityFraction = 0.5f; // we assume that at most visibilityFraction of the mpcs are visible from an mpc
        int estimatedCapacity = (int)(mpcs.Length * mpcs.Length * visibilityFraction);
        lut.Capacity = estimatedCapacity;
        
        // raycast commands preparation under conditions
        NativeArray<RaycastCommand> raycastCommands =
            new NativeArray<RaycastCommand>(mpcs.Length * mpcs.Length, Allocator.TempJob);
        LutRaycasting lutRaycast = new LutRaycasting
        {
            Mpcs = mpcs,
            MaxDistance = distance/2.0f,
            RaycastCommands = raycastCommands
        };
        JobHandle handleInRaycast = lutRaycast.Schedule(mpcs.Length * mpcs.Length, 64);
        handleInRaycast.Complete();
        
        NativeArray<RaycastHit> raycastHit = new NativeArray<RaycastHit>(raycastCommands.Length, Allocator.TempJob);
        JobHandle raycastJob = RaycastCommand.ScheduleBatch(raycastCommands, raycastHit, 64); // Batch size adjusted for efficiency
        raycastJob.Complete();
        
        // storing active raycasting results 
        Lut2Generation lut2Generation = new Lut2Generation()
        {
            MpcLength = mpcs.Length,
            RaycastCommands = raycastCommands,
            RaycastResults = raycastHit,
            MpcFloats = mpcs,
            LambdaOver4PI = lambdaOvert4PI,
            AttenuationThreshold = attenuationThreshold,
            Lut = lut.AsParallelWriter()
        };
        JobHandle handleLut2Generation = lut2Generation.Schedule(raycastHit.Length, 64, raycastJob);
        handleLut2Generation.Complete();
        
        lut.Sort(new LUT2Comparer());




        #region mpcPointers sorting
        
        NativeArray<Vector2Int> mpcPointers = new NativeArray<Vector2Int>(mpcs.Length, Allocator.Persistent);

        int currentStartIndex = -1;
        int currentMpcId = -1;

        int maxVisibilityNumber = 0;
        for (int i = 0; i < lut.Length; i++)
        {
            int mpcId = lut[i].InMpcId;

            // Check if this is a new MPC ID.
            if (mpcId != currentMpcId)
            {
                // Update the pointers array for the previous MPC ID.
                if (currentMpcId >= 0 && currentMpcId < mpcPointers.Length)
                {
                    mpcPointers[currentMpcId] = new Vector2Int(currentStartIndex, i - 1);
                }

                if (maxVisibilityNumber < i - currentStartIndex)
                {
                    maxVisibilityNumber = i - currentStartIndex;
                }
                currentMpcId = mpcId;
                currentStartIndex = i;
            }
        }

        // Handle the last element outside the loop.
        if (currentMpcId >= 0 && currentMpcId < mpcPointers.Length)
        {
            mpcPointers[currentMpcId] = new Vector2Int(currentStartIndex, lut.Length - 1);
        }

        if (maxVisibilityNumber < lut.Length - currentStartIndex)
        {
            maxVisibilityNumber = lut.Length - currentStartIndex;
        }
        #endregion
        
        /*
        // Evaluation draw
        for (int i = 0; i < mpcPointers.Length; i++)
        {
            if (mpcPointers[i].y - mpcPointers[i].x == maxVisibilityNumber - 1)
            {
                Debug.Log("-----------------------------");
                Debug.Log("Max number of visible MPCs = " + maxVisibilityNumber + "; come from MPC[" + i + "];");
                Debug.Log("-----------------------------");
                for (int j = 0; j < maxVisibilityNumber; j++)
                {
                    int tempId = lut[mpcPointers[i].x + j].OutMpcId;
                    Debug.DrawLine(mpcs[i].Point, mpcs[tempId].Point, Color.yellow, 5f);
                }
            }
        }

        for (int i = 0; i < mpcPointers2.Length; i++)
        {
            if (mpcPointers2[i].x > 0 && mpcPointers2[i].y - mpcPointers2[i].x > -1 && mpcPointers2[i].y - mpcPointers2[i].x < 5)
            {
                for (int j = 0; j < mpcPointers2[i].y - mpcPointers2[i].x + 1; j++)
                {
                    int inTestId = lut[mpcPointers2[i].x].InMpcId;
                    int outTestId = lut[mpcPointers2[i].x + j].OutMpcId;
                    Debug.DrawLine(mpcs[inTestId].Point, mpcs[outTestId].Point, Color.white, 5f);
                }
            }
        }
        */
        
        NativeArray<Lut2Dictionary> lut2Dictionaries = new NativeArray<Lut2Dictionary>(lut.Length, Allocator.Persistent);
        Lut2ImagesCalculation lut2ImagesCalculation = new Lut2ImagesCalculation()
        {
            MpcsFloat3 = mpcs,
            Lut2 = lut.AsArray(),
            Lut2Dictionaries = lut2Dictionaries
        };
        JobHandle handleLut2ImagesCalculation = lut2ImagesCalculation.Schedule(lut.Length, 64, handleLut2Generation);
        handleLut2ImagesCalculation.Complete();

        #region Evaluation of the ImageCalculation for Lut2
        /*
        // visually evaluated the results.
        // Evaluation drawing
        int drawInId = lut[0].InMpcId;
        int drawOutId = lut[0].OutMpcId;
        int testInId = lut2Dictionaries[0].InMpcId;
        int testOutId = lut2Dictionaries[0].OutMpcId;

        Vector3 drawNormal = mpcs[drawOutId].Normal;
        Vector3 testNormal = lut2Dictionaries[0].ImageOfNextNormal;
        Vector3 directionFromMpc2ImageMpc = mpcs[drawOutId].Point - new Vector3(lut2Dictionaries[0].ImageOfNextMpc.x, lut2Dictionaries[0].ImageOfNextMpc.y, lut2Dictionaries[0].ImageOfNextMpc.z);
        Debug.Log("MPC Normal: "+ drawNormal + "; Image Normal: " + testNormal + "; Normalized direction: " + directionFromMpc2ImageMpc.normalized);

        Main.MPC c = ImageCalculation(mpcs[testInId], mpcs[testOutId]);
        
        Debug.DrawLine(mpcs[drawInId].Point, mpcs[drawOutId].Point, Color.white, 5f);
        Vector3 shiftVector = Vector3.up;
        Debug.DrawLine(mpcs[testInId].Point + shiftVector, mpcs[testOutId].Point + shiftVector, Color.yellow, 5f);
        Debug.DrawLine(mpcs[testInId].Point, lut2Dictionaries[0].ImageOfNextMpc, Color.magenta, 5f);
        Debug.DrawLine(lut2Dictionaries[0].ImageOfNextMpc, lut2Dictionaries[0].ImageOfNextMpc + 5f*lut2Dictionaries[0].ImageOfNextNormal,Color.green, 5f);
        */
        // visually evaluated the results.
        
        #endregion

        lut.Dispose();
        raycastCommands.Dispose();
        raycastHit.Dispose();
        
        return (maxVisibilityNumber, mpcPointers, lut2Dictionaries);
    }
    
    
    public (int, NativeArray<Vector2Int>, NativeArray<LUT3>) CalculateLUT3(NativeArray<MPCFloat> mpcs, NativeArray<Vector2Int> mpcPointers, NativeArray<Lut2Dictionary> lut3Half, float distance, float attenuationThreshold, float lambdaOvert4PI)
    {
        
        int maxVisibilityNumber = 0;
        for (int i = 0; i < mpcPointers.Length; i++)
        {
            if (maxVisibilityNumber < mpcPointers[i].y - mpcPointers[i].x)
            {
                maxVisibilityNumber = mpcPointers[i].y - mpcPointers[i].x;
            }
        }
        
        // finding the length of LUT3
        NativeList<bool> lut3Length = new NativeList<bool>(lut3Half.Length * maxVisibilityNumber, Allocator.TempJob);
        
        Lut3LengthCalculation lut3LengthCalculation = new Lut3LengthCalculation()
        {
            AngularSpread = 0.0873f, // already converted 5 degrees to radians
            MaxDistance = distance/3.0f,
            MpcArray = mpcs,
            Lut2Dictionaries = lut3Half,
            Mpc2Pointers = mpcPointers,
            LambdaOver4PI = lambdaOvert4PI,
            AttenuationThreshold = attenuationThreshold,
            Lut3Bool = lut3Length.AsParallelWriter()
        };
        JobHandle handleLut3LengthCalculation = lut3LengthCalculation.Schedule(lut3Half.Length, 64);
        handleLut3LengthCalculation.Complete();
        Debug.Log("Lut3 length = " + lut3Length.Length);
        
        // Once the length is found, we can safely allocate NativeList capacity 
        NativeList<LUT3> lut3 = new NativeList<LUT3>(lut3Length.Length, Allocator.Persistent);
        
            Lut3Calculation lut3Calculation = new Lut3Calculation()
            {
                AngularSpread = 0.0873f, // already converted 5 degrees to radians
                MaxDistance = distance/3.0f,
                MpcArray = mpcs,
                Lut2Dictionaries = lut3Half,
                Mpc2Pointers = mpcPointers,
                LambdaOver4PI = lambdaOvert4PI,
                AttenuationThreshold = attenuationThreshold,
                Lut3 = lut3.AsParallelWriter()
            };
            JobHandle handleLut3Calculation = lut3Calculation.Schedule(lut3Half.Length, 64, handleLut3LengthCalculation);
            handleLut3Calculation.Complete();
            Debug.Log("Lut3 has " + lut3.Length + "; elements");
        
            lut3Length.Dispose();
        
            lut3.Sort(new LUT3Comparer());
        
            #region mpc3Pointers sorting
        
            NativeArray<Vector2Int> mpc3Pointers = new NativeArray<Vector2Int>(mpcs.Length, Allocator.Persistent);

            int currentStartIndex = -1;
            int currentMpcId = -1;
        
            for (int i = 0; i < lut3.Length; i++)
            {
                int mpcId = lut3[i].InMpcId;

                // Check if this is a new MPC ID.
                if (mpcId != currentMpcId)
                {
                    // Update the pointers array for the previous MPC ID.
                    if (currentMpcId >= 0 && currentMpcId < mpc3Pointers.Length)
                    {
                        mpc3Pointers[currentMpcId] = new Vector2Int(currentStartIndex, i - 1);
                    }
                
                    currentMpcId = mpcId;
                    currentStartIndex = i;
                }
            }

            // Handle the last element outside the loop.
            if (currentMpcId >= 0 && currentMpcId < mpc3Pointers.Length)
            {
                mpc3Pointers[currentMpcId] = new Vector2Int(currentStartIndex, lut3.Length - 1);
            }

            #endregion


        
            return (maxVisibilityNumber, mpc3Pointers, lut3.AsArray());
        
        
    }
    
    
    [BurstCompile]
    struct Lut3Calculation : IJobParallelFor
    {
        [ReadOnly] public float AngularSpread;
        [ReadOnly] public float MaxDistance;
        [ReadOnly] public NativeArray<MPCFloat> MpcArray;
        [ReadOnly] public NativeArray<Lut2Dictionary> Lut2Dictionaries;
        [ReadOnly] public NativeArray<Vector2Int> Mpc2Pointers;
        [ReadOnly] public float LambdaOver4PI;
        [ReadOnly] public float AttenuationThreshold;

        [WriteOnly] public NativeList<LUT3>.ParallelWriter Lut3;
        
        public void Execute(int index)
        {
            // index runs through all 300000 elements of Lut2Dictionaries
            // for each triplet of Mpcs, we need to check if they satisfy Angular Spread conditions:
            
            // define entry Mpc ID (inMpcId) and middle Mpc ID ad (midMpcId)
            int inMpcId = Lut2Dictionaries[index].InMpcId;
            int midMpcId = Lut2Dictionaries[index].OutMpcId;
            float d1 = Lut2Dictionaries[index].Distance;
            
            // to calculate angles, we need to have Mpcs coordinates
            float3 inMpc = MpcArray[inMpcId].Point;
            float3 midMpc = MpcArray[midMpcId].Point;
            float3 inMpc2MidMpc = math.normalize(midMpc - inMpc); 
            // midMpcId goes to the position of Mpc in Mpc2Pointers,
            // which defines ranges where to search in Lut2Dictionaries
            Vector2Int range = Mpc2Pointers[midMpcId];
            for (int i = range.x; i <= range.y; i++)
            {
                int outMpcId = Lut2Dictionaries[i].OutMpcId;
                // we already calculated Image of MpcArray[outMpcId].Point about midMpc
                // Hence, we access it from Lut2Dictionaries[i] as:
                float3 imageOfOutMpc = Lut2Dictionaries[i].ImageOfNextMpc;
                float3 inMpc2ImageOfOutMpc = math.normalize(imageOfOutMpc - inMpc);
                // Now, we need to calculate angles between (inMpc, midMpc) and (inMpc, imageOfOutMpc)
                float alpha = math.acos(math.dot(inMpc2MidMpc, inMpc2ImageOfOutMpc));
                
                // Calculating beta
                float3 imageOfOutMpc2MidMpc = math.normalize(midMpc - imageOfOutMpc);
                float beta = math.acos(math.dot(imageOfOutMpc2MidMpc, -inMpc2ImageOfOutMpc));

                if (alpha < AngularSpread && beta < AngularSpread)
                {
                    float d = d1 + Lut2Dictionaries[i].Distance;
                    float avgAttenuationCoefficient = (MpcArray[inMpcId].Attenuation + MpcArray[midMpcId].Attenuation +
                                                       MpcArray[outMpcId].Attenuation)/3f;
                    float avgAttenuation = avgAttenuationCoefficient * LambdaOver4PI / d;
                    if (d < MaxDistance && avgAttenuation > AttenuationThreshold)
                    {
                        MPCFloat imOfMidMpcIn = ImageCalculation(MpcArray[inMpcId], MpcArray[midMpcId]);
                        MPCFloat imOfMidMpcOut = ImageCalculation(MpcArray[outMpcId], MpcArray[midMpcId]);
                        LUT3 tempLut3 = new LUT3(inMpcId, midMpcId, outMpcId, d, imOfMidMpcIn.Point,
                            imOfMidMpcOut.Point, avgAttenuationCoefficient);
                        Lut3.AddNoResize(tempLut3);
                    }
                }

            }
        }
    }
    
    
    [BurstCompile]
    struct Lut3LengthCalculation : IJobParallelFor
    {
        [ReadOnly] public float AngularSpread;
        [ReadOnly] public float MaxDistance;
        [ReadOnly] public NativeArray<MPCFloat> MpcArray;
        [ReadOnly] public NativeArray<Lut2Dictionary> Lut2Dictionaries;
        [ReadOnly] public NativeArray<Vector2Int> Mpc2Pointers;
        [ReadOnly] public float LambdaOver4PI;
        [ReadOnly] public float AttenuationThreshold;

        [WriteOnly] public NativeList<bool>.ParallelWriter Lut3Bool;
        public void Execute(int index)
        {
            // index runs through all 300000 elements of Lut2Dictionaries
            // for each triplet of Mpcs, we need to check if they satisfy Angular Spread conditions:
            
            // define entry Mpc ID (inMpcId) and middle Mpc ID ad (midMpcId)
            int inMpcId = Lut2Dictionaries[index].InMpcId;
            int midMpcId = Lut2Dictionaries[index].OutMpcId;
            float d1 = Lut2Dictionaries[index].Distance;
            
            // to calculate angles, we need to have Mpcs coordinates
            float3 inMpc = MpcArray[inMpcId].Point;
            float3 midMpc = MpcArray[midMpcId].Point;
            float3 inMpc2MidMpc = math.normalize(midMpc - inMpc); 
            // midMpcId goes to the position of Mpc in Mpc2Pointers,
            // which defines ranges where to search in Lut2Dictionaries
            Vector2Int range = Mpc2Pointers[midMpcId];
            for (int i = range.x; i <= range.y; i++)
            {
                int outMpcId = Lut2Dictionaries[i].OutMpcId;
                // we already calculated Image of MpcArray[outMpcId].Point about midMpc
                // Hence, we access it from Lut2Dictionaries[i] as:
                float3 imageOfOutMpc = Lut2Dictionaries[i].ImageOfNextMpc;
                float3 inMpc2ImageOfOutMpc = math.normalize(imageOfOutMpc - inMpc);
                // Now, we need to calculate angles between (inMpc, midMpc) and (inMpc, imageOfOutMpc)
                float alpha = math.acos(math.dot(inMpc2MidMpc, inMpc2ImageOfOutMpc));
                
                // Calculating beta
                float3 imageOfOutMpc2MidMpc = math.normalize(midMpc - imageOfOutMpc);
                float beta = math.acos(math.dot(imageOfOutMpc2MidMpc, -inMpc2ImageOfOutMpc));

                if (alpha < AngularSpread && beta < AngularSpread)
                {
                    float d = d1 + Lut2Dictionaries[i].Distance;
                    float avgAttenuationCoefficient = (MpcArray[inMpcId].Attenuation + MpcArray[midMpcId].Attenuation +
                                                       MpcArray[outMpcId].Attenuation)/3f;
                    float avgAttenuation = avgAttenuationCoefficient * LambdaOver4PI / d;
                    if (d < MaxDistance && avgAttenuation > AttenuationThreshold)
                    {
                        MPCFloat imOfMidMpcIn = ImageCalculation(MpcArray[inMpcId], MpcArray[midMpcId]);
                        MPCFloat imOfMidMpcOut = ImageCalculation(MpcArray[outMpcId], MpcArray[midMpcId]);
                        LUT3 tempLut3 = new LUT3(inMpcId, midMpcId, outMpcId, d, imOfMidMpcIn.Point,
                            imOfMidMpcOut.Point, avgAttenuationCoefficient);
                        Lut3Bool.AddNoResize(true);
                    }
                }

            }
        }
    }
    
    public static MPCFloat ImageCalculation(MPCFloat a, MPCFloat b)
    {
        float3 n1 = a.Normal, p1 = a.Point;
        float3 n2 = b.Normal, p2 = b.Point;
        // Calculating image of the next Mpc and its Normal
        float3 direction = p2 - p1;
        float distancePlane = math.dot(n1, direction);
        float3 imageOutPoint = p2 - 2f * distancePlane * n1;
        float3 imageOutNormal = n2 - 2f * math.dot(n1, n2) * n1;
        MPCFloat c = new MPCFloat(imageOutPoint, imageOutNormal, b.Attenuation);
        return c;
    }

    #region Nested structs in BurstCompile

    // Calculation for float3
    [BurstCompile]
    struct Lut2ImagesCalculation : IJobParallelFor
    {
        [ReadOnly] public NativeArray<MPCFloat> MpcsFloat3;
        [ReadOnly] public NativeArray<LUT2> Lut2;
        

        [WriteOnly] public NativeArray<Lut2Dictionary> Lut2Dictionaries;
        public void Execute(int index)
        {
            int inId = Lut2[index].InMpcId;
            int outId = Lut2[index].OutMpcId;
            
            // Parameters of the first mpc2 (inMpc)
            float3 p1 = MpcsFloat3[inId].Point;
            float3 n1 = MpcsFloat3[inId].Normal;
            
            // Parameters of the second mpc2 (outMpc), which is visible from the first mpc2
            float3 p2 = MpcsFloat3[outId].Point;
            float3 n2 = MpcsFloat3[outId].Normal;
            
            float3 direction = p2 - p1;
            
            // Calculating image of the out Mpc
            float distancePlane1 = math.dot(n1, direction);
            float3 imageOutPoint = p2 - 2f * distancePlane1 * n1;
            // Calculating image of the in Mpc
            float distancePlane2 = math.dot(n2, -direction);
            float3 imageThisPoint = p1 - 2f * distancePlane2 * n2;

            float avgAttenuationCoefficient = 0.5f * (MpcsFloat3[inId].Attenuation + MpcsFloat3[outId].Attenuation);
            
            Lut2Dictionary lut2Word =
                new Lut2Dictionary(inId, outId, Lut2[index].Distance, imageOutPoint, imageThisPoint, avgAttenuationCoefficient);
            Lut2Dictionaries[index] = lut2Word;
            

        }
    }
    

    #endregion
    
    [BurstCompile]
    struct Lut2Generation : IJobParallelFor
    {
        [ReadOnly] public int MpcLength;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommands;
        [ReadOnly] public NativeArray<RaycastHit> RaycastResults;
        [ReadOnly] public NativeArray<MPCFloat> MpcFloats;
        [ReadOnly] public float LambdaOver4PI;
        [ReadOnly] public float AttenuationThreshold;
        
        [WriteOnly] public NativeList<LUT2>.ParallelWriter Lut;
        public void Execute(int index)
        {
            if (RaycastResults[index].distance == 0 && RaycastCommands[index].distance > 0)
            {
                int i = index / MpcLength;
                int j = index % MpcLength;
                // Attenuation
                float avgAttenuation = (MpcFloats[i].Attenuation + MpcFloats[j].Attenuation)*0.5f*LambdaOver4PI/RaycastCommands[index].distance;
                if (avgAttenuation > AttenuationThreshold)
                {
                    LUT2 tempLut = new LUT2(i, j, RaycastCommands[index].distance);
                    Lut.AddNoResize(tempLut);
                }
            }
        }
    }
    
    [BurstCompile]
    struct LutRaycasting : IJobParallelFor
    {
        [ReadOnly] public NativeArray<MPCFloat> Mpcs;
        [ReadOnly] public float MaxDistance;
        [WriteOnly] public NativeArray<RaycastCommand> RaycastCommands;
        public void Execute(int index)
        {
            int i = index / Mpcs.Length;
            int j = index % Mpcs.Length;
            
            // avoid considering i == j
            if (i == j)
            {
                RaycastCommands[index] = new RaycastCommand();
                return; // Early return if comparing the same MPC
            }
            
            MPCFloat mpc1 = Mpcs[i];
            MPCFloat mpc2 = Mpcs[j];
            
            float3 direction12 = mpc2.Point - mpc1.Point;
            float distance = math.distance(mpc2.Point,mpc1.Point);
            if (distance < MaxDistance)
            {
                float3 normalizedDirection = math.normalize(direction12);
                if (math.dot(mpc1.Normal, normalizedDirection) > 0.01f && math.dot(mpc2.Normal, normalizedDirection) < -0.01f)
                {
                    RaycastCommands[index] = new RaycastCommand(mpc1.Point, normalizedDirection, QueryParameters.Default, distance);
                }
                else
                {
                    RaycastCommands[index] = new RaycastCommand();
                }
            }
            else
            {
                RaycastCommands[index] = new RaycastCommand();
            }
            
        }
    }
}

public class NativeListFloatConversion
{
    public NativeArray<MPCFloat> FloatConversion(NativeArray<MPC.ScatterPoint> mpcList)
    {
        NativeArray<MPCFloat> mpcFloats = new NativeArray<MPCFloat>(mpcList.Length, Allocator.Persistent);
        
        for (int i = 0; i < mpcList.Length; i++)
        {
            float3 floatPoint = new float3(mpcList[i].Point.x, mpcList[i].Point.y, mpcList[i].Point.z);
            float3 floatNormal = new float3(mpcList[i].Normal.x, mpcList[i].Normal.y, mpcList[i].Normal.z);
            float floatAttenuation = math.pow(10f, mpcList[i].Attentuation/20f);
            MPCFloat mpcFloat = new MPCFloat(floatPoint, floatNormal, floatAttenuation);
            mpcFloats[i] = mpcFloat;
        }
        
        return mpcFloats;
    }

}