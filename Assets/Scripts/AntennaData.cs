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

#region For parallel jobs
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;
#endregion

public struct AntennaData
{
    public float3 Position;
    public float3 Forward;
    public float3 Up;
    public float3 Right;
}


public struct AntennaAndRotationData
{
    public float3 Position;
    public float3x3 Rotation; // Store the entire rotation matrix
}

public struct UpdateAntennaDataJob : IJobParallelForTransform
{
    public NativeArray<AntennaAndRotationData> AntennaAndRotationDataArray;

    public void Execute(int index, TransformAccess transform)
    {
        AntennaAndRotationDataArray[index] = new AntennaAndRotationData
        {
            Position = transform.position,
            Rotation = new float3x3(transform.rotation)
        };
    }
}

// The following code is used to calculate the polarization transformation matrix
public static class AFOperations
{
    public static float3x3 PolarizationTransformationMatrix(float3 a)
    {
        return new float3x3(
            1f - a.x * a.x,    a.x * a.y,       a.x * a.z,
              a.y * a.x,     1f - a.y * a.y,    a.y * a.z,
              a.z * a.x,       a.z * a.y,     1f - a.z * a.z
        );
    }
}