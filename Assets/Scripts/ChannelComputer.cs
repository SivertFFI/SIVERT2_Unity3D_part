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
using System.Linq;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

public class ChannelComputer : MonoBehaviour
{
    public ComputeShader channelComputeShader;
    private int kernelHandle;

    void Start()
    {
        kernelHandle = channelComputeShader.FindKernel("CSMain");
    }

    public Vector2[] ComputeChannels(
        int sizeOfH,
        int numberOfLinks,
        NativeArray<Main.PathGainResultEntry> paths1Gain,
        NativeArray<Main.PathGainResultEntry> paths2Gain,
        NativeArray<Main.PathGainResultEntry> paths3Gain,
        NativeArray<float> inverseLambda)
    {
        int totalSize = sizeOfH * numberOfLinks; // You need to determine the total size
        
        // Calculate the stride for PathGainResultEntry
        int pathGainResultEntryStride = sizeof(int) * 4 + sizeof(float) * 2;
        
        // Create compute buffers
        ComputeBuffer paths1Buffer = new ComputeBuffer(paths1Gain.Length, pathGainResultEntryStride);
        ComputeBuffer paths2Buffer = new ComputeBuffer(paths2Gain.Length, pathGainResultEntryStride);
        ComputeBuffer paths3Buffer = new ComputeBuffer(paths3Gain.Length, pathGainResultEntryStride);
        ComputeBuffer inverseLambdaBuffer = new ComputeBuffer(inverseLambda.Length, sizeof(float));
        ComputeBuffer resultBuffer = new ComputeBuffer(totalSize, sizeof(float) * 2);
        ComputeBuffer debugBuffer = new ComputeBuffer(totalSize, sizeof(float) * 4); // Debug buffer


        // Set data
        paths1Buffer.SetData(paths1Gain.ToArray());
        paths2Buffer.SetData(paths2Gain.ToArray());
        paths3Buffer.SetData(paths3Gain.ToArray());
        inverseLambdaBuffer.SetData(inverseLambda.ToArray());

        // Set buffers and variables
        channelComputeShader.SetInt("SizeOfH", sizeOfH);
        channelComputeShader.SetBuffer(kernelHandle, "Paths1Gain", paths1Buffer);
        channelComputeShader.SetBuffer(kernelHandle, "Paths2Gain", paths2Buffer);
        channelComputeShader.SetBuffer(kernelHandle, "Paths3Gain", paths3Buffer);
        channelComputeShader.SetBuffer(kernelHandle, "InverseLambda", inverseLambdaBuffer);
        channelComputeShader.SetBuffer(kernelHandle, "H", resultBuffer);
        channelComputeShader.SetBuffer(kernelHandle, "debugBuffer", debugBuffer);

        // Dispatch
        int threadGroupsX = Mathf.CeilToInt(totalSize / 64f);
        channelComputeShader.Dispatch(kernelHandle, threadGroupsX, 1, 1);

        // Get results
        //NativeArray<Vector2> result = new NativeArray<Vector2>(totalSize, Allocator.Temp);
        //resultBuffer.GetData(result);
        // Get results
        Vector2[] result = new Vector2[totalSize];
        resultBuffer.GetData(result);
        /*
        float4[] debugData = new float4[totalSize];
        debugBuffer.GetData(debugData);

        for (int i = 0; i < debugData.Length; i++)
        {
            Debug.Log($"Debug[{i}]: {debugData[i]}");
        }*/

        // Clean up
        paths1Buffer.Release();
        paths2Buffer.Release();
        paths3Buffer.Release();
        inverseLambdaBuffer.Release();
        resultBuffer.Release();

        // Use 'result' as needed
        // ...

        //result.Dispose();
        return result;
    }
}