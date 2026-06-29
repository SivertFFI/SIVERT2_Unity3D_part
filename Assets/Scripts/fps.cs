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
using UnityEngine.UI;
using System.IO;
using System;

public class FPSDisplay : MonoBehaviour
{
    public Text fpsText;
    public int sampleSize = 60;
    public float logInterval = 5f; // Interval in seconds for logging FPS to a file

    private float[] frameTimes;
    private int frameIndex;
    private bool filled;
    private float logTimer;

    void Start()
    {
        frameTimes = new float[sampleSize];
        logTimer = logInterval;
    }

    void Update()
    {
        // Record the latest frame time
        frameTimes[frameIndex] = Time.unscaledDeltaTime;
        frameIndex = (frameIndex + 1) % sampleSize;
        if (frameIndex == 0) filled = true;

        // Calculate average frame time and FPS
        float averageFrameTime = CalculateAverageFrameTime();
        float fps = 1.0f / averageFrameTime;
        fpsText.text = Mathf.Ceil(fps).ToString() + " FPS";

        // Update the log timer and write to file if needed
        logTimer -= Time.deltaTime;
        if (logTimer <= 0)
        {
            WriteFPSToFile(fps);
            logTimer = logInterval;
        }
    }

    float CalculateAverageFrameTime()
    {
        float totalFrameTime = 0f;
        int count = filled ? sampleSize : frameIndex;
        for (int i = 0; i < count; i++)
        {
            totalFrameTime += frameTimes[i];
        }
        return totalFrameTime / count;
    }

    void WriteFPSToFile(float fps)
    {
        string filePath = Path.Combine(Application.dataPath, "../FPSLog.txt");
        string textToWrite = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss") + ": " + Mathf.Ceil(fps).ToString() + " FPS\n";
        File.AppendAllText(filePath, textToWrite);
    }
}