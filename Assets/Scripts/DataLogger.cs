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
using System.Numerics;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Concurrent;
using System.IO;
using System.Text;
using System.Collections;
using System.Globalization;
using System.Collections.Generic;
using System.Xml;
using System.Linq;
using System;
using UnityEngine.SceneManagement;

public class DataLogger : MonoBehaviour
{

    #region Application mode
    public string applicationMode;
    public string platform;
    #endregion

    private readonly ConcurrentQueue<string> dataQueue = new ConcurrentQueue<string>();
    private readonly char separator = ',';
    private readonly float writeInterval = 1.0f; // Interval in seconds to flush data
    private string _filePath;
    private bool _isLoggingEnabled = true;
    private bool _anyData = false;
    private StreamWriter _streamWriter;

    private void Start()
    {
        StartCoroutine(FlushDataPeriodically());
    }

    private void OnDestroy()
    {
        StopLogging();
        Debug.Log("DataLogger: OnDestroy called, logging stopped.");
    }

    public void StartLogging()
    {
        if (_isLoggingEnabled)
        {
            Debug.LogWarning("DataLogger: Logging is already started.");
            return;
        }

        _isLoggingEnabled = true;
        Debug.Log("DataLogger: Logging started.");
    }

    public void StopLogging()
    {
        if (!_isLoggingEnabled)
        {
            Debug.LogWarning("DataLogger: Logging is already stopped.");
            return;
        }

        _isLoggingEnabled = false;
        FlushAndCloseStreamWriter();
        Debug.Log("DataLogger: Logging stopped and data flushed.");
    }

    public void SetFilePath(string basePath, string baseFileName = "dataLog")
    {
        try
        {
            Scene currentScene = SceneManager.GetActiveScene();
            string dateTimeFormat = "yyyyMMdd_HHmmss";
            string dateTimeString = System.DateTime.Now.ToString(dateTimeFormat, CultureInfo.InvariantCulture);
            string newFilePath = Path.Combine(basePath, $"{baseFileName}_{currentScene.name}_{dateTimeString}_{applicationMode}_{platform}.csv");

            _filePath = newFilePath;
            Debug.Log($"DataLogger file path set to: {_filePath}");

            _streamWriter = new StreamWriter(_filePath, true) { AutoFlush = true };
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"DataLogger: Failed to set file path and create StreamWriter. Error: {ex.Message}");
        }
    }

    public string GetFilePath()
    {
        return _filePath;
    }

    private List<string> GenerateSubHeader(string label, int count) {
        return Enumerable.Range(0, count).Select(i => $"{label}{i}").ToList();
    }

    private List<string> GenerateSubHeaderChannel(int linkCount, string label="CH", int numSubcarriers = 1) {
        return Enumerable.Range(0, linkCount)
            .SelectMany(linkID => Enumerable.Range(0, numSubcarriers),
                        (linkID, subcarrier) => $"{label}:{linkID}:{subcarrier}")
            .ToList();
    }

    public void EnqueueData(float timeStamp, float frameRate, long totalTicks, GameObject[] entityArray, NativeArray<float3> entityCoordinates,
        NativeArray<int2> channelLinkPairs, List<int> channelLinkPaths, NativeArray<Complex> channelData)
    {
        if (!_isLoggingEnabled)
        {
            Debug.LogWarning("DataLogger: Attempted to enqueue data while logging is disabled.");
            return;
        }

        if (!_anyData){
            List<string> header = new List<string>{"timestamp"};
            header.AddRange(new List<string>{"FPS"});
            header.AddRange(new List<string>{"TICKS"});
            header.AddRange(GenerateSubHeader("ENTITY", entityCoordinates.Length));
            header.AddRange(GenerateSubHeader("LINK", channelLinkPairs.Length));
            header.AddRange(new List<string>{"PATHS"});
            header.AddRange(GenerateSubHeaderChannel(linkCount:channelLinkPairs.Length, label:"CH", numSubcarriers:channelData.Length/channelLinkPairs.Length));

            dataQueue.Enqueue(string.Join(separator,header));

            _anyData = true;
        }

        string dataToLog = $"{timeStamp}," + 
                        $"{frameRate}," +
                        $"{totalTicks}," +
                        $"{NativeArrayToCSV(entityCoordinates)}," +
                        $"{NativeArrayToCSV(entityArray, channelLinkPairs)}," +
                        $"{ListToCSV(channelLinkPaths)}," + // Add this line to include channelLinkPaths
                        $"{NativeArrayToCSV(channelData)}";

        dataQueue.Enqueue(dataToLog);
    }

    void Awake()
    {
        platform = Environment.GetEnvironmentVariable("PLATFORM");

        if (string.IsNullOrEmpty(platform))
        {
           
            switch (Application.platform)
            {
                case RuntimePlatform.WindowsPlayer:
                    platform = "Windows";
                    break;
                case RuntimePlatform.OSXPlayer:
                    platform = "macOS";
                    break;
                case RuntimePlatform.LinuxPlayer:
                    platform = "Linux";
                    break;
                default:
                    platform = "Unknown platform";
                    break;
            }
        }

        if (Application.isEditor)
        {
            applicationMode = "Editor";
        }
        else if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            applicationMode = "Headless";
        }
        else
        {
            applicationMode = "Compiled";
        }

        Debug.Log("Application Mode: " + applicationMode);
    }

    // Convert List<int> to CSV string
    private string ListToCSV(List<int> list)
    {
        return  string.Join(":", list); // Use colon as separator if commas are used for other purposes
    }

    //Overloaded NativeArrayToCSV methods to handle different data types
    private string NativeArrayToCSV(NativeArray<float3> coordinates)
    {
        StringBuilder sb = new StringBuilder();
        foreach (var coord in coordinates) sb.Append($"x:{coord.x}y:{coord.y}z:{coord.z}{separator}");
        return sb.ToString().TrimEnd(separator);
    }

    private string NativeArrayToCSV(GameObject[] entityArray, NativeArray<int2> linkPairs)
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < linkPairs.Length; i++)
            sb.Append($"Link{i}:{entityArray[linkPairs[i].x].name}:{entityArray[linkPairs[i].y].name}{separator}");
        return sb.ToString().TrimEnd(separator);
    }

    // Complex data handling remains the same
    private string NativeArrayToCSV(NativeArray<Complex> complexData)
    {
        StringBuilder sb = new StringBuilder();
        foreach (var item in complexData) sb.Append($"{item.Real}{GetSignString((float)item.Imaginary)}{Math.Abs(item.Imaginary)}j{separator}");
        
        return sb.ToString().TrimEnd(separator);
    }
    
    public static string GetSignString(float number)
    {
        if (number >= 0)
        {
            return "+";
        }     
        else 
        {
            return "-";     
        }
    }

    private void FlushAndCloseStreamWriter()
    {
        if (_streamWriter == null) return;

        try
        {
            while (dataQueue.TryDequeue(out string data)) _streamWriter.WriteLine(data);
            _streamWriter.Close();
            _streamWriter = null;
            Debug.Log("DataLogger: StreamWriter flushed and closed.");
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"DataLogger: Error flushing and closing StreamWriter. Error: {ex.Message}");
        }
    }

    private IEnumerator FlushDataPeriodically()
    {
        while (true)
        {
            yield return new WaitForSeconds(writeInterval);
            if (_isLoggingEnabled && dataQueue.Count > 0)
                try
                {
                    while (dataQueue.TryDequeue(out string data)) _streamWriter.WriteLine(data);
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"DataLogger: Error writing data to file. Error: {ex.Message}");
                }
        }
    }
}