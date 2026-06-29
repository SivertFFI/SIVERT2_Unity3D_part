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
// using UnityEngine;
// using UnityEngine.AI;
//
// public class AgentControl : MonoBehaviour
// {
//     public Transform destination;
//     private NavMeshAgent _agent;
//
//     // Start is called before the first frame update
//     void Start()
//     {
//         _agent = GetComponent<NavMeshAgent>();
//     }
//
//     void Update()
//     {
//         _agent.destination = destination.position;
//     }
//
//     private void OnTargetReachedDestination()
//     {
//         // Quit the application
//         Debug.Log("Destination reached. Quitting application.");
//         Application.Quit();
//
//         // If running in the Unity editor
//         #if UNITY_EDITOR
//         UnityEditor.EditorApplication.isPlaying = false;
//         #endif
//     }
// }   

using UnityEngine;
using UnityEngine.AI;

public class AgentPath : MonoBehaviour
{
    public Transform[] waypoints;       // Assign these in the Inspector
    public float stopThreshold = 0.5f;  // Distance to consider "reached"
    
    private NavMeshAgent agent;
    private int currentIndex = 0;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        if (waypoints.Length > 0)
            agent.SetDestination(waypoints[currentIndex].position);
    }

    void Update()
    {
        if (!agent.pathPending && agent.remainingDistance < stopThreshold)
        {
            currentIndex = (currentIndex + 1) % waypoints.Length;
            agent.SetDestination(waypoints[currentIndex].position);
        }
    }
}