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

using System.Collections;
using System.Collections.Generic;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using UnityEngine;
using System.Threading;
using UnityEditor;
using UnityEditor.VersionControl;
using UnityEngine.Rendering;
using UnityEngine.Serialization;
using UnityEngine.UI;
using UnityEngine.UIElements;

// ZeroMQ dependency for API's TCP/IP connection
using ZeroMQ;

// Flatbuffers used as API cross-platform data structures
using Google.FlatBuffers;

// APIs datastructures definition
using SivertAPI.PosUpd;
using SivertAPI.MsgReceived;

using Debug = UnityEngine.Debug;



[InitializeOnLoad]
    public class Sivert_API_GSCM_ECS : MonoBehaviour
    {
        // Start is called before the first frame update

        private GameObject RSSsignPtr;
        private double SignFlashTimer;
        private bool ActivateFlashingSign = false;
        [Tooltip("Maximum play duration in seconds")]
        public float simulationDurationInSeconds = 100f;
        private float playTimeElapsed = 0f;
        public bool UseNS3Binary = true;
        public bool LocalNS3Binary = true;
        public bool DebugNS3 = true;
        
        public string ns3BinaryName = "ns3.42-cttc-nr-mimo-demo-API-debug";
        public string shellScriptName = "NS3DaemonForUnity.sh";
        public string pathToNs3Binary = "/Users/nlyamin/WRK/Simulation/SIVERT2/lena3.3_freshBuild/ns-3-dev/build/contrib/nr/examples";
        public string pathToNs3ProjectRoot = "/Users/nlyamin/WRK/Simulation/SIVERT2/lena3.3_freshBuild/ns-3-dev";
        public string logBasePath = "NS3_Logs";

        // private Process ns3Process;
        
        public bool EnableCamerasBehindVehicles = true;
        private GameObject WarningSignPtr;
        private GameObject WarningSignPtrLoS;
        public float timeScale = 1.0f;
        private Transform goTransform;
        public GameObject[] allVehicles;
        public GameObject[] alleNB;
        public List<int> allVehiclesID;
        private List<GameObject> leftLights;
        private List<GameObject> rightLights;
        // public List<MOBILIDMPathTracker> aiPointer;
        public bool ColorIntersections = true;
        public GameObject[] juncts;
        public Material asp;


        public List<Vector3> vehPos;
        // private List<AILogic> vehControllers;
        public float tm;
        public string topicToNS = "toNS";
        public string topicFromNS = "fromNS";


        private ZeroMQ.ZContext context;
        private ZeroMQ.ZSocket server;
        private ZeroMQ.ZSocket client;


        private ZeroMQ.ZSocket initNSserver;
        private ZeroMQ.ZSocket intitNSresponse;

        // private Thread ns3Thread;
        private Process NS3thread;
        StringBuilder outputBuilder;
        private String output;

        private float Delay;
        public float SimStart = 3.0f;
        public int VehNumToNS = 2;
        public int InterferenceLevelInDbmPer10Mhz = -120; 
        public string PathToNs3Project = "/Users/xxx/xxx/ns-3";
        public enum Scenario{cellular5G, NRV2X, GSCM_11p, LTE_V2X};
        public Scenario Stack_v2x_Tech;
        public bool EMI_NS3_scenario = false;
        
        public bool UseMeasureEMIData = false;
        // public string EMImeasuredFile = "EMI_sample_data_plus_10dBmuV.csv";
        public bool DisbaleEMIforDemo = false;

        public enum AppV2x
        {
            Automatic_intersection,
            Demo_intersection_assist,
            None
        };

        public AppV2x CITS_scenario;

        // public bool TwoAntennas = false;
//        public dynamic GSCMpointer;
        public bool isMA = false; 
        // public ChannelGenManager_MA GSCMpointer_MA;
        // public ChannelGenManager_MA PtrToGSCM;

        public Main PtrToGSCM; 
//        public ChannelGenManager GSCMpointer;
        

        public bool UseGSCM = true;
        // private SivertSqLiteLogger PointerToSqLLogger;

        private void Awake()
        {
            Time.timeScale = timeScale;



            /*
             * NOTE: We run the NS3 process first, when Unity awakes. Thus, we also synchronise the zero time of Unity3D and NS3.
             */



            // TODO: rewrite shell script to pass scenario module name parameter to it directly, thus, having one call shell script.
            Debug.Log("Checking the OS..." );
//            if (RuntimeInformation.OSDescription.Substring(0,4) == "Unix") // If we running MacOS
            if (RuntimeInformation.OSDescription.Substring(0,4) == "Unix") // If we running MacOS
            {
                Debug.Log("We're running MacOS" );

                if (Stack_v2x_Tech == Scenario.cellular5G)
                {
                    if (!DebugNS3) // If NS3 debug is activated, NS3 process is started from C++ IDE in debug mode
                    {
                        Run5gScenarioInNS3(); // New Launcher that creates a directory for the run and copies the binary
                    }
                    
                    
                }
                else if (Stack_v2x_Tech == Scenario.NRV2X)
                {
                    if (!DebugNS3) // If NS3 debug is activated, NS3 process is started from C++ IDE in debug mode
                    {
                        RunNrV2xScenarioInNS3(); // New Launcher that creates a directory for the run and copies the binary
                    }
                }
                else if (Stack_v2x_Tech == Scenario.LTE_V2X)
                {
                  string arg;

                  if (UseNS3Binary) {
                    #if UNITY_EDITOR
                        string LibPath = Application.dataPath;
                        LibPath = LibPath.Replace("Assets", "Packages/SIVERT_NS3_bin");

                    #endif
                    arg = "-e 'tell app \"Terminal\" to do script \"" + LibPath +  "/NS3DaemonForUnity.sh " + LibPath +  " ns3-dev-CITS-GSCM-SIVERT-lteV2X-spectrum-debug" + "\"'";

                  }
                  else { // Specify correct path to NS3 is want to add another scenario from NS3
                    arg = "-e 'tell app \"Terminal\" to do script \"" + PathToNs3Project +  "/NS3DaemonForUnity.sh " + PathToNs3Project + " build/src/wave/examples/ns3-dev-CITS-GSCM-SIVERT-lteV2X-spectrum-debug" + "\"'";
                  }

                    NS3thread = System.Diagnostics.Process.Start ("osascript",arg);

                }
                else
                {
                    Debug.LogError("Scenario for NS3 is incorrectly specified!");
                }
            }
//            else if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux)) // If we running Linux -- Not yet updated and verified!
            if (SystemInfo.operatingSystem.Substring(0,5) == "Linux") // If we running Linux
            {
                Debug.Log("We're running Linux");
                if (Stack_v2x_Tech == Scenario.GSCM_11p)
                {
                    
                    /////// original thread creating process
                    string LibPath = Application.dataPath;
                    LibPath = LibPath.Replace("Assets", "Packages/SIVERT_NS3_bin");

                    if (EMI_NS3_scenario)
                    {
                        ExecuteTerminalCommand("gnome-terminal -x bash -ic 'cd /home/nlyamin/VANETs/Simulations/SIVERT2_dev_editor_upgrade_2020/Packages/SIVERT_NS3_bin; ./NS3DaemonForUnity /home/nlyamin/VANETs/Simulations/NS3_LTH_Linux/ns-3.29 CITS-SIVERT-11p-spectrum-measured-interference-vectorised; bash'");
                    }
                    else
                    {
                        ExecuteTerminalCommand("gnome-terminal -x bash -ic 'cd /home/nlyamin/VANETs/Simulations/SIVERT2_dev_editor_upgrade_2020/Packages/SIVERT_NS3_bin; ./NS3DaemonForUnity /home/nlyamin/VANETs/Simulations/NS3_LTH_Linux/ns-3.29 CITS-SIVERT-11p-spectrum; bash'");
                    }
                   


                    
                    
                    
                }
                else if (Stack_v2x_Tech == Scenario.LTE_V2X)
                {
           
                    
                    string LibPath = Application.dataPath;
                    LibPath = LibPath.Replace("Assets", "Packages/SIVERT_NS3_bin");
                    
                    if (EMI_NS3_scenario)
                    {
                        ExecuteTerminalCommand("gnome-terminal -x bash -ic 'cd /home/nlyamin/VANETs/Simulations/SIVERT2_dev_editor_upgrade_2020/Packages/SIVERT_NS3_bin; ./NS3DaemonForUnity /home/nlyamin/VANETs/Simulations/NS3_LTH_Linux/ns-3.29 CITS-GSCM-SIVERT-lteV2X-spectrum; bash'");
                    }
                    else
                    {
                        ExecuteTerminalCommand("gnome-terminal -x bash -ic 'cd /home/nlyamin/VANETs/Simulations/SIVERT2_dev_editor_upgrade_2020/Packages/SIVERT_NS3_bin; ./NS3DaemonForUnity /home/nlyamin/VANETs/Simulations/NS3_LTH_Linux/ns-3.29 CITS-GSCM-SIVERT-lteV2X-spectrum; bash'");
                    }

                }
                else
                {
                    Debug.LogError("Scenario for NS3 is incorrectly specified!");
                }
            }


            RSSsignPtr = GameObject.Find("RSS Chart");
            
            SimStart = Time.time + Time.fixedDeltaTime;
            // Invoke(nameof(StartAPI), SimStart-1);
        }


        void Start()
        {

            // PointerToSqLLogger = this.GetComponent<SivertSqLiteLogger>();
            
            Invoke(nameof(InitNS3Thread), SimStart-Time.fixedDeltaTime*1);

        }

        private void InitNS3Thread()
        {
            // AsyncIO.ForceDotNet.Force();
            context = ZContext.Create();


            try
            {
                Debug.Log("ZeroMQ socket is setup...");
                initNSserver = new ZSocket(context, ZSocketType.REQ);
                initNSserver.Connect("tcp://127.0.0.1:5555");

                Debug.Log("Sending init parameters...");

                if (EMI_NS3_scenario)
                {
                    initNSserver.SendMore(new ZFrame(Time.fixedDeltaTime.ToString()));
                    initNSserver.SendMore(new ZFrame(SimStart.ToString()));
                    initNSserver.SendMore(new ZFrame(VehNumToNS.ToString()));
                    initNSserver.SendMore(new ZFrame(UseGSCM.ToString()));
                    initNSserver.SendMore(new ZFrame(DisbaleEMIforDemo.ToString()));
                    initNSserver.Send(new ZFrame(InterferenceLevelInDbmPer10Mhz.ToString()));
                }
                else
                {
                    initNSserver.SendMore(new ZFrame(Time.fixedDeltaTime.ToString()));
                    initNSserver.SendMore(new ZFrame(SimStart.ToString()));
                    initNSserver.SendMore(new ZFrame(VehNumToNS.ToString()));
                    initNSserver.Send(new ZFrame(UseGSCM.ToString()));
                }
                

            }
            catch (Exception e)
            {
                initNSserver.Unbind("tcp://127.0.0.1:5555");
                initNSserver.Disconnect("tcp://127.0.0.1:5555");
                initNSserver.Close();
                initNSserver.Dispose();
                Debug.LogError("Init socket gave following error: " + e);
                throw;
            }

            InitVehicles();
            // Invoke(nameof(InitVehicles), SimStart-Time.fixedDeltaTime*2);
        }

        private void InitVehicles()
        {
            server = new ZSocket(context, ZSocketType.PUB);
            server.Linger = TimeSpan.Zero;
            server.Connect("tcp://127.0.0.1:8002");
            client = new ZSocket(context, ZSocketType.SUB);
            client.Linger = TimeSpan.Zero;
            client.ReceiveTimeout = TimeSpan.FromMilliseconds(1);
            client.Connect("tcp://127.0.0.1:8001");
            client.Subscribe(topicFromNS);


            // Getting pointers to all vehicles
            allVehicles = GameObject.FindGameObjectsWithTag("UE");
            alleNB = GameObject.FindGameObjectsWithTag("RBS");
            // Getting pointer to GSCM channel
            // GameObject gscmGO = GameObject.Find("ManagerGSCM");
            
            GameObject _goMain = GameObject.Find("Scripts");
            if (_goMain != null)
            {
               PtrToGSCM = _goMain.GetComponent<Main>();
            }
            else
            {
                Debug.LogError("No pointer to GSCM main is found!");
            }
            
            int k = 0;
            
            foreach (var veh in alleNB) // UEs
            {

                try
                {

                    // allVehiclesID[k] = Int16.Parse(veh.name.Substring(8, 1));
                    allVehiclesID.Add(k);
                    vehPos[k] = veh.transform.position;

                    k++;
                }
                catch (Exception e)
                {
                    Debug.Log(e + "Growing the size of the vehicles dynamically");
                    vehPos.Add(veh.transform.position);
                    k++;

                }

            }
            
            foreach (var veh in allVehicles) // eNBs
            {

                try
                {

                    // allVehiclesID[k] = Int16.Parse(veh.name.Substring(8, 1));
                    allVehiclesID.Add(k);
                    vehPos[k] = veh.transform.position;

                    k++;
                }
                catch (Exception e)
                {
                    Debug.Log(e + "Growing the size of the vehicles dynamically");
                    vehPos.Add(veh.transform.position);
                    k++;

                }

            }
            

        }


        private void Update()
        {
            playTimeElapsed += Time.deltaTime;
            if (playTimeElapsed >= simulationDurationInSeconds)
            {
                Debug.Log($"Play duration of {simulationDurationInSeconds} reached. Current Unity time is {playTimeElapsed}. Wrapping up simulations...");
                StopSimulatioBasedOnDurationParameter();
            }
        }

        private void FixedUpdate()
        {
            // AsyncIO.ForceDotNet.Force();
            tm = Time.time;

            if (tm >= SimStart)
            {

                // Sending the update to API
                var message = new ZMessage();
                message.Add(new ZFrame(string.Format(topicToNS)));
                message.Add(new ZFrame(makeFBposAPI(vehPos, false)));
                server.Send(message);
                Debug.Log("Sending the update to NS3 through API at: " + Time.time);

            

                //////////////// API messages reception
                ///
                ZError error;
                ZMessage msg;
                ZPollItem pollItem = ZPollItem.CreateReceiver();

                while (client.PollIn(pollItem, out msg, out error, TimeSpan.Zero))
                {
                    try
                    {

                        var content = msg.Unwrap().ReadString();
                        byte[] bytesMSG;
                        bytesMSG = msg.Unwrap().Read();

                        // Read flatbuffer
                        try
                        {
                            ByteBuffer buf = new ByteBuffer(bytesMSG);
                            var MsgFromNS3 = MsgRecAPI.GetRootAsMsgRecAPI(buf);
                            // var pos = MsgFromNS3.Pos.Value;
                            var RxInfo = MsgFromNS3.PackContent.Value;
                            var RxID = MsgFromNS3.PackContent.Value.ReceiverID;
                            var TxID = MsgFromNS3.PackContent.Value.SenderID;
                            var Sent = MsgFromNS3.PackContent.Value.Sent;
                            var NS3time = MsgFromNS3.PackContent.Value.TimeReceived; // NS3 time in nanoseconds
                            string BeaconContent = MsgFromNS3.MsgContent;

                            if (Sent)
                            {
                                Debug.Log("Message sent by vehicle: " + TxID +  " at time: " +
                                          RxInfo.TimeReceived + " with content: " + BeaconContent + " UnityTime is: " + tm + " NS3 time: " + NS3time
                                          );
                            }
                            else
                            {

                                Debug.Log("Message received by vehicle: " + RxID + " From vehicle: " + TxID + " at time: " +
                                          RxInfo.TimeReceived + " with content: " + BeaconContent + " UnityTime is: " + tm + " NS3 time: " + NS3time);
                            }

                        }
                        catch (Exception e)
                        {
                            Debug.LogWarning("Can't decode flatbuffer." + " Error: " + e);
                            Time.timeScale = 0; // To pause the run.
                        }

                    }
                    catch (Exception e)
                    {
                        Debug.Log("No message was received at: " + Time.time + " ZError: " + e);
                    }
                }

                ///////////////// end API message receiving



            }
        }



        private byte[] makeFBposAPI(List<Vector3> UnityPos, bool CloseNS3)
        {

            FlatBufferBuilder fbb = new FlatBufferBuilder(128);
            var GSCMrss = GSCMstruct.CreateGSCMstruct(fbb, UseGSCM, 0.0);
            if (UseGSCM)
            {
                GSCMrss = GSCMstruct.CreateGSCMstruct(fbb, UseGSCM, 0);
            }

            PosAPI.StartGSCMvectorVector(fbb, PtrToGSCM.GSCMrss.RSS.Count);

            for (int i = 0; i < PtrToGSCM.GSCMrss.RSS.Count; i++)
            {
                GscmInfo.CreateGscmInfo(fbb, PtrToGSCM.GSCMrss.Links[i].x, PtrToGSCM.GSCMrss.Links[i].y, PtrToGSCM.GSCMrss.RSS[i]);
            }

            var GSCMoffset = fbb.EndVector();


            int vehId = 0;
            PosAPI.StartCITSVector(fbb, allVehicles.Length);

            for (int i = 0; i < allVehicles.Length; i++)
            {

                EEBL.CreateEEBL(fbb, false, vehId);

                vehId++;
            }

            var EEBLoffset = fbb.EndVector();


            PosAPI.StartPosVector(fbb, UnityPos.Count);
            for (int i = 0; i < UnityPos.Count; i++)
            {

                Vector3 p = UnityPos[i];
                Vec3API.CreateVec3API(fbb, p.x, p.y, p.z, allVehiclesID[i]);
            }

            var posOffset = fbb.EndVector();


            PosAPI.StartSupplVector(fbb, allVehicles.Length);
            for (int i = 0; i < allVehicles.Length; i++)
            {
                // VehInfo.CreateVehInfo(fbb, GetVehIdByVehGO(allVehicles[i]), tm);
                VehInfo.CreateVehInfo(fbb, allVehiclesID[i], tm);
            }
            var infoOffset = fbb.EndVector();

        int FreqNum = PtrToGSCM.GSCMSpectrum.RSSspectrumGain[0].Count; // number of RBs/subcarriers per channel
        int linkCount = PtrToGSCM.GSCMSpectrum.Links.Count;            // physical GSCM links
        // Store the offsets for SpectrumValue tables
        var spectrumValueOffsets = new List<Offset<SpectrumValue>>();

        // Create per-channel container lists
        List<List<double>> channels = new List<List<double>>();              // magnitude / (or any scalar) (currently not used when complex sent)
        List<List<double>> channelsComplex = new List<List<double>>();       // packed [mag(or unused?) | real | imag] per RB (as provided by Unity earlier)
        List<List<double>> channelsRl = new List<List<double>>();            // real parts
        List<List<double>> channelsIm = new List<List<double>>();            // imaginary parts

        // Always add exactly linkCount GSCM channels (avoid off-by-one misuse of +1 EMI count)
        for (int ch = 0; ch < linkCount; ch++)
        {
            channels.Add(PtrToGSCM.GSCMSpectrum.RSSspectrumGain[ch]);
            channelsRl.Add(PtrToGSCM.GSCMSpectrum.RSSspectrumGainRl[ch]);
            channelsIm.Add(PtrToGSCM.GSCMSpectrum.RSSspectrumGainIm[ch]);
            channelsComplex.Add(PtrToGSCM.GSCMSpectrum.RSSComplexChannel[ch]);
        }

        if (UseMeasureEMIData)
        {
            // Correct row selection: GetLength(0) gives number of rows, .Length would give total elements
            int emiRows = PtrToGSCM.emiWaveSectrumScaled.GetLength(0);
            int rowIndex = Mathf.FloorToInt(Time.time) % emiRows; // cycle through EMI rows deterministically per second
            List<double> emiVector = GetRowFromMatrix(PtrToGSCM.emiWaveSectrumScaled, rowIndex).Select(f => (double)f).ToList();
            channels.Add(emiVector); // append EMI as last channel
        }

        Debug.Log($"[GSCM->FB] linkCount={linkCount} freqNum={FreqNum} UseEMI={UseMeasureEMIData} totalSerializedChannels={channels.Count} (last is EMI? {UseMeasureEMIData})");

        // Loop over each channel and create SpectrumValue tables
        for (int channelIndex = 0; channelIndex < channels.Count; channelIndex++)
        {



            if (UseMeasureEMIData && (channelIndex == channels.Count-1))
            {
                // Convert the List<double> to an array for FlatBuffers
                double[] psdGainArray = channels[channelIndex].ToArray();
                

                // Create the PSDGainCoefficient vector for this channel
                VectorOffset psdGainVector = SpectrumValue.CreatePSDGainCoefficientVector(fbb, psdGainArray);
                
                int TxID = 99;
                int RxID = 99;
                Offset<SpectrumValue> spectrumValueOffset = SpectrumValue.CreateSpectrumValue(
                    fbb,
                    100,               // ChannelID (use channelIndex)
                    TxID,         // TxID 
                    RxID,         // RxID 
                    true,                      // IsEMIChannel (example flag)
                    psdGainVector,              // PSDGainCoefficient vector
                    100                // CoefInd (example index)
                );
                // Add this SpectrumValue to the list of offsets
                spectrumValueOffsets.Add(spectrumValueOffset);
                Debug.Log($"[GSCM->FB] Serialized EMI channel idx={channelIndex} size={psdGainArray.Length}");
            }
            else
            {
                // Convert the List<double> to an array for FlatBuffers
                double[] psdGainArray = channels[channelIndex].ToArray();
                double[] psdGainArrayRl = channelsRl[channelIndex].ToArray();
                double[] psdGainArrayIm = channelsIm[channelIndex].ToArray();
                double[] psdComplexChannelArray = channelsComplex[channelIndex].ToArray();

                // Create the PSDGainCoefficient vector for this channel
                VectorOffset psdGainVector = SpectrumValue.CreatePSDGainCoefficientVector(fbb, psdGainArray);
                VectorOffset psdComplexChannelVector = SpectrumValue.CreatePSDGainCoefficientVector(fbb, psdComplexChannelArray);
                VectorOffset psdGainVectorRl = SpectrumValue.CreatePSDGainCoefficientVector(fbb, psdGainArrayRl);
                VectorOffset psdGainVectorIm = SpectrumValue.CreatePSDGainCoefficientVector(fbb, psdGainArrayIm);
                
                int TxID = PtrToGSCM.GSCMSpectrum.Links[channelIndex].x;
                int RxID = PtrToGSCM.GSCMSpectrum.Links[channelIndex].y;
                Offset<SpectrumValue> spectrumValueOffset = SpectrumValue.CreateSpectrumValue(
                    fbb,
                    channelIndex,               // ChannelID (use channelIndex)
                    TxID,         // TxID 
                    RxID,         // RxID 
                    false,                      // IsEMIChannel (example flag)
                    psdComplexChannelVector, //psdGainVector,              // PSDGainCoefficient vector
                    channelIndex                // CoefInd (example index)
                );
                // Add this SpectrumValue to the list of offsets
                spectrumValueOffsets.Add(spectrumValueOffset);
                Debug.Log($"[GSCM->FB] Serialized link channelIndex={channelIndex} Tx={TxID} Rx={RxID} complexSize={psdComplexChannelArray.Length}");
                
            }
        }
        // Create a vector of SpectrumValue tables
        VectorOffset spectrumValuesVector = fbb.CreateVectorOfTables(spectrumValueOffsets.ToArray());
        

        // Create the PosAPI table
        PosAPI.StartPosAPI(fbb);
        PosAPI.AddTerminateNS3(fbb, CloseNS3);
        PosAPI.AddPos(fbb, posOffset);
        PosAPI.AddGSCMvector(fbb, GSCMoffset);
        PosAPI.AddSuppl(fbb, infoOffset);
        PosAPI.AddCITS(fbb, EEBLoffset);
        PosAPI.AddGSCM(fbb, GSCMrss);
        PosAPI.AddGSCMSpectruChannels(fbb, spectrumValuesVector);
        PosAPI.AddNumberOfFrequnciesPerChannel(fbb, FreqNum);
        Offset<PosAPI> posAPIOffset = PosAPI.EndPosAPI(fbb);
        // Finish the buffer
        fbb.Finish(posAPIOffset.Value);

        
        return fbb.SizedByteArray();
            
        }
        
        float[] GetRowFromMatrix(float[,] matrix, int rowIndex)
        {
            // Get the number of columns in the matrix
            int columnCount = matrix.GetLength(1);

            // Create a new float array to hold the row
            float[] row = new float[columnCount];

            // Copy the row from the matrix to the new array
            for (int i = 0; i < columnCount; i++)
            {
                row[i] = matrix[rowIndex, i];
            }

            return row;
        }


        private void OnApplicationQuit()
        {
            var message = new ZMessage();
            message.Add(new ZFrame(string.Format(topicToNS)));
            message.Add(new ZFrame(makeFBposAPI(vehPos, true)));
            server.Send(message);

            // server.SendMoreFrame(topicToNS).SendFrame(makeFBposAPI(vehPos, true));
            Debug.Log("Destroying thread for TCP/IP socket between Unity3D and ns3");

            if (NS3thread.Responding)
            {
                NS3thread.Close();
            }
            
            server.Unbind("tcp://127.0.0.1:8002");
            client.Unbind("tcp://127.0.0.1:8001");
            server.Close();
            client.Close();

            initNSserver.Unbind("tcp://127.0.0.1:5555");
            initNSserver.Close();

            context.Shutdown();
        }

        private void StopSimulatioBasedOnDurationParameter()
        {
            

            #if UNITY_EDITOR
                        EditorApplication.isPlaying = false; // Stop Play Mode in the Editor
            #else
                        Application.Quit(); // Quit app in builds
            #endif
        }

        
        private void ExecuteTerminalCommand(string command)
        {
            NS3thread = new System.Diagnostics.Process ();
            NS3thread.StartInfo.FileName = "/bin/bash";
            NS3thread.StartInfo.Arguments = "-c \" " + command + " \"";
            NS3thread.StartInfo.UseShellExecute = false; 
            NS3thread.StartInfo.RedirectStandardOutput = true;
            NS3thread.Start ();

            while (!NS3thread.StandardOutput.EndOfStream) {
                Console.WriteLine (NS3thread.StandardOutput.ReadLine ());
            }
        }
        
        public void Run5gScenarioInNS3()
        {
            string sceneName = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string logDirName = $"{sceneName}_{Stack_v2x_Tech.ToString()}_{timestamp}"; // /Users/nlyamin/WRK/Simulation/SIVERT2/lena3.3_freshBuild/
                                                                                        // string fullLogPath = Path.Combine(Application.dataPath, "..", logBasePath, logDirName);
            // string fullLogPath = Path.Combine("/Users/nlyamin/WRK/Simulation/SIVERT2/lena3.3_freshBuild/", logBasePath, logDirName);
            string fullLogPath = Path.Combine(pathToNs3ProjectRoot, logBasePath, logDirName);
            

            Directory.CreateDirectory(fullLogPath);

            string srcBinary = Path.Combine(pathToNs3Binary, ns3BinaryName);
            string srcScript = Path.Combine(pathToNs3ProjectRoot, shellScriptName);
            string dstBinary = Path.Combine(fullLogPath, ns3BinaryName);
            string dstScript = Path.Combine(fullLogPath, shellScriptName);

            // Copy files
            File.Copy(srcBinary, dstBinary, overwrite: true);
            File.Copy(srcScript, dstScript, overwrite: true);

            // Ensure both are executable
            RunBashCommand($"chmod +x \"{dstBinary}\" \"{dstScript}\"");

            // Launch via osascript (AppleScript)
            string absLogPath = Path.GetFullPath(fullLogPath);
            string arg = $"-e 'tell app \"Terminal\" to do script \"cd \\\"{absLogPath}\\\" && ./NS3DaemonForUnity.sh . {ns3BinaryName} > NS_Debug_Log_Activated_Watchers.log 2>&1\"'";

            Debug.Log("Starting NS3 thread with argument: " + arg);
            NS3thread = Process.Start("osascript", arg);
        }
        
        public void RunNrV2xScenarioInNS3()
        {
            string sceneName = UnityEngine.SceneManagement.SceneManager.GetActiveScene().name;
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string logDirName = $"{sceneName}_{Stack_v2x_Tech.ToString()}_{timestamp}";
            string fullLogPath = Path.Combine(Application.dataPath, "..", logBasePath, logDirName);

            Directory.CreateDirectory(fullLogPath);

            string srcBinary = Path.Combine(pathToNs3Binary, ns3BinaryName);
            string srcScript = Path.Combine(pathToNs3ProjectRoot, shellScriptName);
            string dstBinary = Path.Combine(fullLogPath, ns3BinaryName);
            string dstScript = Path.Combine(fullLogPath, shellScriptName);

            // Copy files
            File.Copy(srcBinary, dstBinary, overwrite: true);
            File.Copy(srcScript, dstScript, overwrite: true);

            // Ensure both are executable
            RunBashCommand($"chmod +x \"{dstBinary}\" \"{dstScript}\"");

            // Launch via osascript (AppleScript)
            string absLogPath = Path.GetFullPath(fullLogPath);
            string arg = $"-e 'tell app \"Terminal\" to do script \"cd \\\"{absLogPath}\\\" && ./NS3DaemonForUnity.sh . {ns3BinaryName} > NS_Debug_Log_Activated_Watchers.log 2>&1\"'";

            Debug.Log("Starting NS3 thread with argument: " + arg);
            NS3thread = Process.Start("osascript", arg);
        }

        private void RunBashCommand(string command)
        {
            var process = new Process
            {
                StartInfo = new ProcessStartInfo
                {
                    FileName = "/bin/bash",
                    Arguments = $"-c \"{command}\"",
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                    CreateNoWindow = true,
                }
            };
            process.Start();
            process.WaitForExit();
            process.Close();
        }

    }