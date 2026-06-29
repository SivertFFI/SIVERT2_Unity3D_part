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
using System.IO;
using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine.SceneManagement;
using Complex = System.Numerics.Complex;
using Debug = UnityEngine.Debug;
using System.Diagnostics;
using Unity.AI.Navigation;
using UnityEngine.AI;

using UnityEngine.UIElements;
using Assert = UnityEngine.Assertions.Assert;
using UnityEditor;
using UnityEngine.Serialization;

#region For parallel jobs
using System.Threading.Tasks;
using Unity.Burst;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using System.Linq;
using System.Collections;
using System.Runtime.CompilerServices;
using System.Diagnostics.Tracing;
using Unity.Loading;
using UnityEngine.Jobs;
using System.Globalization;
#endregion

public partial class Main : MonoBehaviour
{
    #region Compute Shader
    //public ChannelComputer channelComputer;
    public Material uEantennaRenderer;
    public Material bSantennaRenderer;
    public bool drawAntennPattern = false;
    [Range(1, 360)] public int drawEveryNthThetaPhi = 20; // Downsample antenna pattern visualization
    public bool enableEADF = false;
    public bool enableEADF23 = false;
    public GameObject antennaPatternPrefab;

    private NativeArray<float2> thetaPhi;
    #endregion
    

    #region Input Paramaters from Unity
    public int totalNumberOfSnapshots = 200;
    public GameObject mpc1Prefab, mpc2Prefab, mpc3Prefab, dmcPrefab, rbsPrefab;

    public GameObject trackingPrefab;
    private GameObject _rssiObject;

    public bool showArtefacts = false,
        showLoSPath = true,
        showFirstOrderPaths = true,
        showSecondOderPaths = true,
        showThirdOrderPaths = false;

    public TextAsset eadfFile;
    #endregion

    #region Application mode
    public string applicationMode;
    #endregion

    #region FPS
    public int sampleSize = 60;
    public float logInterval = 5f; // Interval in seconds for logging FPS to a file

    private float[] frameTimes;
    private int frameIndex;
    private bool filled;
    #endregion

    #region Spawning
    // MPC Points (Point, Normal, perpendicular ( (normal dot Up)/norm(normal dot Up) ) )
    private NativeArray<MPC.ScatterPoint> _mpc1PointsWithNormal, _mpc2PointsWithNormal, _mpc3PointsWithNormal;

    // Spawning
    private Vector3 _center, _size;
    private GameObject[] _reflectingObstacles, _spawningPlanes, _rbsSpawningAreas, _roads;
    //public NavMeshSurface navMeshSurface;
    public NavMeshSurface navMeshSurface;
    // lists for testing purposes
    private List<GameObject> mpc1GameObjects = new List<GameObject>();
    private List<GameObject> mpc2GameObjects = new List<GameObject>();
    private Renderer[] mpc1Renderers;
    private Renderer[] mpc2Renderers;
    private Material org1Material;
    private Material org2Material;

    // The ground reflection should have the same level of reflection as the first order reflection
    private float _groundReflectionCoefficient;

    #endregion

    #region data stuctures
    public struct EllipseParameters
    {
        // Ellipse definition is x^2/A^2 + y^2/B^2 = 1;
        public float A;
        public float B;
        // Rotation parameters: centerPoint and rotationMatrix
        public float2 CenterPoint;
        public float4 RotationMatrix;

        public EllipseParameters(float a, float b, float2 centerPoint, float4 rMtrx)
        {
            this.A = a;
            this.B = b;
            this.CenterPoint = centerPoint;
            this.RotationMatrix = rMtrx;
        }
    }
    #endregion

    #region Supporting classes
    private MPCFiltering _mpcFiltering = new MPCFiltering();
    private LUTDictionary _lutDictionary = new LUTDictionary();

    private NativeListFloatConversion nativeListFloatConversion = new NativeListFloatConversion();
    #endregion

    #region Channel Matrices

    private NativeArray<System.Numerics.Complex> _HLoS, _HnLoS, _totalH, _mrcH;
    #endregion

    #region NS3_AIP_Related

    public Sivert_API_GSCM_ECS ptrToSivertApi;

    #endregion

    #region Stucture of the Json files

    private ParametersRootObject _parameters;
    private ScenarioRootObject _scenario;

    [System.Serializable]
    public class RbsSpawningParameters
    {
        public float density;
        public string rbsFilePath, shiftFilePath, rbsParent, tag;
        public bool spawn;
        public int numAntennas;
    }

    [System.Serializable]
    public class UeSpawningParameters
    {
        public float density;
        public string ueFilePath, shiftFilePath, ueParent, tag;
        public bool spawn;
    }

    [System.Serializable]
    public class GeneralParameters
    {
        public string channelOutputPath, uePaths;
        public bool logChannelToFile;
        public LinkParameters linkParameters;

    }

    [System.Serializable]
    public class AntennaParameters
    {
        public float2 rbsPolarizationV;
        public float2 uePolarizationV;
        public float antennaSize;
        public string antennaType1, antennaType2;
    }

    [System.Serializable]
    public class TimeParameters
    {
        public float fixedTimestep;
        public float timeScale;
    }

    [System.Serializable]
    public class LinkParameters
    {
        public bool ueToUe;
        public bool ueToRbs;
        public bool rbsToRbs;
    }

    [System.Serializable]
    public class ScenarioRootObject
    {
        public RbsSpawningParameters rbsSpawningParameters;
        public UeSpawningParameters ueSpawningParameters;
        public GeneralParameters general;
        public AntennaParameters antennaParameters;
        public TimeParameters timeParameters;
    }

    [System.Serializable]
    public class MpcSpawningParameters
    {
        public float chiW1, chiW2, chiW3, distMpc1, distMpc2, distMpc3;
        public float minGMpc1, maxGMpc1, minGMpc2, maxGMpc2, minGMpc3, maxGMpc3;
        public string mpcParent, observationPointTag;
    }

    [System.Serializable]
    public class MpcPointCloudParameters
    {
        public string pathPointCloud, pathNormals, pathMesh;
        public int skipEveryNPoints;
        public float offset;
    }

    [System.Serializable]
    public class RadioParameters
    {
        public int numFft;
        public float nearbyMpcRadius;
        public float fixedUpdateTimeStep;
        public float thresholdDistance;
    }

    [System.Serializable]
    public class PhysicalParameters
    {
        public float powerInMilliWatts,
            noiseLevelInDb,
            speedofLight,
            costIraconfc,
            carrierFrequency,
            fSubcarriers,
            snrAvg,
            mrcSnrAvg,
            groundRefCoeff,
            firstOrderAttenuation,
            secondOrderAttenution,
            thirdOrderAttenuation;

        public bool useLoSPaths, useFirstOrderPaths, useSecondOrderPaths, useThirdOrderPaths;
        public float maxPathDistance, angularSpreadFirstOrder, angularSpreadSecondOrder, angularSpreadThirdOrder;
        public bool omniAntenna;
    }



    [System.Serializable]
    public class ParametersRootObject
    {
        public MpcPointCloudParameters mpcPointCloudParameters;
        public MpcSpawningParameters mpcSpawningParameters;
        public RadioParameters radioParameters;
        public PhysicalParameters physicalParameters;
    }

    [System.Serializable]
    public class PathObject
    {
        public String Path;
    }

    #endregion

    #region Entities
    private GameObject[] _ueArray, _rbsArray, _opsArray, _entityArray;
    private List<GameObject> _ueAntennaList, _rbsAntennaList;

    private NativeArray<Vector3> _opsCoordinates;

    private NativeArray<float3> _entityCoordinates, _entityForwardVect;
    private NativeArray<float3x3> _entityRotationMatrices;

    // For MIMO scenarios: UE Id, its antenna number, BS Id, its antenna number
    private NativeArray<int2> _ueIdAntennaNumber, _rbsIdAntennaNumber;

    #endregion

    #region Antennas and channels
    // Channels

    private NativeArray<int2> _channelLinks, _channelLinksLocalAntennaIndex;
    
    private NativeArray<float> _subCarriers, _inverseWavelengths;
    private Dictionary<GameObject, Transform> entityAntennaTransforms = new Dictionary<GameObject, Transform>();
    private Color[] linkColors;
    private TransformAccessArray _antennaTransforms;
    private float _antennSize;
    
    public GSCMstructECS GSCMrss;
    public GSCMspectrumChannel GSCMSpectrum;
    public bool UseMeasuredEMI = true;
    public string MeasuredEmiFileName = "EMI_5G_30.csv";
    public float[,] emiReadData;
    public float[,] emiWaveSectrumScaled;

    private NativeArray<AntennaData> _antennaDataArray;
    private NativeArray<AntennaAndRotationData> _antennaAndRotations;
    private NativeParallelHashMap<int2, float2> allUeAntennaPattern3D; // hasmap of all the UE antenna patterns
    private NativeParallelHashMap<int2, float2> allBsAntennaPattern3D; // hasmap of all the BS antenna patterns
    // Custom struct to store required transform data

    private AntennaType _antennaType1, _antennaType2;

    private NativeArray<float2> _eadfArray;
    private float2[] eadfArray;
    private int eadfRows, eadfCols;
    private int2 _eadfSize;
    private NativeArray<int> _mu1, _mu2;
    #endregion

    #region Ellipse parameters
    private NativeArray<EllipseParameters> _ellipsesSequential, _ellipseParallel;
    #endregion

    #region LUT

    private float costIraconFactor;
    private float lambdaOver4PI;
    private float attenuationThreshold;


    private NativeArray<Lut2Dictionary> _lut2;
    private NativeArray<Vector2Int> _mpc2Pointers;
    private int _lut2MaxVisibility;

    private NativeArray<Lut2Dictionary> lut3Half;
    private NativeArray<Vector2Int> mpc3PointersHalf;
    private NativeArray<LUT3> _lut3;
    private NativeArray<Vector2Int> _mpc3Pointers;
    private int _lut3MaxVisibility;

    private NativeArray<MPCFloat> mpc1Float;
    private NativeArray<MPCFloat> mpc2Float;
    private NativeArray<MPCFloat> mpc3Float;

    #endregion

    #region Other
    // Scene
    private Scene _currentScene;
    private int fixedUpdateCounter = 0; 
    private string _sceneName;

    // Files
    private string _dir, directoryPath;

    // Data logging
    private DataLogger _dataLogger;
    #endregion


    #region EADF parameters

    private NativeArray<float3> _antennaPattern;
    
    #endregion

    void OnEnable()
    {
        #region Read EADF file

        try
        {
            string filePath = "Assets/EADF/2x2_port_eadf.csv";
            string[] lines = File.ReadAllLines(filePath);

            // Read dimensions from the first line
            string[] dimensions = lines[0].Split(',');
            eadfRows = int.Parse(dimensions[0]);
            eadfCols = int.Parse(dimensions[1]);
            // Input to IJobParallelFor
            _eadfSize = new int2(eadfRows, eadfCols);
            
            _mu1 = new NativeArray<int>(eadfRows, Allocator.Persistent);
            _mu2 = new NativeArray<int>(eadfCols, Allocator.Persistent);
            
            int startMu1 = Mathf.FloorToInt(eadfRows / 2.0f);
            for (int i1 = 0; i1 < eadfRows; i1++)
            {
                _mu1[i1] = i1 - startMu1; 
            }
            
            int startMu2 = Mathf.FloorToInt(eadfCols / 2.0f);
            for (int i2 = 0; i2 < eadfCols; i2++)
            {
                _mu2[i2] = i2 - startMu2;
            }
            
            

            Debug.Log($"Original matrix dimensions: {eadfRows} x {eadfCols}");

            int totalElements = eadfRows * eadfCols;
            eadfArray = new float2[totalElements];
            
            
            // Start reading data from the second line
            for (int i = 1; i < lines.Length; i++)
            {
                string[] values = lines[i].Split(',');
                float real = float.Parse(values[0], CultureInfo.InvariantCulture);
                float imag = float.Parse(values[1], CultureInfo.InvariantCulture);
                eadfArray[i - 1] = new float2(real, imag);
            }
            
            // Input to IJobParallelFor
            _eadfArray = new NativeArray<float2>(eadfArray, Allocator.Persistent);
            
            Debug.Log($"EADF loaded. Total elements: {_eadfArray.Length}");
            Debug.Log($"First element: {_eadfArray[0]}");
            Debug.Log($"Last element: {_eadfArray[^1]}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Error reading file: {e.Message}\n{e.StackTrace}");
        }

        

        #endregion

        #region  Name;
        frameTimes = new float[sampleSize];
        #endregion

        #region Scene
        // Name of the scene
        _currentScene = SceneManager.GetActiveScene();
        _sceneName = _currentScene.name;

        // Path
        _dir = "Assets/" + _sceneName + "/";

        directoryPath = Application.dataPath + "/" + SceneManager.GetActiveScene().name;

        Debug.Log($"Current directory: {Environment.CurrentDirectory}");

        if (!System.IO.Directory.Exists(_dir))
        {
            System.IO.Directory.CreateDirectory(_dir);
        }
        #endregion

        #region Retreive Parameters from Json files
        // Parameters
        string parameterJsonFilePath =
            System.IO.Path.Combine(Environment.CurrentDirectory, _dir + "spawning_parameters.json");

        Debug.Log($"parameterJsonFilePath: {parameterJsonFilePath}");
        if (File.Exists(parameterJsonFilePath))
        {
            string paramaterJsonString = File.ReadAllText(parameterJsonFilePath);

            // Create your object and sub-objects with default values
            _parameters = new ParametersRootObject
            {
                mpcPointCloudParameters = new MpcPointCloudParameters
                {
                    pathPointCloud = null,
                    pathNormals = null,
                    pathMesh = null,
                    skipEveryNPoints = 200,
                    offset = 0.1f,
                },
                mpcSpawningParameters = new MpcSpawningParameters
                {
                    chiW1 = 0.8f,
                    chiW2 = 0.2f,
                    chiW3 = 0.2f,
                    distMpc1 = 3f,
                    distMpc2 = 3f,
                    distMpc3 = 3f,
                    mpcParent = "MPCs",
                    observationPointTag = "OPs",
                    minGMpc1 = -65,
                    maxGMpc1 = -48,
                    minGMpc2 = -70,
                    maxGMpc2 = -59,
                    minGMpc3 = -75,
                    maxGMpc3 = -65
                },
                radioParameters = new RadioParameters
                {
                    numFft = 512,
                    nearbyMpcRadius = 300.0f,
                    fixedUpdateTimeStep = Time.fixedDeltaTime,
                    thresholdDistance = 6000.0f // works for 3.75 GHz
                },
                physicalParameters = new PhysicalParameters
                {
                    powerInMilliWatts = 0.1f,
                    noiseLevelInDb = -60.0f,
                    speedofLight = 299792458.0f, // m/s
                    costIraconfc = 5.7e9f,
                    carrierFrequency = (float)(3.75 * Mathf.Pow(10, 9)), // GHz
                    fSubcarriers = 500000,
                    snrAvg = 0,
                    mrcSnrAvg = 0,
                    omniAntenna = false,
                    groundRefCoeff = 0.7f,
                    useLoSPaths = true,
                    useFirstOrderPaths = true,
                    useSecondOrderPaths = true,
                    useThirdOrderPaths = true,
                    maxPathDistance = 600,
                    angularSpreadFirstOrder = 0.0873f,
                    angularSpreadSecondOrder = 0.0873f,
                    angularSpreadThirdOrder = 0.0873f,
                    firstOrderAttenuation = 0.001f,
                    secondOrderAttenution = 0.001f,
                    thirdOrderAttenuation = 0.001f,
                }
            };

            // Parse JSON into the object, replacing fields that are in the JSON
            JsonUtility.FromJsonOverwrite(paramaterJsonString, _parameters);

            Debug.Log(JsonUtility.ToJson(_parameters, true)); // print the object content after parsing
        }
        else
        {
            Debug.LogError("Cannot find file at path: " + parameterJsonFilePath);
        }

        #region Calculate the ground reflection coefficient
        _groundReflectionCoefficient = math.pow(10, _parameters.mpcSpawningParameters.maxGMpc1 / 20);
        #endregion

        # region Check the Communication scenario in Sivert API: 5G or NR-V2X
        bool ueToUeFlagFromSivertApi = false;
        try
        {
            if (ptrToSivertApi)
            {
                if (ptrToSivertApi.Stack_v2x_Tech == Sivert_API_GSCM_ECS.Scenario.cellular5G)
                {
                    ueToUeFlagFromSivertApi = false;
                }
                else if (ptrToSivertApi.Stack_v2x_Tech == Sivert_API_GSCM_ECS.Scenario.NRV2X)
                {
                    ueToUeFlagFromSivertApi = true;
                }
            }
            else
            {
                GameObject _goApi = GameObject.Find("ApiToNs3");
                if (_goApi != null)
                {
                    ptrToSivertApi = _goApi.GetComponent<Sivert_API_GSCM_ECS>();
                }
                if (ptrToSivertApi)
                {
                    if (ptrToSivertApi.Stack_v2x_Tech == Sivert_API_GSCM_ECS.Scenario.cellular5G)
                    {
                        ueToUeFlagFromSivertApi = false;
                    }
                    else if (ptrToSivertApi.Stack_v2x_Tech == Sivert_API_GSCM_ECS.Scenario.NRV2X)
                    {
                        ueToUeFlagFromSivertApi = true;
                    }
                }
            }

        }
        catch (Exception e)
        {
            Debug.LogWarning("No pointer to SIVERT API found - check the scripts for reference to the API GO. Needed to switch between 5G V2N and NR-V2X V2V scenarios.");
        }
        #endregion
        
        
        SaveParameters(_parameters, System.IO.Path.Combine(Environment.CurrentDirectory, _dir + "used_parameters.json"));
        // Scenario
        string scenarioJsonFilePath = System.IO.Path.Combine(Environment.CurrentDirectory, _dir + "scenario.json");
        if (File.Exists(scenarioJsonFilePath))
        {
            string scenarioJsonString = File.ReadAllText(scenarioJsonFilePath);

            // Create your object and sub-objects with default values
            _scenario = new ScenarioRootObject
            {
                rbsSpawningParameters = new RbsSpawningParameters()
                {
                    density = 6.0f,
                    rbsFilePath = "rbs.csv",
                    spawn = false,
                    shiftFilePath = "rbs_shift.csv",
                    rbsParent = "Infrastructure",
                    tag = "RBS",
                    numAntennas = 1,
                },
                ueSpawningParameters = new UeSpawningParameters()
                {
                    density = 1.0f,
                    ueFilePath = "ue.csv",
                    spawn = false,
                    shiftFilePath = "ue_shift.csv",
                    ueParent = "Entities",
                    tag = "UE",
                },
                general = new GeneralParameters()
                {
                    channelOutputPath = "H_Output/",
                    logChannelToFile = true,
                    uePaths = "Paths/",
                    linkParameters = new LinkParameters()
                    {
                        ueToUe = false,
                        ueToRbs = true,
                        rbsToRbs = false,
                    },
                },
                antennaParameters = new AntennaParameters()
                {
                    rbsPolarizationV = new float2(0, 1),
                    uePolarizationV = new float2(0, 1),
                    antennaSize = 1f,
                    antennaType1 = "Omni",
                    antennaType2 = "Omni",
                },
                timeParameters = new TimeParameters()
                {
                    fixedTimestep = 0.01f,
                    timeScale = 1.0f,
                }
            };
            // Parse JSON into the object, replacing fields that are in the JSON
            JsonUtility.FromJsonOverwrite(scenarioJsonString, _scenario);
        }
        else
        {
            Debug.LogError("Cannot find file at path: " + scenarioJsonFilePath);
        }

        if (ueToUeFlagFromSivertApi) // set to ueToUe if NR-V2X is chosen in NS3Api
        {
            _scenario.general.linkParameters.ueToUe = true;
            _scenario.general.linkParameters.ueToRbs = false;
        }
        #endregion



        #region UEs and RBSs
        // For visualization of antenna polarizations and orientations
        _antennSize = _scenario.antennaParameters.antennaSize;

        // If group does not exist
        GameObject rbsParent = GameObject.Find(_scenario.rbsSpawningParameters.rbsParent);
        if (rbsParent == null)
        {
            rbsParent = new GameObject(_scenario.rbsSpawningParameters.rbsParent);
        }

        // If group does not exist
        GameObject ueParent = GameObject.Find(_scenario.ueSpawningParameters.ueParent);
        if (ueParent == null)
        {
            ueParent = new GameObject(_scenario.ueSpawningParameters.ueParent);
        }

        List<Vector3> relativePositionsList = new List<Vector3>();
        List<string> spawnVolumeNamesList = new List<string>();


        string rbsFilePath = System.IO.Path.Combine(Environment.CurrentDirectory, _dir + _scenario.rbsSpawningParameters.rbsFilePath);
        Debug.Log("rbsFilePath: " + rbsFilePath);
        if (File.Exists(rbsFilePath))
        {
            InstantiateGameObjectsFromCSV(
                rbsFilePath,
                _dir + _scenario.rbsSpawningParameters.shiftFilePath,
                rbsPrefab,
                new Vector3Int(0, 2, 1),
                ref relativePositionsList,
                ref spawnVolumeNamesList,
                rbsParent,
                _scenario.rbsSpawningParameters.tag
            );
        }

        
        // Initialize the list to store antennas

        #region UeAntennaPatterns
        
        GameObject[] ueObjects = GameObject.FindGameObjectsWithTag(_scenario.ueSpawningParameters.tag);
        try
        {
            List<TextAsset> antennaEADFsUe = new List<TextAsset>();
            List<string> antUes = new List<string>();
            foreach (GameObject ue in ueObjects)
            {
                foreach (Transform child in ue.transform)
                {
                    
                    if (child.name.Contains("Antenna"))
                    {
                        antennaEADFsUe.Add(child.GetComponent<RadiationPattern3D>()._antennaEADFfile);
                        
                        // Get Y-axis rotation and format it for antenna naming
                        // float yRotation = child.transform.eulerAngles.y;
                        float yRotation = child.transform.localRotation.eulerAngles.y; // Use eulerAngles for rotation
                        
                        // EulerAngles gives 0-360°, Inspector shows -180 to +180°
                        // For naming, we'll use rounded integer degrees (0-360 range)
                        int roundedRotation = Mathf.RoundToInt(yRotation);
                        string rotationText = $"_Y{roundedRotation:000}"; // Zero-padded: _Y000, _Y090, _Y180
                        
                        string antennaNameWithRotation = child.GetComponent<RadiationPattern3D>()._antennaEADFfile.name + rotationText;
                        antUes.Add(antennaNameWithRotation);
                    }
                }
            }

            allUeAntennaPattern3D = new NativeParallelHashMap<int2, float2>(360 * 181 * antennaEADFsUe.Count, Allocator.Persistent);
            string antBinNameString = antUes[0] + "_" + antUes[1];
            if (File.Exists(antBinNameString)) 
            {
                LoadBinIntoExistingNativeParallelHashMap(antBinNameString, ref allUeAntennaPattern3D);
                Debug.LogWarning($"Read antennas from {antBinNameString} binary file");
            }
            else
            {
                int thetaNumRec = 181;
                int phiNumRec = 360;

                NativeArray<int2> thetaPhiReconstruction = new NativeArray<int2>(thetaNumRec * phiNumRec, Allocator.TempJob);
                for (int i = 0; i < thetaNumRec; i++)
                {
                    for (int j = 0; j < phiNumRec; j++)
                    {
                        thetaPhiReconstruction[i * phiNumRec + j] = new int2(i, j);
                    }
                }

                for (int i = 0; i < antennaEADFsUe.Count; i++)
                {
                    var arr = Read3DEADFfile(antennaEADFsUe[i]);
                    Debug.LogWarning($"File {antennaEADFsUe[i].name}: rows={eadfRows}, cols={eadfCols}, expected={eadfRows*eadfCols}, got={arr.Length}");
                    Debug.LogWarning($"Mu1 len={_mu1.Length}, Mu2 len={_mu2.Length}");
                    Stopwatch stopwatch_inner = Stopwatch.StartNew();


                    NativeArray<float2> sliceAntenna = new NativeArray<float2>(Read3DEADFfile(antennaEADFsUe[i]), Allocator.TempJob);

                    EADF3DReconstruct eadfReconstruct = new EADF3DReconstruct()
                    {
                        AntennaID = i,
                        ThetaPhi = thetaPhiReconstruction,
                        Mu1 = _mu1,
                        Mu2 = _mu2,
                        EADF = sliceAntenna,

                        // Output
                        AntennaPattern = allUeAntennaPattern3D.AsParallelWriter()
                    };
                    JobHandle eadfReconstructJobHandle = eadfReconstruct.Schedule(thetaNumRec * phiNumRec, 64);
                    eadfReconstructJobHandle.Complete();
                    sliceAntenna.Dispose();

                    stopwatch_inner.Stop();
                    Debug.LogWarning($"3DEADF Job for Antenna: {i} Execution Time: {stopwatch_inner.ElapsedTicks} ticks ({stopwatch_inner.ElapsedMilliseconds} ms)");
                }

                thetaPhiReconstruction.Dispose();

                SaveNativeParallelHashMapToBinary(allUeAntennaPattern3D, antBinNameString);
                Debug.LogWarning($"Saving new set of the UE antennas as {antBinNameString} - double check if the antennas are correctly set");
            }

        }
        catch (Exception e)
        {
            Debug.LogError($"Error reading UE antenna file: {e.Message}\n{e.StackTrace}");
        }

            #endregion

        #region BsAntennaPatterns
        GameObject[] bsObjects = GameObject.FindGameObjectsWithTag(_scenario.rbsSpawningParameters.tag);
        try
        {
            List<TextAsset> antennaEADFsBs = new List<TextAsset>();
            List<string> antBss = new List<string>();
            foreach (GameObject bs in bsObjects)
            {
                // Find all child objects that contain "Antenna" in their name
                foreach (Transform child in bs.transform)
                {
                    
                    if (child.name.Contains("Antenna"))
                    {
                        antennaEADFsBs.Add(child.GetComponent<RadiationPattern3D>()._antennaEADFfile);
                        antBss.Add(child.GetComponent<RadiationPattern3D>()._antennaEADFfile.name);
                    }
                }
            }

            allBsAntennaPattern3D = new NativeParallelHashMap<int2, float2>(360 * 181 * antennaEADFsBs.Count, Allocator.Persistent);
            string antBin = "";
            if (antennaEADFsBs.Count == 2)
            {
                string rootPath = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                string fullPath = rootPath + "/gNbSectorTwoPortAntenna";
                if (File.Exists(fullPath))
                {
                    Debug.Log("1-sector antenna pattern configuration exists in: " + fullPath);
                    antBin = "gNbSectorTwoPortAntenna"; // static ports pattern configuration for 1-sector gNB
                }
                else
                {
                    antBin = "NaN";
                    Debug.LogWarning($"1-sector antenna pattern configuration not found in {fullPath} , recreating from pattern files");
                }
            }
            else if (antennaEADFsBs.Count == 6)
            {
                string rootPath = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                string fullPath = rootPath + "/gNbSectorSixPortAntenna";
                if (File.Exists(fullPath))
                {
                    Debug.Log("3-sector antenna pattern configuration exists in: " + fullPath);
                    antBin = "gNbSectorSixPortAntenna"; // static ports pattern configuration for 3-sector gNB
                }
                else
                {
                    antBin = "NaN";
                    Debug.LogWarning($"3-sector antenna pattern configuration not found in {fullPath} , recreating from pattern files");
                }
            }
            else if (antennaEADFsBs.Count == 12)
            {
                string rootPath = Path.GetFullPath(Path.Combine(Application.dataPath, ".."));
                string fullPath = rootPath + "/gNbSectorTwelvePortAntennaTwoGnbs";
                if (File.Exists(fullPath))
                {
                    Debug.Log("3-sector antenna pattern for 2 gNBs configuration exists in: " + fullPath);
                    antBin = "gNbSectorTwelvePortAntennaTwoGnbs"; // static ports pattern configuration for 3-sector gNB
                }
                else
                {
                    antBin = "NaN";
                    Debug.LogWarning($"3-sector antenna pattern for 2 gNBs configuration not found in {fullPath} , recreating from pattern files");
                }
            }
            else
            {
                antBin = "NaN";
                Debug.LogWarning($"Antennna patterns for N_gNb antenna configuration = {antennaEADFsBs.Count} antennas might not be supported");
            }
            
            if (antBin != "NaN")
            {
                LoadBinIntoExistingNativeParallelHashMap(antBin, ref allBsAntennaPattern3D);
                Debug.LogWarning($"Read antennas from {antBin} binary file");
            }
            else
            {
                int thetaNumRec = 181;
                int phiNumRec = 360;
            
                NativeArray<int2> thetaPhiReconstruction = new NativeArray<int2>(thetaNumRec * phiNumRec, Allocator.TempJob);
                for (int i = 0; i < thetaNumRec; i++)
                {
                    for (int j = 0; j < phiNumRec; j++)
                    {
                        thetaPhiReconstruction[i * phiNumRec + j] = new int2(i, j); 
                    }
                }
            
                for (int i=0; i<antennaEADFsBs.Count; i++)
                {
                    Stopwatch stopwatch_inner = Stopwatch.StartNew();
                
                    NativeArray<float2> sliceAntenna = new NativeArray<float2>(Read3DEADFfile(antennaEADFsBs[i]), Allocator.TempJob);
                
                    EADF3DReconstruct eadfReconstruct = new EADF3DReconstruct()
                    {
                        AntennaID = i,
                        ThetaPhi = thetaPhiReconstruction,
                        Mu1 = _mu1,
                        Mu2 = _mu2,
                        EADF = sliceAntenna,
            
                        // Output
                        AntennaPattern = allBsAntennaPattern3D.AsParallelWriter()
                    };
                    JobHandle eadfReconstructJobHandle = eadfReconstruct.Schedule(thetaNumRec*phiNumRec, 64);
                    eadfReconstructJobHandle.Complete();
                    sliceAntenna.Dispose();
                
                    stopwatch_inner.Stop();
                    Debug.LogWarning($"3DEADF Job for Antenna: {i} Execution Time: {stopwatch_inner.ElapsedTicks} ticks ({stopwatch_inner.ElapsedMilliseconds} ms)");
                }

                thetaPhiReconstruction.Dispose();

                if (antennaEADFsBs.Count == 6)
                {
                    SaveNativeParallelHashMapToBinary(allBsAntennaPattern3D, "gNbSectorSixPortAntenna");
                    Debug.Log("Saving NAtive parallel BS antenna data for gNbSectorSixPortAntenna");
                }
                else if (antennaEADFsBs.Count == 12)
                {
                    SaveNativeParallelHashMapToBinary(allBsAntennaPattern3D, "gNbSectorTwelvePortAntennaTwoGnbs");
                    Debug.Log("Saving NAtive parallel BS antenna data for gNbSectorTwelvePortAntennaTwoGnbs");
                }
                else if (antennaEADFsBs.Count == 2)
                {
                    SaveNativeParallelHashMapToBinary(allBsAntennaPattern3D, "gNbSectorTwoPortAntenna");
                    Debug.Log("Saving NAtive parallel BS antenna data for gNbSectorTwoPortAntenna");
                }
                else
                {
                    Debug.LogError("Check the antenna patterns for gNB - might be inconsistent. Saving Native antenna hashmap with the name of the first BS antenna.");
                    SaveNativeParallelHashMapToBinary(allBsAntennaPattern3D, antennaEADFsBs[0].name);
                }
                
                
            }

        }
        catch (Exception e)
        {
            Debug.LogError($"Error reading BS antenna file: {e.Message}\n{e.StackTrace}");
        }

        #endregion
        
        _ueAntennaList = new List<GameObject>();
        //if (enableEADF)
        //{
        // Reduce the resolution for faster visualization
        int thetaNum = 180;
        int thetaStep = 180 / thetaNum;
        int phiRes = 1;
        int phiNum = 360 / phiRes;
        // Sampling controls for antenna pattern visualization
        int stepUE = Mathf.Max(1, drawEveryNthThetaPhi);
        int stepBS = Mathf.Max(1, drawEveryNthThetaPhi);
        int sampleIndexUE = 0; // reset per-antenna below before drawing
        int sampleIndexBS = 0; // reset per-antenna below before drawing
        
        // Full 360 degrees antenna visualization (computationally heavy - creates 180x360 = 64800 GO per antenna)
        // int thetaNum = 180;
        // int phiNum = 360;

        thetaPhi = new NativeArray<float2>(thetaNum * phiNum, Allocator.Persistent);
        for (int i = 0; i < thetaNum; i++)
        {
            for (int j = 0; j < phiNum; j++)
            {
                float iTheta = 90.0f - thetaNum/2 + i;
                float iPhi = j*phiRes;
                thetaPhi[i * phiNum + j] = new float2(iTheta, iPhi) * math.PI / 180.0f;
            }
        }

        float3x3  rotationMatrix = new float3x3(new float3(1,0,0), new float3(0,1,0), new float3(0,0,1));
            
        _antennaPattern = new NativeArray<float3>(thetaNum * phiNum, Allocator.Persistent);

        EADFDrawJob eadfDrawJob = new EADFDrawJob()
        {
            RotationMatrix = rotationMatrix,
            ThetaPhi = thetaPhi,
            Mu1 = _mu1,
            Mu2 = _mu2,
            EADF = _eadfArray,

            // Output
            Results = _antennaPattern
        };
        JobHandle eadfDrawJobHandle = eadfDrawJob.Schedule(thetaNum * phiNum, 64);
        eadfDrawJobHandle.Complete();


        NativeArray<float2> thetaPhiDrawing = new NativeArray<float2>(thetaNum * phiNum, Allocator.TempJob);
        for (int i = 0; i < thetaNum; i++)
        {
            for (int j = 0; j < phiNum; j++)
            {
                // float iTheta = 85.0f + (float)i / 1;
                // float iPhi = j*10;
                float iTheta = 90.0f - thetaNum/2 + i;
                float iPhi = j;
                thetaPhiDrawing[i * phiNum + j] = new float2(iTheta, iPhi) * math.PI / 180.0f;
            }
        }

        #region UeAntennaPatternsDrawing

        List<int> ueIdAntennaNumber = new List<int>();
        foreach (GameObject ue in ueObjects)
        {
            int ueAntennaNumber = 0;
            // Find all child objects that contain "Antenna" in their name
            foreach (Transform child in ue.transform)
            {
                if (child.name.Contains("Antenna"))
                {
                    _ueAntennaList.Add(child.gameObject);
                    if (drawAntennPattern)
                    {
                        Vector3 oldPosition = child.position;
                        float max_gain = 0;
                        // Reset sampling counter for this antenna visualization
                        sampleIndexUE = 0;
                        foreach (var thetaPhi in thetaPhiDrawing)
                        {
                            // Only draw/instantiate for each N-th sample
                            if ((sampleIndexUE++ % stepUE) != 0) { continue; }
                            int2 key = new int2((int)(180 * thetaPhi.x / math.PI), (int)(180 * thetaPhi.y / math.PI));
                            allUeAntennaPattern3D.TryGetValue(key + new int2(10000 * ueAntennaNumber), out float2 gain);
                            float X = math.sin(thetaPhi.x) * math.cos(thetaPhi.y); 
                            float Y = math.sin(thetaPhi.x) * math.sin(thetaPhi.y); 
                            float Z = math.cos(thetaPhi.x);  
                            Vector3 eadfWithDirection = math.length(gain)*0.5f *
                                                        (X * rotationMatrix.c2 - Y * rotationMatrix.c0 +
                                                         Z * rotationMatrix.c1);
                            max_gain = Mathf.Max(math.length(gain), max_gain);
                            GameObject vertexObject = GameObject.Instantiate(antennaPatternPrefab);
                            vertexObject.transform.localScale =
                                new Vector3(0.01f, 0.01f, 0.01f); // Set the scale to be smaller
                            vertexObject.transform.SetParent(child);
                            eadfWithDirection.y += 1f; // slight offset to avoid z-fighting with the UE model
                            vertexObject.transform.localPosition = eadfWithDirection;
                            // Name each vertex
                            vertexObject.name = $"Vertex_{thetaPhi}";
                            vertexObject.GetComponent<Renderer>().material = uEantennaRenderer;

                            // Remove the collider component
                            Destroy(vertexObject.GetComponent<Collider>());
                            Debug.DrawLine(oldPosition, eadfWithDirection, Color.red);
                            oldPosition = eadfWithDirection;
                        }
                        Debug.LogWarning($"Max gain for antenna {ueAntennaNumber} is {max_gain}");
                        ueAntennaNumber++;
                        
                    }
                }
                if (child.name.Contains("RSSI"))
                {
                    _rssiObject = child.gameObject;
                }
            }
            ueIdAntennaNumber.Add(ueAntennaNumber);

        }

        #endregion
        
        #region BsAntennaPatternsDrawing
        
        // Initialize the list to store RBS antennas
        _rbsAntennaList = new List<GameObject>();

        List<int> BsIdAntennaNumber = new List<int>();
        foreach (GameObject bs in bsObjects)
        {
            int bsAntennaNumber = 0;
            // Find all child objects that contain "Antenna" in their name
            foreach (Transform child in bs.transform)
            {
                if (child.name.Contains("Antenna"))
                {
                    _rbsAntennaList.Add(child.gameObject);
                    if (drawAntennPattern)
                    {
                        Vector3 oldPosition = child.position;
                        float max_gain = 0;
                        // Reset sampling counter for this antenna visualization
                        sampleIndexBS = 0;
                        foreach (var thetaPhi in thetaPhiDrawing)
                        {
                            // Only draw/instantiate for each N-th sample
                            if ((sampleIndexBS++ % stepBS) != 0) { continue; }
                            int2 key = new int2((int)(180 * thetaPhi.x / math.PI), (int)(180 * thetaPhi.y / math.PI));
                            allBsAntennaPattern3D.TryGetValue(key + new int2(10000 * bsAntennaNumber), out float2 gain);
                            float X = math.sin(thetaPhi.x) * math.cos(thetaPhi.y); 
                            float Y = math.sin(thetaPhi.x) * math.sin(thetaPhi.y); 
                            float Z = math.cos(thetaPhi.x); 
                            Vector3 eadfWithDirection = math.length(gain)*0.5f *
                                                        (X * rotationMatrix.c2 - Y * rotationMatrix.c0 +
                                                         Z * rotationMatrix.c1);
                            max_gain = Mathf.Max(math.length(gain), max_gain);
                            GameObject vertexObject = GameObject.Instantiate(antennaPatternPrefab);
                            vertexObject.transform.localScale =
                                new Vector3(0.1f, 0.1f, 0.1f); // Set the scale to be smaller
                            vertexObject.transform.SetParent(child);
                            vertexObject.transform.localPosition = eadfWithDirection;
                            // Name each vertex
                            vertexObject.name = $"Vertex_{thetaPhi}";
                            vertexObject.GetComponent<Renderer>().material = bSantennaRenderer;

                            // Remove the collider component
                            Destroy(vertexObject.GetComponent<Collider>());
                            Debug.DrawLine(oldPosition, eadfWithDirection, Color.red);
                            oldPosition = eadfWithDirection;
                        }
                        Debug.LogWarning($"Max gain for antenna {bsAntennaNumber} is {max_gain}");
                        bsAntennaNumber++;
                        
                    }
                }
                if (child.name.Contains("RSSI"))
                {
                    _rssiObject = child.gameObject;
                }
            }
            ueIdAntennaNumber.Add(bsAntennaNumber);
            

        }
        thetaPhiDrawing.Dispose();

        #endregion
        
        if (_rssiObject == null)
        {
            // Instantiate a plane
            GameObject plane = GameObject.CreatePrimitive(PrimitiveType.Plane);

            // Set the parent of the plane to ueObjects[0]
            plane.transform.SetParent(ueObjects[0].transform);

            // Optionally, set the local position of the plane
            plane.transform.localPosition = new Vector3(0, -1.0f, 0);
            // Change the size of the plane
            plane.transform.localScale = new Vector3(.3f, 1.0f, .3f); // Example scale

            // Assign the plane to _rssiObject
            _rssiObject = plane;

        }

        // Convert the list to an array and assign it to _ueArray
        _ueArray = _ueAntennaList.ToArray();

        // Find all RBSs using the tag
        GameObject[] rbsObjects = GameObject.FindGameObjectsWithTag(_scenario.rbsSpawningParameters.tag);

        

        // Iterate through each RBS to find its antennas

        List<int> rbsIdAntennaNumber = new List<int>();
        foreach (GameObject rbs in rbsObjects)
        {
            int rbsAntennaNumber = 0;
            // Find all child objects that contain "Antenna" in their name
            foreach (Transform rbsChild in rbs.transform)
            {
                if (rbsChild.name.Contains("Antenna"))
                {
                    // _rbsAntennaList.Add(rbsChild.gameObject);
                    rbsAntennaNumber++;
                }
            }
            rbsIdAntennaNumber.Add(rbsAntennaNumber);

        }

        // Convert the list to an array and assign it to _rbsArray
        _rbsArray = _rbsAntennaList.ToArray();

        _entityArray = _rbsArray.Concat(_ueArray).ToArray();
        Debug.Log($"There are {_entityArray.Length} entities.");
        // Initialize the TransformAccessArray with a capacity
        _antennaTransforms = new TransformAccessArray(_entityArray.Length);
        _antennaDataArray = new NativeArray<AntennaData>(_entityArray.Length, Allocator.Persistent);
        _antennaAndRotations = new NativeArray<AntennaAndRotationData>(_entityArray.Length, Allocator.Persistent);

        foreach (var entity in _entityArray)
        {
            Transform antennaTransform = entity.transform;
            entityAntennaTransforms[entity] = antennaTransform;

            // For efficient tracking of the antenna transforms
            _antennaTransforms.Add(antennaTransform);
        }


        for (int i = 0; i < _rbsArray.Length; i++)
        {
            relativePositionsList.Add(new Vector3(float.NaN, float.NaN, float.NaN));
            spawnVolumeNamesList.Add("None");
        }
        _entityCoordinates = new NativeArray<float3>(_entityArray.Length, Allocator.Persistent);
        _entityRotationMatrices = new NativeArray<float3x3>(_entityArray.Length, Allocator.Persistent);
        _entityForwardVect = new NativeArray<float3>(_entityArray.Length, Allocator.Persistent);

        #endregion

        #region Observation Points
        _opsArray = GameObject.FindGameObjectsWithTag(_parameters.mpcSpawningParameters.observationPointTag);
        _opsCoordinates = new NativeArray<Vector3>(_opsArray.Length, Allocator.Persistent);
        for (int i = 0; i < _opsArray.Length; i++)
        {
            _opsCoordinates[i] = _opsArray[i].transform.position;
        }
        #endregion

        #region Antennas and Channels



        #region Simulation parameters for communication
        // General
        _subCarriers = new NativeArray<float>(_parameters.radioParameters.numFft, Allocator.Persistent);
        _inverseWavelengths = new NativeArray<float>(_parameters.radioParameters.numFft, Allocator.Persistent);
        // Define attenuation threshold

        costIraconFactor = 20 * math.log10(4.0f * math.PI * _parameters.physicalParameters.costIraconfc / _parameters.physicalParameters.speedofLight);


        lambdaOver4PI = _parameters.physicalParameters.speedofLight /
                        (_parameters.physicalParameters.carrierFrequency * 4.0f * Mathf.PI);
        attenuationThreshold = lambdaOver4PI / _parameters.radioParameters.thresholdDistance; //lambdaOver4PI/1000000.0f;// we define threshold for 1000 km.
        for (int i = 0; i < _parameters.radioParameters.numFft; i++)
        {
            _subCarriers[i] = _parameters.physicalParameters.carrierFrequency +
                              _parameters.physicalParameters.fSubcarriers * i;
            _inverseWavelengths[i] = _subCarriers[i] / _parameters.physicalParameters.speedofLight;
        }

        // Channel 
        Debug.Log("Number of subcarriers: " + _parameters.radioParameters.numFft + "; carrier frequency: " +
                  _parameters.physicalParameters.carrierFrequency + "; subcarrier spacing: " +
                  _parameters.physicalParameters.fSubcarriers);

        float4 asd = new float4(1, 0, 0, 1);
        #endregion

        #region MIMO channel links
        _antennaType1 = (AntennaType)Enum.Parse(typeof(AntennaType), _scenario.antennaParameters.antennaType1); ;
        _antennaType2 = (AntennaType)Enum.Parse(typeof(AntennaType), _scenario.antennaParameters.antennaType2);
        // MIMO channel links
        List<int2> mimoChannelLinks = new List<int2>();
        List<int2> entityAntennasInLinks = new List<int2>();
        if (_scenario.general.linkParameters.ueToRbs)
        {
            for (int ueIndex = 0; ueIndex < _ueArray.Length; ueIndex++)
            {
                int entity1Index = _rbsArray.Length + ueIndex;

                for (int rbsIndex = 0; rbsIndex < _rbsArray.Length; rbsIndex++)
                {
                    int entity2Index = rbsIndex;
                    mimoChannelLinks.Add(new int2(entity1Index, entity2Index));
                    entityAntennasInLinks.Add(new int2(ueIndex, rbsIndex));
                }
            }
        }

        // UE to UE
        if (_scenario.general.linkParameters.ueToUe)
        {
            for (int ue1 = 0; ue1 < ueIdAntennaNumber.Count; ue1++)
            {
                int ue1Jump = ueIdAntennaNumber.Take(ue1).Sum();
                for (int i1 = 0; i1 < ueIdAntennaNumber[ue1]; i1++)
                {
                    int entity1Index = _rbsArray.Length + ue1Jump + i1;

                    for (int ue2 = ue1 + 1; ue2 < ueIdAntennaNumber.Count; ue2++)
                    {
                        int ue2Jump = ueIdAntennaNumber.Take(ue2).Sum();
                        for (int i2 = 0; i2 < ueIdAntennaNumber[ue2]; i2++)
                        {
                            int entity2Index = _rbsArray.Length + ue2Jump + i2;
                            mimoChannelLinks.Add(new int2(entity1Index, entity2Index));
                            entityAntennasInLinks.Add(new int2(ue1Jump + i1, ue2Jump + i2));
                        }
                    }

                }
            }
        }

        // RBS to RBS
        if (_scenario.general.linkParameters.rbsToRbs)
        {
            for (int rbs1 = 0; rbs1 < rbsIdAntennaNumber.Count; rbs1++)
            {
                int rbs1Jump = rbsIdAntennaNumber.Take(rbs1).Sum();
                for (int i1 = 0; i1 < rbsIdAntennaNumber[rbs1]; i1++)
                {
                    int entity1Index = rbs1Jump + i1;

                    for (int rbs2 = rbs1 + 1; rbs2 < rbsIdAntennaNumber.Count; rbs2++)
                    {
                        int rbs2Jump = rbsIdAntennaNumber.Take(rbs2).Sum();
                        for (int i2 = 0; i2 < rbsIdAntennaNumber[rbs2]; i2++)
                        {
                            int entity2Index = rbs2Jump + i2;
                            mimoChannelLinks.Add(new int2(entity1Index, entity2Index));
                            entityAntennasInLinks.Add(new int2(rbs1Jump + i1, rbs2Jump + i2));
                        }
                    }

                }
            }
        }
        // Check if _channelLinks is allocated and deallocate if necessary
        if (_channelLinks.IsCreated)
        {
            _channelLinks.Dispose();
        }
        int numChannelLinks = mimoChannelLinks.Count;
        _channelLinks = new NativeArray<int2>(mimoChannelLinks.ToArray(), Allocator.Persistent);
        _channelLinksLocalAntennaIndex = new NativeArray<int2>(entityAntennasInLinks.ToArray(), Allocator.Persistent);

        Debug.Log($"Number of ChannelLinks: {numChannelLinks}");

        for (int linkId = 0; linkId < _channelLinks.Length; linkId++)
        {
            int2 linkPair = entityAntennasInLinks[linkId];
            //int2 linkPair = mimoChannelLinks[linkId];
            Debug.Log($"Link {linkId}: antenna {linkPair.x} of {_entityArray[_channelLinks[linkId].x].transform.parent.name} and antenna {linkPair.y} of {_entityArray[_channelLinks[linkId].y].transform.parent.name}");
        }

        GSCMrss.Links = mimoChannelLinks;
        GSCMSpectrum.Links = mimoChannelLinks;
        GSCMrss.RSS = new List<double>();
        GSCMSpectrum.RSSspectrumGain = new List<List<double>>();
        GSCMSpectrum.RSSComplexChannel = new List<List<double>>();
        GSCMSpectrum.RSSspectrumGainRl = new List<List<double>>();
        GSCMSpectrum.RSSspectrumGainIm = new List<List<double>>();
        double dummyRss = 0.0;
        List<double> tempDummyGain = Enumerable.Repeat(0.0, _parameters.radioParameters.numFft).ToList();
        List<double> tempDummyComplexeChannel = Enumerable.Repeat(0.0, _parameters.radioParameters.numFft*3).ToList();
        for (int l = 0; l < GSCMSpectrum.Links.Count + 1; l++) // + 1 is for EMI
        {
            GSCMrss.RSS.Add(dummyRss); 
            GSCMSpectrum.RSSspectrumGain.Add(tempDummyGain);
            GSCMSpectrum.RSSspectrumGainRl.Add(tempDummyGain);
            GSCMSpectrum.RSSspectrumGainIm.Add(tempDummyGain);
            GSCMSpectrum.RSSComplexChannel.Add(tempDummyComplexeChannel);
        }
        
        #endregion // end of mimo

        #region EMI for NS3 read from file

        #if UNITY_EDITOR
                string emiDataPath = Application.dataPath;
                emiDataPath = emiDataPath.Replace("Assets", "EMI_Measured_Data/");
                emiDataPath += MeasuredEmiFileName;
        #endif
        
        if (UseMeasuredEMI)
        {
            if (File.Exists(emiDataPath))
            {
                emiReadData = ReadCsvToMatrixOmitFirstRow(emiDataPath);

                // Display the matrix in the console
                for (int i = 0; i < emiReadData.GetLength(0); i++)
                {
                    string row = "";
                    for (int j = 0; j < emiReadData.GetLength(1); j++)
                    {
                        row += emiReadData[i, j] + " ";
                    }
                    // Debug.Log(row);
                }
            }
            else
            {
                Debug.LogError("CSV file not found");
            }
        }

        #endregion // end of EMI fro NS3
        
        #endregion

        #region Ellipse parameters

        _ellipsesSequential = new NativeArray<EllipseParameters>(_channelLinks.Length, Allocator.Persistent);
        _ellipseParallel = new NativeArray<EllipseParameters>(_channelLinks.Length, Allocator.Persistent);

        #endregion

        #region Preallocating colors for links
        linkColors = new Color[numChannelLinks];
        // Define the base color in HSV
        float baseHue = 0.33f; // Green in HSV
        float saturation = 1.0f;
        float value = 1.0f;
        float hueStep = 1.0f / numChannelLinks / 2; // 2 is used to have different colors for different types of links

        // Use golden ratio to spread hues evenly
        float goldenRatioConjugate = 0.618033988749895f;

        // Calculate and store the colors in the preallocated array
        for (int i = 0; i < numChannelLinks; i++)
        {
            float hue = Mathf.Repeat(baseHue + goldenRatioConjugate * i, 1.0f);

            // Vary saturation and value to create more distinct colors
            float adjustedSaturation = Mathf.Clamp(saturation + (float)Math.Sin(hue * Math.PI) * 0.2f, 0.7f, 1.0f);
            float adjustedValue = Mathf.Clamp(value + (float)Math.Cos(hue * Math.PI * 2) * 0.1f, 0.8f, 1.0f);

            linkColors[i] = Color.HSVToRGB(hue, adjustedSaturation, adjustedValue);
        }
        #endregion

        
    }

    private void OnDestroy()
    {
        _antennaPattern.Dispose();
        
        allUeAntennaPattern3D.Dispose();
        
        _antennaAndRotations.Dispose();
        _antennaDataArray.Dispose();
        _antennaTransforms.Dispose();

        _eadfArray.Dispose();
        
        _mu1.Dispose();
        _mu2.Dispose();

        _mpc1PointsWithNormal.Dispose();
        _mpc2PointsWithNormal.Dispose();
        _mpc3PointsWithNormal.Dispose();

        _opsCoordinates.Dispose();
        _entityCoordinates.Dispose();
        _entityForwardVect.Dispose();
        _entityRotationMatrices.Dispose();
        thetaPhi.Dispose();


        _ueIdAntennaNumber.Dispose();
        _rbsIdAntennaNumber.Dispose();

        _channelLinks.Dispose();
        _channelLinksLocalAntennaIndex.Dispose();

        _subCarriers.Dispose();
        _inverseWavelengths.Dispose();

        _lut2.Dispose();
        _mpc2Pointers.Dispose();

        mpc3PointersHalf.Dispose();
        lut3Half.Dispose();

        _lut3.Dispose();
        _mpc3Pointers.Dispose();

        mpc1Float.Dispose();
        mpc2Float.Dispose();
        mpc3Float.Dispose();

        _ellipsesSequential.Dispose();
        _ellipseParallel.Dispose();

        _HLoS.Dispose();
        _HnLoS.Dispose();

        _totalH.Dispose();
        _mrcH.Dispose();
    }

    void Awake()
    {
        if (Application.isEditor)
        {
            applicationMode = "Editor";
        }
        else if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            applicationMode = "Compiled and Headless";
        }
        else
        {
            applicationMode = "Compiled";
        }

        Debug.Log("Application Mode: " + applicationMode);

        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            // Disable the camera or other rendering components
            Camera.main.enabled = false;
        }
    }

    void Start()
    {

        #region Detecting headless mode
        if (SystemInfo.graphicsDeviceType == UnityEngine.Rendering.GraphicsDeviceType.Null)
        {
            Debug.Log("Running in headless mode");
        }
        #endregion

        GameObject mpcParent = GameObject.Find(_parameters.mpcSpawningParameters.mpcParent);
        if (mpcParent == null)
        {
            mpcParent = new GameObject(_parameters.mpcSpawningParameters.mpcParent);
        }

        if (_parameters.mpcPointCloudParameters.pathPointCloud is null && _parameters.mpcPointCloudParameters.pathNormals is null)
        {
            #region 1) Spawning points uniformly
            Debug.Log(" - Spawning MPC points");
            Stopwatch stopwatchSpawn = Stopwatch.StartNew();
            // Rnd solution comes from https://forum.unity.com/threads/mathematics-random-with-in-ijobprocesscomponentdata.598192/#post-4009273
            NativeArray<Unity.Mathematics.Random> rngs =
                new NativeArray<Unity.Mathematics.Random>(JobsUtility.MaxJobThreadCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < JobsUtility.MaxJobThreadCount; i++)
            {
                rngs[i] = new Unity.Mathematics.Random((uint)UnityEngine.Random.Range(int.MinValue, int.MaxValue));
            }

            // Retrieve reflecting obstacles and spawning plane form scene
            _reflectingObstacles = GameObject.FindGameObjectsWithTag("Reflecting_Obstacles");
            _spawningPlanes = GameObject.FindGameObjectsWithTag("MPCSpawningPlane");
            _roads = GameObject.FindGameObjectsWithTag("Roads");


            float gMpc1Shift = 0; // dB
            _mpc1PointsWithNormal = MPC.spawnScatterPoints(
                pathOrder: 1,
                spawningPlanes: _spawningPlanes,
                reflectingObstacles: _reflectingObstacles,
                chi: _parameters.mpcSpawningParameters.chiW1,
                distance: _parameters.mpcSpawningParameters.distMpc1,
                attenuation: new float2(_parameters.mpcSpawningParameters.minGMpc1 + costIraconFactor + gMpc1Shift, _parameters.mpcSpawningParameters.maxGMpc1 + costIraconFactor + gMpc1Shift),
                rndNums: rngs,
                observationPoints: _opsCoordinates
            );


            float gMpc2Shift = 0; // dB
            _mpc2PointsWithNormal = MPC.spawnScatterPoints(
                pathOrder: 2,
                spawningPlanes: _spawningPlanes,
                reflectingObstacles: _reflectingObstacles,
                chi: _parameters.mpcSpawningParameters.chiW2,
                distance: _parameters.mpcSpawningParameters.distMpc2,
                attenuation: new float2(_parameters.mpcSpawningParameters.minGMpc2 + costIraconFactor - gMpc2Shift, _parameters.mpcSpawningParameters.maxGMpc2 + costIraconFactor - gMpc2Shift),
                rndNums: rngs,
                observationPoints: _opsCoordinates
            );
            
            float gMpc3Shift = 0; // dB
            _mpc3PointsWithNormal = MPC.spawnScatterPoints(
                pathOrder: 3,
                spawningPlanes: _spawningPlanes,
                reflectingObstacles: _reflectingObstacles,
                chi: _parameters.mpcSpawningParameters.chiW3,
                distance: _parameters.mpcSpawningParameters.distMpc3,
                attenuation: new float2(_parameters.mpcSpawningParameters.minGMpc3 + costIraconFactor - gMpc3Shift, _parameters.mpcSpawningParameters.maxGMpc3 + costIraconFactor - gMpc3Shift),
                rndNums: rngs,
                observationPoints: _opsCoordinates
            );

            rngs.Dispose();

            stopwatchSpawn.Stop();
            Debug.Log("Time taken for Spawning points uniformly: " + stopwatchSpawn.ElapsedMilliseconds + "ms");
            #endregion
        }
        else
        {
            Debug.Log($"Loading point cloud from {_parameters.mpcPointCloudParameters.pathPointCloud} with normals from {_parameters.mpcPointCloudParameters.pathNormals}");

            if (!GameObject.Find("Reflecting_Obstacles"))
            {
                _reflectingObstacles = MeshLoader.LoadAllMeshes(resourcePath: _parameters.mpcPointCloudParameters.pathMesh, parentTransform: null);
            }
            else
            {
                GameObject parentObject = GameObject.Find("Reflecting_Obstacles");
                int childCount = parentObject.transform.childCount;
                _reflectingObstacles = new GameObject[childCount];

                for (int i = 0; i < childCount; i++)
                {
                    _reflectingObstacles[i] = parentObject.transform.GetChild(i).gameObject;
                }
            }

            _opsCoordinates = ObservationPointSelector.SelectObservationPoints(Path.Combine(directoryPath, _scenario.general.uePaths), distance: 1, additionalOPs: _rbsArray, axesMapping: new Vector3Int(0, 2, 1), visualObstacles: _reflectingObstacles);

            (_mpc1PointsWithNormal, _mpc2PointsWithNormal, _mpc3PointsWithNormal) = MPC.CreateMPCFromPointCloud(
                pathPointCloud: Path.Combine(directoryPath, _parameters.mpcPointCloudParameters.pathPointCloud),
                skipEveryNthPoint: _parameters.mpcPointCloudParameters.skipEveryNPoints,
                axesMapping: new Vector3Int(0, 2, 1),
                chiW1: _parameters.mpcSpawningParameters.chiW1,
                chiW2: _parameters.mpcSpawningParameters.chiW2,
                chiW3: _parameters.mpcSpawningParameters.chiW3,
                attenuation1: new float2(_parameters.mpcSpawningParameters.minGMpc1 + costIraconFactor, _parameters.mpcSpawningParameters.maxGMpc1 + costIraconFactor),
                attenuation2: new float2(_parameters.mpcSpawningParameters.minGMpc2 + costIraconFactor, _parameters.mpcSpawningParameters.maxGMpc2 + costIraconFactor),
                attenuation3: new float2(_parameters.mpcSpawningParameters.minGMpc2 + costIraconFactor, _parameters.mpcSpawningParameters.maxGMpc2 + costIraconFactor),
                observationPoints: _opsCoordinates,
                distance1: _parameters.mpcSpawningParameters.distMpc1,
                distance2: _parameters.mpcSpawningParameters.distMpc2,
                distance3: _parameters.mpcSpawningParameters.distMpc3,
                offset: _parameters.mpcPointCloudParameters.offset,
                reflectingObstacles: _reflectingObstacles
            );
        }

        // Check that there are MPCs
        Assert.IsTrue(_mpc1PointsWithNormal.Length > 0, "No 1st order MPCs");
        Assert.IsTrue(_mpc2PointsWithNormal.Length > 0, "No 2nd order MPCs");
        Assert.IsTrue(_mpc3PointsWithNormal.Length > 0, "No 3rd order MPCs");

        Debug.Log($"Number of components MPC1:{_mpc1PointsWithNormal.Length}, MPC2:{_mpc2PointsWithNormal.Length}, MPC3:{_mpc3PointsWithNormal.Length}");

        #region Show scatter points

        if (showArtefacts)
        {
            Stopwatch stopwatchArtefacts = Stopwatch.StartNew();
            
            for (int i = 0; i < _mpc1PointsWithNormal.Length; i++)
            {
                GameObject spawnedObject = Instantiate(mpc1Prefab, _mpc1PointsWithNormal[i].Point, Quaternion.identity);
                spawnedObject.transform.SetParent(mpcParent.transform);
                spawnedObject.name = "MPC1 nbr " + i + " " + _mpc1PointsWithNormal[i].Normal;
                //mpc1GameObjects.Add(spawnedObject);
                //Debug.DrawLine(_mpc1PointsWithNormal[i].Point, _mpc1PointsWithNormal[i].Point + _mpc1PointsWithNormal[i].Normal, Color.cyan, 10);
            }

            for (int i = 0; i < _mpc2PointsWithNormal.Length; i++)
            {
                GameObject spawnedObject = Instantiate(mpc2Prefab, _mpc2PointsWithNormal[i].Point, Quaternion.identity);
                spawnedObject.transform.SetParent(mpcParent.transform);
                spawnedObject.name = "MPC2 nbr " + i;
                //mpc2GameObjects.Add(spawnedObject);
                //Debug.DrawLine(_mpc2PointsWithNormal[i].Point, _mpc2PointsWithNormal[i].Point + _mpc2PointsWithNormal[i].Normal, Color.cyan, 10);
            }

            for (int i = 0; i < _mpc3PointsWithNormal.Length; i++)
            {
                GameObject spawnedObject = Instantiate(mpc3Prefab, _mpc3PointsWithNormal[i].Point, Quaternion.identity);
                spawnedObject.transform.SetParent(mpcParent.transform);
                spawnedObject.name = "MPC3 nbr " + i;
                //mpc3GameObjects.Add(spawnedObject);
                //Debug.DrawLine(_mpc3PointsWithNormal[i].Point, _mpc3PointsWithNormal[i].Point + _mpc3PointsWithNormal[i].Normal, Color.cyan, 10);
            }

            stopwatchArtefacts.Stop();
            Debug.Log("Time taken render MPCs: " + stopwatchArtefacts.ElapsedMilliseconds + "ms");
        }
        #endregion


        #region Channel logging
        if (_scenario.general.logChannelToFile)
        {
            GameObject loggerObject = GameObject.Find("Scripts");
            if (loggerObject != null)
            {
                _dataLogger = loggerObject.GetComponent<DataLogger>();

                // Ensure the directory exists
                string fullPath = Path.Combine(Environment.CurrentDirectory, _scenario.general.channelOutputPath);
                Directory.CreateDirectory(fullPath); // This will create the directory if it doesn't exist

                // Set the path for the DataLogger
                _dataLogger.SetFilePath(Environment.CurrentDirectory, _scenario.general.channelOutputPath);
                Debug.Log($"DataLogger GameObject found. Outputs to: {_dataLogger.GetFilePath()}");
            }
            else
            {
                Debug.Log("DataLogger GameObject not found in the scene!");
            }
        }
        #endregion


        #region 2.7) Converting Vector3 MPC.Point MPC.Normal struct to float3 MPCFloat.Point MPCFloat.Normal
        // Attenuation coefficients in dB are also converted to linear scale using math.pow(10f, mpcList[i].Attentuation/20f);
        mpc1Float = nativeListFloatConversion.FloatConversion(_mpc1PointsWithNormal);
        mpc2Float = nativeListFloatConversion.FloatConversion(_mpc2PointsWithNormal);
        mpc3Float = nativeListFloatConversion.FloatConversion(_mpc3PointsWithNormal);
        #endregion

        #region 3) Distance Matrices Calculation
        Stopwatch stopwatchLUT = Stopwatch.StartNew();

        // Creating LUT for MPC2s
        (_lut2MaxVisibility, _mpc2Pointers, _lut2) = _lutDictionary.CalculateLUT2(mpc2Float, _parameters.physicalParameters.maxPathDistance, attenuationThreshold, lambdaOver4PI);

        // Creating LUT for MPC3s
        (_lut3MaxVisibility, mpc3PointersHalf, lut3Half) = _lutDictionary.CalculateLUT2(mpc3Float, _parameters.physicalParameters.maxPathDistance, attenuationThreshold, lambdaOver4PI);

        (_lut3MaxVisibility, _mpc3Pointers, _lut3) = _lutDictionary.CalculateLUT3(mpc3Float, mpc3PointersHalf, lut3Half,
            _parameters.physicalParameters.maxPathDistance, attenuationThreshold, lambdaOver4PI);

        Debug.Log($"LUT 2 has: {_lut2.Length} entries, and LUT 3 has: {_lut3.Length} entries");

        #region Debug drawing
        /*
        for (int i = 0; i < 100; i++)
        {
            LUT3 lut3TestElement = _lut3[i];
            Debug.DrawLine(mpc3Float[lut3TestElement.InMpcId].Point, mpc3Float[lut3TestElement.MidMpcId].Point,
                Color.yellow, 5f);
            Debug.DrawLine(mpc3Float[lut3TestElement.MidMpcId].Point, mpc3Float[lut3TestElement.OutMpcId].Point,
                Color.yellow, 5f);
        }
        */
        #endregion

        stopwatchLUT.Stop();
        Debug.Log("Time taken generate LUTs: " + stopwatchLUT.ElapsedMilliseconds + "ms");

        #endregion

        #region Channels
        _HLoS = new NativeArray<System.Numerics.Complex>(_parameters.radioParameters.numFft * _channelLinks.Length,
            Allocator.Persistent);
        _HnLoS = new NativeArray<System.Numerics.Complex>(_parameters.radioParameters.numFft * _channelLinks.Length,
            Allocator.Persistent);
        _totalH = new NativeArray<System.Numerics.Complex>(_parameters.radioParameters.numFft * _channelLinks.Length,
            Allocator.Persistent);
        _mrcH = new NativeArray<System.Numerics.Complex>(_parameters.radioParameters.numFft, Allocator.Persistent);
        
        

        //Debug.Log("Size of H LoS: " + _HLoS.Length);
        //Debug.Log("Size of H nLoS: " + _HnLoS.Length);
        //Debug.Log("Size of H total: " + _totalH.Length);
        #endregion

        //Time.fixedDeltaTime = _scenario.timeParameters.fixedTimestep;
        //Time.timeScale = _scenario.timeParameters.timeScale;

    }

    void Update()
    {
        // Record the latest frame time
        frameTimes[frameIndex] = Time.unscaledDeltaTime;
        frameIndex = (frameIndex + 1) % sampleSize;
        if (frameIndex == 0) filled = true;

        //Debug.Log
    }

    float CalculateFPS()
    {
        float totalFrameTime = 0f;
        int count = filled ? sampleSize : frameIndex;
        for (int i = 0; i < count; i++)
        {
            totalFrameTime += frameTimes[i];
        }
        return 1.0f / (totalFrameTime / count);
    }

    void FixedUpdate()
    {
        fixedUpdateCounter++;

        if (fixedUpdateCounter >= totalNumberOfSnapshots)
        {
            Debug.Log("Fixed update counter reached the limit. Stopping the simulation.");

            Application.Quit(); // Stop the application
            // If running in the Unity editor
            #if UNITY_EDITOR
                UnityEditor.EditorApplication.isPlaying = false;
            #endif
        }

        Stopwatch totalTicks = new Stopwatch();
        totalTicks.Start();

        #region 1 Update antenna positions and forward vectors


        for (int i = 0; i < _antennaTransforms.length; i++)
        {
            
            _entityCoordinates[i] = _antennaTransforms[i].position;
            _entityRotationMatrices[i] = new float3x3(_antennaTransforms[i].rotation);
            
            float3 tempRight = _antennaTransforms[i].right;
            float3 tempUp = _antennaTransforms[i].up;
            float3 tempForward = _antennaTransforms[i].forward;
            
            Vector3 temPosition = _antennaTransforms[i].position;
            Vector3 temForward = _entityRotationMatrices[i].c2;
            Vector3 temUp = _entityRotationMatrices[i].c1;
            Vector3 temRight = _entityRotationMatrices[i].c0;
            
            Debug.DrawLine(temPosition, temPosition + temForward * 5.0f, Color.blue);
            Debug.DrawLine(temPosition, temPosition + 5.0f * temRight, Color.red);
            Debug.DrawLine(temPosition, temPosition + 5.0f * temUp, Color.green);

            
        }

        #endregion

        #region 2 Ellipse calculation parallel
        float testMaxDistance = _parameters.physicalParameters.maxPathDistance;
        EllipseCalculationParallel ellipseCalculationParallel = new EllipseCalculationParallel
        {
            EntityCoordinates = _entityCoordinates,

            ChannelLinks = _channelLinks,
            HalfMaxDistance = testMaxDistance, //_parameters.physicalParameters.maxPathDistance/2f,

            EllipseParallel = _ellipseParallel
        };
        JobHandle parallelEllipseCalculationHandle =
            ellipseCalculationParallel.Schedule(_channelLinks.Length, 64);
        parallelEllipseCalculationHandle.Complete();

        #endregion

        #region 3 Valid for channel calculation MPCs search

        #region 3.1 First order paths (out pathsGainFirstOrderResult)
        #region 3.1.1 Elipse search 1st order
        NativeList<RayCastIndex> mpc1Entity1RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);
        NativeList<RayCastIndex> mpc1Entity2RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);

        mpc1Entity1RayCastIndex.Capacity = mpc1Float.Length * _channelLinks.Length; 
        mpc1Entity2RayCastIndex.Capacity = mpc1Float.Length * _channelLinks.Length;

        InEllipseMpcsSearchAllLinks inEllipseMpcsSearch = new InEllipseMpcsSearchAllLinks
        {
            // Input
            EntityCoordinates = _entityCoordinates,
            ChannelLinks = _channelLinks,
            EllipseInformation = _ellipseParallel,
            Mpcs = mpc1Float,

            RCEntity1Index = mpc1Entity1RayCastIndex.AsParallelWriter(),
            RCEntity2Index = mpc1Entity2RayCastIndex.AsParallelWriter()
        };
        JobHandle handleInEllipse1 =
            inEllipseMpcsSearch.Schedule(mpc1Float.Length * _channelLinks.Length, 64);
        handleInEllipse1.Complete();
        #endregion
        #region 3.1.2 Results sorting (For first order paths it is not necessary)

        var jobEntity1 = new SortJob { Data = mpc1Entity1RayCastIndex.AsArray() };
        JobHandle handle = jobEntity1.Schedule();
        handle.Complete();

        var jobEntity2 = new SortJob { Data = mpc1Entity2RayCastIndex.AsArray() };
        JobHandle handleRbs = jobEntity2.Schedule();
        handleRbs.Complete();

        #endregion

        #region 3.1.3 First order raycast
        #region 3.1.3.0 Findining the boundaries of the links for UE: linkUeBoundaries

        NativeArray<int2> mpc1AndEntity1IndexArray = new NativeArray<int2>(mpc1Entity1RayCastIndex.Length, Allocator.TempJob); // Disposed? Yes 
        NativeArray<RaycastCommand> raycast1Entity1 = new NativeArray<RaycastCommand>(mpc1Entity1RayCastIndex.Length, Allocator.TempJob); // Dispose? Yes
        NativeArray<int> linkEntity1Boundaries = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);

        var prepareRaycastEntity1Job = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc1AndEntity1IndexArray,
            RaycastCommands = raycast1Entity1,
            LinkBoundaries = linkEntity1Boundaries,

            MpcRayCastLink = mpc1Entity1RayCastIndex.AsArray()
        };

        var jobHandle = prepareRaycastEntity1Job.Schedule();
        jobHandle.Complete();

        #endregion
        #region 3.1.3.1 Entity1 raycast

        NativeArray<RaycastHit> raycastHit1Entity1 = new NativeArray<RaycastHit>(mpc1Entity1RayCastIndex.Length, Allocator.TempJob); // Dispose
        JobHandle raycastJob1Entity1 = RaycastCommand.ScheduleBatch(raycast1Entity1, raycastHit1Entity1, 16);
        raycastJob1Entity1.Complete();


        #endregion

        #region 3.1.3.2 Findining the boundaries of the links for Entity2: linkRbsBoundaries

        NativeArray<int2> mpc1AndEntity2IndexArray = new NativeArray<int2>(mpc1Entity2RayCastIndex.Length, Allocator.TempJob);// TODO: Dispose
        NativeArray<RaycastCommand> raycast1Entity2 = new NativeArray<RaycastCommand>(mpc1Entity2RayCastIndex.Length, Allocator.TempJob);// TODO: Dispose
        NativeArray<int> linkEntity2Boundaries = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);
        var prepareRaycastEntity2Job = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc1AndEntity2IndexArray,
            RaycastCommands = raycast1Entity2,
            LinkBoundaries = linkEntity2Boundaries,

            MpcRayCastLink = mpc1Entity2RayCastIndex.AsArray()
        };

        var jobRbsHandle = prepareRaycastEntity2Job.Schedule();
        jobRbsHandle.Complete();

        #endregion
        #region 3.1.3.3 RBS raycast
        NativeArray<RaycastHit> raycastHit1Entity2 = new NativeArray<RaycastHit>(mpc1Entity2RayCastIndex.Length, Allocator.TempJob); // TODO: Dispose

        JobHandle raycastJob1Entity2 = RaycastCommand.ScheduleBatch(raycast1Entity2, raycastHit1Entity2, 16);
        raycastJob1Entity2.Complete();


        #endregion
        #endregion

        #region 3.1.4 First order path search
        NativeList<PathGainResultEntry> pathsGainFirstOrderResult = new NativeList<PathGainResultEntry>(Allocator.TempJob);

        pathsGainFirstOrderResult.Capacity = raycastHit1Entity1.Length * _channelLinks.Length;



        FirstOrderPathsAllLinks firstOrderPaths = new FirstOrderPathsAllLinks()
        {
            AntennaPatternsUe = allUeAntennaPattern3D,
            AntennaPatternsBs = allBsAntennaPattern3D,
            AntennaType1 = _antennaType1,
            AntennaType2 = _antennaType2,

            Threshold = 0.01f * attenuationThreshold,
            CosAngularSpread = math.cos(_parameters.physicalParameters.angularSpreadFirstOrder),

            // Mpcs
            Mpcs = mpc1Float,

            Links = _channelLinks,
            LocalAntennaIds = _channelLinksLocalAntennaIndex,

            // Indexing
            MpcAndUeIndex = mpc1AndEntity1IndexArray,
            MpcAndRbsIndex = mpc1AndEntity2IndexArray,

            LinkEntity2Bounds = linkEntity2Boundaries,

            // Entity related
            EntityCoordinates = _entityCoordinates,
            RaycastHitEntity1 = raycastHit1Entity1,
            RaycastHitEntity2 = raycastHit1Entity2,

            // 3D antenna radiation parameters
            RaycastCommandsEntity1 = raycast1Entity1,
            RaycastCommandsEntity2 = raycast1Entity2,
            EntityRotationMatrices = _entityRotationMatrices,
            
            // EADF
            EnableEADF = enableEADF,
            Mu1 = _mu1,
            Mu2 = _mu2,
            EADF = _eadfArray,

            //Output 
            Result = pathsGainFirstOrderResult.AsParallelWriter()
        };
        JobHandle handleFirstOrderPaths = firstOrderPaths.Schedule(mpc1AndEntity1IndexArray.Length, 64);
        handleFirstOrderPaths.Complete();



        raycast1Entity1.Dispose();
        raycast1Entity2.Dispose();
        #endregion
        #region Drawing Paths1


        //Debug.Log($"First order paths: {pathsGainFirstOrderResult.Length}");
        if (showFirstOrderPaths)
        {
            for (int i = 0; i < pathsGainFirstOrderResult.Length; i++)
            {
                PathGainResultEntry pathGainFirstOrderResultEntry = pathsGainFirstOrderResult[i];
                int mpcId = pathGainFirstOrderResultEntry.MpcIndices.x;
                int linkId = pathGainFirstOrderResultEntry.LinkIndex;
                int TxId = _channelLinks[linkId].x;
                int RxId = _channelLinks[linkId].y;
                //Debug.DrawLine(_entityCoordinates[TxId], mpc1Float[mpcId].Point, linkColors[linkId]);
                //Debug.DrawLine(_entityCoordinates[RxId], mpc1Float[mpcId].Point, linkColors[linkId]);
                
                Debug.DrawLine(_entityCoordinates[TxId], mpc1Float[mpcId].Point, Color.green);
                Debug.DrawLine(_entityCoordinates[RxId], mpc1Float[mpcId].Point, Color.green);

            }
        }

        #endregion
        #endregion

        #region 3.2 Second order paths (out pathsGainSecondOrderResults)
        #region 3.2.1 Elipse search 2nd order
        int estimatedCapacity2 = mpc2Float.Length * _channelLinks.Length;

        NativeList<RayCastIndex> mpc2Entity1RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);
        NativeList<RayCastIndex> mpc2Entity2RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);

        mpc2Entity1RayCastIndex.Capacity = estimatedCapacity2;
        mpc2Entity2RayCastIndex.Capacity = estimatedCapacity2;

        InEllipseMpcsSearchAllLinks inEllipseMpc2Search = new InEllipseMpcsSearchAllLinks
        {
            EntityCoordinates = _entityCoordinates,
            ChannelLinks = _channelLinks,
            EllipseInformation = _ellipseParallel,
            Mpcs = mpc2Float,

            RCEntity1Index = mpc2Entity1RayCastIndex.AsParallelWriter(),
            RCEntity2Index = mpc2Entity2RayCastIndex.AsParallelWriter()
        };
        JobHandle handleInEllipse2 =
            inEllipseMpc2Search.Schedule(mpc2Float.Length * _channelLinks.Length, 64);
        handleInEllipse2.Complete();


        #endregion
        #region 3.2.2 Results sorting (For first order paths it is not necessary)
        var sortJob2Entity1 = new SortJob { Data = mpc2Entity1RayCastIndex.AsArray() };
        JobHandle sort2HandleEntity1 = sortJob2Entity1.Schedule();
        sort2HandleEntity1.Complete();

        var sort2JobEntity2 = new SortJob { Data = mpc2Entity2RayCastIndex.AsArray() };
        JobHandle sort2HandleEntity2 = sort2JobEntity2.Schedule();
        sort2HandleEntity2.Complete();

        #endregion

        #region 3.2.3 Second order raycast 
        #region 3.2.3.1 Entity1 raycast 
        NativeArray<int2> mpc2AndEntity1IndexArray = new NativeArray<int2>(mpc2Entity1RayCastIndex.Length, Allocator.TempJob);
        NativeArray<RaycastCommand> raycast2Entity1 = new NativeArray<RaycastCommand>(mpc2Entity1RayCastIndex.Length, Allocator.TempJob);
        NativeArray<int> linkEntity1Boundaries2 = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);
        var prepareRaycast2Entity1Job = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc2AndEntity1IndexArray,
            RaycastCommands = raycast2Entity1,
            LinkBoundaries = linkEntity1Boundaries2,

            MpcRayCastLink = mpc2Entity1RayCastIndex.AsArray()
        };
        var job2Entity1Handle = prepareRaycast2Entity1Job.Schedule();
        job2Entity1Handle.Complete();

        NativeArray<RaycastHit> raycastHit2Entity1 = new NativeArray<RaycastHit>(mpc2Entity1RayCastIndex.Length, Allocator.TempJob);
        JobHandle raycastJob2Entity1 = RaycastCommand.ScheduleBatch(raycast2Entity1, raycastHit2Entity1, 16);
        raycastJob2Entity1.Complete();
        #endregion

        #region 3.2.3.2 RBS raycast
        NativeArray<int2> mpc2AndEntity2IndexArray = new NativeArray<int2>(mpc2Entity2RayCastIndex.Length, Allocator.TempJob);
        NativeArray<RaycastCommand> raycast2Entity2 = new NativeArray<RaycastCommand>(mpc2Entity2RayCastIndex.Length, Allocator.TempJob);
        NativeArray<int> linkEntity2Boundaries2 = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);
        var prepareRaycast2Entity2Job = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc2AndEntity2IndexArray,
            RaycastCommands = raycast2Entity2,
            LinkBoundaries = linkEntity2Boundaries2,

            MpcRayCastLink = mpc2Entity2RayCastIndex.AsArray()
        };
        var job2Entity2Handle = prepareRaycast2Entity2Job.Schedule();
        job2Entity2Handle.Complete();

        NativeArray<RaycastHit> raycastHit2Entity2 = new NativeArray<RaycastHit>(mpc2Entity2RayCastIndex.Length, Allocator.TempJob); // TODO: Dispose
        JobHandle raycastJob2Entity2 = RaycastCommand.ScheduleBatch(raycast2Entity2, raycastHit2Entity2, 16);
        raycastJob2Entity2.Complete();
        #endregion
        #endregion

        #region 3.2.4 Second order path search

        NativeList<PathGainResultEntry> pathsGainSecondOrderResults = new NativeList<PathGainResultEntry>(Allocator.TempJob);
        pathsGainSecondOrderResults.Capacity = raycastHit2Entity1.Length * _channelLinks.Length * _lut2MaxVisibility;

        SecondOrderPathsAllLinks secondOrderPaths = new SecondOrderPathsAllLinks()
        {
            AntennaPatternsUe = allUeAntennaPattern3D,
            AntennaPatternsBs = allBsAntennaPattern3D,
            AntennaType1 = _antennaType1,
            AntennaType2 = _antennaType2,

            Threshold = 0.01f * attenuationThreshold,
            CosAngularSpread = math.cos(_parameters.physicalParameters.angularSpreadSecondOrder),
            Links = _channelLinks,
            LocalAntennaIds = _channelLinksLocalAntennaIndex,

            Lut2 = _lut2,
            Lut2Pointers = _mpc2Pointers,
            Mpcs = mpc2Float,

            EntityCoordinates = _entityCoordinates,

            // Ue related data
            Entity1MpcAndLink = mpc2AndEntity1IndexArray,
            RaycastHit2Entity1 = raycastHit2Entity1,

            // Rbs related data
            Entity2MpcAndLink = mpc2AndEntity2IndexArray,
            RaycastHit2Entity2 = raycastHit2Entity2,

            Entity2LinkBounds = linkEntity2Boundaries2,

            // 3D antenna radiation parameters
            RaycastCommandsEntity1 = raycast2Entity1,
            RaycastCommandsEntity2 = raycast2Entity2,
            EntityRotationMatrices = _entityRotationMatrices,
            
            EnableEADF = enableEADF23,
            Mu1 = _mu1,
            Mu2 = _mu2,
            EADF = _eadfArray,
            
            // Results
            Results = pathsGainSecondOrderResults.AsParallelWriter()
        };
        JobHandle handleSecondOrderPaths = secondOrderPaths.Schedule(mpc2AndEntity1IndexArray.Length, 16);
        handleSecondOrderPaths.Complete();
        #endregion
        #region Drawing Paths2
        
        //Debug.Log($"Second order paths: {pathsGainSecondOrderResults.Length}");
        if (showSecondOderPaths)
        {
            for (int i = 0; i < pathsGainSecondOrderResults.Length; i++)
            {
                PathGainResultEntry pathGainSecondOrderResultEntry = pathsGainSecondOrderResults[i];
                int mpcId1 = pathGainSecondOrderResultEntry.MpcIndices.x;
                int mpcId2 = pathGainSecondOrderResultEntry.MpcIndices.y;

                int linkId = pathGainSecondOrderResultEntry.LinkIndex;
                int TxId = _channelLinks[linkId].x;
                int RxId = _channelLinks[linkId].y;

                Debug.DrawLine(_entityCoordinates[TxId], mpc2Float[mpcId1].Point, Color.white);
                Debug.DrawLine(mpc2Float[mpcId1].Point, mpc2Float[mpcId2].Point, Color.white);
                Debug.DrawLine(_entityCoordinates[RxId], mpc2Float[mpcId2].Point, Color.white);
                /*
                if (linkId == 0)
                {
                    Debug.DrawLine(_entityCoordinates[TxId], mpc2Float[mpcId1].Point, Color.cyan);
                    Debug.DrawLine(mpc2Float[mpcId1].Point, mpc2Float[mpcId2].Point, Color.cyan);
                    Debug.DrawLine(_entityCoordinates[RxId], mpc2Float[mpcId2].Point, Color.cyan);
                }
                else if (linkId == 1)
                {
                    Debug.DrawLine(_entityCoordinates[TxId], mpc2Float[mpcId1].Point + new float3(0f,0.5f,0f), Color.yellow);
                    Debug.DrawLine(mpc2Float[mpcId1].Point + new float3(0f,0.5f,0f), mpc2Float[mpcId2].Point + new float3(0f,0.5f,0f), Color.yellow);
                    Debug.DrawLine(_entityCoordinates[RxId], mpc2Float[mpcId2].Point + new float3(0f,0.5f,0f), Color.yellow);
                }
                else
                {
                    Debug.DrawLine(_entityCoordinates[TxId], mpc2Float[mpcId1].Point + new float3(0f,1f,0f), Color.red);
                    Debug.DrawLine(mpc2Float[mpcId1].Point + new float3(0f,1f,0f), mpc2Float[mpcId2].Point + new float3(0f,1f,0f), Color.red);
                    Debug.DrawLine(_entityCoordinates[RxId], mpc2Float[mpcId2].Point + new float3(0f,1f,0f), Color.red);
                }*/
            }
        }

        #endregion

        #endregion

        #region 3.3 Third order paths (out pathsGainThirdOrderResults)

        #region 3.3.1 Elipse search 3rd order
        int estimatedCapacity3 = mpc3Float.Length * _channelLinks.Length;

        NativeList<RayCastIndex> mpc3Entity1RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);
        NativeList<RayCastIndex> mpc3Entity2RayCastIndex = new NativeList<RayCastIndex>(Allocator.TempJob);

        mpc3Entity1RayCastIndex.Capacity = estimatedCapacity3;
        mpc3Entity2RayCastIndex.Capacity = estimatedCapacity3;

        InEllipseMpcsSearchAllLinks inEllipseMpc3Search = new InEllipseMpcsSearchAllLinks
        {
            EntityCoordinates = _entityCoordinates,
            ChannelLinks = _channelLinks,
            EllipseInformation = _ellipseParallel,
            Mpcs = mpc3Float,

            RCEntity1Index = mpc3Entity1RayCastIndex.AsParallelWriter(),
            RCEntity2Index = mpc3Entity2RayCastIndex.AsParallelWriter()
        };
        JobHandle handleInEllipse3 =
            inEllipseMpc3Search.Schedule(mpc3Float.Length * _channelLinks.Length, 64);
        handleInEllipse3.Complete();

        #endregion
        #region 3.3.2 Results sorting (For first order paths it is not necessary)
        var sortJob3Entity1 = new SortJob { Data = mpc3Entity1RayCastIndex.AsArray() };
        JobHandle sort3HandleEntity1 = sortJob3Entity1.Schedule();
        sort3HandleEntity1.Complete();

        var sort3JobEntity2 = new SortJob { Data = mpc3Entity2RayCastIndex.AsArray() };
        JobHandle sort3HandleEntity2 = sort3JobEntity2.Schedule();
        sort3HandleEntity2.Complete();
        #endregion

        #region 3.3.3 Third order raycast
        #region 3.3.3.1 Entity1 raycast
        NativeArray<int2> mpc3AndEntitiy1IndexArray = new NativeArray<int2>(mpc3Entity1RayCastIndex.Length, Allocator.TempJob);
        NativeArray<RaycastCommand> raycast3Entity1 = new NativeArray<RaycastCommand>(mpc3Entity1RayCastIndex.Length, Allocator.TempJob);
        NativeArray<int> linkEntity1Boundaries3 = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);
        var prepareRaycast3Entity1Job = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc3AndEntitiy1IndexArray,
            RaycastCommands = raycast3Entity1,
            LinkBoundaries = linkEntity1Boundaries3,

            MpcRayCastLink = mpc3Entity1RayCastIndex.AsArray()
        };
        var job3Entity1Handle = prepareRaycast3Entity1Job.Schedule();
        job3Entity1Handle.Complete();

        NativeArray<RaycastHit> raycastHit3Entity1 = new NativeArray<RaycastHit>(mpc3Entity1RayCastIndex.Length, Allocator.TempJob);
        JobHandle raycastJob3Entity1 = RaycastCommand.ScheduleBatch(raycast3Entity1, raycastHit3Entity1, 16);
        raycastJob3Entity1.Complete();
        #endregion

        #region 3.3.3.2 Entity2 raycast
        NativeArray<int2> mpc3AndEntity1IndexArray = new NativeArray<int2>(mpc3Entity2RayCastIndex.Length, Allocator.TempJob);
        NativeArray<RaycastCommand> raycast3Entity2 = new NativeArray<RaycastCommand>(mpc3Entity2RayCastIndex.Length, Allocator.TempJob);
        NativeArray<int> linkEntity2Boundaries3 = new NativeArray<int>(_channelLinks.Length, Allocator.TempJob);
        var prepareRaycast3RbsJob = new PrepareRaycastJob
        {
            MpcAndLinkArray = mpc3AndEntity1IndexArray,
            RaycastCommands = raycast3Entity2,
            LinkBoundaries = linkEntity2Boundaries3,

            MpcRayCastLink = mpc3Entity2RayCastIndex.AsArray()
        };
        var job3Entity2Handle = prepareRaycast3RbsJob.Schedule();
        job3Entity2Handle.Complete();

        NativeArray<RaycastHit> raycastHit3Entity2 = new NativeArray<RaycastHit>(mpc3Entity2RayCastIndex.Length, Allocator.TempJob);
        JobHandle raycastJob3Entity2 = RaycastCommand.ScheduleBatch(raycast3Entity2, raycastHit3Entity2, 16);
        raycastJob3Entity2.Complete();
        #endregion

        #region 3.3.3.3 Drawing reachability of mpc3

        // Color[] colors = { Color.red, Color.green, Color.blue, Color.yellow };
        // // Ue raycastResults
        // for (int i = 0; i < mpc3AndUeIndexArray.Length; i++)
        // {
        //     if (raycastHit3Ue[i].distance == 0f)
        //     {
        //         int mpcId = mpc3AndUeIndexArray[i].x;
        //         int linkId = mpc3AndUeIndexArray[i].y;
        //         int ueId = _channelLinks[linkId].x;
        //         Debug.DrawLine(ueCoordinates[ueId], mpc3Float[mpcId].Point, colors[linkId]);
        //     }
        // }
        // // Rbs raycastResults
        // for (int i = 0; i < mpc3AndRbsIndexArray.Length; i++)
        // {
        //     if (raycastHit3Rbs[i].distance == 0f)
        //     {
        //         int mpcId = mpc3AndRbsIndexArray[i].x;
        //         int linkId = mpc3AndRbsIndexArray[i].y;
        //         int rbsId = _channelLinks[linkId].y;
        //         Debug.DrawLine(rbsCoordinates[rbsId], mpc3Float[mpcId].Point, colors[linkId]);
        //     }
        // }

        #endregion
        #endregion

        #region 3.3.4 Third order path search
        NativeList<PathGainResultEntry> pathsGainThirdOrderResults = new NativeList<PathGainResultEntry>(Allocator.TempJob);
        pathsGainThirdOrderResults.Capacity = mpc3AndEntitiy1IndexArray.Length * _channelLinks.Length * _lut3MaxVisibility;
        ThirdOrderPathsAllLinks thirdOrderPaths = new ThirdOrderPathsAllLinks()
        {
            AntennaPatternsUe = allUeAntennaPattern3D,
            AntennaPatternsBs = allBsAntennaPattern3D,
            AntennaType1 = _antennaType1,
            AntennaType2 = _antennaType2,

            AngularSpread = _parameters.physicalParameters.angularSpreadThirdOrder,
            Threshold = 0.01f * attenuationThreshold,

            Links = _channelLinks,
            LocalAntennaIds = _channelLinksLocalAntennaIndex,

            Lut3 = _lut3,
            Lut3Pointers = _mpc3Pointers,
            Mpcs = mpc3Float,

            // Entity related
            Entity1MpcAndLink = mpc3AndEntitiy1IndexArray,
            RaycastHit3Entity1 = raycastHit3Entity1,
            EntityCoordinates = _entityCoordinates,
            Entity2MpcAndLinks = mpc3AndEntity1IndexArray,
            RaycastHit3Entity2 = raycastHit3Entity2,

            Entity2LinkBounds = linkEntity2Boundaries3,

            // 3D antenna radiation parameters
            RaycastCommandsEntity1 = raycast3Entity1,
            RaycastCommandsEntity2 = raycast3Entity2,
            EntityRotationMatrices = _entityRotationMatrices,
            
            EnableEADF = enableEADF23,
            Mu1 = _mu1,
            Mu2 = _mu2,
            EADF = _eadfArray,
            
            // Results
            Results = pathsGainThirdOrderResults.AsParallelWriter()
        };
        JobHandle handleThirdOrderPaths = thirdOrderPaths.Schedule(mpc3AndEntitiy1IndexArray.Length, 16);
        handleThirdOrderPaths.Complete();

        #endregion


        #region Drawing Paths3

        if (showThirdOrderPaths)
        {
            for (int i = 0; i < pathsGainThirdOrderResults.Length; i++)
            {
                PathGainResultEntry pathGainThirdOrderResultEntry = pathsGainThirdOrderResults[i];
                int mpc3Id1 = pathGainThirdOrderResultEntry.MpcIndices.x;
                int mpc3Id2 = pathGainThirdOrderResultEntry.MpcIndices.y;
                int mpc3Id3 = pathGainThirdOrderResultEntry.MpcIndices.z;

                int linkId = pathGainThirdOrderResultEntry.LinkIndex;
                int ueId = _channelLinks[linkId].x;
                int rbsId = _channelLinks[linkId].y;
                if (linkId == 0)
                {
                    Debug.DrawLine(_entityCoordinates[ueId], mpc3Float[mpc3Id1].Point, Color.cyan);
                    Debug.DrawLine(mpc3Float[mpc3Id1].Point, mpc3Float[mpc3Id2].Point, Color.magenta);
                    Debug.DrawLine(mpc3Float[mpc3Id2].Point, mpc3Float[mpc3Id3].Point, Color.magenta);
                    Debug.DrawLine(_entityCoordinates[rbsId], mpc3Float[mpc3Id3].Point, Color.blue);
                }
                else if (linkId == 1)
                {
                    Debug.DrawLine(_entityCoordinates[ueId], mpc3Float[mpc3Id1].Point, Color.yellow);
                    Debug.DrawLine(mpc3Float[mpc3Id1].Point, mpc3Float[mpc3Id2].Point, Color.yellow);
                    Debug.DrawLine(mpc3Float[mpc3Id2].Point, mpc3Float[mpc3Id3].Point, Color.yellow);
                    Debug.DrawLine(_entityCoordinates[rbsId], mpc3Float[mpc3Id3].Point, Color.yellow);
                }
                else
                {
                    Debug.DrawLine(_entityCoordinates[ueId], mpc3Float[mpc3Id1].Point, Color.red);
                    Debug.DrawLine(mpc3Float[mpc3Id1].Point, mpc3Float[mpc3Id2].Point, Color.red);
                    Debug.DrawLine(mpc3Float[mpc3Id2].Point, mpc3Float[mpc3Id3].Point, Color.red);
                    Debug.DrawLine(_entityCoordinates[rbsId], mpc3Float[mpc3Id3].Point, Color.red);
                }
            }
        }

        #endregion

        #endregion

        #endregion

        //#region 4 Channel component calculations
        
        if (_parameters.physicalParameters.useLoSPaths)
        {
            ChannelH.FindLoSChannel(
                enableEADF: enableEADF,
                allRadiationPattern: allUeAntennaPattern3D,
                antennaType1: _antennaType1,
                antennaType2: _antennaType2,
                maxMpc1ReflectionCoefficient: _groundReflectionCoefficient,
                channelLinkPairs: _channelLinks,
                entityCoordinates: _entityCoordinates,
                entityRotationMatices: _entityRotationMatrices,
                numFft: _parameters.radioParameters.numFft,
                inverseWavelengths: _inverseWavelengths,
                eadfSize: _eadfSize,
                EADF: _eadfArray,
                mu1: _mu1,
                mu2: _mu2,
                H0: _HLoS);
        }

        
        ChannelsForAll channelCalculation = new ChannelsForAll
        {
            SizeOfH = _parameters.radioParameters.numFft,
            // MPC1 indexes and path1 gains
            Paths1Gain = pathsGainFirstOrderResult.AsArray(),
            // MPC2 indexes and path2 gains
            Paths2Gain = pathsGainSecondOrderResults.AsArray(),
            // MPC3 indexes and path3 gains
            Paths3Gain = pathsGainThirdOrderResults.AsArray(),
            // Inverse Lambdas
            InverseLambda = _inverseWavelengths,
            // NLoS Channel
            H = _HnLoS,
        };
        JobHandle handleChannelCalculation = channelCalculation.Schedule(_HnLoS.Length, 16);
        handleChannelCalculation.Complete();
        
        int sizeOfH = _parameters.radioParameters.numFft;
        int numberOfLinks = _channelLinks.Length;
        NativeArray<PathGainResultEntry> paths1Gain = pathsGainFirstOrderResult.AsArray();
        NativeArray<PathGainResultEntry> paths2Gain = pathsGainSecondOrderResults.AsArray();
        NativeArray<PathGainResultEntry> paths3Gain = pathsGainThirdOrderResults.AsArray();
        NativeArray<float> inverseLambda = _inverseWavelengths;


        // #region All NLoS paths in one
        #region Calculate H

        
        Stopwatch stopwatchChannelCalculation2 = new Stopwatch();
        stopwatchChannelCalculation2.Start();

        ChannelSumJob channelSumJob = new ChannelSumJob
        {
            HLoS = _HLoS,
            HnLoS = _HnLoS,
            TotalH = _totalH
        };
        JobHandle channelSumHandle = channelSumJob.Schedule();
        channelSumHandle.Complete();
        
        MrcChannelSumJob mrcChannelSumJob = new MrcChannelSumJob
        {
            HLoS = _HLoS,
            HnLoS = _HnLoS,
            MrcH = _mrcH
        };
        JobHandle mrcChannelSumHandle = mrcChannelSumJob.Schedule();
        mrcChannelSumHandle.Complete();
        
        stopwatchChannelCalculation2.Stop();
        Debug.Log("Time taken for total channel calculation in job parallel: " + stopwatchChannelCalculation2.ElapsedTicks + " ticks");
        
        
        
        // Clamp the value between -100 and -20
        float colorMin = -130f;
        float colorMax = -60f;
        double channelGain = 20.0f * math.log10(Complex.Abs(_totalH[0]));
        float clampedValue = Mathf.Clamp((float)channelGain, colorMin, colorMax);

        // Normalize the value to be between 0 and 1
        float t = math.unlerp(colorMin, colorMax, clampedValue); //Mathf.InverseLerp(-130f, -60f, clampedValue);

        // Interpolate between green and red
        // Color newColor = Color.Lerp(Color.green, Color.red, t);
        // Debug.Log("Channel Gain: " + channelGain);
        Renderer RSSI_randerer = _rssiObject.GetComponent<Renderer>();
        _rssiObject.GetComponent<Renderer>().material.color = Color.Lerp(Color.green, Color.red, t);

        #region NS3 spectrum channel from H_total

        List<double> rssTemp = new List<double>();
        int _RbNum = _parameters.radioParameters.numFft;

        for (int chN = 0; chN < GSCMSpectrum.Links.Count; chN++)
        {
            float RSS = 0;
            
            List<double> ComplexSpectrumGainChannel = new List<double>();
            List<double> SpectrumGainChannel = new List<double>();
            List<double> SpectrumGainChannelRl = new List<double>();
            List<double> SpectrumGainChannelIm = new List<double>();
            ComplexSpectrumGainChannel.AddRange(Enumerable.Repeat(0.0, 3*_RbNum));
            SpectrumGainChannel.AddRange(Enumerable.Repeat(0.0, _RbNum));
            SpectrumGainChannelRl.AddRange(Enumerable.Repeat(0.0, _RbNum));
            SpectrumGainChannelIm.AddRange(Enumerable.Repeat(0.0, _RbNum));
            for (int Nfr = 0; Nfr < _RbNum; Nfr++)
            {

                double H_rb = Complex.Abs(_totalH[Nfr + chN*_RbNum]);
                double H_rl = _totalH[Nfr + chN*_RbNum].Real;
                double H_im = _totalH[Nfr + chN*_RbNum].Imaginary;
                if (H_rb < 1e-15) 
                {
                    H_rl = UnityEngine.Random.Range(-10e-15f, 10e-15f);
                    H_im = UnityEngine.Random.Range(-10e-15f, 10e-15f);
                    H_rb = UnityEngine.Random.Range(-10e-15f, 10e-15f);

                }
                SpectrumGainChannelRl[Nfr] = H_rl; // RB real part
                SpectrumGainChannelIm[Nfr] = H_im; // RB img part
                SpectrumGainChannel[Nfr] = H_rb/180000; // RB total gain
                ComplexSpectrumGainChannel[Nfr] = H_rb; // RB total gain
                ComplexSpectrumGainChannel[Nfr + _RbNum] = H_rl; // RB channel real part
                ComplexSpectrumGainChannel[Nfr + 2*_RbNum] = H_im; // RB channel img part

                
                RSS += (float) Complex.Abs(_totalH[Nfr + chN*_RbNum]); //Channel gain
                
            }
            
            GSCMSpectrum.RSSspectrumGain[chN] = SpectrumGainChannel;
            GSCMSpectrum.RSSspectrumGainRl[chN] = SpectrumGainChannelRl;
            GSCMSpectrum.RSSspectrumGainIm[chN] = SpectrumGainChannelIm;
            GSCMSpectrum.RSSComplexChannel[chN] = ComplexSpectrumGainChannel;
            
            rssTemp.Add(10 * Mathf.Log10( RSS/_RbNum)); 
            
            if (UseMeasuredEMI) // copy EMI measured data to fit the Spectrum channel
            {
                // Get dimensions of the original matrix
                int rows = emiReadData.GetLength(0);
                int cols = emiReadData.GetLength(1);

                emiWaveSectrumScaled = new float[rows, cols];
                emiWaveSectrumScaled = emiReadData;
        
            }

        }

        GSCMrss.RSS = rssTemp;


        #endregion
        
        totalTicks.Stop();


        //Debug.Log("Channel calculation completed");
        #endregion
        #region Save H
        _dataLogger.EnqueueData(
            timeStamp: Time.time,
            frameRate: CalculateFPS(),
            totalTicks: totalTicks.ElapsedTicks,
            entityArray: _entityArray,
            entityCoordinates: _entityCoordinates,
            channelLinkPairs: _channelLinks,
            channelLinkPaths: new List<int> { 2, pathsGainFirstOrderResult.Length, pathsGainSecondOrderResults.Length, pathsGainThirdOrderResults.Length },
            channelData: _totalH // refrence ?
        );
        #endregion

        #region Paths search nativestuctures disposing

        // Paths1 related ones
        mpc1AndEntity1IndexArray.Dispose();
        mpc1Entity1RayCastIndex.Dispose();
        mpc1Entity2RayCastIndex.Dispose();

        linkEntity1Boundaries.Dispose();

        pathsGainFirstOrderResult.Dispose();
        raycastHit1Entity1.Dispose();
        mpc1AndEntity2IndexArray.Dispose();
        raycastHit1Entity2.Dispose();
        linkEntity2Boundaries.Dispose();

        // Paths2 related ones
        mpc2Entity1RayCastIndex.Dispose();
        mpc2Entity2RayCastIndex.Dispose();
        mpc2AndEntity1IndexArray.Dispose();
        raycast2Entity1.Dispose();
        raycastHit2Entity1.Dispose();
        linkEntity1Boundaries2.Dispose();
        mpc2AndEntity2IndexArray.Dispose();
        raycast2Entity2.Dispose();
        raycastHit2Entity2.Dispose();
        linkEntity2Boundaries2.Dispose();

        pathsGainSecondOrderResults.Dispose();

        // Paths3 related ones
        mpc3Entity1RayCastIndex.Dispose();
        mpc3Entity2RayCastIndex.Dispose();

        mpc3AndEntitiy1IndexArray.Dispose();
        raycast3Entity1.Dispose();
        linkEntity1Boundaries3.Dispose();
        raycastHit3Entity1.Dispose();

        mpc3AndEntity1IndexArray.Dispose();
        raycast3Entity2.Dispose();
        linkEntity2Boundaries3.Dispose();
        raycastHit3Entity2.Dispose();

        pathsGainThirdOrderResults.Dispose();
        #endregion
    }

    [BurstCompile]
    struct ChannelSumJob : IJob
    {
        [ReadOnly] public NativeArray<Complex> HLoS;
        [ReadOnly] public NativeArray<Complex> HnLoS;

        [WriteOnly] public NativeArray<Complex> TotalH;

        public void Execute()
        {
            for (int i = 0; i < TotalH.Length; i++)
            {
                TotalH[i] = HLoS[i] + HnLoS[i];
            }
        }
    }
    [BurstCompile]
    struct MrcChannelSumJob : IJob
    {
        [ReadOnly] public NativeArray<Complex> HLoS;
        [ReadOnly] public NativeArray<Complex> HnLoS;

        [WriteOnly] public NativeArray<Complex> MrcH;

        public void Execute()
        {
            
            for (int i = 0; i < MrcH.Length; i++)
            {
                MrcH[i] = math.sqrt(Complex.Abs(HLoS[i] + HnLoS[i])*Complex.Abs(HLoS[i] + HnLoS[i]) + Complex.Abs(HLoS[i + MrcH.Length] + HnLoS[i + MrcH.Length])*Complex.Abs(HLoS[i + MrcH.Length] + HnLoS[i + MrcH.Length]));
            }
        }
    }

    [BurstCompile]
    struct ChannelSumParallel : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Complex> HLoS;
        [ReadOnly] public NativeArray<Complex> HnLoS;

        [WriteOnly] public NativeArray<Complex> TotalH;

        public void Execute(int index)
        {
            TotalH[index] = HLoS[index] + HnLoS[index];
        }
    }

    [BurstCompile]
    struct ChannelsForAll : IJobParallelFor
    {
        [ReadOnly] public int SizeOfH;
        // MPC1 indexes and path1 gains
        [ReadOnly] public NativeArray<PathGainResultEntry> Paths1Gain;
        // MPC2 indexes and path2 gains
        [ReadOnly] public NativeArray<PathGainResultEntry> Paths2Gain;
        // MPC3 indexes and path3 gains
        [ReadOnly] public NativeArray<PathGainResultEntry> Paths3Gain;
        // Inverse Lambdas
        [ReadOnly] public NativeArray<float> InverseLambda;
        // NLoS Channel
        [WriteOnly] public NativeArray<Complex> H;
        public void Execute(int index)
        {
            int linkId = (int)math.floor(index / SizeOfH);
            int subCarrierId = index % SizeOfH;

            float2 float2H = new float2(0.0f, 0.0f);

            // Counting paths1
            float2H += SumPathContributions(Paths1Gain, subCarrierId, linkId);
            // Counting paths2
            float2H += SumPathContributions(Paths2Gain, subCarrierId, linkId);
            // Counting paths3
            float2H += SumPathContributions(Paths3Gain, subCarrierId, linkId);
            // Total NLoS channel
            H[index] = new Complex(float2H.x, float2H.y);
        }
        private float2 SumPathContributions(NativeArray<PathGainResultEntry> PathsParameters, int Subcarrier, int Link)
        {
            
            float2 pathContribution = new float2(0f, 0f);
            const float twoPi = 2f * math.PI; // Precomputed constant

            for (int i = 0; i < PathsParameters.Length; i++)
            {
                if (PathsParameters[i].LinkIndex == Link)
                {
                    float2 pathAttenuation = PathsParameters[i].PathGain / InverseLambda[Subcarrier];
                    float phase = twoPi * InverseLambda[Subcarrier] * PathsParameters[i].PathDistance;
                    float2 ePhase = new float2(math.cos(phase), -math.sin(phase));

                    pathContribution += ComplexMul(pathAttenuation, ePhase);
                }
            }

            return pathContribution;
        }

    }



    [BurstCompile]
    struct ThirdOrderPathsAllLinks : IJobParallelFor
    {
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsUe;
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsBs;
        // Physical parameters
        [ReadOnly] public AntennaType AntennaType1;
        [ReadOnly] public AntennaType AntennaType2;

        [ReadOnly] public float AngularSpread;
        
        [ReadOnly] public float Threshold;

        // Links
        [ReadOnly] public NativeArray<int2> Links;
        [ReadOnly] public NativeArray<int2> LocalAntennaIds;

        // Common data
        [ReadOnly] public NativeArray<LUT3> Lut3;
        [ReadOnly] public NativeArray<Vector2Int> Lut3Pointers;
        [ReadOnly] public NativeArray<MPCFloat> Mpcs;

        // Entity related data
        [ReadOnly] public NativeArray<float3> EntityCoordinates;
        [ReadOnly] public NativeArray<int2> Entity1MpcAndLink; 
        [ReadOnly] public NativeArray<int2> Entity2MpcAndLinks;
        [ReadOnly] public NativeArray<RaycastHit> RaycastHit3Entity1;
        [ReadOnly] public NativeArray<RaycastHit> RaycastHit3Entity2;

        [ReadOnly] public NativeArray<int> Entity2LinkBounds;

        /// <summary>
        /// For 3D antenna radiation pattern
        /// </summary>
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity1;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity2;
        [ReadOnly] public NativeArray<float3x3> EntityRotationMatrices;
        
        // enable EADF mu1 mu2 EADF
        [ReadOnly] public bool EnableEADF;
        [ReadOnly] public NativeArray<int> Mu1;
        [ReadOnly] public NativeArray<int> Mu2;
        [ReadOnly] public NativeArray<float2> EADF;

        // Resulting indexes
        [WriteOnly] public NativeList<PathGainResultEntry>.ParallelWriter Results;

        private const float FourPi = 4f * math.PI;
        public void Execute(int index)
        {
            if (RaycastHit3Entity1[index].distance == 0)
            {
                int mpc3Id = Entity1MpcAndLink[index].x;
                int link3Id = Entity1MpcAndLink[index].y;

                // Defining boundaries of potential Mpcs for the link from the Rbs side
                int entity2MpcLowerBound = 0;
                int entity2MpcUpperBound = Entity2LinkBounds[link3Id];
                if (link3Id > 0)
                {
                    entity2MpcLowerBound = Entity2LinkBounds[link3Id - 1];
                }
                // Defining boundaries of LUT2 for the Mpc2
                int lut3Start = Lut3Pointers[mpc3Id].x;
                int lut3Finish = Lut3Pointers[mpc3Id].y;
                for (int i = lut3Start; i < lut3Finish + 1; i++)
                {
                    int outMpc3Id = Lut3[i].OutMpcId;
                    for (int j = entity2MpcLowerBound; j < entity2MpcUpperBound; j++)
                    {
                        if (outMpc3Id == Entity2MpcAndLinks[j].x)
                        {
                            if (RaycastHit3Entity2[j].distance == 0)
                            {
                                #region 3D antenna radiation pattern
                                float3 entity1Direction = RaycastCommandsEntity1[index].direction;
                                float3 entity2Direction = RaycastCommandsEntity2[j].direction;
                                float3x3 entity1RotationMatrix = EntityRotationMatrices[Links[link3Id].x];
                                float3x3 entity2RotationMatrix = EntityRotationMatrices[Links[link3Id].y];

                                #endregion


                                float3 entity1Coor = EntityCoordinates[Links[link3Id].x];
                                float3 entity2Coor = EntityCoordinates[Links[link3Id].y];

                                float3 entity12Mpc = math.normalize(Mpcs[mpc3Id].Point - entity1Coor);
                                float3 entity12ImageOfNextMpc = math.normalize(Lut3[i].ImOfMidMpcIn - entity1Coor);
                                float alpha = math.acos(math.dot(entity12Mpc, entity12ImageOfNextMpc));
                                if (alpha < AngularSpread)
                                {
                                    float distEntity1 = math.distance(Mpcs[mpc3Id].Point, entity1Coor);
                                    float distEntity2 = math.distance(Mpcs[outMpc3Id].Point, entity2Coor);
                                    float path3Distance = distEntity1 + distEntity2 + Lut3[i].Distance;

                                    float path3Gain = Lut3[i].Attenuation / (FourPi * path3Distance);
                                    
                                    if (path3Gain < Threshold) continue;
                                    
                                    
                                    if (EnableEADF)
                                    {
                                        float2 ueEADF = EADFReconstruction(entity1Direction, entity1RotationMatrix, Mu1, Mu2, EADF);
                                        float2 rbsEADF = EADFReconstruction(entity2Direction, entity2RotationMatrix, Mu1, Mu2, EADF);
                                        path3Gain *= math.length(ueEADF) * math.length(rbsEADF);
                                    }
                                    
                                    float2 antennaPatternTx = float2.zero;
                                    int2 thetaPhiTx = ComputeNLoSAngles(entity1Direction, entity1RotationMatrix);
                                    int ueId = LocalAntennaIds[link3Id].x;
                                    AntennaPatternsUe.TryGetValue(thetaPhiTx + new int2(10000 * ueId), out antennaPatternTx);
                                    
                                    float2 antennaPatternRx = float2.zero;
                                    int2 thetaPhiRx = ComputeNLoSAngles(entity2Direction, entity2RotationMatrix);
                                    int bsId = LocalAntennaIds[link3Id].x;
                                    AntennaPatternsBs.TryGetValue(thetaPhiRx + new int2(10000 * bsId), out antennaPatternRx);

                                        
                                    Results.AddNoResize(new PathGainResultEntry(
                                            mpcIndices: new int3(mpc3Id, Lut3[i].MidMpcId, outMpc3Id),
                                            linkIndex: link3Id,
                                            pathGain: path3Gain * antennaPatternTx * antennaPatternRx,
                                            pathDistance: path3Distance
                                        )
                                    );
                                }
                            }
                            break;
                        }
                    }
                }
            }


        }
    }



    [BurstCompile]
    struct SecondOrderPathsAllLinks : IJobParallelFor
    {
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsUe;
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsBs;
        
        [ReadOnly] public AntennaType AntennaType1;
        [ReadOnly] public AntennaType AntennaType2;
        // Physical parameters
        [ReadOnly] public float Threshold;
        [ReadOnly] public float CosAngularSpread;

        // Links
        [ReadOnly] public NativeArray<int2> Links;
        [ReadOnly] public NativeArray<int2> LocalAntennaIds;

        // Common data
        [ReadOnly] public NativeArray<Lut2Dictionary> Lut2;
        [ReadOnly] public NativeArray<Vector2Int> Lut2Pointers;
        [ReadOnly] public NativeArray<MPCFloat> Mpcs;

        // Ue related data
        [ReadOnly] public NativeArray<float3> EntityCoordinates;
        [ReadOnly] public NativeArray<int2> Entity1MpcAndLink;
        [ReadOnly] public NativeArray<int2> Entity2MpcAndLink;
        [ReadOnly] public NativeArray<RaycastHit> RaycastHit2Entity1;
        [ReadOnly] public NativeArray<RaycastHit> RaycastHit2Entity2;

        [ReadOnly] public NativeArray<int> Entity2LinkBounds;

        /// <summary>
        /// For 3D antenna radiation pattern
        /// </summary>
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity1;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity2;
        [ReadOnly] public NativeArray<float3x3> EntityRotationMatrices;
        
        // enable EADF mu1 mu2 EADF
        [ReadOnly] public bool EnableEADF;
        [ReadOnly] public NativeArray<int> Mu1;
        [ReadOnly] public NativeArray<int> Mu2;
        [ReadOnly] public NativeArray<float2> EADF;
        // End 3D antenna radiation pattern
        
        // Resulting indexes
        [WriteOnly] public NativeList<PathGainResultEntry>.ParallelWriter Results;

        private const float FourPi = 4f * math.PI;
        public void Execute(int index)
        {
            if (RaycastHit2Entity1[index].distance == 0)
            {
                int mpc2Id = Entity1MpcAndLink[index].x;
                int link2Id = Entity1MpcAndLink[index].y;

                int rbsMpcLowerBound = (link2Id > 0) ? Entity2LinkBounds[link2Id - 1] : 0;
                int rbsMpcUpperBound = Entity2LinkBounds[link2Id];
                // Defining coordinates of the UE and RBS for convenience
                float3 ueCoor = EntityCoordinates[Links[link2Id].x];
                float3 rbsCoor = EntityCoordinates[Links[link2Id].y];

                // Defining boundaries of LUT2 for the Mpc2
                int lut2Start = Lut2Pointers[mpc2Id].x;
                int lut2Finish = Lut2Pointers[mpc2Id].y;

                for (int i = lut2Start; i < lut2Finish + 1; i++)
                {
                    int outMpc2Id = Lut2[i].OutMpcId;
                    for (int j = rbsMpcLowerBound; j < rbsMpcUpperBound; j++)
                    {
                        if (outMpc2Id != Entity2MpcAndLink[j].x || RaycastHit2Entity2[j].distance != 0) continue;

                        #region 3D antenna radiation pattern
                        float3 entity1Direction = RaycastCommandsEntity1[index].direction;
                        float3 entity2Direction = RaycastCommandsEntity2[j].direction;
                        float3x3 entity1RotationMatrix = EntityRotationMatrices[Links[link2Id].x];
                        float3x3 entity2RotationMatrix = EntityRotationMatrices[Links[link2Id].y];
                        #endregion

                        float3 ue2Mpc = math.normalize(Mpcs[mpc2Id].Point - ueCoor);
                        float3 rbs2Mpc = math.normalize(Mpcs[outMpc2Id].Point - rbsCoor);

                        float3 ue2ImageOfNextMpc = math.normalize(Lut2[i].ImageOfNextMpc - ueCoor);
                        float3 rbs2ImageOfThisMpc = math.normalize(Lut2[i].ImageOfThisMpc - rbsCoor);

                        float cosAlpha = math.dot(ue2Mpc, ue2ImageOfNextMpc);
                        float cosBeta = math.dot(rbs2Mpc, rbs2ImageOfThisMpc);

                        float angularGainFunc1 = math.min(1f, FastExp(100f * (cosAlpha - CosAngularSpread)));
                        float angularGainFunc2 = math.min(1f, FastExp(100f * (cosBeta - CosAngularSpread)));
                        float angularGainProduct = angularGainFunc1 * angularGainFunc2;

                        if (angularGainProduct <= Threshold) continue;

                        float distUe = math.distance(Mpcs[mpc2Id].Point, ueCoor);
                        float distRbs = math.distance(Mpcs[outMpc2Id].Point, rbsCoor);
                        float path2Distance = distUe + distRbs + Lut2[i].Distance;

                        float path2Gain = angularGainProduct * Lut2[i].Attenuation / (FourPi * path2Distance);

                        if (path2Gain < Threshold) continue;
                        
                        
                        if (EnableEADF)
                        {
                            float2 ueEADF = EADFReconstruction(entity1Direction, entity1RotationMatrix, Mu1, Mu2, EADF);
                            float2 rbsEADF = EADFReconstruction(entity2Direction, entity2RotationMatrix, Mu1, Mu2, EADF);
                            path2Gain *= math.length(ueEADF) * math.length(rbsEADF);
                        }
                        
                        float2 antennaPatternTx = float2.zero;
                        int2 thetaPhiTx = ComputeNLoSAngles(entity1Direction, entity1RotationMatrix);
                        int ueId = LocalAntennaIds[link2Id].x;
                        AntennaPatternsUe.TryGetValue(thetaPhiTx + new int2(10000 * ueId), out antennaPatternTx);

                        float2 antennaPatternRx = float2.zero;
                        int2 thetaPhiRx = ComputeNLoSAngles(entity2Direction, entity2RotationMatrix);
                        int bsId = LocalAntennaIds[link2Id].x;
                        AntennaPatternsBs.TryGetValue(thetaPhiRx + new int2(10000 * bsId), out antennaPatternRx);
                        
                        Results.AddNoResize(new PathGainResultEntry(
                                mpcIndices: new int3(mpc2Id, outMpc2Id, 0),
                                linkIndex: link2Id,
                                pathGain: path2Gain * antennaPatternTx * antennaPatternRx,
                                pathDistance: path2Distance
                            )
                        );

                        break;
                    }
                }
            }


        }
    }



    [BurstCompile]
    struct FirstOrderPathsAllLinks : IJobParallelFor
    {
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsUe;
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatternsBs;
        [ReadOnly] public AntennaType AntennaType1;
        [ReadOnly] public AntennaType AntennaType2;

        [ReadOnly] public float Threshold;
        [ReadOnly] public float CosAngularSpread;

        [ReadOnly] public NativeArray<MPCFloat> Mpcs;

        // Links
        [ReadOnly] public NativeArray<float3> EntityCoordinates;

        [ReadOnly] public NativeArray<int2> Links;

        [ReadOnly] public NativeArray<int2> LocalAntennaIds;

        [ReadOnly] public NativeArray<int2> MpcAndUeIndex;
        [ReadOnly] public NativeArray<int2> MpcAndRbsIndex;

        [ReadOnly] public NativeArray<int> LinkEntity2Bounds;

        [ReadOnly] public NativeArray<RaycastHit> RaycastHitEntity1;
        [ReadOnly] public NativeArray<RaycastHit> RaycastHitEntity2;

        /// <summary>
        /// For 3D antenna radiation pattern 
        /// </summary>
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity1;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommandsEntity2;
        [ReadOnly] public NativeArray<float3x3> EntityRotationMatrices;
        
        // enable EADF mu1 mu2 EADF
        [ReadOnly] public bool EnableEADF;
        //[ReadOnly] public bool Enable3DEADF;
        [ReadOnly] public NativeArray<int> Mu1;
        [ReadOnly] public NativeArray<int> Mu2;
        [ReadOnly] public NativeArray<float2> EADF;

        [WriteOnly] public NativeList<PathGainResultEntry>.ParallelWriter Result;
        public void Execute(int mpcIndex)
        {
            if (RaycastHitEntity1[mpcIndex].distance != 0f) return;


            int mpcId = MpcAndUeIndex[mpcIndex].x;
            int linkId = MpcAndUeIndex[mpcIndex].y;

            int lowerBound = (linkId > 0) ? LinkEntity2Bounds[linkId - 1] : 0;
            int upperBound = LinkEntity2Bounds[linkId];


            for (int i = lowerBound; i < upperBound; i++)
            {
                if (MpcAndRbsIndex[i].x != mpcId || RaycastHitEntity2[i].distance != 0f) continue;

                #region 3D antenna radiation pattern

                // ue => mpc
                float3 entity1Direction = RaycastCommandsEntity1[mpcIndex].direction;
                // rbs => mpc
                float3 entity2Direction = RaycastCommandsEntity2[i].direction;
                // ue rotation matrix
                float3x3 entity1RotationMatrix = EntityRotationMatrices[Links[linkId].x];
                // rbs rotation matrix
                float3x3 entity2RotationMatrix = EntityRotationMatrices[Links[linkId].y];
                #endregion

                MPCFloat mpc = Mpcs[mpcId];
                float3 entity1Coordinates = EntityCoordinates[Links[linkId].x];
                float3 imageOfEntity1 = ImageCalculation(Mpcs[mpcId], entity1Coordinates);

                float3 entity2Coordinates = EntityCoordinates[Links[linkId].y];

                // entity2 to image of entity1
                float3 imageOfEntity1ToEntity2 = math.normalize(imageOfEntity1 - entity2Coordinates);
                // entity2 to mpc
                float3 mpc2Entity2 = math.normalize(Mpcs[mpcId].Point - entity2Coordinates);
                // imageEntity1 to mpc
                float3 mpc2ImageEntity1 = math.normalize(Mpcs[mpcId].Point - imageOfEntity1);

                float cosAlpha = math.dot(mpc2ImageEntity1, -imageOfEntity1ToEntity2);
                float cosBeta = math.dot(mpc2Entity2, imageOfEntity1ToEntity2);

                float angularGainFunc1 = math.min(1f, FastExp(2000f * (cosAlpha - CosAngularSpread)));
                float angularGainFunc2 = math.min(1f, FastExp(2000f * (cosBeta - CosAngularSpread)));

                float angularGainProduct = angularGainFunc1 * angularGainFunc2;
                if (angularGainProduct <= Threshold) continue;

                float distanceEntity1 = math.distance(mpc.Point, entity1Coordinates);
                float distanceEntity2 = math.distance(mpc.Point, entity2Coordinates);
                float path1Distance = distanceEntity1 + distanceEntity2;

                float path1Gain = angularGainProduct * mpc.Attenuation / (4f * math.PI * path1Distance);


                if (path1Gain < Threshold) continue;
                if (EnableEADF)
                {
                    float2 ueEADF = EADFReconstruction(entity1Direction, entity1RotationMatrix, Mu1, Mu2, EADF);
                    float2 rbsEADF = EADFReconstruction(entity2Direction, entity2RotationMatrix, Mu1, Mu2, EADF);
                    path1Gain *= math.length(ueEADF) * math.length(rbsEADF);
                }
                
                float2 antennaPatternTx = float2.zero;
                int2 thetaPhiTx = ComputeNLoSAngles(entity1Direction, entity1RotationMatrix);
                int ueId = LocalAntennaIds[linkId].x;
                AntennaPatternsUe.TryGetValue(thetaPhiTx + new int2(10000 * ueId), out antennaPatternTx);

                #region BS antenna pattern calcualtion
                float2 antennaPatternRx = float2.zero;
                int2 thetaPhiRx = ComputeNLoSAngles(entity2Direction, entity2RotationMatrix);
                int bsId = LocalAntennaIds[linkId].y;
                AntennaPatternsBs.TryGetValue(thetaPhiRx + new int2(10000 * bsId), out antennaPatternRx);
                #endregion
                
                
                Result.AddNoResize(new PathGainResultEntry(
                    mpcIndices: new int3(mpcId, 0, 0),
                    linkIndex: linkId,
                    pathGain: path1Gain * antennaPatternTx * antennaPatternRx,
                    pathDistance: path1Distance
                ));


            }


        }
    }
    public static int2 ComputeNLoSAngles(float3 nlosDir, float3x3 antRotation)
        {
            // Step 1: Convert LoS_dir into Ant1's local space using the inverse of its rotation
            float3 localDir = new float3 (math.dot(antRotation.c0, nlosDir), 
                math.dot(antRotation.c1, nlosDir), 
                math.dot(antRotation.c2, nlosDir));

            // Step 2: Convert to spherical coordinates
            float phi = math.degrees(math.atan2(-localDir.x, localDir.z));; // Longitude (XZ-plane), from Z-plane
            float theta = math.degrees(math.acos(math.clamp(localDir.y, -1f, 1f))); // Latitude (angle from Y-axis), clamped for numerical safety
            
            phi = math.fmod(phi + 360f, 360f);
            
            // TL
            int2 TL = new int2(Mathf.FloorToInt(theta), Mathf.CeilToInt(phi)); // TL = TopLeft corner of unit square for 3D Antenna Pattern approximation
            float weightTL = math.distance((float2)TL, new float2(theta, phi)); //  weightTL ~ euclidean distance between TL and actual antenna direction in global rotation
            // BL
            int2 BL = new int2(Mathf.FloorToInt(theta), Mathf.FloorToInt(phi)); // BL = BottomLeft corner of unit square for 3D Antenna Pattern approximation
            float weightBL = math.distance((float2)BL, new float2(theta, phi)); //  weightBL ~ euclidean distance between BL and actual antenna direction in global rotation
            // TR
            int2 TR = new int2(Mathf.CeilToInt(theta), Mathf.CeilToInt(phi)); // TR = TopRight corner of unit square for 3D Antenna Pattern approximation
            float weightTR = math.distance((float2)TR, new float2(theta, phi)); //  weightTR ~ euclidean distance between TR and actual antenna direction in global rotation
            // BR
            int2 BR = new int2(Mathf.CeilToInt(theta), Mathf.FloorToInt(phi)); // BR = BottomRight corner of unit square for 3D Antenna Pattern approximation
            float weightBR = math.distance((float2)BR, new float2(theta, phi)); //  weightBR ~ euclidean distance between BR and actual antenna direction in global rotation
        
            // Normalising factor - wighting factor of all four square Vertices: TL--BL--BR--TR
            float minWeight = math.min(math.min(weightTL, weightTR), math.min(weightBL, weightBR));
            int2 finalCorner = TL;
            if (minWeight == weightTR) finalCorner = TR;
            if (minWeight == weightBL) finalCorner = BL;
            if (minWeight == weightBR) finalCorner = BR;

            return finalCorner;
        }
    public struct PathGainResultEntry
    {
        public int3 MpcIndices;
        public int LinkIndex;
        public float2 PathGain;
        public float PathDistance;

        public PathGainResultEntry(int3 mpcIndices, int linkIndex, float2 pathGain, float pathDistance)
        {
            MpcIndices = mpcIndices;
            LinkIndex = linkIndex;
            PathGain = pathGain;
            PathDistance = pathDistance;
        }
    }

    public static float FastExp(float x)
    {
        x = math.clamp(x, -88.7f, 88.7f);
        float result = math.asfloat((int)(12102203.0f * x) + 127 * (1 << 23));
        return float.IsNaN(result) ? 0f : result;
    }

    public static float3 ImageCalculation(MPCFloat a, float3 entity)
    {
        float3 n1 = a.Normal, p1 = a.Point;
        // Calculating image of the next Mpc and its Normal
        float3 direction = entity - p1;
        float distancePlane = math.dot(n1, direction);
        float3 imageEntity = entity - 2f * distancePlane * n1;

        return imageEntity;
    }

    public (float3[] vertices, float3[] normals) AntennaMesh(int nHorizontal, int mVertical)
    {
        float3[] vertices = new float3[nHorizontal * mVertical];
        float3[] normals = new float3[nHorizontal * mVertical];
        
        float deltaPhi = 2 * math.PI / nHorizontal;
        float deltaTheta = math.PI / mVertical;
        
        for (int i = 0; i < nHorizontal*mVertical; i++)
        {
            int iRow = i % nHorizontal;
            int iColumn = i / nHorizontal;
            float phi = iRow * deltaPhi;
            float theta = iColumn * deltaTheta;
            float3 temp = RadiationValue(phi, theta);
            vertices[i] = temp;
            normals[i] = math.normalize(temp);
        }
        return (vertices, normals);
    }
    public float3 RadiationValue(float angle1, float angle2)
    {
        //_eadfArray
        int minRow = -(eadfRows - 1) / 2;
        int minColumn = -(eadfCols - 1) / 2;
        
        Complex radPat = new Complex(0, 0);
        for (int i = 0; i < eadfArray.Length; i++)
        {
            
            int iRow = i % eadfRows;
            int iColumn = i / eadfRows;
            float argument1 = angle1 * (minRow + iRow);
            float argument2 = angle2 * (minColumn + iColumn);
            
            Complex eadf = new Complex(eadfArray[i].x, eadfArray[i].y);
            Complex ai = new Complex(math.cos(argument1), math.sin(argument1));
            Complex bj = new Complex(math.cos(argument2), math.sin(argument2));
            Complex tempEADF = ai * bj * eadf;
            
            radPat += tempEADF;
        }
        float xLocal = (float)Complex.Abs(radPat) * math.sin(angle1);
        float yLocal = (float)Complex.Abs(radPat) * math.cos(angle1);

        float x3D = xLocal;
        float y3D = yLocal * math.sin(angle2);
        float z3D = yLocal * math.cos(angle2);
        
        return new float3(-y3D, z3D, -x3D);

    }

    public float3 getUEPosition(int userID)
    {
        return _entityCoordinates[userID];
    }
    
    public static float2 EADFReconstruction(float3 direction, float3x3 matrix, NativeArray<int> mu1, NativeArray<int> mu2, NativeArray<float2> EADF)
    {
        float cosTheta = math.dot(matrix.c1, direction);
            
        // calculate phi from atan2
        float yProj = math.dot(-matrix.c0, math.normalize(direction- cosTheta * matrix.c1));
        float xProj = math.dot(matrix.c2, math.normalize(direction - cosTheta * matrix.c1));
            
        float phi = math.atan2(yProj, xProj);
        float theta = math.acos(cosTheta);

        int pointId = 0;
        float2 result = new float2(0, 0);
        for (int i1 = 0; i1 < mu1.Length; i1++)
        {
            for (int i2 = 0; i2 < mu2.Length; i2++)
            {
                float2 eTheta = new float2(math.cos(theta*mu1[i1]), math.sin(theta*mu1[i1]));
                    
                float2 ePhi = new float2(math.cos(phi*mu2[i2]), math.sin(phi*mu2[i2]));
                    
                float2 temp = ComplexMul(EADF[pointId], eTheta);
                temp = ComplexMul(temp, ePhi);
                    
                result += temp;
                pointId += 1;
            }

                
        }
            
        return result;
    }
    
    private static float2 ComplexMul(float2 a, float2 b)
    {
        return new float2(
            a.x * b.x - a.y * b.y,
            a.x * b.y + a.y * b.x
        );
    }
    
    [BurstCompile]
    public struct EADFDrawJob : IJobParallelFor
    {

        [ReadOnly] public NativeArray<float2> ThetaPhi;
        [ReadOnly] public float3x3 RotationMatrix;
        
        [ReadOnly] public NativeArray<int> Mu1; // Theta, elevation: must be from -L1/2 to L1/2 - 1
        [ReadOnly] public NativeArray<int> Mu2; // Phi, azimuth: must be from -L2/2 to L2/2 - 1
        [ReadOnly] public NativeArray<float2> EADF; // EADF values read from csv file
        
        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<float3> Results; // results have size of mu1.Length
        

        public void Execute(int index)
        {
            
            
            float theta = ThetaPhi[index].x;
            float phi = ThetaPhi[index].y;

            float X = math.sin(theta) * math.cos(phi);  
            float Y = math.sin(theta) * math.sin(phi);  
            float Z = math.cos(theta);                  
            
            float3 direction = X * RotationMatrix.c2 - Y * RotationMatrix.c0 + Z * RotationMatrix.c1;
            float EADFAbs = math.length(DrawEADFResuls(theta, phi));
            Results[index] = EADFAbs * direction;
        }

        private float2 DrawEADFResuls(float _theta, float _phi)
        {
            

            float phi = _phi;
            float theta = _theta;
            
            int pointId = 0;
            float2 result = new float2(0, 0);
            for (int i1 = 0; i1 < Mu1.Length; i1++)
            {
                float2 eTheta = new float2(math.cos(theta*Mu1[i1]), math.sin(theta*Mu1[i1]));
                for (int i2 = 0; i2 < Mu2.Length; i2++)
                {
                    
                    float2 ePhi = new float2(math.cos(phi*Mu2[i2]), math.sin(phi*Mu2[i2]));
                    
                    float2 temp = ComplexMul(EADF[pointId], eTheta);
                    temp = ComplexMul(temp, ePhi);
                    
                    result += temp;
                    
                    pointId += 1;
                }
            }
            
            return result;
        }
        
    }
    
    [BurstCompile]
public struct EADF3DReconstruct : IJobParallelFor
    {

        [ReadOnly] public int AntennaID;
        [ReadOnly] public NativeArray<int2> ThetaPhi;
        [ReadOnly] public NativeArray<int> Mu1; // Theta, elevation: must be from -L1/2 to L1/2 - 1
        [ReadOnly] public NativeArray<int> Mu2; // Phi, azimuth: must be from -L2/2 to L2/2 - 1
        [ReadOnly] public NativeSlice<float2> EADF; // EADF values read from csv file
        
        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeParallelHashMap<int2, float2>.ParallelWriter AntennaPattern; // Look-up reconstruction of 3DEADF

        public void Execute(int index)
        {
            
            
            float theta = ThetaPhi[index].x*math.PI/180;
            float phi = ThetaPhi[index].y*math.PI/180;

            float X = math.sin(theta) * math.cos(phi); 
            float Y = math.sin(theta) * math.sin(phi);
            float Z = math.cos(theta);
            
            AntennaPattern.TryAdd(ThetaPhi[index]+new int2(10000 * AntennaID), Reconstruct3DEADF(theta, phi)); // we add new int2(10000 * AntennaID) to keep int2 hashmap key for computational efficiency
        }

        private float2 Reconstruct3DEADF(float _theta, float _phi)
        {
            

            float phi = _phi;
            float theta = _theta;
            
            int pointId = 0;
            float2 result = new float2(0, 0);
            for (int i1 = 0; i1 < Mu1.Length; i1++)
            {
                float2 eTheta = new float2(math.cos(theta*Mu1[i1]), math.sin(theta*Mu1[i1]));
                for (int i2 = 0; i2 < Mu2.Length; i2++)
                {
                    
                    float2 ePhi = new float2(math.cos(phi*Mu2[i2]), math.sin(phi*Mu2[i2]));
                    
                    float2 temp = ComplexMul(EADF[pointId], eTheta);
                    temp = ComplexMul(temp, ePhi);
                    
                    result += temp;
                    
                    pointId += 1;
                }
            }
            
            return result;
        }
        
    }
    
    #region Supporting functions
    
    public static NativeArray<int2> FindStartAndStopIndices(NativeArray<int> array, int outputLength)
    {
        NativeArray<int2> indicesArray = new NativeArray<int2>(outputLength, Allocator.TempJob);

        if (array.Length == 0 || outputLength == 0)
        {
            return indicesArray;
        }

        int startIndex = 0;
        int currentIndex = array[0];
        int outputIndex = 0;

        for (int i = 1; i < array.Length; i++)
        {
            if (array[i] != currentIndex)
            {
                indicesArray[outputIndex] = new int2(startIndex, i - 1);
                startIndex = i;
                currentIndex = array[i];
                outputIndex++;
            }
        }

        // Add the last range
        indicesArray[outputIndex] = new int2(startIndex, array.Length - 1);

        return indicesArray;
    }

    void SpawnRBS(float density, GameObject[] spawnVolumes, GameObject prefab, ref GameObject[] existingObjects, List<Vector3> relativePositionsList, List<string> spawnVolumeNamesList, GameObject parentObject, GameObject entitiesParentObject, string rbsTag)
    {
        List<GameObject> existingObjectList = new List<GameObject>(existingObjects);

        // Assuming entities are all direct children of the parent
        var entities = new List<GameObject>();
        foreach (Transform child in entitiesParentObject.transform)
        {
            entities.Add(child.gameObject);
        }

        foreach (var spawnVolume in spawnVolumes)
        {
            Vector3 volumeDimensions = spawnVolume.transform.localScale;
            float radius = Mathf.Min(volumeDimensions.x, volumeDimensions.y, volumeDimensions.z) / 2.0f;
            var spawnVolumeCenter = spawnVolume.transform.position;

            for (int i = 0; i < density; i++)
            {
                Vector3 spawnPosition;

                do
                {
                    Vector3 randomPositionOnUnitSphere = UnityEngine.Random.onUnitSphere;

                    // Determine the face of the spawn volume that is facing the entities
                    Vector3 spawnVolumeFaceNormal = Vector3.zero;
                    float minAngle = float.MaxValue;
                    foreach (var entity in entities)
                    {
                        Vector3 directionToEntity = (entity.transform.position - spawnVolumeCenter).normalized;
                        float angle = Vector3.Angle(directionToEntity, randomPositionOnUnitSphere);
                        if (angle < minAngle)
                        {
                            minAngle = angle;
                            spawnVolumeFaceNormal = randomPositionOnUnitSphere;
                        }
                    }

                    Vector3 scaledPosition = new Vector3(spawnVolumeFaceNormal.x * volumeDimensions.x / 2, spawnVolumeFaceNormal.y * volumeDimensions.y / 2, spawnVolumeFaceNormal.z * volumeDimensions.z / 2);
                    spawnPosition = spawnVolumeCenter + scaledPosition;
                }
                while (Physics.OverlapSphere(spawnPosition, radius, LayerMask.GetMask("avoid")).Length > 0);

                GameObject spawnedObject = Instantiate(prefab, spawnPosition, Quaternion.identity);
                spawnedObject.transform.SetParent(parentObject.transform);
                spawnedObject.tag = rbsTag;
                spawnedObject.name = System.Guid.NewGuid().ToString();

                Vector3 relativePosition = new Vector3(
                    (spawnPosition.x - spawnVolume.transform.position.x) / volumeDimensions.x,
                    (spawnPosition.y - spawnVolume.transform.position.y) / volumeDimensions.y,
                    (spawnPosition.z - spawnVolume.transform.position.z) / volumeDimensions.z
                );

                existingObjectList.Add(spawnedObject);
                relativePositionsList.Add(relativePosition);
                spawnVolumeNamesList.Add(spawnVolume.name);
            }
        }
        existingObjects = existingObjectList.ToArray();
    }

    void SaveGameObjectsToCSV(GameObject[] objects, string filePath, List<Vector3> relativePositionsList = null, List<string> spawnVolumeNamesList = null)
    {
        StringBuilder builder = new StringBuilder();
        builder.AppendLine("Name,PositionX,PositionY,PositionZ,RotationX,RotationY,RotationZ,RotationW,RelativePositionX,RelativePositionY,RelativePositionZ,SpawnVolumeName");

        for (int i = 0; i < objects.Length; i++)
        {
            Vector3 position = objects[i].transform.position;
            Quaternion rotation = objects[i].transform.rotation;

            // If relativePositionsList and spawnVolumeNamesList are not null, append their values, otherwise append "none"
            string relativePositionX = relativePositionsList != null ? relativePositionsList[i].x.ToString() : "none";
            string relativePositionY = relativePositionsList != null ? relativePositionsList[i].y.ToString() : "none";
            string relativePositionZ = relativePositionsList != null ? relativePositionsList[i].z.ToString() : "none";
            string spawnVolumeName = spawnVolumeNamesList != null ? spawnVolumeNamesList[i] : "none";

            builder.AppendLine($"{objects[i].name},{position.x},{position.y},{position.z},{rotation.x},{rotation.y},{rotation.z},{rotation.w},{relativePositionX},{relativePositionY},{relativePositionZ},{spawnVolumeName}");
        }

        File.WriteAllText(filePath, builder.ToString());
    }

    void InstantiateGameObjectsFromCSV(string filePath, string shiftFilePath, GameObject prefab, Vector3Int axesMapping, ref List<Vector3> relativePositionsList, ref List<string> spawnVolumeNamesList, GameObject parentObject, string rbsTag)
    {
        // Load the shift values from the CSV
        Dictionary<string, Vector3> shiftValues = new Dictionary<string, Vector3>();
        if (File.Exists(shiftFilePath))
        {
            string[] shiftData = File.ReadAllLines(shiftFilePath);
            for (int i = 1; i < shiftData.Length; i++)
            {
                string[] row = shiftData[i].Split(',');
                string name = row[0];
                Vector3 shift = new Vector3(float.Parse(row[1]), float.Parse(row[2]), float.Parse(row[3]));
                shiftValues[name] = shift;
            }
        }

        // Dictionary for existing child game objects of the parent object
        Dictionary<string, GameObject> existingObjects = new Dictionary<string, GameObject>();
        foreach (Transform child in parentObject.transform)
        {
            existingObjects[child.name] = child.gameObject;
        }

        if (File.Exists(filePath))
        {
            Debug.Log($"Loading GameObjects from file {filePath} with tag {rbsTag}");

            string[] data = File.ReadAllLines(filePath);
            for (int i = 1; i < data.Length; i++)  // Start from 1 to skip the header line
            {
                string[] row = data[i].Split(',');

                string name = row[0];
                Debug.Log($" - Loading {rbsTag}: {name}");

                // If the game object with this name already exists, skip the creation
                if (existingObjects.ContainsKey(name))
                    continue;

                Vector3 position = new Vector3(
                    float.Parse(row[1 + axesMapping.x]),
                    float.Parse(row[1 + axesMapping.y]),
                    float.Parse(row[1 + axesMapping.z])
                );
                Quaternion rotation = new Quaternion(float.Parse(row[4]), float.Parse(row[5]), float.Parse(row[6]), float.Parse(row[7]));

                GameObject obj = Instantiate(prefab, position, rotation);
                obj.transform.SetParent(parentObject.transform);
                obj.name = name;
                obj.tag = rbsTag;

                // Apply shift if this object has a shift value in the CSV
                if (shiftValues.ContainsKey(name))
                {
                    obj.transform.position += shiftValues[name];
                }

                if (row[8] != "none" && row[9] != "none" && row[10] != "none")
                {
                    Vector3 relativePosition = new Vector3(float.Parse(row[8]), float.Parse(row[9]), float.Parse(row[10]));
                    relativePositionsList.Add(relativePosition);
                }

                if (row[11] != "none")
                {
                    spawnVolumeNamesList.Add(row[11]);
                }
            }
        }
        else
        {
            Debug.LogError("Cannot find file at path: " + filePath);
        }
    }

    [BurstCompile]
    struct EllipseCalculationParallel : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> EntityCoordinates;
        [ReadOnly] public NativeArray<int2> ChannelLinks;
        [ReadOnly] public float HalfMaxDistance;

        [WriteOnly] public NativeArray<EllipseParameters> EllipseParallel;
        public void Execute(int index)
        {
            float3 entity1Coordinates = EntityCoordinates[ChannelLinks[index].x];
            float3 entity2Coordinates = EntityCoordinates[ChannelLinks[index].y];

            // converting to 2D
            float2 entity1 = new float2(entity1Coordinates.x, entity1Coordinates.z);
            float2 entity2 = new float2(entity2Coordinates.x, entity2Coordinates.z);

            float2 tx2rx = entity2 - entity1;
            float distanceTx2Rx = math.distance(entity2, entity1);
            float2 directionTx2Rx = tx2rx / distanceTx2Rx;

            float2 centerPoint = (entity1 + entity2) * 0.5f;

            // ellipse parameters
            float c = distanceTx2Rx * 0.5f;
            float aa = HalfMaxDistance * HalfMaxDistance;
            float a = 1f / (aa);
            float b = 1f / (aa - c * c);

            // rotation
            float cosA = directionTx2Rx.x;
            float sinA = Mathf.Sqrt(1f - cosA * cosA);

            // Determine rotation matrix sign based on direction
            float sign = directionTx2Rx.y < 0 ? 1f : -1f;
            float4 rotationMatrix = new Vector4(cosA, -sinA * sign, sinA * sign, cosA);

            EllipseParallel[index] = new EllipseParameters(a, b, centerPoint, rotationMatrix);
        }
    }


    [BurstCompile]
    public struct SortJob : IJob
    {
        public NativeArray<RayCastIndex> Data;
        public void Execute()
        {
            Data.Sort();
        }
    }

    [BurstCompile]
    public struct PrepareRaycastJob : IJob
    {
        public NativeArray<int2> MpcAndLinkArray;
        public NativeArray<RaycastCommand> RaycastCommands;
        public NativeArray<int> LinkBoundaries;

        [ReadOnly] public NativeArray<RayCastIndex> MpcRayCastLink;

        public void Execute()
        {
            int linkIndex = 0;
            for (int i = 0; i < MpcRayCastLink.Length; i++)
            {
                RaycastCommands[i] = MpcRayCastLink[i].Command;
                MpcAndLinkArray[i] = new int2(MpcRayCastLink[i].Index, MpcRayCastLink[i].LinkID);
                if (MpcRayCastLink[i].LinkID != linkIndex)
                {
                    LinkBoundaries[linkIndex] = i;
                    linkIndex = MpcRayCastLink[i].LinkID;
                }
            }
            LinkBoundaries[linkIndex] = MpcRayCastLink.Length;
        }
    }

    [BurstCompile]
    public struct RayCastIndex : IComparable<RayCastIndex>
    {
        public int Index;
        public RaycastCommand Command;
        public int LinkID;

        public RayCastIndex(int index, RaycastCommand command, int linkID)
        {
            Index = index;
            Command = command;
            LinkID = linkID;
        }
        public int CompareTo(RayCastIndex other)
        {
            // Compare by LinkID
            return LinkID.CompareTo(other.LinkID);
        }
    }

    [BurstCompile]
    struct InEllipseMpcsSearchAllLinks : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> EntityCoordinates;
        [ReadOnly] public NativeArray<int2> ChannelLinks;
        [ReadOnly] public NativeArray<EllipseParameters> EllipseInformation;
        [ReadOnly] public NativeArray<MPCFloat> Mpcs;

        public NativeList<RayCastIndex>.ParallelWriter RCEntity1Index;
        public NativeList<RayCastIndex>.ParallelWriter RCEntity2Index;

        public void Execute(int index)
        {
            // Extract indices
            int linkIndex = (int)math.floor(index / Mpcs.Length);
            int mpcIndex = index % Mpcs.Length;

            // Indices
            int entity1Index = ChannelLinks[linkIndex].x;
            int entity2Index = ChannelLinks[linkIndex].y;

            // Define the MPC coordinates
            float3 mpcCoordinates = Mpcs[mpcIndex].Point;
            // Define the MPC normal
            float3 mpcNorm = Mpcs[mpcIndex].Normal;

            // Define the MPC 2D coordinates
            float2 mpc2D = new float2(Mpcs[mpcIndex].Point.x, Mpcs[mpcIndex].Point.z);

            // Translate and Rotate MPC coordinates
            float2 moveMpc2D = (mpc2D - EllipseInformation[linkIndex].CenterPoint);
            float4 rotationM = EllipseInformation[linkIndex].RotationMatrix;
            float2 newMpc2D = new float2(
                moveMpc2D.x * rotationM.x + moveMpc2D.y * rotationM.y,
                moveMpc2D.x * rotationM.z + moveMpc2D.y * rotationM.w
            );

            // This code checks if the point lies within the ellipse
            float a = EllipseInformation[linkIndex].A;
            float b = EllipseInformation[linkIndex].B;
            float x = newMpc2D.x;
            float y = newMpc2D.y;

            if ((x * x) * a + (y * y) * b < 1)
            {
                // Raycasting parameters
                float3 entity1Coordinates = EntityCoordinates[entity1Index];
                float3 entity2Coordinates = EntityCoordinates[entity2Index];


                // // Backhit evaluation parameters:
                float3 entity12Mpc = mpcCoordinates - entity1Coordinates;   //  ue => mpc
                float3 entity22Mpc = mpcCoordinates - entity2Coordinates; // rbs => mpc


                if (math.dot(mpcNorm, math.normalize(entity12Mpc)) < -0.02f)
                {
                    // RaycastCommand from Ue to MPC
                    float distanceUe = math.distance(mpcCoordinates, entity1Coordinates);
                    float3 test = new float3(distanceUe, mpcIndex, 1f);
                    RaycastCommand raycastCommandUe = new RaycastCommand(entity1Coordinates, math.normalize(entity12Mpc), QueryParameters.Default, distanceUe); // updated

                    RayCastIndex rcIndex = new RayCastIndex(mpcIndex, raycastCommandUe, linkIndex);
                    RCEntity1Index.AddNoResize(rcIndex);
                }


                if (math.dot(mpcNorm, math.normalize(entity22Mpc)) < -0.02f)
                {
                    // RaycastCommand from Rbs to MPC
                    RaycastCommand raycastCommandRbs = new RaycastCommand(entity2Coordinates, math.normalize(entity22Mpc), QueryParameters.Default, math.distance(mpcCoordinates, entity2Coordinates)); // updated

                    RayCastIndex rcIndex = new RayCastIndex(mpcIndex, raycastCommandRbs, linkIndex);
                    RCEntity2Index.AddNoResize(rcIndex);
                }
            }
        }
    }

    
    public static void SaveParameters(ParametersRootObject parameters, string filePath)
    {
        string json = JsonUtility.ToJson(parameters, true);
        File.WriteAllText(filePath, json);
    }
    
    
    static void CopyMatrix(float[,] source, float[,] destination, int colOffset, int cols)
    {
        int rows = source.GetLength(0);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                destination[i, j + colOffset] = source[i, j];
            }
        }
    }
    
    static void CopyEmptyMatrix(float[,] source, float[,] destination, int colOffset, int cols)
    {
        int rows = source.GetLength(0);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                destination[i, j + colOffset] = 0.0f;
            }
        }
    }
    
    
    float[,] ReadCsvToMatrixOmitFirstRow(string filePath)
    {
        List<float[]> rows = new List<float[]>();
        bool isFirstRow = true;

        using (StreamReader reader = new StreamReader(filePath))
        {
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                // Skip the header row
                if (isFirstRow)
                {
                    isFirstRow = false;
                    continue;
                }

                string[] fields = line.Split(';');
                float[] row = Array.ConvertAll(fields, s => (float)double.Parse(s, CultureInfo.InvariantCulture));
                rows.Add(row);
            }
        }

        int rowCount = rows.Count;
        int colCount = rows[0].Length;

        float[,] matrix = new float[rowCount, colCount];
        for (int i = 0; i < rowCount; i++)
        for (int j = 0; j < colCount; j++)
            matrix[i, j] = rows[i][j];

        return matrix;
    }
    
    float2[] Read3DEADFfile(TextAsset antEadf)
    {
        
        string[] lines = antEadf.text.Split(new[] { "\r\n", "\r", "\n" }, System.StringSplitOptions.None);
        // Read dimensions from the first line
        string[] dimensions = lines[0].Split(',');
        int eadfRows = int.Parse(dimensions[0]);
        int eadfCols = int.Parse(dimensions[1]);
        
        Debug.Log($"3D EADF in AllVehicleControl Original matrix dimensions: {eadfRows} x {eadfCols}");

        int totalElements = eadfRows * eadfCols;
        float2[] eadfArray = new float2[totalElements];
            
            
        // Start reading data from the second line
        for (int i = 1; i <= totalElements; i++)
        {
            string[] values = lines[i].Split(',');
            float real = float.Parse(values[0], CultureInfo.InvariantCulture);
            float imag = float.Parse(values[1], CultureInfo.InvariantCulture);
            eadfArray[i - 1] = new float2(real, imag);
        }

        return eadfArray;
    }
    
    public static void SaveNativeParallelHashMapToBinary(NativeParallelHashMap<int2, float2> map, string path)
    {
        using BinaryWriter writer = new BinaryWriter(File.Open(path, FileMode.Create));
        writer.Write(map.Count());

        foreach (var kvp in map)
        {
            writer.Write(kvp.Key.x);
            writer.Write(kvp.Key.y);
            writer.Write(kvp.Value.x);
            writer.Write(kvp.Value.y);
        }
    }
    
    public static void LoadBinIntoExistingNativeParallelHashMap(string path, ref NativeParallelHashMap<int2, float2> targetMap)
    {
        using BinaryReader reader = new BinaryReader(File.OpenRead(path));
        int count = reader.ReadInt32();

        for (int i = 0; i < count; i++)
        {
            int kx = reader.ReadInt32();
            int ky = reader.ReadInt32();
            float vx = reader.ReadSingle();
            float vy = reader.ReadSingle();
            targetMap.TryAdd(new int2(kx, ky), new float2(vx, vy));
        }
    }
    
    #endregion
}

public struct GSCMspectrumChannel
{
    public List<int2> Links;
    public List<List<double>> RSSspectrumGain;
    public List<List<double>> RSSComplexChannel;
    public List<List<double>> RSSspectrumGainRl;
    public List<List<double>> RSSspectrumGainIm;
    public List<List<double>> EMIspectrumGain;
}


public struct GSCMstructECS
{
    public List<int2> Links;
    public List<double> RSS;
    public List<double> RSRP1;
    public List<double> RSRP2;
}