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
using Unity.Jobs;
using Unity.Burst;
using Complex = System.Numerics.Complex;
using System.Linq;
using Unity.Mathematics;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System;
using System.Runtime.CompilerServices;



public static class ChannelH
{
    
    public static void FindLoSChannel(
        bool enableEADF,
        NativeParallelHashMap<int2, float2> allRadiationPattern,
        AntennaType antennaType1,
        AntennaType antennaType2,
        float maxMpc1ReflectionCoefficient,
        NativeArray<int2> channelLinkPairs,
        NativeArray<float3> entityCoordinates,
        NativeArray<float3x3> entityRotationMatices,
        int numFft,
        NativeArray<float> inverseWavelengths,
        int2 eadfSize,
        NativeArray<float2> EADF,
        NativeArray<int> mu1,
        NativeArray<int> mu2,
        NativeArray<Complex> H0
    )
    {
        NativeArray<RaycastCommand> raycastCommandsLoS = new NativeArray<RaycastCommand>(channelLinkPairs.Length, Allocator.TempJob);
        NativeArray<RaycastHit> raycastHitLoS = new NativeArray<RaycastHit>(channelLinkPairs.Length, Allocator.TempJob);

        // Ground reflection detection
        NativeArray<RaycastCommand> txRaycastCommandGround = new NativeArray<RaycastCommand>(channelLinkPairs.Length, Allocator.TempJob);
        NativeArray<RaycastCommand> rxRaycastCommandGround = new NativeArray<RaycastCommand>(channelLinkPairs.Length, Allocator.TempJob);
        NativeArray<RaycastHit> txRaycastHitGround = new NativeArray<RaycastHit>(channelLinkPairs.Length, Allocator.TempJob);
        NativeArray<RaycastHit> rxRaycastHitGround = new NativeArray<RaycastHit>(channelLinkPairs.Length, Allocator.TempJob);

        // Angular factor for the ground reflection
        // NativeArray<float> groundAngularFactor = new NativeArray<float>(channelLinkPairs.Length, Allocator.TempJob);  

        ParallelLoSDetection loSDetection = new ParallelLoSDetection
        {
            Links = channelLinkPairs,
            EntityCoordinates = entityCoordinates,
            RayCastcommands = raycastCommandsLoS,

            TxRaycastGround = txRaycastCommandGround,
            RxRaycastGround = rxRaycastCommandGround
        };

        // Create all commands in parallel
        JobHandle loSDetectionHandle = loSDetection.Schedule(channelLinkPairs.Length, 4); // Batch size adjusted for efficiency
        loSDetectionHandle.Complete();

        // Run ray-cast commands
        JobHandle raycastJobLoS = RaycastCommand.ScheduleBatch(raycastCommandsLoS, raycastHitLoS, 64, default); // Batch size adjusted for efficiency
        raycastJobLoS.Complete();

        // Run ray-cast commands for ground reflection
        JobHandle txRaycastGroundJob = RaycastCommand.ScheduleBatch(txRaycastCommandGround, txRaycastHitGround, 64, default); // Batch size adjusted for efficiency
        txRaycastGroundJob.Complete();

        JobHandle rxRaycastGroundJob = RaycastCommand.ScheduleBatch(rxRaycastCommandGround, rxRaycastHitGround, 64, default); // Batch size adjusted for efficiency
        rxRaycastGroundJob.Complete();


        for (int i = 0; i < channelLinkPairs.Length; i++)
        {

            if (raycastHitLoS[i].distance == 0)
            {
                Vector3 entity1AntennaPosition = entityCoordinates[channelLinkPairs[i].x];
                Vector3 entity2AntennaPosition = entityCoordinates[channelLinkPairs[i].y];
                Debug.DrawLine(entity1AntennaPosition, entity2AntennaPosition, Color.yellow);
            }


            if (txRaycastHitGround[i].distance + rxRaycastHitGround[i].distance > txRaycastCommandGround[i].distance - 0.5f)
            {
                Vector3 entity1AntennaPosition = entityCoordinates[channelLinkPairs[i].x];
                Vector3 entity2AntennaPosition = entityCoordinates[channelLinkPairs[i].y];
                Debug.DrawLine(entity1AntennaPosition, txRaycastHitGround[i].point, Color.red);
                Debug.DrawLine(entity2AntennaPosition, rxRaycastHitGround[i].point, Color.red);
            }

        }
        
        
        // LoS channel calculation
        ParalleLoSChannel paralleLoSChannel = new ParalleLoSChannel
        {
            AntennaPatterns = allRadiationPattern,
            
            AntennaType1 = antennaType1,
            AntennaType2 = antennaType2,
            ChannelLinkPairs = channelLinkPairs,
            EntityCoordinates = entityCoordinates,
            EntityRotationMatrices = entityRotationMatices,
            RaycastHits = raycastHitLoS,
            RaycastCommand = raycastCommandsLoS,
            // start ground reflection
            TxRaycastHitGround = txRaycastHitGround,
            TxRaycastCommandGround = txRaycastCommandGround,
            RxRaycastHitGround = rxRaycastHitGround,
            RxRaycastCommandGround = rxRaycastCommandGround,
            GroundReflectionCoefficient = maxMpc1ReflectionCoefficient,
            // end ground reflection

            FourPi = 4 * math.PI,
            Nfft = numFft,
            InverseLambdas = inverseWavelengths,
            
            
            // Output
            HLoS = H0
        };

        // Create all LoS channel calculation commands in parallel
        JobHandle paralleLoSChannelHandle = paralleLoSChannel.Schedule(numFft * channelLinkPairs.Length, 64); // Batch size adjusted for efficiency
        paralleLoSChannelHandle.Complete();
        
        raycastCommandsLoS.Dispose();
        raycastHitLoS.Dispose();
        txRaycastCommandGround.Dispose();
        rxRaycastCommandGround.Dispose();
        txRaycastHitGround.Dispose();
        rxRaycastHitGround.Dispose();
        //found.Dispose();

        //return numLoS;
    }
    
    [BurstCompile]
    public struct EADFReconstructionJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int2> ChannelLinkPairs;
        [ReadOnly] public NativeArray<float3x3> EntityRotationMatrices;

        // LoS: direcion Tx => Rx
        [ReadOnly] public NativeArray<RaycastHit> RaycastHits;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommand;

        // Tx ground
        [ReadOnly] public NativeArray<RaycastHit> TxRaycastHitGround;
        [ReadOnly] public NativeArray<RaycastCommand> TxRaycastCommandGround;

        // Rx ground
        [ReadOnly] public NativeArray<RaycastHit> RxRaycastHitGround;
        [ReadOnly] public NativeArray<RaycastCommand> RxRaycastCommandGround;
        
        [ReadOnly] public NativeArray<int> Mu1; // Theta, elevation: must be from -L1/2 to L1/2 - 1
        [ReadOnly] public NativeArray<int> Mu2; // Phi, azimuth: must be from -L2/2 to L2/2 - 1
        [ReadOnly] public int2 EADFSize; // L1, L2
        [ReadOnly] public NativeArray<float2> EADF; // EADF values read from csv file
        
        [NativeDisableParallelForRestriction]
        [WriteOnly] public NativeArray<float2> ResultsLoS; // results have size of mu1.Length
        [WriteOnly] public NativeArray<float2> ResultsGround; // results have size of mu1.Length

        public void Execute(int index)
        {
            int i1 = index % EADFSize.y; // Theta, elevation, slice
            int linkId = index / EADFSize.y;
            
            int2 linkPair = ChannelLinkPairs[linkId];
            float3x3 txRotationMatrix = EntityRotationMatrices[linkPair.x];
            float3x3 rxRotationMatrix = EntityRotationMatrices[linkPair.y];
            
            // LoS
            if (RaycastHits[linkId].distance != 0)
            {
                ResultsLoS[index] = new float2(0, 0);
            }
            else
            {
                float2 txPartialResult = CalculateEADFResuls(RaycastCommand[linkId].direction, txRotationMatrix, i1);
                float2 rxPartialResult = CalculateEADFResuls(-RaycastCommand[linkId].direction, rxRotationMatrix, i1);

                ResultsLoS[index] = ComplexMul(txPartialResult, rxPartialResult);
            }
            // Ground
            if (TxRaycastHitGround[linkId].distance + RxRaycastHitGround[linkId].distance < TxRaycastCommandGround[linkId].distance - 0.5f)
            {
                ResultsGround[index] = new float2(0, 0);
            }
            else
            {
                float2 txGroundPartialResult = CalculateEADFResuls(TxRaycastCommandGround[linkId].direction, txRotationMatrix, i1);
                float2 rxGroundPartialResult = CalculateEADFResuls(RxRaycastCommandGround[linkId].direction, rxRotationMatrix, i1);

                ResultsGround[index] = ComplexMul(txGroundPartialResult, rxGroundPartialResult);
            }
        }

        private float2 CalculateEADFResuls(float3 direction, float3x3 matrix, int i1)
        {
            float cosTheta = math.dot(matrix.c1, direction);
            float3 normalizedDiff = math.normalize(direction - cosTheta * matrix.c1);
            float yProj = math.dot(-matrix.c0, normalizedDiff);
            float xProj = math.dot(matrix.c2, normalizedDiff);

            float phi = math.atan2(yProj, xProj);
            float theta = math.acos(cosTheta);
            
            // Cache theta trig values
            float2 eTheta = new float2(math.cos(theta * Mu1[i1]), math.sin(theta * Mu1[i1]));
            
            float2 partialResult = float2.zero;
            int baseIndex = i1 * Mu2.Length;

            for (int i2 = 0; i2 < Mu2.Length; i2++)
            {
                float2 ePhi = new float2(math.cos(phi * Mu2[i2]), math.sin(phi * Mu2[i2]));

                float2 temp = ComplexMul(EADF[baseIndex + i2], eTheta);
                temp = ComplexMul(temp, ePhi);

                partialResult += temp;
            }

            return partialResult;
        }
        
    }
    
    [BurstCompile]
    public struct ParalleLoSChannel : IJobParallelFor
    {
        [ReadOnly] public NativeParallelHashMap<int2, float2> AntennaPatterns;

        [ReadOnly] public AntennaType AntennaType1;
        [ReadOnly] public AntennaType AntennaType2;

        [ReadOnly] public NativeArray<int2> ChannelLinkPairs;
        [ReadOnly] public NativeArray<float3> EntityCoordinates;
        [ReadOnly] public NativeArray<float3x3> EntityRotationMatrices;

        // LoS: direcion Tx => Rx
        [ReadOnly] public NativeArray<RaycastHit> RaycastHits;
        [ReadOnly] public NativeArray<RaycastCommand> RaycastCommand;

        // Tx ground
        [ReadOnly] public NativeArray<RaycastHit> TxRaycastHitGround;
        [ReadOnly] public NativeArray<RaycastCommand> TxRaycastCommandGround;

        // Rx ground
        [ReadOnly] public NativeArray<RaycastHit> RxRaycastHitGround;
        [ReadOnly] public NativeArray<RaycastCommand> RxRaycastCommandGround;


        [ReadOnly] public float GroundReflectionCoefficient;
        [ReadOnly] public float FourPi;
        [ReadOnly] public int Nfft;
        [ReadOnly] public NativeArray<float> InverseLambdas;
        

        [WriteOnly] public NativeArray<Complex> HLoS;


        public void Execute(int index)
        {
            int i_link = index / Nfft;
            int i_sub = index % Nfft;

            if (RaycastHits[i_link].distance == 0)
            {
                int ueId = ChannelLinkPairs[i_link].x;
                int bsId = ChannelLinkPairs[i_link].y;

                int2 linkPair = ChannelLinkPairs[i_link];
                float3 txPos = EntityCoordinates[linkPair.x];
                float3x3 txRotationMatrix = EntityRotationMatrices[linkPair.x];

                float3 rxPos = EntityCoordinates[linkPair.y];
                float3x3 rxRotationMatrix = EntityRotationMatrices[linkPair.y];

                Complex losInfluence = CalculateLoSInfluence(i_link, i_sub, txPos, txRotationMatrix, rxPos,
                    rxRotationMatrix, AntennaType1, AntennaType2);
                Complex groundInfluence = CalculateGroundInfluence(i_link, i_sub, txRotationMatrix, rxRotationMatrix,
                    AntennaType1, AntennaType2);


                float3 asd = math.normalize(rxPos - txPos);

                HLoS[index] = losInfluence + groundInfluence;
            }
            else
            {
                HLoS[index] = Complex.Zero; // No LoS path
            }
        }
        
        
        private Complex CalculateLoSInfluence(int i_link, int i_sub, float3 txPos, float3x3 txMtrx, float3 rxPos, float3x3 rxMtrx, AntennaType antennaType1, AntennaType antennaType2)
        {
            if (RaycastHits[i_link].distance != 0) return Complex.Zero;

            #region Antenna radiation pattern
            // Direction of the LoS: Tx => Rx
            float3 LoSDirection = RaycastCommand[i_link].direction; // direction
            
            int2 thetaPhiTx = ComputeLoSAngles( LoSDirection, txMtrx);
            int ueId = ChannelLinkPairs[i_link].x - 1;
            AntennaPatterns.TryGetValue(thetaPhiTx + new int2(10000 * ueId), out float2 funcAntennaGain);
            

            #endregion


            float distance = math.distance(txPos, rxPos);
            float inverseLambda = InverseLambdas[i_sub];
            float2 gainLoS = funcAntennaGain / (inverseLambda * FourPi * distance);

            
            
            float phase = 2 * math.PI * inverseLambda * distance;
            float2 expPhase = new float2(math.cos(phase), -math.sin(phase));
            
            float2 result = ComplexMul(gainLoS, expPhase);
            return new Complex(result.x, result.y);
        }
        
        

        private Complex CalculateGroundInfluence(int i_link, int i_sub, float3x3 txMtrx, float3x3 rxMtrx, AntennaType antennaType1, AntennaType antennaType2)
        {
            float distanceGroundReflection = TxRaycastCommandGround[i_link].distance;
            if (TxRaycastHitGround[i_link].distance + RxRaycastHitGround[i_link].distance < distanceGroundReflection - 0.5f)
                return Complex.Zero;

            #region Antenna radiation pattern

            float3 txGroundDirection = TxRaycastCommandGround[i_link].direction;
            float3 rxGroundDirection = RxRaycastCommandGround[i_link].direction;

            int2 thetaPhiTx = ComputeLoSAngles( txGroundDirection, txMtrx);
            int ueId = ChannelLinkPairs[i_link].x - 1;
            AntennaPatterns.TryGetValue(thetaPhiTx + new int2(10000 * ueId), out float2 funcAntennaGainGround);


            #endregion

            float inverseLambda = InverseLambdas[i_sub];
            float3 direction = TxRaycastCommandGround[i_link].direction;
            float groundAngularFactor = 1f + direction.y * (1f - GroundReflectionCoefficient);


            float2 gainGround = groundAngularFactor * funcAntennaGainGround / (inverseLambda * FourPi * distanceGroundReflection);

            if (Double.IsNaN(gainGround.x) || Double.IsNaN(gainGround.y))
            {
                //Debug.LogWarning($"NaN detected in gainGround for link {i_link}, subcarrier {i_sub}. Setting gain to zero.");
                gainGround = new float2(0, 0);
            }
            {
                gainGround = 0;
            }

            float phase = 2 * math.PI * inverseLambda * distanceGroundReflection;
            float2 expPhase = new float2(math.cos(phase), -math.sin(phase));
            float2 result = ComplexMul(gainGround, expPhase);
            return new Complex(result.x, result.y);
        }
        // Compute Theta and Phi Given LoS Direction and Antenna Rotation (Quaternion)
        private static int2 ComputeLoSAngles(float3 losDir, float3x3 antRotation)
        {
            // Step 1: Convert LoS_dir into Ant1's local space using the inverse of its rotation
            float3 localDir = new float3 (math.dot(antRotation.c0, losDir), 
                math.dot(antRotation.c1, losDir), 
                math.dot(antRotation.c2, losDir));

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
    }

    [BurstCompile]
    public struct ParallelLoSDetection : IJobParallelFor
    {
        [ReadOnly] public NativeArray<int2> Links;
        [ReadOnly] public NativeArray<float3> EntityCoordinates;

        [WriteOnly] public NativeArray<RaycastCommand> RayCastcommands;
        [WriteOnly] public NativeArray<RaycastCommand> TxRaycastGround;
        [WriteOnly] public NativeArray<RaycastCommand> RxRaycastGround;

        public void Execute(int index)
        {
            float3 txAntennaPosition = EntityCoordinates[Links[index].x];
            float3 rxAntennaPosition = EntityCoordinates[Links[index].y];
            // LoS ray-cast
            Vector3 direction = rxAntennaPosition - txAntennaPosition;
            RayCastcommands[index] = new RaycastCommand(txAntennaPosition, direction.normalized, QueryParameters.Default, direction.magnitude);

            // Ground reflection: 2 ray-casts
            Vector3 txDirection = new float3(rxAntennaPosition.x, -rxAntennaPosition.y, rxAntennaPosition.z) - txAntennaPosition; // Tx => Ground
            Vector3 rxDirection = new float3(txAntennaPosition.x, -txAntennaPosition.y, txAntennaPosition.z) - rxAntennaPosition; // Rx => Ground

            TxRaycastGround[index] = new RaycastCommand(txAntennaPosition, txDirection.normalized, QueryParameters.Default, txDirection.magnitude);
            RxRaycastGround[index] = new RaycastCommand(rxAntennaPosition, rxDirection.normalized, QueryParameters.Default, rxDirection.magnitude);
        }
    }

    
    private static float2 ComplexMul(float2 a, float2 b)
    {
        return new float2(
            a.x * b.x - a.y * b.y,
            a.x * b.y + a.y * b.x
        );
    }

    
}


public enum AntennaType
{
    CosSinSquare,
    Omni,
    Patch
}


