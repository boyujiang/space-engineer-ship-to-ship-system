using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

// ====================================================================
//  MOTHERGRID FIRE CONTROL RADAR SYSTEM (IFF & TRACKING CORE)
// ====================================================================

        const string CAMERA_NAME_TAG = "Camera_Tracker";
        const string RADIOTAG = "WenYue"; 
        const string LCD_NAME = "LCD_Radar_Log"; 

        IMyCameraBlock trackerCamera;
        IMyTextPanel logLCD;
        Vector3D lastKnownTargetPos = Vector3D.Zero;
        double trackDistance = 1000; 
        bool isTracking = false;

        System.Text.StringBuilder lcdBuffer = new System.Text.StringBuilder();

        public Program()
        {
            // Set execution rate to every 100 frames (~0.16 seconds) 
            // This provides responsive vector streaming while conserving CPU cycles.
            Runtime.UpdateFrequency = UpdateFrequency.Update10;
        }

        void InitializeBlocks()
        {
            // Initialize targeting optics
            List<IMyCameraBlock> cameras = new List<IMyCameraBlock>();
            GridTerminalSystem.GetBlocksOfType(cameras, c => c.CustomName.Contains(CAMERA_NAME_TAG));
            if (cameras.Count > 0)
            {
                trackerCamera = cameras[0];
                trackerCamera.EnableRaycast = true; 
            }

            // Initialize display telemetry
            logLCD = GridTerminalSystem.GetBlockWithName(LCD_NAME) as IMyTextPanel;
            if (logLCD != null)
            {
                logLCD.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
            }
        }

        void Log(string message)
        {
            Echo(message); 
            lcdBuffer.AppendLine(message); 
        }

        public void Main(string argument, UpdateType updateSource)
        {
            lcdBuffer.Clear();

            if (trackerCamera == null || logLCD == null)
            {
                InitializeBlocks();
            }

            // Telemetry UI Header
            lcdBuffer.AppendLine("======= FIRE CONTROL RADAR SYSTEM =======");
            lcdBuffer.AppendLine($"STATUS: {(isTracking ? "[LOCKED - TRACKING]" : "[STANDBY - AWAITING ACQUISITION]")}");
            lcdBuffer.AppendLine("-----------------------------------------");

            if (trackerCamera == null) { Log("ERROR: Local targeting camera not found."); FlushLCD(); return; }
            if (logLCD == null) { Echo($"WARNING: Target telemetry screen [{LCD_NAME}] missing."); }

            if (argument.ToUpper() == "LOCK")
            {
                StartInitialLock();
                FlushLCD();
                return;
            }

            if (isTracking)
            {
                ContinueTrackingLoop();
            }

            FlushLCD();
        }

        void FlushLCD()
        {
            if (logLCD != null)
            {
                logLCD.WriteText(lcdBuffer.ToString());
            }
        }

        // 1. Initial Target Acquisition (Forced IFF Check)
        void StartInitialLock()
        {
            double initialScanDist = 1000;
            Log($"Initiating forward scan vector: {initialScanDist}m...");
            
            if (trackerCamera.CanScan(initialScanDist))
            {
                MyDetectedEntityInfo info = trackerCamera.Raycast(initialScanDist, 0, 0); 
                
                if (!info.IsEmpty() && (info.Type == MyDetectedEntityType.LargeGrid || info.Type == MyDetectedEntityType.SmallGrid))
                {
                    // IFF Core Filter: Abort if target is not a confirmed threat vector
                    if (info.Relationship != MyRelationsBetweenPlayerAndBlock.Enemies)
                    {
                        isTracking = false;
                        Log("\n[IFF INTERCEPT]: Lock authorization DENIED!");
                        Log($"Reason: Target classified as [{info.Relationship}]. Not tagged as Enemies.");
                        return;
                    }

                    lastKnownTargetPos = info.HitPosition.Value;
                    trackDistance = Vector3D.Distance(trackerCamera.GetPosition(), lastKnownTargetPos);
                    isTracking = true;
                    
                    Log("\n[TARGET ACQUIRED]");
                    Log($"ID: {info.Name}");
                    Log($"Range: {trackDistance:F1}m");
                    Log("IFF Class: Enemies");
                }
                else
                {
                    Log("\nAcquisition failed: No valid target grid in crosshairs.");
                }
            }
            else
            {
                Log("\nOptics raycast charging... Retrying on next tick.");
            }
        }

        // 2. Continuous Active Tracking Loop
        void ContinueTrackingLoop()
        {
            double nextScanDist = Vector3D.Distance(trackerCamera.GetPosition(), lastKnownTargetPos) + 100;

            if (trackerCamera.CanScan(nextScanDist))
            {
                MyDetectedEntityInfo info = trackerCamera.Raycast(nextScanDist, 0, 0);;

                if (!info.IsEmpty())
                {
                    // Continuous IFF Verification (Handles target ownership transfers or neutralization)
                    if (info.Relationship != MyRelationsBetweenPlayerAndBlock.Enemies)
                    {
                        isTracking = false;
                        Log("\n[TRACK ABORTED]: Target safety profile changed. Ceasing fire sequence.");
                        return;
                    }

                    lastKnownTargetPos = info.HitPosition.Value; 
                    string serializedPos = lastKnownTargetPos.ToString();

                    // Broadcast verified coordinate package to mid-flight missile networks
                    IGC.SendBroadcastMessage(RADIOTAG, serializedPos);

                    trackDistance = Vector3D.Distance(trackerCamera.GetPosition(), lastKnownTargetPos);
                    Log($"Target: {info.Name}");
                    Log($"Range: {trackDistance:F1}m");
                    Log("Comms Link: Streaming data package... OK");
                    Log($"Vector Data:\n {serializedPos}");
                }
                else
                {
                    isTracking = false;
                    Log("\n[WARNING]: Target broke radar illumination arc. LOCK LOST.");
                }
            }
            else
            {
                Log("Optics power low. Maintaining inertial track matrix...");
            }
        }
    }
}
