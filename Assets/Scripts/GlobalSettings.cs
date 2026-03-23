using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using System.IO.Compression;
using Unity.Mathematics;
using Unity.VisualScripting.Dependencies.NCalc;
using UnityEngine;
using System;

//Class containing all the core settings used by the program. Accessible through the GlobalObjects asset.
[CreateAssetMenu(fileName = "GlobalGameSettings", menuName = "Game Configuration/Global Settings")]
public class GlobalSettings : ScriptableObject {

    //Controls for the rest of the program to be able to access these values.
    [NonSerialized]
    private GlobalSettings Previous; //Previous State of Settings (for use in comparisons).

    [NonSerialized]
    private static bool JustReset = false;

    public static void ResetSettings() {
        SettingsChanged = null;
        JustReset = true;
    }


    public static event Action<int> SettingsChanged;
    private void OnValidate() {
#if UNITY_EDITOR
        //Settings have changed. Check if this is due to the program starting, or a mid-simulation change.
        if (JustReset || Previous == null) {
            //Need to be instantiated - create a new instance for the Previous State.
            Previous = CreateInstance<GlobalSettings>();
            JustReset = false;
        } else {
            //Checks if a core, or non-core setting has been changed (requiring the need to reset the simulation, or just alter the physics in real time).
            //A code of 1 represents an update to system timings (non-essential), 2 represents a non-grid related change, requiring only the particles
            // to be redrawn, whilst 3 requires a full simulation reset.
            int SimResetCode = 0;
            if (SyncTimeStepToSystem != Previous.SyncTimeStepToSystem || (SyncTimeStepToSystem && TimeStep != Previous.TimeStep)) SimResetCode = 1;
            else if (ParticleCount != Previous.ParticleCount) SimResetCode = 2;
            else if (math.any(GridDimensions != Previous.GridDimensions) || SpawnType != Previous.SpawnType || PeriodicBCs != Previous.PeriodicBCs) SimResetCode = 3;
            SettingsChanged?.Invoke(SimResetCode);
        }
        //Make a new instance, in a way that won't recursively call OnValidate again :).
        string json = JsonUtility.ToJson(this);
        JsonUtility.FromJsonOverwrite(json, Previous);
#endif
    }


    public bool isRunning = false; //As in, the simulation itself.

    public int2 GridDimensions = new int2(80, 45); //How many grid cells.

    [Min(1)]
    public int ParticleCount = 1000; //Number of particles to simulate.

    public bool RenderParticles = true;

    [Range(0.001f, 0.1f)]
    public float TimeStep = 1 / 60f; //Step between each Simulation Frame (in which two iterations are performed).
    public bool SyncTimeStepToSystem = true;

    public bool UpdateTimeStepSafety = true;
    public bool PeriodicBCs = true;

    [Range(0, 1)]
    public float ProjectionStepSize = 0.0005f;

    [Range(-1, 10)]
    public float Stiffness = 1f;


    [Min(0)]
    public float gMagnitude = 9.81f; //Strength of gravity.

    [Range(0f, 360f)]
    public float gDirection = 270; //Direction of gravity (with 0 = +x, anti-clockwise convention).

    public bool MouseGravity = false; //Allows the user to make their mouse a 'black hole' :D.

    [Range(0f, 1f)]
    public float PICFLIPRatio = 0.9f; //Specifies the ratio of the PIC to FLIP model we apply to the system.

    [Range(0f, 10f)]
    public float MouseRadius = 4f; //Specifies how many grid cells will be affected by the user's mouse, as a multiple of CellSize.

    [Range(0f, 100f)]
    public float MouseStrength = 10f; //Denotes the maximum magnitude of the force applied to grid cells, from the user's mouse.


    public enum ParticleSpawnType {
        LHSSquare,
        LRRectangle,
        CentreSquare,
        CentreBottomRectangle,
        CentreCircle,
        Wave,
        Random
    }
    public ParticleSpawnType SpawnType = ParticleSpawnType.LHSSquare;
    [Range(-5f, 5f)]
    public float InitialAngularMomentum = 0f;
    [Min(0)]
    public float InitialTGVVelocity = 0f;

    public enum TransferMethodType {
        PIC,
        RigidPIC,
        AffinePIC

    }
    public TransferMethodType TransferMethod = TransferMethodType.PIC;
}