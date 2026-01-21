using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using UnityEditor;
using System;

//Adds custom buttons to the GlobalSettings asset.
[CustomEditor(typeof(GlobalSettings))]
public class CustomInspector : Editor {
    public override void OnInspectorGUI() {
        //Draws all the Global Settings controls.
        DrawDefaultInspector();
        EditorGUILayout.Space();

        //Runs specific methods in the FluidsSim script.
        FluidSim TargetScript = FindObjectOfType<FluidSim>();
        EditorGUILayout.LabelField("Total Time:", TargetScript.SimulationTime.ToString("F2") + "s");
        if (GUILayout.Button("Reset Simulation")) {TargetScript.Init(); }
        else if (GUILayout.Button("Run Simulation")) {TargetScript.Run(); }
        else if (GUILayout.Button("Add Angular Force")) {TargetScript.AddAngularMomentum(); }
        else if (GUILayout.Button("Add Shear Force")) {TargetScript.AddShear(); }
    }
}
