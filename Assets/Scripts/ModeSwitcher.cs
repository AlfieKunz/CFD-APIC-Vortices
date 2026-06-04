using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UI;

public class MethodSwitcher : MonoBehaviour {

    public enum SimulationMethod { APIC, RPIC, PIC, PICFLIP, FLIP }

    public float[] PICFLIPValues = new float[] { 0.0f, 0.0f, 0.0f, 0.95f, 1.0f };

    [Header("Current State")]
    public SimulationMethod currentMethod = SimulationMethod.APIC;
    public FluidSim FluidSimRef;

    public void SetAPIC(bool HasBeenSelected) {
        if (HasBeenSelected) { ChangeMethod(SimulationMethod.APIC); }
    }

    public void SetRPIC(bool HasBeenSelected) {
        if (HasBeenSelected) { ChangeMethod(SimulationMethod.RPIC); }
    }

    public void SetPIC(bool HasBeenSelected) {
        if (HasBeenSelected) { ChangeMethod(SimulationMethod.PIC); }
    }

    public void SetPICFLIP(bool HasBeenSelected) {
        if (HasBeenSelected) { ChangeMethod(SimulationMethod.PICFLIP); }
    }

    public void SetFLIP(bool HasBeenSelected) {
        if (HasBeenSelected) { ChangeMethod(SimulationMethod.FLIP); }
    }

    private void ChangeMethod(SimulationMethod newMethod) {
        currentMethod = newMethod;
        if (FluidSimRef != null) {
            switch (newMethod) {
                case SimulationMethod.PIC:
                case SimulationMethod.PICFLIP:
                case SimulationMethod.FLIP:
                    FluidSimRef.Settings.TransferMethod =  GlobalSettings.TransferMethodType.PIC;
                    FluidSimRef.Settings.PICFLIPRatio = PICFLIPValues[(int)newMethod];
                    break;
                case SimulationMethod.RPIC:
                    FluidSimRef.Settings.TransferMethod =  GlobalSettings.TransferMethodType.RigidPIC;
                    FluidSimRef.Settings.PICFLIPRatio = 0f;
                    break;
                case SimulationMethod.APIC:
                    FluidSimRef.Settings.TransferMethod =  GlobalSettings.TransferMethodType.AffinePIC;
                    FluidSimRef.Settings.PICFLIPRatio = 0f;
                    break;
            }
        }
    }
}