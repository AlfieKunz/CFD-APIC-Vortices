using UnityEngine;

public static class GlobalSettingsResetter {
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void OnRuntimeLoad() {
        GlobalSettings.ResetSettings();
    }
}