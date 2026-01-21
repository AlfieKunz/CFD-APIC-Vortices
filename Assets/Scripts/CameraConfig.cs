using UnityEngine;

//Controls for the camera operations, namely scaling the camera to ensure that all the grid cells fit correctly to the screen.
[RequireComponent(typeof(Camera))]
public class CameraController : MonoBehaviour
{
    private Camera cam;
    public static float WorldWidth { get; private set; }
    public static float WorldHeight { get; private set; }

    //Ability to reset the camera if the Global Settings have been changed.
    private void OnEnable() { GlobalSettings.SettingsChanged += HandleSettingsChanged; }
    private void OnDisable() { GlobalSettings.SettingsChanged -= HandleSettingsChanged; }
    private void HandleSettingsChanged(int SettingChangeCode) { if (SettingChangeCode == 3) { Init(); } }


    void Awake() {
        cam = GetComponent<Camera>();
        Init();
    }

    public void Init() {
        //Default settings for the camera.
        cam.orthographic = true;
        WorldHeight = 100f; //Arbitrary?
        WorldWidth = WorldHeight * cam.aspect;
        cam.orthographicSize = WorldHeight / 2f;
        transform.position = new Vector3(WorldWidth, WorldHeight, -10f); //z=-10 for proper displaying.
    }
}