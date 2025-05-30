using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using PassthroughCameraSamples;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class AprilTagReader : MonoBehaviour
{
    [SerializeField] private AprilTagWrapper m_aprilTagWrapper;
    [Tooltip("Size of the AprilTag (edge-to-edge) in meters, e.g. 0.1 for 10cm.")]
    [SerializeField] private float tagSize;
    [SerializeField] private AprilTagFamily tagFamily;
    [SerializeField] private Vector2Int tagResolution = new(640, 480);
    [SerializeField] public PassthroughCameraPermissions CameraPermissions;
    
    private bool running;
    private WebCamTexture tex;
    bool _running = true;
    private bool newFrameAvailable;
    private List<AprilTagPose> latestTags = new();
    private bool tagsUpdated;
    private Vector3 targetPos;
    
    private static Vector3 camToWorldRotation = new(90, 180, 0);
    
    private Dictionary<int, AprilTagWorldInfo> tagWorldData = new();

    private bool m_hasPermission;
    
    Stopwatch sw = Stopwatch.StartNew();
    
    private void Awake()
    {
#if UNITY_ANDROID
        Debug.Log($"[AprilTag] Debug Hangup! Awake");
        CameraPermissions.AskCameraPermissions();
#endif
    }
    
    private IEnumerator Start()
    {
        while(PassthroughCameraPermissions.HasCameraPermission != true){
            yield return null;
        }
        Debug.Log($"[AprilTag] Debug Hangup! Permissions Granted");
        m_hasPermission = true;
        StartAprilTagDetector();
    }
    
    private void StartAprilTagDetector()
    {
        m_aprilTagWrapper.Init(tagSize, tagFamily, tagResolution.x, tagResolution.y);
        running = true;
    }
    
    void OnDestroy() {
        _running = false;
    }
    
    private void OnApplicationPause(bool paused)
    {
        if (paused)
        {
            m_aprilTagWrapper.StopCamera();
            _running = false;
            Debug.Log("App Paused - Disabling VR tracking...");
        }
        else
        {
            if(!_running){
                StartAprilTagDetector();
            }
            Debug.Log("App Resumed - Re-enabling VR tracking...");
        }
    }

    private void OnApplicationFocus(bool focus)
    {
        if (focus)
        {
            Debug.Log("App Gained Focus - Checking Tracking...");
        }
    }
    
    void Update() {
        
        if (running){
            sw = Stopwatch.StartNew();
            Debug.Log($"--> START update loop: {sw.Elapsed.TotalMilliseconds}");
            latestTags = m_aprilTagWrapper.GetLatestPoses();
            Debug.Log($"--> Gotten latest poses: {sw.Elapsed.TotalMilliseconds}");
            Debug.Log($"[AprilTag] Detector Running: {latestTags.Count} tags detected");
            // Apply latest tag transforms on main thread
            if (latestTags.Count > 0) {
                foreach (var aprilTag in latestTags) {
                    var tagID = aprilTag.id;
                    
                    var tagPosZ = aprilTag.position.z;
                    Debug.Log($"[AprilTag] NEW! Tag Z Raw Position is: ({tagPosZ})");

                    if (!tagWorldData.ContainsKey(tagID))
                        tagWorldData[tagID] = new AprilTagWorldInfo();
                    
                    tagWorldData[tagID].tagPose = TagFromCamera(aprilTag);
                    Debug.Log($"[AprilTag] Tag Position in Camera View is: ({tagWorldData[tagID].tagPose.GetPosition().x}, {tagWorldData[tagID].tagPose.GetPosition().y}, {tagWorldData[tagID].tagPose.GetPosition().z})");
                }
                Debug.Log($"--> End Tag From Camera Loop: {sw.Elapsed.TotalMilliseconds}");
            }
            foreach (var aprilTag in tagWorldData)
            {
                var cameraPoseInWorld = PassthroughCameraUtils.GetCameraPoseInWorld(PassthroughCameraEye.Left);
                Debug.Log($"[AprilTag] Camera Position in World is: ({cameraPoseInWorld.position.x}, {cameraPoseInWorld.position.y}, {cameraPoseInWorld.position.z})");
                
                var cameraWorld = Matrix4x4.TRS(
                    cameraPoseInWorld.position,
                    cameraPoseInWorld.rotation,
                    Vector3.one);
                
                
                var worldPose = cameraWorld * aprilTag.Value.tagPose;
                                
                var tagPositionWorld = worldPose.GetPosition();
                var tagRotationWorld = Quaternion.LookRotation(worldPose.GetColumn(2), worldPose.GetColumn(1));
                
                //Threshold built in - to avoid updating on only camera movement
                aprilTag.Value.worldPosition = tagPositionWorld;
                aprilTag.Value.worldRotation = tagRotationWorld;
                
            }
            Debug.Log($"--> End Camera Pose in World Loop: {sw.Elapsed.TotalMilliseconds}");
        }
        Debug.Log($"--> END update loop: {sw.Elapsed.TotalMilliseconds}");
    }
    
    /// Converts the AprilTag pose from camera space to world space
    static public Matrix4x4 TagFromCamera(AprilTagPose tagPose)
    {
        var invertedTagPos = new Vector3(tagPose.position.x, -tagPose.position.y, tagPose.position.z);
        var invertedTagRot = new Quaternion(
            tagPose.rotation.x,
            -tagPose.rotation.y,
            tagPose.rotation.z,
            -tagPose.rotation.w
        );

        var cubeOffsetPose = Matrix4x4.TRS(Vector3.zero,Quaternion.Euler(camToWorldRotation), Vector3.one);
        var tagPoseCam = Matrix4x4.TRS(invertedTagPos, invertedTagRot, Vector3.one);

        var tagPoseWorld = tagPoseCam * cubeOffsetPose;

        return tagPoseWorld;
    }
    
    public bool TryGetTagWorldPose(int tagID, out Vector3 pos, out Quaternion rot) {
        if (tagWorldData.TryGetValue(tagID, out var info)) {
            pos = info.worldPosition;
            rot = info.worldRotation;
            return true;
        }
        pos = Vector3.zero;
        rot = Quaternion.identity;
        return false;
    }
    
    public void Toggle()
    {
        running = !running;
    }
    
}
public class AprilTagWorldInfo
{
    public Vector3 worldPosition;
    public Quaternion worldRotation;
    public Matrix4x4 tagPose;
}

public enum AprilTagFamily
{
    Tag16h5,
    // Tag25h9,
    // Tag36h10,
    // Tag36h11,
    // TagCircle21h7,
    // TagCircle49h12,
    // TagCustom48h12,
    TagStandard41h12,
    // TagStandard52h13
}