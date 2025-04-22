using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Threading;
using PassthroughCameraSamples;
using Unity.Profiling;
using UnityEngine;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;

public class AprilTagReader : MonoBehaviour
{
    [SerializeField] private WebCamTextureManager m_webCamTextureManager;
    [SerializeField] private AprilTagWrapper m_aprilTagWrapper;
    [Tooltip("Size of the AprilTag (edge-to-edge) in meters, e.g. 0.1 for 10cm.")]
    [SerializeField] private float tagSize;
    [SerializeField] private AprilTagFamily tagFamily;

    private bool running;
    private PassthroughCameraIntrinsics intrinsics;
    private byte[] rgbaBytes;
    private WebCamTexture tex;
    Thread _detectionThread;
    bool _running = true;
    private bool newFrameAvailable;
    private readonly object _frameLock = new();
    private readonly object _resultLock = new();
    private List<AprilTagPose> latestTags = new();
    private bool tagsUpdated;
    private Color32[] latestPixels;
    private Vector3 targetPos;
    
    private static Vector3 camToWorldRotation = new(90, 180, 0);
    
    private Dictionary<int, AprilTagWorldInfo> tagWorldData = new();
    
    private IEnumerator Start()
    {
        while (m_webCamTextureManager.WebCamTexture == null)
        {
            yield return null;
        }
        
        intrinsics = PassthroughCameraUtils.GetCameraIntrinsics(m_webCamTextureManager.Eye);
        tex = m_webCamTextureManager.WebCamTexture;
        
        var referenceResolution = new Vector2Int(1280, 960);
        var camRes = new Vector2(tex.width, tex.height);

        var scaleX = camRes.x / referenceResolution.x;
        var scaleY = camRes.y / referenceResolution.y;

        var fx = intrinsics.FocalLength.x * scaleX;
        var fy = intrinsics.FocalLength.y * scaleY;
        var cx = intrinsics.PrincipalPoint.x * scaleX;
        var cy = intrinsics.PrincipalPoint.y * scaleY;

        m_aprilTagWrapper.Init(tagSize, tagFamily, fx, fy, cx, cy);
        
        running = true;
        
        _detectionThread = new Thread(DetectionLoop);
        _detectionThread.Name = "AprilTagDetectionThread";
        
        _detectionThread.Start();
    }
    
    void OnDestroy() {
        _running = false;
        _detectionThread.Join();
    }

    void DetectionLoop() {
        while (_running) {
            if (newFrameAvailable) {
                Stopwatch sw = Stopwatch.StartNew();
                Color32[] pixelCopy;
                lock (_frameLock) {
                    pixelCopy = latestPixels;
                    newFrameAvailable = false;
                }
                ConvertToByteArray(pixelCopy, rgbaBytes);
                UnityEngine.Debug.Log($"[AprilTag] ConvertToByteArray took {sw.ElapsedMilliseconds}ms");

                
                var tags = m_aprilTagWrapper.GetLatestPoses(rgbaBytes, m_webCamTextureManager.WebCamTexture.width, m_webCamTextureManager.WebCamTexture.height);
                UnityEngine.Debug.Log($"[AprilTag] Detection took {sw.ElapsedMilliseconds}ms");

                lock (_resultLock) {
                    latestTags = tags;
                    tagsUpdated = true;
                }
            }
            Thread.Sleep(1); // or use ManualResetEvent or a more efficient sync
        }
    }

    void Update() {
        Stopwatch sw = Stopwatch.StartNew();
        if (running){
            lock (_frameLock) {
                latestPixels = tex.GetPixels32();
                UnityEngine.Debug.Log($"[AprilTag] GetPixels32 took {sw.ElapsedMilliseconds}ms");
                var width = tex.width;
                var height = tex.height;

                if (rgbaBytes == null || rgbaBytes.Length != width * height * 4)
                    rgbaBytes = new byte[width * height * 4];

                newFrameAvailable = true;
            }

            // Apply latest tag transforms on main thread
            lock (_resultLock) {
                if (tagsUpdated && latestTags.Count > 0) {
                    
                    foreach (var aprilTag in latestTags) {
                        var tagID = aprilTag.id;

                        if (!tagWorldData.ContainsKey(tagID))
                            tagWorldData[tagID] = new AprilTagWorldInfo();
                        
                        tagWorldData[tagID].tagPose = TagFromCamera(aprilTag);
                        UnityEngine.Debug.Log($"[AprilTag] Tage Position in Camera View is: ({tagWorldData[tagID].tagPose.GetPosition().x}, {tagWorldData[tagID].tagPose.GetPosition().y}, {tagWorldData[tagID].tagPose.GetPosition().z})");
                    }
                    
                    tagsUpdated = false;
                }
            }
        }
        
        foreach (var aprilTag in tagWorldData)
        {
            var cameraPoseInWorld = PassthroughCameraUtils.GetCameraPoseInWorld(m_webCamTextureManager.Eye);
            UnityEngine.Debug.Log($"[AprilTag] Camera Position in World is: ({cameraPoseInWorld.position.x}, {cameraPoseInWorld.position.y}, {cameraPoseInWorld.position.z})");
            
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
    }

    unsafe void ConvertToByteArray(Color32[] pixels, byte[] rgbaBytes)
    {
        fixed (Color32* pSrc = pixels)
        fixed (byte* pDst = rgbaBytes)
        {
            Buffer.MemoryCopy(pSrc, pDst, rgbaBytes.Length, rgbaBytes.Length);
        }
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