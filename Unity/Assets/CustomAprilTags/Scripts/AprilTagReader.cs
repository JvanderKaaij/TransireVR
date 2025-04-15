using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using PassthroughCameraSamples;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
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

    [SerializeField] private Transform debugTarget;

    [SerializeField] private float smoothSpeed;

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
    
    private Vector3 camToWorldRotation = new(90, 180, 0);
    
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

        m_aprilTagWrapper.Init(tagSize, fx, fy, cx, cy);
        
        running = true;
        
        _detectionThread = new Thread(DetectionLoop);
        _detectionThread.Start();
    }
    
    void OnDestroy() {
        _running = false;
        _detectionThread.Join();
    }

    void DetectionLoop() {
        while (_running) {
            if (newFrameAvailable) {
                Color32[] pixelCopy;
                lock (_frameLock) {
                    pixelCopy = latestPixels;
                    newFrameAvailable = false;
                }
                ConvertToByteArray(pixelCopy, rgbaBytes);

                var tags = m_aprilTagWrapper.GetLatestPoses(rgbaBytes, m_webCamTextureManager.WebCamTexture.width, m_webCamTextureManager.WebCamTexture.height);

                lock (_resultLock) {
                    latestTags = tags;
                    tagsUpdated = true;
                }
            }

            Thread.Sleep(1); // or use ManualResetEvent or a more efficient sync
        }
    }
    
    Vector3 tagPositionInWorld = Vector3.zero;
    Quaternion tagRotationInWorld = Quaternion.identity;

    void Update() {
        if (!running || tex == null || !tex.didUpdateThisFrame)
            return;

        lock (_frameLock) {
            latestPixels = tex.GetPixels32();
            var width = tex.width;
            var height = tex.height;

            if (rgbaBytes == null || rgbaBytes.Length != width * height * 4)
                rgbaBytes = new byte[width * height * 4];

            newFrameAvailable = true;
        }

        // Apply latest tag transforms on main thread
        lock (_resultLock) {
            if (tagsUpdated && latestTags.Count > 0) {
                
                var invertedTagPos = new Vector3(latestTags[0].position.x, -latestTags[0].position.y, latestTags[0].position.z);

                var invertedTagRot = new Quaternion(
                    latestTags[0].rotation.x,
                    -latestTags[0].rotation.y,
                    latestTags[0].rotation.z,
                    -latestTags[0].rotation.w
                );
                
                var tagPoseCam = Matrix4x4.TRS(invertedTagPos, invertedTagRot, Vector3.one);
                
                var cubeOffsetPose = Matrix4x4.TRS(Vector3.zero,Quaternion.Euler(camToWorldRotation), Vector3.one);
                
                var cameraPoseInWorld = PassthroughCameraUtils.GetCameraPoseInWorld(m_webCamTextureManager.Eye);

                var cameraWorld = Matrix4x4.TRS(
                    cameraPoseInWorld.position,
                    cameraPoseInWorld.rotation,
                    Vector3.one);
                
                var tagPose = cameraWorld * tagPoseCam * cubeOffsetPose;
                
                tagPositionInWorld = tagPose.GetPosition();
                
                tagRotationInWorld = Quaternion.LookRotation(tagPose.GetColumn(2), tagPose.GetColumn(1));
                
                tagsUpdated = false;
            }
        }
        
        debugTarget.position = Vector3.Lerp(debugTarget.position, tagPositionInWorld, Time.deltaTime * smoothSpeed);
        debugTarget.rotation = Quaternion.Lerp(debugTarget.rotation, tagRotationInWorld,Time.deltaTime * smoothSpeed);
    }

    unsafe void ConvertToByteArray(Color32[] pixels, byte[] rgbaBytes)
    {
        fixed (Color32* pSrc = pixels)
        fixed (byte* pDst = rgbaBytes)
        {
            Buffer.MemoryCopy(pSrc, pDst, rgbaBytes.Length, rgbaBytes.Length);
        }
    }
    
}
