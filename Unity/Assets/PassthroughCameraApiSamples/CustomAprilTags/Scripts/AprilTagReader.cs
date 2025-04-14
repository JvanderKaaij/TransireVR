using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using PassthroughCameraSamples;
using TMPro;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

public class AprilTagReader : MonoBehaviour
{
    [SerializeField] private WebCamTextureManager m_webCamTextureManager;
    [SerializeField] private RawImage m_image;
    [SerializeField] private AprilTagWrapper m_aprilTagWrapper;
    [SerializeField] private TextMeshProUGUI debugTxt1;
    [SerializeField] private TextMeshProUGUI debugTxt2;
    [Tooltip("Size of the AprilTag (edge-to-edge) in meters, e.g. 0.1 for 10cm.")]
    [SerializeField] private float tagSize;

    [SerializeField] private Transform debugTarget;

    [SerializeField] private float smoothSpeed;
    
    [SerializeField] private Vector3 offsetRotation;
    
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
    
    private IEnumerator Start()
    {
        while (m_webCamTextureManager.WebCamTexture == null)
        {
            yield return null;
        }

        m_image.texture = m_webCamTextureManager.WebCamTexture;
        intrinsics = PassthroughCameraUtils.GetCameraIntrinsics(m_webCamTextureManager.Eye);
        tex = m_webCamTextureManager.WebCamTexture;

        m_aprilTagWrapper.Init(tagSize, intrinsics.FocalLength.x, intrinsics.FocalLength.y, intrinsics.PrincipalPoint.x,
            intrinsics.PrincipalPoint.y);
        
        Debug.Log($"Focal Length : ({intrinsics.FocalLength.x}, {intrinsics.FocalLength.y})");
        Debug.Log($"Principal Point : ({intrinsics.PrincipalPoint.x}, {intrinsics.PrincipalPoint.y})");
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
    
    Vector3 finalPosition = Vector3.zero;
    Quaternion tagRotationInWorld = Quaternion.identity;

    void Update() {
        if (!running || tex == null || !tex.didUpdateThisFrame)
            return;

        
        
        lock (_frameLock) {
            latestPixels = tex.GetPixels32();
            int width = tex.width;
            int height = tex.height;

            if (rgbaBytes == null || rgbaBytes.Length != width * height * 4)
                rgbaBytes = new byte[width * height * 4];

            newFrameAvailable = true;
        }

        // Apply latest tag transforms on main thread
        lock (_resultLock) {
            if (tagsUpdated && latestTags.Count > 0) {
                
                Vector3 tagPosInPassthrough = latestTags[0].position;

                var invertedTagPos = new Vector3(tagPosInPassthrough.x, -tagPosInPassthrough.y, tagPosInPassthrough.z);
                
                Quaternion flippedRot = new Quaternion(
                    latestTags[0].rotation.x,
                    -latestTags[0].rotation.y,
                    latestTags[0].rotation.z,
                    -latestTags[0].rotation.w
                );
                
                Matrix4x4 tagPoseCam = Matrix4x4.TRS(invertedTagPos, flippedRot, Vector3.one);
                
                Matrix4x4 cubeOffsetPose = Matrix4x4.TRS(Vector3.zero,Quaternion.Euler(offsetRotation), Vector3.one);
                
                Pose cameraPoseInWorld = PassthroughCameraUtils.GetCameraPoseInWorld(m_webCamTextureManager.Eye);

                Matrix4x4 cameraWorld = Matrix4x4.TRS(
                    cameraPoseInWorld.position,
                    cameraPoseInWorld.rotation,
                    Vector3.one);
                
                Matrix4x4 tagPose = cameraWorld * tagPoseCam * cubeOffsetPose;

                Vector3 tagPoseInWorld = tagPose.GetColumn(3);
                
                finalPosition = tagPoseInWorld;
                
                tagRotationInWorld = Quaternion.LookRotation(tagPose.GetColumn(2), tagPose.GetColumn(1));
                
                tagsUpdated = false;
            }
        }
        
        debugTarget.position = Vector3.Lerp(debugTarget.position, finalPosition, Time.deltaTime * smoothSpeed);
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
