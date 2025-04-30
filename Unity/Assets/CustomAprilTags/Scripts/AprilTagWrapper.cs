using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using UnityEngine;
using Debug = UnityEngine.Debug;
using System.Diagnostics;
using System.Threading;

public class AprilTagWrapper : MonoBehaviour
{
    
    [DllImport("apriltagnative")]
    private static extern int count_apriltags();
    
    [DllImport("apriltagnative")]
    private static extern IntPtr get_latest_poses();
    
    [DllImport("apriltagnative")]
    private static extern void start_camera_native(float tagsize, int tagFamily);
    
    Stopwatch sw = Stopwatch.StartNew();
    List<AprilTagPose> poses = new();

    private int returnStringFromTagFamilyEnum(AprilTagFamily family)
    {
        return family switch
        {
            AprilTagFamily.Tag16h5 => 0,
            // AprilTagFamily.Tag25h9 => "tag25h9",
            // AprilTagFamily.Tag36h11 => "tag36h11",
            // AprilTagFamily.TagCircle21h7 => "tagCircle21h7",
            // AprilTagFamily.TagCircle49h12 => "tagCircle49h12",
            AprilTagFamily.TagStandard41h12 => 5,
            _ => throw new ArgumentOutOfRangeException(nameof(family), family, null)
        };
    }
    
    public void Init(float tagsize, AprilTagFamily family)
    {
        
        //convert family to string: family
        start_camera_native(tagsize, returnStringFromTagFamilyEnum(family));
    }
    
    public List<AprilTagPose> GetLatestPoses()
    {
        sw = Stopwatch.StartNew();
        Debug.Log($"Start Get Latest Poses: {sw.Elapsed.TotalMilliseconds}");
        poses = new List<AprilTagPose>();

        Debug.Log($"Before Detect: {sw.Elapsed.TotalMilliseconds}");
        int detected = count_apriltags();
        if (detected <= 0) return poses;
        Debug.Log($"After Detect: {sw.Elapsed.TotalMilliseconds}. Detected {detected} tags.");

        int stride = 13;
        IntPtr dataPtr = get_latest_poses();
        double[] buffer = new double[detected*stride];
        Marshal.Copy(dataPtr, buffer, 0, buffer.Length);
        
        //Below is for debugging:
        for (int i = 0; i < detected; i++)
        {
            int baseIdx = i * stride;
            int id = (int)buffer[baseIdx];
            Vector3 pos = new Vector3(
                (float)buffer[baseIdx + 1],
                (float)buffer[baseIdx + 2],
                (float)buffer[baseIdx + 3]
            );
            Debug.Log($"[ParseCheck] Tag {id} parsed position: {pos}");
        }
        
        Debug.Log($"!!! --> Got Buffer {buffer.Length}");
        for (int i = 0; i < buffer.Length; i += stride)
        {
            int id = (int)buffer[i];
            Vector3 pos = new Vector3((float)buffer[i + 1], (float)buffer[i + 2], (float)buffer[i + 3]);
            Matrix4x4 rot = new Matrix4x4();
            rot.m00 = (float)buffer[i + 4];
            rot.m01 = (float)buffer[i + 5];
            rot.m02 = (float)buffer[i + 6];
            rot.m10 = (float)buffer[i + 7];
            rot.m11 = (float)buffer[i + 8];
            rot.m12 = (float)buffer[i + 9];
            rot.m20 = (float)buffer[i + 10];
            rot.m21 = (float)buffer[i + 11];
            rot.m22 = (float)buffer[i + 12];
            rot.m33 = 1.0f;
            
            poses.Add(new AprilTagPose
            {
                id = id,
                position = pos,
                rotation = rot.rotation
            });
        }

        return poses;
    }
}

public struct AprilTagPose
{
    public int id;
    public Vector3 position;
    public Quaternion rotation;
}