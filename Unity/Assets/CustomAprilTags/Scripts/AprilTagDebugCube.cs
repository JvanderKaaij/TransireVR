using System;
using UnityEngine;

namespace CustomAprilTags.Scripts
{
    public class AprilTagDebugCube : MonoBehaviour
    {
        [SerializeField] private AprilTagReader _aprilTagReader;
        [SerializeField] private int tagID;
        [SerializeField] private float smoothSpeed;

        Vector3 targetPosition = new(0, 0, 0);
        Quaternion targetRotation = Quaternion.identity;

        Vector3 velocity = Vector3.zero;

        private void Update()
        {
            if (_aprilTagReader == null) return;

            if (_aprilTagReader.TryGetTagWorldPose(tagID, out targetPosition, out targetRotation))
            {
                transform.position = Vector3.SmoothDamp(
                    transform.position, targetPosition, ref velocity, 0.1f); // tunable
                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * smoothSpeed);
            }
        }
    }
}