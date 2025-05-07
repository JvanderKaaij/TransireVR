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
        
        private Vector3 currentPos;
        private Vector3 currentVel;

        private SimpleKalmanFilter _kalmanFilter = new();

        private void Update()
        {
            if (_aprilTagReader == null) return;

            if (_aprilTagReader.TryGetTagWorldPose(tagID, out targetPosition, out targetRotation))
            {
                float dt = Time.deltaTime;

                // Kalman update
                Vector3 filteredPos = _kalmanFilter.Update(targetPosition, dt);
                currentVel = _kalmanFilter.GetEstimatedVelocity();

                // Predict forward to current render time
                Vector3 predictedPos = filteredPos + currentVel * dt;

                // Smooth the transition visually
                currentPos = Vector3.Lerp(currentPos, predictedPos, 0.4f);
                transform.position = currentPos;

                transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, dt * smoothSpeed);
            }
        }
    }
}