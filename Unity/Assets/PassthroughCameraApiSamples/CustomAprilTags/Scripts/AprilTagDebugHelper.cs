using TMPro;
using UnityEngine;

public class AprilTagDebugHelper : MonoBehaviour
{
    [SerializeField] private Camera m_camera;
    [SerializeField] private TextMeshProUGUI m_txtCamFov;
    
    public void CameraFOV(float fov)
    {
        m_camera.fieldOfView = fov;
        Debug.Log($"Camera FOV: {fov}");
        m_txtCamFov.text = $"Camera FOV: {m_camera.fieldOfView}";
    }

    public void CameraSeperation(float ss)
    {
    }
    
    public void CameraConverge(float cc)
    {
        m_camera.stereoConvergence = cc;
    }
}
