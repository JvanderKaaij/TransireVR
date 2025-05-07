using UnityEngine;

public class SimpleKalmanFilter
{
    private Vector3 _estimatedPosition;
    private Vector3 _velocity;

    private float _q = 0.1f; // process noise
    private float _r = 0.05f;  // measurement noise
    private float _p = 1.0f;  // estimate error

    private bool _initialized = false;

    public Vector3 Update(Vector3 measuredPosition, float deltaTime)
    {
        if (!_initialized)
        {
            _estimatedPosition = measuredPosition;
            _velocity = Vector3.zero;
            _initialized = true;
        }

        // Predict
        _estimatedPosition += _velocity * deltaTime;
        _p += _q;

        // Kalman Gain
        float k = _p / (_p + _r);

        // Correct
        Vector3 residual = measuredPosition - _estimatedPosition;
        _estimatedPosition += k * residual;
        _p *= (1 - k);

        // Update velocity estimate (assumes linear motion)
        _velocity = residual / deltaTime;

        return _estimatedPosition;
    }
    
    public Vector3 GetEstimatedVelocity()
    {
        return _velocity;
    }
}