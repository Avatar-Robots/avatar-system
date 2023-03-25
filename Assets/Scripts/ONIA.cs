using System;
using UnityEngine;

public class ONIA : MonoBehaviour
{
    private ArmConfiguration _config;

    private Quaternion _defaultForeArmRotation;
    private Vector3 _defaultForeArmScale;
    private Vector3 _defaultForeArmTransform;
    private Quaternion _defaultUpperArmRotation;
    private Vector3 _defaultUpperArmScale;
    private Vector3 _defaultUpperArmTransform;

    private Transform _foreArmOffsetter;
    private Transform _upperArmOffsetter;

    public ArmSide Side;
    public float ElbowTwistMultiplier = 0.5f;

    // Start is called before the first frame update
    private void Start()
    {
        _config = ArmConfiguration.GetInstance(transform, Side);
        Debug.Log($"Config for {Side}: {_config}");
        _defaultUpperArmTransform = Quaternion.Inverse(_config.UpperArmTransform.rotation) *
                                    (_config.ForeArmTransform.position -
                                     _config.UpperArmTransform.position);
        _defaultUpperArmRotation = Quaternion.Inverse(_config.ShoulderTransform.rotation) *
                                   _config.UpperArmTransform.rotation;
        _defaultUpperArmScale = _config.UpperArmTransform.localScale;
        _defaultForeArmTransform = Quaternion.Inverse(_config.ForeArmTransform.rotation) *
                                   (_config.WristTransform.position -
                                    _config.ForeArmTransform.position);
        _defaultForeArmRotation = Quaternion.Inverse(_config.UpperArmTransform.rotation) *
                                  _config.ForeArmTransform.rotation;
        _defaultForeArmScale = _config.ForeArmTransform.localScale;
        _upperArmOffsetter = IKUtils.AddScaleOffsetter(_config.UpperArmTransform);
        _foreArmOffsetter = IKUtils.AddScaleOffsetter(_config.ForeArmTransform);
    }

#if UNITY_EDITOR
    public void OnDrawGizmos()
    {
        if (!Application.isPlaying || _config == null || _config.ElbowReferenceAxis == null) return;
        Gizmos.color = Color.magenta;
        Gizmos.DrawRay(_config.ForeArmTransform.position,
            (Vector3) _config.ElbowReferenceAxis * 0.1f);
    }
#endif

    // Update is called once per frame
    private void Update()
    {
        var stopWatch = new System.Diagnostics.Stopwatch();
        stopWatch.Start();
        ComputeIK();
        stopWatch.Stop();
        _config.MillisecondsSpentInIK = stopWatch.Elapsed.TotalMilliseconds;
    }

    private void ComputeIK()
    {
        if (_config.ElbowReferencePosition == null || _config.ElbowReferenceAxis == null) return;
        _config.UpperArmTransform.rotation =
            _config.ShoulderTransform.rotation * _defaultUpperArmRotation;

        var upperArm = (Vector3) _config.ElbowReferencePosition -
                       _config.UpperArmTransform.position;
        AdjustYScale(upperArm.magnitude, _defaultUpperArmTransform, _defaultUpperArmScale,
            _config.UpperArmTransform, _upperArmOffsetter);
        RotateTo(upperArm, _config.UpperArmTransform, _config.ForeArmTransform);

        var projectedCurrentElbowAxis = _config.UpperArmTransform.rotation * _config.ElbowAxis;
        Vector3.OrthoNormalize(ref upperArm, ref projectedCurrentElbowAxis);
        var projectedIdealElbowAxis = (Vector3) _config.ElbowReferenceAxis;
        Vector3.OrthoNormalize(ref upperArm, ref projectedIdealElbowAxis);
        _config.UpperArmTransform.rotation =
            Quaternion.FromToRotation(projectedCurrentElbowAxis, projectedIdealElbowAxis) *
            _config.UpperArmTransform.rotation;

        _config.ForeArmTransform.position = (Vector3) _config.ElbowReferencePosition;

        if (_config.WristReferencePosition == null) return;
        _config.ForeArmTransform.rotation =
            _config.UpperArmTransform.rotation * _defaultForeArmRotation;

        var foreArm = (Vector3) _config.WristReferencePosition - _config.ForeArmTransform.position;

        AdjustYScale(foreArm.magnitude, _defaultForeArmTransform, _defaultForeArmScale,
            _config.ForeArmTransform, _foreArmOffsetter);
        RotateTo(foreArm, _config.ForeArmTransform, _config.WristTransform);
        _config.WristTransform.position = (Vector3) _config.WristReferencePosition;

        if (_config.HandReferenceOrientation == null) return;

        var projectedElbowForward = _config.ForeArmTransform.forward;
        Vector3.OrthoNormalize(ref foreArm, ref projectedElbowForward);
        var projectedWristForward = (Quaternion) _config.HandReferenceOrientation * Vector3.forward;
        Vector3.OrthoNormalize(ref foreArm, ref projectedWristForward);

        _config.ForeArmTransform.rotation = Quaternion.Lerp(Quaternion.identity,
            Quaternion.FromToRotation(projectedElbowForward, projectedWristForward),
            ElbowTwistMultiplier) * _config.ForeArmTransform.rotation;
        if (_config.HandReferenceOrientation != null)
            _config.WristTransform.rotation = (Quaternion) _config.HandReferenceOrientation;


        void AdjustYScale(float idealLength, Vector3 defaultTransform, Vector3 defaultScale,
            Transform self, Transform offsetter)
        {
            var newScale = defaultScale;
            var scaleY = MathF.Sqrt((Sq(idealLength) - defaultTransform.sqrMagnitude) /
                Sq(defaultTransform.y) + 1);
            newScale.y *= scaleY;
            self.localScale = newScale;
            offsetter.localScale = new Vector3(1, 1 / scaleY, 1);

            float Sq(float x)
            {
                return x * x;
            }
        }

        void RotateTo(Vector3 idealDirection, Transform root, Transform target)
        {
            root.rotation = Quaternion.FromToRotation(
                Vector3.Normalize(target.position - root.position),
                Vector3.Normalize(idealDirection)) * root.rotation;
        }
    }
}