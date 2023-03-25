using UnityEngine;

public class GripperGestureSynchronizer : MonoBehaviour
{
    private ArmConfiguration _config;

    private Quaternion[,] _fingerDefaultRotations;
    private Transform[,] _fingerTransforms;
    private Quaternion[] _thumbDefaultRotations;
    private Transform[] _thumbTransforms;
    private Transform _wristTransform;
    private Transform[] _palmTransforms;
    private Quaternion[] _palmDefaultRotations;
    public float FingerTheta = 40;

    public ArmSide Side;
    public float ThumbJointAxisAngle = 0;

    public float ThumbTheta = 15;

    private float _timeSinceCreation;

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isEditor || Application.isPlaying)
            return;

        Initialize();
        var thumbAxis = ComputeAxis(ThumbTheta);
        var fingerAxis = ComputeAxis(FingerTheta);
        var thumbJointAxis = _thumbTransforms[0].rotation * new Vector3(
            Mathf.Cos(ThumbJointAxisAngle * Mathf.Deg2Rad), 0,
            Mathf.Sin(ThumbJointAxisAngle * Mathf.Deg2Rad));

        Gizmos.color = Color.magenta;
        Gizmos.DrawRay(_thumbTransforms[0].position, 0.05f * thumbAxis);
        Gizmos.color = Color.cyan;
        foreach (var joint in _fingerTransforms)
        {
            Gizmos.DrawLine(joint.position - 0.01f * fingerAxis,
                joint.position + 0.01f * fingerAxis);
        }

        foreach (var joint in _thumbTransforms)
        {
            Gizmos.DrawLine(joint.position, joint.position + 0.02f * thumbJointAxis);
        }
    }
#endif

    // Start is called before the first frame update
    private void Start()
    {
        Initialize();

        _timeSinceCreation = Time.timeSinceLevelLoad;
        var up = _wristTransform.up;
        var rot = Quaternion.AngleAxis(_config.WristTwistAngle, up);
        var downward = rot * _wristTransform.right;
        var inward = rot * _wristTransform.forward * (Side == ArmSide.Left ? 1 : -1);
        var axis = Vector3.Cross(downward, inward);

        _thumbTransforms[0].position += 0.02f * downward + 0.02f * up + 0.01f * inward;
        _thumbTransforms[0].rotation =
            Quaternion.AngleAxis(40, axis) * _thumbTransforms[0].rotation;
    }

    private void Initialize()
    {
        if (_config == null) _config = ArmConfiguration.GetInstance(transform, Side);
        if (_wristTransform == null) _wristTransform = _config.WristTransform;

        if (_fingerTransforms == null || _fingerDefaultRotations == null || _palmTransforms == null || _palmDefaultRotations == null)
        {
            var fingerNames = new[] {"index", "middle", "ring", "pinky"};
            _fingerTransforms = new Transform[4, 3];
            _fingerDefaultRotations = new Quaternion[4, 3];
            _palmTransforms = new Transform[4];
            _palmDefaultRotations = new Quaternion[4];
            for (var i = 0; i < fingerNames.Length; i++)
            {
                _palmTransforms[i] =
                    ArmConfiguration.FindWithSameSuffix(_wristTransform, $"ORG-palm.0{i + 1}");
                _palmDefaultRotations[i] = Quaternion.Inverse(_palmTransforms[i].parent.rotation) *
                                           _palmTransforms[i].rotation;
                var joint = ArmConfiguration.FindWithSameSuffix(_palmTransforms[i], $"DEF-f_{fingerNames[i]}.01");
                for (var j = 0; j < 3; j++)
                {
                    _fingerTransforms[i, j] = joint;
                    _fingerDefaultRotations[i, j] =
                        Quaternion.Inverse(joint.parent.rotation) * joint.rotation;
                    joint = ArmConfiguration.FindWithSameSuffix(joint,
                        $"DEF-f_{fingerNames[i]}.0{j + 2}");
                }
            }
        }

        if (_thumbTransforms == null || _thumbDefaultRotations == null)
        {
            var joint = ArmConfiguration.FindWithSameSuffix(_wristTransform, "ORG-palm.01");
            _thumbTransforms = new Transform[3];
            _thumbDefaultRotations = new Quaternion[3];
            for (var i = 0; i < 3; i++)
            {
                _thumbTransforms[i] =
                    ArmConfiguration.FindWithSameSuffix(joint, $"DEF-thumb.0{i + 1}");
                _thumbDefaultRotations[i] =
                    Quaternion.Inverse(joint.rotation) * _thumbTransforms[i].rotation;
                joint = _thumbTransforms[i];
            }
        }
    }

    private Vector3 ComputeAxis(float theta)
    {
        var up = _wristTransform.up;
        var rot = Quaternion.AngleAxis(_config.WristTwistAngle, up);

        return rot * (-Mathf.Cos(theta * Mathf.Deg2Rad) * _wristTransform.forward -
                      Mathf.Sin(theta * Mathf.Deg2Rad) * _wristTransform.right *
                      (Side == ArmSide.Left ? 1 : -1));
    }

    // Update is called once per frame
    private void Update()
    {
        var state = (Mathf.Cos((Time.timeSinceLevelLoad - _timeSinceCreation) * 1.0f) + 1) / 2;
        state = _config.GripperPosition;
        var angle = 2; // + 30 * (1 - state);
        var thumbAngle = 25 + 30 * state;

        var thumbAxis = ComputeAxis(ThumbTheta);
        var fingerAxis = ComputeAxis(FingerTheta);

        for (var i = 0; i < _fingerTransforms.GetLength(0); i++)
        {
            /*
            _palmTransforms[i].rotation = Quaternion.AngleAxis(angle, fingerAxis) *
                                          _palmTransforms[i].parent.rotation *
                                          _palmDefaultRotations[i];
            */
            for (var j = 0; j < 3; j++)
            {
                _fingerTransforms[i, j].rotation = Quaternion.AngleAxis(angle, fingerAxis) *
                                                   _fingerTransforms[i, j].parent.rotation *
                                                   _fingerDefaultRotations[i, j];
            }

            _fingerTransforms[i, 0].rotation =
                Quaternion.FromToRotation(_fingerTransforms[i, 0].up, _wristTransform.up) *
                _fingerTransforms[i, 0].rotation;
        }

        _thumbTransforms[0].rotation = Quaternion.AngleAxis(thumbAngle, thumbAxis) *
                                       Quaternion.FromToRotation(_thumbTransforms[0].up,
                                           _wristTransform.up) * _thumbTransforms[0].rotation;
        var thumbJointAxis = _thumbTransforms[0].rotation * new Vector3(
            Mathf.Cos(ThumbJointAxisAngle * Mathf.Deg2Rad), 0,
            Mathf.Sin(ThumbJointAxisAngle * Mathf.Deg2Rad));
        for (var i = 1; i < 3; i++)
            _thumbTransforms[i].rotation = Quaternion.AngleAxis(10, thumbJointAxis) *
                                           _thumbTransforms[i - 1].rotation *
                                           _thumbDefaultRotations[i];
    }
}