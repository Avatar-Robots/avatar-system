using System;
using System.Linq;
using UnityEngine;

public class ArmConfiguration : MonoBehaviour
{
    public const float MaxShoulderAngle = 85.0f;
    public const float MaxShoulderTwist = 75.0f;
    public const float MaxElbowAngle = 150.0f;
    public const float MaxElbowTwist = 40.0f;
    public const float MaxWristAngle = 50.0f;
    public const float MaxWristTwist = 40.0f;

    public const float MinUpperArmScale = 0.7f;
    public const float MaxUpperArmScale = 1.3f;
    public const float MinForeArmScale = 0.7f;
    public const float MaxForeArmScale = 1.3f;

    private Transform _foreArmTransform;
    private Transform _shoulderTransform;
    private Transform _upperArmTransform;
    private Transform _wristTransform;

    public float ElbowAxisAngle;
    public Vector3? ElbowReferenceAxis;
    public Vector3? ElbowReferencePosition;
    public float GripperPosition;

    [NonSerialized]
    // ReSharper disable once InconsistentNaming
    public double MillisecondsSpentInIK;

    public Quaternion? RawHandReferenceOrientation;

    public ArmSide Side;

    public Vector3? WristReferencePosition;
    public float WristTwistAngle;

    public Quaternion? HandReferenceOrientation
    {
        get => RawHandReferenceOrientation == null
            ? null
            : (Quaternion) RawHandReferenceOrientation *
              Quaternion.AngleAxis(WristTwistAngle, Vector3.up);
        set => RawHandReferenceOrientation = value == null
            ? null
            : (Quaternion) value * Quaternion.AngleAxis(-WristTwistAngle, Vector3.up);
    }


    public Vector3 ElbowAxis
    {
        get
        {
            var axisAngleRad = ElbowAxisAngle * Mathf.Deg2Rad;
            return new Vector3(Mathf.Cos(axisAngleRad), 0, Mathf.Sin(axisAngleRad));
        }
    }

    public Transform ShoulderTransform => _shoulderTransform == null
        ? _shoulderTransform = FindShoulderFromPrefabRoot(transform, Side)
        : _shoulderTransform;

    public Transform UpperArmTransform => _upperArmTransform == null
        ? _upperArmTransform = FindUpperArmFromShoulder(ShoulderTransform)
        : _upperArmTransform;

    public Transform ForeArmTransform => _foreArmTransform == null
        ? _foreArmTransform = FindForeArmFromUpperArm(UpperArmTransform)
        : _foreArmTransform;

    public Transform WristTransform => _wristTransform == null
        ? _wristTransform = FindWristFromForeArm(ForeArmTransform)
        : _wristTransform;

#if UNITY_EDITOR
    public void OnDrawGizmos()
    {
        if (Application.isPlaying || !isActiveAndEnabled) return;
        Gizmos.color = Color.green;
        Gizmos.DrawLine(ForeArmTransform.position,
            ForeArmTransform.position + UpperArmTransform.rotation * ElbowAxis * 0.1f);
        var turn = Quaternion.AngleAxis(WristTwistAngle, WristTransform.up);
        Gizmos.color = Color.blue;
        Gizmos.DrawRay(WristTransform.position, turn * WristTransform.forward * 0.1f);
        Gizmos.color = Color.red;
        Gizmos.DrawRay(WristTransform.position, turn * WristTransform.right * 0.1f);
    }
#endif

    public static ArmConfiguration GetInstance(Transform rigRoot, ArmSide side)
    {
        return rigRoot.GetComponents<ArmConfiguration>()
            .FirstOrDefault(armConfig => armConfig.Side == side);
    }

    private static string GetSideSuffix(ArmSide side)
    {
        return side == ArmSide.Left ? "L" : "R";
    }

    public static Transform FindShoulderFromPrefabRoot(Transform prefabRoot, ArmSide side)
    {
        return prefabRoot.Find(
            $"rig/root/DEF-spine/DEF-spine.001/DEF-spine.002/DEF-spine.003/DEF-shoulder.{GetSideSuffix(side)}");
    }

    public static Transform FindWithSameSuffix(Transform transform, string prefix)
    {
        var suffix = transform.name[^1];
        var ret = transform.Find($"{prefix}.{suffix}");
        return ret != null
            ? ret
            : transform.Find($"{IKUtils.ScaleOffsetterName}/{prefix}.{suffix}");
    }

    public static Transform FindUpperArmFromShoulder(Transform shoulder)
    {
        return FindWithSameSuffix(shoulder, "DEF-upper_arm");
    }

    public static Transform FindForeArmFromUpperArm(Transform upperArm)
    {
        return FindWithSameSuffix(upperArm, "DEF-forearm");
    }

    public static Transform FindWristFromForeArm(Transform foreArm)
    {
        return FindWithSameSuffix(foreArm, "DEF-hand");
    }

    public static Transform FindHandFromWrist(Transform wrist)
    {
        return FindWithSameSuffix(wrist, "ORG-palm.02");
    }
}

public enum ArmSide
{
    Left,
    Right
}