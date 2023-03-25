using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IKUtils
{
    public const string ScaleOffsetterName = "ScaleOffset";

    public static Transform AddScaleOffsetter(Transform transform)
    {
        if (transform.childCount == 1 && transform.GetChild(0).name == ScaleOffsetterName)
            return transform.GetChild(0);

        var offsetter = new GameObject(ScaleOffsetterName);
        var ret = offsetter.transform;
        ret.SetParent(transform, false);
        for (var i = 0; i < transform.childCount; i++)
        {
            var ch = transform.GetChild(i);
            if (ch != ret) ch.SetParent(ret);
        }

        return ret;
    }

    /**
     * Limits the rotation to just one DoF: just around the the given axis.
     */
    public static Quaternion Limit1DoF(Vector3 axis, Quaternion rotation)
    {
        return Quaternion.FromToRotation(rotation * axis, axis) * rotation;
    }

    /**
     * Limit the rotation to two degrees of freedom: an axial rotation around secondary, followed
     * by a rotation around the primary axis.
     */
    public static Quaternion Limit2DoF(Vector3 axis, Vector3 secondary, Quaternion rotation)
    {
        var rotated = rotation * secondary;
        var projected = rotated;
        var normal = axis;
        // Note: "projected." is modified to be normal to "normal."
        Vector3.OrthoNormalize(ref normal, ref projected);
        return Quaternion.FromToRotation(rotated, projected) * rotation;
    }

    public static Quaternion SeparateAxial(Vector3 axis, Quaternion rotation)
    {
        // Decompose: rotation = FromToRotation(axis, rotation * axis) * axial rotation
        return Quaternion.Inverse(Quaternion.FromToRotation(axis, rotation * axis)) * rotation;
    }

    /**
     * A hinge limit similar to Limit2DoF, with angle range and twist range.
     * Reference: FinalIK code.
     */
    public static Func<Quaternion, Quaternion> LimitHinge(Vector3 axis, Vector3 secondaryAxis,
        float minAngle, float maxAngle, float maxTwist)
    {
        var lastAngle = 0f;

        Quaternion Limit(Quaternion rotation)
        {
            var limited = Limit2DoF(axis, secondaryAxis, rotation);

            // Isolate and limit the twist first.
            var axial = SeparateAxial(secondaryAxis, limited);
            var limitedAxial = Quaternion.RotateTowards(Quaternion.identity, axial, maxTwist);

            // Now this one is just brilliant.
            var workingSpace = Quaternion.Inverse(Quaternion.AngleAxis(lastAngle, axis) *
                                                  Quaternion.LookRotation(secondaryAxis, axis));
            // limited * secondaryAxis => secondary axis rotated around the hinge axis.
            // apply inverse of AngleAxis(lastAngle, axis) => rotate -lastAngle around the hinge axis.
            // apply inverse of LookRotation(secondaryAxis, axis) => re-orient the frame such that
            // hinge axis is always the y axis and the secondary axis is always the z axis.
            var d = workingSpace * limited * secondaryAxis;
            var deltaAngle = Mathf.Atan2(d.x, d.z) * Mathf.Rad2Deg;

            lastAngle = Mathf.Clamp(lastAngle + deltaAngle, minAngle, maxAngle);
            return Quaternion.AngleAxis(lastAngle, axis) * limitedAxial;
        }

        return Limit;
    }

    public static Func<Quaternion, Quaternion> Limit3DoF(Vector3 axis, float maxAngle,
        float maxTwist)
    {
        var secondaryAxis = new Vector3(axis.y, axis.z, axis.x);

        Quaternion LimitTwist(Quaternion rotation)
        {
            if (maxTwist >= 180) return rotation;

            var normal = rotation * axis;
            var orthoTangent = secondaryAxis;
            Vector3.OrthoNormalize(ref normal, ref orthoTangent);

            var rotatedOrthoTangent = rotation * secondaryAxis;
            Vector3.OrthoNormalize(ref normal, ref rotatedOrthoTangent);

            var fixedRotation =
                Quaternion.FromToRotation(rotatedOrthoTangent, orthoTangent) * rotation;

            return maxTwist <= 0
                ? fixedRotation
                : Quaternion.RotateTowards(fixedRotation, rotation, maxTwist);
        }

        Quaternion LimitSwing(Quaternion rotation)
        {
            if (axis == Vector3.zero) return rotation; // Ignore with zero axes
            if (rotation == Quaternion.identity)
                return rotation; // Assuming initial rotation is in the reachable area
            if (maxAngle >= 180) return rotation;

            var swingAxis = rotation * axis;

            var swingRotation = Quaternion.FromToRotation(axis, swingAxis);
            var limitedSwingRotation =
                Quaternion.RotateTowards(Quaternion.identity, swingRotation, maxAngle);

            var toLimits = Quaternion.FromToRotation(swingAxis, limitedSwingRotation * axis);

            return toLimits * rotation;
        }

        return q => LimitTwist(LimitSwing(q));
    }
}
