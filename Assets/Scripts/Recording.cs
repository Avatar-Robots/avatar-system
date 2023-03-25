using System;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
internal class Recording
{
    [NonSerialized]
    public string FileName;

    public List<RecordingItem> Items;
    public int Version;
}

[Serializable]
internal class RecordingItem
{
    public ArmState[] AvatarArmConfigs;
    public Vector3 AvatarPosition;
    public Quaternion AvatarRotation;
    public Vector3 BasePosition;
    public Quaternion BaseRotation;
    public Vector3 CameraPosition;
    public Quaternion CameraRotation;
    public Quaternion[] GripperRotations;
    public float[] JointAngles;
    public Vector3[] JointCoords;
    public float Time;
}

[Serializable]
internal class ArmState
{
    public Quaternion ElbowRotation;
    public float ForeArmStretch;
    public Quaternion ShoulderRotation;
    public float UpperArmStretch;
    public Quaternion WristRotation;
    // ReSharper disable once InconsistentNaming
    public double MillisecondsSpentInIK;

    public ArmState(ArmConfiguration config)
    {
        ShoulderRotation = config.UpperArmTransform.rotation;
        ElbowRotation = config.ForeArmTransform.rotation;
        WristRotation = config.WristTransform.rotation;
        UpperArmStretch = config.UpperArmTransform.localScale.y;
        ForeArmStretch = config.ForeArmTransform.localScale.y;
        MillisecondsSpentInIK = config.MillisecondsSpentInIK;
    }
}
