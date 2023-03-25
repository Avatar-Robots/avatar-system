using System;
using System.Linq;
using UnityEngine;

/**
 * A custom FABRIK-based inverse kinematic solver.
 *
 * The code is based on FinalIK. The implementation of rotational constraints in FinalIK is a gem.
 */
public class FABRIK : MonoBehaviour
{
    private FABRIKSolver _solver;
    private ArmConfiguration _config;

    public int MaxIterations;
    public int MaxIterationsInBinarySearch;

    private Bone _shoulderBone;
    private Bone _elbowBone;
    private Bone _wristBone;
    private Bone _handBone;

    /**
     * The angle in degree the arm needs to be raised up so that the arm is parallel to the ground
     * (Forming a perfect T posture)
     */
    public float ArmRaise = 45;

    public float BinarySearchPrecision = 0.003f;

    public ArmSide Side;

    public GameObject IndicatorPrefab;
    private GameObject[] _indicators;

    private float _maxDeviation;

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        var config = ArmConfiguration.GetInstance(transform, Side);
        // Handles.Label(config.ForeArmTransform.position, $"Max deviation: {_maxDeviation}");
    }
#endif

    // Start is called before the first frame update
    private void Start()
    {
        var sign = Side == ArmSide.Left ? 1 : -1;
        _config = ArmConfiguration.GetInstance(transform, Side);

        _config.UpperArmTransform.rotation =
            Quaternion.AngleAxis(-sign * ArmRaise, transform.forward) *
            _config.UpperArmTransform.rotation;

        _shoulderBone = new Bone(_config.UpperArmTransform,
            _config.ShoulderTransform, _config.ForeArmTransform,
            IKUtils.Limit3DoF(Vector3.up, ArmConfiguration.MaxShoulderAngle, ArmConfiguration.MaxShoulderTwist),
            new Scalability(Scalability.Y, ArmConfiguration.MinUpperArmScale, ArmConfiguration.MaxUpperArmScale));
        _elbowBone = new Bone(_config.ForeArmTransform,
            _config.UpperArmTransform, _config.WristTransform,
            IKUtils.LimitHinge(_config.ElbowAxis, Vector3.up, 0, 
                ArmConfiguration.MaxElbowAngle, ArmConfiguration.MaxElbowTwist),
            new Scalability(Scalability.Y, ArmConfiguration.MinForeArmScale, ArmConfiguration.MaxForeArmScale));
        _wristBone = new Bone(_config.WristTransform, _config.ForeArmTransform, null,
            IKUtils.Limit3DoF(Vector3.up, ArmConfiguration.MaxWristAngle,
                ArmConfiguration.MaxWristTwist));
        _solver = new FABRIKSolver(_shoulderBone, _elbowBone, _wristBone);

        InitializeIndicators();
    }

    private void InitializeIndicators()
    {
        if (IndicatorPrefab == null) return;
        _indicators = new GameObject[4];
        for (var i = 0; i < 4; i++)
        {
            _indicators[i] = Instantiate(IndicatorPrefab, Vector3.zero, Quaternion.identity);
            _indicators[i].name = $"IK joint {i}";
        }
    }

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
        if (IndicatorPrefab != null)
        {
            _indicators[0].transform.position = _handBone.SolverPosition;
            _indicators[1].transform.position = _wristBone.SolverPosition;
            _indicators[2].transform.position = _elbowBone.SolverPosition;
            _indicators[3].transform.position = _shoulderBone.SolverPosition;
        }

        if (_config.WristReferencePosition == null)
            return;

        _wristBone.ReferencePosition = _config.WristReferencePosition;
        _wristBone.ReferenceDeviationTolerance = 0.1f;
        _elbowBone.ReferencePosition = null;


        if (_config.HandReferenceOrientation != null)
            _wristBone.SolverRotation = (Quaternion) _config.HandReferenceOrientation;

        for (var i = 0; i < MaxIterations; i++)
            _solver.Solve();

        if (!_solver.IsSuccessful() || _config.ElbowReferencePosition == null)
        {
            _solver.Apply();
            return;
        }

        _elbowBone.ReferencePosition = _config.ElbowReferencePosition;
        // _wristBone.ReferencePosition = WristReference.position;

        if (_config.HandReferenceOrientation != null)
            _wristBone.SolverRotation = (Quaternion) _config.HandReferenceOrientation;

        var feasibleState = _solver.CloneState();
        var maxDeviation = _solver.GetMaxDeviation();
        var upper = 0.3f;
        var lower = 0f;
        while (upper - lower > BinarySearchPrecision)
        {
            var mid = (upper + lower) / 2;
            _elbowBone.ReferenceDeviationTolerance = mid;
            for (var i = 0; i < MaxIterationsInBinarySearch; i++)
                _solver.Solve();
            maxDeviation = _solver.GetMaxDeviation();
            if (_solver.IsSuccessful())
            {
                feasibleState = _solver.CloneState();
                upper = maxDeviation;
            }
            else
            {
                lower = mid;
                _solver.RestoreState(feasibleState);
                _shoulderBone = feasibleState[0];
                _elbowBone = feasibleState[1];
                _wristBone = feasibleState[2];
                // _handBone = feasibleState[3];
            }
        }
        _maxDeviation = maxDeviation;

        _solver.Apply();
        if (_config.HandReferenceOrientation != null)
            _config.WristTransform.rotation = (Quaternion) _config.HandReferenceOrientation;
    }

    private class FABRIKSolver
    {
        private readonly Bone[] _bones;

        public FABRIKSolver(params Bone[] bones)
        {
            _bones = bones;
        }

        public void Solve()
        {
            ForwardReach();
            BackwardReach();
        }

        public void Apply()
        {
            _bones[0].Transform.position = _bones[0].SolverPosition;
            foreach (var bone in _bones)
            {
                bone.Transform.rotation = bone.SolverRotation;
                bone.Transform.position = bone.SolverPosition;
                bone.ApplyScale();
            }
        }

        public bool IsSuccessful()
        {
            return !(from bone in _bones where !bone.IsWithinTolerance() select bone).Any();
        }

        public float GetMaxDeviation()
        {
            return _bones.Select(b => b.GetDeviation()).Max();
        }

        public Bone[] CloneState()
        {
            var ret = new Bone[_bones.Length];
            for (var i = 0; i < _bones.Length; i++)
                ret[i] = _bones[i].Clone();
            return ret;
        }

        // ReSharper disable once SuggestBaseTypeForParameter
        public void RestoreState(Bone[] state)
        {
            for (var i = 0; i < _bones.Length; i++)
                _bones[i] = state[i];
        }

        /**
         * Work from the end of the kinematic chain all the way to the root.
         */
        private void ForwardReach()
        {
            var referencePosition = _bones[^1].ReferencePosition;
            if (referencePosition != null)
                _bones[^1].SolverPosition = (Vector3) referencePosition;
            foreach (var bone in _bones)
                bone.Limited = false;
            for (var i = _bones.Length - 2; i >= 0; i--)
            {
                _bones[i].SolverPosition = SolveJointWithScale(_bones[i].GetPositionWithReference(),
                    _bones[i + 1].SolverPosition,
                    _bones[i].Axis, _bones[i].Scale);
                LimitForward(i, i + 1);
            }

            LimitForward(0, 0);
        }

        private void LimitForward(int rotateBone, int limitBone)
        {
            var lastBoneBeforeLimit = _bones[^1].SolverPosition;
            for (var i = rotateBone; i < _bones.Length - 1; i++)
            {
                if (_bones[i].Limited) break;
                var rotation = Quaternion.FromToRotation(
                    _bones[i].SolverRotation * _bones[i].ScaledAxis,
                    _bones[i + 1].SolverPosition - _bones[i].SolverPosition);
                SolverRotate(i, rotation, false);
            }

            var limitedRotation =
                GetLimitedRotation(limitBone, _bones[limitBone].SolverRotation, out var changed);
            if (changed)
            {
                if (limitBone < _bones.Length - 1)
                {
                    var delta = limitedRotation *
                                Quaternion.Inverse(_bones[limitBone].SolverRotation);
                    _bones[limitBone].SolverRotation = limitedRotation;
                    SolverRotate(limitBone + 1, delta, true);
                    SolverMoveChildrenAround(limitBone, delta);

                    delta = Quaternion.FromToRotation(
                        _bones[^1].SolverPosition - _bones[rotateBone].SolverPosition,
                        lastBoneBeforeLimit - _bones[rotateBone].SolverPosition);
                    SolverRotate(rotateBone, delta, true);
                    SolverMoveChildrenAround(rotateBone, delta);
                    SolverMove(rotateBone, lastBoneBeforeLimit - _bones[^1].SolverPosition);
                }
                else
                {
                    _bones[limitBone].SolverRotation = limitedRotation;
                }
            }

            _bones[limitBone].Limited = true;
        }

        private void BackwardReach()
        {
            _bones[0].SolverPosition = _bones[0].Transform.position;
            for (var i = 0; i < _bones.Length - 1; i++)
            {
                var nextPosition = SolveJointWithScale(_bones[i + 1].SolverPosition,
                    _bones[i].GetPositionWithReference(), _bones[i].Axis, _bones[i].Scale);

                var swing = Quaternion.FromToRotation(
                    _bones[i].SolverRotation * _bones[i].ScaledAxis,
                    nextPosition - _bones[i].SolverPosition);

                var targetRotation = swing * _bones[i].SolverRotation;
                targetRotation = GetLimitedRotation(i, targetRotation, out _);

                var delta = targetRotation * Quaternion.Inverse(_bones[i].SolverRotation);
                _bones[i].SolverRotation = targetRotation;
                SolverRotate(i + 1, delta, true);
                _bones[i + 1].SolverPosition = _bones[i].SolverPosition +
                                               _bones[i].SolverRotation * _bones[i].ScaledAxis;
            }

            // Reconstruct solver rotations to protect from invalid Quaternions
            foreach (var bone in _bones)
            {
                bone.SolverRotation = Quaternion.LookRotation(
                    bone.SolverRotation * Vector3.forward,
                    bone.SolverRotation * Vector3.up);
            }
        }

        private Quaternion GetParentRotation(int index)
        {
            if (index > 0)
                return _bones[index - 1].SolverRotation;
            return _bones[index].Transform.parent == null
                ? Quaternion.identity
                : _bones[index].Transform.parent.rotation;
        }

        private Quaternion GetLimitedRotation(int index, Quaternion rotation, out bool changed)
        {
            var parentRotation = GetParentRotation(index);
            var localRotation = Quaternion.Inverse(parentRotation) * rotation;
            var limitedRotation = _bones[index].GetLimitedLocalRotation(localRotation, out changed);
            return parentRotation * limitedRotation;
        }

        private void SolverRotate(int index, Quaternion rotation, bool recursive)
        {
            for (var i = index; i < _bones.Length; i++)
            {
                _bones[i].SolverRotation = rotation * _bones[i].SolverRotation;
                if (!recursive) break;
            }
        }

        private void SolverMove(int index, Vector3 offset)
        {
            for (var i = index; i < _bones.Length; i++)
            {
                _bones[i].SolverPosition += offset;
            }
        }

        private void SolverMoveChildrenAround(int index, Quaternion rotation)
        {
            for (var i = index + 1; i < _bones.Length; i++)
            {
                var dir = _bones[i].SolverPosition - _bones[index].SolverPosition;
                _bones[i].SolverPosition = _bones[index].SolverPosition + rotation * dir;
            }
        }

        private static Vector3 SolveJoint(Vector3 self, Vector3 target, float length)
        {
            var lambda = length / Vector3.Distance(target, self);
            var result = Vector3.LerpUnclamped(target, self, lambda);
            return result;
        }

        private static Vector3 SolveJointWithScale(Vector3 self, Vector3 target, Vector3 axis,
            Scalability scale)
        {
            if (scale == null)
            {
                var lambda = axis.magnitude / Vector3.Distance(target, self);
                return Vector3.LerpUnclamped(target, self, lambda);
            }

            var sqrDis = (target - self).sqrMagnitude;
            var sqrNonScaling = axis.sqrMagnitude - Square(axis[scale.Axis]);
            var sqrMinDis = sqrNonScaling + Square(scale.MinScale * axis[scale.Axis]);
            var sqrMaxDis = sqrNonScaling + Square(scale.MaxScale * axis[scale.Axis]);

            if (sqrMinDis <= sqrDis && sqrDis <= sqrMaxDis)
            {
                scale.Scale = MathF.Sqrt((sqrDis - sqrNonScaling) / Square(axis[scale.Axis]));
                return self;
            }

            if (sqrDis > sqrMaxDis)
            {
                scale.Scale = scale.MaxScale;
                return Vector3.LerpUnclamped(target, self, MathF.Sqrt(sqrMaxDis / sqrDis));
            }

            scale.Scale = scale.MinScale;
            return Vector3.LerpUnclamped(target, self, MathF.Sqrt(sqrMinDis / sqrDis));

            float Square(float x)
            {
                return x * x;
            }
        }
    }

    private class Scalability
    {
        public const int X = 0;
        public const int Y = 1;
        public const int Z = 2;

        public readonly int Axis;
        public readonly float MaxScale;
        public readonly float MinScale;
        public float Scale;

        public Scalability(int axis, float minScale, float maxScale)
        {
            Axis = axis;
            MinScale = minScale;
            MaxScale = maxScale;
            Scale = MathF.Max(minScale, MathF.Min(1, maxScale));
        }
    }

    private class Bone
    {
        private readonly Quaternion _defaultLocalRotation;
        private readonly Vector3 _defaultLocalScale;
        private readonly Transform _offsetter;
        private readonly Func<Quaternion, Quaternion> _rotationLimit;

        public readonly Vector3 Axis;
        public readonly Scalability Scale;
        public readonly Transform Transform;

        public bool Limited;
        public float ReferenceDeviationTolerance;

        public Vector3? ReferencePosition;

        public Vector3 SolverPosition;
        public Quaternion SolverRotation;


        public Bone(Transform self, Transform parent, Transform child,
            Func<Quaternion, Quaternion> rotationLimit = null, Scalability scale = null)
        {
            Transform = self;
            Axis = child != null
                ? Quaternion.Inverse(self.rotation) * (child.position - self.position)
                : Vector3.zero;
            SolverPosition = self.position;
            SolverRotation = self.rotation;
            ReferencePosition = null;
            ReferenceDeviationTolerance = 0;
            var parentRotation = parent?.rotation ?? self.parent?.rotation ?? Quaternion.identity;
            _defaultLocalScale = Transform.localScale;
            _defaultLocalRotation = Quaternion.Inverse(parentRotation) * self.rotation;

            _rotationLimit = rotationLimit ?? (q => q);
            Limited = false;

            Scale = scale;
            if (scale != null) _offsetter = IKUtils.AddScaleOffsetter(Transform);
        }


        public Vector3 ScaledAxis
        {
            get
            {
                if (Scale == null) return Axis;
                var ret = Axis;
                ret[Scale.Axis] *= Scale.Scale;
                return ret;
            }
        }

        public void ApplyScale()
        {
            if (Scale == null)
                return;
            var newScale = _defaultLocalScale;
            newScale[Scale.Axis] *= Scale.Scale;
            var offsetterScale = Vector3.one;
            offsetterScale[Scale.Axis] /= Scale.Scale;
            Transform.localScale = newScale;
            _offsetter.localScale = offsetterScale;
        }

        public Quaternion GetLimitedLocalRotation(Quaternion localRotation, out bool changed)
        {
            var relativeRotation = Quaternion.Inverse(_defaultLocalRotation) * localRotation;
            var limitedRotation = _rotationLimit(relativeRotation);
            changed = relativeRotation != limitedRotation;
            return _defaultLocalRotation * limitedRotation;
        }

        public float GetDeviation()
        {
            return ReferencePosition == null
                ? 0.0f
                : Vector3.Distance((Vector3) ReferencePosition, SolverPosition);
        }

        public bool IsWithinTolerance()
        {
            if (ReferencePosition == null)
                return true;
            return Vector3.Distance((Vector3) ReferencePosition, SolverPosition) <=
                   ReferenceDeviationTolerance;
        }

        public Vector3 GetPositionWithReference()
        {
            if (ReferencePosition == null)
                return SolverPosition;
            // return (SolverPosition + (Vector3) ReferencePosition) / 2;
            var distance = Vector3.Distance(SolverPosition, (Vector3) ReferencePosition);
            if (distance <= ReferenceDeviationTolerance)
                return (Vector3) ReferencePosition;
            return Vector3.Lerp(SolverPosition, (Vector3) ReferencePosition,
                ReferenceDeviationTolerance / distance);
        }

        public Bone Clone()
        {
            return (Bone) MemberwiseClone();
        }
    }
}