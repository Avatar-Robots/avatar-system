using System;
using System.Diagnostics.CodeAnalysis;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

public class JacobianIK : MonoBehaviour
{
    private readonly float _elbowTwist = 0;
    private ArmConfiguration _config;

    private float _elbowAngle;
    private Vector3 _foreArm;

    private Quaternion _foreArmDefaultRotation;
    private Transform _foreArmOffsetter;
    private float _foreArmScale = 1;

    private Vector3 _upperArm;

    private Quaternion _upperArmDefaultRotation;

    private Transform _upperArmOffsetter;
    private float _upperArmScale = 1;

    private float _upperArmSwingX;
    private float _upperArmSwingZ;
    private float _upperArmTwist;

    public ArmSide Side;
    public double DampingFactor = 0.5;

    // Start is called before the first frame update
    private void Start()
    {
        _config = ArmConfiguration.GetInstance(transform, Side);
        _upperArmDefaultRotation = Quaternion.Inverse(_config.ShoulderTransform.rotation) *
                                   _config.UpperArmTransform.rotation;
        _upperArm = Quaternion.Inverse(_config.UpperArmTransform.rotation) *
                    (_config.ForeArmTransform.position - _config.UpperArmTransform.position);
        _foreArmDefaultRotation = Quaternion.Inverse(_config.UpperArmTransform.rotation) *
                                  _config.ForeArmTransform.rotation;
        _foreArm = Quaternion.Inverse(_config.ForeArmTransform.rotation) *
                   (_config.WristTransform.position - _config.ForeArmTransform.position);

        _upperArmOffsetter = IKUtils.AddScaleOffsetter(_config.UpperArmTransform);
        _foreArmOffsetter = IKUtils.AddScaleOffsetter(_config.ForeArmTransform);

        /*
        _elbowAngle = 45 * Mathf.Deg2Rad;
        _upperArmTwist = 0 * Mathf.Deg2Rad;
        _config.UpperArmTransform.rotation = GetUpperArmRotation();
        _config.ForeArmTransform.rotation =
            _config.UpperArmTransform.rotation * GetForeArmRotation();
        */
        var sign = Side == ArmSide.Left ? 1 : -1;
        _upperArmTwist = sign * -40 * Mathf.Deg2Rad;
        _config.UpperArmTransform.rotation =
            Quaternion.AngleAxis(_upperArmTwist * Mathf.Rad2Deg, _config.UpperArmTransform.up) *
            _config.UpperArmTransform.rotation;
        _elbowAngle = 80 * Mathf.Deg2Rad;
        _config.ForeArmTransform.rotation =
            _config.ForeArmTransform.rotation * Quaternion.AngleAxis(_elbowAngle * Mathf.Rad2Deg, _config.ElbowAxis);
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

    [SuppressMessage("ReSharper", "InconsistentNaming")]
    private void ComputeIK()
    {
        if (_config.ElbowReferencePosition == null ||
            _config.WristReferencePosition == null) return;
        var error = UnityVectorToVector((Vector3) _config.WristReferencePosition -
                                        _config.WristTransform.position);
        var refError = 0.1 * UnityVectorToVector((Vector3) _config.ElbowReferencePosition -
                                                 _config.ForeArmTransform.position);

        var M = Matrix<double>.Build;
        var V = Vector<double>.Build;

        var upperArmSwingAxis = V.DenseOfArray(new double[] {_upperArmSwingX, 0, _upperArmSwingZ});
        var upperArmBaseRotation = _config.ShoulderTransform.rotation * _upperArmDefaultRotation;
        var swingInverse = Quaternion.Inverse(upperArmBaseRotation * GetUpperArmSwing());

        var JUpperArmSwing = QuaternionToRotationMatrix(upperArmBaseRotation) * GetJacobianOfExpMap(
            upperArmSwingAxis,
            swingInverse * (_config.WristTransform.position - _config.UpperArmTransform.position));
        var JUpperArmSwingRef = QuaternionToRotationMatrix(upperArmBaseRotation) *
                                GetJacobianOfExpMap(upperArmSwingAxis,
                                    swingInverse * (_config.ForeArmTransform.position -
                                                    _config.UpperArmTransform.position));
        var JUpperArmScale =
            UnityVectorToVector(_config.UpperArmTransform.up * _upperArm.magnitude);
        var JForeArmScale = UnityVectorToVector(_config.ForeArmTransform.up * _foreArm.magnitude);
        var JUpperArmTwist = UnityVectorToVector(Vector3.Cross(
            _config.UpperArmTransform.up,
            _config.WristTransform.position - _config.UpperArmTransform.position
        ));

        var JElbowAngle = UnityVectorToVector(Vector3.Cross(
            _config.UpperArmTransform.rotation * _config.ElbowAxis,
            _config.WristTransform.position - _config.ForeArmTransform.position));

        var J = M.DenseOfMatrixArray(new[,]
        {
            {
                JUpperArmSwing.Column(0).ToColumnMatrix(),
                JUpperArmSwing.Column(2).ToColumnMatrix(),
                JUpperArmTwist.ToColumnMatrix(),
                JElbowAngle.ToColumnMatrix(),
                JUpperArmScale.ToColumnMatrix(),
                JForeArmScale.ToColumnMatrix()
            },
            {
                JUpperArmSwingRef.Column(0).ToColumnMatrix(),
                JUpperArmSwing.Column(2).ToColumnMatrix(),
                M.Dense(3, 1),
                M.Dense(3, 1),
                JUpperArmScale.ToColumnMatrix(),
                M.Dense(3, 1)
            }
        });

        error = error.ToColumnMatrix().Stack(refError.ToColumnMatrix()).Column(0);
        var delta = ApplyDLS(J, error);

        _upperArmSwingX += (float) delta[0];
        _upperArmSwingZ += (float) delta[1];
        _upperArmTwist = Mathf.Clamp(_upperArmTwist + (float) delta[2], -ArmConfiguration.MaxShoulderTwist * Mathf.Deg2Rad,
            ArmConfiguration.MaxShoulderTwist * Mathf.Deg2Rad);
        _elbowAngle = Mathf.Clamp(_elbowAngle + (float) delta[3], 0,
            ArmConfiguration.MaxElbowAngle * Mathf.Deg2Rad);
        _upperArmScale = Mathf.Clamp(_upperArmScale + (float) delta[4],
            ArmConfiguration.MinUpperArmScale, ArmConfiguration.MaxUpperArmScale);
        _foreArmScale = Mathf.Clamp(_foreArmScale + (float) delta[5],
            ArmConfiguration.MinForeArmScale, ArmConfiguration.MaxForeArmScale);

        var theta = GetUpperArmTheta();
        if (theta > MathF.PI)
        {
            _upperArmSwingX *= (theta - 2 * MathF.PI) / theta;
            _upperArmSwingZ *= (theta - 2 * MathF.PI) / theta;
        }

        theta = GetUpperArmTheta();
        if (theta > ArmConfiguration.MaxShoulderAngle * Mathf.Deg2Rad)
        {
            _upperArmSwingX *= ArmConfiguration.MaxShoulderAngle * Mathf.Deg2Rad / theta;
            _upperArmSwingZ *= ArmConfiguration.MaxShoulderAngle * Mathf.Deg2Rad / theta;
        }

        _config.UpperArmTransform.rotation =
            _config.ShoulderTransform.rotation * _upperArmDefaultRotation *
            GetUpperArmSwing() *
            Quaternion.AngleAxis(_upperArmTwist * Mathf.Rad2Deg, Vector3.up);
        _config.ForeArmTransform.rotation =
            _config.UpperArmTransform.rotation *
            Quaternion.AngleAxis(_elbowAngle * Mathf.Rad2Deg, _config.ElbowAxis) *
            _foreArmDefaultRotation *
            Quaternion.AngleAxis(_elbowTwist * Mathf.Rad2Deg, Vector3.up);

        _config.UpperArmTransform.localScale = new Vector3(1, _upperArmScale, 1);
        _upperArmOffsetter.localScale = new Vector3(1, 1 / _upperArmScale, 1);
        _config.ForeArmTransform.position =
            _config.UpperArmTransform.rotation * _upperArm * _upperArmScale +
            _config.UpperArmTransform.position;
        _config.ForeArmTransform.localScale = new Vector3(1, _foreArmScale, 1);
        _foreArmOffsetter.localScale = new Vector3(1, 1 / _foreArmScale, 1);
        _config.WristTransform.position =
            _config.ForeArmTransform.rotation * _foreArm * _foreArmScale +
            _config.ForeArmTransform.position;


        if (_config.HandReferenceOrientation == null) return;

        var foreArm = (Vector3) _config.WristReferencePosition - _config.ForeArmTransform.position;
        var projectedElbowForward = _config.ForeArmTransform.forward;
        Vector3.OrthoNormalize(ref foreArm, ref projectedElbowForward);
        var projectedWristForward = (Quaternion) _config.HandReferenceOrientation * Vector3.forward;
        Vector3.OrthoNormalize(ref foreArm, ref projectedWristForward);

        _config.ForeArmTransform.rotation = Quaternion.Lerp(Quaternion.identity,
            Quaternion.FromToRotation(projectedElbowForward, projectedWristForward),
            0.4f) * _config.ForeArmTransform.rotation;

        _config.WristTransform.rotation = (Quaternion) _config.HandReferenceOrientation;
    }

    private Quaternion GetUpperArmSwing()
    {
        var theta = GetUpperArmTheta();
        return Quaternion.AngleAxis(theta * Mathf.Rad2Deg,
            new Vector3(_upperArmSwingX / theta, 0, _upperArmSwingZ / theta));
    }

    private float GetUpperArmTheta()
    {
        return MathF.Sqrt(_upperArmSwingX * _upperArmSwingX + _upperArmSwingZ * _upperArmSwingZ);
    }

    [SuppressMessage("ReSharper", "InconsistentNaming")]
    private Vector<double> ApplyJacobianTranspose(Matrix<double> J, Vector<double> error)
    {
        var JTe = J.TransposeThisAndMultiply(error);
        var JJTe = J.TransposeAndMultiply(J) * error;
        return JTe.DotProduct(error) / JJTe.DotProduct(JJTe) * JTe;
    }

    [SuppressMessage("ReSharper", "InconsistentNaming")]
    private Vector<double> ApplyDLS(Matrix<double> J, Vector<double> error)
    {
        var M = Matrix<double>.Build;
        return J.TransposeThisAndMultiply((J.TransposeAndMultiply(J) +
                                           M.DenseDiagonal(J.RowCount,
                                               DampingFactor * DampingFactor)).Solve(error));
        var extendedError = error.ToColumnMatrix().Stack(M.Dense(J.ColumnCount, 1)).Column(0);
        return M.DenseOfMatrixArray(new [,]
        {
            {J}, {M.DenseDiagonal(J.ColumnCount, DampingFactor * DampingFactor)}
        }).Svd().Solve(extendedError);
    }

    private Vector<double> UnityVectorToVector(Vector3 x)
    {
        return Vector<double>.Build.DenseOfArray(new double[] {x.x, x.y, x.z});
    }

    // ReSharper disable once InconsistentNaming
    private Matrix<double> GetJacobianOfQuaternion(double x, double y, double z, double w,
        Vector3 u_)
    {
        // ReSharper disable once InconsistentNaming
        var M = Matrix<double>.Build;
        // ReSharper disable once InconsistentNaming
        var V = Vector<double>.Build;

        var uSkew = M.DenseOfArray(new double[,]
        {
            {0, -u_.z, u_.y},
            {u_.z, 0, -u_.x},
            {-u_.y, u_.x, 0}
        });
        var u = V.DenseOfArray(new double[] {u_.x, u_.y, u_.z});
        var v = V.DenseOfArray(new[] {x, y, z});
        return 2 * M.DenseOfMatrixArray(new[,]
        {
            {
                v.OuterProduct(u) - 2 * u.OuterProduct(v) - w * uSkew +
                M.DenseDiagonal(3, v.DotProduct(u)),
                (-uSkew * v).ToColumnMatrix()
            }
        });
    }

    private Matrix<double> QuaternionToRotationMatrix(Quaternion q)
    {
        var x = q * Vector3.right;
        var y = q * Vector3.up;
        var z = q * Vector3.forward;
        return Matrix<double>.Build.DenseOfArray(new double[,]
        {
            {x.x, y.x, z.x},
            {x.y, y.y, z.y},
            {x.z, y.z, z.z}
        });
    }

    private Matrix<double> GetJacobianOfExpMap(Vector<double> r, Vector3 u)
    {
        const double cutOff = 1e-4;

        // ReSharper disable once InconsistentNaming
        var M = Matrix<double>.Build;

        var theta = r.L2Norm();
        var s = theta > cutOff ? Math.Sin(theta / 2) / theta : 0.5 - theta * theta / 48;
        var c = Math.Cos(theta);
        var m = theta > cutOff ? (c / 2 - s) / (theta * theta) : -1.0 / 24 + theta * theta / 960;
        return GetJacobianOfQuaternion(s * r[0], s * r[1], s * r[2], c, u) *
               M.DenseOfMatrixArray(new[,]
               {
                   {M.DenseDiagonal(3, s) + m * r.OuterProduct(r)},
                   {(-s / 2 * r).ToRowMatrix()}
               });
    }
}