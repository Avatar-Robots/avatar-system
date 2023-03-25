using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Vuforia;
using CompressionLevel = System.IO.Compression.CompressionLevel;
#if ENABLE_WINMD_SUPPORT
using Windows.Storage;
using System.Runtime.InteropServices.WindowsRuntime;
#endif

public class JointPositionRetriever : MonoBehaviour
{
    private const int Left = 0;
    private const int Right = 10;
    private const int NCoords = 2 * Right;

    private const int Hand = 7;
    private const int Wrist = 6;
    private const int Elbow = 3;
    private const int UpperArm = 2;
    private const int UpperArmRoot = 8;
    private const int ForeArm = 4;
    private const int ForeArmRoot = 9;

    private readonly object _coordsLock = new();
    private ArmConfiguration _avatarLeftArm;
    private ArmConfiguration _avatarRightArm;

    private Vector3? _basePosition;
    private Quaternion? _baseRotation;
    private bool _connected;

    private Vector3[] _coords;
    private float[] _grippers;
    private float[] _angles;
    private GameObject[] _indicators;
    private Recording _recording;
    private Quaternion[] _rotations;

    public Transform CameraTransform;
    public GameObject Avatar;
    public float AvatarOffset = 0.00f;
    public float FilterAlpha = 0.9f;
    public string HostIp;
    public GameObject IndicatorSpherePrefab;
    public ModelTargetBehaviour ModelTarget;
    public int Port;

    public bool Record = false;
    public int Timeout = 2000;

    [NonSerialized]
    public string DebugInfo = "";

    [NonSerialized]
    public TimeSpan AverageUpdateInterval = TimeSpan.Zero;

#if ENABLE_WINMD_SUPPORT
    private StorageFile _storageFile;
#endif

    // Start is called before the first frame update
    private void Start()
    {
        lock (_coordsLock)
        {
            _coords = new Vector3[NCoords];
            _rotations = new Quaternion[2];
            _angles = new float[14];
            _grippers = new float[2];
        }

        CameraTransform = VuforiaBehaviour.Instance.transform;

        if (IndicatorSpherePrefab != null)
        {
            _indicators = new GameObject[NCoords + 1];
            for (var i = 0; i < NCoords + 1; i++)
            {
                _indicators[i] = Instantiate(IndicatorSpherePrefab, Vector3.zero,
                    Quaternion.identity,
                    transform);
                _indicators[i].name = $"Joint position indicator {i}";
            }
        }

        Task.Run(Loop);

        if (Avatar == null) return;
        _avatarLeftArm = ArmConfiguration.GetInstance(Avatar.transform, ArmSide.Left);
        _avatarRightArm = ArmConfiguration.GetInstance(Avatar.transform, ArmSide.Right);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (_indicators is not {Length: NCoords + 1}) return;
        Draw(Left);
        Draw(Right);

        if (Application.isPlaying)
        {
            Gizmos.color = Color.red;
            // Handles.Label(_indicators[NCoords].transform.position, $"Update interval: {_updateInterval.TotalSeconds}s");
        }


        void Draw(int side)
        {
            Vector3? dummy = null;
            var te = ComputeTrueElbow(side, out dummy);
            Gizmos.color = Color.green;
            Gizmos.DrawLine(_indicators[side + UpperArm].transform.position, te);
            Gizmos.DrawLine(_indicators[side + ForeArmRoot].transform.position, te);
        }
    }
#endif

    /*
     * Computes the "true elbow position" as the midpoint of the common perpendicular of upper arm
     * segment and the fore arm segment.
     */
    private Vector3 ComputeTrueElbow(int side, out Vector3? axis)
    {
        var r1 = RobotToWorld(_coords[side + UpperArmRoot]);
        var c1 = RobotToWorld(_coords[side + UpperArm]);
        var v1 = (c1 - r1).normalized;
        var r2 = RobotToWorld(_coords[side + ForeArmRoot]);
        var c2 = RobotToWorld(_coords[side + ForeArm]);
        var v2 = (c2 - r2).normalized;
        var k2 = Vector3.Dot(c1 - r2 + Vector3.Dot(r2 - c1, v1) * v1, v2) /
                 (1 - Vector3.Dot(v1, v2) * Vector3.Dot(v1, v2));
        var p2 = r2 + k2 * v2;
        var p1 = Vector3.Project(p2 - c1, v1) + c1;
        axis = Vector3.Normalize(p2 - p1);
        return (p2 + p1) / 2;
    }

    // Update is called once per frame
    private void Update()
    {
        if (ModelTarget.TargetStatus.Status == Status.TRACKED &&
            ModelTarget.TargetStatus.StatusInfo == StatusInfo.NORMAL)
        {
            _basePosition = _basePosition == null
                ? ModelTarget.transform.position
                : Vector3.Lerp(ModelTarget.transform.position, (Vector3) _basePosition,
                    FilterAlpha);
            _baseRotation = _baseRotation == null
                ? ModelTarget.transform.rotation
                : Quaternion.Lerp(ModelTarget.transform.rotation, (Quaternion) _baseRotation,
                    FilterAlpha);
        }

        lock (_coordsLock)
        {
            if (IndicatorSpherePrefab != null)
            {
                for (var i = 0; i < NCoords; i++)
                    _indicators[i].transform.position = RobotToWorld(_coords[i]);
                Vector3? dummyAxis = null;
                _indicators[Left + Elbow].transform.position =
                    ComputeTrueElbow(Left, out dummyAxis);
                _indicators[Right + Elbow].transform.position =
                    ComputeTrueElbow(Right, out dummyAxis);
                _indicators[NCoords].transform.position = RobotToWorld(Vector3.zero);
                _indicators[NCoords].transform.rotation = _rotations[0];
            }

            if (_connected)
            {
                _avatarLeftArm.RawHandReferenceOrientation = GetBaseRotation() * _rotations[0];
                _avatarLeftArm.WristReferencePosition = RobotToWorld(_coords[Left + Wrist]);
                var temp = ComputeTrueElbow(Left, out _avatarLeftArm.ElbowReferenceAxis);
                _avatarLeftArm.ElbowReferencePosition =
                    Vector3.Lerp(RobotToWorld(_coords[Left + Elbow]), temp, 0.8f);


                _avatarRightArm.RawHandReferenceOrientation = GetBaseRotation() * _rotations[1];
                _avatarRightArm.WristReferencePosition = RobotToWorld(_coords[Right + Wrist]);
                temp = ComputeTrueElbow(Right, out _avatarRightArm.ElbowReferenceAxis);
                _avatarRightArm.ElbowReferencePosition =
                    Vector3.Lerp(RobotToWorld(_coords[Right + Elbow]), temp, 0.8f);

                _avatarLeftArm.GripperPosition = _grippers[0];
                _avatarRightArm.GripperPosition = _grippers[1];
            }
        }

        if (Record)
            UpdateRecording();


        if (Avatar == null) return;

        var leftShoulder = RobotToWorld(_coords[Left]);
        var rightShoulder = RobotToWorld(_coords[Right]);

        var forward = Vector3.Cross(rightShoulder - leftShoulder, Vector3.up);
        Avatar.transform.rotation = Quaternion.LookRotation(forward, Vector3.up);
        var avatarShoulderCenter =
            (_avatarLeftArm.UpperArmTransform.position +
             _avatarRightArm.UpperArmTransform.position) / 2;
        var yumiShoulderCenter =
            (RobotToWorld(_coords[Left + 1]) + RobotToWorld(_coords[Right + 1])) / 2;
        Avatar.transform.position +=
            yumiShoulderCenter + AvatarOffset * forward - avatarShoulderCenter;
    }

    private void UpdateRecording()
    {
        if (CameraTransform == null) return;
        _recording ??= new Recording
        {
            Version = 1,
            Items = new List<RecordingItem>(),
            FileName = $"Session-{DateTime.Now:MM-dd-HH-mm-ss}.jz"
        };
        var newItem = new RecordingItem
        {
            Time = Time.timeSinceLevelLoad,
            BasePosition = _basePosition ?? Vector3.zero,
            BaseRotation = _baseRotation ?? Quaternion.identity,
            CameraPosition = CameraTransform.position,
            CameraRotation = CameraTransform.rotation,
            GripperRotations = new[]
                {GetBaseRotation() * _rotations[0], GetBaseRotation() * _rotations[1]},
            JointAngles = (float[]) _angles.Clone(),
            // JointCoords = (Vector3[]) _coords.Clone(),
            AvatarPosition = Avatar.transform.position,
            AvatarRotation = Avatar.transform.rotation,
            AvatarArmConfigs = new[] {new ArmState(_avatarLeftArm), new ArmState(_avatarRightArm)}
        };
        _recording.Items.Add(newItem);
    }

    public async void SaveRecording()
    {
        if (!Record || _recording == null) return;
        var json = JsonUtility.ToJson(_recording);
        var encoded = Encoding.UTF8.GetBytes(json);
        byte[] compressed;
        using (var memoryStream = new MemoryStream())
        {
            await using (var gZipStream = new GZipStream(memoryStream, CompressionLevel.Optimal))
            {
                gZipStream.Write(encoded, 0, encoded.Length);
            }

            compressed = memoryStream.ToArray();
        }

        DebugInfo = $"JSON length: {json.Length} Compressed: {compressed.Length}";
        try
        {
#if ENABLE_WINMD_SUPPORT
            _storageFile ??=
 await DownloadsFolder.CreateFileAsync(_recording.FileName, CreationCollisionOption.GenerateUniqueName);
            await FileIO.WriteBufferAsync(_storageFile, compressed.AsBuffer());
#else
            await File.WriteAllBytesAsync(_recording.FileName, compressed);
#endif
        }
        catch (Exception e)
        {
            DebugInfo += $"\nError: {e}";
        }
    }

    private Quaternion GetBaseRotation()
    {
        return Application.isEditor || _baseRotation == null
            ? Quaternion.Euler(-90, 180, 90)
            : (Quaternion) _baseRotation;
    }

    private Vector3 RobotToWorld(Vector3 coord)
    {
        return (_basePosition ?? Vector3.zero) +
               GetBaseRotation() * new Vector3(-coord.x, coord.y + 200, coord.z) * 1e-3f;
    }

    private void Loop()
    {
        while (true)
        {
            Socket socket = null;
            try
            {
                _connected = false;
                socket = new Socket(SocketType.Stream, ProtocolType.Tcp);
                socket.SendTimeout = Timeout;
                socket.ReceiveTimeout = Timeout;
                socket.Connect(HostIp, Port);

                var buffer = new byte[4096];
                var lastUpdate = DateTime.Now;
                while (true)
                {
                    _connected = true;
                    socket.Send(Encoding.UTF8.GetBytes(" "));
                    var count = socket.Receive(buffer);
                    const int floatWidth = 4;
                    var numbers = new float[count / floatWidth];
                    if (BitConverter.IsLittleEndian)
                    {
                        for (var i = 0; i < count; i += floatWidth)
                            Array.Reverse(buffer, i, floatWidth);
                    }

                    for (var i = 0; i < numbers.Length; i++)
                        numbers[i] = BitConverter.ToSingle(buffer, i * floatWidth);

                    const int sides = 2;
                    const int numPerCoord = 3;
                    const int sizeJointCoords = numPerCoord * NCoords;
                    const int coordPerOrientation = 2;
                    const int sizeOrientations = sides * coordPerOrientation * numPerCoord;
                    const int sizeGripperPos = sides * 1;
                    const int sizeJointAngles = sides * 7;
                    const int sizeTotal = sizeJointCoords + sizeOrientations + sizeGripperPos +
                                          sizeJointAngles;

                    if (numbers.Length == sizeTotal)
                    {
                        lock (_coordsLock)
                        {
                            for (var i = 0; i < NCoords; i++)
                            {
                                for (var j = 0; j < numPerCoord; j++)
                                    _coords[i][j] = numbers[numPerCoord * i + j];
                            }

                            var offset = sizeJointCoords;

                            for (var i = 0; i < sides; i++)
                            {
                                var forward = new Vector3(-numbers[offset], numbers[offset + 1],
                                    numbers[offset + 2]);
                                var upwards = new Vector3(-numbers[offset + 3], numbers[offset + 4],
                                    numbers[offset + 5]);
                                _rotations[i] = Quaternion.LookRotation(forward, upwards);
                                offset += coordPerOrientation * numPerCoord;
                            }

                            for (var i = 0; i < sides; i++)
                                _grippers[i] = numbers[offset + i] / 25;

                            offset += sizeGripperPos;
                            for (var i = 0; i < sizeJointAngles; i++)
                                _angles[i] = numbers[offset + i];
                        }
                    }

                    Thread.Sleep(10);
                    AverageUpdateInterval = DateTime.Now - lastUpdate;
                    lastUpdate = DateTime.Now;
                }
            }
            catch (SocketException)
            {
                _connected = false;
                try
                {
                    socket?.Close();
                }
                catch
                {
                    // Do nothing.
                }

                Debug.LogWarning("Disconnection detected! Reconnecting in 1s.");
                Thread.Sleep(1000);
            }
        }
    }
}