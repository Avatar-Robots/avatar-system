using System.Linq;
using TMPro;
using UnityEngine;
using Vuforia;

public class DebugOverlay : MonoBehaviour
{
    private ArmConfiguration[] _armConfigs;
    private Transform _cameraTransform;

    private TouchScreenKeyboard _keyboard;
    public Transform AvatarTransform;
    public JointPositionRetriever JointPositionClient;

    public ModelTargetBehaviour ModelTarget;
    public TextMeshPro TextMesh;

    public void OnSaveRecordingButtonClicked()
    {
        JointPositionClient.SaveRecording();
    }

    public void OnToggleRecordingButtonClicked()
    {
        JointPositionClient.Record = !JointPositionClient.Record;
    }

    public void OnChangeIPButtonClicked()
    {
        _keyboard = TouchScreenKeyboard.Open(JointPositionClient.HostIp,
            TouchScreenKeyboardType.DecimalPad, false, false, false);
    }

    // Start is called before the first frame update
    private void Start()
    {
        _cameraTransform = VuforiaBehaviour.Instance.transform;
        _armConfigs = AvatarTransform.GetComponents<ArmConfiguration>();
        /*
        var host = Dns.GetHostEntry(Dns.GetHostName());
        _ips = string.Join(", ",
            host.AddressList.Where(ip => ip.AddressFamily == AddressFamily.InterNetwork)
                .OrderBy(ip => ip.ToString()));
        */
    }

    // Update is called once per frame
    private void Update()
    {
        var text = $"Recording {(JointPositionClient.Record ? "on" : "off")}\n";
        text += $"Coords: {_cameraTransform.position}\n";
        if (ModelTarget != null)
        {
            text +=
                $"Tracking status: {ModelTarget.TargetStatus.StatusInfo} - {ModelTarget.TargetStatus.Status}\n";
            text +=
                $"Target pos: {ModelTarget.transform.position} rot: {ModelTarget.transform.rotation.eulerAngles}\n";
        }

        text +=
            $"Joint sync interval: {JointPositionClient.AverageUpdateInterval.TotalMilliseconds:000.00}ms, IK Time:";
        text = _armConfigs.Aggregate(text,
            (current, config) => current + $" {config.MillisecondsSpentInIK:00.000}ms") + "\n";

        var serverIp = _keyboard == null ? JointPositionClient.HostIp : _keyboard.text;
        if (_keyboard is
            {
                status: TouchScreenKeyboard.Status.Done
                or TouchScreenKeyboard.Status.Canceled
            })
        {
            JointPositionClient.HostIp = _keyboard.text;
            _keyboard = null;
        }

        text += $"Server IP: {serverIp}\n";
        text += JointPositionClient.DebugInfo;
        TextMesh.text = text;
    }
}