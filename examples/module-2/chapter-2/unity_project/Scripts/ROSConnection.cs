using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Manages the ROS-Unity TCP connection.
///
/// This script initializes the connection to the ROS-TCP-Endpoint
/// and provides a foundation for publishing and subscribing to ROS topics.
///
/// Usage:
///   1. Attach this script to an empty GameObject in your scene
///   2. Ensure ROS Settings are configured (Robotics > ROS Settings)
///   3. Start the ROS-TCP-Endpoint before entering Play mode
///
/// Requirements:
///   - Unity 2022.3 LTS
///   - ROS-TCP-Connector package installed
///   - ROS-TCP-Endpoint running on ROS 2 machine
/// </summary>
public class ROSConnectionManager : MonoBehaviour
{
    [Header("Connection Settings")]
    [Tooltip("Name to identify this Unity instance in ROS")]
    public string unityNodeName = "unity_digital_twin";

    [Header("Debug Settings")]
    [Tooltip("Log incoming messages to console")]
    public bool logMessages = true;

    // Reference to the ROS connection singleton
    private ROSConnection ros;

    // Connection state
    private bool isConnected = false;

    void Start()
    {
        // Get or create the ROS connection singleton
        ros = ROSConnection.GetOrCreateInstance();

        // Register publishers
        RegisterPublishers();

        // Register subscribers
        RegisterSubscribers();

        // Send initial connection status
        PublishStatus("Unity connected: " + unityNodeName);

        Debug.Log($"[ROSConnection] Initialized as '{unityNodeName}'");
    }

    void RegisterPublishers()
    {
        // Status messages from Unity
        ros.RegisterPublisher<StringMsg>("/unity/status");

        // Add more publishers as needed:
        // ros.RegisterPublisher<JointStateMsg>("/unity/joint_states");
        // ros.RegisterPublisher<PoseMsg>("/unity/robot_pose");
    }

    void RegisterSubscribers()
    {
        // Commands from ROS
        ros.Subscribe<StringMsg>("/ros/commands", OnCommandReceived);

        // Add more subscribers as needed:
        // ros.Subscribe<JointStateMsg>("/joint_commands", OnJointCommand);
        // ros.Subscribe<TwistMsg>("/cmd_vel", OnVelocityCommand);
    }

    void OnCommandReceived(StringMsg msg)
    {
        if (logMessages)
        {
            Debug.Log($"[ROSConnection] Command received: {msg.data}");
        }

        // Process commands
        ProcessCommand(msg.data);
    }

    void ProcessCommand(string command)
    {
        // Parse and execute commands from ROS
        switch (command.ToLower())
        {
            case "reset":
                ResetSimulation();
                break;
            case "pause":
                Time.timeScale = 0f;
                break;
            case "resume":
                Time.timeScale = 1f;
                break;
            default:
                Debug.LogWarning($"[ROSConnection] Unknown command: {command}");
                break;
        }
    }

    void ResetSimulation()
    {
        Debug.Log("[ROSConnection] Resetting simulation...");
        // Implement reset logic here
        // e.g., reset robot position, clear obstacles, etc.
    }

    /// <summary>
    /// Publish a status message to ROS.
    /// </summary>
    /// <param name="message">Status message content</param>
    public void PublishStatus(string message)
    {
        var msg = new StringMsg(message);
        ros.Publish("/unity/status", msg);

        if (logMessages)
        {
            Debug.Log($"[ROSConnection] Published status: {message}");
        }
    }

    void Update()
    {
        // Periodic status updates (every 5 seconds)
        if (Time.frameCount % (60 * 5) == 0)
        {
            PublishStatus($"Heartbeat from {unityNodeName}");
        }
    }

    void OnApplicationQuit()
    {
        PublishStatus("Unity disconnecting");
        Debug.Log("[ROSConnection] Shutting down...");
    }
}
