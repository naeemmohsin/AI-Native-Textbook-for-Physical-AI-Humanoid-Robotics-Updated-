using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

/// <summary>
/// Manages Human-Robot Interaction (HRI) scenarios in Unity.
///
/// Features:
/// - Proximity detection between human and robot
/// - Safety zone visualization
/// - State notifications to ROS 2
/// - Configurable safety behaviors
///
/// Usage:
///   1. Attach this script to an HRI manager GameObject
///   2. Assign robot and human transforms
///   3. Configure safety distance and behaviors
///
/// Requirements:
///   - Robot and human GameObjects in scene
///   - ROSConnectionManager for ROS notifications
/// </summary>
public class HRIManager : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Transform of the robot's base or center")]
    public Transform robot;

    [Tooltip("Transform of the human avatar")]
    public Transform human;

    [Header("Safety Settings")]
    [Tooltip("Distance at which safety behavior activates (meters)")]
    public float safetyDistance = 1.5f;

    [Tooltip("Critical distance for emergency stop (meters)")]
    public float criticalDistance = 0.5f;

    [Tooltip("Color of safety zone when safe")]
    public Color safeColor = Color.green;

    [Tooltip("Color of safety zone when warning")]
    public Color warningColor = Color.yellow;

    [Tooltip("Color of safety zone when critical")]
    public Color criticalColor = Color.red;

    [Header("Behavior Settings")]
    [Tooltip("Slow robot when human enters safety zone")]
    public bool slowOnProximity = true;

    [Tooltip("Speed multiplier when in warning zone (0-1)")]
    [Range(0f, 1f)]
    public float warningSpeedFactor = 0.5f;

    [Tooltip("Stop robot when human enters critical zone")]
    public bool stopOnCritical = true;

    [Header("ROS Settings")]
    [Tooltip("Publish HRI events to ROS")]
    public bool publishToROS = true;

    [Tooltip("Topic for HRI state messages")]
    public string hriStateTopic = "/unity/hri_state";

    // State tracking
    private enum ProximityState { Safe, Warning, Critical }
    private ProximityState currentState = ProximityState.Safe;
    private ProximityState previousState = ProximityState.Safe;

    // ROS connection
    private ROSConnection ros;

    // Cached distance
    private float currentDistance;

    void Start()
    {
        // Get ROS connection if publishing enabled
        if (publishToROS)
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<StringMsg>(hriStateTopic);
        }

        // Validate references
        if (robot == null)
        {
            Debug.LogError("[HRIManager] Robot transform not assigned!");
        }
        if (human == null)
        {
            Debug.LogWarning("[HRIManager] Human transform not assigned - HRI disabled");
        }

        Debug.Log($"[HRIManager] Initialized with safety distance: {safetyDistance}m");
    }

    void Update()
    {
        if (robot == null || human == null) return;

        // Calculate distance between robot and human
        currentDistance = Vector3.Distance(robot.position, human.position);

        // Determine proximity state
        UpdateProximityState();

        // Handle state changes
        if (currentState != previousState)
        {
            OnStateChanged();
            previousState = currentState;
        }
    }

    void UpdateProximityState()
    {
        if (currentDistance < criticalDistance)
        {
            currentState = ProximityState.Critical;
        }
        else if (currentDistance < safetyDistance)
        {
            currentState = ProximityState.Warning;
        }
        else
        {
            currentState = ProximityState.Safe;
        }
    }

    void OnStateChanged()
    {
        string stateMessage = "";

        switch (currentState)
        {
            case ProximityState.Safe:
                stateMessage = "SAFE";
                OnHumanDepart();
                break;

            case ProximityState.Warning:
                stateMessage = "WARNING";
                OnHumanApproach();
                break;

            case ProximityState.Critical:
                stateMessage = "CRITICAL";
                OnHumanCritical();
                break;
        }

        Debug.Log($"[HRIManager] State changed to: {stateMessage} (distance: {currentDistance:F2}m)");

        // Publish to ROS
        if (publishToROS && ros != null)
        {
            var msg = new StringMsg($"{stateMessage}:{currentDistance:F2}");
            ros.Publish(hriStateTopic, msg);
        }
    }

    void OnHumanApproach()
    {
        Debug.Log("[HRIManager] Human entered safety zone - reducing speed");

        if (slowOnProximity)
        {
            // Reduce robot speed
            Time.timeScale = warningSpeedFactor;
        }

        // Additional safety behaviors can be added here:
        // - Activate warning lights
        // - Play warning sound
        // - Reduce joint torques
    }

    void OnHumanCritical()
    {
        Debug.LogWarning("[HRIManager] CRITICAL: Human too close!");

        if (stopOnCritical)
        {
            // Stop robot motion
            Time.timeScale = 0f;
            Debug.LogWarning("[HRIManager] Robot stopped for safety");
        }

        // Emergency behaviors:
        // - Lock all joints
        // - Trigger emergency stop
        // - Sound alarm
    }

    void OnHumanDepart()
    {
        Debug.Log("[HRIManager] Human left safety zone - resuming normal operation");

        // Restore normal speed
        Time.timeScale = 1f;

        // Reset safety behaviors
    }

    /// <summary>
    /// Get the current HRI state as a string.
    /// </summary>
    public string GetStateString()
    {
        return currentState.ToString();
    }

    /// <summary>
    /// Get the current distance between human and robot.
    /// </summary>
    public float GetDistance()
    {
        return currentDistance;
    }

    /// <summary>
    /// Check if the human is within the safety zone.
    /// </summary>
    public bool IsHumanInSafetyZone()
    {
        return currentState != ProximityState.Safe;
    }

    /// <summary>
    /// Manually trigger emergency stop.
    /// </summary>
    public void EmergencyStop()
    {
        Time.timeScale = 0f;
        Debug.LogWarning("[HRIManager] Emergency stop triggered!");

        if (publishToROS && ros != null)
        {
            ros.Publish(hriStateTopic, new StringMsg("EMERGENCY_STOP"));
        }
    }

    /// <summary>
    /// Resume operation after emergency stop.
    /// </summary>
    public void Resume()
    {
        if (currentState != ProximityState.Critical)
        {
            Time.timeScale = currentState == ProximityState.Warning
                ? warningSpeedFactor
                : 1f;
            Debug.Log("[HRIManager] Resumed operation");

            if (publishToROS && ros != null)
            {
                ros.Publish(hriStateTopic, new StringMsg("RESUMED"));
            }
        }
        else
        {
            Debug.LogWarning("[HRIManager] Cannot resume - human still in critical zone");
        }
    }

    void OnDrawGizmos()
    {
        if (robot == null) return;

        // Draw safety zones
        Color zoneColor;
        switch (currentState)
        {
            case ProximityState.Critical:
                zoneColor = criticalColor;
                break;
            case ProximityState.Warning:
                zoneColor = warningColor;
                break;
            default:
                zoneColor = safeColor;
                break;
        }

        // Outer safety zone
        Gizmos.color = new Color(zoneColor.r, zoneColor.g, zoneColor.b, 0.3f);
        Gizmos.DrawSphere(robot.position, safetyDistance);

        // Wire outline
        Gizmos.color = zoneColor;
        Gizmos.DrawWireSphere(robot.position, safetyDistance);

        // Critical zone
        Gizmos.color = new Color(criticalColor.r, criticalColor.g, criticalColor.b, 0.2f);
        Gizmos.DrawSphere(robot.position, criticalDistance);
        Gizmos.color = criticalColor;
        Gizmos.DrawWireSphere(robot.position, criticalDistance);

        // Line to human
        if (human != null)
        {
            Gizmos.color = zoneColor;
            Gizmos.DrawLine(robot.position, human.position);
        }
    }
}
