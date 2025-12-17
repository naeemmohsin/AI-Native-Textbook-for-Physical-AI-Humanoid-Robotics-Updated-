using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System.Collections.Generic;

/// <summary>
/// Controls a URDF-imported robot via ROS 2 messages.
///
/// This script:
/// - Subscribes to joint command messages from ROS 2
/// - Applies joint positions to ArticulationBody components
/// - Publishes current joint states back to ROS 2
///
/// Usage:
///   1. Import your URDF robot using the URDF Importer
///   2. Attach this script to the root robot GameObject
///   3. Ensure ROS connection is established
///
/// Requirements:
///   - Robot imported via URDF Importer
///   - ArticulationBody components on joints
///   - ROSConnectionManager in scene
/// </summary>
public class RobotController : MonoBehaviour
{
    [Header("ROS Topics")]
    [Tooltip("Topic to receive joint commands")]
    public string commandTopic = "/joint_commands";

    [Tooltip("Topic to publish joint states")]
    public string stateTopic = "/unity/joint_states";

    [Header("Control Settings")]
    [Tooltip("How quickly joints move to target (degrees/second)")]
    public float jointSpeed = 90f;

    [Tooltip("Publish joint states at this rate (Hz)")]
    public float publishRate = 30f;

    // ROS connection
    private ROSConnection ros;

    // Robot articulation bodies (joints)
    private ArticulationBody[] joints;
    private string[] jointNames;

    // State publishing timer
    private float publishTimer = 0f;
    private float publishInterval;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Find all articulation bodies in the robot hierarchy
        InitializeJoints();

        // Register publisher for joint states
        ros.RegisterPublisher<JointStateMsg>(stateTopic);

        // Subscribe to joint commands
        ros.Subscribe<JointStateMsg>(commandTopic, OnJointCommand);

        publishInterval = 1f / publishRate;

        Debug.Log($"[RobotController] Initialized with {joints.Length} joints");
    }

    void InitializeJoints()
    {
        // Get all ArticulationBody components except the root
        var allBodies = GetComponentsInChildren<ArticulationBody>();
        var jointList = new List<ArticulationBody>();
        var nameList = new List<string>();

        foreach (var body in allBodies)
        {
            // Skip root body (has no parent joint)
            if (body.isRoot) continue;

            // Only include revolute and prismatic joints
            if (body.jointType == ArticulationJointType.RevoluteJoint ||
                body.jointType == ArticulationJointType.PrismaticJoint)
            {
                jointList.Add(body);
                nameList.Add(body.name);
            }
        }

        joints = jointList.ToArray();
        jointNames = nameList.ToArray();
    }

    void OnJointCommand(JointStateMsg msg)
    {
        // Apply commanded positions to joints
        for (int i = 0; i < msg.position.Length && i < joints.Length; i++)
        {
            SetJointTarget(i, (float)msg.position[i]);
        }
    }

    void SetJointTarget(int jointIndex, float targetRadians)
    {
        if (jointIndex < 0 || jointIndex >= joints.Length) return;

        var joint = joints[jointIndex];
        var drive = joint.xDrive;

        // Convert radians to degrees for Unity
        drive.target = targetRadians * Mathf.Rad2Deg;
        joint.xDrive = drive;
    }

    void Update()
    {
        // Publish joint states periodically
        publishTimer += Time.deltaTime;
        if (publishTimer >= publishInterval)
        {
            PublishJointStates();
            publishTimer = 0f;
        }
    }

    void PublishJointStates()
    {
        var msg = new JointStateMsg();

        // Header with timestamp
        msg.header = new RosMessageTypes.Std.HeaderMsg();
        msg.header.stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
        {
            sec = (int)Time.time,
            nanosec = (uint)((Time.time % 1) * 1e9)
        };
        msg.header.frame_id = "base_link";

        // Joint names
        msg.name = jointNames;

        // Get current positions, velocities, efforts
        double[] positions = new double[joints.Length];
        double[] velocities = new double[joints.Length];
        double[] efforts = new double[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            // Get joint position (convert from degrees to radians)
            positions[i] = joints[i].jointPosition[0] * Mathf.Deg2Rad;

            // Get joint velocity
            velocities[i] = joints[i].jointVelocity[0] * Mathf.Deg2Rad;

            // Effort (torque) - not directly available, set to 0
            efforts[i] = 0.0;
        }

        msg.position = positions;
        msg.velocity = velocities;
        msg.effort = efforts;

        ros.Publish(stateTopic, msg);
    }

    /// <summary>
    /// Get current joint positions in radians.
    /// </summary>
    public float[] GetJointPositions()
    {
        float[] positions = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            positions[i] = joints[i].jointPosition[0] * Mathf.Deg2Rad;
        }
        return positions;
    }

    /// <summary>
    /// Set all joints to their home position (0 degrees).
    /// </summary>
    public void HomeJoints()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            SetJointTarget(i, 0f);
        }
        Debug.Log("[RobotController] Joints homed");
    }

    void OnDrawGizmosSelected()
    {
        // Visualize joint axes in editor
        if (joints == null) return;

        foreach (var joint in joints)
        {
            if (joint == null) continue;
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(joint.transform.position, 0.02f);
        }
    }
}
