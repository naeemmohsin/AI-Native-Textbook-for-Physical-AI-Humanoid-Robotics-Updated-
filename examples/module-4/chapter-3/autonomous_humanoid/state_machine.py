#!/usr/bin/env python3
"""
YASMIN State Machine for Autonomous Humanoid

This module implements a hierarchical state machine that orchestrates
the complete autonomous humanoid behavior: listening for commands,
planning actions, executing tasks, and handling errors.

Dependencies:
    - yasmin
    - yasmin_ros
    - yasmin_viewer (optional, for visualization)

Usage:
    ros2 run autonomous_humanoid state_machine_node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from yasmin import State, StateMachine, Blackboard
from yasmin_ros import ActionState, MonitorState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from typing import Optional
import time


class IdleState(State):
    """
    Initial state where robot waits for voice activation.

    Outcomes:
        - command_received: Voice command detected
        - shutdown: Node shutdown requested
    """

    def __init__(self, node: Node):
        super().__init__(outcomes=["command_received", "shutdown"])
        self.node = node
        self.command_received = False

        # Subscribe to voice intent topic
        self.intent_sub = node.create_subscription(
            String,
            '/voice/intent',
            self.intent_callback,
            10
        )

    def intent_callback(self, msg: String):
        """Handle incoming voice intent."""
        self.command_received = True
        self.last_intent = msg.data

    def execute(self, blackboard: Blackboard) -> str:
        """Wait for voice command."""
        self.node.get_logger().info("State: IDLE - Waiting for command...")
        self.command_received = False

        # Publish status
        status_pub = self.node.create_publisher(String, '/state_machine/status', 10)
        status_pub.publish(String(data="idle"))

        while rclpy.ok():
            if self.command_received:
                blackboard["raw_intent"] = self.last_intent
                self.node.get_logger().info(f"Command received: {self.last_intent}")
                return "command_received"

            rclpy.spin_once(self.node, timeout_sec=0.1)

        return "shutdown"


class ListeningState(State):
    """
    Process voice input and extract structured intent.

    Outcomes:
        - intent_parsed: Successfully extracted intent
        - parse_failed: Could not parse voice input
    """

    TIMEOUT_SEC = 10.0

    def __init__(self, node: Node):
        super().__init__(outcomes=["intent_parsed", "parse_failed"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        """Parse voice command into structured intent."""
        self.node.get_logger().info("State: LISTENING - Processing command...")

        raw_intent = blackboard.get("raw_intent", "")
        if not raw_intent:
            blackboard["error"] = "No intent received"
            return "parse_failed"

        # Parse the intent (simplified parsing)
        try:
            intent = self.parse_intent(raw_intent)
            blackboard["intent"] = intent
            self.node.get_logger().info(f"Parsed intent: {intent}")
            return "intent_parsed"

        except Exception as e:
            blackboard["error"] = f"Parse error: {str(e)}"
            self.node.get_logger().error(f"Failed to parse: {e}")
            return "parse_failed"

    def parse_intent(self, raw: str) -> dict:
        """
        Parse raw intent string into structured format.

        Expected format: "action:fetch|object:bottle|location:kitchen"
        """
        intent = {
            "action": None,
            "object": None,
            "location": None,
            "raw": raw
        }

        # Parse key:value pairs
        for part in raw.split("|"):
            if ":" in part:
                key, value = part.split(":", 1)
                intent[key.strip()] = value.strip()

        if not intent["action"]:
            raise ValueError("No action specified in intent")

        return intent


class PlanningState(State):
    """
    Generate action plan using LLM planner service.

    Outcomes:
        - plan_ready: Successfully generated plan
        - plan_failed: Could not generate valid plan
    """

    TIMEOUT_SEC = 30.0

    def __init__(self, node: Node):
        super().__init__(outcomes=["plan_ready", "plan_failed"])
        self.node = node

        # Service client for LLM planner
        self.planner_client = node.create_client(
            # Assuming a custom PlanTask service
            # PlanTask,
            String,  # Simplified for example
            '/planner/plan_task'
        )

    def execute(self, blackboard: Blackboard) -> str:
        """Generate action plan from intent."""
        self.node.get_logger().info("State: PLANNING - Generating action plan...")

        intent = blackboard.get("intent", {})
        if not intent:
            blackboard["error"] = "No intent available for planning"
            return "plan_failed"

        # Generate plan based on action type
        try:
            plan = self.generate_plan(intent)
            blackboard["action_plan"] = plan
            self.node.get_logger().info(f"Generated plan with {len(plan)} actions")
            return "plan_ready"

        except Exception as e:
            blackboard["error"] = f"Planning error: {str(e)}"
            self.node.get_logger().error(f"Planning failed: {e}")
            return "plan_failed"

    def generate_plan(self, intent: dict) -> list:
        """
        Generate action sequence from intent.

        This is a simplified planner. In production, this would
        call the LLM planner service.
        """
        action = intent.get("action")
        obj = intent.get("object")
        location = intent.get("location")

        if action == "fetch":
            return [
                {"type": "navigate", "target": location},
                {"type": "detect", "object": obj},
                {"type": "pick", "object": obj},
                {"type": "navigate", "target": "user"},
                {"type": "present", "object": obj},
                {"type": "speak", "message": f"Here is the {obj}"}
            ]

        elif action == "navigate":
            return [
                {"type": "navigate", "target": location},
                {"type": "speak", "message": f"I have arrived at {location}"}
            ]

        elif action == "find":
            return [
                {"type": "navigate", "target": location},
                {"type": "detect", "object": obj},
                {"type": "speak", "message": f"I found the {obj}"}
            ]

        else:
            raise ValueError(f"Unknown action type: {action}")


class ExecutingState(State):
    """
    Execute action plan through behavior tree.

    Outcomes:
        - task_complete: All actions executed successfully
        - task_failed: An action failed
        - needs_replan: Execution requires replanning
    """

    def __init__(self, node: Node):
        super().__init__(outcomes=["task_complete", "task_failed", "needs_replan"])
        self.node = node

        # Action client for navigation
        self.nav_client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Publishers for feedback
        self.progress_pub = node.create_publisher(
            String,
            '/executor/progress',
            10
        )

    def execute(self, blackboard: Blackboard) -> str:
        """Execute the action plan."""
        self.node.get_logger().info("State: EXECUTING - Running action plan...")

        plan = blackboard.get("action_plan", [])
        if not plan:
            blackboard["error"] = "No action plan to execute"
            return "task_failed"

        # Execute each action in sequence
        for i, action in enumerate(plan):
            self.progress_pub.publish(
                String(data=f"Executing action {i+1}/{len(plan)}: {action['type']}")
            )

            result = self.execute_action(action, blackboard)

            if result == "failed":
                blackboard["failed_action"] = action
                return "task_failed"

            elif result == "replan":
                return "needs_replan"

            self.node.get_logger().info(f"Action {i+1} complete: {action['type']}")

        return "task_complete"

    def execute_action(self, action: dict, blackboard: Blackboard) -> str:
        """
        Execute a single action.

        Returns: "success", "failed", or "replan"
        """
        action_type = action.get("type")

        if action_type == "navigate":
            return self.execute_navigate(action)

        elif action_type == "detect":
            return self.execute_detect(action, blackboard)

        elif action_type == "pick":
            return self.execute_pick(action)

        elif action_type == "present":
            return self.execute_present(action)

        elif action_type == "speak":
            return self.execute_speak(action)

        else:
            self.node.get_logger().warn(f"Unknown action type: {action_type}")
            return "success"  # Skip unknown actions

    def execute_navigate(self, action: dict) -> str:
        """Execute navigation action."""
        target = action.get("target")
        self.node.get_logger().info(f"Navigating to: {target}")

        # In production, this would send goal to Nav2
        # Simulated success for example
        time.sleep(2.0)
        return "success"

    def execute_detect(self, action: dict, blackboard: Blackboard) -> str:
        """Execute object detection."""
        obj = action.get("object")
        self.node.get_logger().info(f"Detecting: {obj}")

        # In production, this would query perception system
        # Simulated detection
        time.sleep(1.0)
        blackboard["detected_object"] = {
            "class": obj,
            "position": {"x": 1.0, "y": 0.5, "z": 0.8}
        }
        return "success"

    def execute_pick(self, action: dict) -> str:
        """Execute pick action."""
        obj = action.get("object")
        self.node.get_logger().info(f"Picking up: {obj}")

        # In production, this would call manipulation action server
        time.sleep(2.0)
        return "success"

    def execute_present(self, action: dict) -> str:
        """Execute present action."""
        obj = action.get("object")
        self.node.get_logger().info(f"Presenting: {obj}")
        time.sleep(1.0)
        return "success"

    def execute_speak(self, action: dict) -> str:
        """Execute speech action."""
        message = action.get("message")
        self.node.get_logger().info(f"Speaking: {message}")

        # Publish to TTS topic
        # speech_pub.publish(String(data=message))
        return "success"


class ErrorState(State):
    """
    Handle errors and attempt recovery.

    Outcomes:
        - recovered: Successfully recovered from error
        - fatal: Unrecoverable error, return to idle
    """

    MAX_RETRIES = 3

    def __init__(self, node: Node):
        super().__init__(outcomes=["recovered", "fatal"])
        self.node = node
        self.retry_count = 0

    def execute(self, blackboard: Blackboard) -> str:
        """Attempt to recover from error."""
        error = blackboard.get("error", "Unknown error")
        self.node.get_logger().warn(f"State: ERROR - {error}")

        # Increment retry counter
        self.retry_count += 1

        if self.retry_count > self.MAX_RETRIES:
            self.node.get_logger().error("Max retries exceeded, returning to idle")
            self.retry_count = 0
            return "fatal"

        # Attempt recovery based on error type
        if "navigation" in error.lower():
            return self.recover_navigation(blackboard)

        elif "detection" in error.lower():
            return self.recover_detection(blackboard)

        elif "planning" in error.lower():
            return self.recover_planning(blackboard)

        else:
            self.node.get_logger().error(f"No recovery strategy for: {error}")
            return "fatal"

    def recover_navigation(self, blackboard: Blackboard) -> str:
        """Attempt navigation recovery."""
        self.node.get_logger().info("Attempting navigation recovery...")
        # Clear costmap, retry navigation
        return "recovered"

    def recover_detection(self, blackboard: Blackboard) -> str:
        """Attempt detection recovery."""
        self.node.get_logger().info("Attempting detection recovery...")
        # Move robot slightly, retry detection
        return "recovered"

    def recover_planning(self, blackboard: Blackboard) -> str:
        """Attempt planning recovery."""
        self.node.get_logger().info("Attempting planning recovery...")
        # Simplify task, retry planning
        return "recovered"


def create_state_machine(node: Node) -> StateMachine:
    """
    Create the complete autonomous humanoid state machine.

    State Machine Structure:
        IDLE → LISTENING → PLANNING → EXECUTING → IDLE
                  ↓           ↓           ↓
                ERROR ←───────┴───────────┘
    """
    sm = StateMachine(outcomes=["shutdown"])

    # Add states
    sm.add_state(
        "IDLE",
        IdleState(node),
        transitions={
            "command_received": "LISTENING",
            "shutdown": "shutdown"
        }
    )

    sm.add_state(
        "LISTENING",
        ListeningState(node),
        transitions={
            "intent_parsed": "PLANNING",
            "parse_failed": "ERROR"
        }
    )

    sm.add_state(
        "PLANNING",
        PlanningState(node),
        transitions={
            "plan_ready": "EXECUTING",
            "plan_failed": "ERROR"
        }
    )

    sm.add_state(
        "EXECUTING",
        ExecutingState(node),
        transitions={
            "task_complete": "IDLE",
            "task_failed": "ERROR",
            "needs_replan": "PLANNING"
        }
    )

    sm.add_state(
        "ERROR",
        ErrorState(node),
        transitions={
            "recovered": "EXECUTING",
            "fatal": "IDLE"
        }
    )

    return sm


class StateMachineNode(Node):
    """ROS 2 node that runs the state machine."""

    def __init__(self):
        super().__init__('state_machine_node')

        # Create state machine
        self.sm = create_state_machine(self)

        # Create blackboard
        self.blackboard = Blackboard()

        # Optional: YASMIN viewer for visualization
        self.viewer = YasminViewerPub(self, "autonomous_humanoid", self.sm)

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/state_machine/current_state',
            10
        )

        self.get_logger().info("State machine node initialized")

    def run(self):
        """Run the state machine."""
        self.get_logger().info("Starting state machine...")

        try:
            outcome = self.sm(self.blackboard)
            self.get_logger().info(f"State machine finished with: {outcome}")

        except Exception as e:
            self.get_logger().error(f"State machine error: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = StateMachineNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
