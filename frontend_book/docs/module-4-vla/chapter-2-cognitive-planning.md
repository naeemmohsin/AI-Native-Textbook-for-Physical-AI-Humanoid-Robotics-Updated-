---
sidebar_position: 2
title: "Chapter 2: Language-Driven Cognitive Planning"
description: "Implement LLM-based task planning with robot capability grounding and behavior tree execution"
---

# Chapter 2: Language-Driven Cognitive Planning

Transform high-level natural language commands into executable robot action sequences using large language models and behavior trees.

## Learning Objectives

By the end of this chapter, you will be able to:

1. Integrate LLM APIs (OpenAI) for natural language understanding
2. Define robot capabilities in a machine-readable format
3. Decompose high-level commands into action sequences
4. Generate ROS 2-compatible execution plans
5. Monitor execution and handle failures with replanning

## Prerequisites

- Completed Chapter 1 (Voice-to-Action)
- OpenAI API key (or local Ollama setup)
- Understanding of ROS 2 actions and services
- Python 3.10+ with `openai` package installed

:::info API Costs
OpenAI API usage incurs costs. For development:
- **GPT-4**: ~$0.03/1K input tokens, ~$0.06/1K output tokens
- **GPT-3.5-turbo**: ~$0.0005/1K tokens (10x cheaper)

Typical planning request: 500-1000 tokens = $0.03-0.10 per command.
:::

:::tip Local Alternative: Ollama
Run LLMs locally with [Ollama](https://ollama.ai) for free:
```bash
ollama pull llama3
ollama serve
```
Configure endpoint: `http://localhost:11434/api/generate`
:::

## 2.1 LLMs for Robot Task Planning

Large Language Models have emerged as powerful tools for robot task planning. Their ability to understand context, decompose problems, and generate structured outputs makes them ideal for translating human intentions into robot-executable plans.

### Why LLMs for Robotics?

Traditional robot programming requires explicit coding of every possible scenario. LLMs offer a different approach:

| Traditional Approach | LLM-Based Approach |
|---------------------|-------------------|
| Predefined command sets | Natural language understanding |
| Rigid state machines | Flexible task decomposition |
| Manual error handling | Context-aware replanning |
| Limited adaptability | Transfer learning from training |

**Example Transformation:**

```
Human: "Clean up the living room"

Traditional: ❌ Command not recognized

LLM Planner:
1. Navigate to living room
2. Scan for misplaced objects
3. For each object found:
   a. Identify object type
   b. Determine correct location
   c. Pick up object
   d. Navigate to destination
   e. Place object
4. Report completion
```

### Grounding Language in Physical Reality

The critical challenge is **grounding**—connecting abstract language concepts to the robot's physical capabilities and environment:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Grounding Pipeline                           │
├─────────────────────────────────────────────────────────────────┤
│  "Pick up the red cup"                                          │
│         │                                                       │
│         ▼                                                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐ │
│  │   Object    │───▶│  Location   │───▶│   Robot Action      │ │
│  │   Resolver  │    │   Finder    │    │   Capability        │ │
│  └─────────────┘    └─────────────┘    └─────────────────────┘ │
│         │                 │                      │              │
│         ▼                 ▼                      ▼              │
│    "cup_001"         (0.5, 1.2, 0.8)      "pick_object"        │
│    color: red        frame: base_link     max_weight: 2kg      │
└─────────────────────────────────────────────────────────────────┘
```

### Limitations and Failure Modes

LLMs have significant limitations that must be understood:

:::warning LLM Limitations
1. **Hallucination**: May suggest impossible actions or non-existent objects
2. **Physical Reasoning**: Poor understanding of physics and spatial relationships
3. **Safety Blindness**: Cannot inherently understand dangerous situations
4. **Latency**: API calls add seconds to response time
5. **Cost**: Per-token billing can accumulate quickly
:::

**Mitigation Strategies:**
- Validate all LLM outputs against capability registry
- Use simulation to verify plans before execution
- Implement hard safety constraints independent of LLM
- Cache common plans to reduce API calls
- Provide fallback behaviors for LLM failures

### Ethical Considerations

Deploying LLM-controlled robots raises important ethical questions:

1. **Accountability**: Who is responsible when an LLM-planned action causes harm?
2. **Transparency**: Users should know when AI is making decisions
3. **Privacy**: LLM APIs may log conversation data
4. **Bias**: Training data biases can affect robot behavior
5. **Dependency**: Over-reliance on cloud services creates fragility

**Best Practice**: Always implement human-in-the-loop confirmation for irreversible actions.

## 2.2 Robot Capability Registry

Before an LLM can plan robot actions, it must know what the robot can do. A capability registry provides this machine-readable knowledge.

### YAML Capability Schema

Define robot capabilities in a structured format:

```yaml title="examples/module-4/chapter-2/llm_planner/capabilities.yaml"
robot:
  name: "humanoid_assistant"
  version: "1.0"

capabilities:
  - name: navigate_to
    description: "Move the robot to a specified location"
    action_server: /navigate_to_pose
    parameters:
      - name: target_pose
        type: geometry_msgs/PoseStamped
        required: true
      - name: speed
        type: float
        default: 0.5
        range: [0.1, 1.0]
    preconditions:
      - "robot.localized == true"
      - "robot.battery > 10"
    effects:
      - "robot.position = target_pose"
    timeout: 120.0

  - name: pick_object
    description: "Pick up an object with the gripper"
    action_server: /pick_object
    parameters:
      - name: object_id
        type: string
        required: true
      - name: grasp_type
        type: string
        enum: ["top", "side", "pinch"]
        default: "side"
    preconditions:
      - "robot.gripper.state == 'open'"
      - "object.reachable == true"
      - "object.weight < robot.max_payload"
    effects:
      - "robot.gripper.holding = object_id"
      - "object.location = 'gripper'"
    timeout: 30.0

  - name: place_object
    description: "Place held object at target location"
    action_server: /place_object
    parameters:
      - name: target_location
        type: geometry_msgs/Point
        required: true
    preconditions:
      - "robot.gripper.holding != null"
    effects:
      - "robot.gripper.holding = null"
      - "robot.gripper.state = 'open'"
    timeout: 30.0

  - name: speak
    description: "Speak a message through text-to-speech"
    action_server: /speak
    parameters:
      - name: message
        type: string
        required: true
      - name: language
        type: string
        default: "en"
    preconditions: []
    effects: []
    timeout: 10.0

  - name: wait
    description: "Wait for specified duration"
    action_server: null  # Local implementation
    parameters:
      - name: duration
        type: float
        required: true
        range: [0.1, 60.0]
    preconditions: []
    effects: []
```

### Mapping to ROS 2 Action Servers

Each capability maps to a ROS 2 interface:

```python
class CapabilityLoader:
    """Load and validate robot capabilities."""

    def __init__(self, capabilities_file: str):
        with open(capabilities_file, 'r') as f:
            self.config = yaml.safe_load(f)

        self.capabilities = {}
        for cap in self.config['capabilities']:
            self.capabilities[cap['name']] = Capability(
                name=cap['name'],
                description=cap['description'],
                action_server=cap.get('action_server'),
                parameters=cap.get('parameters', []),
                preconditions=cap.get('preconditions', []),
                effects=cap.get('effects', []),
                timeout=cap.get('timeout', 30.0)
            )

    def get_capability(self, name: str) -> Optional[Capability]:
        """Get capability by name."""
        return self.capabilities.get(name)

    def get_all_descriptions(self) -> str:
        """Get all capability descriptions for LLM prompt."""
        descriptions = []
        for cap in self.capabilities.values():
            params = ", ".join([
                f"{p['name']}: {p['type']}"
                for p in cap.parameters
            ])
            descriptions.append(
                f"- {cap.name}({params}): {cap.description}"
            )
        return "\n".join(descriptions)
```

### Dynamic Capability Discovery

Capabilities can be discovered at runtime from available ROS 2 action servers:

```python
def discover_capabilities(self):
    """Discover available capabilities from ROS 2 action servers."""
    available = []

    for name, cap in self.capabilities.items():
        if cap.action_server:
            # Check if action server is available
            if self.action_client_exists(cap.action_server):
                available.append(name)
                self.get_logger().info(f'Capability available: {name}')
            else:
                self.get_logger().warning(
                    f'Action server not found for {name}: {cap.action_server}'
                )
        else:
            # Local capability, always available
            available.append(name)

    return available
```

## 2.3 LLM Integration with ROS 2

Now we'll create a ROS 2 node that uses an LLM for task planning.

### OpenAI API Setup

First, configure API access:

```bash
# Install the OpenAI package
pip install openai

# Set API key (use environment variable for security)
export OPENAI_API_KEY="your-api-key-here"
```

:::caution Security
Never hardcode API keys in source code. Use environment variables or secure secret management.
:::

### Creating an LLM Planner Node

```python title="examples/module-4/chapter-2/llm_planner/planner_node.py"
#!/usr/bin/env python3
"""
LLM-Based Task Planner Node

Converts natural language commands into executable action plans
using OpenAI's GPT models with robot capability grounding.
"""

import os
import json
import yaml
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI


@dataclass
class ActionStep:
    """Single step in an action plan."""
    action: str
    parameters: dict
    description: str


@dataclass
class ActionPlan:
    """Complete action plan from LLM."""
    goal: str
    steps: List[ActionStep]
    estimated_duration: float
    confidence: float


class LLMPlannerNode(Node):
    """ROS 2 node for LLM-based task planning."""

    def __init__(self):
        super().__init__('llm_planner')

        # Declare parameters
        self.declare_parameter('capabilities_file', '')
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('temperature', 0.2)
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('use_local_llm', False)
        self.declare_parameter('local_llm_url', 'http://localhost:11434')

        # Get parameters
        capabilities_file = self.get_parameter('capabilities_file').value
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.use_local = self.get_parameter('use_local_llm').value
        self.local_url = self.get_parameter('local_llm_url').value

        # Initialize OpenAI client
        if self.use_local:
            # Use Ollama or other local LLM
            self.client = OpenAI(
                base_url=f"{self.local_url}/v1",
                api_key="ollama"  # Ollama doesn't require a real key
            )
            self.get_logger().info(f'Using local LLM at {self.local_url}')
        else:
            api_key = os.environ.get('OPENAI_API_KEY')
            if not api_key:
                self.get_logger().error('OPENAI_API_KEY not set!')
                raise ValueError("OPENAI_API_KEY environment variable required")
            self.client = OpenAI(api_key=api_key)
            self.get_logger().info(f'Using OpenAI API with model: {self.model}')

        # Load capabilities
        self.capabilities = {}
        if capabilities_file:
            self.load_capabilities(capabilities_file)

        # Load prompt template
        self.system_prompt = self.build_system_prompt()

        # Create ROS interfaces
        self.intent_sub = self.create_subscription(
            String,
            '/voice/intent',
            self.intent_callback,
            10
        )

        self.plan_pub = self.create_publisher(
            String,
            '/planner/action_plan',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/planner/status',
            10
        )

        self.get_logger().info('LLM Planner node initialized')

    def load_capabilities(self, filepath: str):
        """Load robot capabilities from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            for cap in data.get('capabilities', []):
                self.capabilities[cap['name']] = cap

            self.get_logger().info(
                f'Loaded {len(self.capabilities)} capabilities'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load capabilities: {e}')

    def build_system_prompt(self) -> str:
        """Build system prompt with capability information."""
        cap_descriptions = []
        for name, cap in self.capabilities.items():
            params = []
            for p in cap.get('parameters', []):
                param_str = f"{p['name']}: {p['type']}"
                if not p.get('required', True):
                    param_str += f" (default: {p.get('default')})"
                params.append(param_str)

            cap_descriptions.append(
                f"- {name}({', '.join(params)})\n"
                f"  Description: {cap['description']}\n"
                f"  Preconditions: {cap.get('preconditions', [])}"
            )

        return f"""You are a robot task planner. Given a high-level goal, decompose it into a sequence of robot actions.

AVAILABLE ROBOT CAPABILITIES:
{chr(10).join(cap_descriptions)}

RULES:
1. Only use capabilities from the list above
2. Consider preconditions for each action
3. Handle potential failures with appropriate fallbacks
4. Keep plans simple and efficient
5. If a task is impossible, explain why

OUTPUT FORMAT (JSON):
{{
    "goal": "original goal description",
    "feasible": true/false,
    "reason": "explanation if not feasible",
    "steps": [
        {{
            "action": "capability_name",
            "parameters": {{"param1": "value1"}},
            "description": "what this step does"
        }}
    ],
    "estimated_duration": seconds,
    "confidence": 0.0-1.0
}}
"""

    def intent_callback(self, msg: String):
        """Handle incoming intent and generate plan."""
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            # Treat as raw text command
            intent = {"action": "complex", "raw_text": msg.data}

        # Generate plan from intent
        self.publish_status("Planning...")
        plan = self.generate_plan(intent)

        if plan:
            # Publish plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            self.publish_status(f"Plan generated: {len(plan.get('steps', []))} steps")
            self.get_logger().info(f'Generated plan with {len(plan.get("steps", []))} steps')
        else:
            self.publish_status("Planning failed")

    def generate_plan(self, intent: dict) -> Optional[dict]:
        """Generate action plan using LLM."""
        # Build user prompt from intent
        if intent.get('raw_text'):
            user_prompt = intent['raw_text']
        else:
            user_prompt = self.intent_to_prompt(intent)

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                response_format={"type": "json_object"}
            )

            # Parse response
            content = response.choices[0].message.content
            plan = json.loads(content)

            # Validate plan
            if self.validate_plan(plan):
                return plan
            else:
                self.get_logger().warning('Plan validation failed')
                return None

        except Exception as e:
            self.get_logger().error(f'LLM planning error: {e}')
            return None

    def intent_to_prompt(self, intent: dict) -> str:
        """Convert structured intent to natural language prompt."""
        action = intent.get('action', 'unknown')
        target = intent.get('target', '')
        params = intent.get('parameters', {})

        if action == 'fetch':
            return f"Fetch the {target} and bring it to me"
        elif action == 'navigate':
            return f"Navigate to {target}"
        elif action == 'pick':
            return f"Pick up the {target}"
        elif action == 'place':
            return f"Place the object at {target}"
        else:
            return intent.get('raw_text', f"Perform {action} on {target}")

    def validate_plan(self, plan: dict) -> bool:
        """Validate that plan only uses available capabilities."""
        if not plan.get('feasible', True):
            return True  # Infeasible plans are valid responses

        for step in plan.get('steps', []):
            action = step.get('action')
            if action not in self.capabilities:
                self.get_logger().warning(
                    f'Unknown capability in plan: {action}'
                )
                return False

        return True

    def publish_status(self, status: str):
        """Publish planner status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Prompt Engineering for Task Decomposition

Effective prompts are crucial for reliable planning:

```text title="examples/module-4/chapter-2/llm_planner/prompts/task_planner.txt"
You are a robot task planner for a humanoid service robot. Your job is to convert high-level human commands into executable action sequences.

## Context
- The robot operates in indoor environments (homes, offices)
- It can navigate, manipulate objects, and communicate
- Safety is the top priority

## Available Capabilities
{{CAPABILITIES}}

## Planning Guidelines

1. **Decomposition**: Break complex tasks into simple, atomic actions
2. **Sequencing**: Order actions logically, respecting preconditions
3. **Verification**: Add perception steps to verify action success
4. **Feedback**: Include verbal feedback to keep users informed
5. **Safety**: Never plan actions that could harm humans or property

## Example Plans

### Command: "Bring me a glass of water"
```json
{
  "goal": "Bring water to user",
  "steps": [
    {"action": "speak", "parameters": {"message": "I'll get you some water"}},
    {"action": "navigate_to", "parameters": {"target": "kitchen"}},
    {"action": "locate_object", "parameters": {"object_type": "glass"}},
    {"action": "pick_object", "parameters": {"object_id": "detected_glass"}},
    {"action": "navigate_to", "parameters": {"target": "water_dispenser"}},
    {"action": "fill_container", "parameters": {"source": "water_dispenser"}},
    {"action": "navigate_to", "parameters": {"target": "user"}},
    {"action": "place_object", "parameters": {"target": "user_hand"}},
    {"action": "speak", "parameters": {"message": "Here's your water"}}
  ],
  "estimated_duration": 120,
  "confidence": 0.85
}
```

### Command: "What's on the table?"
```json
{
  "goal": "Describe objects on table",
  "steps": [
    {"action": "navigate_to", "parameters": {"target": "table"}},
    {"action": "scan_surface", "parameters": {"target": "table"}},
    {"action": "speak", "parameters": {"message": "{{DETECTED_OBJECTS}}"}}
  ],
  "estimated_duration": 30,
  "confidence": 0.9
}
```

## Current Task
{{USER_COMMAND}}

Generate an action plan following the JSON format above.
```

### Local Alternatives: Ollama + Llama

For offline operation or cost savings, use a local LLM:

```bash
# Install Ollama
curl https://ollama.ai/install.sh | sh

# Pull a capable model
ollama pull llama3.1:8b

# Start server (runs on port 11434 by default)
ollama serve
```

Configure the planner node for local LLM:

```yaml
llm_planner:
  ros__parameters:
    use_local_llm: true
    local_llm_url: "http://localhost:11434"
    model: "llama3.1:8b"
    temperature: 0.1
```

## 2.4 Task Decomposition and Planning

Let's examine how the LLM breaks down complex commands.

### Breaking Down "Pick Up the Cup"

Even a simple command requires multiple steps:

```
User: "Pick up the cup"
          │
          ▼
┌─────────────────────────────────────┐
│         Task Decomposition          │
├─────────────────────────────────────┤
│ 1. Acknowledge command              │
│ 2. Locate cup in scene              │
│ 3. Verify cup is reachable          │
│ 4. Plan grasp approach              │
│ 5. Navigate to grasp position       │
│ 6. Execute pick action              │
│ 7. Verify successful grasp          │
│ 8. Report success/failure           │
└─────────────────────────────────────┘
```

### Preconditions and Effects

Each action has preconditions that must be satisfied:

```python
class PlanValidator:
    """Validate action plans against preconditions."""

    def validate_sequence(self, steps: List[ActionStep]) -> bool:
        """Check that all preconditions are met."""
        world_state = self.get_initial_state()

        for step in steps:
            capability = self.capabilities[step.action]

            # Check preconditions
            for precond in capability.preconditions:
                if not self.evaluate_condition(precond, world_state):
                    self.get_logger().error(
                        f'Precondition failed for {step.action}: {precond}'
                    )
                    return False

            # Apply effects
            for effect in capability.effects:
                self.apply_effect(effect, world_state)

        return True
```

### Handling Impossible Tasks

The LLM must recognize when tasks cannot be completed:

```json
{
  "goal": "Fly to the kitchen",
  "feasible": false,
  "reason": "The robot does not have flight capability. It can only navigate by walking. Would you like me to walk to the kitchen instead?",
  "steps": [],
  "alternative_plan": {
    "goal": "Navigate to kitchen by walking",
    "steps": [
      {"action": "navigate_to", "parameters": {"target": "kitchen"}}
    ]
  }
}
```

## 2.5 Action Execution with Behavior Trees

Behavior trees provide a robust framework for executing action sequences.

### BehaviorTree.CPP Overview

BehaviorTree.CPP is the standard behavior tree library for ROS 2:

```bash
sudo apt install ros-humble-behaviortree-cpp
```

Key concepts:

| Node Type | Description | Success Condition |
|-----------|-------------|-------------------|
| **Sequence** | Execute children in order | All children succeed |
| **Fallback** | Try children until one succeeds | One child succeeds |
| **Parallel** | Execute children simultaneously | Policy-dependent |
| **Action** | Execute robot action | Action completes |
| **Condition** | Check world state | Condition is true |

### Creating Action Nodes

Wrap ROS 2 actions as behavior tree nodes:

```cpp
// pick_action_node.cpp
class PickActionNode : public BT::RosActionNode<PickObject>
{
public:
    PickActionNode(const std::string& name, const BT::NodeConfig& config,
                   const BT::RosNodeParams& params)
        : BT::RosActionNode<PickObject>(name, config, params)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id"),
            BT::InputPort<std::string>("grasp_type")
        };
    }

    bool setGoal(PickObject::Goal& goal) override
    {
        goal.object_id = getInput<std::string>("object_id").value();
        goal.grasp_type = getInput<std::string>("grasp_type").value_or("side");
        return true;
    }
};
```

### Sample Behavior Tree

```xml title="examples/module-4/chapter-2/action_executor/behavior_tree.xml"
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="FetchObjectTree">

    <BehaviorTree ID="FetchObjectTree">
        <Sequence name="fetch_sequence">
            <!-- Announce intention -->
            <Speak message="I will fetch the {object_name} for you"/>

            <!-- Navigate to object location -->
            <Fallback name="navigation_with_retry">
                <NavigateToPose target="{object_location}"/>
                <Sequence name="retry_navigation">
                    <Speak message="Path blocked, finding alternative"/>
                    <ClearCostmap/>
                    <NavigateToPose target="{object_location}"/>
                </Sequence>
            </Fallback>

            <!-- Locate and pick object -->
            <Sequence name="pick_sequence">
                <LocateObject object_type="{object_type}"
                              output_id="{detected_id}"/>

                <Fallback name="pick_with_retry">
                    <PickObject object_id="{detected_id}" grasp_type="side"/>
                    <Sequence>
                        <Speak message="Grasp failed, retrying"/>
                        <PickObject object_id="{detected_id}" grasp_type="top"/>
                    </Sequence>
                </Fallback>
            </Sequence>

            <!-- Deliver to user -->
            <NavigateToPose target="{user_location}"/>

            <PlaceObject target="user_hand"/>

            <Speak message="Here is your {object_name}"/>
        </Sequence>
    </BehaviorTree>

    <!-- Subtree for error recovery -->
    <BehaviorTree ID="ErrorRecovery">
        <Fallback name="recovery_options">
            <Sequence name="minor_recovery">
                <Speak message="Encountered an issue, attempting recovery"/>
                <Wait duration="2.0"/>
                <RetryLastAction/>
            </Sequence>
            <Sequence name="major_recovery">
                <Speak message="Unable to complete task. Returning home."/>
                <NavigateToPose target="home_base"/>
            </Sequence>
        </Fallback>
    </BehaviorTree>

</root>
```

### Connecting to ROS 2 Action Servers

```python title="examples/module-4/chapter-2/action_executor/executor_node.py"
#!/usr/bin/env python3
"""
Action Executor Node

Executes action plans using behavior trees and ROS 2 action clients.
"""

import json
from typing import Dict, Any, Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


@dataclass
class ExecutionResult:
    """Result of plan execution."""
    success: bool
    steps_completed: int
    total_steps: int
    error_message: Optional[str] = None
    final_state: Optional[Dict] = None


class ActionExecutorNode(Node):
    """Execute action plans from LLM planner."""

    def __init__(self):
        super().__init__('action_executor')

        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('step_timeout', 60.0)

        self.max_retries = self.get_parameter('max_retries').value
        self.step_timeout = self.get_parameter('step_timeout').value

        # Action clients
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.callback_group
        )

        # Add more action clients as needed
        self.action_clients: Dict[str, ActionClient] = {
            'navigate_to': self.nav_client,
            # 'pick_object': self.pick_client,
            # 'place_object': self.place_client,
        }

        # Execution state
        self.current_plan = None
        self.is_executing = False
        self.should_abort = False

        # ROS interfaces
        self.plan_sub = self.create_subscription(
            String,
            '/planner/action_plan',
            self.plan_callback,
            10
        )

        self.abort_sub = self.create_subscription(
            String,
            '/executor/abort',
            self.abort_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/executor/status',
            10
        )

        self.result_pub = self.create_publisher(
            String,
            '/executor/result',
            10
        )

        self.get_logger().info('Action executor initialized')

    def plan_callback(self, msg: String):
        """Handle incoming action plan."""
        if self.is_executing:
            self.get_logger().warning('Already executing a plan')
            return

        try:
            plan = json.loads(msg.data)

            if not plan.get('feasible', True):
                self.get_logger().info(f"Plan not feasible: {plan.get('reason')}")
                self.publish_status(f"Cannot execute: {plan.get('reason')}")
                return

            self.current_plan = plan
            self.execute_plan(plan)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid plan JSON: {e}')

    def abort_callback(self, msg: String):
        """Handle abort request."""
        self.should_abort = True
        self.get_logger().info('Abort requested')
        self.publish_status('Aborting...')

    def execute_plan(self, plan: dict):
        """Execute action plan step by step."""
        self.is_executing = True
        self.should_abort = False

        steps = plan.get('steps', [])
        steps_completed = 0

        self.get_logger().info(f"Executing plan: {plan.get('goal')}")
        self.publish_status(f"Starting: {plan.get('goal')}")

        for i, step in enumerate(steps):
            if self.should_abort:
                self.get_logger().info('Plan aborted by user')
                break

            action = step.get('action')
            params = step.get('parameters', {})
            description = step.get('description', action)

            self.get_logger().info(f'Step {i+1}/{len(steps)}: {description}')
            self.publish_status(f'Step {i+1}: {description}')

            success = self.execute_step(action, params)

            if success:
                steps_completed += 1
            else:
                self.get_logger().error(f'Step {i+1} failed: {action}')
                # Try recovery or abort
                if not self.attempt_recovery(step):
                    break

        # Report result
        result = ExecutionResult(
            success=(steps_completed == len(steps)),
            steps_completed=steps_completed,
            total_steps=len(steps),
            error_message=None if steps_completed == len(steps) else 'Execution incomplete'
        )

        self.publish_result(result)
        self.is_executing = False
        self.current_plan = None

    def execute_step(self, action: str, params: dict) -> bool:
        """Execute a single action step."""
        if action == 'navigate_to':
            return self.execute_navigate(params)
        elif action == 'speak':
            return self.execute_speak(params)
        elif action == 'wait':
            return self.execute_wait(params)
        elif action == 'pick_object':
            return self.execute_pick(params)
        elif action == 'place_object':
            return self.execute_place(params)
        else:
            self.get_logger().warning(f'Unknown action: {action}')
            return False

    def execute_navigate(self, params: dict) -> bool:
        """Execute navigation action."""
        target = params.get('target_pose') or params.get('target')

        if isinstance(target, str):
            # Look up named location
            pose = self.lookup_location(target)
            if not pose:
                self.get_logger().error(f'Unknown location: {target}')
                return False
        else:
            pose = target

        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pose.get('x', 0.0)
        goal.pose.pose.position.y = pose.get('y', 0.0)
        goal.pose.pose.orientation.w = 1.0

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Send goal
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.step_timeout)

        result = result_future.result()
        return result.status == 4  # SUCCEEDED

    def execute_speak(self, params: dict) -> bool:
        """Execute speak action (placeholder)."""
        message = params.get('message', '')
        self.get_logger().info(f'Speaking: "{message}"')
        # In real implementation, publish to TTS node
        return True

    def execute_wait(self, params: dict) -> bool:
        """Execute wait action."""
        import time
        duration = params.get('duration', 1.0)
        time.sleep(duration)
        return True

    def execute_pick(self, params: dict) -> bool:
        """Execute pick action (placeholder)."""
        object_id = params.get('object_id')
        self.get_logger().info(f'Picking: {object_id}')
        # Implement actual pick action client
        return True

    def execute_place(self, params: dict) -> bool:
        """Execute place action (placeholder)."""
        target = params.get('target_location')
        self.get_logger().info(f'Placing at: {target}')
        # Implement actual place action client
        return True

    def lookup_location(self, name: str) -> Optional[dict]:
        """Look up named location coordinates."""
        # In real implementation, query location service
        locations = {
            'kitchen': {'x': 5.0, 'y': 2.0},
            'living_room': {'x': 0.0, 'y': 0.0},
            'bedroom': {'x': -3.0, 'y': 4.0},
            'home_base': {'x': 0.0, 'y': 0.0},
        }
        return locations.get(name)

    def attempt_recovery(self, failed_step: dict) -> bool:
        """Attempt to recover from failed step."""
        self.get_logger().info('Attempting recovery...')
        # Implement recovery strategies
        return False

    def publish_status(self, status: str):
        """Publish executor status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_result(self, result: ExecutionResult):
        """Publish execution result."""
        msg = String()
        msg.data = json.dumps({
            'success': result.success,
            'steps_completed': result.steps_completed,
            'total_steps': result.total_steps,
            'error': result.error_message
        })
        self.result_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 2.6 Execution Monitoring and Replanning

Robust execution requires monitoring progress and replanning when things go wrong.

### Tracking Action Progress

```python
class ExecutionMonitor:
    """Monitor execution progress and detect failures."""

    def __init__(self, node: Node):
        self.node = node
        self.current_step = 0
        self.step_start_time = None
        self.retry_count = 0

    def start_step(self, step_index: int):
        """Mark step as started."""
        self.current_step = step_index
        self.step_start_time = self.node.get_clock().now()
        self.retry_count = 0

    def check_timeout(self, timeout: float) -> bool:
        """Check if current step has exceeded timeout."""
        if self.step_start_time is None:
            return False

        elapsed = (self.node.get_clock().now() - self.step_start_time).nanoseconds / 1e9
        return elapsed > timeout

    def should_retry(self, max_retries: int = 3) -> bool:
        """Check if step should be retried."""
        self.retry_count += 1
        return self.retry_count <= max_retries
```

### Detecting Failures

Common failure modes and detection:

| Failure Type | Detection Method | Response |
|--------------|------------------|----------|
| Action timeout | Timer exceeded | Retry or abort |
| Navigation stuck | Position unchanged | Clear costmap, replan |
| Grasp failed | Force sensor / vision | Retry with different grasp |
| Object not found | Perception returns empty | Search wider area |
| Collision detected | Bumper / force sensors | Emergency stop |

### Triggering Replanning

When execution fails, request a new plan:

```python
def request_replan(self, original_plan: dict, failure_info: dict):
    """Request replanning after failure."""
    replan_request = {
        'original_goal': original_plan['goal'],
        'completed_steps': self.steps_completed,
        'failure_step': failure_info['step'],
        'failure_reason': failure_info['reason'],
        'current_state': self.get_current_state(),
        'constraint': 'avoid_previous_approach'
    }

    # Publish to planner
    msg = String()
    msg.data = json.dumps(replan_request)
    self.replan_pub.publish(msg)
```

### Natural Language Failure Explanations

Keep users informed about failures:

```python
def explain_failure(self, failure_info: dict) -> str:
    """Generate natural language explanation of failure."""
    step = failure_info['step']
    reason = failure_info['reason']

    explanations = {
        'timeout': f"The {step['action']} action took too long and timed out.",
        'collision': "I detected an obstacle and stopped for safety.",
        'object_not_found': f"I couldn't find the {step.get('parameters', {}).get('object_id', 'object')}.",
        'action_rejected': f"The {step['action']} action was rejected. The robot may not be ready.",
        'navigation_failed': "I couldn't find a path to the destination.",
    }

    return explanations.get(reason, f"Step {step['action']} failed: {reason}")
```

## 2.7 Hands-On Exercise: Pick and Place Planner

Build a complete planning and execution system for pick and place tasks.

### Step 1: Define Capabilities

Create your capability file:

```bash
cd examples/module-4/chapter-2/llm_planner
# Edit capabilities.yaml with your robot's capabilities
```

### Step 2: Launch the LLM Planner

```bash
# Set API key
export OPENAI_API_KEY="your-key"

# Launch planner
ros2 run llm_planner planner_node --ros-args \
    -p capabilities_file:=capabilities.yaml \
    -p model:=gpt-4 \
    -p temperature:=0.2
```

### Step 3: Launch the Executor

```bash
# In another terminal
ros2 run action_executor executor_node --ros-args \
    -p max_retries:=3 \
    -p step_timeout:=60.0
```

### Step 4: Test with Voice Commands

If you have Chapter 1 running:

```bash
# Speak: "Pick up the water bottle and bring it to me"
```

Or send test intents directly:

```bash
ros2 topic pub /voice/intent std_msgs/String "data: '{\"action\": \"fetch\", \"target\": \"water bottle\"}'" --once
```

### Step 5: Monitor Execution

```bash
# Watch planner status
ros2 topic echo /planner/status

# Watch execution progress
ros2 topic echo /executor/status

# View final result
ros2 topic echo /executor/result
```

### Step 6: Handle Failures

Test failure scenarios:

```bash
# Request impossible task
ros2 topic pub /voice/intent std_msgs/String "data: '{\"action\": \"fly\", \"target\": \"kitchen\"}'" --once

# Request object not in scene
ros2 topic pub /voice/intent std_msgs/String "data: '{\"action\": \"pick\", \"target\": \"unicorn\"}'" --once
```

### Success Criteria Checklist

- [ ] LLM generates valid plans for 8/10 test commands
- [ ] Plans only use defined capabilities
- [ ] Impossible tasks are correctly identified
- [ ] Executor completes simple navigation tasks
- [ ] Failures trigger appropriate error messages
- [ ] Planning response time is under 5 seconds

## Key Takeaways

1. **Capability Grounding**: LLMs need explicit knowledge of robot capabilities to plan effectively
2. **Prompt Engineering**: Well-designed prompts significantly improve plan quality
3. **Validation**: Always validate LLM outputs against known capabilities
4. **Behavior Trees**: Provide robust execution with built-in retry and fallback mechanisms
5. **Monitoring**: Track execution progress and detect failures early
6. **Replanning**: Systems must handle failures gracefully with replanning capabilities

## Next Steps

In [Chapter 3: Capstone – The Autonomous Humanoid](./chapter-3-capstone.md), we'll integrate voice control, cognitive planning, navigation, and perception into a complete autonomous system capable of executing complex real-world tasks.

## Additional Resources

- [OpenAI API Documentation](https://platform.openai.com/docs)
- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/)
- [Ollama](https://ollama.ai/) - Local LLM server
- [LangChain for Robotics](https://python.langchain.com/)
