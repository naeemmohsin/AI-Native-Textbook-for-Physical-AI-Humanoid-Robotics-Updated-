#!/usr/bin/env python3
"""
LLM-Based Task Planner Node

Converts natural language commands into executable action plans
using OpenAI's GPT models with robot capability grounding.

Topics:
    Subscribed: /voice/intent (std_msgs/String) - JSON intent
    Published: /planner/action_plan (std_msgs/String) - JSON plan
    Published: /planner/status (std_msgs/String) - Status updates

Parameters:
    capabilities_file: Path to robot capabilities YAML
    model: LLM model to use (gpt-4, gpt-3.5-turbo, llama3.1:8b)
    temperature: LLM temperature (0.0-1.0)
    max_tokens: Maximum response tokens
    use_local_llm: Whether to use local Ollama server
    local_llm_url: Ollama server URL

Environment:
    OPENAI_API_KEY: Required for OpenAI models

Usage:
    ros2 run llm_planner planner_node --ros-args \
        -p capabilities_file:=capabilities.yaml \
        -p model:=gpt-4 \
        -p temperature:=0.2
"""

import os
import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# YAML support
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

# OpenAI support
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


@dataclass
class Capability:
    """Robot capability definition."""
    name: str
    description: str
    action_server: Optional[str] = None
    parameters: List[Dict] = field(default_factory=list)
    preconditions: List[str] = field(default_factory=list)
    effects: List[str] = field(default_factory=list)
    timeout: float = 30.0


@dataclass
class ActionStep:
    """Single step in an action plan."""
    action: str
    parameters: Dict[str, Any]
    description: str

    def to_dict(self) -> Dict:
        return {
            'action': self.action,
            'parameters': self.parameters,
            'description': self.description
        }


@dataclass
class ActionPlan:
    """Complete action plan."""
    goal: str
    feasible: bool = True
    reason: Optional[str] = None
    steps: List[ActionStep] = field(default_factory=list)
    estimated_duration: float = 0.0
    confidence: float = 0.0

    def to_dict(self) -> Dict:
        return {
            'goal': self.goal,
            'feasible': self.feasible,
            'reason': self.reason,
            'steps': [s.to_dict() for s in self.steps],
            'estimated_duration': self.estimated_duration,
            'confidence': self.confidence
        }


class LLMPlannerNode(Node):
    """ROS 2 node for LLM-based task planning."""

    def __init__(self):
        super().__init__('llm_planner')

        if not OPENAI_AVAILABLE:
            self.get_logger().error('OpenAI package not installed!')
            raise ImportError("Install with: pip install openai")

        # Declare parameters
        self.declare_parameter('capabilities_file', '')
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('temperature', 0.2)
        self.declare_parameter('max_tokens', 1500)
        self.declare_parameter('use_local_llm', False)
        self.declare_parameter('local_llm_url', 'http://localhost:11434')
        self.declare_parameter('prompt_template_file', '')

        # Get parameters
        capabilities_file = self.get_parameter('capabilities_file').value
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.use_local = self.get_parameter('use_local_llm').value
        self.local_url = self.get_parameter('local_llm_url').value
        prompt_file = self.get_parameter('prompt_template_file').value

        # Initialize OpenAI client
        self._init_client()

        # Load capabilities
        self.capabilities: Dict[str, Capability] = {}
        if capabilities_file:
            self.load_capabilities(capabilities_file)

        # Load or build prompt template
        if prompt_file and Path(prompt_file).exists():
            with open(prompt_file, 'r') as f:
                self.prompt_template = f.read()
        else:
            self.prompt_template = None

        # Build system prompt
        self.system_prompt = self.build_system_prompt()

        # Planning history for context
        self.planning_history: List[Dict] = []

        # Create ROS interfaces
        self.intent_sub = self.create_subscription(
            String,
            '/voice/intent',
            self.intent_callback,
            10
        )

        # Also accept raw text commands
        self.command_sub = self.create_subscription(
            String,
            '/planner/command',
            self.command_callback,
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
        self.get_logger().info(f'Model: {self.model}')
        self.get_logger().info(f'Capabilities loaded: {len(self.capabilities)}')

    def _init_client(self):
        """Initialize OpenAI client."""
        if self.use_local:
            self.client = OpenAI(
                base_url=f"{self.local_url}/v1",
                api_key="ollama"
            )
            self.get_logger().info(f'Using local LLM at {self.local_url}')
        else:
            api_key = os.environ.get('OPENAI_API_KEY')
            if not api_key:
                self.get_logger().error('OPENAI_API_KEY not set!')
                raise ValueError("Set OPENAI_API_KEY environment variable")
            self.client = OpenAI(api_key=api_key)
            self.get_logger().info(f'Using OpenAI API')

    def load_capabilities(self, filepath: str):
        """Load robot capabilities from YAML file."""
        if not YAML_AVAILABLE:
            self.get_logger().error('PyYAML not installed!')
            return

        path = Path(filepath)
        if not path.exists():
            self.get_logger().warning(f'Capabilities file not found: {filepath}')
            return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            for cap_data in data.get('capabilities', []):
                cap = Capability(
                    name=cap_data['name'],
                    description=cap_data['description'],
                    action_server=cap_data.get('action_server'),
                    parameters=cap_data.get('parameters', []),
                    preconditions=cap_data.get('preconditions', []),
                    effects=cap_data.get('effects', []),
                    timeout=cap_data.get('timeout', 30.0)
                )
                self.capabilities[cap.name] = cap

            self.get_logger().info(f'Loaded {len(self.capabilities)} capabilities')

        except Exception as e:
            self.get_logger().error(f'Failed to load capabilities: {e}')

    def build_system_prompt(self) -> str:
        """Build system prompt with capability information."""
        cap_descriptions = []

        for name, cap in self.capabilities.items():
            # Format parameters
            param_strs = []
            for p in cap.parameters:
                p_str = f"{p['name']}: {p['type']}"
                if p.get('required', True):
                    p_str += " (required)"
                else:
                    p_str += f" (default: {p.get('default', 'none')})"
                param_strs.append(p_str)

            cap_descriptions.append(f"""
- **{name}**
  Description: {cap.description}
  Parameters: {', '.join(param_strs) if param_strs else 'none'}
  Preconditions: {cap.preconditions if cap.preconditions else 'none'}
""")

        capabilities_text = '\n'.join(cap_descriptions)

        return f"""You are a robot task planner for a humanoid service robot. Your role is to convert natural language commands into executable action sequences.

## Robot Capabilities
{capabilities_text}

## Planning Rules
1. ONLY use actions from the capabilities list above
2. Check preconditions before using an action
3. Sequence actions logically
4. Include verbal feedback (speak) to keep users informed
5. Handle potential failures with appropriate strategies
6. If a task is impossible, explain why

## Output Format
Respond with valid JSON matching this schema:
{{
    "goal": "description of the goal",
    "feasible": true,
    "reason": null,
    "steps": [
        {{
            "action": "capability_name",
            "parameters": {{"param_name": "value"}},
            "description": "what this step accomplishes"
        }}
    ],
    "estimated_duration": 60.0,
    "confidence": 0.85
}}

If the task is impossible, set "feasible": false and explain in "reason".

## Safety
- Never plan actions that could harm humans
- Always include safety checks before manipulation
- Stop immediately if uncertain
"""

    def intent_callback(self, msg: String):
        """Handle incoming structured intent."""
        try:
            intent = json.loads(msg.data)
            self.process_intent(intent)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid intent JSON')

    def command_callback(self, msg: String):
        """Handle raw text commands."""
        intent = {
            'action': 'complex',
            'raw_text': msg.data
        }
        self.process_intent(intent)

    def process_intent(self, intent: dict):
        """Process intent and generate plan."""
        self.publish_status("Planning...")

        # Convert intent to prompt
        user_prompt = self.intent_to_prompt(intent)
        self.get_logger().info(f'Planning for: "{user_prompt}"')

        # Generate plan
        plan = self.generate_plan(user_prompt)

        if plan:
            # Validate and publish
            if self.validate_plan(plan):
                self.publish_plan(plan)
                self.publish_status(f"Plan ready: {len(plan.get('steps', []))} steps")
            else:
                self.publish_status("Plan validation failed")
        else:
            self.publish_status("Planning failed")

    def intent_to_prompt(self, intent: dict) -> str:
        """Convert structured intent to natural language prompt."""
        if intent.get('raw_text'):
            return intent['raw_text']

        action = intent.get('action', 'unknown')
        target = intent.get('target', '')
        params = intent.get('parameters', {})

        # Map common intents to prompts
        prompts = {
            'fetch': f"Fetch the {target} and bring it to me",
            'navigate': f"Navigate to the {target}",
            'pick': f"Pick up the {target}",
            'place': f"Place the held object at {target}",
            'patrol': f"Patrol the {target} area",
            'status': "Report your current status",
            'describe_scene': "Describe what you see around you",
        }

        return prompts.get(action, f"Perform {action} on {target}")

    def generate_plan(self, user_prompt: str) -> Optional[dict]:
        """Generate action plan using LLM."""
        try:
            messages = [
                {"role": "system", "content": self.system_prompt},
            ]

            # Add history for context (last 2 exchanges)
            for hist in self.planning_history[-4:]:
                messages.append(hist)

            messages.append({"role": "user", "content": user_prompt})

            # Call LLM
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                response_format={"type": "json_object"}
            )

            content = response.choices[0].message.content
            plan = json.loads(content)

            # Update history
            self.planning_history.append({"role": "user", "content": user_prompt})
            self.planning_history.append({"role": "assistant", "content": content})

            # Keep history bounded
            if len(self.planning_history) > 10:
                self.planning_history = self.planning_history[-10:]

            return plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse LLM response: {e}')
            return None
        except Exception as e:
            self.get_logger().error(f'LLM API error: {e}')
            return None

    def validate_plan(self, plan: dict) -> bool:
        """Validate that plan uses only available capabilities."""
        if not plan.get('feasible', True):
            # Infeasible plans are valid responses
            return True

        steps = plan.get('steps', [])

        for i, step in enumerate(steps):
            action = step.get('action')

            if action not in self.capabilities:
                self.get_logger().warning(
                    f'Step {i+1}: Unknown capability "{action}"'
                )
                return False

            # Validate required parameters
            cap = self.capabilities[action]
            step_params = step.get('parameters', {})

            for param_def in cap.parameters:
                if param_def.get('required', True):
                    if param_def['name'] not in step_params:
                        self.get_logger().warning(
                            f'Step {i+1}: Missing required param "{param_def["name"]}"'
                        )
                        # Don't fail, LLM might use alternative forms
                        pass

        return True

    def publish_plan(self, plan: dict):
        """Publish action plan."""
        msg = String()
        msg.data = json.dumps(plan)
        self.plan_pub.publish(msg)

        self.get_logger().info(
            f'Published plan: {plan.get("goal")} '
            f'({len(plan.get("steps", []))} steps, '
            f'confidence: {plan.get("confidence", 0):.2f})'
        )

    def publish_status(self, status: str):
        """Publish planner status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().debug(f'Status: {status}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LLMPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
