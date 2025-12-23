#!/usr/bin/env python3
"""
Intent Parser for Robot Voice Commands

Extracts structured intents from natural language transcriptions.
Uses rule-based parsing for common commands with LLM fallback option.

Topics:
    Subscribed: /voice/transcript (std_msgs/String)
    Published: /voice/intent (std_msgs/String) - JSON format

Parameters:
    confidence_threshold: Minimum confidence to publish intent (0.0-1.0)
    commands_file: Path to custom commands YAML file
    enable_llm_fallback: Whether to use LLM for unrecognized commands

Intent JSON Format:
    {
        "action": "navigate",
        "target": "kitchen",
        "parameters": {"speed": "normal"},
        "confidence": 0.95,
        "raw_text": "go to the kitchen"
    }

Usage:
    ros2 run command_parser intent_parser --ros-args \
        -p confidence_threshold:=0.7 \
        -p commands_file:=commands.yaml
"""

import re
import json
from dataclasses import dataclass, asdict, field
from typing import Optional, Dict, List, Tuple, Any
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Optional YAML support
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False


@dataclass
class RobotIntent:
    """Structured robot command intent."""
    action: str
    target: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    raw_text: str = ""

    def to_json(self) -> str:
        """Convert intent to JSON string."""
        return json.dumps(asdict(self))


class IntentParser(Node):
    """Parse voice transcriptions into robot intents."""

    # Command patterns with named capture groups
    # Format: (regex_pattern, action_name)
    PATTERNS: List[Tuple[str, str]] = [
        # Movement commands
        (r"(?:please\s+)?(?:go|move|walk|navigate|head)\s+(?:to\s+)?(?:the\s+)?(?P<target>[\w\s]+?)(?:\s+please)?$",
         "navigate"),
        (r"(?:please\s+)?(?:come\s+)?(?:here|to\s+me)",
         "come_here"),
        (r"(?:please\s+)?(?:stop|halt|freeze)(?:\s+(?:moving|now))?",
         "stop"),
        (r"(?:please\s+)?(?:turn|rotate)\s+(?P<direction>left|right)"
         r"(?:\s+(?P<angle>\d+)\s*(?:degrees?|deg)?)?",
         "turn"),
        (r"(?:please\s+)?move\s+(?P<direction>forward|backward|back|left|right)"
         r"(?:\s+(?P<distance>[\d.]+)\s*(?:meters?|m|feet|ft)?)?",
         "move_direction"),

        # Manipulation commands
        (r"(?:please\s+)?(?:pick\s+up|grab|take|get)\s+(?:the\s+)?(?P<target>[\w\s]+)",
         "pick"),
        (r"(?:please\s+)?(?:put\s+down|place|drop|release)\s*(?:(?:the\s+)?(?P<target>[\w\s]+))?",
         "place"),
        (r"(?:please\s+)?(?:give|hand|pass)\s+(?:me\s+)?(?:the\s+)?(?P<target>[\w\s]+)",
         "give"),
        (r"(?:please\s+)?(?:bring|fetch)\s+(?:me\s+)?(?:the\s+)?(?P<target>[\w\s]+)",
         "fetch"),

        # Status and query commands
        (r"(?:what(?:'s|\s+is)\s+your\s+)?(?:status|state)",
         "status"),
        (r"(?:where\s+are\s+you|what(?:'s|\s+is)\s+your\s+(?:location|position))",
         "location"),
        (r"(?:what\s+(?:can\s+you\s+)?do|help|commands?)",
         "help"),
        (r"(?:what\s+do\s+you\s+see|describe\s+(?:the\s+)?(?:scene|view|surroundings?))",
         "describe_scene"),

        # System commands
        (r"(?:please\s+)?(?:go\s+)?home",
         "home"),
        (r"(?:please\s+)?(?:shut\s*down|power\s+off|turn\s+off)",
         "shutdown"),
        (r"(?:please\s+)?(?:restart|reboot)",
         "restart"),
        (r"(?:emergency\s+)?stop|e[\s-]?stop",
         "emergency_stop"),

        # Patrol and surveillance
        (r"(?:please\s+)?(?:patrol|survey|scan)\s+(?:the\s+)?(?P<target>[\w\s]+)",
         "patrol"),
        (r"(?:please\s+)?(?:watch|monitor|guard)\s+(?:the\s+)?(?P<target>[\w\s]+)",
         "monitor"),

        # Acknowledgment
        (r"(?:yes|yeah|yep|correct|affirmative|okay|ok)",
         "confirm"),
        (r"(?:no|nope|negative|cancel|abort)",
         "cancel"),
    ]

    def __init__(self):
        super().__init__('intent_parser')

        # Declare parameters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('commands_file', '')
        self.declare_parameter('enable_llm_fallback', False)
        self.declare_parameter('debug_mode', False)

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        commands_file = self.get_parameter('commands_file').value
        self.enable_llm_fallback = self.get_parameter('enable_llm_fallback').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Compile regex patterns for efficiency
        self.compiled_patterns: List[Tuple[re.Pattern, str]] = [
            (re.compile(pattern, re.IGNORECASE), action)
            for pattern, action in self.PATTERNS
        ]

        # Load custom commands
        self.custom_commands: Dict[str, Dict] = {}
        if commands_file:
            self.load_custom_commands(commands_file)

        # Object synonyms for better matching
        self.object_synonyms = {
            'cup': ['mug', 'glass', 'drink'],
            'bottle': ['water bottle', 'container'],
            'phone': ['mobile', 'cell phone', 'smartphone'],
            'remote': ['controller', 'tv remote'],
        }

        # Create subscriber and publisher
        self.transcript_sub = self.create_subscription(
            String,
            '/voice/transcript',
            self.transcript_callback,
            10
        )

        self.intent_pub = self.create_publisher(
            String,
            '/voice/intent',
            10
        )

        self.get_logger().info('Intent parser initialized')
        self.get_logger().info(f'Loaded {len(self.compiled_patterns)} command patterns')

    def load_custom_commands(self, filepath: str) -> None:
        """Load custom command definitions from YAML file."""
        if not YAML_AVAILABLE:
            self.get_logger().warning('PyYAML not installed, skipping custom commands')
            return

        path = Path(filepath)
        if not path.exists():
            self.get_logger().warning(f'Commands file not found: {filepath}')
            return

        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            if 'commands' in data:
                self.custom_commands = data['commands']
                self.get_logger().info(
                    f'Loaded {len(self.custom_commands)} custom commands'
                )

            if 'object_synonyms' in data:
                self.object_synonyms.update(data['object_synonyms'])

        except Exception as e:
            self.get_logger().error(f'Error loading commands file: {e}')

    def transcript_callback(self, msg: String) -> None:
        """Process incoming transcription."""
        text = msg.data.strip()

        if not text:
            return

        # Normalize text
        normalized = self.normalize_text(text)

        if self.debug_mode:
            self.get_logger().debug(f'Processing: "{normalized}"')

        # Parse intent
        intent = self.parse_intent(normalized)

        if intent and intent.confidence >= self.confidence_threshold:
            # Publish as JSON
            out_msg = String()
            out_msg.data = intent.to_json()
            self.intent_pub.publish(out_msg)

            self.get_logger().info(
                f'Intent: {intent.action} -> {intent.target} '
                f'(confidence: {intent.confidence:.2f})'
            )
        else:
            self.get_logger().warning(f'Could not parse: "{text}"')

            if self.enable_llm_fallback:
                self.get_logger().info('LLM fallback not implemented in this version')

    def normalize_text(self, text: str) -> str:
        """Normalize text for better pattern matching."""
        # Convert to lowercase
        text = text.lower()

        # Remove filler words
        fillers = ['um', 'uh', 'like', 'you know', 'basically']
        for filler in fillers:
            text = re.sub(rf'\b{filler}\b', '', text)

        # Normalize whitespace
        text = ' '.join(text.split())

        # Remove trailing punctuation
        text = text.rstrip('.,!?')

        return text

    def parse_intent(self, text: str) -> Optional[RobotIntent]:
        """Parse normalized text into a structured intent."""
        # Try rule-based patterns first (fast path)
        for pattern, action in self.compiled_patterns:
            match = pattern.search(text)
            if match:
                groups = match.groupdict()

                # Extract target and clean it
                target = groups.get('target')
                if target:
                    target = target.strip()
                    target = self.resolve_synonym(target)

                # Extract other parameters
                parameters = {
                    k: v.strip() if isinstance(v, str) else v
                    for k, v in groups.items()
                    if k != 'target' and v is not None
                }

                # Calculate confidence
                confidence = self.calculate_confidence(text, match)

                return RobotIntent(
                    action=action,
                    target=target,
                    parameters=parameters,
                    confidence=confidence,
                    raw_text=text
                )

        # Check custom commands
        for phrase, action_config in self.custom_commands.items():
            if phrase.lower() in text:
                return RobotIntent(
                    action=action_config.get('action', phrase),
                    target=action_config.get('target'),
                    parameters=action_config.get('parameters', {}),
                    confidence=0.90,
                    raw_text=text
                )

        # No match found
        return RobotIntent(
            action='unknown',
            target=None,
            parameters={},
            confidence=0.0,
            raw_text=text
        )

    def resolve_synonym(self, target: str) -> str:
        """Resolve object synonyms to canonical names."""
        target_lower = target.lower()

        for canonical, synonyms in self.object_synonyms.items():
            if target_lower in synonyms:
                return canonical

        return target

    def calculate_confidence(self, text: str, match: re.Match) -> float:
        """Calculate confidence score for a pattern match."""
        base_confidence = 0.95

        # Calculate match coverage
        match_text = match.group()
        coverage = len(match_text) / len(text) if text else 0

        # Adjust confidence based on coverage
        confidence = base_confidence * (0.5 + 0.5 * coverage)

        # Boost for exact matches
        if match_text.strip() == text.strip():
            confidence = min(1.0, confidence + 0.05)

        # Reduce for very short inputs (might be transcription errors)
        if len(text) < 3:
            confidence *= 0.5

        return round(confidence, 3)


def main(args=None):
    rclpy.init(args=args)
    node = IntentParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
