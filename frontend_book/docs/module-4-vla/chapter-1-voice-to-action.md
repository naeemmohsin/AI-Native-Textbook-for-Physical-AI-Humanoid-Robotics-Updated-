---
sidebar_position: 1
title: "Chapter 1: Voice-to-Action Interfaces"
description: "Build voice-controlled robot interfaces using OpenAI Whisper and ROS 2"
---

# Chapter 1: Voice-to-Action Interfaces

Transform spoken commands into robot actions using speech recognition, intent parsing, and ROS 2 integration.

## Learning Objectives

By the end of this chapter, you will be able to:

1. Configure audio capture and processing in ROS 2
2. Integrate OpenAI Whisper for speech-to-text transcription
3. Implement command parsing to extract robot intents
4. Map voice commands to ROS 2 topics and actions
5. Add wake-word detection for hands-free activation

## Prerequisites

- Completed Module 1 (ROS 2 nodes, topics, actions)
- Working ROS 2 Humble installation
- Python 3.10+ with pip
- Microphone connected to your system
- OpenAI API key (optional, for cloud Whisper)

:::info Hardware Requirements
For local Whisper inference, you'll need:
- **Minimum**: 4GB RAM (tiny model, CPU)
- **Recommended**: NVIDIA GPU with 4+ GB VRAM (base/small models)
- **Optimal**: NVIDIA RTX GPU with 8+ GB VRAM (medium/large models)

CPU-only inference is possible but significantly slower (~10x).
:::

:::tip Platform Notes
**Linux (Ubuntu 22.04)**: Best supported. Use `arecord` for audio testing.

**Windows**: Use WSL2 with USB audio passthrough or native Windows with PyAudio.

**macOS**: Use `sox` for audio testing. M1/M2 chips work well with MLX-optimized Whisper.
:::

## 1.1 Introduction to Voice-Controlled Robotics

Voice control represents the most natural human-robot interface. Instead of programming trajectories or clicking buttons, operators simply speak commands like "move forward" or "pick up the red cup." This paradigm shift enables non-technical users to interact with sophisticated robotic systems.

### Voice Interface Architecture

A voice-controlled robot requires a carefully designed pipeline that transforms acoustic signals into actionable commands:

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│   Mic    │───▶│ Wake-Word│───▶│   STT    │───▶│  Intent  │───▶│  Action  │
│  Input   │    │ Detector │    │ (Whisper)│    │  Parser  │    │ Executor │
└──────────┘    └──────────┘    └──────────┘    └──────────┘    └──────────┘
     │               │               │               │               │
     │               │               │               │               │
   Audio         Activation      Transcript       Intent         ROS 2
  Stream          Signal           Text          + Params        Action
```

Each stage has specific responsibilities:

- **Audio Input**: Continuous microphone capture with noise filtering
- **Wake-Word Detection**: Activation phrase ("Hey Robot") to trigger listening
- **Speech-to-Text**: Convert audio to text using Whisper
- **Intent Parser**: Extract action type and parameters from text
- **Action Executor**: Translate intent to ROS 2 commands

### Why Voice for Humanoid Robots?

Humanoid robots operating in human environments benefit from voice interfaces because:

1. **Natural Interaction**: Humans instinctively communicate through speech
2. **Hands-Free Operation**: Useful when carrying objects or during manipulation tasks
3. **Accessibility**: Enables control by users with limited mobility
4. **Contextual Commands**: Voice allows nuanced, context-dependent instructions

## 1.2 Audio Capture with ROS 2

Before we can transcribe speech, we need to capture audio from a microphone and publish it as ROS 2 messages.

### The audio_common Package

ROS 2 provides the `audio_common` stack for audio capture and playback. Install it with:

```bash
sudo apt install ros-humble-audio-common
```

The package provides several nodes:

| Node | Description |
|------|-------------|
| `audio_capture` | Captures audio from microphone |
| `audio_play` | Plays audio through speakers |
| `audio_to_spectrogram` | Converts audio to spectrograms |

### Microphone Configuration

First, identify your audio devices:

```bash
# List recording devices
arecord -l

# Test recording (5 seconds)
arecord -d 5 -f cd test.wav
aplay test.wav
```

Configure the audio capture node with a YAML file:

```yaml title="examples/module-4/chapter-1/audio_capture/audio_config.yaml"
audio_capture_node:
  ros__parameters:
    device: "default"           # ALSA device name
    format: "wave"              # Output format
    channels: 1                 # Mono audio
    sample_rate: 16000          # 16kHz for Whisper
    sample_format: "S16LE"      # 16-bit signed little-endian
    chunk_size: 4096            # Samples per message
    bitrate: 128                # Encoding bitrate
```

:::tip Whisper Optimization
Whisper performs best with 16kHz mono audio. Using higher sample rates doesn't improve accuracy but increases processing time.
:::

### Audio Message Types

The `audio_common` package defines custom message types:

```python
# audio_common_msgs/msg/AudioData
uint8[] data  # Raw audio bytes

# audio_common_msgs/msg/AudioInfo
uint32 channels
uint32 sample_rate
string sample_format
uint32 bitrate
string coding_format
```

### Launching Audio Capture

Create a launch file to start the audio pipeline:

```python title="examples/module-4/chapter-1/audio_capture/audio_capture.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='audio_capture',
            executable='audio_capture_node',
            name='audio_capture',
            parameters=['config/audio_config.yaml'],
            remappings=[
                ('audio', '/voice/audio_raw'),
            ]
        ),
    ])
```

Verify audio is being published:

```bash
# Check audio topic
ros2 topic hz /voice/audio_raw

# Should show ~31 Hz for 16kHz/512 sample chunks
```

## 1.3 Speech-to-Text with OpenAI Whisper

OpenAI's Whisper is a state-of-the-art speech recognition model that handles multiple languages, accents, and background noise remarkably well.

### Whisper Model Overview

Whisper comes in several sizes with different accuracy/speed tradeoffs:

| Model | Parameters | VRAM | Relative Speed | Use Case |
|-------|------------|------|----------------|----------|
| tiny | 39M | ~1 GB | ~32x | Testing, low-resource |
| base | 74M | ~1 GB | ~16x | Quick transcription |
| small | 244M | ~2 GB | ~6x | Good balance |
| medium | 769M | ~5 GB | ~2x | High accuracy |
| large-v3 | 1.55B | ~10 GB | 1x | Maximum accuracy |

For robotics applications, we recommend:
- **Development**: `base` or `small` for fast iteration
- **Production**: `medium` or `large-v3` for accuracy

### Local vs API Deployment

**Local Deployment** (Recommended for robotics):
- No internet latency
- No API costs
- Privacy-preserving
- Requires GPU for real-time

**API Deployment**:
- No local GPU needed
- Always latest model
- Per-minute billing
- Requires internet connection

Install the local Whisper library:

```bash
pip install openai-whisper
# Or with faster-whisper for optimized inference
pip install faster-whisper
```

### Creating a Whisper ROS 2 Node

Here's a complete ROS 2 node that subscribes to audio and publishes transcriptions:

```python title="examples/module-4/chapter-1/whisper_node/whisper_node.py"
#!/usr/bin/env python3
"""
Whisper Speech-to-Text ROS 2 Node

Subscribes to raw audio and publishes transcriptions.
Uses local Whisper model for low-latency inference.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
import whisper
from collections import deque
import threading


class WhisperNode(Node):
    """ROS 2 node for speech-to-text using OpenAI Whisper."""

    def __init__(self):
        super().__init__('whisper_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('buffer_seconds', 5.0)
        self.declare_parameter('silence_threshold', 0.01)
        self.declare_parameter('min_audio_length', 0.5)

        # Get parameters
        model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.buffer_seconds = self.get_parameter('buffer_seconds').value
        self.silence_threshold = self.get_parameter('silence_threshold').value
        self.min_audio_length = self.get_parameter('min_audio_length').value

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        self.model = whisper.load_model(model_size)
        self.get_logger().info('Whisper model loaded successfully')

        # Audio buffer
        buffer_samples = int(self.buffer_seconds * self.sample_rate)
        self.audio_buffer = deque(maxlen=buffer_samples)
        self.is_speaking = False
        self.speech_buffer = []

        # Thread lock for buffer access
        self.buffer_lock = threading.Lock()

        # Create subscriber and publisher
        self.audio_sub = self.create_subscription(
            AudioData,
            '/voice/audio_raw',
            self.audio_callback,
            10
        )

        self.transcript_pub = self.create_publisher(
            String,
            '/voice/transcript',
            10
        )

        # Processing timer (check for complete utterances)
        self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Whisper node initialized')

    def audio_callback(self, msg: AudioData):
        """Handle incoming audio data."""
        # Convert bytes to numpy array (assuming 16-bit audio)
        audio_data = np.frombuffer(msg.data, dtype=np.int16)
        audio_float = audio_data.astype(np.float32) / 32768.0

        with self.buffer_lock:
            # Check for speech activity
            rms = np.sqrt(np.mean(audio_float ** 2))

            if rms > self.silence_threshold:
                if not self.is_speaking:
                    self.is_speaking = True
                    self.get_logger().debug('Speech detected')
                self.speech_buffer.extend(audio_float)
            else:
                if self.is_speaking:
                    # End of speech detected
                    self.is_speaking = False
                    self.get_logger().debug('Speech ended')

    def process_audio(self):
        """Process accumulated speech and transcribe."""
        with self.buffer_lock:
            if not self.is_speaking and len(self.speech_buffer) > 0:
                audio_length = len(self.speech_buffer) / self.sample_rate

                if audio_length >= self.min_audio_length:
                    # Transcribe the speech buffer
                    audio_array = np.array(self.speech_buffer, dtype=np.float32)
                    self.speech_buffer = []

                    # Run transcription in separate thread to avoid blocking
                    threading.Thread(
                        target=self.transcribe,
                        args=(audio_array,)
                    ).start()
                else:
                    # Too short, discard
                    self.speech_buffer = []

    def transcribe(self, audio: np.ndarray):
        """Transcribe audio array using Whisper."""
        try:
            result = self.model.transcribe(
                audio,
                language=self.language,
                fp16=False,  # Set True if using GPU
                task='transcribe'
            )

            text = result['text'].strip()

            if text:
                self.get_logger().info(f'Transcription: "{text}"')

                msg = String()
                msg.data = text
                self.transcript_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

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

### Tuning for Accuracy and Latency

Optimize Whisper performance with these strategies:

**Reduce Latency:**
```python
# Use faster-whisper with CTranslate2 backend
from faster_whisper import WhisperModel

model = WhisperModel("base", device="cuda", compute_type="float16")
segments, info = model.transcribe(audio, beam_size=1)
```

**Improve Accuracy:**
```python
# Add initial prompt for domain context
result = model.transcribe(
    audio,
    initial_prompt="Robot commands: move, stop, pick, place, navigate",
    temperature=0.0,  # Deterministic output
    best_of=5,        # Multiple samples
)
```

:::warning GPU Memory
Large Whisper models require significant VRAM. Monitor usage with `nvidia-smi` and consider using `faster-whisper` for 4x memory reduction.
:::

## 1.4 Command Parsing and Intent Extraction

Raw transcriptions like "could you please move to the kitchen" must be converted into structured intents that the robot can execute.

### Intent Schema

Define a standard intent format:

```python
@dataclass
class RobotIntent:
    action: str           # move, stop, pick, place, navigate
    target: str | None    # Object or location
    parameters: dict      # Action-specific parameters
    confidence: float     # 0.0 to 1.0
    raw_text: str         # Original transcription
```

### Rule-Based Parsing

For common commands, rule-based parsing is fast and reliable:

```python title="examples/module-4/chapter-1/command_parser/intent_parser.py"
#!/usr/bin/env python3
"""
Intent Parser for Robot Voice Commands

Extracts structured intents from natural language transcriptions.
Uses rule-based parsing for common commands with LLM fallback.
"""

import re
import yaml
from dataclasses import dataclass, asdict
from typing import Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


@dataclass
class RobotIntent:
    """Structured robot command intent."""
    action: str
    target: Optional[str] = None
    parameters: dict = None
    confidence: float = 1.0
    raw_text: str = ""

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}


class IntentParser(Node):
    """Parse voice transcriptions into robot intents."""

    # Command patterns with named groups
    PATTERNS = [
        # Movement commands
        (r"(?:please\s+)?(?:go|move|walk|navigate)\s+(?:to\s+)?(?:the\s+)?(?P<target>\w+)",
         "navigate"),
        (r"(?:please\s+)?stop(?:\s+moving)?",
         "stop"),
        (r"(?:please\s+)?(?:turn|rotate)\s+(?P<direction>left|right)(?:\s+(?P<angle>\d+)\s*degrees?)?",
         "turn"),

        # Manipulation commands
        (r"(?:please\s+)?(?:pick\s+up|grab|take)\s+(?:the\s+)?(?P<target>\w+(?:\s+\w+)?)",
         "pick"),
        (r"(?:please\s+)?(?:put\s+down|place|drop)\s+(?:the\s+)?(?P<target>\w+)?",
         "place"),
        (r"(?:please\s+)?(?:give|hand)\s+(?:me\s+)?(?:the\s+)?(?P<target>\w+)",
         "give"),

        # Status commands
        (r"(?:what(?:'s|\s+is)\s+your\s+)?status",
         "status"),
        (r"(?:where\s+are\s+you|what(?:'s|\s+is)\s+your\s+location)",
         "location"),

        # System commands
        (r"(?:please\s+)?(?:go\s+)?home",
         "home"),
        (r"(?:please\s+)?(?:shut\s*down|power\s+off)",
         "shutdown"),
    ]

    def __init__(self):
        super().__init__('intent_parser')

        # Declare parameters
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('commands_file', '')

        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        commands_file = self.get_parameter('commands_file').value

        # Load custom commands if provided
        self.custom_commands = {}
        if commands_file:
            self.load_custom_commands(commands_file)

        # Compile regex patterns
        self.compiled_patterns = [
            (re.compile(pattern, re.IGNORECASE), action)
            for pattern, action in self.PATTERNS
        ]

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

    def load_custom_commands(self, filepath: str):
        """Load custom command definitions from YAML."""
        try:
            with open(filepath, 'r') as f:
                self.custom_commands = yaml.safe_load(f)
            self.get_logger().info(f'Loaded custom commands from {filepath}')
        except Exception as e:
            self.get_logger().warning(f'Could not load commands file: {e}')

    def transcript_callback(self, msg: String):
        """Process incoming transcription."""
        text = msg.data.strip().lower()

        if not text:
            return

        intent = self.parse_intent(text)

        if intent and intent.confidence >= self.confidence_threshold:
            # Publish as JSON
            intent_json = json.dumps(asdict(intent))

            out_msg = String()
            out_msg.data = intent_json
            self.intent_pub.publish(out_msg)

            self.get_logger().info(
                f'Intent: {intent.action} -> {intent.target} '
                f'(confidence: {intent.confidence:.2f})'
            )
        else:
            self.get_logger().warning(f'Could not parse: "{text}"')

    def parse_intent(self, text: str) -> Optional[RobotIntent]:
        """Parse text into a structured intent."""
        # Try rule-based patterns first
        for pattern, action in self.compiled_patterns:
            match = pattern.search(text)
            if match:
                groups = match.groupdict()

                # Extract target and parameters
                target = groups.get('target')
                parameters = {k: v for k, v in groups.items()
                             if k != 'target' and v is not None}

                return RobotIntent(
                    action=action,
                    target=target,
                    parameters=parameters,
                    confidence=0.95,
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

        # No match found - could trigger LLM fallback here
        return RobotIntent(
            action='unknown',
            target=None,
            parameters={},
            confidence=0.0,
            raw_text=text
        )


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
```

### Command Vocabulary

Define supported commands in a YAML file for easy customization:

```yaml title="examples/module-4/chapter-1/command_parser/commands.yaml"
# Custom command definitions for voice control
# Format: "trigger phrase": {action, target, parameters}

commands:
  # Navigation shortcuts
  "go to charging station":
    action: navigate
    target: charging_station
    parameters:
      priority: high

  "come here":
    action: navigate
    target: operator
    parameters:
      follow: true

  "patrol the area":
    action: patrol
    target: current_zone
    parameters:
      duration: 300

  # Manipulation shortcuts
  "fetch my coffee":
    action: fetch
    target: coffee_cup
    parameters:
      delivery_location: operator

  # Emergency commands
  "emergency stop":
    action: emergency_stop
    parameters:
      priority: critical

# Synonyms for common objects
object_synonyms:
  cup: [mug, glass, container]
  bottle: [water bottle, drink]
  box: [package, parcel, container]

# Location aliases
location_aliases:
  kitchen: [cooking area, food prep]
  living room: [lounge, sitting area]
  garage: [car port, workshop]
```

### Confidence Scoring

Implement confidence scoring to handle ambiguous commands:

```python
def calculate_confidence(self, text: str, pattern_match) -> float:
    """Calculate confidence based on match quality."""
    base_confidence = 0.95

    # Reduce for partial matches
    match_ratio = len(pattern_match.group()) / len(text)
    confidence = base_confidence * match_ratio

    # Boost for exact phrases
    if pattern_match.group() == text:
        confidence = min(1.0, confidence + 0.05)

    # Reduce for unusual word order
    if text.startswith(('um', 'uh', 'like')):
        confidence *= 0.9

    return confidence
```

### LLM Fallback for Complex Commands

When rule-based parsing fails, fall back to an LLM:

```python
async def llm_parse_intent(self, text: str) -> RobotIntent:
    """Use LLM to parse complex commands."""
    prompt = f"""Parse this robot command into a structured intent.

Command: "{text}"

Available actions: navigate, pick, place, stop, turn, status, home
Available targets: cup, bottle, box, kitchen, living_room, garage

Respond with JSON:
{{"action": "...", "target": "...", "parameters": {{}}}}
"""

    response = await self.openai_client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": prompt}],
        response_format={"type": "json_object"},
        max_tokens=100
    )

    result = json.loads(response.choices[0].message.content)
    return RobotIntent(
        action=result.get('action', 'unknown'),
        target=result.get('target'),
        parameters=result.get('parameters', {}),
        confidence=0.75,  # Lower confidence for LLM parsing
        raw_text=text
    )
```

## 1.5 Mapping Intents to ROS 2 Actions

Once we have structured intents, we need to execute them on the robot through ROS 2.

### Command-to-Action Routing

Create a router that maps intents to appropriate ROS 2 interfaces:

```python
class ActionRouter(Node):
    """Route intents to ROS 2 topics and actions."""

    def __init__(self):
        super().__init__('action_router')

        # Action clients for long-running tasks
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        self.pick_client = ActionClient(
            self, PickObject, '/pick_object'
        )

        # Publishers for immediate commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.emergency_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )

        # Intent subscriber
        self.intent_sub = self.create_subscription(
            String, '/voice/intent',
            self.intent_callback, 10
        )

        # Action routing table
        self.action_handlers = {
            'navigate': self.handle_navigate,
            'stop': self.handle_stop,
            'pick': self.handle_pick,
            'place': self.handle_place,
            'turn': self.handle_turn,
            'status': self.handle_status,
            'emergency_stop': self.handle_emergency,
        }
```

### Topics vs Action Clients

Choose the right ROS 2 interface for each command type:

| Command Type | Interface | Reason |
|--------------|-----------|--------|
| Emergency stop | Publisher | Immediate, no feedback needed |
| Velocity commands | Publisher | Continuous, real-time control |
| Navigation | Action Client | Long-running, needs progress feedback |
| Manipulation | Action Client | Multi-step, can be preempted |
| Status query | Service Client | Request-response pattern |

### Parameter Passing

Extract and validate parameters from intents:

```python
def handle_navigate(self, intent: RobotIntent):
    """Handle navigation intent."""
    target = intent.target

    # Look up target location
    if target in self.known_locations:
        pose = self.known_locations[target]
    elif target == 'operator':
        pose = self.get_operator_location()
    else:
        self.get_logger().error(f'Unknown location: {target}')
        self.speak_feedback(f"I don't know where {target} is")
        return

    # Create navigation goal
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = pose['x']
    goal.pose.pose.position.y = pose['y']
    goal.pose.pose.orientation.w = pose.get('orientation_w', 1.0)

    # Send goal with feedback callback
    self.nav_client.send_goal_async(
        goal,
        feedback_callback=self.nav_feedback_callback
    )

    self.speak_feedback(f"Navigating to {target}")
```

### Acknowledgment and Feedback

Provide audio feedback for user confidence:

```python
def speak_feedback(self, message: str):
    """Provide audio feedback to user."""
    # Publish to text-to-speech node
    msg = String()
    msg.data = message
    self.tts_pub.publish(msg)

def nav_feedback_callback(self, feedback_msg):
    """Handle navigation progress feedback."""
    feedback = feedback_msg.feedback
    distance = feedback.distance_remaining

    # Periodic progress updates
    if int(distance) % 5 == 0:  # Every 5 meters
        self.speak_feedback(f"{distance:.0f} meters remaining")
```

## 1.6 Wake-Word Detection

Continuous listening is resource-intensive and raises privacy concerns. Wake-word detection solves this by only activating the full pipeline when a specific phrase is spoken.

### Why Wake-Words?

- **Resource Efficiency**: Only run Whisper when needed
- **Privacy**: Don't process unintended audio
- **User Experience**: Clear activation signal
- **Battery Life**: Important for mobile robots

### Detection Options

**Porcupine** (Picovoice):
- Commercial, high accuracy
- Custom wake-words available
- Cross-platform support

**OpenWakeWord**:
- Open-source alternative
- Pre-trained models available
- Easy customization

```bash
pip install openwakeword
```

### Integration with Audio Pipeline

```python
from openwakeword import Model

class WakeWordDetector(Node):
    """Detect wake-word and trigger voice pipeline."""

    def __init__(self):
        super().__init__('wake_word_detector')

        # Load wake-word model
        self.model = Model(
            wakeword_models=["hey_robot"],
            inference_framework="onnx"
        )

        self.is_listening = False
        self.listen_timeout = 10.0  # Seconds after wake-word

        # Audio subscription
        self.audio_sub = self.create_subscription(
            AudioData, '/audio/raw',
            self.audio_callback, 10
        )

        # Control voice pipeline
        self.voice_active_pub = self.create_publisher(
            Bool, '/voice/active', 10
        )

    def audio_callback(self, msg: AudioData):
        """Check for wake-word in audio."""
        audio = np.frombuffer(msg.data, dtype=np.int16)

        # Run wake-word detection
        prediction = self.model.predict(audio)

        if prediction['hey_robot'] > 0.5:
            self.activate_listening()

    def activate_listening(self):
        """Activate voice pipeline for limited time."""
        self.is_listening = True

        msg = Bool()
        msg.data = True
        self.voice_active_pub.publish(msg)

        self.get_logger().info('Wake-word detected! Listening...')

        # Set timeout to deactivate
        self.create_timer(
            self.listen_timeout,
            self.deactivate_listening,
            callback_group=None
        )
```

### Voice Activity Detection (VAD)

Combine wake-word detection with VAD for robust activation:

```python
import webrtcvad

class VADProcessor:
    """Voice Activity Detection for audio segmentation."""

    def __init__(self, aggressiveness=2):
        # aggressiveness: 0-3, higher = more aggressive filtering
        self.vad = webrtcvad.Vad(aggressiveness)
        self.sample_rate = 16000
        self.frame_duration = 30  # ms

    def is_speech(self, audio_bytes: bytes) -> bool:
        """Check if audio frame contains speech."""
        return self.vad.is_speech(audio_bytes, self.sample_rate)
```

## 1.7 Hands-On Exercise: Voice-Controlled Movement

Let's build a complete voice control system for moving a robot.

### Step 1: Set Up Audio Capture

```bash
# Terminal 1: Start audio capture
ros2 run audio_capture audio_capture_node --ros-args \
  -p device:=default \
  -p sample_rate:=16000 \
  -r audio:=/voice/audio_raw
```

Verify audio is flowing:

```bash
# Terminal 2: Check audio topic
ros2 topic echo /voice/audio_raw --no-arr
```

### Step 2: Launch Whisper Node

```bash
# Terminal 3: Start Whisper transcription
cd examples/module-4/chapter-1/whisper_node
python3 whisper_node.py --ros-args \
  -p model_size:=base \
  -p language:=en
```

Test with spoken input:

```bash
# Terminal 4: Monitor transcriptions
ros2 topic echo /voice/transcript
```

### Step 3: Start Intent Parser

```bash
# Terminal 5: Launch intent parser
cd examples/module-4/chapter-1/command_parser
python3 intent_parser.py --ros-args \
  -p confidence_threshold:=0.7 \
  -p commands_file:=commands.yaml
```

### Step 4: Test with Simulated Robot

Launch a simple robot simulator:

```bash
# Terminal 6: Start robot simulation (using turtlesim as example)
ros2 run turtlesim turtlesim_node
```

Create a simple action router:

```bash
# Terminal 7: Run action router
ros2 run voice_control action_router_node
```

### Step 5: Complete Pipeline Test

```bash
# Use the launch file for full system
ros2 launch voice_control voice_control.launch.py
```

Test commands:
1. "Move forward" - Robot should start moving
2. "Stop" - Robot should stop
3. "Turn left ninety degrees" - Robot should rotate
4. "Go to kitchen" - Robot should navigate (if location defined)

### Success Criteria Checklist

- [ ] Audio capture publishes at ~31 Hz
- [ ] Whisper transcribes speech within 2 seconds
- [ ] Intent parser recognizes 8/10 test commands
- [ ] Actions execute on simulated robot
- [ ] System handles unknown commands gracefully

## Key Takeaways

1. **Modular Pipeline**: Separate audio capture, STT, parsing, and execution for maintainability
2. **Whisper Integration**: Local deployment with `base` model offers good speed/accuracy tradeoff
3. **Hybrid Parsing**: Rule-based for common commands, LLM fallback for complex ones
4. **ROS 2 Integration**: Use Action Clients for long-running tasks, Publishers for immediate commands
5. **Wake-Word Detection**: Essential for practical deployment to save resources and protect privacy

## Next Steps

In [Chapter 2: Language-Driven Cognitive Planning](./chapter-2-cognitive-planning.md), we'll extend our voice interface to handle complex, multi-step commands like "clean up the living room" by integrating LLM-based task planning.

## Additional Resources

- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [audio_common ROS 2 Package](https://github.com/ros-drivers/audio_common)
- [OpenWakeWord](https://github.com/dscripka/openWakeWord)
- [Porcupine Wake Word](https://picovoice.ai/platform/porcupine/)
