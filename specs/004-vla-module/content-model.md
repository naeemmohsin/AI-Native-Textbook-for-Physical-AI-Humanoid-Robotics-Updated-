# Content Model: Module 4 – Vision-Language-Action (VLA)

**Branch**: `004-vla-module`
**Date**: 2025-12-22

## Module Overview

| Attribute | Value |
|-----------|-------|
| Module Title | Module 4: Vision-Language-Action (VLA) |
| Target Audience | AI and robotics engineers |
| Total Word Count | ~8000-9000 words |
| Total Duration | 4-5 hours |
| Chapters | 3 |
| Code Examples | 15-20 files |

## Chapter 1: Voice-to-Action Interfaces

### Metadata

| Attribute | Value |
|-----------|-------|
| Sidebar Position | 1 |
| Word Count | 2500-3000 words |
| Duration | 90-120 minutes |
| Priority | P1 (MVP) |
| Examples | 5-6 files |

### Learning Objectives

1. Configure audio capture and processing in ROS 2
2. Integrate OpenAI Whisper for speech-to-text transcription
3. Implement command parsing to extract robot intents
4. Map voice commands to ROS 2 topics and actions
5. Add wake-word detection for hands-free activation

### Section Outline

#### 1.1 Introduction to Voice-Controlled Robotics (300 words)
- Why voice control for humanoid robots
- Voice interface architecture overview
- Pipeline: Audio → Wake-word → STT → Intent → Action

#### 1.2 Audio Capture with ROS 2 (400 words)
- The `audio_common` package
- Microphone configuration and testing
- Audio message types and topics
- Handling audio streams in Python

#### 1.3 Speech-to-Text with OpenAI Whisper (600 words)
- Whisper model overview and sizes
- Local vs API deployment options
- Creating a Whisper ROS 2 node
- Handling streaming vs batch transcription
- Tuning for accuracy and latency

#### 1.4 Command Parsing and Intent Extraction (500 words)
- Rule-based parsing for common commands
- Intent schema and parameters
- Confidence scoring and thresholds
- Handling ambiguous or unknown commands
- LLM fallback for complex parsing

#### 1.5 Mapping Intents to ROS 2 Actions (400 words)
- Command-to-action routing
- Topic publishers vs action clients
- Parameter passing from voice to robot
- Acknowledgment and feedback

#### 1.6 Wake-Word Detection (300 words)
- Why wake-words for continuous listening
- Porcupine and OpenWakeWord options
- Integration with audio pipeline
- Voice activity detection (VAD)

#### 1.7 Hands-On Exercise: Voice-Controlled Movement (500 words)
- Step-by-step: Set up audio capture
- Implement Whisper transcription node
- Create command parser
- Test with simulated robot
- Success criteria checklist

### Code Examples

| File | Description | Location |
|------|-------------|----------|
| whisper_node.py | Whisper ROS 2 transcription node | chapter-1/whisper_node/ |
| audio_config.yaml | Audio capture configuration | chapter-1/audio_capture/ |
| intent_parser.py | Rule-based intent extraction | chapter-1/command_parser/ |
| commands.yaml | Command vocabulary definition | chapter-1/command_parser/ |
| voice_control.launch.py | Full voice pipeline launch | chapter-1/ |

---

## Chapter 2: Language-Driven Cognitive Planning

### Metadata

| Attribute | Value |
|-----------|-------|
| Sidebar Position | 2 |
| Word Count | 2500-3000 words |
| Duration | 90-120 minutes |
| Priority | P2 |
| Examples | 5-6 files |

### Learning Objectives

1. Integrate LLM APIs (OpenAI) for natural language understanding
2. Define robot capabilities in a machine-readable format
3. Decompose high-level commands into action sequences
4. Generate ROS 2-compatible execution plans
5. Monitor execution and handle failures with replanning

### Section Outline

#### 2.1 LLMs for Robot Task Planning (400 words)
- Why LLMs for robotics
- Grounding language in physical reality
- Limitations and failure modes
- Ethical considerations for AI-controlled robots

#### 2.2 Robot Capability Registry (400 words)
- Defining what the robot can do
- YAML capability schema
- Mapping to ROS 2 action servers
- Dynamic capability discovery

#### 2.3 LLM Integration with ROS 2 (500 words)
- OpenAI API setup and authentication
- Creating an LLM planner node
- Prompt engineering for task decomposition
- Function calling for structured output
- Local alternatives (Ollama + Llama)

#### 2.4 Task Decomposition and Planning (500 words)
- Breaking down "pick up the cup"
- Preconditions and effects
- Action sequencing strategies
- Handling impossible tasks

#### 2.5 Action Execution with Behavior Trees (500 words)
- BehaviorTree.CPP overview
- Creating action nodes
- Sequence, fallback, and parallel
- Connecting to ROS 2 action servers

#### 2.6 Execution Monitoring and Replanning (400 words)
- Tracking action progress
- Detecting failures
- Triggering replanning
- Natural language failure explanations

#### 2.7 Hands-On Exercise: Pick and Place Planner (500 words)
- Step-by-step: Define capabilities
- Create LLM planner node
- Test with "pick up the bottle" command
- Handle failure scenarios
- Success criteria checklist

### Code Examples

| File | Description | Location |
|------|-------------|----------|
| planner_node.py | LLM-based task planner | chapter-2/llm_planner/ |
| capabilities.yaml | Robot capability definitions | chapter-2/llm_planner/ |
| task_planner.txt | Prompt template for planning | chapter-2/llm_planner/prompts/ |
| executor_node.py | Action sequence executor | chapter-2/action_executor/ |
| behavior_tree.xml | Sample behavior tree | chapter-2/action_executor/ |
| planning.launch.py | Planning system launch | chapter-2/ |

---

## Chapter 3: Capstone – The Autonomous Humanoid

### Metadata

| Attribute | Value |
|-----------|-------|
| Sidebar Position | 3 |
| Word Count | 2500-3000 words |
| Duration | 120-150 minutes |
| Priority | P3 |
| Examples | 5-6 files |

### Learning Objectives

1. Integrate all previous modules into a complete system
2. Design a high-level state machine for robot behavior
3. Combine voice control, planning, navigation, and perception
4. Implement monitoring and visualization
5. Execute end-to-end autonomous tasks

### Section Outline

#### 3.1 System Architecture Overview (400 words)
- Full pipeline: Voice → Plan → Execute → Sense → Report
- Module integration points
- ROS 2 node graph visualization
- Topic and action connections

#### 3.2 High-Level State Machine (500 words)
- YASMIN state machine framework
- States: Idle, Listening, Planning, Executing, Error
- Transitions and triggers
- Timeout and error handling

#### 3.3 Integrating Navigation (Nav2) (400 words)
- Connecting planner to Nav2 actions
- Goal pose generation from language
- Navigation feedback and progress
- Handling navigation failures

#### 3.4 Integrating Perception (Isaac ROS) (400 words)
- Object detection for task grounding
- Visual SLAM for localization
- Scene understanding for planning
- Perception feedback to planner

#### 3.5 Integrating Manipulation (Simulated) (400 words)
- Simulated manipulation actions
- Pick and place in Isaac Sim
- Grasp planning concepts
- Manipulation feedback

#### 3.6 Monitoring Dashboard (300 words)
- ROS 2 Web Bridge setup
- Status visualization options
- Audio, plan, and execution displays
- Debugging with logs

#### 3.7 Hands-On Exercise: Fetch the Object (600 words)
- Complete demo scenario setup
- "Bring me the water bottle" task
- Launch full system in simulation
- Execute and observe
- Troubleshooting common issues
- Success criteria checklist

### Code Examples

| File | Description | Location |
|------|-------------|----------|
| state_machine.py | YASMIN state machine | chapter-3/autonomous_humanoid/ |
| humanoid_config.yaml | System configuration | chapter-3/autonomous_humanoid/ |
| full_system.launch.py | Complete system launch | chapter-3/autonomous_humanoid/launch/ |
| dashboard_config.yaml | Monitoring configuration | chapter-3/monitoring/ |
| fetch_object.yaml | Demo scenario definition | chapter-3/demo_scenarios/ |
| navigate_and_report.yaml | Alternative demo | chapter-3/demo_scenarios/ |

---

## Module Index Structure

```markdown
# Module 4: Vision-Language-Action (VLA)

## What You'll Learn
- Voice-controlled robot interfaces with OpenAI Whisper
- LLM-based task planning and action sequencing
- End-to-end autonomous humanoid system integration

## Prerequisites
- Completed Modules 1-3
- NVIDIA RTX GPU with 8+ GB VRAM
- OpenAI API key (or local Whisper/Ollama setup)
- Microphone for voice input

## Chapter Overview
1. Voice-to-Action Interfaces (90-120 min)
2. Language-Driven Cognitive Planning (90-120 min)
3. Capstone – The Autonomous Humanoid (120-150 min)
```

---

## Cross-Chapter Terminology

| Term | Definition | First Used |
|------|------------|------------|
| Intent | Structured representation of user command with action type and parameters | Ch 1 |
| Capability | Description of a robot action with constraints and parameters | Ch 2 |
| Action Plan | Ordered sequence of robot actions to achieve a goal | Ch 2 |
| Grounding | Connecting language concepts to physical scene elements | Ch 2 |
| State Machine | High-level behavior controller with discrete states | Ch 3 |
| Pipeline | End-to-end data flow from input to output | All |

---

## Success Criteria Mapping

| Success Criteria | Chapter | Validation Method |
|------------------|---------|-------------------|
| SC-001: >90% transcription accuracy | Ch 1 | Hands-on exercise |
| SC-002: >85% intent recognition | Ch 1 | Command test suite |
| SC-003: <5s planning response | Ch 2 | Hands-on exercise |
| SC-004: >80% task completion | Ch 3 | Capstone demo |
| SC-005: >70% failure recovery | Ch 2, 3 | Error injection tests |
| SC-006: Chapter completion time | All | Duration estimates |
| SC-007: Code examples work | All | Syntax validation |
| SC-008: Fetch task demo | Ch 3 | Capstone exercise |
