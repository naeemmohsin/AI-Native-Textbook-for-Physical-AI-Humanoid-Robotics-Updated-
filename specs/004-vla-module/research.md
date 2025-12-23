# Research: Module 4 – Vision-Language-Action (VLA)

**Feature Branch**: `004-vla-module`
**Date**: 2025-12-22
**Spec**: [spec.md](./spec.md)

## Research Questions

This document consolidates technology decisions and best practices for implementing the VLA module content.

---

## TD-001: Speech-to-Text Technology

**Question**: Which speech recognition system should be used for voice command input?

**Decision**: OpenAI Whisper (local deployment with whisper.cpp or Python whisper)

**Rationale**:
- Open-source and free to use locally
- Excellent accuracy (>90% WER in typical conditions)
- Multiple model sizes (tiny to large) for performance/accuracy tradeoffs
- ROS 2 integration available via community packages
- Works offline after model download (no API costs)

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Google Cloud Speech | Requires API key, ongoing costs, network dependency |
| Amazon Transcribe | Same concerns as Google, vendor lock-in |
| Vosk | Lower accuracy than Whisper, fewer languages |
| Mozilla DeepSpeech | Deprecated, limited model updates |

**Implementation Notes**:
- Use `whisper.cpp` for C++ integration or `openai-whisper` Python package
- Recommend `base` or `small` model for real-time robotics (balance speed/accuracy)
- Streaming mode via chunked audio for real-time feedback

---

## TD-002: ROS 2 Audio Integration

**Question**: How should audio input be integrated with ROS 2?

**Decision**: Use `audio_common` package with custom Whisper node

**Rationale**:
- `audio_common` provides standard ROS 2 audio message types
- Microphone input via `audio_capture` node
- Whisper processing in separate node for modularity
- Follows ROS 2 best practices for sensor integration

**Architecture**:
```text
Microphone → audio_capture → /audio_raw → whisper_node → /speech_text
                                                              ↓
                                              command_parser → /robot_intent
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Direct PyAudio | Not ROS 2 native, harder to integrate |
| GStreamer | More complex, overkill for simple capture |
| ALSA direct | Lower level, less portable |

---

## TD-003: Command Parsing Approach

**Question**: How should natural language commands be parsed into robot intents?

**Decision**: Hybrid approach - rule-based patterns for common commands, LLM fallback for complex

**Rationale**:
- Rule-based parsing is fast and deterministic for known commands
- LLM provides flexibility for novel phrasings
- Reduces latency for common operations (stop, move, turn)
- Cost-effective (no API calls for simple commands)

**Command Categories**:
1. **Motion commands**: move, stop, turn, rotate (rule-based)
2. **Navigation commands**: go to, navigate, find (rule-based + scene context)
3. **Manipulation commands**: pick, place, grasp (LLM-assisted)
4. **Complex tasks**: multi-step, conditional (LLM planning)

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| LLM-only | High latency, API costs, network dependency |
| Rule-based only | Inflexible, poor handling of variations |
| RASA NLU | Additional complexity, training overhead |

---

## TD-004: LLM Integration for Task Planning

**Question**: Which LLM approach should be used for cognitive planning?

**Decision**: OpenAI API (GPT-4) with local alternative guidance (Ollama + Llama)

**Rationale**:
- GPT-4 provides best-in-class reasoning for task decomposition
- OpenAI API is widely accessible with straightforward Python integration
- Include guidance for Ollama + Llama 2/3 for offline/cost-conscious users
- Structured output via function calling for reliable action sequences

**Implementation Notes**:
- Use `openai` Python package with ROS 2 node wrapper
- Prompt engineering for robot action domain
- Robot capabilities provided as function definitions
- Retry logic with exponential backoff for API resilience

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Anthropic Claude | Similar capability, less widespread adoption |
| Google Gemini | Newer, less ecosystem tooling |
| Local-only LLM | Lower capability for complex planning |
| Custom fine-tuned | High development cost, maintenance burden |

---

## TD-005: Action Execution Framework

**Question**: How should planned actions be executed on the robot?

**Decision**: ROS 2 Action Servers with BehaviorTree.CPP integration

**Rationale**:
- Action servers provide feedback and cancellation for long-running tasks
- BehaviorTree.CPP enables complex action sequencing and recovery
- Aligns with Nav2 architecture (readers already familiar from Module 3)
- Industry standard for robot behavior orchestration

**Architecture**:
```text
LLM Plan → Action Sequence → BehaviorTree → Action Clients → Robot
                                    ↓
                            Status Feedback → Plan Monitor
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Simple service calls | No feedback, no cancellation support |
| Custom state machine | Reinventing the wheel, less maintainable |
| SMACH | Python-only, older framework |

---

## TD-006: Wake-Word Detection

**Question**: How should wake-word detection be implemented?

**Decision**: Porcupine (Picovoice) with fallback to OpenWakeWord

**Rationale**:
- Porcupine: lightweight, accurate, free tier available
- OpenWakeWord: fully open-source alternative
- Both work offline with low CPU usage
- Enables hands-free activation

**Implementation Notes**:
- Default wake-word: "Hey Robot" or custom keyword
- Continuous listening with wake-word → command recording → Whisper
- VAD (Voice Activity Detection) for end-of-utterance detection

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| Snowboy | Discontinued |
| Precise (Mycroft) | Limited community support |
| Always-on Whisper | High CPU/GPU usage, inefficient |

---

## TD-007: Robot Capabilities Registry

**Question**: How should robot capabilities be defined and registered?

**Decision**: YAML-based capability definitions with ROS 2 parameter server

**Rationale**:
- YAML is human-readable and easy to edit
- Parameter server enables runtime capability discovery
- Matches LLM function calling format for direct mapping
- Extensible for new capabilities without code changes

**Capability Schema**:
```yaml
capabilities:
  - name: navigate_to
    description: "Navigate to a named location or coordinates"
    parameters:
      - name: target
        type: string
        description: "Location name or 'x,y' coordinates"
    action_server: /navigate_to_pose

  - name: pick_object
    description: "Pick up an object by name"
    parameters:
      - name: object_name
        type: string
        description: "Name of the object to pick"
    action_server: /pick_object
```

---

## TD-008: System State Machine

**Question**: How should high-level robot behavior be managed?

**Decision**: YASMIN (Yet Another State MachINe) for Python, compatible with ROS 2

**Rationale**:
- Pure Python, easy to understand and extend
- ROS 2 native with action/service integration
- Visualization via YASMIN Viewer
- Simpler than BT for top-level states

**State Machine Structure**:
```text
IDLE → LISTENING → PROCESSING → PLANNING → EXECUTING → IDLE
  ↑                                              ↓
  └──────────── ERROR_RECOVERY ←─────────────────┘
```

**Alternatives Considered**:
| Alternative | Why Rejected |
|-------------|--------------|
| FlexBE | More complex, heavier setup |
| ros2_lifecycle | Too low-level for behavior orchestration |
| Custom FSM | More development effort |

---

## TD-009: Monitoring and Visualization

**Question**: How should system status be monitored and displayed?

**Decision**: ROS 2 Web Bridge + React dashboard (or RViz2 for simpler option)

**Rationale**:
- rosbridge_suite enables web-based monitoring
- React dashboard provides rich visualization
- RViz2 serves as simpler alternative for ROS-native users
- Both show: audio waveform, transcription, plan, execution status

**Dashboard Components**:
1. Audio input visualization
2. Transcription display
3. Current action plan
4. Execution progress
5. Robot status (battery, position, sensors)

---

## TD-010: Simulation Environment

**Question**: Which simulation environment should be used for the capstone?

**Decision**: Isaac Sim (primary) with Gazebo fallback

**Rationale**:
- Isaac Sim provides photorealistic rendering for perception testing
- Readers already set up Isaac Sim in Module 3
- Gazebo Classic as fallback for lower hardware requirements
- Both support ROS 2 Humble

**Simulation Requirements**:
- Humanoid robot with manipulation capability
- Indoor environment with navigable rooms
- Objects for fetch/manipulation tasks
- Audio simulation not required (use real microphone)

---

## Hardware Requirements Summary

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | NVIDIA RTX 2070 | NVIDIA RTX 3080+ |
| VRAM | 8 GB | 12+ GB |
| RAM | 32 GB | 64 GB |
| Storage | 50 GB SSD | 100 GB NVMe |
| Audio | USB Microphone | Dedicated audio interface |
| CPU | 8 cores | 12+ cores |

---

## Software Stack Summary

| Layer | Technology |
|-------|------------|
| Speech Recognition | OpenAI Whisper (local) |
| Wake-Word Detection | Porcupine / OpenWakeWord |
| Audio Capture | audio_common (ROS 2) |
| Command Parsing | Rule-based + LLM hybrid |
| LLM Integration | OpenAI API (GPT-4) |
| Action Execution | BehaviorTree.CPP |
| State Machine | YASMIN |
| Navigation | Nav2 (from Module 3) |
| Perception | Isaac ROS (from Module 3) |
| Simulation | Isaac Sim / Gazebo |
| Middleware | ROS 2 Humble |
