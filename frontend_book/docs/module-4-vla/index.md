---
sidebar_position: 0
title: "Module 4: Vision-Language-Action (VLA)"
description: "Voice-controlled robotics with LLM-based cognitive planning and autonomous humanoid integration"
---

# Module 4: Vision-Language-Action (VLA)

Transform your humanoid robot into an intelligent assistant that understands natural language commands, plans complex tasks, and executes them autonomously.

## What You'll Learn

- **Voice-Controlled Interfaces**: Capture audio, transcribe speech with OpenAI Whisper, and convert commands to robot actions
- **LLM-Based Task Planning**: Use large language models to decompose high-level instructions into executable action sequences
- **End-to-End Integration**: Combine voice, planning, navigation, perception, and manipulation into a complete autonomous system

## Prerequisites

Before starting this module, ensure you have:

- Completed **Module 1: ROS 2 Fundamentals** (nodes, topics, actions)
- Completed **Module 2: Digital Twin** (simulation concepts)
- Completed **Module 3: NVIDIA Isaac** (perception pipelines)
- **NVIDIA RTX GPU** with 8+ GB VRAM
- **OpenAI API key** (or local Whisper/Ollama setup)
- **Microphone** for voice input testing

## Chapter Overview

| Chapter | Title | Duration | Description |
|---------|-------|----------|-------------|
| 1 | [Voice-to-Action Interfaces](./chapter-1-voice-to-action.md) | 90-120 min | Audio capture, Whisper STT, command parsing, ROS 2 integration |
| 2 | [Language-Driven Cognitive Planning](./chapter-2-cognitive-planning.md) | 90-120 min | LLM integration, task decomposition, behavior tree execution |
| 3 | [Capstone – The Autonomous Humanoid](./chapter-3-capstone.md) | 120-150 min | Full system integration, state machine, end-to-end demo |

## Module Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Autonomous Humanoid System                    │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────────────┐   │
│  │   Voice     │   │   Cognitive │   │   Execution         │   │
│  │   Input     │──▶│   Planning  │──▶│   Pipeline          │   │
│  │  (Ch. 1)    │   │   (Ch. 2)   │   │   (Ch. 3)           │   │
│  └─────────────┘   └─────────────┘   └─────────────────────┘   │
│        │                 │                     │                │
│        ▼                 ▼                     ▼                │
│  ┌───────────┐    ┌───────────┐    ┌─────────────────────┐     │
│  │ Whisper   │    │ GPT-4     │    │ Nav2 + Isaac ROS    │     │
│  │ STT       │    │ Planner   │    │ + Manipulation      │     │
│  └───────────┘    └───────────┘    └─────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

## Key Technologies

| Technology | Purpose | Chapter |
|------------|---------|---------|
| OpenAI Whisper | Speech-to-text transcription | 1 |
| audio_common | ROS 2 audio capture | 1 |
| OpenAI GPT-4 | Task planning and decomposition | 2 |
| BehaviorTree.CPP | Action sequence execution | 2 |
| YASMIN | High-level state machine | 3 |
| Nav2 | Autonomous navigation | 3 |
| Isaac ROS | Perception and scene understanding | 3 |

## Success Criteria

By the end of this module, you will be able to:

- [ ] Achieve >90% speech transcription accuracy with Whisper
- [ ] Recognize robot intents with >85% accuracy
- [ ] Generate action plans in <5 seconds
- [ ] Complete autonomous tasks with >80% success rate
- [ ] Recover from failures with >70% success rate
- [ ] Execute the "fetch the water bottle" capstone demo

## Getting Started

Begin with [Chapter 1: Voice-to-Action Interfaces](./chapter-1-voice-to-action.md) to set up your voice-controlled robot interface.

:::tip Hardware Setup
Ensure your microphone is connected and tested before starting Chapter 1. Use `arecord -l` on Linux to verify audio devices.
:::
