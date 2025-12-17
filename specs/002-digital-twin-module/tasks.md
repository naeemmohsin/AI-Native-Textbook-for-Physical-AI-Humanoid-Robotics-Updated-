# Tasks: Module 2 – The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin-module/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, content-model.md, quickstart.md

**Tests**: Not required - this is a documentation module. Validation is via build verification and content review.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Chapter 1, US2=Chapter 2, US3=Chapter 3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `frontend_book/docs/module-2-digital-twin/`
- **Code Examples**: `examples/module-2/`
- **Static Assets**: `static/img/module-2/`

---

## Phase 1: Setup (Module Structure)

**Purpose**: Create module directory structure and navigation

- [x] T001 Create module directory at frontend_book/docs/module-2-digital-twin/
- [x] T002 Create _category_.json with module metadata in frontend_book/docs/module-2-digital-twin/_category_.json
- [x] T003 Create index.md module overview in frontend_book/docs/module-2-digital-twin/index.md
- [x] T004 [P] Create examples directory structure at examples/module-2/ with chapter subdirectories
- [x] T005 [P] Create static assets directory at static/img/module-2/

---

## Phase 2: Foundational (Gazebo/Unity Example Infrastructure)

**Purpose**: Create shared example code that chapters will reference

**⚠️ CRITICAL**: Example code must exist before chapter content can reference it

- [x] T006 Create humanoid Gazebo world file at examples/module-2/chapter-1/humanoid_gazebo/worlds/humanoid_world.sdf
- [x] T007 [P] Create ROS 2 spawn launch file at examples/module-2/chapter-1/humanoid_gazebo/launch/spawn_humanoid.launch.py
- [x] T008 [P] Create URDF-to-SDF conversion script at examples/module-2/chapter-1/humanoid_gazebo/urdf_to_sdf.sh
- [x] T009 Create ROS TCP Endpoint launch file at examples/module-2/chapter-2/ros_unity_bridge/ros_tcp_endpoint.launch.py
- [x] T010 [P] Create sensor configuration files at examples/module-2/chapter-3/sensor_configs/ (lidar_config.sdf, depth_camera_config.sdf, imu_config.sdf)
- [x] T011 [P] Create sensor subscriber node at examples/module-2/chapter-3/sensor_processing/sensor_subscriber.py

**Checkpoint**: All example code ready - chapter content can now reference working examples

---

## Phase 3: User Story 1 - Physics Simulation with Gazebo (Priority: P1) 🎯 MVP

**Goal**: Readers can import URDF humanoids into Gazebo and configure physics simulation

**Independent Test**: Reader successfully imports Module 1 URDF into Gazebo, observes gravity and collision behavior

### Implementation for User Story 1

- [x] T012 [US1] Write Chapter 1 introduction section (digital twin concepts, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T013 [US1] Write physics engine fundamentals section (rigid body dynamics, gravity, collisions, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T014 [US1] Write Gazebo Harmonic overview section (architecture, ros_gz integration, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T015 [US1] Write setting up Gazebo environment section (installation, GUI, CLI tools, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T016 [US1] Write importing URDF humanoids section (conversion, spawning, verification, 600 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T017 [US1] Write creating world files section (SDF structure, ground planes, physics, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T018 [US1] Write running simulations section (controls, ROS integration, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T019 [US1] Write hands-on exercise section with step-by-step instructions in frontend_book/docs/module-2-digital-twin/chapter-1-gazebo.md
- [x] T020 [US1] Add learning objectives and key takeaways to chapter-1-gazebo.md
- [x] T021 [US1] Verify build passes with Chapter 1 content (npm run build)

**Checkpoint**: Chapter 1 complete - readers can learn Gazebo physics simulation independently

---

## Phase 4: User Story 2 - Digital Twins & HRI in Unity (Priority: P2)

**Goal**: Readers can set up Unity-ROS 2 integration and create HRI scenarios

**Independent Test**: Reader establishes bidirectional Unity-ROS 2 communication and renders humanoid in Unity

### Implementation for User Story 2

- [x] T022 [US2] Create Unity C# scripts directory at examples/module-2/chapter-2/unity_project/Scripts/
- [x] T023 [P] [US2] Create ROSConnection.cs example at examples/module-2/chapter-2/unity_project/Scripts/ROSConnection.cs
- [x] T024 [P] [US2] Create RobotController.cs example at examples/module-2/chapter-2/unity_project/Scripts/RobotController.cs
- [x] T025 [P] [US2] Create HRIManager.cs example at examples/module-2/chapter-2/unity_project/Scripts/HRIManager.cs
- [x] T026 [US2] Write Chapter 2 introduction section (Unity for robotics, Gazebo comparison, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T027 [US2] Write Unity environment setup section (Unity Hub, packages, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T028 [US2] Write ROS-Unity integration section (TCP connector, endpoint, 600 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T029 [US2] Write importing robot models section (URDF importer, materials, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T030 [US2] Write creating realistic environments section (lighting, materials, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T031 [US2] Write bidirectional communication section (publish, subscribe, services, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T032 [US2] Write HRI basics section (avatars, proximity, safety zones, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T033 [US2] Write hands-on exercise section with step-by-step instructions in frontend_book/docs/module-2-digital-twin/chapter-2-unity.md
- [x] T034 [US2] Add learning objectives and key takeaways to chapter-2-unity.md
- [x] T035 [US2] Verify build passes with Chapter 2 content (npm run build)

**Checkpoint**: Chapter 2 complete - readers can create Unity digital twins independently

---

## Phase 5: User Story 3 - Sensor Simulation & Validation (Priority: P3)

**Goal**: Readers can configure simulated sensors with realistic noise and feed data to ROS 2

**Independent Test**: Reader adds LiDAR, depth camera, and IMU to simulated robot and verifies data in ROS 2

### Implementation for User Story 3

- [x] T036 [US3] Create sensor bridge launch file at examples/module-2/chapter-3/sensor_processing/sensor_bridge.launch.py
- [x] T037 [US3] Write Chapter 3 introduction section (sensor simulation fundamentals, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T038 [US3] Write LiDAR simulation section (plugin config, point clouds, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T039 [US3] Write depth camera simulation section (RGB-D output, FOV, 500 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T040 [US3] Write IMU simulation section (accelerometer, gyroscope, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T041 [US3] Write sensor noise models section (Gaussian, bias, drift, 400 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T042 [US3] Write ROS 2 sensor integration section (ros_gz_bridge, topics, 300 words) in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T043 [US3] Write hands-on exercise section with step-by-step instructions in frontend_book/docs/module-2-digital-twin/chapter-3-sensors.md
- [x] T044 [US3] Add learning objectives and key takeaways to chapter-3-sensors.md
- [x] T045 [US3] Verify build passes with Chapter 3 content (npm run build)

**Checkpoint**: All chapters complete - full Module 2 content ready

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and improvements across all chapters

- [x] T046 [P] Add cross-references between chapters (Next/Previous links)
- [x] T047 [P] Verify all code examples are syntactically correct
- [x] T048 Run full Docusaurus build and fix any errors (npm run build)
- [x] T049 Verify internal links work correctly
- [x] T050 Review content against spec.md success criteria

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - creates example code
- **User Stories (Phase 3-5)**: Depend on Foundational phase for example code
  - US1 (Chapter 1) can start after Foundational
  - US2 (Chapter 2) can start after Foundational (parallel with US1)
  - US3 (Chapter 3) can start after Foundational (parallel with US1, US2)
- **Polish (Phase 6)**: Depends on all chapters being complete

### User Story Dependencies

- **User Story 1 (P1)**: Gazebo - Foundation for simulation concepts
- **User Story 2 (P2)**: Unity - Can be implemented independently but references Gazebo concepts
- **User Story 3 (P3)**: Sensors - Uses Gazebo setup from US1 but can be written independently

### Within Each User Story

- Example code (Foundational phase) before chapter content
- Introduction before detailed sections
- Core content before exercises
- Exercises before summary/takeaways
- Build verification after chapter complete

### Parallel Opportunities

- T004 and T005 can run in parallel (directory creation)
- T007 and T008 can run in parallel (different launch/script files)
- T010 and T011 can run in parallel (sensor configs vs. subscriber)
- T023, T024, T025 can run in parallel (different C# scripts)
- T046 and T047 can run in parallel (cross-references vs. code review)
- Chapters US1, US2, US3 can be written in parallel by different authors

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# After Foundational phase, these chapter sections can be worked on:
# Note: Sections should be written sequentially within the chapter for narrative flow,
# but example code (already in Foundational) is ready for reference

# Foundational example code (already complete):
# - examples/module-2/chapter-1/humanoid_gazebo/worlds/humanoid_world.sdf
# - examples/module-2/chapter-1/humanoid_gazebo/launch/spawn_humanoid.launch.py

# Chapter content (sequential for narrative):
T012 → T013 → T014 → T015 → T016 → T017 → T018 → T019 → T020 → T021
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational - Gazebo examples only (T006-T008)
3. Complete Phase 3: User Story 1 - Chapter 1 (T012-T021)
4. **STOP and VALIDATE**: Build passes, Chapter 1 readable
5. Deploy/demo if ready - readers can learn Gazebo basics

### Incremental Delivery

1. Setup + Foundational → Module structure ready
2. Add Chapter 1 (US1) → Build → **MVP complete!**
3. Add Chapter 2 (US2) → Build → Unity content ready
4. Add Chapter 3 (US3) → Build → Sensors content ready
5. Polish (Phase 6) → Final validation → Module 2 complete

### Content Word Count Targets

| Chapter | Target Words | Estimated Time |
|---------|--------------|----------------|
| Chapter 1 | 2500-3000 | 90-120 min |
| Chapter 2 | 2500-3000 | 90-120 min |
| Chapter 3 | 2000-2500 | 60-90 min |
| **Total** | **7000-8500** | **4-5.5 hours** |

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story/chapter
- Each chapter should be independently completable and readable
- Build verification (npm run build) after each chapter
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- All technical content must match official Gazebo/Unity documentation

---

## Summary

| Phase | Tasks | Parallel Tasks | Description |
|-------|-------|----------------|-------------|
| Phase 1: Setup | 5 | 2 | Module directory structure |
| Phase 2: Foundational | 6 | 4 | Example code infrastructure |
| Phase 3: US1 (Chapter 1) | 10 | 0 | Gazebo physics simulation |
| Phase 4: US2 (Chapter 2) | 14 | 3 | Unity digital twins |
| Phase 5: US3 (Chapter 3) | 10 | 0 | Sensor simulation |
| Phase 6: Polish | 5 | 2 | Final validation |
| **Total** | **50** | **11** | |

**MVP Scope**: Phases 1-3 (21 tasks) delivers Chapter 1 - Gazebo Physics Simulation
