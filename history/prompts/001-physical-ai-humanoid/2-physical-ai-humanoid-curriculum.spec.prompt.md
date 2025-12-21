---
id: 2
title: Physical AI Humanoid Curriculum
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-humanoid
branch: 001-physical-ai-humanoid
user: user
command: /sp.specify
labels: [curriculum,physical-ai,robotics,humanoid,ros2]
links:
  spec: specs/001-physical-ai-humanoid/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-physical-ai-humanoid/spec.md
 - specs/001-physical-ai-humanoid/checklists/requirements.md
 - history/prompts/001-physical-ai-humanoid/2-physical-ai-humanoid-curriculum.spec.prompt.md
tests:
---

## Prompt

```
text
objective: Define the knowledge, skills and capabilities the student must gain from this module.focus on physical-world execution, not theoretical AI only. Target Audience

Students in advanced AI/robotics programs

Robotics engineers transitioning into embodied AI

Developers learning ROS 2, Gazebo, Isaac Sim, and VLA

Educators building curriculum for humanoid robotics

Focus

The book teaches how to bridge AI models with physical robots, specifically humanoids, through:

Physical AI foundations & embodied intelligence

ROS 2 for robot motor/behavior control

Gazebo & Unity for physics simulation

NVIDIA Isaac for perception, VSLAM, synthetic data, and manipulation

Vision-Language-Action (VLA) pipelines using LLMs

Capstone: Autonomous Humanoid executing natural language tasks

Success Criteria

Book introduces Physical AI clearly and accurately

Covers 4 major modules: ROS2, Digital Twin, Isaac, VLA

Includes hands-on tutorials with runnable ROS 2 code

Explains simulation workflows (Gazebo, Unity, Isaac) with screenshots/steps

Guides reader from fundamentals → advanced humanoid robotics

Contains at least 20 fully tested code examples

Final chapter prepares reader to build a voice-controlled humanoid robot

All hardware recommendations based on real-world feasibility

After reading, user can:

Build & simulate a humanoid robot

Use Isaac Sim for perception

Deploy nodes to Jetson Orin

Run VLA pipelines (Whisper + GPT + ROS actions)

Constraints

Format:

Must be written in Markdown for Docusaurus

Each chapter must be self-contained with:

Objective

Concept explanation

Step-by-step tutorial

Tested code samples

Common errors + debugging section

Summary

Length:

Entire book: ~20,000–30,000 words

Each module: 3–5 chapters

Capstone: 1 extensive walkthrough

Technical Requirements:

All examples must run on:

Ubuntu 22.04

ROS 2 Humble or Iron

Isaac Sim 4.x

Gazebo Fortress or Garden

Code tested using Claude Code and rclpy

Sources:

Official ROS 2 documentation

Gazebo and Isaac Sim docs

NVIDIA Robotics papers

Academic robotics/embodied AI publications (optional, not required every chapter)

Timeline:

Completion window: 3–4 weeks

Module-by-
```

## Response snapshot

Created a comprehensive feature specification for a Physical AI Humanoid Robotics Curriculum. The specification includes:

- Four prioritized user stories covering different audience types (students, engineers, developers, educators)
- Detailed functional requirements (FR-001 to FR-015) covering curriculum content, structure, and technical requirements
- Key entities defined (Curriculum Module, Chapter, Code Example, etc.)
- Measurable success criteria (SC-001 to SC-008) focusing on student outcomes and content delivery

The specification addresses the core objectives of teaching Physical AI and embodied intelligence, ROS 2 for robot control, simulation environments, NVIDIA Isaac for perception, and VLA pipelines. It ensures the curriculum is structured with self-contained chapters and provides measurable outcomes for students to build and simulate humanoid robots, deploy to hardware, and run VLA pipelines.

## Outcome

- ✅ Impact: Created complete specification for physical AI humanoid curriculum feature
- 🧪 Tests: No tests required for specification creation
- 📁 Files: Created specs/001-physical-ai-humanoid/spec.md and specs/001-physical-ai-humanoid/checklists/requirements.md
- 🔁 Next prompts: Ready to proceed with planning (/sp.plan) or clarification (/sp.clarify)
- 🧠 Reflection: Specification covers all required aspects with clear user scenarios and measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation planning based on completed specification