---
id: 3
title: Create Specification for Physical AI & Humanoid Robotics Book/Course
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-humanoid-robotics
branch: master
user: user
command: /sp.specify Write a comprehensive specification for a “Physical AI & Humanoid Robotics” book/course that teaches AI systems in the physical world and embodied intelligence using ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA (Vision-Language-Action). Target audience: Advanced AI and robotics students preparing for a capstone in embodied intelligence. Focus: Bridging digital AI concepts with physical robotics implementations to teach theory, simulation, perception, and real-world humanoid behavior. Success criteria: - Covers all 4 core modules (ROS 2, Simulation, NVIDIA Isaac, VLA) - Includes weekly contents, learning outcomes, and assessments - Explains hardware requirements and provides edge/cloud lab options - Practical labs and capstone project with step-by-step instructions - Each technical topic has measurable learning checkpoints Constraints: - Format: Markdown/MDX compatible with Docusaurus - Structure: Clear chapters per module, tables, diagrams, and glossary - Word count per chapter: 1500–3000 words - Timeline: Complete within 3 weeks - Use only educational, publicly available frameworks and tools Not building: - Complete commercial robot hardware teardown guides - In-depth GPU driver or OS setup beyond course context - Ethics, policy, or socio-economic analysis of AI robotics - Vendor-specific proprietary SDK deep dives outside open frameworks
labels: ["specification", "physical-ai", "robotics", "book-course"]
links:
  spec: specs/physical-ai-humanoid-robotics/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-humanoid-robotics/spec.md
tests:

---

## Prompt

```text
/sp.specify Write a comprehensive specification for a “Physical AI & Humanoid Robotics” book/course that teaches AI systems in the physical world and embodied intelligence using ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA (Vision-Language-Action).

Target audience: Advanced AI and robotics students preparing for a capstone in embodied intelligence.

Focus: Bridging digital AI concepts with physical robotics implementations to teach theory, simulation, perception, and real-world humanoid behavior.

Success criteria:
- Covers all 4 core modules (ROS 2, Simulation, NVIDIA Isaac, VLA)
- Includes weekly contents, learning outcomes, and assessments
- Explains hardware requirements and provides edge/cloud lab options
- Practical labs and capstone project with step-by-step instructions
- Each technical topic has measurable learning checkpoints

Constraints:
- Format: Markdown/MDX compatible with Docusaurus
- Structure: Clear chapters per module, tables, diagrams, and glossary
- Word count per chapter: 1500–3000 words
- Timeline: Complete within 3 weeks
- Use only educational, publicly available frameworks and tools

Not building:
- Complete commercial robot hardware teardown guides
- In-depth GPU driver or OS setup beyond course context
- Ethics, policy, or socio-economic analysis of AI robotics
- Vendor-specific proprietary SDK deep dives outside open frameworks
```

## Response snapshot

```text
The comprehensive specification for the "Physical AI & Humanoid Robotics" book/course has been created at `specs/physical-ai-humanoid-robotics/spec.md`. It covers user scenarios, functional requirements, key entities, and measurable success criteria, aligning with the user's detailed request. The specification outlines four core modules (ROS 2, Simulation, NVIDIA Isaac, VLA), includes requirements for weekly content, practical labs, and hardware considerations, and adheres to Docusaurus compatibility.
```

## Outcome

- ✅ Impact: Comprehensive specification created.
- 🧪 Tests: No explicit tests were run for this task, but the specification includes acceptance scenarios for user stories.
- 📁 Files: Created `specs/physical-ai-humanoid-robotics/spec.md`.
- 🔁 Next prompts: The user will likely review the specification and proceed to planning.
- 🧠 Reflection: Successfully translated detailed user requirements into a structured specification, addressing all key areas and constraints. Handled missing script gracefully.

## Evaluation notes (flywheel)

- Failure modes observed: Script for PHR creation was not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Continue to ensure all required project structure for Spec-Kit Plus is present.
