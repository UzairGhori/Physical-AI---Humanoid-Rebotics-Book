---
id: 2
title: Add RAG Chatbot Principles to Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "rag-chatbot", "openai-sdk", "chatkit", "qdrant", "fastapi"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

/sp.constitution

Purpose:
Define principles and quality standards for the Physical AI & Humanoid Robotics Book Project, including the Integrated RAG Chatbot using the OpenAI Python SDK (Agents & ChatKit), with optional OpenRouter model fallback.

Core Principles:
- Use the official OpenAI Python SDK for agent and chat workflow logic.
- Use ChatKit Python SDK to embed chat UI in Docusaurus.
- Embeddings must be generated using Qwen and Bonsai in Python.
- Retrieval must use Qdrant vector DB with Neon Serverless Postgres metadata.
- Chatbot responses must be grounded in selected book text chunks only.

Rules / Constraints:
- Chatbot must only answer using book content embeddings.
- Python server (FastAPI) will host the agent/chat endpoints.
- Chat UI embedded in the Docusaurus site integrated via JavaScript.
- Must support OpenRouter as an optional model endpoint via Python SDK config.

## Response snapshot

Updated constitution from v2.0.0 to v3.0.0 with:
- 2 new principles: "RAG-Grounded Responses" and "SDK-First Architecture"
- 3 new sections: Technology Stack, RAG Chatbot Architecture, Integration Points
- Expanded Rules/Constraints for chatbot requirements
- Added Governance section with amendment process and compliance review

## Outcome

- ✅ Impact: Major version bump (2.0.0 → 3.0.0) - added RAG chatbot governance and technology stack requirements
- 🧪 Tests: N/A (constitution document)
- 📁 Files: 1 modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Consider /sp.specify for RAG chatbot feature, /sp.adr for architecture decisions
- 🧠 Reflection: Constitution now comprehensively covers both book content generation AND chatbot architecture requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test constitution gates during next /sp.plan execution
