---
id: 004
title: Document RAG Chatbot Architecture Decisions
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-rag-chatbot
branch: 002-rag-chatbot
user: user
command: /sp.adr
labels: ["adr", "architecture", "technology-stack", "embeddings", "persistence"]
links:
  spec: specs/002-rag-chatbot/spec.md
  ticket: null
  adr: history/adr/ADR-002-RAG-Chatbot-Backend-Technology-Stack.md
  pr: null
files:
  - history/adr/ADR-002-RAG-Chatbot-Backend-Technology-Stack.md
  - history/adr/ADR-003-RAG-Data-Persistence-Stack.md
  - history/adr/ADR-004-Embedding-Strategy-Primary-Fallback.md
tests:
  - null
---

## Prompt

/sp.adr "RAG Chatbot Technology Stack Selection"

## Response snapshot

Analyzed planning artifacts (plan.md, research.md, data-model.md) and extracted 3 architecturally significant decision clusters:

1. **ADR-002: RAG Chatbot Backend Technology Stack**
   - FastAPI + OpenAI Python SDK + Uvicorn + Python 3.11
   - Alternatives: Django+DRF, Flask+Quart, Node.js

2. **ADR-003: RAG Data Persistence Stack**
   - Qdrant Cloud + Neon Serverless Postgres + asyncpg
   - Alternatives: pgvector only, Pinecone+Supabase, Self-hosted, Weaviate

3. **ADR-004: Embedding Strategy with Primary-Fallback Pattern**
   - Qwen primary (5s timeout) + Bonsai fallback, 1536 dimensions
   - Alternatives: Ensemble averaging, Dual index, Sequential reranking

Each ADR includes: context, decision details, positive/negative consequences, alternatives considered with rejection rationale, and references to spec/plan.

No conflicts detected with existing ADR-001 (Task Structure).

## Outcome

- ✅ Impact: 3 ADRs created documenting all significant architectural decisions
- 🧪 Tests: N/A (documentation)
- 📁 Files: 3 created (ADR-002, ADR-003, ADR-004)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation task list
- 🧠 Reflection: All ADRs properly clustered (not atomic), include alternatives with rationale, and reference constitution requirements. Decisions are traceable to spec clarifications.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL):
  - Clustered decisions (not atomic): PASS
  - Explicit alternatives with rationale: PASS
  - Clear pros/cons: PASS
  - Concise but sufficient: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
