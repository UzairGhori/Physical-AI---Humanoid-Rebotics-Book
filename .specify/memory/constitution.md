<!--
Version change: 2.0.0 → 3.0.0
List of modified principles:
- Renamed: "Spec-First" → "Spec-First Development"
- Renamed: "Accuracy Over Speed" → "Accuracy Over Speed"
- Renamed: "Transparency" → "Transparency & Grounding"
- Added: "RAG-Grounded Responses" (new principle for chatbot)
- Added: "SDK-First Architecture" (new principle for OpenAI/ChatKit integration)

Added sections:
- Technology Stack (new)
- RAG Chatbot Architecture (new)
- Integration Points (new)

Removed sections:
- None

Templates requiring updates:
- .specify/templates/plan-template.md: ✅ compatible (Constitution Check section)
- .specify/templates/spec-template.md: ✅ compatible (no changes needed)
- .specify/templates/tasks-template.md: ✅ compatible (no changes needed)
- README.md: ⚠ pending (may need chatbot integration docs)

Follow-up TODOs:
- None - all placeholders resolved
-->
# Constitution

## Purpose

This project creates a spec-driven technical book on Physical AI & Humanoid Robotics using Spec-Kit Plus, Claude Code, Docusaurus, and GitHub Pages. The project includes an Integrated RAG Chatbot that provides book-grounded responses using the OpenAI Python SDK (Agents & ChatKit), with optional OpenRouter model fallback. All content MUST be generated from approved specifications only.

## Core Principles

1. **Spec-First Development** — No content or code is created without a written specification. All chatbot features, book chapters, and integrations require approved specs before implementation.

2. **Clarity & Consistency** — Terminology, style, formatting, and structure MUST remain uniform across all book content and chatbot responses. API contracts and data models MUST follow established patterns.

3. **Accuracy Over Speed** — AI-generated content MUST prioritize correctness over quick generation. Chatbot responses MUST be verified against book embeddings before delivery.

4. **Iterative Improvement** — Every draft may be refined based on updated specs. Chatbot knowledge base MUST be updated when book content changes.

5. **Transparency & Grounding** — AI MUST ask clarifying questions when specs are incomplete. Chatbot MUST indicate when queries fall outside book content scope.

6. **RAG-Grounded Responses** — Chatbot responses MUST be grounded exclusively in book content embeddings. The system MUST NOT hallucinate or provide information beyond the indexed book text chunks.

7. **SDK-First Architecture** — All agent and chat workflow logic MUST use the official OpenAI Python SDK. ChatKit Python SDK MUST be used for embedding chat UI in Docusaurus.

## Technology Stack

| Component | Technology | Constraint |
|-----------|------------|------------|
| Agent Logic | OpenAI Python SDK (Agents) | MUST use official SDK |
| Chat UI | ChatKit Python SDK | Embed in Docusaurus via JavaScript |
| Embeddings | Qwen + Bonsai (Python) | MUST generate in Python |
| Vector Store | Qdrant | Primary retrieval database |
| Metadata | Neon Serverless Postgres | Store chunk metadata |
| API Server | FastAPI (Python) | Host agent/chat endpoints |
| Book Framework | Docusaurus | Static site with GitHub Pages |
| Model Fallback | OpenRouter (optional) | Configure via Python SDK |

## RAG Chatbot Architecture

### Data Flow

1. **Ingestion**: Book content → Chunk extraction → Qwen/Bonsai embeddings → Qdrant storage
2. **Query**: User question → Embedding → Qdrant similarity search → Context retrieval
3. **Response**: Context + Query → OpenAI Agent → Grounded response → ChatKit UI

### Constraints

- Chatbot MUST only answer using book content embeddings
- Responses MUST cite source chapter/section when possible
- Out-of-scope queries MUST be gracefully declined with guidance
- OpenRouter fallback MUST be configurable via environment variables

## Integration Points

| Integration | Method | Owner |
|-------------|--------|-------|
| Docusaurus ↔ ChatKit | JavaScript embed | Frontend |
| FastAPI ↔ OpenAI SDK | Python agent endpoints | Backend |
| FastAPI ↔ Qdrant | Vector search queries | Backend |
| FastAPI ↔ Neon Postgres | Metadata queries | Backend |
| OpenRouter (optional) | SDK config swap | Backend |

## Rules / Constraints

- AI MUST strictly follow specifications and avoid assumptions
- Every chapter MUST follow the same structural standard
- Docusaurus MUST be used as the book framework with a defined sidebar
- All files MUST be version-controlled using GitHub
- Final output MUST deploy on GitHub Pages using a clean build
- Chatbot MUST only answer using book content embeddings
- Python server (FastAPI) MUST host the agent/chat endpoints
- Chat UI MUST be embedded in the Docusaurus site via JavaScript
- OpenRouter MUST be supported as an optional model endpoint via Python SDK config
- Secrets and API keys MUST NOT be hardcoded; use `.env` files

## Governance

### Amendment Process

1. Propose changes via Pull Request with rationale
2. Changes require review and approval
3. Version bump follows semantic versioning:
   - **MAJOR**: Backward-incompatible principle changes or removals
   - **MINOR**: New principles/sections added or materially expanded
   - **PATCH**: Clarifications, typos, non-semantic refinements

### Compliance Review

- All PRs MUST be checked against constitution principles
- Chatbot responses MUST be periodically audited for grounding compliance
- Spec violations MUST be documented and addressed before merge

This constitution governs all decisions, workflows, and AI interactions throughout the project.

**Version**: 3.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-17
