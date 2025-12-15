<!--
Version change: old → 1.0.0 → new → 2.0.0
List of modified principles:
- Old principles removed, new numbered principles added.
  - Follow Specifications Exactly (removed)
  - Maintain Consistency (removed)
  - Prioritize Documentation Standards (removed)
- New principles added:
  - 1. Spec-First
  - 2. Clarity & Consistency
  - 3. Accuracy Over Speed
  - 4. Iterative Improvement
  - 5. Transparency

Added sections:
- Purpose
- Rules / Constraints

Removed sections:
- Book Organization (content moved to Rules / Constraints)
- Specific governance rules (simplified)

Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated (no explicit changes needed)
- .specify/templates/spec-template.md: ✅ updated (no explicit changes needed)
- .specify/templates/tasks-template.md: ✅ updated (no explicit changes needed)
- .specify/templates/commands/*.md: ⚠ pending (directory not found)
- README.md: ⚠ pending (file not found)
- docs/quickstart.md: ⚠ pending (file not found)

Follow-up TODOs:
- TODO(RATIFICATION_DATE): Clarify original ratification date if different from last amended date.
- Investigate missing .specify/templates/commands/ directory.
- Investigate missing README.md and docs/quickstart.md.
-->
# Constitution

## Purpose
This project will create a fully spec-driven technical book using Spec-Kit Plus, Claude Code, Docusaurus, and GitHub Pages. All content must be generated from approved specifications only.

## Core Principles
1. **Spec-First** — No content is created without a written specification.
2. **Clarity & Consistency** — Terminology, style, formatting, and structure must stay uniform.
3. **Accuracy Over Speed** — AI should prioritize correctness, not quick generation.
4. **Iterative Improvement** — Every draft may be refined based on updated specs.
5. **Transparency** — AI must ask clarifying questions when specs are incomplete.

## Rules / Constraints
- AI must strictly follow specifications and avoid assumptions.
- Every chapter must follow the same structural standard.
- Docusaurus will be used as the book framework with a defined sidebar.
- All files must be version-controlled using GitHub.
- Final output must deploy on GitHub Pages using a clean build.

This constitution governs all decisions, workflows, and AI interactions throughout the project.

**Version**: 2.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
