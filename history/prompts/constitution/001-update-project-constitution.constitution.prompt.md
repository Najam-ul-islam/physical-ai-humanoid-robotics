---
id: 001
title: Update Project Constitution
stage: constitution
date: 2025-12-21
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Muhammad Najam
command: sp.constitution
labels: [constitution, ai-book, rag, chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - none
---

## Prompt

```text
Project: AI-Authored Technical Book with Embedded RAG Chatbot

Objective:
Create a technical book using AI/Spec-Driven Development, published with Docusaurus on GitHub Pages, and embed a RAG chatbot that answers questions strictly from the book content or user-selected text.

Core Principles:
- Spec-first, AI-driven authoring
- Book is the single source of truth
- Zero hallucination: answers must be retrieval-grounded
- Fully reproducible from repository
- Production-grade clarity and structure

Standards:

Book:
- Tooling: Docusaurus (Markdown/MDX)
- AI-generated via Claude Code + Spec-Kit Plus
- Each chapter defined by a spec with objectives and acceptance criteria
- Consistent terminology and valid cross-references

Chatbot (RAG):
- Backend: FastAPI
- Vector DB: Qdrant Cloud (Free Tier)
- DB: Neon Serverless Postgres
- SDKs: OpenAI Agents / ChatKit
- Answers limited to retrieved book chunks or selected text
- If insufficient context: respond with a refusal message
- Responses must cite chapter/section
- Embedded UI inside Docusaurus
- Supports "Ask about selected text"

Data & Behavior:
- Documented chunking and metadata strategy
- No external web access at inference
- Deterministic responses for identical inputs
- Clear separation of ingestion, retrieval, generation

Constraints:
- GitHub Pages deployment
- Free-tier infrastructure only
- No external or proprietary data
- Modular and extensible design

Documentation:
- Architecture overview
- RAG pipeline description
- Deployment and setup instructions

Success Criteria:
- Book deploys successfully
- Chatbot answers are fully grounded
- Selected-text Q&A works reliably
- No hallucinations
- All specs satisfied with zero open items
```

## Response snapshot

Updated the project constitution to reflect the AI-authored technical book with embedded RAG chatbot project. The constitution now includes six core principles:

1. Spec-First, AI-Driven Authoring
2. Single Source of Truth
3. Zero Hallucination Guarantee (NON-NEGOTIABLE)
4. Reproducible and Deterministic
5. Production-Grade Quality
6. Free-Tier Infrastructure Compliance

Updated dependent templates to align with the new constitution:
- Modified plan-template.md to include constitution checks for grounding guarantees
- Updated spec-template.md with functional requirements for zero hallucination
- Enhanced tasks-template.md with foundational tasks for RAG infrastructure

## Outcome

- ✅ Impact: Created comprehensive project constitution with 6 core principles aligned to AI-book and RAG requirements
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated 4 files (.specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- 🔁 Next prompts: Project can now proceed with spec-driven development following new constitution
- 🧠 Reflection: Constitution provides strong grounding guarantees essential for trustworthiness of the RAG chatbot system

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Begin with spec creation for first book chapter