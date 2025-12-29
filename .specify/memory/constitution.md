<!-- SYNC IMPACT REPORT
Version change: N/A (initial version) → 1.0.0
Added sections: All principles and sections for AI-Authored Technical Book project
Removed sections: None
Modified principles: None (first version)
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated
  - .specify/templates/spec-template.md ✅ updated
  - .specify/templates/tasks-template.md ✅ updated
  - .specify/templates/commands/*.md ⚠ pending (no specific changes needed for this project)
Runtime docs requiring updates:
  - README.md ⚠ pending
Follow-up TODOs: None
-->

# AI-Authored Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First, AI-Driven Authoring
All book content and features begin with a formal specification; AI tools (Claude Code + Spec-Kit Plus) are used for generation; Each chapter must have clear objectives and acceptance criteria before implementation. Rationale: Ensures structured, high-quality content that meets project goals through systematic development.

### Single Source of Truth
The book content serves as the definitive source for all information; All chatbot responses must be grounded in book content only; No external data sources allowed during inference. Rationale: Maintains integrity and reliability of the knowledge base by preventing conflicting or unauthorized information.

### Zero Hallucination Guarantee (NON-NEGOTIABLE)
All chatbot responses must be retrieval-grounded from book content or selected text; If insufficient context exists, respond with refusal message; Responses must cite specific chapters/sections. Rationale: Critical requirement for trustworthiness - chatbot must never fabricate information or make claims not supported by the book content.

### Reproducible and Deterministic
All processes must be fully reproducible from repository; Identical inputs produce identical outputs; Clear separation between ingestion, retrieval, and generation phases. Rationale: Ensures consistent, predictable behavior and enables verification of grounding guarantees.

### Production-Grade Quality
Book content must have consistent terminology and valid cross-references; Chatbot must provide reliable, grounded responses; All specs must be satisfied with zero open items. Rationale: Maintains professional standards and user confidence in the system's reliability and accuracy.

### Free-Tier Infrastructure Compliance
All infrastructure must run on free-tier services (Qdrant Cloud, Neon Serverless Postgres, GitHub Pages); Architecture must be modular and extensible within these constraints. Rationale: Maintains cost-effectiveness while enabling sustainable, scalable deployment within budget constraints.

## Technical Standards

Docusaurus for publishing, FastAPI backend, OpenAI Agents/ChatKit SDKs, vector database for RAG, embedded UI in Docusaurus. All technology choices must support the zero hallucination guarantee and reproducible deployment. Data storage follows documented chunking and metadata strategy with Neon Serverless Postgres for metadata and Qdrant Cloud for vector storage.

## Development Workflow

Spec-driven development, AI-assisted authoring, rigorous testing of RAG responses, cross-reference validation, deployment to GitHub Pages. All changes must preserve grounding guarantees and maintain deterministic behavior. Code reviews must verify compliance with zero hallucination requirements and proper citation of sources.

## Governance

This constitution supersedes all other practices and guidelines. All changes must maintain zero hallucination guarantee and grounding in book content. Amendments require documentation of impact on grounding guarantees and explicit approval from project stakeholders. Version control ensures reproducibility from repository state.

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21