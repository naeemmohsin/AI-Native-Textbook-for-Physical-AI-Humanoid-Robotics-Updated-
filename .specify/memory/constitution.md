<!--
Sync Impact Report
==================
Version change: 0.0.0 → 1.0.0 (MAJOR - initial constitution ratification)
Modified principles: N/A (initial creation)
Added sections:
  - Core Principles (5 principles)
  - Technical Stack & Standards
  - Development Workflow
  - Governance
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ No updates needed (Constitution Check section is generic)
  - .specify/templates/spec-template.md: ✅ No updates needed (technology-agnostic)
  - .specify/templates/tasks-template.md: ✅ No updates needed (structure-agnostic)
Follow-up TODOs: None
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Workflow

All development MUST follow the Spec-Kit Plus workflow. Features begin with specifications (`/sp.specify`), progress through planning (`/sp.plan`), task generation (`/sp.tasks`), and implementation (`/sp.implement`). No code changes without a corresponding spec. This ensures traceability, consistent quality, and clear documentation of intent before implementation.

### II. Technical Accuracy

All content MUST be sourced from official documentation and authoritative references. The book and chatbot responses MUST NOT contain hallucinated or unverified information. Every technical claim requires citation or verification against primary sources. When official sources conflict, document the discrepancy and prefer the most recent authoritative source.

### III. Clear Developer-Focused Writing

Content MUST be written for developers with clarity and precision. Use concrete examples over abstract explanations. Prefer code samples that are complete and runnable. Avoid jargon without definition. Structure content progressively from fundamentals to advanced topics. Each section MUST have clear learning objectives and practical takeaways.

### IV. Reproducible Setup & Development

All code samples, configurations, and setup instructions MUST be reproducible. Environment setup MUST be documented step-by-step. Dependencies MUST be pinned to specific versions. The entire project MUST be buildable from a clean clone with documented commands. No implicit assumptions about the reader's environment.

### V. RAG Chatbot Grounding

The embedded chatbot MUST ground all responses exclusively in book content or user-selected text. Responses MUST NOT include information outside the provided context. When the chatbot cannot answer from available content, it MUST clearly state this limitation rather than fabricate information. Citation of source sections is required for all substantive responses.

## Technical Stack & Standards

**Documentation Platform**: Docusaurus
**Deployment**: GitHub Pages
**Source Control**: GitHub
**AI/RAG Stack**: OpenAI Agents SDK, ChatKit
**Backend**: FastAPI
**Database**: Neon Postgres (relational), Qdrant Cloud (vector store)

**Code Standards**:
- All code MUST be well-documented with inline comments for complex logic
- All code samples in the book MUST be tested and runnable
- API endpoints MUST have OpenAPI documentation
- Environment variables MUST be documented in `.env.example`
- Secrets MUST NOT be committed; use `.env` and document in setup guides

**Testing Requirements**:
- All backend endpoints MUST have integration tests
- RAG retrieval accuracy MUST be validated against known queries
- Book build MUST pass without errors before deployment

## Development Workflow

**Branch Strategy**: Feature branches from `master`, merged via PR after review
**Commit Standards**: Conventional commits with clear, descriptive messages
**PR Requirements**:
- Linked to spec or task
- Passing CI checks
- Self-review completed

**Documentation Updates**:
- Code changes MUST include corresponding documentation updates
- ADRs MUST be created for significant architectural decisions
- PHRs MUST be created for all development interactions

**Quality Gates**:
- Docusaurus build passes
- No broken links in documentation
- All code samples execute successfully
- RAG chatbot responds accurately to test queries

## Governance

This constitution is the authoritative source for project standards and practices. All contributors MUST adhere to these principles. Deviations require explicit justification documented in an ADR.

**Amendment Process**:
1. Propose amendment via PR with rationale
2. Document impact on existing specs and code
3. Update version following semantic versioning:
   - MAJOR: Breaking changes to principles or governance
   - MINOR: New principles or significant expansions
   - PATCH: Clarifications and non-semantic refinements
4. Update dependent templates if affected

**Compliance Review**:
- All PRs MUST verify adherence to constitution principles
- Constitution Check in `plan.md` MUST pass before implementation
- Violations MUST be justified in Complexity Tracking section

**Runtime Guidance**: Refer to `CLAUDE.md` for agent-specific development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
