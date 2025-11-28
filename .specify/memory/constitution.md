<!--
SYNC IMPACT REPORT
==================
Constitution Version: 1.1.0 → 1.2.0 (MINOR bump: Addressed 11 gaps; added 4 new principles + critical quality gates.)

Version Bump Justification: MINOR (1.2.0)
- Reason: Expanded constitution with 4 new principles (VII-X) addressing pedagogy, hardware, feedback, cost. Enhanced documentation standards with RAG quality metrics, hallucination auditing, accessibility, Urdu translation, and content versioning. Added performance SLOs, subagents governance. No existing principles removed or redefined; expansions clarify and operationalize.

Principles Added/Enhanced:
- V. Test-First for Code, Validation-First for Content (ENHANCED: added measurable content quality targets: readability ≤grade 10, 100% accuracy review, ≥3 examples per concept)
- VII. Curriculum Structure & Prerequisite Clarity (NEW: explicit module path ROS2→Digital Twin→Isaac→VLA→Capstone, prerequisite gating)
- VIII. Hardware Assumptions & Target Learners (NEW: RTX 4070 Ti, Jetson Orin Nano, Unitree Go2/G1, cloud alternatives with disclaimers)
- IX. Learner Feedback as Governance (NEW: weekly review, routing critical/pedagogical/feature requests, feedback metrics)
- X. Minimal Operational Cost & Transparency (NEW: <$100/month cap, free tier prioritization, learner cost estimates per chapter)

Sections Added:
- Performance & Reliability Standards (RAG latency p95 <2s, personalization p95 <500ms, 99% uptime, weekly deployments)
- Content Versioning & Migration (major/minor versioning, learner notifications, backward compatibility)
- Subagents & Reusable Intelligence Governance (scope definition, versioning, integration gates, deprecation)
- Enhanced Compliance Checkpoints (weekly, chapter-end, module-end, pre-delivery with specific metrics)

Documentation Standards Expanded:
- Content: Added learning objectives, key takeaways, ≥3 examples, alt text, video transcripts, prerequisites
- Accessibility: WCAG 2.1 AA compliance, alt text for diagrams, captions for videos, RTL verification for Urdu
- Urdu Translation: Native speaker + domain expert review, technical glossary, RTL layout, code remains English
- RAG Quality & Hallucination Auditing: Mandatory source citations, ≥0.85 cosine similarity threshold, weekly 5% spot-checks, escalation protocol for hallucinations, graceful degradation

Gaps Addressed:
✅ Gap #1: Student feedback loop (Principle IX, updated compliance checkpoints)
✅ Gap #2: Vague RAG metrics (RAG Quality section: 0.85 threshold, weekly spot-checks, escalation)
✅ Gap #3: Hallucination auditing (Mandatory weekly spot-checks, escalation protocol)
✅ Gap #4: Chapter dependency graph (Principle VII: prerequisite gating)
✅ Gap #5: Content quality metrics (Principle V: readability, accuracy review, examples)
✅ Gap #6: Urdu translation standards (Documentation Standards: native speaker, glossary, RTL)
✅ Gap #7: Content accessibility (Documentation Standards: WCAG 2.1 AA, alt text, captions, transcripts)
✅ Gap #8: Content versioning (New section: versioning, migration, backward compatibility)
✅ Gap #9: Cost constraints (Principle X: <$100/month, free tier prioritization)
✅ Gap #10: Performance SLOs (New section: RAG latency, uptime, deployment frequency)
✅ Gap #11: Subagents governance (New section: scope, versioning, integration gates, deprecation)

Dependent Templates Validation:
✅ spec-template.md — Aligns with Principles I, VII (user scenarios, prerequisites)
✅ plan-template.md — Aligns with Principles I, IV, VII (architecture, structure, prerequisites)
✅ tasks-template.md — Aligns with Principle IV (chapter-based breakdown, phasing)
✅ adr-template.md — Aligns with Governance section (decisions for RAG, personalization, Urdu)
✅ phr-template.md — Aligns with Governance/feedback tracking

Deferred Items: None (all gaps resolved)

Files Modified:
- .specify/memory/constitution.md (v1.1.0 → v1.2.0 with expansions)

Suggested Commit Message:
docs: amend constitution to v1.2.0 (add principles VII-X, RAG quality gates, content versioning, subagents governance)
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Spec-Driven Development

All work begins with written specifications. Every feature, module, and learning outcome MUST be defined in a spec before implementation. Specs MUST include: user scenarios with acceptance criteria, functional and non-functional requirements, success metrics, and dependencies. Ambiguity blocks implementation; clarification happens before code.

**Rationale**: This ensures pedagogical clarity, consistent student learning outcomes, and reduces rework due to unclear requirements.

### II. Content-First, Technology-Second

Pedagogical content (learning objectives, explanations, examples) takes priority over technical implementation details. The chatbot, personalization, and translation features exist to serve the textbook content, not the reverse. Content authors decide what features are needed; engineers implement them.

**Rationale**: The primary deliverable is an excellent textbook. Technology is a means to enhance learning, not an end in itself. Students learn robotics first; infrastructure is invisible to them.

### III. Accessibility & Inclusivity Through Personalization

The textbook MUST support multiple learning paths. Students from different hardware/software backgrounds MUST be able to tailor content to their level. Translation support (Urdu and other languages) is non-negotiable for serving diverse learner populations.

**Rationale**: Physical AI is accessible only if learners from all backgrounds can engage with it. Personalization removes barriers; localization removes language barriers.

### IV. Iterative Chapter-Based Development

Development proceeds module-by-module, chapter-by-chapter, with each chapter independently deployable. A chapter is production-ready only when: spec approved, content written, RAG index built, personalization rules defined, translation complete, acceptance tests pass.

**Rationale**: This enables continuous delivery, early learner feedback, and parallel author/engineer work without blocking the entire textbook launch.

### V. Test-First for Code, Validation-First for Content

All code (backend, RAG pipeline, personalization logic) MUST be test-driven (TDD). Content MUST be validated by domain experts (roboticists, ML engineers) before integration. RAG chatbot answers MUST be grounded in actual chapter text; no hallucinations. Content MUST achieve: readability grade ≤10, 100% technical accuracy peer review, ≥3 worked examples per concept.

**Rationale**: Correctness is non-negotiable in technical education. Failing tests catch bugs; peer review catches pedagogical errors and misconceptions. Measurable content quality ensures learner success.

### VI. Minimal, Measurable Scope Creep Prevention

Features MUST deliver one of the stated hackathon objectives: (1) base textbook + deployment, (2) RAG chatbot, (3) subagents/skills, (4) authentication + signup, (5) personalization, (6) Urdu translation. Anything beyond these is out-of-scope unless explicitly prioritized. YAGNI principle enforced.

**Rationale**: The hackathon has a fixed timeline and scoring rubric. Building "nice-to-haves" ahead of core objectives risks incomplete delivery and lower final score.

### VII. Curriculum Structure & Prerequisite Clarity

The textbook follows a strict modular path: Module 1 (ROS 2: Nervous System) → Module 2 (Gazebo/Unity: Digital Twin) → Module 3 (NVIDIA Isaac: Brain) → Module 4 (VLA: Mind) → Capstone (Autonomous Humanoid). Each module has explicit prerequisites. Learners may skip forward only if personalization confirms mastery of prerequisites.

**Rationale**: Physical AI is cumulative; concepts build on prior knowledge. Clear prerequisites prevent learner frustration and reduce support burden.

### VIII. Hardware Assumptions & Target Learners

All code examples and simulation guidance assume: Primary Workstation (NVIDIA RTX 4070 Ti, 64GB RAM, Ubuntu 22.04), Edge Device (Jetson Orin Nano/NX), and target robot (Unitree Go2 quadruped or G1 humanoid). Alternative hardware paths documented in "getting started" chapter with trade-offs explicitly stated. Cloud-based labs must include latency/cost disclaimers.

**Rationale**: Transparent hardware assumptions reduce learner setup barriers and ensure reproducibility. Cloud alternatives acknowledge cost-constrained learners.

### IX. Learner Feedback as Governance

Student feedback (GitHub issues, chapter comments, survey forms) is reviewed weekly and routed: Critical bugs → immediate hotfix, Pedagogical confusion → content revision + PR, Feature requests → evaluation against Principle VI (scope). Feedback metrics tracked: completion rate, question volume, error patterns per chapter.

**Rationale**: Living textbooks must evolve with learner needs. Systematic feedback prevents stale content and improves retention.

### X. Minimal Operational Cost & Transparency

Monthly operational costs MUST NOT exceed $100 USD (including OpenAI API, Qdrant, Neon, Better-Auth, deployment). Free tiers and low-cost alternatives prioritized. All learner-facing cost estimates (cloud compute, robotics hardware, simulation software) documented per chapter. Cost transparency removes access barriers for students in resource-constrained regions.

**Rationale**: Hackathon context demands cost efficiency. Transparent pricing builds trust with students in underserved markets (Pakistan, South Asia).

## Technology Architecture & Standards

### Technology Stack

**Frontend/Content**: Docusaurus 3.x (React, TypeScript, MDX) deployed to GitHub Pages.

**Backend**: FastAPI (Python 3.11+) for RAG API and personalization service.

**Database**: Neon Serverless Postgres (connection pooling required for edge compute).

**Vector Store**: Qdrant Cloud (free tier, ~100M embeddings capacity).

**AI/Embeddings**: OpenAI (gpt-4-turbo or gpt-3.5-turbo for chat; text-embedding-3-small for RAG).

**Authentication**: Better Auth (Svelte wrapper if needed, or pure integration).

**Deployment**: GitHub Pages (frontend), Vercel/Railway/Render (API backend, ephemeral compute acceptable).

**Rationale**: This stack is open-source-friendly, cost-effective, and proven for SDD projects. Serverless reduces ops burden during a hackathon.

### Code Quality & Testing Standards

- **Python Code**: Black formatting, type hints mandatory, pytest for unit + integration tests, minimum 70% coverage for new modules.
- **TypeScript/React**: Prettier + ESLint, unit tests via Vitest, Cypress for integration tests, accessibility (WCAG 2.1 AA).
- **Database**: Migration scripts versioned in git, schema changes documented in ADRs.
- **RAG Indexing**: Chunk size validated against embedding token limits; retrieval precision (>0.8) via unit tests.

### Documentation Standards

- **Code**: JSDoc/Python docstrings for all public APIs. README in each module.
- **Content**: Every chapter has: "Learning Objectives" (start), "Key Takeaways" (end), ≥3 worked examples, alt text for ALL diagrams, transcripts for videos/demos, prerequisites listed. Readability target: grade 10 or lower (Flesch-Kincaid).
- **Accessibility**: WCAG 2.1 AA compliance for all web components. All static diagrams (PlantUML, Draw.io) MUST include alt text. All videos captioned in English; Urdu subtitles for translated content. Code terminal outputs included as text (not screenshots).
- **Urdu Translation**: Native Urdu speaker + domain expert review required. Technical terms maintained in glossary. RTL layout verified. All code examples remain English; explanations translated.
- **RAG Quality & Hallucination Auditing**:
  - MANDATORY: Every RAG response includes source citation (chapter + section).
  - Semantic similarity threshold: ≥0.85 cosine distance (validated via OpenAI embeddings).
  - Weekly spot-check: Random sample 5% of RAG responses; human reviewer validates accuracy vs. chapter text.
  - Hallucination escalation: If hallucination detected → pause RAG for that topic, investigate root cause, add guardrails/fine-tuning, re-enable only after verification.
  - Response degradation: If no relevant context found, RAG MUST respond with "I don't have information on this in the textbook. Please refer to Chapter X or ask an instructor."
- **Decisions**: Architecturally significant decisions (VLA model choice, personalization strategy, Urdu implementation approach) MUST be documented in Architecture Decision Records (ADRs).

## Development Workflow & Review

### Feature Development Lifecycle

1. **Spec**: Write `specs/<feature>/spec.md` with user scenarios, requirements, success criteria. Requires content author + tech lead approval.
2. **Plan**: Run `/sp.plan` to generate architecture, data models, test strategy. Approval gate before implementation.
3. **Tasks**: Run `/sp.tasks` to break into testable, 4-8 hour tasks. Task queue visible to team.
4. **Implementation (Red-Green-Refactor)**:
   - Red: Write failing tests.
   - Green: Implement minimal code to pass.
   - Refactor: Clean up, validate against spec.
5. **Code Review**: All PRs require 1 peer review + CI passing (tests, lint, type checks).
6. **Deployment**: Merge to `main` triggers automated deployment to staging. Manual approval for production.

### Chat Bot Quality Gate

Every RAG query result MUST:
- Return source citations (chapter + section).
- Match content within ±2% semantic similarity (validate via manual spot checks).
- Degrade gracefully if no relevant context found (return "Not found in textbook" vs. generating answer).

### Performance & Reliability Standards

- **RAG Latency**: p95 <2s, p99 <4s. Timeout: 5s max.
- **Personalization API**: p95 <500ms, p99 <1s. Timeout: 2s max.
- **Uptime**: 99% (52.6 min downtime allowed per month).
- **Deployment Frequency**: At least weekly (continuous delivery target). Blue-green deployment to minimize disruption.

### Content Versioning & Migration

- **Major Content Changes** (e.g., restructuring a section, significant corrections): Increment chapter version (e.g., "Chapter 3 v1.1 → v2.0"). Notify all active learners via email/dashboard.
- **Minor Content Changes** (typos, clarifications, example updates): No learner notification required. Auto-propagate.
- **Learner Re-engagement**: If learner is past a majorly-updated chapter, offer choice: "This chapter was updated. Review changes? Skip?" Completion credits preserved.
- **Backward Compatibility**: Deprecated code examples tagged "[LEGACY]"; updated versions shown alongside with migration guide.

### Subagents & Reusable Intelligence Governance (Bonus Objective)

- **Scope Definition**: Subagents MUST solve a single, well-defined pedagogical problem (e.g., "VLA model selection advisor", "ROS 2 debugging agent", "Gazebo simulation error resolver").
- **Versioning**: Subagents versioned independently (e.g., `vla-advisor@1.2.0`). Semantic versioning enforced.
- **Integration Gate**: Subagents integrated into textbook ONLY if:
  - Spec written and approved (define input/output/limitations).
  - Contract tests pass (predictable behavior on known inputs).
  - Performance meets standards (p95 latency <3s for advisor calls).
  - Documentation clear (when/why to use, failure modes).
- **Deprecation**: Subagent deprecated if unused for 2+ months or superseded by newer version. 30-day notice given.

### Compliance Checkpoints

- **Weekly**: Task completion, unblocked items, learner feedback volume/sentiment.
- **Chapter-End**: Content approved by domain expert, RAG index validated, tests passing, cost audit within budget, performance benchmarks met.
- **Module-End**: All chapters in module complete, accessibility audit passed, Urdu translation (if applicable) validated, subagent integrations documented.
- **Pre-Delivery**: Full textbook deployed, all bonus objectives evaluated, hallucination audits passed, learner survey >4/5 stars, cost tracking showed <$100/month.

## Governance

### Constitution as Authority

This Constitution supersedes all other development guidance. If a conflict arises between this Constitution and informal practices, the Constitution wins. When ambiguity exists, consult `/sp.clarify` to resolve.

### Amendment Process

Amendments to the Constitution MUST:
1. Be justified in writing (rationale, impact on goals, risk mitigation).
2. Receive approval from Project Lead + Tech Lead.
3. Be merged via a dedicated PR with the commit message: `docs: amend constitution to vX.Y.Z`.
4. Trigger a re-evaluation of affected specs, plans, and tasks.

### Version Numbering

- **MAJOR** (1.0 → 2.0): Removal or fundamental redefinition of a principle.
- **MINOR** (1.0 → 1.1): Addition of new principle, significant new section, or material expansion of guidance.
- **PATCH** (1.0 → 1.0.1): Clarifications, wording improvements, non-semantic refinements.

### Compliance Enforcement

- All PRs MUST reference which principle(s) they fulfill.
- Peer reviews MUST verify: tests exist, documentation is updated, no out-of-scope changes.
- CI/CD enforces: test passage, type safety, linting. Manual gates enforce spec alignment.

### Guidance Resources

- **Runtime Development**: Refer to `.specify/memory/constitution.md` (this file) and `CLAUDE.md` for Spec-Driven Development practices.
- **Clarification**: Use `/sp.clarify` to resolve ambiguities in specs, plans, or requirements.
- **Decision Logging**: Use `/sp.adr` to document architecturally significant decisions.
- **Retrospectives**: Use `/sp.phr` to record prompt history and iterate on process.

**Version**: 1.2.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
