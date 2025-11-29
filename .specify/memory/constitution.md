
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

Development proceeds module-by-module, chapter-by-chapter. Chapters are independently versioned and deployable to staging. A chapter is **production-ready** when ALL of the following are confirmed:

- Content spec approved by domain lead + pedagogical reviewer
- Content written + peer-reviewed for clarity and accuracy
- RAG index built and validated (5% spot-check passed)
- Personalization rules defined and tested
- Translation (if applicable) reviewed by native speaker + domain expert
- Acceptance tests passing (unit + integration + accessibility)
- Cost audit complete (per-chapter resource usage logged)

Deployment to production requires an additional gate: module-level validation must pass (all chapters in the module verified for coherence and prerequisite correctness).

**Rationale**: This enables continuous delivery, early learner feedback, and parallel author/engineer work without blocking the entire textbook launch. Clear gates prevent premature or incomplete releases.

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

Student feedback is reviewed weekly via designated channels: GitHub discussions (bugs, technical issues), chapter inline comments (content clarity), weekly survey form (overall experience, satisfaction). Routing protocol:

- **Critical Bugs** (deployment broken, security issue, RAG hallucination): Hotfix within 24h, learner notified
- **Pedagogical Confusion** (concept unclear, example wrong, missing prerequisite): Root cause analysis, content revision PR, affected learners notified of updates
- **Feature Requests**: Evaluated against Principle VI (scope alignment). Out-of-scope requests logged and triaged for future work.

Feedback metrics tracked per chapter: completion rate, avg time spent, questions volume, error patterns, sentiment (survey). Dashboard reviewed weekly by content lead + tech lead. If completion rate drops >10% on a chapter, trigger content review within 5 business days.

**Rationale**: Living textbooks must evolve with learner needs. Systematic feedback prevents stale content and improves retention. Clear channels and routing ensure fast response to critical issues.

### X. Minimal Operational Cost & Transparency

Monthly operational costs MUST NOT exceed $100 USD (OpenAI API, Qdrant, Neon, Better-Auth, deployment). Free tiers and low-cost alternatives prioritized. Cost tracking mechanism:

- **Daily**: API usage logged (token counts, embeddings, API calls) in cost dashboard.
- **Weekly**: Cost review conducted by tech lead. If cumulative cost exceeds $60 (60% of budget), optimize immediately (batch API calls, reduce model complexity, consolidate queries).
- **Monthly**: Full audit. If monthly cost >$100, post-mortem conducted and cost-reduction PR submitted within 5 days.

All learner-facing cost estimates (cloud compute, robotics hardware, simulation software) documented per chapter. Hardware cost estimates updated quarterly. Cost transparency removes access barriers for students in resource-constrained regions.

**Rationale**: Hackathon context demands cost efficiency. Transparent pricing builds trust with students in underserved markets (Pakistan, South Asia). Continuous monitoring prevents budget overruns.

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

### Ownership & Escalation

| Role | Responsibility | Escalation Path |
|------|---|---|
| **Content Lead** | Principle II (pedagogy), Principle V (content validation), Principle IX (feedback routing) | Tech Lead if principle violated; Project Lead for scope conflicts |
| **Tech Lead** | Principle I (spec), Principle IV (production gates), Principle X (cost monitoring) | Project Lead if principles violated; architecture review required |
| **Accessibility Lead** | Principle III (WCAG 2.1 AA), content standards (alt text, captions) | Content Lead; block chapter if not met |
| **Project Lead** | Principle VI (scope), amendments, final arbitration on conflicts | Steering committee (if applicable) |

**Escalation Triggers**:
- Cost exceeds $60 in a week → tech lead investigates within 24h
- Hallucination detected in RAG → pause feature, root cause analysis, no re-enable without explicit sign-off
- Content completion rate drops >10% → content lead conducts review within 5 business days
- Principle violation in PR → rejected; must resubmit aligned with constitution

### Compliance Checkpoints

| Checkpoint | Owner | Success Criteria | Failure Action |
|---|---|---|---|
| **Weekly** | Content + Tech Lead | All tasks tracked, no blockers >3 days, feedback volume monitored | Daily standup until blockers cleared |
| **Chapter-End** | Content Lead + Tech Lead | Spec approved, content peer-reviewed, RAG validated (5% spot-check), tests ≥70% coverage, accessibility audit passed, cost logged | Return to development; retest before retry |
| **Module-End** | Content Lead + Tech Lead + Accessibility Lead | All chapters production-ready, module-level prerequisite graph validated, Urdu translation (if applicable) verified by native speaker, subagents documented | Hold module release; remediate in next iteration |
| **Pre-Delivery** | Project Lead | Full textbook deployed to production, all bonus objectives rated, hallucination audits passed (zero hallucinations in sample), learner NPS ≥4/5, monthly cost <$100, deployment logs clean | Delay release; address blockers; revalidate |

### Technical Debt & Refactoring Prioritization

Technical debt and refactoring work is prioritized against new feature development using this framework:

**Tier 1 (Urgent)**: Blocks content delivery or violates a principle.
- Examples: Security vulnerability, RAG system broken, test coverage <50%, cost exceeding budget
- Action: Hotfix or sprint allocation; blocks new features until resolved

**Tier 2 (High)**: Increases dev velocity or improves learner experience materially.
- Examples: Slow CI/CD (>10 min), repetitive manual testing, unclear architecture, high latency (>2s)
- Action: Allocate 10–20% of sprint capacity; prioritize at planning stage

**Tier 3 (Low)**: Nice-to-have improvements that don't affect delivery.
- Examples: Code style cleanup, minor test refactors, documentation wording
- Action: Opportunistic work; include in PR if touching related code; never block on this

**Decision Rule**: If technical debt appears in an active PR, assess its tier:
- Tier 1: Block and fix.
- Tier 2: Create linked issue; fix in next sprint if capacity permits.
- Tier 3: Log for future; don't block current work.

## Governance

### Constitution as Authority

This Constitution supersedes all other development guidance. If a conflict arises between this Constitution and informal practices, the Constitution wins. When ambiguity exists, consult `/sp.clarify` to resolve.

### Amendment Process

All amendments to the Constitution require explicit approval and follow semantic versioning:

**Amendment Steps:**
1. Author writes justification document: rationale, impact on active specs/plans, risk mitigation, examples of how the change improves execution.
2. Submit as PR with detailed description (not inline edits). Include section "Breaking Changes?" to clarify if active specs/plans must be revised.
3. Review gate: Project Lead + Tech Lead approve. If amendment affects multiple principles or requires active work to re-align specs, Steering Committee approval required.
4. Merge with commit: `docs: amend constitution to vX.Y.Z`
5. **Re-evaluation trigger**: If amendment is MAJOR or affects active modules, affected specs/plans re-evaluated within 5 business days. Non-blocking but documented.

**Breaking Changes Protocol** (when amendment materially changes a principle):
- Identify all active specs/plans/tasks referencing the changed principle.
- Issue 5-day notice to affected teams with transition guidance.
- Provide migration path (e.g., "Old approach still valid through Month X; new approach preferred from Date Y").

### Version Numbering

- **MAJOR** (1.0 → 2.0): Removal or fundamental redefinition of a principle. Or: Addition of new, conflicting constraint. Requires steering approval.
- **MINOR** (1.0 → 1.1): Addition of new principle, major new section, or significant clarification/operationalization of existing principle. Requires Project + Tech Lead approval.
- **PATCH** (1.0 → 1.0.1): Clarifications, wording, non-semantic refinements, updated examples. Tech Lead approval only.

### Compliance Enforcement

- All PRs MUST reference which principle(s) they fulfill.
- Peer reviews MUST verify: tests exist, documentation is updated, no out-of-scope changes.
- CI/CD enforces: test passage, type safety, linting. Manual gates enforce spec alignment.

### Guidance Resources

- **Runtime Development**: Refer to `.specify/memory/constitution.md` (this file) and `CLAUDE.md` for Spec-Driven Development practices.
- **Clarification**: Use `/sp.clarify` to resolve ambiguities in specs, plans, or requirements.
- **Decision Logging**: Use `/sp.adr` to document architecturally significant decisions.
- **Retrospectives**: Use `/sp.phr` to record prompt history and iterate on process.

**Version**: 1.2.1 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29
