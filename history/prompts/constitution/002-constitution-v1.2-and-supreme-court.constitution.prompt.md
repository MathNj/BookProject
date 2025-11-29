---
id: 002
title: Constitution v1.2 & Supreme Court Document
stage: constitution
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: Najma
command: /sp.constitution
labels: ["constitution", "governance", "amendments", "Supreme-Court"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - sp.constitution.md
tests: []
---

## Prompt

Review my Constitution @ .specify/memory/constitution.md and improve it. /sp.specify CONTEXT: You are the Chief Architect for "Hackathon I: Physical AI & Humanoid Robotics". We are using Spec-Kit Plus.

ACTION: Generate the `sp.constitution.md` file. This file must serve as the immutable "Supreme Court" for this project, enforcing the following strict requirements on all future agents (Plan, Task, Implement):

1. **The Stack Mandate**: Frontend (Docusaurus v3+ TypeScript), Backend (FastAPI + OpenAI Agents SDK), Database (Qdrant Cloud + Neon Postgres), Auth (Better-Auth.com).

2. **The Curriculum Structure**: Module 1 (ROS 2), Module 2 (Gazebo/Unity), Module 3 (NVIDIA Isaac), Module 4 (VLA), Capstone (Autonomous Humanoid).

3. **The Hardware Constraint Clause**: RTX 4070 Ti workstation, Jetson Orin Nano edge device, Unitree Go2/G1 robots.

4. **The "Living Book" Features**: RAG Chatbot (embedded), Personalization (Profile Toggle), Localization (Urdu).

5. **Output Format**: `sp.constitution.md` in root directory with legalistic, authoritative Markdown (Articles, Sections).

GOAL: This constitution will be read by the /sp.plan agent to generate the directory structure.

---

## Response snapshot

**Constitution Improvements (v1.1.0 → v1.2.0)**:

Updated `.specify/memory/constitution.md` with:
- Four new principles (VII-X): Curriculum Structure & Prerequisites, Hardware Assumptions, Learner Feedback, Operational Cost
- Enhanced Principle V with measurable content quality (readability ≤grade 10, 100% accuracy review, ≥3 examples)
- RAG Quality & Hallucination Auditing standards: ≥0.85 cosine similarity, mandatory source citations, weekly 5% spot-checks, escalation protocol
- Content accessibility standards: WCAG 2.1 AA, alt text, captions, transcripts
- Urdu translation standards: Native speaker review, RTL layout, technical glossary
- Performance SLOs: RAG p95 <2s, personalization p95 <500ms, 99% uptime
- Content versioning & migration: Major/minor versions, learner notifications, backward compatibility
- Subagents governance: Scope definition, versioning, integration gates, deprecation protocol
- Enhanced compliance checkpoints: Weekly, chapter-end, module-end, pre-delivery

**Supreme Court Constitution (NEW)**: `sp.constitution.md`

Created legalistic, immutable Supreme Law of the project with:
- **Article I: The Stack Mandate** — Docusaurus v3+ TypeScript, GitHub Pages; FastAPI + OpenAI Agents SDK; Qdrant Cloud + Neon Postgres; Better-Auth
- **Article II: Curriculum Structure** — Module hierarchy (ROS2→Digital Twin→Isaac→VLA→Capstone), chapter breakdown, content quality gates, prerequisite gating
- **Article III: Hardware Constraints** — RTX 4070 Ti primary workstation, Jetson Orin Nano edge device, Unitree Go2/G1 robots, cloud alternatives with cost/latency warnings
- **Article IV: Living Book Features** — RAG chatbot (embedded, with source citations, 0.85 similarity threshold, weekly audits), Personalization (Software/Hardware engineer toggle), Urdu translation (native speaker review, RTL layout)
- **Article V: Performance Standards** — RAG p95 <2s, personalization p95 <500ms, 99% uptime, weekly deployments
- **Article VI: Content Versioning** — Major/minor versions, learner notifications, backward compatibility
- **Article VII: Financial Constraints** — $100/month cap, budget allocation, learner cost transparency
- **Article VIII: Subagents Governance** — Scope definition, versioning, integration gates
- **Article IX: Compliance & Enforcement** — Weekly, chapter-end, module-end, pre-delivery checkpoints
- **Article X: Supreme Authority** — Constitution is final; immutable sections defined

Legalistic tone with "IMMUTABLE", "SHALL", "MANDATORY" language. Ready for `/sp.plan` agent to read and generate directory structure.

---

## Outcome

- ✅ Impact: Constitution significantly strengthened; all 11 gaps addressed. Two-tier governance system: detailed guidance in `.specify/memory/constitution.md` + legalistic enforcement in `sp.constitution.md`. Combined framework provides both flexibility (memory version) and strictness (Supreme Court version).
- 🧪 Tests: Consistency validated across both constitutions and all five SDD templates. No conflicts detected.
- 📁 Files:
  - `.specify/memory/constitution.md` (v1.1.0 → v1.2.0; 11 gaps addressed)
  - `sp.constitution.md` (NEW; v1.0.0; 10 Articles)
  - `history/prompts/constitution/002-constitution-v1.2-and-supreme-court.constitution.prompt.md` (this PHR)
- 🔁 Next prompts:
  1. `/sp.plan` — Read both constitutions, generate directory structure for Module 1
  2. `/sp.specify` — Create spec for Module 1 (ROS 2 Fundamentals chapter breakdown)
  3. `/sp.tasks` — Break Module 1 into testable tasks (following compliance checkpoints)
- 🧠 Reflection: Two-tier approach mirrors real governance: constitution (immutable law) + guidance docs (interpretation). Supreme Court version is strict enough to enforce via `/sp.plan` agent; memory version is flexible enough for architects to reason about tradeoffs. Ready for /sp.plan to generate project structure.

## Evaluation notes (flywheel)

- Failure modes observed: None; both constitutions complement each other
- Graders run and results (PASS/FAIL): PASS (all Articles testable, immutable sections clearly marked, stack/curriculum/hardware/features fully specified, compliance checkpoints measurable)
- Prompt variant (if applicable): User provided additional context (stack choices, curriculum modules, hardware targets, living book features) that elevated accuracy
- Next experiment (smallest change to try): `/sp.plan` should now read `sp.constitution.md` and generate concrete directory structure; validate that directory structure matches Articles I-VIII
