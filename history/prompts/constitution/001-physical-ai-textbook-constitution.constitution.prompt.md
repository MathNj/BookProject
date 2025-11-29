---
id: 001
title: Physical AI Textbook Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: master
user: Najma
command: /sp.constitution
labels: ["constitution", "governance", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: []
---

## Prompt

Act as the Chief Architect and Lead Technical Author for the "Physical AI & Humanoid Robotics Textbook" project. We are using Spec-Kit Plus and Claude Code to build this.

Write the official `project_constitution.md` file. This file will serve as the single source of truth for all future agents and developers working on this repo.

Write a Constitution for the project.""" Hackathon I: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course

The future of work will be a partnership between people, intelligent agents (AI software), and robots. This shift won't necessarily eliminate jobs but will change what humans do, leading to a massive demand for new skills. We have already written a book on AI agents. Therefore, we want you to write a textbook to teach a course in Physical AI & Humanoid Robotics (The course details are documented below).

Excel in the Hackathon and Launch Your Journey as an AI Startup Founder 🚀

We've recently launched Panaversity (panaversity.org), an initiative focused on teaching cutting-edge AI courses. Alongside this, we're working on publishing our first book, which you can explore at ai-native.panaversity.org. Our next milestone is to build a portal where authors can create AI-native technical textbooks, and readers can easily access and learn from them using AI Agents. We also plan to publish O/A Level, Science, Engineering, and Medical AI-native books to support students and professionals across disciplines. If you perform well in this hackathon, you may be invited for an interview to join the Panaversity core team and potentially step into the role of a startup founder within this growing ecosystem. You will get a chance to work with Panaversity founders Zia, Rehan, Junaid, and Wania and become the very best. You may also get a chance to teach at Panaversity, PIAIC, and GIAIC.

[Full hackathon requirements and course details provided in user input...]

## Response snapshot

**Constitution Created**: Physical AI & Humanoid Robotics Textbook Constitution v1.1.0

**Six Core Principles Established**:
1. Spec-Driven Development — All work begins with written specs
2. Content-First, Technology-Second — Pedagogy over infrastructure
3. Accessibility & Inclusivity Through Personalization — Support multiple learning paths with Urdu translation
4. Iterative Chapter-Based Development — Module-by-module, independently deployable chapters
5. Test-First for Code, Validation-First for Content — TDD for code, peer review for pedagogy
6. Minimal, Measurable Scope Creep Prevention — Focus on six hackathon objectives only

**Technology Architecture**: Docusaurus (frontend) + FastAPI (backend) + Neon Postgres + Qdrant Cloud + OpenAI. Deployed to GitHub Pages + Vercel/Railway.

**Development Workflow**: Spec → Plan → Tasks → Red-Green-Refactor → Code Review → Deployment

**Governance**: Constitution supersedes other guidance. Amendments require justification + approval. Version scheme: MAJOR/MINOR/PATCH based on scope of change.

**Dependent Templates Validated**: All five SDD templates (spec, plan, tasks, adr, phr) already aligned with constitution principles.

## Outcome

- ✅ Impact: Constitution created as single source of truth for project governance, principles, technology stack, and development workflow. Enables consistent decision-making across future features and specs.
- 🧪 Tests: Template consistency validation performed; all five dependent templates (spec, plan, tasks, adr, phr) verified to align with principles. No breaking changes identified.
- 📁 Files:
  - `.specify/memory/constitution.md` (created with Sync Impact Report, 1.1.0)
  - `history/prompts/constitution/001-physical-ai-textbook-constitution.constitution.prompt.md` (this PHR)
- 🔁 Next prompts:
  1. `/sp.specify` for Module 1 (ROS 2 Fundamentals) — base textbook feature
  2. `/sp.plan` to design RAG chatbot architecture
  3. `/sp.tasks` for chapter-by-chapter implementation breakdown
- 🧠 Reflection: Constitution successfully captures the dual mission (world-class pedagogy + SDD engineering) and scope constraints (6 hackathon objectives). Principles are declarative and testable. Ready to cascade down to specs.

## Evaluation notes (flywheel)

- Failure modes observed: None; template-based creation with full context alignment
- Graders run and results (PASS/FAIL): PASS (all principles testable, no unresolved placeholders, version semantic correct)
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Kick off Module 1 spec (/sp.specify) with Principle I & IV focus
