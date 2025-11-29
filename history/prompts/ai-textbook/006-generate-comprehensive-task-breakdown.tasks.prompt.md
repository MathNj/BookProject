---
id: 006
title: Generate Comprehensive Task Breakdown for AI Textbook
stage: tasks
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: ai-textbook
branch: 001-ai-textbook
user: user
command: "/sp.tasks GOAL: Decompose the Master Plan into atomic, verifiable development tasks. CONTEXT: Monorepo structure (Frontend Docusaurus + Backend FastAPI). REQUIREMENTS: Granular tasks (no \"Build the backend\"), task grouping by phase + user story, verification checkpoints, dependency ordering."
labels: ["task-generation", "implementation", "monorepo", "granular-breakdown", "user-story-driven"]
links:
  spec: specs/001-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook/tasks.md (created, 542 lines, 97 atomic tasks organized by phase + user story)
tests:
  - Granularity check: All tasks small enough for 1-3 day execution
  - Dependency validation: Phase 2 blocking all user stories, user stories parallelizable
  - File path validation: Every task includes explicit target file path
  - Checklist format validation: All tasks follow [ID] [P?] [Story?] Description pattern
---

## Prompt

GOAL: Decompose the Master Plan into atomic, verifiable development tasks (`sp.tasks.md`).

CONTEXT:
We have a validated `sp.plan.md` that defines a Monorepo structure (Frontend `docs-website` + Backend `rag-backend`).
I need you to break this down into a granular Task List.

REQUIREMENTS FOR TASK GENERATION:

1.  **Granularity Rule:**
    - No task should be "Build the backend." (Too big).
    - Correct granularity: "Initialize Poetry project", then "Install FastAPI", then "Create main.py hello world".
    - *Why:* This prevents you (the Agent) from getting stuck or hallucinating file paths.

2.  **Task Grouping Strategy:**
    - **Group 1: Project Skeleton:** [Setup tasks]
    - **Group 2: Frontend Foundation:** Installing Docusaurus, cleaning bloat, and configuring the `docusaurus.config.ts`.
    - **Group 3: Curriculum Skeleton:** Creating the 5 empty Module folders in `/docs` with placeholder `intro.md` files.
    - **Group 4: Backend Core:** Setting up Python, Poetry, and the basic FastAPI server.
    - **Group 5: Intelligence Features:**
        - Task: Implement `rag-service.py`.
        - Task: Swizzle Docusaurus theme to add the "Chatbot" component.
        - Task: Implement the "Translate to Urdu" button logic.

3.  **Verification Checkpoints:**
    - After every major group, insert a "Validation Task".
    - Example: "Task 2.5: Run `npm start` and verify the landing page loads on localhost:3000."

4.  **Dependencies:**
    - Ensure tasks are ordered so that dependencies exist before they are used (e.g., Create the "Ask AI" API endpoint *before* building the Frontend React component that calls it).

OUTPUT:
Generate the `sp.tasks.md` file. Use standard Todo-list formatting (checkboxes) so we can track progress.

## Response snapshot

Generated comprehensive task breakdown with 97 atomic, testable tasks organized in 9 phases:

**Phase 1: Setup** (4 tasks) — Monorepo initialization, GitHub Actions template, .env setup

**Phase 2: Foundational** (23 tasks) — Frontend (Docusaurus init, i18n config, landing page, 5 modules) + Backend (Poetry, FastAPI init, Neon DB, JWT middleware, CORS, RAG infrastructure). **BLOCKING gate** - must complete before user stories.

**Phase 3: User Story 1 - RAG Chatbot** (15 tasks)
- Backend: RAG service (embed, retrieve, generate), API endpoints (/api/rag/query)
- Frontend: RAGWidget component, useRAGQuery hook, text highlighting, chapter metadata
- Verification: End-to-end tests (query → response with citation)

**Phase 4: User Story 2 - Auth & Personalization** (14 tasks)
- Backend: Auth service (register, login, JWT), Profile endpoints, Personalization service
- Frontend: SignupForm, LoginForm, useAuth hook, PersonalizeButton, profile-based intro loading
- Verification: Signup → personalized intro → verify background-appropriate content

**Phase 5: User Story 3 - Urdu Localization** (13 tasks)
- Build-time: Translation script (OpenAI), Docusaurus i18n config, RTL styling
- Frontend: LanguageSwitcher component, fallback banner (missing translations), localStorage persistence
- Verification: Toggle Urdu → RTL layout + Urdu text, code remains English

**Phase 6: User Story 4 - CI/CD Deployment** (8 tasks)
- CI/CD Pipeline: GitHub Actions workflow (lint → build → RAG ingest → benchmark → deploy)
- Validation: Chapter structure checker (frontmatter, examples, readability), RAG deployment test
- Verification: Push chapter → workflow runs → live on GitHub Pages + RAG searchable within 5 min

**Phase 7: Infrastructure** (5 tasks) — Vercel/Railway setup, Neon Postgres, Qdrant collection, CORS, monitoring

**Phase 8: Testing** (4 tasks, optional) — Unit tests (RAG, auth), integration tests, end-to-end

**Phase 9: Polish** (11 tasks) — Quickstart guide, contributor guide, API docs, deployment guide, cleanup, security audit

Task organization follows strict checklist format:
- All 97 tasks have ID (T001-T097), [P] parallelization markers, [Story] labels (US1-US4)
- Every task includes exact file path (no vague references)
- Dependencies explicitly ordered (Phase 2 before Phase 3-6, sequential within phases)

Parallel opportunities identified:
- Phase 1: All setup tasks [P]
- Phase 2: Frontend [P] with Backend [P]; within each side, multiple tasks [P]
- Phase 3-6: All user stories [P] after Phase 2 complete
- Within each story: Models [P], then services/endpoints, then integration

MVP Scope: Phases 1-3 only (18-22 hours) = RAG chatbot MVP
Full Feature: Phases 1-6 (30-38 hours) = All user stories delivered
Production: Phases 1-9 (48-60 hours) = Release-ready system

Verification checkpoints after each phase ensure independent testing of each story.

## Outcome

- ✅ Impact: Comprehensive, atomic task breakdown ready for implementation. 97 tasks span all 5 clarified features (RAG, auth, personalization, Urdu, CI/CD). Each task independently executable with clear file paths. No blockers or ambiguities remain.
- 🧪 Tests:
  - Granularity validation: All tasks 1-3 day scope (none exceed 8 hours)
  - Format validation: All 97 tasks follow [ID] [P?] [Story?] Description + filepath pattern
  - Dependency validation: Phase 2 blocking, user stories parallelizable, sequential within stories
  - File path validation: 97/97 tasks have explicit target files (no "create model" without path)
- 📁 Files:
  - specs/001-ai-textbook/tasks.md (542 lines, 97 tasks, 9 phases, execution time estimates)
- 🔁 Next prompts:
  - Begin Phase 1 (Setup) immediately - 4-hour sprint
  - Begin Phase 2 (Foundational) - 8-10 hour sprint, BLOCKING gate
  - After Phase 2, user stories US1-US4 can run in parallel (if team capacity)
  - Recommend MVP approach: Complete Phases 1-3 first (18-22 hours), validate US1 independently, then add US2-US4
- 🧠 Reflection: Task breakdown is granular and dependency-aware. Clear checkpoints enable validation at each phase. MVP-first strategy minimizes time-to-demo (can show working RAG chatbot in 2-3 days). Full feature scope achievable in 1 week with focused team. Parallel opportunities identified for 4-person team workflow.

## Evaluation notes (flywheel)

- Failure modes observed: None—all 97 tasks are atomic, no circular dependencies, clear paths.
- Graders run and results (PASS/FAIL): ✅ PASS
  - Granularity: All tasks 1-3 day scope ✅
  - Format: All 97 follow checklist pattern ✅
  - Dependencies: Phase 2 blocking, stories parallelizable ✅
  - Paths: All explicit (no vague references) ✅
  - Checkpoints: After each phase for validation ✅
- Prompt variant (if applicable): None (single generation per /sp.tasks)
- Next experiment: During implementation (red-green-refactor), track actual time spent vs. estimates (Phase 1: 4h, Phase 2: 8-10h). Adjust Phase 3-6 estimates based on actual velocity.
