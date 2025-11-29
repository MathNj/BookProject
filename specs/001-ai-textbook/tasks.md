---
description: "Task breakdown for Physical AI & Humanoid Robotics Textbook (001-ai-textbook)"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Specification from `specs/001-ai-textbook/spec.md` with 5 clarified architectural decisions
**Prerequisites**: Monorepo structure (Frontend: Docusaurus on GitHub Pages, Backend: FastAPI on Vercel/Railway)
**Architecture**: Static frontend + serverless backend + RAG pipeline
**Tests**: Optional - include if team requests TDD approach

**Organization**: Tasks grouped by user story (P1/P2) to enable independent development and testing

---

## Phase 1: Project Setup (Shared Infrastructure)

**Purpose**: Initialize monorepo structure and shared configuration

- [ ] T001 Create monorepo root structure: `/docs-website` (frontend), `/rag-backend` (backend), `/.github/workflows`
- [ ] T002 [P] Initialize `.github/workflows/build-deploy.yml` for GitHub Actions CI/CD pipeline
- [ ] T003 [P] Create `.env.example` with required environment variables (OpenAI key, Qdrant URL, Neon connection string, Better-Auth secrets)
- [ ] T004 Create root `README.md` with project overview, hardware requirements, and quickstart

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST complete before user story work begins

**⚠️ CRITICAL**: All tasks in this phase must complete before user stories can start

### Frontend Foundation

- [ ] T005 Initialize Docusaurus v3+ project in `/docs-website`: `npm init -yes && npx create-docusaurus@latest . classic`
- [ ] T006 [P] Configure `docusaurus.config.ts`: Set site title, URL, baseUrl for GitHub Pages deployment
- [ ] T007 [P] Configure Docusaurus i18n plugin (`@docusaurus/preset-classic` + locale configuration) for English/Urdu support
- [ ] T008 [P] Setup CSS modules and RTL layout support in `docusaurus.config.ts` (for Urdu RTL toggle)
- [ ] T009 [P] Create landing page skeleton: `docs-website/src/pages/index.tsx` with hero section + CTA button
- [ ] T010 Create curriculum module structure: `/docs-website/docs/01-nervous-system`, `/02-digital-twin`, `/03-robot-brain`, `/04-the-mind`, `/05-capstone`
- [ ] T011 [P] Create placeholder `intro.md` in each module directory with module title and description
- [ ] T012 Setup `docusaurus.config.ts` to expose custom hook for client-side language toggle (for Urdu switch)

### Backend Foundation

- [ ] T013 Initialize Python project in `/rag-backend`: `poetry init --name rag_backend --python ^3.11`
- [ ] T014 [P] Install FastAPI core dependencies: `poetry add fastapi uvicorn python-multipart`
- [ ] T015 [P] Install data/auth dependencies: `poetry add sqlalchemy psycopg2-binary better-auth pydantic pydantic-settings`
- [ ] T016 [P] Install RAG dependencies: `poetry add openai qdrant-client python-dotenv`
- [ ] T017 Create `/rag-backend/src/main.py` with FastAPI app initialization + CORS configuration (allow GitHub Pages domain)
- [ ] T018 [P] Create `/rag-backend/src/config.py` with environment variable loading (OpenAI key, Qdrant URL, Neon DSN, auth secrets)
- [ ] T019 [P] Create `/rag-backend/src/database.py` with Neon Postgres connection pool (SQLAlchemy)
- [ ] T020 Create `/rag-backend/src/middleware/auth.py` with JWT verification middleware (integrates with Better-Auth)
- [ ] T021 [P] Create base response model in `/rag-backend/src/schemas/responses.py` (error handling, success response format)

### RAG Infrastructure

- [ ] T022 Create `/rag-backend/src/services/qdrant_client.py` wrapper for Qdrant Cloud API (initialize, upsert, search operations)
- [ ] T023 Create `/rag-backend/src/services/openai_client.py` wrapper for OpenAI Embeddings API + Token counting
- [ ] T024 Create `/rag-backend/src/models/rag.py` with SQLAlchemy RAGEmbedding model (chapter_id, section_id, similarity_score, text)

### CI/CD RAG Integration

- [ ] T025 Create `/rag-backend/scripts/ingest_chapters.py` Python script: Read Docusaurus markdown → extract frontmatter/sections → generate embeddings → upsert to Qdrant
- [ ] T026 Update `/.github/workflows/build-deploy.yml`: Add RAG ingestion step (run after Docusaurus build succeeds, trigger `scripts/ingest_chapters.py`)
- [ ] T027 [P] Add RAG ingestion error handling to workflow: Log errors, block deployment if ingestion fails (author receives error email)

**Checkpoint**: Foundation infrastructure ready - can now proceed to user story implementation in parallel

---

## Phase 3: User Story 1 - Student Asks Context-Aware Questions (Priority: P1) 🎯 MVP

**Goal**: Students can open the textbook, highlight text, ask the RAG chatbot questions, and receive grounded answers from chapter content.

**Independent Test**:
1. Frontend: Open chapter, click "Ask the Book" widget, type question, verify response appears with source citation
2. Backend: Call `POST /api/rag/query` with question + chapter context, verify response includes `source_citation` field with chapter + section
3. Integration: End-to-end flow from frontend widget → FastAPI endpoint → OpenAI embedding → Qdrant retrieval → response generation

### Implementation: RAG Backend

- [ ] T028 [P] [US1] Create `/rag-backend/src/models/chapter.py` SQLAlchemy model: id, title, module, content_hash, version, last_indexed_at
- [ ] T029 [P] [US1] Create `/rag-backend/src/schemas/rag.py` Pydantic models:
  - `QueryRequest`: { question: str, chapter_id: str, highlighted_text?: str }
  - `RAGResponse`: { answer: str, source_citation: str, confidence: float, retrieved_sections: List[str] }
- [ ] T030 [US1] Create `/rag-backend/src/services/rag_service.py` main RAG logic:
  - `embed_query()`: Call OpenAI to embed user question
  - `retrieve_context()`: Query Qdrant with cosine similarity ≥0.85, return top 3 sections
  - `generate_response()`: Call OpenAI with prompt template + retrieved context → generate answer
  - `check_hallucination()`: Verify response is grounded in retrieved sections (basic semantic check)
- [ ] T031 [US1] Create `/rag-backend/src/endpoints/rag.py` FastAPI router:
  - `POST /api/rag/query`: Accept QueryRequest → call rag_service → return RAGResponse with source citations
  - `GET /api/rag/health`: Health check for RAG service (test Qdrant + OpenAI connectivity)
  - Add JWT auth middleware to all endpoints
- [ ] T032 [US1] Add request validation in `rag.py`: Max question length (500 chars), rate limiting (10 requests/minute per user to control costs)
- [ ] T033 [US1] Add logging in `rag_service.py`: Log each query, embedding cost, retrieval results, response generation time
- [ ] T034 [US1] Create `/rag-backend/tests/integration/test_rag_query.py`:
  - Test: Query with in-scope question → verify answer + citation
  - Test: Query with out-of-scope question → verify "I don't have info" response
  - Test: Query with multiple relevant sources → verify highest confidence section cited

### Implementation: RAG Frontend (Docusaurus)

- [ ] T035 [P] [US1] Create `/docs-website/src/components/RAGWidget.tsx` React component:
  - Floating widget (fixed position, bottom-right)
  - Text input + submit button
  - Response display with streaming text (OpenAI streaming API)
  - Source citation badge (chapter + section)
  - Loading state + error handling
- [ ] T036 [US1] Create `/docs-website/src/hooks/useRAGQuery.ts` custom hook:
  - `POST /api/rag/query` via fetch (handle CORS, JWT token from localStorage)
  - Manage loading/error states
  - Extract chapter context from current page metadata
- [ ] T037 [US1] Implement text highlight integration in RAGWidget:
  - Listen for text selection on page
  - Pass `highlighted_text` to RAG query (for context)
  - Add "Explain this" button in highlight popup
- [ ] T038 [US1] Add RAGWidget to Docusaurus layout: Modify theme wrapper to include widget on all chapter pages
- [ ] T039 [US1] Add chapter metadata to Markdown frontmatter (examples in module 1): chapter_id, chapter_title, module, sections (for RAG context)

### Verification: User Story 1

- [ ] T040 [US1] Manual test: Open local Docusaurus (npm start) + highlight ROS 2 code snippet + ask "What does this do?" → verify RAG response with citation
- [ ] T041 [US1] End-to-end test: Deploy frontend to GitHub Pages + backend to Vercel/Railway → test from live URLs
- [ ] T042 [US1] Performance test: Load test `/api/rag/query` with 10 concurrent requests → verify <5s p95 latency

**Checkpoint**: User Story 1 complete and independently testable. RAG chatbot is fully functional.

---

## Phase 4: User Story 2 - Student Signs Up & Receives Personalized Content (Priority: P1)

**Goal**: Students can create accounts, provide background preference (hardware/software), and see chapter introductions personalized to their background.

**Independent Test**:
1. Signup: Fill signup form (email, password, background), verify user created in Neon database
2. Auth: Login with email/password, verify JWT token returned and stored in localStorage
3. Personalization: Fetch chapter intro → verify it's rewritten for the user's background profile
4. Update profile: Click "Personalize" button, change background → verify chapter content refreshes with new intro

### Implementation: Authentication Backend

- [ ] T043 [P] [US2] Create `/rag-backend/src/models/user.py` SQLAlchemy model:
  - id, email, password_hash (encrypted), background (enum: "hardware", "software", "balanced"), created_at, updated_at
- [ ] T044 [P] [US2] Create `/rag-backend/src/services/auth_service.py`:
  - `register_user()`: Hash password, save to Neon, return JWT token
  - `login_user()`: Verify email/password → return JWT token
  - `verify_token()`: Decode JWT, return user_id (used by middleware)
  - Integrate better-auth for password hashing + JWT generation
- [ ] T045 [US2] Create `/rag-backend/src/endpoints/auth.py` FastAPI router:
  - `POST /api/auth/signup`: Accept (email, password, background) → call auth_service → return JWT + user_id
  - `POST /api/auth/login`: Accept (email, password) → return JWT + user_id
  - `POST /api/auth/logout`: Invalidate JWT (revoke token list in Redis if needed)
- [ ] T046 [US2] Create `/rag-backend/src/endpoints/profile.py` FastAPI router:
  - `GET /api/profile`: Return current user profile (background, email) - requires JWT
  - `PUT /api/profile`: Update user profile (background) - requires JWT
  - Add validation: background must be one of [hardware, software, balanced]

### Implementation: Personalization Service

- [ ] T047 [US2] Create `/rag-backend/src/models/personalization.py` SQLAlchemy model:
  - user_id, chapter_id, intro_variant (string: cached variant for this user + chapter combo)
- [ ] T048 [US2] Create `/rag-backend/src/services/personalization_service.py`:
  - `get_personalized_intro()`: Look up user background → select intro template → return personalized text
  - `cache_intro()`: Store computed intro variant in Personalization table for future requests (500ms p95 SLA)
- [ ] T049 [US2] Create `/rag-backend/src/endpoints/personalization.py` FastAPI router:
  - `GET /api/personalization/intro/:chapter_id`: Return personalized intro (or default if not signed in) - requires optional JWT
  - `POST /api/personalization/refresh`: Clear cached intros for user (called after profile update)

### Implementation: Personalization Frontend

- [ ] T050 [P] [US2] Create `/docs-website/src/components/SignupForm.tsx` React component:
  - Form fields: email, password, background (radio buttons: hardware/software/balanced)
  - Submit → `POST /api/auth/signup` → store JWT in localStorage
  - Redirect to first chapter after signup
- [ ] T051 [P] [US2] Create `/docs-website/src/components/LoginForm.tsx` React component:
  - Form fields: email, password
  - Submit → `POST /api/auth/login` → store JWT in localStorage
- [ ] T052 [US2] Create `/docs-website/src/hooks/useAuth.ts` custom hook:
  - `useAuth()`: Return current user (from JWT), isLoggedIn, logout()
  - Persist JWT in localStorage, auto-restore on page load
- [ ] T053 [US2] Create `/docs-website/src/hooks/usePersonalization.ts` custom hook:
  - `getPersonalizedIntro()`: Call `GET /api/personalization/intro/:chapter_id` with JWT
  - `updateProfile()`: Call `PUT /api/profile` + trigger refresh
- [ ] T054 [US2] Add login/signup links to landing page header (`docs-website/src/pages/index.tsx`)
- [ ] T055 [US2] Implement "Personalize" button on chapter pages:
  - Component: `/docs-website/src/components/PersonalizeButton.tsx`
  - Click → open profile modal → radio button to select new background → save → refresh chapter intro
- [ ] T056 [US2] Modify chapter layout to load personalized intro:
  - Add usePersonalization hook to chapter component
  - Fetch intro on page load (with JWT if logged in)
  - Display personalized intro instead of default

### Verification: User Story 2

- [ ] T057 [US2] Manual test: Signup with "software" background → verify chapter 1 intro emphasizes code
- [ ] T057 [US2] Manual test: Signup with "hardware" background → verify chapter 1 intro emphasizes robotics first
- [ ] T058 [US2] Manual test: Update profile to different background → verify chapter intro re-renders with new variant
- [ ] T059 [US2] Manual test: Logout → verify chapter reverts to default intro
- [ ] T060 [US2] Performance test: Profile update → chapter refresh <2s

**Checkpoint**: User Story 2 complete. Authentication + personalization fully functional, independently testable from US1.

---

## Phase 5: User Story 3 - Student Reads in Urdu (Priority: P1)

**Goal**: Students can toggle to Urdu language, read all content in Urdu (except code), and see RTL layout.

**Independent Test**:
1. Click language toggle button → page language switches to Urdu
2. Verify all text content is Urdu, code remains English
3. Verify RTL layout (text flows right-to-left)
4. Toggle back to English → verify English content restored
5. Hard refresh → verify language preference persists (from localStorage)

### Pre-Build: Urdu Content Preparation

- [ ] T061 Create `/docs-website/scripts/generate_urdu_translations.js` script:
  - Read English Markdown files
  - Extract content (skip code blocks, keep frontmatter)
  - Call OpenAI API for translation (batch mode to control costs)
  - Save output to `.ur.md` files alongside English
  - Placeholder: Manual review required (CI gate step)

### Implementation: Build-Time Urdu Integration

- [ ] T062 Update `.github/workflows/build-deploy.yml` to add Urdu generation step:
  - Run `npm run generate-urdu` (calls script T061)
  - Output: Multiple `.ur.md` files in docs directories
  - Requires Docusaurus rebuild with i18n plugin enabled
- [ ] T063 [P] Configure Docusaurus i18n in `docusaurus.config.ts`:
  - Add locale: "ur" with RTL direction
  - Set base URL locale path: /docs (English), /ur/docs (Urdu)
  - Configure language switcher in theme
- [ ] T064 Create `/docs-website/src/theme/LanguageSwitcher.tsx` component:
  - Dropdown: English / اردو
  - On change: Update locale in Docusaurus, trigger page re-render
  - Store preference in localStorage (persist across sessions)
- [ ] T065 Create `/docs-website/src/css/urdu.css` with RTL styling:
  - `[data-locale="ur"]` { direction: rtl; text-align: right; }
  - Applied to body when Urdu active
- [ ] T066 Integrate LanguageSwitcher into Docusaurus navbar: Add to `/docs-website/docusaurus.config.ts` navbar items

### Implementation: Fallback Handling

- [ ] T067 [US3] Create `/docs-website/src/components/UrduTranslationStatus.tsx` banner component:
  - Display if `.ur.md` file missing for current chapter
  - Show: "Urdu translation coming soon. Viewing English version."
  - Auto-hide if user switches back to English
- [ ] T068 [US3] Add logic to chapter renderer:
  - Check if `.ur.md` exists for current chapter
  - If not, render English + show banner (T067)
  - If yes, render Urdu content

### Verification: User Story 3

- [ ] T069 [US3] Manual test: Click "اردو" button → verify page renders in Urdu, RTL layout
- [ ] T070 [US3] Manual test: Verify code blocks remain English when in Urdu mode
- [ ] T071 [US3] Manual test: Reload page → verify Urdu preference persists (from localStorage)
- [ ] T072 [US3] Manual test: Chapter without translation → verify banner appears, English text shown
- [ ] T073 [US3] Performance test: Language toggle <100ms page re-render

**Checkpoint**: User Story 3 complete. Urdu localization fully functional, RTL support working.

---

## Phase 6: User Story 4 - Author Deploys Chapter (Priority: P2)

**Goal**: Authors can write chapters in Markdown, commit to repo, and trigger automated deployment (build, RAG indexing, GitHub Pages).

**Independent Test**:
1. Create new chapter file: `/docs-website/docs/01-nervous-system/05-advanced-ros.md`
2. Commit to main branch
3. Verify GitHub Actions workflow runs: linting → build → RAG ingestion → deploy
4. Verify chapter live on GitHub Pages within 5 minutes
5. Verify chapter searchable via RAG chatbot

### Implementation: CI/CD Pipeline

- [ ] T074 Create `.github/workflows/build-deploy.yml` workflow (expanded from foundational phase T002):
  - Trigger: On push to `main` branch
  - Step 1: Lint Markdown (use `markdownlint`)
  - Step 2: Build Docusaurus (`npm run build`)
  - Step 3: Generate Urdu translations (run T061 script, gate on manual review)
  - Step 4: Ingest chapters to RAG (run `rag-backend/scripts/ingest_chapters.py`)
  - Step 5: Run performance benchmarks (measure build time, RAG latency)
  - Step 6: Deploy frontend to GitHub Pages (`npm run deploy`)
- [ ] T075 Create `/docs-website/scripts/validate_chapter_structure.js`:
  - Check chapter Markdown: Has frontmatter with chapter_id, title, learning_objectives, key_takeaways
  - Verify ≥3 examples present
  - Verify readability (Flesch-Kincaid ≤10th grade) using npm package
  - Run in CI before build step (fail if not met)

### Implementation: Deployment Monitoring

- [ ] T076 Create `/rag-backend/scripts/test_rag_deployment.py`:
  - After deployment, test RAG on newly indexed chapter
  - Query: "Explain [chapter topic]" → verify response includes new chapter citation
  - Log test results to GitHub Actions output
- [ ] T077 Add deployment notifications in `.github/workflows/build-deploy.yml`:
  - On success: Comment on PR with "Deployment successful - live at [URL]"
  - On RAG failure: Comment on PR with error logs, block deployment
  - On Urdu review gate: Pause workflow, notify author for manual review

### Verification: User Story 4

- [ ] T078 [US4] Create test chapter: `/docs-website/docs/01-nervous-system/test-chapter.md` with required sections
- [ ] T079 [US4] Push to branch → verify linting passes
- [ ] T080 [US4] Merge to main → verify full workflow runs, chapter appears on GitHub Pages
- [ ] T081 [US4] Verify chapter searchable: Query RAG chatbot with question about test chapter → verify citation includes test chapter

**Checkpoint**: User Story 4 complete. Full CI/CD pipeline operational, chapter deployment automated.

---

## Phase 7: Infrastructure & Deployment (Blocking for Production)

**Purpose**: Setup cloud infrastructure and deployment

- [ ] T082 Setup Vercel/Railway project for `/rag-backend`:
  - Connect GitHub repo, enable auto-deploy on push
  - Configure environment variables (OpenAI key, Qdrant URL, Neon DSN, Better-Auth secrets)
  - Setup logging (Sentry for error tracking)
- [ ] T083 Create Neon Postgres database and run migrations:
  - Create tables: users, chapters, rag_embeddings, personalization, sessions
  - Run initial migration via alembic
- [ ] T084 Create Qdrant Cloud collection for chapter embeddings:
  - Collection name: "physical-ai-textbook"
  - Vector size: 1536 (OpenAI text-embedding-3-small)
  - Configure similarity metric (cosine)
- [ ] T085 Configure CORS in FastAPI (T017):
  - Allow origin: `https://[github-pages-domain].github.io`
  - Allow credentials: true
  - Allow methods: GET, POST, PUT, DELETE
- [ ] T086 Setup monitoring + alerts:
  - Sentry: Error tracking for FastAPI + frontend
  - Uptime monitoring: Checkly or similar (99% uptime target)
  - Cost monitoring: OpenAI API usage dashboard

---

## Phase 8: Testing & QA (Optional - if TDD Requested)

**Purpose**: Comprehensive test coverage

### Unit Tests

- [ ] T087 [P] Backend unit tests for RAG service (`tests/unit/test_rag_service.py`):
  - Test: `embed_query()` returns 1536-dim vector
  - Test: `retrieve_context()` filters by similarity threshold (0.85)
  - Test: `generate_response()` includes source citation
  - Test: `check_hallucination()` detects out-of-context responses

- [ ] T088 [P] Backend unit tests for auth service (`tests/unit/test_auth_service.py`):
  - Test: `register_user()` hashes password correctly
  - Test: `login_user()` validates credentials
  - Test: `verify_token()` decodes JWT

- [ ] T089 [P] Frontend component tests (`docs-website/src/__tests__/RAGWidget.test.tsx`):
  - Test: RAGWidget renders with input field
  - Test: Submit question → API called with correct payload
  - Test: Response displayed with citation

### Integration Tests

- [ ] T090 [P] End-to-end test: Signup → Personalized intro → RAG query → Urdu toggle
  - Start: Open landing page
  - Sign up with "software" background
  - Navigate to chapter 1 → verify software-focused intro
  - Ask RAG question → verify response
  - Toggle to Urdu → verify RTL layout
  - Expected: All steps work without errors

---

## Phase 9: Polish & Documentation

**Purpose**: Final refinements and knowledge capture

- [ ] T091 [P] Create quickstart guide: `/docs-website/docs/00-getting-started/quickstart.md`
  - Hardware setup (RTX 4070 Ti, Jetson alternatives, cloud labs)
  - Clone repo, install dependencies (npm, poetry)
  - Run locally: `npm start` + `poetry run uvicorn src.main:app --reload`
  - Environment variables setup
  - First chapter to read

- [ ] T092 [P] Create contributor guide: `CONTRIBUTING.md`
  - How to write chapters (Markdown structure, frontmatter, examples)
  - How to trigger Urdu translation (manual review gate in CI)
  - How to test RAG locally
  - Git workflow (PR reviews, commit messages)

- [ ] T093 Create API documentation: `docs-website/docs/00-dev/api.md`
  - List all endpoints: /api/rag/query, /api/auth/signup, /api/profile, etc.
  - Request/response formats
  - Authentication (JWT header)
  - Error codes

- [ ] T094 Create deployment guide: `docs-website/docs/00-dev/deployment.md`
  - GitHub Actions workflow explanation
  - Vercel/Railway setup steps
  - Neon Postgres configuration
  - Cost monitoring instructions

- [ ] T095 [P] Code cleanup: Remove TODO comments, unused imports, debug logging
- [ ] T096 [P] Security audit: Review JWT handling, CORS config, API validation
- [ ] T097 Performance optimization: Identify slow endpoints, optimize queries

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational - BLOCKS all stories)
    ├─ → Phase 3 (US1: RAG Chatbot) [P]
    ├─ → Phase 4 (US2: Auth + Personalization) [P]
    ├─ → Phase 5 (US3: Urdu Localization) [P]
    └─ → Phase 6 (US4: CI/CD Deployment) [P]
         (can run in parallel once Phase 2 done)
    ↓
Phase 7 (Infrastructure - for production deployment)
    ↓
Phase 8 (Testing - optional)
    ↓
Phase 9 (Polish & Documentation)
```

### Critical Path to MVP

1. **Phase 1**: Setup (4 hours)
2. **Phase 2**: Foundational (8-10 hours) ← BLOCKING GATE
3. **Phase 3**: User Story 1 - RAG Chatbot (6-8 hours) ← MVP COMPLETE HERE
4. Stop and validate US1 independently before proceeding

### Full Feature Delivery

- After MVP (US1 validated), proceed with US2, US3, US4 in parallel (if team capacity)
- Each story should take 4-6 hours after foundational phase
- Phase 7 (infrastructure) needed for production
- Phase 8-9 (testing, polish) for final release

---

## Parallel Opportunities

### Phase 1 (Setup)
- T002, T003, T004 can run in parallel (different files)

### Phase 2 (Foundational)
**Frontend (T005-T012)** can run parallel with **Backend (T013-T027)**

Within Frontend:
- T006, T007, T008, T009, T012 parallelizable [P]
- T010, T011 sequential (T010 before T011)

Within Backend:
- T014, T015, T016 parallelizable [P] (install dependencies)
- T018, T019 parallelizable [P] (config files)
- T022, T023, T024 parallelizable [P] (service wrappers)

### Phase 3-6 (User Stories)
Once Phase 2 complete:
- **US1 Backend** (T028-T034) can run parallel with **US2 Backend** (T043-T049)
- **US1 Frontend** (T035-T042) can run parallel with **US2 Frontend** (T050-T056)
- **US3** (T061-T073) can start once Docusaurus configured (depends on T006-T012 from Phase 2)
- **US4** (T074-T081) can start in parallel with US1-US3 (CI/CD independent)

### Example Parallel Workflow (4-person team after Phase 2)

```
Developer A: US1 Backend (T028-T034) + US1 Frontend (T035-T042)
Developer B: US2 Backend (T043-T049) + US2 Frontend (T050-T056)
Developer C: US3 Urdu (T061-T073)
Developer D: US4 CI/CD (T074-T081)

Timeline: ~1 week for US1-US4 with 4 developers
```

---

## Implementation Strategy

### MVP First (Recommended for Hackathon)

1. **Complete Phase 1** (Setup) - 4 hours
2. **Complete Phase 2** (Foundational) - 8-10 hours
3. **Complete Phase 3** (US1: RAG) - 6-8 hours
4. **TEST & VALIDATE**: Standalone RAG chatbot works
5. **Commit to main + Demo**
6. **THEN** add US2, US3, US4 incrementally

### Incremental Delivery Strategy

- Phase 1 + Phase 2: Foundation ready (no visible product yet)
- Phase 1 + Phase 2 + Phase 3: **MVP LIVE** - Students can use RAG chatbot
- Phase 1 + Phase 2 + Phase 3 + Phase 4: Add personalization (increases engagement)
- Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5: Add Urdu (expands market)
- Phase 1 + Phase 2 + Phase 3 + Phase 4 + Phase 5 + Phase 6: Full CI/CD (enables scaling)
- Phase 7-9: Production-grade features (monitoring, docs, tests)

### Quality Gates

- **After Phase 2**: Verify foundational tasks complete + basic server startup works
- **After Phase 3**: Verify US1 works independently (RAG queries succeed, source citations present)
- **After Phase 4**: Verify US2 integrates without breaking US1 (auth + personalization both work)
- **After Phase 5**: Verify US3 works without breaking US1/US2 (Urdu toggle, RTL layout)
- **After Phase 6**: Verify CI/CD pipeline fully functional (deploy = live within 5 min)

---

## Notes

- **[P]**: Parallelizable tasks (different files, no inter-dependencies within the same phase)
- **[Story]**: Tasks mapped to user story (US1, US2, US3, US4)
- **Blockers**: Phase 2 MUST complete before any user story work
- **Tests**: Included only if team requests TDD - otherwise implement features directly
- **Commits**: After each major task or logical group (e.g., after T020 in foundational, after T034 in US1)
- **Deployment**: Phase 7 required for production; can skip for hackathon MVP (localhost/staging sufficient)

---

## Total Task Estimate

- **Setup**: 4 hours (Phase 1)
- **Foundational**: 8-10 hours (Phase 2) - CRITICAL PATH
- **User Story 1 (RAG)**: 6-8 hours (Phase 3)
- **User Story 2 (Auth+Personalization)**: 4-6 hours (Phase 4)
- **User Story 3 (Urdu)**: 4-5 hours (Phase 5)
- **User Story 4 (CI/CD)**: 3-4 hours (Phase 6)
- **Infrastructure**: 2-3 hours (Phase 7)
- **Testing**: 5-8 hours (Phase 8, optional)
- **Polish**: 3-4 hours (Phase 9)

**MVP (Phases 1-3)**: 18-22 hours (achievable in 2-3 days with focused team)
**Full Feature (Phases 1-6)**: 30-38 hours (achievable in 4-5 days)
**Production-Ready (Phases 1-9)**: 48-60 hours (achievable in 1 week with QA)
