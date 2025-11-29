# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-ai-textbook`
**Created**: 2025-11-29
**Status**: Draft
**Input**: Master specification for AI-native textbook teaching embodied intelligence with Docusaurus, RAG chatbot, authentication, personalization, and Urdu localization.

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Student Reads Chapter and Asks Context-Aware Questions (Priority: P1)

A student opens the textbook, reads a chapter on ROS 2 Topics, highlights a code snippet, and asks the floating "Ask the Book" widget: "Explain this code". The chatbot retrieves relevant content from the same chapter and provides an answer grounded in the textbook.

**Why this priority**: This is the core value proposition—a living, interactive textbook. Without this, the system is just a static site.

**Independent Test**: Student can highlight text, open the RAG widget, ask a question, and receive an answer sourced from chapter context (with attribution). This alone demonstrates the textbook's value.

**Acceptance Scenarios**:

1. **Given** a student is reading Module 1 Chapter 2 (ROS 2 Nodes), **When** they highlight a code block and ask "What does this callback do?", **Then** the system retrieves explanation from the same section and responds with source citation (e.g., "From Chapter 2: Nodes section").
2. **Given** a student asks a question outside the textbook scope (e.g., "What's the weather?"), **When** they submit the query, **Then** the system responds: "I don't have information on this in the textbook. Please refer to Chapter X or ask an instructor."
3. **Given** a question has multiple relevant sources, **When** the chatbot responds, **Then** it cites the most relevant section with high confidence (≥0.85 cosine similarity).

---

### User Story 2 - Student Signs Up and Receives Personalized Content (Priority: P1)

A student clicks "Sign Up", enters email via `better-auth`, and answers a brief profile question: "Are you more comfortable with hardware (robotics) or software (code)?" On their next login, chapter introductions are rewritten to match their background. A "Personalize" button allows them to update their profile and refresh content.

**Why this priority**: Personalization increases engagement and retention. This drives conversion and reduces support burden (fewer confused students).

**Independent Test**: Student signs up, completes profile, and sees personalized chapter intro. Can be tested end-to-end without other features.

**Acceptance Scenarios**:

1. **Given** a student with "software" background views Chapter 1, **When** the page loads, **Then** the intro emphasizes code examples and programming concepts first.
2. **Given** a student with "hardware" background views the same chapter, **When** the page loads, **Then** the intro emphasizes robotics and physical systems first, with code as secondary.
3. **Given** a student clicks "Personalize" button, **When** they update their profile, **Then** chapter content refreshes within 2 seconds.
4. **Given** a new visitor (not signed up), **When** they view a chapter, **Then** the default intro is shown (generic, not personalized).

---

### User Story 3 - Student Reads Textbook in Urdu (Priority: P1)

A student clicks a "Translate to Urdu" button in the header. All explanatory text (paragraphs, examples, learning objectives, key takeaways) toggles to Urdu, while code remains English. They can toggle back to English at any time.

**Why this priority**: Accessibility and inclusivity are core values (Principle III). Serving Pakistani/South Asian learners expands reach and aligns with project goals.

**Independent Test**: Toggle button switches language for all content sections (except code). Can be tested independently.

**Acceptance Scenarios**:

1. **Given** a student views a chapter in English, **When** they click "Translate to Urdu", **Then** all text is translated to Urdu within 1 second.
2. **Given** a student is viewing content in Urdu, **When** they read a code example, **Then** the code remains in English.
3. **Given** a student toggles back to English, **When** the page refreshes, **Then** English text is restored.
4. **Given** an RTL (right-to-left) layout is required, **When** Urdu is active, **Then** the page layout adjusts to RTL formatting.

---

### User Story 4 - Admin/Author Deploys New Chapter (Priority: P2)

An author writes a new chapter in Markdown, includes learning objectives, examples, and prerequisites. They commit to the repo. GitHub Actions triggers an automated build: tests run, RAG index updates, performance benchmarks are validated. On success, the textbook is deployed to GitHub Pages automatically.

**Why this priority**: Continuous deployment enables rapid iteration and weekly releases (per Constitution). Without this, the textbook becomes a static artifact.

**Independent Test**: Author commits a new chapter, CI/CD succeeds, and the chapter is live on GitHub Pages within 5 minutes.

**Acceptance Scenarios**:

1. **Given** an author commits a new chapter with proper Markdown structure, **When** GitHub Actions runs, **Then** the build completes within 5 minutes without errors.
2. **Given** a chapter lacks required sections (e.g., learning objectives), **When** CI runs, **Then** the build fails with a clear error message indicating what's missing.
3. **Given** the build succeeds, **When** the workflow completes, **Then** the new chapter is deployed to GitHub Pages and accessible at the correct URL.

---

### Edge Cases

- **RAG out-of-scope query**: What happens when a student asks the RAG chatbot about a topic not covered in any chapter? → System responds: "I don't have information on this in the textbook. Please refer to Chapter X or ask an instructor."
- **Hardware mismatch (clarified)**: What if a student's hardware (e.g., mobile, CPU-only machine) doesn't meet chapter requirements (Isaac Sim, VSLAM)? → System detects via WebGL/device memory heuristics and shows non-blocking warning banner: "This chapter requires GPU hardware. [View cloud lab alternatives]". User can proceed to read theory or click to cloud options (Google Colab, Paperspace, AWS).
- **Incomplete Urdu translation**: What if Urdu translation is incomplete for a chapter? → System displays banner: "Urdu translation coming soon. Viewing English version." and renders English content.
- **RAG ingestion failure**: What if the RAG index fails to build during deployment? → Docusaurus build succeeds, but RAG ingestion step fails → deployment blocks (chapter not live). Author receives error log and retries after fixing issues (e.g., malformed chapter sections, embedding API rate limit).

## Requirements *(mandatory)*

### Functional Requirements

#### Core Book Engine (Docusaurus v3+)

- **FR-001**: System MUST be built on Docusaurus v3+ with TypeScript, supporting MDX for interactive content.
- **FR-002**: System MUST deploy to GitHub Pages via GitHub Actions without manual intervention.
- **FR-003**: System MUST include a landing page that clearly communicates the value of Physical AI (e.g., "Learn embodied AI with real robots and simulations").
- **FR-004**: System MUST organize content into 5 distinct modules (Nervous System, Digital Twin, Robot Brain, The Mind, Capstone), each as a separate directory structure.
- **FR-005**: System MUST support Markdown with code syntax highlighting (Python, C++, bash, YAML for ROS configs).

#### Content & Curriculum

- **FR-006**: Each chapter MUST include: learning objectives (start), content, ≥3 worked examples, key takeaways (end), and prerequisites.
- **FR-007**: System MUST clearly document hardware assumptions: NVIDIA RTX 4070 Ti (primary), Jetson Orin Nano/NX (edge), and Unitree Go2/G1 robots. Alternative paths (cloud labs) must include cost disclaimers.
- **FR-008**: System MUST validate content readability at grade 10 or lower (Flesch-Kincaid metric).
- **FR-009**: All diagrams MUST include alt text. All videos MUST include captions in English and (if translated) Urdu subtitles.
- **FR-010**: System MUST track content versioning: major changes trigger learner notifications; minor changes auto-propagate.

#### RAG Chatbot ("Ask the Book" Widget)

- **FR-011**: Frontend MUST include a floating "Ask the Book" widget (accessible from every page, sticky positioning).
- **FR-012**: Widget MUST allow students to: (a) highlight text and ask a question, or (b) type a free-form question.
- **FR-013**: Backend MUST use OpenAI Agents SDK for query understanding, Qdrant (cloud) for vector retrieval, and Neon (serverless Postgres) for metadata/history.
- **FR-014**: Every RAG response MUST include source citation (chapter + section), and MUST NOT exceed 5 seconds latency (p95).
- **FR-015**: RAG MUST enforce semantic similarity threshold: only retrieve context with ≥0.85 cosine distance to query. If no match, respond: "I don't have information on this in the textbook."
- **FR-016**: System MUST perform weekly spot-checks on 5% of RAG responses to validate accuracy vs. chapter text. Hallucinations trigger escalation and feature pause.

#### Authentication & Profiling

- **FR-017**: System MUST integrate `better-auth` for secure user signup/login (email + password, OAuth2 optional). Backend MUST expose `/api/profile` endpoint (protected by JWT) for profile management.
- **FR-018**: On signup, system MUST collect a user profile: background preference (hardware vs. software vs. balanced).
- **FR-019**: System MUST securely store user profiles in Neon Serverless Postgres with encrypted passwords (handled by better-auth). Sessions managed via JWT tokens (stored in browser localStorage, refresh tokens rotated server-side).
- **FR-020**: Users MUST be able to view and update their profile at any time via `/api/profile` endpoint.

#### Personalization

- **FR-021**: System MUST detect user profile (hardware/software/balanced) and rewrite chapter introductions accordingly.
- **FR-022**: A "Personalize" button MUST be visible on every chapter, allowing students to update their profile and refresh content.
- **FR-023**: Personalized content MUST load within 500ms (p95 latency).
- **FR-024**: For anonymous users (not signed up), system MUST show default chapter intro (generic, non-personalized).

#### Localization (Urdu)

- **FR-025**: System MUST include a language toggle button (English / Urdu) in the header, visible on every page. Switching languages MUST be client-side (no API call).
- **FR-026**: Clicking "Translate to Urdu" MUST load pre-built Urdu markdown files (`chapter.ur.md`) and render within <100ms (<1s page re-render). All explanatory text (paragraphs, learning objectives, key takeaways, examples) MUST be in Urdu.
- **FR-027**: Code examples, terminal outputs, and code comments MUST remain in English even when Urdu is active.
- **FR-028**: When Urdu is active, page layout MUST adjust to RTL (right-to-left) formatting via Docusaurus i18n configuration.
- **FR-029**: Urdu translations MUST be generated at build time (Docusaurus CI/CD → OpenAI API → native speaker review → `.ur.md` files). All translations MUST be reviewed by a native Urdu speaker + domain expert before merge. Technical glossary MUST be maintained.
- **FR-030**: System MUST gracefully fall back to English if Urdu translation is incomplete for a chapter (banner: "Urdu translation coming soon").

#### Deployment & CI/CD

- **FR-031**: GitHub Actions workflow MUST run on every commit to `main` or merge to `main`.
- **FR-032**: Workflow MUST: (a) lint content, (b) build Docusaurus (including Urdu translation generation + native speaker review gate), (c) update RAG index (automatic batch ingestion to Qdrant), (d) run performance benchmarks, (e) deploy to GitHub Pages. RAG ingestion MUST complete within 2 minutes.
- **FR-033**: If any step fails, deployment MUST stop and notify the author with detailed error logs. If RAG ingestion fails, block deployment (requires manual intervention).
- **FR-034**: Successful deployments MUST complete within 10 minutes (excluding manual review gates like Urdu translation approval).

#### Reusable Intelligence (Claude Skills)

- **FR-035**: Repository MUST contain `.claude/skills/` directory with at least one reusable prompt/tool (e.g., "ROS 2 Node Generator").
- **FR-036**: Each skill MUST be documented with: input specification, output specification, example usage, and limitations.

### Key Entities

- **Chapter**: Content unit with learning objectives, sections, examples, key takeaways, prerequisites, and version info.
- **User**: Student or instructor with email, password (via better-auth), profile (hardware/software preference), and content preferences.
- **RAG Embedding**: Vector representation of chapter sections (via OpenAI embeddings, stored in Qdrant).
- **RAG Retrieval Result**: Ranked list of chapter sections matching a query, with similarity scores.
- **Personalization Rule**: Maps user profile to chapter intro variant (hardware-focused, software-focused, balanced).
- **Content Version**: Major/minor version for chapters, with learner notification rules and backward compatibility tags.

## Success Criteria *(mandatory)*

### Measurable Outcomes

#### Functionality & Reliability

- **SC-001**: Landing page clearly communicates Physical AI value and converts visitors to sign up. Verification: Landing page loads without errors, includes value propositions (e.g., "Build embodied AI with real robots"), and sign-up form is visible.
- **SC-002**: Textbook contains all 5 modules with ≥3 chapters each, total ≥15 chapters deployed and accessible. Verification: Each chapter is live on GitHub Pages, has learning objectives, and includes ≥3 examples.
- **SC-003**: RAG chatbot responds to student queries in <5s (p95 latency). Verification: Load test with 100 concurrent queries; measure response times.
- **SC-004**: RAG responses include correct source citations in ≥95% of responses. Verification: Manual review of 5% weekly spot-check sample; no hallucinations detected.
- **SC-005**: Students can toggle Urdu language and see all content (except code) in Urdu within 1 second. Verification: Functional test: click toggle, measure render time, verify text language matches.
- **SC-006**: Deployment workflow completes successfully within 10 minutes. Verification: GitHub Actions logs show all steps complete, artifact deployed to GitHub Pages.

#### User Experience & Engagement

- **SC-007**: ≥80% of signed-up students complete at least one chapter. Verification: Analytics dashboard tracks completion rates.
- **SC-008**: Personalized chapter intros are perceived as more relevant by ≥70% of students (survey). Verification: Post-chapter survey: "Did the intro match your background?" ≥70% say "yes".
- **SC-009**: RAG chatbot is used by ≥50% of students at least once per chapter. Verification: Event tracking in analytics; measure "Ask the Book" button clicks per student.
- **SC-010**: ≥90% of students successfully complete signup and profile setup on first attempt. Verification: Funnel analysis; measure drop-off at each step.

#### Cost & Operations

- **SC-011**: Monthly operational costs <$100 USD (OpenAI API, Qdrant, Neon, Better-Auth, GitHub Pages). Verification: Monthly cost audit; if >$100, cost-reduction PR submitted within 5 days.
- **SC-012**: System uptime ≥99% (monitoring via Sentry). Verification: Sentry dashboard shows error rate <1%.
- **SC-013**: RAG index updates complete within 2 minutes of chapter deployment. Verification: Measure time from merge to RAG index update in CI logs.

#### Content Quality

- **SC-014**: All chapters meet readability target (grade 10 or lower). Verification: Flesch-Kincaid score calculated for each chapter; if >grade 10, content is revised.
- **SC-015**: All chapters include alt text for diagrams and captions for videos. Verification: Accessibility audit; manual verification of ≥5 random chapters.
- **SC-016**: Urdu translations are accurate and reviewed by native speaker. Verification: Native speaker review checklist signed off; technical glossary maintained.

#### Bonus Objectives

- **SC-017**: Authentication via better-auth works without errors. Verification: Signup workflow tested end-to-end; user can login with email/password.
- **SC-018**: User profile persists across sessions. Verification: Student logs out, logs in again; profile data is restored.
- **SC-019**: At least one reusable skill (e.g., ROS 2 Node Generator) is available in `.claude/skills/`. Verification: File exists, includes documentation, and can be invoked from Claude Code.

---

## Assumptions

1. **Technology Stack**: Docusaurus, React, TypeScript, FastAPI (for RAG backend), OpenAI API, Qdrant Cloud, Neon Serverless Postgres, Better-Auth, GitHub Pages.
2. **Hardware**: Primary target is NVIDIA RTX 4070 Ti (64GB RAM, Ubuntu 22.04); alternative paths for Jetson Orin and cloud-based labs.
3. **Content Readiness**: Authors will provide chapters in Markdown with required sections (learning objectives, examples, key takeaways).
4. **Localization**: Initial launch supports English + Urdu. Other languages can be added post-launch.
5. **RAG Accuracy**: OpenAI's embedding model (text-embedding-3-small) provides sufficient semantic understanding for robotics domain. If hallucinations occur, fine-tuning or retrieval augmentation (e.g., keyword filters) will be applied.
6. **User Behavior**: Students will sign up and complete profiles; personalization value is validated by increased completion rates.
7. **Cost**: Free tiers of Qdrant, Neon, and Better-Auth are sufficient for launch. Cost tracking is monitored weekly; if budget exceeded, features are optimized (e.g., batch API calls, model downgrade).

---

## Out of Scope (For This Phase)

- Mobile app (web-only for v1).
- Real-time collaboration (e.g., shared notebooks).
- Advanced analytics dashboards (basic event tracking only).
- Payment/licensing (free for all students in hackathon context).
- Automated grading or homework submission.
- Video hosting (links to external platforms only).

---

## Dependencies

- **External**: OpenAI API, Qdrant Cloud, Neon, Better-Auth, GitHub (for Pages and Actions).
- **Internal**: Constitution (Principle II: content-first; Principle IV: chapter-based deployment; Principle V: RAG quality validation).
- **Team**: Content authors (minimum 1), ML engineer (RAG indexing), frontend engineer (Docusaurus + personalization), backend engineer (RAG API).

---

## Clarifications

### Session 2025-11-29

- **Q1: Backend Hosting & CORS** → A: Vercel/Railway (serverless Python) with explicit CORS allowlist for GitHub Pages frontend.
- **Q2: Auth & Profile Persistence** → A: Better-Auth + FastAPI custom endpoint for profile management; user profiles stored in Neon Postgres with encrypted passwords.
- **Q3: RAG Ingestion Trigger** → A: Automatic ingestion on every successful Docusaurus build via GitHub Actions; embeds new/modified chapters to Qdrant within ~2 minutes.
- **Q4: Urdu Translation Strategy** → A: Pre-built at Docusaurus build time; static Urdu files generated during CI/CD and toggled client-side (<100ms). Requires native Urdu speaker + domain expert review per chapter.
- **Q5: Hardware Fallback Behavior** → A: Warning banner on GPU-intensive chapters (Isaac Sim, VSLAM) with cloud lab alternatives and cost disclaimers. Non-blocking; users can proceed to read or redirect to cloud options.

---

## Architecture Decisions (from Clarifications)

### Backend Deployment
- **Hosting**: FastAPI backend on Vercel or Railway (serverless, <$20/mo combined with frontend CORS allowlist).
- **CORS Policy**: Allow requests from `https://github-pages-domain.github.io` (allowlist, not wildcard).
- **Database**: Neon Serverless Postgres (free tier supports initial load; monitored for scaling).

### Authentication & User Profiles
- **Flow**: Better-Auth handles signup/login; FastAPI exposes `/api/profile` endpoint (protected by JWT from Better-Auth).
- **Storage**: User profiles (email, background preference, preferences) persisted in Neon; passwords encrypted (better-auth manages hashing).
- **Session**: JWT tokens, stored in browser localStorage; refresh tokens rotated on server-side.

### RAG Ingestion & Indexing
- **Trigger**: GitHub Actions workflow on every successful Docusaurus build.
- **Process**: Extract chapter frontmatter + sections → embed via OpenAI API → upsert to Qdrant.
- **Cost Control**: Batch API calls; embed only new/modified chapters (delta indexing). Monitor token usage weekly.
- **Failure Handling**: If ingestion fails, build succeeds but deployment blocked (requires manual intervention + retry).

### Urdu Localization (Pre-Built)
- **Build Process**: During Docusaurus build, for each chapter:
  1. Extract English markdown frontmatter + body.
  2. Send to OpenAI for translation → review by native speaker (blocking step in CI).
  3. Write `chapter.ur.md` alongside `chapter.md`.
  4. Configure Docusaurus i18n to load `.ur.md` files when locale = Urdu.
- **Toggle UX**: Client-side language switch (no API call) → re-renders page with `.ur.md` content (<100ms).
- **Fallback**: If `.ur.md` missing, show banner "Urdu translation coming soon" and render English.

### Hardware Fallback Strategy
- **Chapter Metadata**: Add `requiredHardware: ["GPU", "Jetson"]` to chapter frontmatter (e.g., Isaac Sim chapters).
- **Client Detection**: On page load, check device type (mobile, tablet, CPU-only heuristic) via feature detection (WebGL, device memory API).
- **Warning Banner**: If user device doesn't match chapter requirements, show non-blocking banner: "This chapter requires [hardware]. [View cloud lab alternatives →]"
- **Cloud Alternatives**: Link to Google Colab / Paperspace / AWS notebooks with pre-configured environments (maintained as bonus task).

---

## Open Questions / Clarifications Needed

None at this stage. All critical architectural decisions finalized. Ready for `/sp.plan` phase.
