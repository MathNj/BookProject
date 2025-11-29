---
id: 005
title: Clarify Architectural Decisions for AI Textbook
stage: spec
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: ai-textbook
branch: 001-ai-textbook
user: user
command: "/sp.clarify GOAL: Interrogate the Specification to identify edge cases, architectural gaps, and missing constraints. CONTEXT: Hybrid application (Static Frontend + Intelligent Backend). AREAS TO CRITIQUE: 1) Static vs. Dynamic Conflict (GitHub Pages + FastAPI backend hosting), 2) Better-Auth Integration & user profile persistence, 3) RAG Chatbot Ingestion logistics, 4) Urdu Translation strategy (on-the-fly vs pre-built), 5) Hardware dependencies & fallback."
labels: ["clarification", "architecture", "backend-deployment", "authentication", "rag-ingestion", "localization", "hardware-fallback"]
links:
  spec: specs/001-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-ai-textbook/spec.md (modified, added 63 lines with clarifications + architecture decisions)
tests:
  - All 5 questions answered with explicit decision
  - Functional requirements updated with architectural specifics
  - Edge cases enriched with implementation guidance
  - No unresolved ambiguities remain
---

## Prompt

GOAL: Interrogate the Specification to identify edge cases, architectural gaps, and missing constraints.

CONTEXT:
We have a draft `sp.spec.md` for a "Physical AI" textbook. However, this is a complex hybrid application (Static Frontend + Intelligent Backend). I need you to find the hidden risks.

AREAS TO CRITIQUE (Ask me specific questions about these):

1.  **The "Static vs. Dynamic" Conflict:**
    - The book is hosted on GitHub Pages (Static). The Chatbot and Auth need a backend (Python/FastAPI).
    - *Question to resolve:* Where are we hosting the Python Backend? (Render? Railway? Localhost for now?). How do we handle CORS?

2.  **The "Better-Auth" Integration:**
    - GitHub Pages cannot run server-side Node.js auth middleware.
    - *Question to resolve:* Are we using a client-side auth flow (OAuth) talking to a separate auth server, or a comprehensive BaaS? How do we persist the "User Profile" (Hardware vs Software Engineer) securely?

3.  **The RAG Chatbot Logistics:**
    - *Question to resolve:* When does the "Ingestion" happen? (Do we scrape our own book every time we push code? Or is it a manual script?).
    - *Question to resolve:* How do we limit costs? (Do we need rate limiting so one user doesn't burn all our OpenAI credits?).

4.  **Localization (Urdu) Strategy:**
    - *Question to resolve:* Is the Urdu translation generated "On the Fly" (slow, expensive API calls) or "Pre-Built" (static files generated at build time)? The prompt mentions "pressing a button," which implies dynamic, but pre-built is safer for a Hackathon.

5.  **Hardware Dependencies:**
    - *Question to resolve:* If a user visits the site on a mobile phone (without an RTX 4070), should the heavy "Isaac Sim" tutorials show a warning banner?

OUTPUT:
Analyze the spec and present a list of **Clarifying Questions** (Q1, Q2, Q3...) that I must answer to finalize the requirements.

## Response snapshot

Presented 5 critical architectural questions in sequential format, one at a time:

**Q1: Backend Hosting & CORS** → User selected **Option A**: Vercel/Railway serverless Python with CORS allowlist.

**Q2: Auth & Profile Persistence** → User selected **Option A**: Better-Auth + FastAPI /api/profile endpoint + Neon Postgres for profile storage + JWT sessions.

**Q3: RAG Ingestion Trigger** → User selected **Option A**: Automatic ingestion on every successful Docusaurus build via GitHub Actions (~2 min latency, batch processing).

**Q4: Urdu Translation Strategy** → User selected **Option A**: Pre-built at build time (static Urdu .md files generated during CI/CD, client-side toggle <100ms, requires native speaker review).

**Q5: Hardware Fallback Behavior** → User selected **Option A**: Warning banner on GPU-intensive chapters (Isaac Sim, VSLAM) + cloud lab alternatives (Google Colab, Paperspace, AWS) with cost disclaimers. Non-blocking user can read theory or redirect.

All clarifications integrated into spec:
- Functional requirements updated with architectural specifics (JWT tokens, Neon DB, /api/profile, RAG 2min SLA, build-time Urdu translation)
- New "Architecture Decisions" section with implementation guidance for: backend deployment (Vercel/Railway, CORS), auth flow (JWT + Neon), RAG ingestion (batch, delta indexing), Urdu (i18n + pre-built), hardware (WebGL detection, banner)
- Edge cases enriched with hardware detection mechanism + cloud lab fallbacks

## Outcome

- ✅ Impact: All critical architectural ambiguities resolved. Specification now includes explicit deployment targets, auth flow details, cost control mechanisms, build-time constraints, and fallback strategies. Ready for `/sp.plan` phase with zero blockers.
- 🧪 Tests: All 5 questions answered with user-selected decisions; functional requirements updated; edge cases enriched; no unresolved placeholders.
- 📁 Files:
  - specs/001-ai-textbook/spec.md (63 lines added: clarifications section + architecture decisions + updated FRs + enriched edge cases)
  - history/prompts/ai-textbook/005-clarify-architectural-decisions.spec.prompt.md (PHR documenting session)
- 🔁 Next prompts:
  - Run `/sp.plan` to generate detailed architecture, API contracts, data models, deployment strategy
  - Consider `/sp.adr` for significant decisions (pre-built Urdu vs. runtime, RAG delta indexing, hardware detection heuristics)
- 🧠 Reflection: Clarification workflow was highly effective. 5 targeted questions covering deployment, auth, data pipeline, localization, and UX all drove toward explicit, implementable decisions. No ambiguity remains. User selected recommended options consistently (likely due to alignment with hackathon constraints: budget <$100/mo, tight timeline, cost-conscious tradeoffs).

## Evaluation notes (flywheel)

- Failure modes observed: None—all questions well-scoped, user answers aligned with project constraints (hackathon budget, timeline, accessibility goals).
- Graders run and results (PASS/FAIL): ✅ PASS (all 5 questions answered, spec updated, no contradictions, ready for planning).
- Prompt variant (if applicable): None (standard 5-question clarification flow).
- Next experiment: Monitor during planning whether pre-built Urdu strategy requires manual bottleneck (native speaker review gate in CI/CD). If so, propose async review process or batch chapter translations to reduce blocking.
