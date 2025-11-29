# SUPREME COURT CONSTITUTION
## Hackathon I: Physical AI & Humanoid Robotics Textbook
### Version 1.0.0 | Effective Date: 2025-11-29

---

## PREAMBLE

This Constitution is the immutable Supreme Law of the Physical AI & Humanoid Robotics Textbook project. All agents, developers, architects, and contributors SHALL adhere strictly to the Articles and Sections herein. In the event of conflict between this Constitution and any other document, policy, or informal guidance, **this Constitution prevails absolutely**. No deviation, exception, or interpretation is permitted without formal Constitutional Amendment (Article X).

---

## ARTICLE I: THE STACK MANDATE

### Section 1.1: Frontend & Content Delivery (IMMUTABLE)

**Frontend Framework**: Docusaurus v3.0 or higher, configured with:
- Language: TypeScript (strict mode)
- Styling: Tailwind CSS v3 or equivalent
- Content Format: MDX (Markdown + React components)
- Theme Customization: Via Docusaurus swizzling for RAG chatbot, personalization widgets

**Deployment Target**: GitHub Pages
- Domain: GitHub-managed CNAME or default GitHub Pages subdomain
- SSL/TLS: Automatic via GitHub
- CDN: GitHub's built-in CDN (no external CDN required)
- Update Frequency: Atomic pushes to `main` branch trigger automatic builds and deployments

**Rationale**: Docusaurus is battle-tested for technical documentation, GitHub Pages is free, and TypeScript ensures type safety across interactive components (RAG widget, personalization toggles).

---

### Section 1.2: Backend API & RAG Service (IMMUTABLE)

**Backend Framework**: FastAPI (Python 3.11+)
- OpenAI Agents SDK for multi-turn RAG conversations
- Structured logging with Python logging module
- CORS enabled for Docusaurus frontend (same-origin + GitHub Pages domain)

**RAG Pipeline**:
- Embeddings Model: OpenAI `text-embedding-3-small`
- Chat Model: OpenAI `gpt-4-turbo` (or fallback `gpt-3.5-turbo` for cost optimization)
- Context Window: Minimum 128k tokens (for gpt-4-turbo)
- Chunk Strategy: Semantic chunking with overlap (chunk size 512 tokens, overlap 50 tokens)

**Rationale**: FastAPI is lightweight, async-ready, and integrates seamlessly with OpenAI SDKs. Semantic chunking prevents artificial context boundaries that cause hallucinations.

---

### Section 1.3: Data Persistence (IMMUTABLE)

**Vector Database**: Qdrant Cloud (Free Tier)
- Endpoint: Cloud-hosted (no self-hosted clusters)
- Index Strategy: Namespaced by chapter (e.g., `module_1_chapter_1`, `module_2_chapter_3`)
- Backup: Daily snapshots (Qdrant Cloud feature)
- Capacity Limit: 100M vectors (free tier limit; architecture must plan for growth within this cap)

**Relational Database**: Neon Serverless Postgres
- Connection Pooling: PgBouncer or Neon's built-in pooling (mandatory for edge compute)
- Schema: Versioned migrations in `backend/migrations/` directory
- Backup: Automated daily backups via Neon (included in free tier)
- Tables Required:
  - `users` — authentication and profile data
  - `user_profiles` — personalization settings (software/hardware background, language preference)
  - `chapters` — chapter metadata (version, translations available, prerequisites)
  - `chapter_feedback` — learner comments, issues, ratings
  - `rag_interactions` — query logs (for hallucination auditing)
  - `deployment_log` — deployment timestamps and performance metrics

**Rationale**: Qdrant Cloud is free and managed; Neon Serverless eliminates ops overhead during a hackathon and automatically scales. PgBouncer is critical for cost control (serverless Postgres charges by connection minutes).

---

### Section 1.4: Authentication & User Management (IMMUTABLE)

**Auth Provider**: Better-Auth.com
- OAuth2 support (Google, GitHub recommended for students)
- Local password auth as fallback
- Session duration: 30 days
- Multi-factor authentication: Optional (not required for MVP)

**User Onboarding (Mandatory for Bonus Points)**:
Upon first login, users MUST answer:
1. Software Background: "Beginner / Intermediate / Advanced"
2. Hardware Background: "No robotics experience / ROS 1 / ROS 2 / Gazebo"
3. Target Robot: "Quadruped (Go2) / Humanoid (G1) / Unsure"
4. Preferred Language: "English / Urdu"

Responses stored in `user_profiles` table; used to drive personalization engine.

**Rationale**: Better-Auth eliminates proprietary lock-in; self-hosted OAuth is possible if needed. User profiling enables immediate personalization (Bonus Objective #5).

---

### Section 1.5: Deployment Infrastructure (IMMUTABLE)

**Frontend Deployment**: GitHub Pages (configured in `docusaurus.config.js`)

**Backend Deployment**: One of:
- Vercel (recommended; zero-config FastAPI support via `/api` convention)
- Railway.app (cheap serverless compute)
- Render.com (managed Python environment)

**Ephemeral Compute Acceptable**: Backend MAY run on ephemeral instances (cold starts allowed). P95 latency targets allow for cold starts <500ms (see Article V).

**Environment Configuration**:
- `.env.local` (git-ignored): Local development
- `.env.production`: Production secrets (stored in deployment platform)
- No secrets committed to git

**Rationale**: Serverless eliminates ops burden and is cost-effective for a hackathon. GitHub Pages is free; backend compute cost capped at $50/month (Article VII).

---

## ARTICLE II: CURRICULUM STRUCTURE & CONTENT MANDATE

### Section 2.1: Module Hierarchy (IMMUTABLE)

The textbook MUST contain exactly FOUR (4) instructional modules plus ONE (1) capstone project, in this order:

| Module | Title | Focus Area | Prerequisites |
|--------|-------|------------|-----------------|
| 1 | **The Nervous System** | ROS 2, Nodes, Topics, Services, rclpy | None |
| 2 | **The Digital Twin** | Gazebo, Unity, Physics, Sensor Simulation | Complete Module 1 |
| 3 | **The Brain** | NVIDIA Isaac Sim, VSLAM, Nav2, Manipulation | Complete Module 2 |
| 4 | **The Mind** | VLA Models, Whisper (Voice), LLM Planning | Complete Module 3 |
| Capstone | **The Autonomous Humanoid** | Sim-to-Real, Full Integration | Complete Module 4 |

**Section 2.2: Chapter Breakdown (ADVISORY)**

Each module MAY contain 2–5 chapters. Minimum 12 chapters total (3 per module average). Example structure:

- **Module 1** (ROS 2): 4 chapters — Intro to ROS, Nodes & Topics, Services & Actions, rclpy Programming
- **Module 2** (Digital Twin): 3 chapters — Gazebo Setup, URDF & Physics, Sensor Simulation in Unity
- **Module 3** (Isaac): 3 chapters — Isaac Sim Basics, VSLAM & Navigation, Manipulation
- **Module 4** (VLA): 3 chapters — Vision-Language Models, Voice-to-Action, LLM Planning
- **Capstone**: 1 chapter — End-to-End Project (student builds autonomous humanoid)

**Section 2.3: Content Quality Gates (MANDATORY)**

Every chapter MUST satisfy:
1. **Learning Objectives** (at chapter start): 3–5 clear, measurable outcomes (e.g., "Student will be able to create a ROS 2 node that publishes sensor data")
2. **Code Examples**: ≥3 worked examples with copy-paste-able code, fully commented
3. **Readability**: Flesch-Kincaid Grade Level ≤10 (technical language allowed, but explanations must be clear)
4. **Accuracy**: 100% peer review by domain expert (roboticist or ML engineer)
5. **Accessibility**:
   - All diagrams include alt text (PlantUML or Draw.io files)
   - All videos captioned in English (YouTube auto-captions acceptable with review)
   - Code terminal outputs as text, not screenshots
6. **Key Takeaways** (at chapter end): 3–5 bullet points summarizing key concepts
7. **Prerequisites** (if not Module 1 Chapter 1): Explicitly list which prior chapters are required

**Rationale**: Measurable gates ensure pedagogical quality and consistency. Accessibility ensures diverse learners (including those with disabilities) can learn effectively.

---

### Section 2.4: Prerequisite Gating in Personalization (MANDATORY)

The personalization system MUST enforce chapter prerequisites:
- If a user attempts to skip Module 1 and jump to Module 3, the system SHALL display: "This chapter requires Module 2 knowledge. Please complete Module 2 first."
- Users MAY unlock chapters out-of-order IF they score ≥80% on a prerequisite assessment
- Assessments SHALL be brief (5–10 questions) and OPTIONAL for users who completed prerequisites in order

**Rationale**: ROS 2 concepts are prerequisites for Isaac Sim; skipping them leads to learner frustration. Flexible gating allows advanced learners to progress faster.

---

## ARTICLE III: HARDWARE CONSTRAINTS & TARGET LEARNERS

### Section 3.1: Primary Workstation Configuration (EXPLICIT ASSUMPTIONS)

All simulation code, training loops, and visualizations assume the user has access to:

**Hardware**:
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM minimum) — Required for Isaac Sim
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9 (7000 series+)
- **RAM**: 64GB DDR5 (32GB absolute minimum, will cause crashes in complex scenes)
- **Storage**: 500GB SSD (for Isaac Sim assets, Gazebo models, ROS 2 packages)
- **OS**: Ubuntu 22.04 LTS (mandatory; ROS 2 Humble is native to Ubuntu)

**Software Stack**:
- NVIDIA CUDA Toolkit 12.2+
- NVIDIA cuDNN 8.6+
- ROS 2 Humble (or Iron, but Humble is default)
- Gazebo 11+ or Gazebo Garden
- NVIDIA Isaac Sim 4.0+

**Section 3.2: Edge Compute Device (Physical Hardware)**

For deploying trained models and running inference on "real" robots (or physical test rigs):

**Device**:
- NVIDIA Jetson Orin Nano (8GB) or Jetson Orin NX (16GB) — industry standard for embedded AI
- 64GB+ microSD card (UHS-II, high endurance)
- Power supply: 25W minimum (Orin Nano), 40W recommended (Orin NX)
- Cooling: Passive heatsink adequate; active cooling recommended for sustained use

**Sensors** (Optional but recommended):
- Intel RealSense D435i (RGB + Depth + IMU)
- USB microphone array (e.g., ReSpeaker) for Whisper integration
- Additional LiDAR/IMU optional for advanced modules

**Rationale**: Transparent hardware assumptions prevent learner surprise ("my laptop can't run this"). Jetson Orin Nano is <$300 and is THE standard for embodied AI.

---

### Section 3.3: Robot Targets (OPTIONAL FOR LEARNERS; REQUIRED FOR CONTENT EXAMPLES)

Content examples SHALL default to:

**Quadruped Option** (Recommended for cost):
- Unitree Go2 Edu (~$2,500–3,000)
- Pros: Durable, excellent ROS 2 SDK, ~10,000 units deployed globally
- Cons: Not a humanoid; bipedal locomotion examples don't apply directly

**Humanoid Option** (Premium):
- Unitree G1 (~$16,000) or H1 (~$90,000)
- Pros: Matches course theme; demonstrates sim-to-real for bipeds
- Cons: Expensive; inaccessible to most students

**Content Strategy**: Examples default to Go2 (quadruped); humanoid code shown as "bonus challenge" with notes on kinematic differences.

---

### Section 3.4: Cloud-Based Alternative (CONDITIONAL)

If a learner lacks RTX 4070 Ti, cloud-based simulation is permitted:
- AWS RoboMaker or NVIDIA Omniverse Cloud
- Cost disclaimer MANDATORY: "Cloud sim costs ~$X per hour. Estimated course cost: $Y."
- Latency warning: "Cloud latency ~500ms–2s. Real-time control not possible; suitable for training only."

**Rationale**: Acknowledges learners in bandwidth-limited or cost-constrained regions (Pakistan, South Asia). Transparency prevents surprise bills.

---

## ARTICLE IV: LIVING BOOK FEATURES (BONUS OBJECTIVES)

### Section 4.1: RAG Chatbot (BASE OBJECTIVE; MANDATORY)

**Embedding Integration**:
- RAG chatbot SHALL be embedded as a swizzled Docusaurus component (e.g., chatbot icon in bottom-right corner)
- Component available on every chapter page

**RAG Behavior**:
- Query: User asks a question (e.g., "How do I create a ROS 2 node?")
- Retrieval: Backend queries Qdrant for semantically similar chunks (top-5 results)
- Generation: FastAPI calls OpenAI with context + user query
- Response: Chat response with source citations (Chapter X, Section Y)

**Quality Gate**:
- Semantic similarity threshold: ≥0.85 (cosine distance)
- Source citation: MANDATORY in every response
- Response format: "Based on Chapter 1 (Section 1.2): [answer]"
- Graceful degradation: If no relevant context, respond with "I don't have specific information on this in the textbook. Please refer to Chapter X or ask an instructor."
- Weekly auditing: 5% random sample of responses manually verified for hallucinations; escalation protocol if detected

**Rationale**: RAG ensures answers are grounded in textbook content. Source citations make answers verifiable. Weekly audits catch drift.

---

### Section 4.2: Personalization (BONUS OBJECTIVE; 50 POINTS)

**Profile Dimensions** (from Better-Auth onboarding):
1. **Software Background**: Beginner / Intermediate / Advanced
2. **Hardware Background**: No robotics / ROS 1 / ROS 2 / Gazebo
3. **Target Robot**: Quadruped / Humanoid / Unsure
4. **Language**: English / Urdu

**Personalization Button** (MANDATORY):
- Button at top-left of every chapter: "Personalize Content"
- Toggles between:
  - **Software Engineer View**: Emphasizes algorithms, math, code design patterns
  - **Hardware Engineer View**: Emphasizes mechanical design, motor control, sensor integration
  - (Third view optional: "Beginner" / "Advanced" code examples)

**Implementation**:
- Profile stored in `user_profiles` table
- On chapter load, filter code examples based on profile
- Example: If "Hardware Engineer", show motor PWM details; if "Software Engineer", show ROS middleware abstractions
- No page reload required; CSS `display: none` or conditional rendering

**Content Authoring**:
- Authors tag code examples: `<!-- [SOFTWARE_ENGINEER] -->` and `<!-- [HARDWARE_ENGINEER] -->`
- Docusaurus build-time or runtime filtering

**Rationale**: Learners have different backgrounds; personalization increases relevance and engagement. 50-point bonus incentivizes completion.

---

### Section 4.3: Urdu Translation (BONUS OBJECTIVE; 50 POINTS)

**Localization Scope**:
- Chapter prose (explanations, learning objectives, key takeaways) translated to Urdu
- Code examples: REMAIN IN ENGLISH (standards require English syntax)
- Technical terminology: Maintain glossary (English ↔ Urdu mapping)
- Diagrams: Labels translated to Urdu (if non-technical), diagrams themselves remain unchanged

**Translation Quality Gate**:
- Native Urdu speaker (fluent; ideally from Pakistan/South Asia)
- Domain expert (roboticist or engineer) must review translated content
- RTL (right-to-left) layout verified: Docusaurus config must set `dir="rtl"` for Urdu pages
- Technical terms consistency: All instances of "Node", "Topic", "Service" translated identically

**Localization Button** (MANDATORY):
- Button at top-right of every chapter: "عربی / Urdu" or "English / اردو"
- Toggles between English and Urdu prose (code remains English)
- User preference saved in `user_profiles.language` field

**Deployment**:
- Docusaurus versioning: `/docs/module-1/chapter-1/` (English) and `/docs/ur/module-1/chapter-1/` (Urdu)
- Or: Single-language site with runtime translation (less preferred; server-side rendering preferred)

**Rationale**: Urdu translation serves Pakistan's AI engineering market (Panaversity target region). Transparency that code remains English prevents learner confusion.

---

## ARTICLE V: PERFORMANCE & RELIABILITY STANDARDS

### Section 5.1: RAG API Latency (MANDATORY)

- **p95 latency**: <2 seconds (end-to-end, from user query to response display)
- **p99 latency**: <4 seconds
- **Timeout**: 5 seconds hard timeout (return error if exceeds)

**Measurement**: Log all RAG queries with timestamp; compute percentiles weekly.

---

### Section 5.2: Personalization API Latency (MANDATORY)

- **p95 latency**: <500 milliseconds
- **p99 latency**: <1 second
- **Timeout**: 2 seconds hard timeout

**Rationale**: Personalization must be snappy (users expect instant toggles). Timeouts prevent hanging requests.

---

### Section 5.3: Uptime & Availability (MANDATORY)

- **Target**: 99% uptime (52.6 minutes downtime allowed per month)
- **Deployment frequency**: At least weekly (continuous delivery)
- **Deployment strategy**: Blue-green (zero-downtime deployments)

---

## ARTICLE VI: CONTENT VERSIONING & MIGRATION

### Section 6.1: Chapter Versioning

**Major Content Changes** (e.g., restructuring, significant corrections):
- Increment version: `Chapter 3 v1.0 → v2.0`
- Email notification to all active learners: "[Content Update] Module X, Chapter Y has been revised. Review changes?"
- Learners who completed the chapter MAY review changes at their discretion
- Completion credits preserved (no re-taking required)

**Minor Content Changes** (typos, clarifications, example updates):
- No version bump
- No learner notification
- Auto-propagate to all instances

**Backward Compatibility**:
- Deprecated code examples tagged: `[LEGACY — See updated version below]`
- Provide side-by-side old/new code with migration guide

---

## ARTICLE VII: FINANCIAL CONSTRAINTS & COST GOVERNANCE

### Section 7.1: Operational Budget (IMMUTABLE)

**Monthly limit**: $100 USD (all services combined)

**Budget allocation** (suggested):
- OpenAI API (embeddings + chat): $30–40/month
- Qdrant Cloud: $0 (free tier)
- Neon Postgres: $10–15/month
- Better-Auth: $0 (free tier)
- Vercel/Railway/Render: $10–20/month
- Miscellaneous: $10–15/month

**Monitoring**: Backend logs all API calls with cost estimates; monthly cost report generated automatically.

**Overage Alert**: If projected monthly cost exceeds $80, automatic alert to project lead.

---

### Section 7.2: Learner Cost Transparency (MANDATORY)

Every chapter MUST document:
1. **Hardware cost**: "This chapter requires [RTX 4070 Ti (~$2,500) OR cloud rental (~$10–50)]"
2. **Software cost**: "All software is free and open-source"
3. **Cloud costs**: If applicable, estimated hourly cost and total course cost
4. **Robotics hardware cost**: "Optional. Unitree Go2 (~$2,500) recommended; Jetson Orin Nano (~$250) required for Module 3+"

**Rationale**: Transparent pricing removes barriers for students in underserved regions.

---

## ARTICLE VIII: SUBAGENTS & REUSABLE INTELLIGENCE GOVERNANCE

### Section 8.1: Scope Definition (MANDATORY)

Subagents (bonus objective; up to 50 points) MUST:
- Solve a **single, well-defined pedagogical problem** (e.g., "VLA Model Selection Advisor")
- NOT be a generic chatbot or search engine
- Have clear input/output contracts
- Include failure mode documentation

**Examples of valid subagents**:
- "ROS 2 Error Debugger" — Takes error log → Returns debugging steps
- "Hardware Compatibility Checker" — Takes user hardware specs → Returns feasibility + cost estimate
- "Gazebo Model Converter" — Takes CAD file → Returns URDF with annotations

**Examples of invalid subagents**:
- "General chatbot" — Too vague
- "Wikipedia searcher" — Not pedagogical
- "Email client" — Out of scope

---

### Section 8.2: Subagent Versioning & Integration

- **Versioning**: Semantic versioning (e.g., `vla-advisor@1.2.0`)
- **Integration gate**: Subagent integrated ONLY if:
  1. Spec written (input/output/limitations documented)
  2. Contract tests pass (predictable behavior on known inputs)
  3. Performance meets SLOs (p95 <3 seconds)
  4. Documentation clear (when to use, failure modes)
- **Deprecation**: If unused for 2+ months, deprecated with 30-day notice

---

## ARTICLE IX: COMPLIANCE & ENFORCEMENT

### Section 9.1: Compliance Checkpoints

**Weekly**:
- [ ] Task completion + unblocked items
- [ ] Learner feedback volume/sentiment reviewed
- [ ] Cost audit: monthly projection ≤$100

**Chapter-End**:
- [ ] Content approved by domain expert
- [ ] RAG index built and spot-checked (5% sample)
- [ ] Tests passing (70%+ code coverage for new Python modules)
- [ ] Accessibility audit: alt text present, captions present, readability ≤grade 10

**Module-End**:
- [ ] All chapters in module complete
- [ ] Urdu translation (if applicable) validated by native speaker
- [ ] Subagent integrations documented in ADRs
- [ ] Performance benchmarks met (RAG p95 <2s, uptime ≥99%)

**Pre-Delivery (Final Textbook)**:
- [ ] All 5 modules + capstone deployed
- [ ] All bonus objectives evaluated (RAG, auth, personalization, subagents, Urdu)
- [ ] Hallucination audit: Zero hallucinations in final spot-check
- [ ] Learner feedback: >4/5 stars on satisfaction survey
- [ ] Cost tracking: <$100/month demonstrated

---

### Section 9.2: Amendment Protocol

This Constitution may be amended ONLY via:
1. **Proposal**: Written justification (rationale, impact, risk mitigation)
2. **Review**: Project Lead + Tech Lead approval
3. **Merge**: Dedicated PR with commit message `docs: amend sp.constitution.md to v[X.Y.Z]`
4. **Cascade**: Affected specs, plans, tasks re-evaluated

**Version Numbering**:
- MAJOR: Removal/fundamental redefinition of Article
- MINOR: New Article or material expansion
- PATCH: Clarifications, wording fixes

---

## ARTICLE X: SUPREME AUTHORITY

This Constitution is the final authority on all project decisions. In case of ambiguity:
1. Consult `.specify/memory/constitution.md` (detailed guidance)
2. If still unclear, use `/sp.clarify` command to resolve
3. Escalate to Project Lead only if above fails

**Immutable Sections** (cannot be amended without MAJOR version bump):
- Article I (Stack Mandate)
- Article II (Curriculum Structure, Modules 1–4 + Capstone)
- Article III (Hardware Constraints)
- Article IV (Living Book Features: RAG, Auth, Personalization, Urdu)

---

## SIGNATURE

**Approved by**: Chief Architect
**Date**: 2025-11-29
**Version**: 1.0.0
**Status**: EFFECTIVE IMMEDIATELY

This Constitution is binding on all `/sp.plan`, `/sp.tasks`, and `/sp.implement` agents. No exceptions.

---

**Next Steps for Agents**:
1. `/sp.plan` agent: Read this constitution and generate directory structure accordingly
2. `/sp.tasks` agent: Ensure all tasks comply with Article IX (Compliance Checkpoints)
3. `/sp.implement` agent: Verify implementation against all Articles, especially Article I (Stack) and Article II (Curriculum)
