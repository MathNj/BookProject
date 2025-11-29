# Physical AI & Humanoid Robotics Textbook

An AI-native, interactive textbook teaching embodied intelligence with real robots and simulations.

## Features

- 📚 **5 Curriculum Modules**: ROS 2 → Digital Twin → Isaac Sim → VLA → Capstone
- 🤖 **RAG Chatbot**: "Ask the Book" widget for context-aware Q&A grounded in chapter content
- 🔐 **Authentication**: User signup with hardware/software profile
- 🎯 **Personalization**: Chapter intros rewritten based on user background
- 🌍 **Localization**: English + Urdu with RTL layout support
- ⚡ **CI/CD**: Automated chapter deployment, RAG indexing, GitHub Pages publishing

## Hardware Requirements

### Primary Workstation
- **GPU**: NVIDIA RTX 4070 Ti (or equivalent)
- **RAM**: 64GB
- **OS**: Ubuntu 22.04 LTS

### Edge Device
- **NVIDIA Jetson Orin Nano** or **Jetson Orin NX**

### Target Robot
- **Unitree Go2** (quadruped) or **Unitree G1** (humanoid)

### Cloud Alternative
- Google Colab, Paperspace, or AWS (for labs without local GPU)

## Project Structure

```
BookProject/
├── docs-website/               # Frontend: Docusaurus v3+
│   ├── docs/                   # Curriculum chapters
│   │   ├── 01-nervous-system/
│   │   ├── 02-digital-twin/
│   │   ├── 03-robot-brain/
│   │   ├── 04-the-mind/
│   │   └── 05-capstone/
│   ├── src/
│   │   ├── components/         # RAGWidget, SignupForm, etc.
│   │   ├── hooks/              # useRAGQuery, useAuth, etc.
│   │   └── pages/              # Landing page
│   └── docusaurus.config.ts    # Site config (i18n, URLs, etc.)
│
├── rag-backend/                # Backend: FastAPI
│   ├── src/
│   │   ├── main.py             # FastAPI app entry point
│   │   ├── config.py           # Environment variables
│   │   ├── database.py         # Neon Postgres connection
│   │   ├── models/             # SQLAlchemy models (User, Chapter, RAGEmbedding)
│   │   ├── schemas/            # Pydantic models (request/response)
│   │   ├── services/           # RAG, Auth, Personalization services
│   │   ├── endpoints/          # API routes (/rag, /auth, /profile)
│   │   └── middleware/         # JWT auth middleware
│   ├── scripts/
│   │   └── ingest_chapters.py  # RAG indexing script
│   └── tests/
│       └── integration/        # End-to-end tests
│
├── .github/
│   └── workflows/
│       └── build-deploy.yml    # GitHub Actions CI/CD
│
├── specs/
│   └── 001-ai-textbook/        # Specification & tasks
│       ├── spec.md             # Requirements
│       └── tasks.md            # Task breakdown
│
└── .env.example                # Environment variables template
```

## Quick Start

### Prerequisites
- Node.js 18+ and npm
- Python 3.11+
- Poetry (Python dependency manager)

### Setup Frontend

```bash
cd docs-website
npm install
npm start
```

Visit `http://localhost:3000` to see the site.

### Setup Backend

```bash
cd rag-backend
poetry install
poetry run uvicorn src.main:app --reload
```

Backend runs at `http://localhost:8000`.

### Environment Variables

Copy `.env.example` to `.env` and fill in your API keys:

```bash
cp .env.example .env
```

Required:
- `OPENAI_API_KEY`: OpenAI API key for embeddings + chat
- `QDRANT_URL` & `QDRANT_API_KEY`: Qdrant Cloud for vector search
- `DATABASE_URL`: Neon Postgres connection string

## Architecture

### Frontend (Static)
- **Docusaurus v3+** on GitHub Pages
- React components (RAGWidget, SignupForm, PersonalizeButton)
- Client-side Urdu translation toggle (no API calls)
- JWT stored in localStorage

### Backend (Serverless)
- **FastAPI** on Vercel/Railway
- **Neon Postgres** for users, chapters, personalization
- **Qdrant Cloud** for RAG embeddings
- **OpenAI API** for chat + embeddings

### Data Flow

```
User Question
    ↓
Frontend RAGWidget
    ↓
POST /api/rag/query (with JWT)
    ↓
Backend RAG Service
    ├─ Embed question (OpenAI)
    ├─ Search Qdrant (cosine similarity ≥0.85)
    ├─ Generate response (OpenAI + context)
    └─ Check for hallucinations
    ↓
RAGResponse { answer, source_citation, confidence }
    ↓
Frontend displays response + citation
```

## Cost Control

**Monthly Budget**: <$100 USD

- OpenAI API: Embeddings (~$2-5), Chat (~$5-10)
- Qdrant Cloud: Free tier (100M vectors)
- Neon Postgres: Free tier (<5GB storage)
- Better-Auth: Free tier
- GitHub Pages: Free
- Vercel/Railway: Free tier or ~$5-10/month

Weekly cost tracking: Monitor OpenAI usage dashboard.

## Deployment

### Frontend → GitHub Pages

```bash
cd docs-website
npm run build
npm run deploy
```

### Backend → Vercel/Railway

```bash
cd rag-backend
vercel deploy  # or railway deploy
```

### CI/CD Pipeline

GitHub Actions triggers on push to `main`:
1. Lint Markdown chapters
2. Build Docusaurus
3. Generate Urdu translations (manual review gate)
4. Ingest chapters to RAG (Qdrant)
5. Run performance benchmarks
6. Deploy frontend to GitHub Pages

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for:
- How to write chapters (Markdown structure, frontmatter)
- How to trigger Urdu translation
- How to test locally
- Git workflow

## Documentation

- [Specification](specs/001-ai-textbook/spec.md)
- [Task Breakdown](specs/001-ai-textbook/tasks.md)
- [API Reference](docs-website/docs/00-dev/api.md)
- [Deployment Guide](docs-website/docs/00-dev/deployment.md)

## License

[MIT License](LICENSE)

## Contact

For questions or collaboration, open a GitHub issue or email the maintainers.

---

**Version**: 1.0.0
**Last Updated**: 2025-11-29
**Status**: In Development (MVP Phase)
