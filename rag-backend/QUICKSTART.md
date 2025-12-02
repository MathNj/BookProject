# RAG Backend Quick Start Guide

Get the RAG backend running in 5 minutes!

## Prerequisites

- Python 3.11+
- Poetry (Python package manager)
- Docker (for Qdrant)
- OpenAI API key

## Step 1: Get Your OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with `sk-`)
4. ⚠️ **Never share this key!**

## Step 2: Clone and Setup

```bash
# Navigate to RAG backend directory
cd rag-backend

# Copy environment template
cp .env.example .env

# Edit .env and add your OpenAI API key
nano .env
# Add this line:
# OPENAI_API_KEY=sk-your-key-here
```

## Step 3: Install Dependencies

```bash
# Install Python dependencies
poetry install

# This installs:
# - FastAPI (web framework)
# - OpenAI client (embeddings)
# - Qdrant client (vector database)
# - And more...
```

## Step 4: Start Qdrant Vector Database

In a new terminal:

```bash
# Option 1: Docker (Recommended)
docker run -p 6333:6333 qdrant/qdrant:latest

# Option 2: Local installation (skip Docker)
# See https://qdrant.tech/documentation/

# Verify it's running:
curl http://localhost:6333/health
```

## Step 5: Ingest Textbook

In the rag-backend directory:

```bash
poetry run python scripts/ingest_book.py
```

**Expected output:**
```
🚀 Starting book ingestion...
✓ Found 27 Markdown files
✓ Ingested 45 chunks from 01-intro.md
...
📊 INGESTION SUMMARY
Total files processed: 27
Total chunks ingested: 1,243
Failed files: 0
✓ Collection: textbook_content
✓ Total points: 1,243
```

## Step 6: Start RAG Backend

In the rag-backend directory:

```bash
poetry run python -m src.main
```

**Expected output:**
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Application startup complete
```

## Step 7: Test the System

In a new terminal:

### Test Health Check
```bash
curl http://localhost:8000/health
```

### Test Search
```bash
curl -X POST "http://localhost:8000/search?query=ROS%202&limit=3"
```

### Test Chat
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "What is ROS 2?"}
    ]
  }'
```

## Step 8: Interactive API Documentation

Open your browser to:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

Try requests interactively!

## What's Running

```
Frontend (React)
    ↓ HTTP
    ↓ /chat, /search
    ↓
RAG Backend (FastAPI) ← You are here
    ↓ OpenAI API
    ↓ Embeddings (text-embedding-3-small)
    ↓
Qdrant Vector Database ← Running in Docker
    ↓ Stores embeddings
    ↓
Textbook Content ← Ingested from /docs-website/docs
```

## Common Commands

```bash
# Start everything in background (macOS/Linux)
(poetry run python -m src.main &) && sleep 2 && \
docker run -p 6333:6333 qdrant/qdrant:latest &

# Stop everything
pkill -f "python -m src.main"
docker stop $(docker ps -q)

# Check logs
tail -f ~/.cache/poetry/...

# Reset Qdrant (delete all data)
docker exec qdrant rm -rf /qdrant/storage/collections
```

## Troubleshooting

### Error: "OPENAI_API_KEY not found"

```bash
# Check if .env exists
ls -la .env

# Check if key is set
cat .env | grep OPENAI_API_KEY

# If not, add it:
echo "OPENAI_API_KEY=sk-your-key" >> .env
```

### Error: "Connection refused" (Qdrant)

```bash
# Ensure Docker is running
docker ps

# Start Qdrant if not running
docker run -p 6333:6333 qdrant/qdrant:latest
```

### Error: "No embeddings found"

```bash
# Re-ingest textbook
poetry run python scripts/ingest_book.py

# Check if data was saved
curl http://localhost:6333/collections/textbook_content
```

### Error: "Port 8000 already in use"

```bash
# Find what's using port 8000
lsof -i :8000

# Kill the process
kill -9 <PID>

# Or use a different port
poetry run python -m src.main --port 8001
```

## Cost Estimation

Your first run:

1. **Ingestion:** ~$0.005 (one-time, ~1,000 textbook chunks)
2. **First 10 queries:** ~$0.00001 (negligible)

**Monthly (if running 24/7):**
- 1000 queries/day = ~$0.03/month
- Total = ~$0.03-0.05/month

Very affordable! ✨

## Next Steps

1. ✅ **Setup complete!** You now have a working RAG system
2. 📚 **Read**: Check out [RAG_IMPLEMENTATION.md](./RAG_IMPLEMENTATION.md) for deep dive
3. 📖 **Reference**: See [API_REFERENCE.md](./API_REFERENCE.md) for all endpoints
4. 🔗 **Integrate**: Connect frontend ChatWidget to `/chat` endpoint
5. 🚀 **Deploy**: Follow deployment guide in RAG_IMPLEMENTATION.md

## Environment Variables (Complete Reference)

```env
# Required: OpenAI API Key
OPENAI_API_KEY=sk-your-key-here

# Optional: Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Only if using Qdrant Cloud

# Optional: Model Configuration
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gemini-2.5-flash

# Optional: Frontend Configuration
FRONTEND_URL=http://localhost:3000
API_PORT=8000

# Optional: Logging
LOG_LEVEL=INFO
```

## Directory Structure

```
rag-backend/
├── src/
│   ├── main.py                 # FastAPI app (endpoints)
│   ├── agent.py                # RAG Agent (search tool)
│   ├── config.py               # Configuration
│   ├── schemas.py              # Data models
│   └── services/
│       └── vector_store.py     # OpenAI embeddings + Qdrant
├── scripts/
│   └── ingest_book.py          # Ingest textbook chunks
├── .env.example                # Environment template
├── pyproject.toml              # Dependencies
├── QUICKSTART.md               # This file
├── RAG_IMPLEMENTATION.md       # Full documentation
└── API_REFERENCE.md            # API docs
```

## Need Help?

1. **Setup Issues**: Check [RAG_IMPLEMENTATION.md](./RAG_IMPLEMENTATION.md#troubleshooting)
2. **API Questions**: See [API_REFERENCE.md](./API_REFERENCE.md)
3. **Code Issues**: Check FastAPI docs: https://fastapi.tiangolo.com/
4. **OpenAI Issues**: Check: https://platform.openai.com/docs/guides/embeddings

## Success Checklist

- ✅ OpenAI API key in `.env`
- ✅ `poetry install` completed
- ✅ Qdrant running on `localhost:6333`
- ✅ Ingestion script completed (1000+ chunks)
- ✅ Backend running on `localhost:8000`
- ✅ `/health` returns 200 OK
- ✅ `/search` returns relevant results
- ✅ `/chat` returns AI responses

If all checked, **you're ready to use the RAG system!** 🎉
