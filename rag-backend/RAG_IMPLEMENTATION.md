# RAG Implementation with OpenAI Text Embeddings

This document describes the Retrieval-Augmented Generation (RAG) system implementation using OpenAI's text-embedding-3 models.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Frontend (React/Docusaurus)              │
│                                                               │
│  ChatWidget communicates with RAG Backend via HTTP          │
└──────────────────────────┬──────────────────────────────────┘
                           │ POST /chat
                           │ POST /search
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    FastAPI RAG Backend                       │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Chat Endpoint (/chat)                                │  │
│  │ - Receives user messages                             │  │
│  │ - Routes to RAG Agent                                │  │
│  │ - Returns AI-generated response                      │  │
│  └────────────────────┬─────────────────────────────────┘  │
│                       │                                      │
│  ┌────────────────────▼──────────────────────────────────┐  │
│  │ RAG Agent (OpenAI Agents SDK + Gemini 2.5-flash)    │  │
│  │ - search_book_tool: Semantic search function         │  │
│  │ - System prompt: Context-aware assistant            │  │
│  └────────────────────┬──────────────────────────────────┘  │
│                       │                                      │
│  ┌────────────────────▼──────────────────────────────────┐  │
│  │ Vector Store Service (VectorStoreService)            │  │
│  │ - embed_text(): OpenAI embedding API                │  │
│  │ - search(): Qdrant semantic search                   │  │
│  │ - upsert_document(): Add/update vectors             │  │
│  └────────────────────┬──────────────────────────────────┘  │
│                       │                                      │
│                       ▼                                      │
│  ┌────────────────────────────────────────────────────────┐  │
│  │ OpenAI Embeddings API                                │  │
│  │                                                        │  │
│  │ Model: text-embedding-3-small (1536 dimensions)     │  │
│  │ - High-quality, cost-effective embeddings            │  │
│  │ - 625K tokens per min throughput                     │  │
│  │ - Excellent for semantic search                      │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────┬──────────────────────────────────────────────┘
               │
┌──────────────▼──────────────────────────────────────────────┐
│                    Qdrant Vector Database                    │
│                                                               │
│  Collection: textbook_content                               │
│  - Vector dimensions: 1536 (OpenAI embedding size)          │
│  - Distance metric: Cosine similarity                       │
│  - Points: One for each document chunk                      │
│  - Metadata: source, chapter, section                       │
│                                                               │
│  Payload:                                                    │
│  {                                                            │
│    "doc_id": "chapter_01_section_2",                        │
│    "text": "First 500 chars of content...",                 │
│    "source": "01-nervous-system.md",                        │
│    "chapter": "The Nervous System (ROS 2)",                │
│    "section": "Section 1"                                   │
│  }                                                            │
└──────────────────────────────────────────────────────────────┘
```

## OpenAI Integration Details

### Embedding Model: text-embedding-3-small

**Why this model?**
- **High quality**: Latest OpenAI embedding model with improved semantic understanding
- **Cost-effective**: Cheaper than text-embedding-3-large
- **Optimal dimensions**: 1536-dimensional vectors (vs 3072 for large)
- **Fast inference**: 625K tokens/min throughput
- **Well-tested**: Proven performance for RAG applications

**Specifications:**
```
Model: text-embedding-3-small
Dimensions: 1536
Max input tokens: 8191
Cost: $0.02 per 1M tokens
Pricing example: 1M textbook words = ~$0.10-0.30
```

### Vector Database Configuration

Qdrant is configured with OpenAI embedding dimensions:

```python
VectorParams(
    size=1536,  # text-embedding-3-small produces 1536-dim vectors
    distance=Distance.COSINE,  # Cosine similarity metric
)
```

### API Keys Required

1. **OpenAI API Key** (for embeddings)
   - Used for: `text-embedding-3-small` model
   - Cost: Pay-as-you-go embeddings
   - Get it: https://platform.openai.com/api-keys

2. **Qdrant Connection** (for vector storage)
   - Local: `http://localhost:6333` (default)
   - Cloud: Qdrant Cloud account with API key

3. **Gemini API Key** (for chat, optional if using OpenAI chat)
   - Used for: Chat responses (fallback)
   - Get it: https://makersuite.google.com/app/apikey

## File Structure

```
rag-backend/
├── src/
│   ├── main.py                      # FastAPI app with /chat and /search endpoints
│   ├── config.py                    # Configuration management (environment variables)
│   ├── agent.py                     # RAG Agent with search_book_tool
│   ├── schemas.py                   # Pydantic models for requests/responses
│   └── services/
│       ├── vector_store.py          # VectorStoreService with OpenAI embeddings
│       └── gemini_client.py         # Gemini API client (chat)
├── scripts/
│   └── ingest_book.py               # Book ingestion script (chunks + OpenAI embeddings)
├── pyproject.toml                   # Poetry dependencies
├── .env.example                     # Environment variables template
├── RAG_IMPLEMENTATION.md            # This file
└── README.md                        # Original README
```

## Setup Instructions

### 1. Install Dependencies

```bash
cd rag-backend
poetry install
```

### 2. Configure Environment Variables

Create `.env` file from `.env.example`:

```bash
cp .env.example .env
```

Update with your API keys:

```env
# Required: OpenAI API Key for embeddings
OPENAI_API_KEY=sk-your-openai-api-key

# Required: Qdrant connection
QDRANT_URL=http://localhost:6333  # Local setup
# OR for cloud:
# QDRANT_URL=https://your-qdrant-instance.qdrant.io
# QDRANT_API_KEY=your-api-key

# Optional: Gemini API for chat
API_KEY=your-gemini-api-key

# Optional: Embedding model (default: text-embedding-3-small)
EMBEDDING_MODEL=text-embedding-3-small
```

### 3. Start Qdrant Vector Database

**Local Setup (Docker):**

```bash
docker run -p 6333:6333 qdrant/qdrant:latest
```

**Cloud Setup (Qdrant Cloud):**

1. Create account at https://qdrant.tech/
2. Create project and get API key
3. Update `QDRANT_URL` and `QDRANT_API_KEY` in `.env`

### 4. Ingest Textbook Content

Before the RAG system works, you need to ingest the textbook chapters:

```bash
# From rag-backend directory
poetry run python scripts/ingest_book.py
```

**What happens:**
1. Scans `docs-website/docs/` for markdown files
2. Extracts frontmatter (title, chapters)
3. Chunks text into 500-character pieces
4. Generates OpenAI embeddings for each chunk
5. Uploads to Qdrant vector store
6. Prints summary with chunk count

**Expected output:**
```
🚀 Starting book ingestion...
✓ Found 27 Markdown files
✓ Ingested 123 chunks from 01-intro.md
✓ Ingested 89 chunks from 02-digital-twin.md
...
📊 INGESTION SUMMARY
Total files processed: 27
Total chunks ingested: 1,243
Failed files: 0
✓ Collection: textbook_content
✓ Total points: 1,243
```

### 5. Start RAG Backend

```bash
# Development mode with auto-reload
poetry run python -m src.main

# Or with uvicorn directly
poetry run uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

**Check it's running:**

```bash
curl http://localhost:8000/health
# Response: {"status":"ok","service":"rag-backend","version":"0.1.0"}
```

### 6. Test the System

**Chat Endpoint:**

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "messages": [
      {"role": "user", "content": "What is ROS 2?"}
    ]
  }'
```

**Search Endpoint:**

```bash
curl -X POST "http://localhost:8000/search?query=ROS%202&limit=3"
```

## Cost Estimation

### OpenAI Embeddings

**Pricing:** $0.02 per 1M input tokens

**Textbook costs:**
- 27 markdown files
- ~1,000 chunks (500 chars each)
- ~250K tokens total
- **One-time ingestion cost: ~$0.005 USD**

**Per query cost:**
- Search query: ~50 tokens
- **Cost per query: ~$0.000001 USD (negligible)**

**Monthly estimate (assuming 1000 queries/day):**
```
Ingestion:        $0.005  (one-time)
Queries:          $0.03   (1000/day × 30 days × $0.000001)
Total monthly:    ~$0.04 USD
```

### Qdrant Cloud (Optional)

- **Free tier:** Up to 1GB storage
- **Paid:** Starting at $9.99/month for 10GB
- Sufficient for textbook with thousands of chunks

## Configuration Reference

All settings are in `src/config.py`:

| Setting | Default | Description |
|---------|---------|-------------|
| `OPENAI_API_KEY` | `""` | OpenAI API key for embeddings |
| `EMBEDDING_MODEL` | `text-embedding-3-small` | OpenAI embedding model |
| `QDRANT_URL` | `http://localhost:6333` | Qdrant connection URL |
| `QDRANT_COLLECTION_NAME` | `textbook_content` | Collection name in Qdrant |
| `CHAT_MODEL` | `gemini-2.5-flash` | Chat model (Gemini) |
| `FRONTEND_URL` | `http://localhost:3000` | Frontend URL for CORS |

## Key Classes

### VectorStoreService (services/vector_store.py)

```python
class VectorStoreService:
    def embed_text(text: str) -> List[float]
        # Generate OpenAI embedding for text

    def search(query: str, limit: int, min_similarity: float) -> List[Dict]
        # Semantic search in Qdrant

    def upsert_document(doc_id: str, text: str, metadata: Dict) -> None
        # Add/update document in vector store

    def get_collection_info() -> Dict
        # Get collection statistics
```

### RAGAgent (agent.py)

```python
class RAGAgent:
    async def run_sync(messages: List[Dict]) -> str
        # Run agent with search_book_tool
        # Returns AI-generated response
```

## Troubleshooting

### Issue: "OpenAI API Key not found"

**Solution:**
```bash
# Check .env file exists
ls -la .env

# Verify OPENAI_API_KEY is set
echo $OPENAI_API_KEY

# If not set, add to .env and reload
export OPENAI_API_KEY=sk-your-key
```

### Issue: "Qdrant connection failed"

**Solution:**
```bash
# Check if Qdrant is running
curl http://localhost:6333/health

# If not, start Docker container
docker run -p 6333:6333 qdrant/qdrant:latest
```

### Issue: "No embedding found after ingestion"

**Solution:**
```bash
# Re-run ingestion
poetry run python scripts/ingest_book.py

# Check collection
curl http://localhost:6333/collections/textbook_content
```

### Issue: "Search returns empty results"

**Solution:**
- Ensure ingest completed successfully
- Check `min_similarity` threshold (default 0.85 is strict)
- Verify Qdrant has points: `curl http://localhost:6333/collections/textbook_content`

## Performance Metrics

### Ingestion

- **Speed:** ~500 chunks/minute (depends on OpenAI API rate limits)
- **Total time for textbook:** ~3-5 minutes
- **Cost:** ~$0.005 USD

### Search/Chat

- **Query latency:**
  - OpenAI embedding: 100-300ms
  - Qdrant search: 10-50ms
  - Gemini chat: 1-3 seconds
  - **Total:** ~2-4 seconds per query

- **Throughput:**
  - Supported: 1000+ concurrent queries (limited by OpenAI rate limits)
  - Rate limit: 625K tokens/min (~10K queries/min for search)

## Future Enhancements

1. **Batch Embeddings**: Use batch API for faster ingestion
2. **Caching**: Cache embeddings for common queries
3. **Reranking**: Use a reranker to improve result quality
4. **Hybrid Search**: Combine semantic + keyword search
5. **User Feedback Loop**: Track query effectiveness
6. **Multi-language**: Support Urdu and other languages
7. **Custom Embeddings**: Fine-tune embeddings on domain-specific data

## References

- [OpenAI Embeddings API](https://platform.openai.com/docs/guides/embeddings)
- [text-embedding-3 Models](https://platform.openai.com/docs/guides/embeddings/embedding-models)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [OpenAI Agents SDK](https://github.com/openai/openai-python)
