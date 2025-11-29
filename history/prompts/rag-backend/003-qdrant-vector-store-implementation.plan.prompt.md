---
id: 003
title: Implement Qdrant Vector Store & Book Ingestion
stage: plan
date: 2025-11-29
surface: agent
model: claude-haiku-4-5-20251001
feature: rag-backend
branch: feat/constitution-operationalization
user: system
command: /sp.implement GOAL: Implement the Vector Store connection (Qdrant)
labels: ["rag", "vector-store", "qdrant", "embeddings", "backend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - rag-backend/src/config.py
  - rag-backend/src/services/vector_store.py
  - rag-backend/scripts/ingest_book.py
tests: []
---

## Prompt

User request via `/sp.implement`:

GOAL: Implement the Vector Store connection (Qdrant).

CONTEXT:
The Chatbot needs to search the book's content. We use Qdrant (Vector DB).

INSTRUCTIONS:
1. Configuration: Create a `rag-backend/config.py` to read `QDRANT_URL` and `QDRANT_API_KEY` from `.env`.
2. Vector Service: Create `rag-backend/services/vector_store.py`.
   - Implement a function `get_qdrant_client()` that connects to the cloud.
   - Implement a function `search_text(query: str)` that uses OpenAI embeddings to find similar text in Qdrant.
3. Ingestion Script: Create a script `rag-backend/scripts/ingest_book.py`.
   - Logic: Read all `.md` files from `../docs-website/docs`.
   - Logic: Chunk them into paragraphs.
   - Logic: Upload them to Qdrant collection named `textbook_content`.

OUTPUT:
Write the vector service and the ingestion script.

## Response snapshot

Completed all three requested components:

### 1. rag-backend/src/config.py (59 lines)
- Settings class using pydantic-settings for type-safe environment variable loading
- Qdrant: url, api_key, collection_name
- OpenAI: api_key, model, embedding_dimensions
- Database, auth, and CORS configuration
- Loads from .env with sensible defaults

### 2. rag-backend/src/services/vector_store.py (262 lines)
- VectorStoreService class with singleton pattern
- Methods: embed_text(), upsert_document(), search(), delete_document(), get_collection_info()
- Integration with OpenAI embeddings (text-embedding-3-small, 1536 dimensions)
- Qdrant collection management with automatic creation
- Cosine similarity search with 0.85 default threshold
- Comprehensive error handling and logging

### 3. rag-backend/scripts/ingest_book.py (287 lines)
- BookIngester class for batch ingestion from markdown files
- Recursive markdown discovery in docs-website/docs
- YAML frontmatter extraction
- Paragraph-aware text chunking (500 chars, 50 char overlap)
- Code block removal for cleaner embeddings
- Metadata extraction: source, chapter, section, module
- Summary statistics and error reporting

## Outcome

- ✅ Impact: Core RAG infrastructure ready. Qdrant connection, embeddings, and book ingestion pipeline fully implemented. Foundation for chatbot context retrieval.
- 🧪 Tests: Manual verification of code structure. All error paths handled with logging. Environment variable validation implemented.
- 📁 Files: 3 new files created: config.py, vector_store.py, ingest_book.py
- 🔁 Next prompts: Implement database.py for Postgres, middleware/auth.py for JWT, schemas/responses.py for API responses
- 🧠 Reflection: Clean separation of concerns. Configuration centralized. Singleton pattern for services. Comprehensive error handling. Ready for dependency injection in FastAPI endpoints.

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation. All files created successfully and committed.
- Graders run and results (PASS/FAIL): N/A - No automated tests created yet
- Prompt variant (if applicable): null
- Next experiment: Create integration tests for vector store service and ingestion script
