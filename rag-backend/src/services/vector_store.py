"""
Vector Store Service using Qdrant Cloud.

Handles:
- Connection to Qdrant vector database
- Embedding generation using OpenAI
- Text search with semantic similarity
- Collection management
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
import logging
from typing import List, Dict, Any, Optional
from src.config import settings

logger = logging.getLogger(__name__)


class VectorStoreService:
    """Service for managing embeddings and vector search in Qdrant."""

    def __init__(self):
        """Initialize Qdrant and OpenAI clients."""
        # Defer Qdrant connection until first use (lazy initialization)
        self._qdrant_client = None
        self._qdrant_initialized = False
        # Use api_key if openai_api_key is empty (for Gemini key usage)
        api_key = settings.openai_api_key or settings.api_key
        self.openai_client = OpenAI(api_key=api_key)
        self.collection_name = settings.qdrant_collection_name
        logger.info("✓ VectorStoreService initialized (Qdrant connection deferred)")

    @property
    def qdrant_client(self) -> QdrantClient:
        """Lazy initialization of Qdrant client."""
        if self._qdrant_client is None:
            self._qdrant_client = self._get_qdrant_client()
            if not self._qdrant_initialized:
                self._ensure_collection_exists()
                self._qdrant_initialized = True
        return self._qdrant_client

    def _get_qdrant_client(self) -> QdrantClient:
        """
        Get or create Qdrant client connection.

        Returns:
            QdrantClient: Connected Qdrant client

        Raises:
            ValueError: If QDRANT_URL is not configured
        """
        if not settings.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

        try:
            # Create client - handles both local and cloud URLs
            client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )

            # Test connection
            client.get_collections()
            logger.info(f"✓ Connected to Qdrant at {settings.qdrant_url}")
            return client
        except Exception as e:
            logger.error(f"✗ Failed to connect to Qdrant: {e}")
            raise

    def _ensure_collection_exists(self) -> None:
        """Create Qdrant collection if it doesn't exist."""
        try:
            collections = self.qdrant_client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                self.qdrant_client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=settings.openai_embedding_dimensions,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"✓ Created collection: {self.collection_name}")
            else:
                logger.info(f"✓ Collection exists: {self.collection_name}")
        except Exception as e:
            logger.error(f"✗ Failed to ensure collection: {e}")
            raise

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for text using OpenAI.

        Args:
            text: Text to embed

        Returns:
            List[float]: Embedding vector (1536 dimensions)

        Raises:
            ValueError: If text is empty
            Exception: If OpenAI API call fails
        """
        if not text or not text.strip():
            raise ValueError("Text cannot be empty for embedding")

        try:
            response = self.openai_client.embeddings.create(
                input=text.strip(),
                model=settings.openai_model,
            )
            embedding = response.data[0].embedding
            logger.debug(f"✓ Generated embedding for {len(text)} chars")
            return embedding
        except Exception as e:
            logger.error(f"✗ Embedding generation failed: {e}")
            raise

    def upsert_document(
        self,
        doc_id: str,
        text: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Add or update a document in the vector store.

        Args:
            doc_id: Unique document identifier
            text: Document content to embed and store
            metadata: Optional metadata (chapter, section, source, etc.)

        Raises:
            ValueError: If doc_id or text is empty
        """
        if not doc_id or not text:
            raise ValueError("doc_id and text are required")

        try:
            # Generate embedding
            embedding = self.embed_text(text)

            # Create point with metadata
            point = PointStruct(
                id=hash(doc_id) % (10**8),  # Convert to positive int for Qdrant
                vector=embedding,
                payload={
                    "doc_id": doc_id,
                    "text": text[:500],  # Store first 500 chars for reference
                    "source": metadata.get("source") if metadata else "unknown",
                    "chapter": metadata.get("chapter") if metadata else "unknown",
                    "section": metadata.get("section") if metadata else "unknown",
                },
            )

            # Upsert to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=[point],
            )
            logger.info(f"✓ Upserted document: {doc_id}")
        except Exception as e:
            logger.error(f"✗ Failed to upsert document {doc_id}: {e}")
            raise

    def search(
        self,
        query: str,
        limit: int = 3,
        min_similarity: float = 0.85,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar documents using semantic search.

        Args:
            query: Search query text
            limit: Maximum number of results (default: 3)
            min_similarity: Minimum cosine similarity threshold (0-1, default: 0.85)

        Returns:
            List of search results with:
            - text: Document text snippet
            - score: Similarity score (0-1)
            - metadata: source, chapter, section

        Raises:
            ValueError: If query is empty
        """
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        try:
            # Generate query embedding
            query_embedding = self.embed_text(query)

            # Search in Qdrant
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=min_similarity,
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "text": result.payload.get("text", ""),
                    "score": result.score,
                    "source": result.payload.get("source", "unknown"),
                    "chapter": result.payload.get("chapter", "unknown"),
                    "section": result.payload.get("section", "unknown"),
                    "doc_id": result.payload.get("doc_id", ""),
                })

            logger.info(f"✓ Found {len(formatted_results)} results for query")
            return formatted_results
        except Exception as e:
            logger.error(f"✗ Search failed: {e}")
            raise

    def delete_document(self, doc_id: str) -> None:
        """
        Delete a document from the vector store.

        Args:
            doc_id: Document identifier to delete
        """
        try:
            point_id = hash(doc_id) % (10**8)
            self.qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=[point_id],
            )
            logger.info(f"✓ Deleted document: {doc_id}")
        except Exception as e:
            logger.error(f"✗ Failed to delete document {doc_id}: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection stats
        """
        try:
            info = self.qdrant_client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": info.points_count,
                "vectors_count": info.vectors_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"✗ Failed to get collection info: {e}")
            raise


# Create singleton instance
vector_store = VectorStoreService()


def get_vector_store() -> VectorStoreService:
    """
    Get the vector store service instance.

    Returns:
        VectorStoreService: Singleton instance
    """
    return vector_store
