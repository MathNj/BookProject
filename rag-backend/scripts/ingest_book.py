#!/usr/bin/env python3
"""
Book Ingestion Script for Qdrant Vector Store.

Reads Markdown files from the Docusaurus textbook, chunks them,
generates embeddings, and uploads to Qdrant for RAG retrieval.

Usage:
    poetry run python scripts/ingest_book.py
"""

import os
import sys
import logging
from pathlib import Path
from typing import List, Dict, Any
import re

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.services.vector_store import get_vector_store
from src.config import settings

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


class BookIngester:
    """Ingests Markdown chapters into vector store."""

    def __init__(self):
        """Initialize the ingester."""
        self.vector_store = get_vector_store()
        self.docs_path = Path(__file__).parent.parent.parent / "docs-website" / "docs"
        self.chunk_size = 500  # Characters per chunk
        self.chunk_overlap = 50  # Overlap between chunks

    def find_markdown_files(self) -> List[Path]:
        """
        Find all Markdown files in the docs directory.

        Returns:
            List of .md file paths
        """
        if not self.docs_path.exists():
            logger.error(f"Docs path does not exist: {self.docs_path}")
            return []

        md_files = list(self.docs_path.rglob("*.md"))
        logger.info(f"✓ Found {len(md_files)} Markdown files")
        return sorted(md_files)

    def extract_frontmatter(self, content: str) -> Dict[str, Any]:
        """
        Extract YAML frontmatter from Markdown.

        Args:
            content: File content

        Returns:
            Dictionary with frontmatter fields
        """
        frontmatter = {}
        if content.startswith("---"):
            try:
                # Split by first occurrence of '---'
                parts = content.split("---", 2)
                if len(parts) >= 2:
                    # Parse simple YAML (basic key: value parsing)
                    fm_content = parts[1].strip()
                    for line in fm_content.split("\n"):
                        if ":" in line:
                            key, value = line.split(":", 1)
                            frontmatter[key.strip()] = value.strip().strip('"\'')
            except Exception as e:
                logger.debug(f"Failed to parse frontmatter: {e}")
        return frontmatter

    def remove_code_blocks(self, text: str) -> str:
        """
        Remove code blocks from text (keep them separate for RAG).

        Args:
            text: Text content

        Returns:
            Text without code blocks
        """
        # Remove code blocks (```...```)
        text = re.sub(r"```[\s\S]*?```", "[CODE_BLOCK]", text)
        # Remove inline code references
        text = re.sub(r"`[^`]+`", "[CODE]", text)
        return text

    def chunk_text(self, text: str) -> List[str]:
        """
        Split text into overlapping chunks for embedding.

        Args:
            text: Text to chunk

        Returns:
            List of text chunks
        """
        chunks = []
        text = text.strip()

        if len(text) <= self.chunk_size:
            return [text]

        # Split by paragraphs first
        paragraphs = re.split(r"\n\n+", text)

        current_chunk = ""
        for paragraph in paragraphs:
            if len(current_chunk) + len(paragraph) <= self.chunk_size:
                current_chunk += paragraph + "\n\n"
            else:
                if current_chunk.strip():
                    chunks.append(current_chunk.strip())
                current_chunk = paragraph + "\n\n"

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        logger.debug(f"Created {len(chunks)} chunks")
        return chunks

    def process_file(self, file_path: Path) -> int:
        """
        Process a single Markdown file and ingest chunks.

        Args:
            file_path: Path to .md file

        Returns:
            Number of chunks ingested
        """
        try:
            content = file_path.read_text(encoding="utf-8")
        except Exception as e:
            logger.error(f"✗ Failed to read {file_path}: {e}")
            return 0

        # Extract metadata
        frontmatter = self.extract_frontmatter(content)
        title = frontmatter.get("title", file_path.stem)
        module = self._get_module_from_path(file_path)

        # Remove frontmatter from content
        if content.startswith("---"):
            content = content.split("---", 2)[2].strip()

        # Remove code blocks for cleaner chunks
        text_content = self.remove_code_blocks(content)

        # Chunk the text
        chunks = self.chunk_text(text_content)

        if not chunks:
            logger.warning(f"⚠ No content to ingest from {file_path}")
            return 0

        # Ingest each chunk
        ingested_count = 0
        for i, chunk in enumerate(chunks):
            if not chunk.strip():
                continue

            try:
                doc_id = f"{file_path.stem}_chunk_{i}"
                metadata = {
                    "source": file_path.name,
                    "chapter": title,
                    "section": f"Section {i+1}",
                    "module": module,
                }

                self.vector_store.upsert_document(doc_id, chunk, metadata)
                ingested_count += 1
            except Exception as e:
                logger.error(f"✗ Failed to ingest chunk {i} from {file_path}: {e}")
                continue

        logger.info(f"✓ Ingested {ingested_count} chunks from {file_path.name}")
        return ingested_count

    def _get_module_from_path(self, file_path: Path) -> str:
        """
        Extract module name from file path.

        Args:
            file_path: Path to file

        Returns:
            Module name (e.g., "01-nervous-system")
        """
        relative_path = file_path.relative_to(self.docs_path)
        parts = relative_path.parts
        if parts:
            return parts[0]
        return "unknown"

    def ingest_all(self) -> int:
        """
        Ingest all Markdown files from the textbook.

        Returns:
            Total number of chunks ingested
        """
        md_files = self.find_markdown_files()

        if not md_files:
            logger.warning("⚠ No Markdown files found to ingest")
            return 0

        total_ingested = 0
        failed_files = []

        for file_path in md_files:
            try:
                ingested = self.process_file(file_path)
                total_ingested += ingested
            except Exception as e:
                logger.error(f"✗ Error processing {file_path}: {e}")
                failed_files.append(str(file_path))

        # Print summary
        logger.info("\n" + "="*60)
        logger.info("📊 INGESTION SUMMARY")
        logger.info("="*60)
        logger.info(f"Total files processed: {len(md_files)}")
        logger.info(f"Total chunks ingested: {total_ingested}")
        logger.info(f"Failed files: {len(failed_files)}")

        if failed_files:
            logger.warning("Failed to process:")
            for f in failed_files:
                logger.warning(f"  - {f}")

        # Get collection info
        try:
            collection_info = self.vector_store.get_collection_info()
            logger.info(f"\n✓ Collection: {collection_info['name']}")
            logger.info(f"✓ Total points: {collection_info['points_count']}")
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")

        logger.info("="*60)
        return total_ingested


def main():
    """Main entry point."""
    logger.info("🚀 Starting book ingestion...")

    if not settings.openai_api_key:
        logger.error("✗ OPENAI_API_KEY environment variable is not set")
        sys.exit(1)

    if not settings.qdrant_url:
        logger.error("✗ QDRANT_URL environment variable is not set")
        sys.exit(1)

    try:
        ingester = BookIngester()
        total = ingester.ingest_all()

        if total > 0:
            logger.info(f"\n✅ Successfully ingested {total} chunks")
            sys.exit(0)
        else:
            logger.warning("\n⚠ No chunks were ingested")
            sys.exit(1)

    except Exception as e:
        logger.error(f"\n✗ Ingestion failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
