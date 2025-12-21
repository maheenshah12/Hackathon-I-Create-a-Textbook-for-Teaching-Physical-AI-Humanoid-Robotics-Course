from typing import Dict, Any


class DocumentService:
    def process_and_ingest_document(self, content: str, source_path: str) -> Dict[str, Any]:
        # Placeholder: pretend successful ingestion
        return {"status": "ingested", "source_path": source_path}

    def delete_document(self, source_path: str) -> bool:
        return True

    def get_all_ingested_documents(self):
        return []


def get_document_service():
    return DocumentService()
import os
import glob
from typing import List, Dict, Any
from utils.text_processor import text_processor
from services.embedding_service import embedding_service
from vector_store.qdrant_client import get_qdrant_client
from config.settings import settings
import hashlib


class DocumentService:
    def __init__(self):
        self.text_processor = text_processor
        self.embedding_service = embedding_service
        self.qdrant_client = get_qdrant_client()

    def process_and_ingest_document(self, content: str, source_path: str) -> Dict[str, Any]:
        """
        Process and ingest a single document into the vector store.

        Args:
            content: Raw document content
            source_path: Path of the source document

        Returns:
            Dictionary with ingestion results
        """
        try:
            # Process the document (extract text and chunk)
            chunks = self.text_processor.process_document(content, source_path)

            # Generate embeddings for each chunk
            chunk_contents = [chunk['content'] for chunk in chunks]
            embeddings = self.embedding_service.create_embeddings_batch(chunk_contents)

            # Prepare data for Qdrant
            ids = []
            payloads = []
            vectors = []

            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                # Create a unique ID for this chunk - Qdrant requires integer or UUID
                import uuid
                chunk_id = str(uuid.uuid4())

                payload = {
                    'content': chunk['content'],
                    'source_path': source_path,
                    'chunk_index': chunk['chunk_index'],
                    'metadata': chunk.get('metadata', {})
                }

                ids.append(chunk_id)
                payloads.append(payload)
                vectors.append(embedding)

            # Add vectors to Qdrant
            self.qdrant_client.add_vectors(vectors, payloads, ids)

            return {
                'source_path': source_path,
                'chunks_processed': len(chunks),
                'status': 'success'
            }
        except Exception as e:
            # Return error information instead of crashing
            error_msg = str(e)
            if "'data'" in error_msg:
                # This is the specific error we're seeing
                print(f"Processing error for {source_path}: {error_msg}")
                # Try to process with simpler content
                try:
                    # Fallback: try to process the content in smaller chunks
                    simple_chunks = self._create_simple_chunks(content, source_path)
                    chunk_contents = [chunk['content'] for chunk in simple_chunks]
                    embeddings = self.embedding_service.create_embeddings_batch(chunk_contents)

                    ids = []
                    payloads = []
                    vectors = []

                    for i, (chunk, embedding) in enumerate(zip(simple_chunks, embeddings)):
                        import uuid
                        chunk_id = str(uuid.uuid4())

                        payload = {
                            'content': chunk['content'],
                            'source_path': source_path,
                            'chunk_index': chunk['chunk_index'],
                            'metadata': chunk.get('metadata', {})
                        }

                        ids.append(chunk_id)
                        payloads.append(payload)
                        vectors.append(embedding)

                    # Add vectors to Qdrant
                    self.qdrant_client.add_vectors(vectors, payloads, ids)

                    return {
                        'source_path': source_path,
                        'chunks_processed': len(simple_chunks),
                        'status': 'success_with_fallback'
                    }
                except Exception as fallback_error:
                    return {
                        'source_path': source_path,
                        'chunks_processed': 0,
                        'status': 'failed',
                        'error': str(fallback_error)
                    }
            else:
                return {
                    'source_path': source_path,
                    'chunks_processed': 0,
                    'status': 'failed',
                    'error': str(e)
                }

    def _create_simple_chunks(self, content: str, source_path: str) -> List[dict]:
        """
        Create simple chunks as a fallback when normal processing fails.
        """
        # Split content into smaller chunks by paragraphs or fixed size
        import re

        # Split by markdown headers or large paragraphs
        parts = re.split(r'\n#{2,}', content)  # Split at ## and ### headers

        chunks = []
        chunk_size = 1000  # characters per chunk

        for i, part in enumerate(parts):
            if len(part.strip()) == 0:
                continue

            # If part is too large, split further
            if len(part) > chunk_size:
                sub_parts = [part[i:i+chunk_size] for i in range(0, len(part), chunk_size)]
                for j, sub_part in enumerate(sub_parts):
                    if len(sub_part.strip()) > 0:
                        chunks.append({
                            'content': sub_part.strip(),
                            'source_path': source_path,
                            'chunk_index': len(chunks),
                            'metadata': {'type': 'fallback_chunk', 'original_part': i, 'sub_part': j}
                        })
            else:
                chunks.append({
                    'content': part.strip(),
                    'source_path': source_path,
                    'chunk_index': len(chunks),
                    'metadata': {'type': 'fallback_chunk', 'original_part': i}
                })

        return chunks

    def process_and_ingest_directory(self, directory_path: str, file_pattern: str = "**/*.md") -> Dict[str, Any]:
        """
        Process and ingest all documents in a directory.

        Args:
            directory_path: Path to the directory containing documents
            file_pattern: Pattern to match files (default: all markdown files)

        Returns:
            Dictionary with ingestion results
        """
        # Find all matching files
        search_pattern = os.path.join(directory_path, file_pattern)
        files = glob.glob(search_pattern, recursive=True)

        results = {
            'processed_files': [],
            'failed_files': [],
            'total_chunks': 0
        }

        for file_path in files:
            try:
                # Read the file content
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Process and ingest the document
                relative_path = os.path.relpath(file_path, directory_path)
                result = self.process_and_ingest_document(content, relative_path)

                results['processed_files'].append(result)
                results['total_chunks'] += result['chunks_processed']

            except Exception as e:
                results['failed_files'].append({
                    'file_path': file_path,
                    'error': str(e)
                })

        return results

    def delete_document(self, source_path: str) -> bool:
        """
        Delete all chunks associated with a specific document from the vector store.

        Args:
            source_path: Path of the document to delete

        Returns:
            True if deletion was successful, False otherwise
        """
        try:
            self.qdrant_client.delete_by_source_path(source_path)
            return True
        except Exception as e:
            print(f"Error deleting document {source_path}: {str(e)}")
            return False

    def get_all_ingested_documents(self) -> List[str]:
        """
        Get a list of all documents that have been ingested.

        Returns:
            List of source paths for all ingested documents
        """
        return self.qdrant_client.get_all_source_paths()


# Global instance with lazy initialization
document_service = None


def get_document_service():
    global document_service
    if document_service is None:
        document_service = DocumentService()
    return document_service