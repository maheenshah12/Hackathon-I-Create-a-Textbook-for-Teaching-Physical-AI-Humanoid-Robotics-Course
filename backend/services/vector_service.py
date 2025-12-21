from typing import List, Dict, Any
from vector_store.qdrant_client import get_qdrant_client


class VectorService:
    def __init__(self):
        self.qdrant_client = get_qdrant_client()

    def search(self, query_vector: List[float], top_k: int = 5, filters: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors to the query vector.

        Args:
            query_vector: The embedding vector to search for
            top_k: Number of top results to return
            filters: Optional filters for the search

        Returns:
            List of matching documents with payload and score
        """
        return self.qdrant_client.search(query_vector, top_k, filters)

    def add_document(self, content: str, source_path: str, chunk_index: int = 0, doc_id: str = None) -> str:
        """
        Add a document to the vector store.

        Args:
            content: Content of the document chunk
            source_path: Path of the source document
            chunk_index: Index of this chunk in the original document
            doc_id: Optional document ID (will be generated if not provided)

        Returns:
            ID of the added document
        """
        import uuid
        from services.embedding_service import embedding_service

        if not doc_id:
            doc_id = str(uuid.uuid4())

        # Create embedding for the content
        embedding = embedding_service.create_embedding(content)

        # Prepare payload
        payload = {
            "content": content,
            "source_path": source_path,
            "chunk_index": chunk_index,
            "content_hash": hash(content)  # Simple hash for change detection
        }

        # Add to vector store
        self.qdrant_client.add_vectors(
            vectors=[embedding],
            payloads=[payload],
            ids=[doc_id]
        )

        return doc_id

    def delete_by_source_path(self, source_path: str):
        """
        Delete all documents associated with a specific source path.

        Args:
            source_path: Path of the document to delete
        """
        self.qdrant_client.delete_by_source_path(source_path)

    def get_all_source_paths(self) -> List[str]:
        """
        Get all unique source paths in the vector store.

        Returns:
            List of unique source paths
        """
        return self.qdrant_client.get_all_source_paths()


# Global instance with lazy initialization
vector_service = None


def get_vector_service():
    global vector_service
    if vector_service is None:
        vector_service = VectorService()
    return vector_service