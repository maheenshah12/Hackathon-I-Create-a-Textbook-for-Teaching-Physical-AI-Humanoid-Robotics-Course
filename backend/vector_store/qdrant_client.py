from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from config.settings import settings


class QdrantStore:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Using HTTP for compatibility
        )
        self.collection_name = "book_content"
        self._initialize_collection()

    def _initialize_collection(self):
        """
        Initialize the Qdrant collection if it doesn't exist.
        """
        try:
            # Check if collection exists
            collection_info = self.client.get_collection(self.collection_name)

            # Check if the vector size matches our expected size
            expected_size = settings.vector_size
            actual_size = collection_info.config.params.vectors.get("size") if hasattr(collection_info.config.params.vectors, 'get') else collection_info.config.params.vectors.size

            if actual_size != expected_size:
                print(f"Vector dimension mismatch: expected {expected_size}, got {actual_size}. Recreating collection...")
                # Delete the existing collection and create a new one with correct dimensions
                self.client.delete_collection(self.collection_name)
                self._create_collection_with_correct_dimensions()
            else:
                print(f"Collection exists with correct dimensions: {actual_size}")

        except:
            # Create collection if it doesn't exist
            self._create_collection_with_correct_dimensions()

    def _create_collection_with_correct_dimensions(self):
        """Create a new collection with the correct vector dimensions."""
        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=settings.vector_size,  # Based on embedding model
                distance=models.Distance.COSINE
            )
        )

        # Create payload index for faster filtering
        self.client.create_payload_index(
            collection_name=self.collection_name,
            field_name="source_path",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

    def add_vectors(self, vectors: List[List[float]], payloads: List[Dict[str, Any]], ids: List[str]):
        """
        Add vectors with their payloads to the collection.

        Args:
            vectors: List of embedding vectors
            payloads: List of metadata payloads
            ids: List of unique IDs for the vectors
        """
        points = [
            models.PointStruct(
                id=idx,
                vector=vector,
                payload=payload
            )
            for idx, vector, payload in zip(ids, vectors, payloads)
        ]

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def search(self, query_vector: List[float], top_k: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar vectors to the query vector.

        Args:
            query_vector: The embedding vector to search for
            top_k: Number of top results to return
            filters: Optional filters for the search

        Returns:
            List of matching points with payload and score
        """
        search_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                filter_conditions.append(
                    models.FieldCondition(
                        key=key,
                        match=models.MatchValue(value=value)
                    )
                )
            if filter_conditions:
                search_filter = models.Filter(must=filter_conditions)

        results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=top_k,
            query_filter=search_filter
        )

        # The query_points method returns a QueryResponse object with a 'points' attribute
        search_results = results.points if hasattr(results, 'points') else results

        return [
            {
                "id": result.id,
                "score": result.score,
                "payload": result.payload,
                "content": result.payload.get("content", ""),
                "source_path": result.payload.get("source_path", ""),
                "chunk_index": result.payload.get("chunk_index", 0)
            }
            for result in search_results
        ]

    def delete_by_source_path(self, source_path: str):
        """
        Delete all vectors associated with a specific source path.

        Args:
            source_path: Path of the document to delete
        """
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_path",
                            match=models.MatchValue(value=source_path)
                        )
                    ]
                )
            )
        )

    def get_all_source_paths(self) -> List[str]:
        """
        Get all unique source paths in the collection.

        Returns:
            List of unique source paths
        """
        try:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="source_path",
                            match=models.MatchAny(any=[])
                        )
                    ]
                ),
                limit=10000,  # Adjust as needed
                with_payload=True
            )

            source_paths = set()
            # Handle the results based on the Qdrant client version
            if isinstance(results, tuple) and len(results) == 2:
                # Newer version returns (records, next_page_offset)
                records, _ = results
                for record in records:
                    if hasattr(record, 'payload') and "source_path" in record.payload:
                        source_paths.add(record.payload["source_path"])
            else:
                # Older version or different format
                for item in results:
                    if hasattr(item, 'payload') and "source_path" in item.payload:
                        source_paths.add(item.payload["source_path"])
                    elif isinstance(item, tuple) and len(item) == 2:
                        record, _ = item
                        if hasattr(record, 'payload') and "source_path" in record.payload:
                            source_paths.add(record.payload["source_path"])

            return list(source_paths)
        except Exception as e:
            # If there's an error, return an empty list instead of crashing
            print(f"Error getting source paths: {str(e)}")
            return []


# Global instance with lazy initialization
qdrant_client = None


def get_qdrant_client():
    global qdrant_client
    if qdrant_client is None:
        qdrant_client = QdrantStore()
    return qdrant_client