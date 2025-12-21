from typing import List
from .llm_service import llm_service


class EmbeddingService:
    def __init__(self):
        # Using the same LLM service for embeddings
        pass

    def create_embedding(self, text: str) -> List[float]:
        """
        Create an embedding for the given text using the LLM service.

        Args:
            text: Text to generate embedding for

        Returns:
            List of float values representing the embedding vector
        """
        return llm_service.generate_embedding(text)

    def create_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for a batch of texts.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors
        """
        embeddings = []
        for text in texts:
            embedding = self.create_embedding(text)
            embeddings.append(embedding)
        return embeddings


# Global instance
embedding_service = EmbeddingService()