from typing import List, Dict, Any, Optional
from .llm_service import llm_service
from .embedding_service import embedding_service
from .vector_service import get_vector_service
from config.settings import settings


class RAGService:
    def __init__(self):
        self.llm_service = llm_service
        self.embedding_service = embedding_service
        self.vector_service = get_vector_service()

    def generate_response(self, query: str, conversation_id: str = None, selected_text: str = None) -> Dict[str, Any]:
        """
        Generate a response using RAG methodology.

        Args:
            query: The user's question
            conversation_id: Optional conversation ID for context
            selected_text: Optional selected text for restricted mode

        Returns:
            Dictionary containing response, sources, and conversation ID
        """
        # If selected text is provided, use ONLY that as context (skip vector search)
        if selected_text:
            context = selected_text
            sources = ["selected_text"]
        else:
            # Perform similarity search to retrieve relevant context
            query_embedding = self.embedding_service.create_embedding(query)
            retrieved_docs = self.vector_service.search(query_embedding, top_k=3)

            # Assemble context from retrieved documents
            context_parts = []
            sources = []
            for doc in retrieved_docs:
                context_parts.append(doc['content'])
                if doc['source_path'] not in sources:
                    sources.append(doc['source_path'])

            context = "\n\n".join(context_parts)

        # Generate response using LLM with context
        response = self.llm_service.generate_response(
            prompt=query,
            context=context,
            selected_text=selected_text  # Pass selected text to ensure it's used in LLM service
        )

        return {
            "response": response,
            "sources": sources,
            "conversation_id": conversation_id or self._generate_conversation_id()
        }

    def _generate_conversation_id(self) -> str:
        """
        Generate a unique conversation ID.
        """
        import uuid
        return str(uuid.uuid4())


# Global instance with lazy initialization
rag_service = None


def get_rag_service():
    global rag_service
    if rag_service is None:
        rag_service = RAGService()
    return rag_service