# Backend for RAG Chatbot

This backend implements the requirements specified in the RAG Chatbot feature specification.

## Architecture

- FastAPI backend for chat functionality
- Integration with OpenRouter API for LLM access
- Qwen models for chat completion and text embeddings
- Qdrant Cloud as vector database
- Neon Serverless Postgres for metadata and conversation state

## Components

- API endpoints for chat interactions
- Services for document ingestion and embedding
- Vector search and retrieval system
- Conversation management
- Secure API key handling