# Research: RAG Chatbot for Docusaurus Book

## Decision: OpenRouter API with Qwen Models
**Rationale**: OpenRouter provides access to various LLMs including Qwen models, which meet the specific requirements for both chat completion and text embeddings. This approach offers flexibility and cost-effectiveness compared to other LLM providers.

**Alternatives considered**:
- OpenAI API: More expensive, limited model choices
- Anthropic Claude: Good quality but limited embedding options
- Self-hosted models: Higher complexity and infrastructure requirements

## Decision: Qwen Model Selection
**Rationale**: Qwen models offer both chat completion and embedding capabilities required for the RAG system. The Qwen-2 series provides good performance for both tasks.

**Alternatives considered**:
- Different Qwen variants (Qwen-1, Qwen-Max, etc.)
- Other open-source models via OpenRouter
- Mixed approach with different models for embeddings vs chat

## Decision: Vector Database Strategy
**Rationale**: Qdrant Cloud free tier provides sufficient capacity for book content indexing with good performance and minimal setup overhead.

**Alternatives considered**:
- Pinecone: More expensive
- Weaviate: Self-hosting required
- Supabase Vector: Additional infrastructure complexity

## Decision: Database for Metadata
**Rationale**: Neon Serverless Postgres offers serverless scalability and compatibility with existing PostgreSQL tools while providing the necessary metadata storage for conversations.

**Alternatives considered**:
- PostgreSQL self-hosted: More operational overhead
- MongoDB: Different skill set required
- SQLite: Insufficient for concurrent access

## Decision: Frontend Integration Approach
**Rationale**: Embedding the chat widget directly in Docusaurus pages provides the best user experience with seamless integration into the reading flow.

**Alternatives considered**:
- Standalone chat interface: Poorer user experience
- Third-party chat solutions: Less customization control
- Browser extension: Additional complexity for users

## Key Findings

1. **Qwen Embedding Models**: Qwen models accessible via OpenRouter can handle both chat and embedding tasks, simplifying the architecture.

2. **Text Chunking Strategy**: Overlapping chunks of 512-1024 tokens with 10% overlap provide optimal context preservation for RAG.

3. **Selected Text Mode**: Implementation requires JavaScript to detect and extract user-selected text before sending to backend.

4. **Performance Considerations**: Caching frequently accessed embeddings and responses can significantly improve response times.

5. **Security**: All API keys should be stored as environment variables and never exposed to the frontend.

6. **Deployment**: Backend can be deployed to various cloud providers (Vercel, Render, Railway) while Docusaurus can remain on GitHub Pages with API calls to the backend.