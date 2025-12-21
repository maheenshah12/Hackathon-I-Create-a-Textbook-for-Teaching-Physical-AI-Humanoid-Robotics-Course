# Data Model: RAG Pipeline

## Entities

### DocumentChunk
- **id**: Unique identifier for the chunk (UUID)
- **content**: Text content of the chunk (string)
- **embedding**: Vector representation of the content (array of floats)
- **source_url**: URL where the content was extracted from (string)
- **metadata**: Additional information about the chunk (dictionary)
  - title: Page/chapter title
  - section: Section within the document
  - position: Position of the chunk in the original document
- **created_at**: Timestamp when the chunk was created (datetime)

### QdrantPayload
- **content**: The text content of the chunk
- **source_url**: URL of the original page
- **title**: Title of the page/chapter
- **section**: Section name (if applicable)
- **chunk_index**: Position of this chunk in the document sequence

### EmbeddingRequest
- **texts**: Array of text strings to embed (array of strings)
- **model**: Embedding model to use (string)

### EmbeddingResponse
- **embeddings**: Array of embedding vectors (array of arrays)
- **model**: Model used for embedding (string)
- **usage**: Token usage information (dictionary)

## Relationships
- One source URL can generate multiple DocumentChunk entities
- Each DocumentChunk has one embedding vector
- All chunks are stored in the rag_embedding collection in Qdrant