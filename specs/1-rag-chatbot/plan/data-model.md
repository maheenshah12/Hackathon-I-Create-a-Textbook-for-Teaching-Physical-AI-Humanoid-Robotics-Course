# Data Model: RAG Chatbot for Docusaurus Book

## Entities

### ChatSession
- **id**: UUID (Primary Key)
- **user_id**: UUID (Optional, for logged-in users)
- **session_token**: String (For anonymous users)
- **created_at**: DateTime
- **updated_at**: DateTime
- **metadata**: JSON (Additional session data)
- **is_active**: Boolean

### Message
- **id**: UUID (Primary Key)
- **session_id**: UUID (Foreign Key to ChatSession)
- **role**: String (user|assistant|system)
- **content**: Text (Message content)
- **timestamp**: DateTime
- **metadata**: JSON (Additional message data)
- **parent_message_id**: UUID (For message threading)

### BookContent
- **id**: UUID (Primary Key)
- **source_path**: String (Path to markdown file)
- **content_hash**: String (Hash of content for change detection)
- **chunk_index**: Integer (Position in document)
- **content**: Text (Chunked content)
- **embedding**: Vector (Embedding vector for similarity search)
- **metadata**: JSON (Source, title, section info)
- **created_at**: DateTime
- **updated_at**: DateTime

### EmbeddingVector
- **id**: UUID (Primary Key)
- **content_id**: UUID (Foreign Key to BookContent)
- **vector**: Vector (Embedding values)
- **vector_index**: Integer (Position in document)
- **created_at**: DateTime

### UserSelection
- **id**: UUID (Primary Key)
- **session_id**: UUID (Foreign Key to ChatSession)
- **selected_text**: Text (User-selected content)
- **context_info**: JSON (Page, section, etc.)
- **created_at**: DateTime
- **expires_at**: DateTime (TTL for temporary selections)