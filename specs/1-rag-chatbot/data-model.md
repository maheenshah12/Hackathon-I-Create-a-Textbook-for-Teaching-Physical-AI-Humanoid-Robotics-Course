# Data Model: RAG Chatbot for Docusaurus Book

## Entity: Conversation
**Description**: Represents a single chat session between user and bot

**Fields**:
- conversation_id (string, primary key): Unique identifier for the conversation
- created_at (timestamp): When the conversation was started
- updated_at (timestamp): When the conversation was last updated
- metadata (JSON): Additional conversation metadata (user preferences, etc.)

**Validation Rules**:
- conversation_id must be unique
- created_at must be in the past
- updated_at must be >= created_at

## Entity: Message
**Description**: Represents a single message in a conversation

**Fields**:
- message_id (string, primary key): Unique identifier for the message
- conversation_id (string, foreign key): Links to parent conversation
- role (string): Either "user" or "assistant"
- content (text): The message content
- timestamp (timestamp): When the message was created
- sources (JSON): Document references for the response (for bot messages)
- selected_text (text, optional): Text selected by user when message was sent

**Validation Rules**:
- role must be one of ["user", "assistant"]
- content must not be empty
- conversation_id must reference an existing conversation
- timestamp must be in the past

## Entity: BookContent
**Description**: Represents processed book content for RAG retrieval

**Fields**:
- content_id (string, primary key): Unique identifier for the content chunk
- source_path (string): Path to the original document
- chunk_index (integer): Position of this chunk in the original document
- content_text (text): The actual content of this chunk
- embedding_vector (JSON): Vector representation of the content
- metadata (JSON): Additional information (title, section, etc.)
- hash (string): Hash of the content for change detection

**Validation Rules**:
- content_id must be unique
- source_path must be valid
- chunk_index must be non-negative
- content_text must not be empty
- embedding_vector must be a valid vector representation

## Entity: UserSession
**Description**: Tracks user session information (if needed for advanced features)

**Fields**:
- session_id (string, primary key): Unique identifier for the session
- user_id (string, optional): Identifier for registered users
- created_at (timestamp): When the session was created
- expires_at (timestamp): When the session expires
- active_conversation_id (string, optional): Current active conversation

**Validation Rules**:
- session_id must be unique
- expires_at must be after created_at
- active_conversation_id must reference an existing conversation if present

## Relationships

1. **Conversation** 1 → * **Message**: One conversation has many messages
2. **Message** * → 1 **Conversation**: Many messages belong to one conversation
3. **Message** 0..1 → * **BookContent**: Messages may reference multiple book content chunks (for bot responses)

## State Transitions

### Conversation States
- **Active**: New conversation created, ready for messages
- **Inactive**: Conversation has no activity for a specified period
- **Archived**: Conversation is preserved but not actively used

### Message States
- **Pending**: Message sent, waiting for response (user messages only)
- **Processing**: System is generating response (bot messages only)
- **Complete**: Message fully processed and ready for display
- **Error**: Message processing failed

## Indexing Strategy

1. **Conversations**: Index on (updated_at) for retrieving recent conversations
2. **Messages**: Index on (conversation_id, timestamp) for chronological message retrieval
3. **BookContent**: Index on (source_path, chunk_index) for document organization
4. **BookContent**: Vector index on (embedding_vector) for similarity search