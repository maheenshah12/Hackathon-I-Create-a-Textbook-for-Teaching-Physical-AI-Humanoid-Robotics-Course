# Quickstart: RAG Chatbot for Docusaurus Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Access to OpenRouter API
- Qdrant Cloud account (free tier)
- Neon Serverless Postgres account

## Environment Setup

### Backend Environment Variables
Create a `.env` file in the backend directory:

```bash
# OpenRouter API
OPENROUTER_API_KEY="your-openrouter-api-key"

# Qdrant Configuration
QDRANT_URL="your-qdrant-cluster-url"
QDRANT_API_KEY="your-qdrant-api-key"

# Neon Postgres
NEON_DATABASE_URL="your-neon-database-url"

# Application Settings
ENVIRONMENT="development"
DEBUG="true"
EMBEDDING_MODEL="qwen/qwen-2-7b-instruct:free"
CHAT_MODEL="qwen/qwen-2-72b-instruct"
```

### Backend Installation

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run the application
uvicorn main:app --reload --port 8000
```

## Database Setup

### Neon Postgres Schema

```sql
-- Create conversations table
CREATE TABLE IF NOT EXISTS conversations (
    conversation_id VARCHAR(255) PRIMARY KEY,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    metadata JSONB
);

-- Create messages table
CREATE TABLE IF NOT EXISTS messages (
    message_id VARCHAR(255) PRIMARY KEY,
    conversation_id VARCHAR(255) REFERENCES conversations(conversation_id),
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    sources JSONB,
    selected_text TEXT
);

-- Create book_content table
CREATE TABLE IF NOT EXISTS book_content (
    content_id VARCHAR(255) PRIMARY KEY,
    source_path VARCHAR(500) NOT NULL,
    chunk_index INTEGER NOT NULL,
    content_text TEXT NOT NULL,
    embedding_vector JSONB,
    metadata JSONB,
    hash VARCHAR(255) NOT NULL
);

-- Create indexes
CREATE INDEX IF NOT EXISTS idx_conversations_updated_at ON conversations(updated_at);
CREATE INDEX IF NOT EXISTS idx_messages_conversation_id ON messages(conversation_id, timestamp);
CREATE INDEX IF NOT EXISTS idx_book_content_source ON book_content(source_path, chunk_index);
```

## Qdrant Setup

### Collection Schema

```javascript
// Create Qdrant collection for book content
{
  "collection_name": "book_content",
  "vectors_config": {
    "size": 384,  // Adjust based on Qwen embedding model
    "distance": "Cosine"
  },
  "payload_schema": {
    "source_path": {
      "type": "keyword"
    },
    "chunk_index": {
      "type": "integer"
    },
    "metadata": {
      "type": "keyword"
    }
  }
}
```

## API Endpoints

### Chat Endpoints

```
POST /chat
{
  "message": "Your question here",
  "conversation_id": "optional existing conversation ID",
  "selected_text": "optional selected text for restricted mode"
}

Response:
{
  "response": "Generated response",
  "conversation_id": "conversation ID",
  "sources": ["source references"]
}
```

```
GET /conversations/{conversation_id}
Response: {conversation data}
```

```
DELETE /conversations/{conversation_id}
Response: {status}
```

### Document Ingestion Endpoints

```
POST /documents/ingest
{
  "source_path": "path/to/document",
  "content": "document content",
  "metadata": {}
}
```

### Health Check

```
GET /health
Response: {"status": "healthy"}
```

## Frontend Integration

### Docusaurus Configuration

Add to your `docusaurus.config.js`:

```javascript
// In the plugins array
plugins: [
  // ... your existing plugins
  [
    '@docusaurus/plugin-content-docs',
    {
      id: 'docs',
      path: 'docs',
      routeBasePath: '/',
      // ... other config
    },
  ],
],

// Add the chatbot component to your layout
themeConfig: {
  // ... existing theme config
  chatbot: {
    enabled: true,
    backendUrl: process.env.BACKEND_URL || 'http://localhost:8000'
  }
}
```

### Chat Widget Component

The chat widget will be available as a React component that can be embedded in your Docusaurus pages.

## Running the System

### Development Mode

1. Start the backend:
```bash
cd backend
source venv/bin/activate
uvicorn main:app --reload --port 8000
```

2. Start Docusaurus:
```bash
cd your-docusaurus-project
npm start
```

### Production Deployment

1. Build the backend (if using Docker):
```bash
docker build -t rag-chatbot-backend .
docker run -p 8000:8000 rag-chatbot-backend
```

2. Build and deploy Docusaurus:
```bash
npm run build
# Deploy the build folder to your hosting provider
```

## Testing

### Backend Tests

```bash
cd backend
python -m pytest tests/
```

### API Testing

Use the following curl commands to test the API:

```bash
# Test chat endpoint
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is this book about?",
    "selected_text": "Optional selected text for restricted mode"
  }'

# Test health check
curl http://localhost:8000/health
```

## Troubleshooting

### Common Issues

1. **API Key Issues**: Verify all API keys are correctly set in environment variables
2. **Database Connection**: Check that Neon Postgres connection string is correct
3. **Vector Database**: Ensure Qdrant cluster is accessible and collection exists
4. **CORS Issues**: Verify that the backend allows requests from your frontend domain

### Logging

The application logs to standard output. For more detailed logging, set `DEBUG=true` in environment variables.