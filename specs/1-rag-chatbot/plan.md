# Implementation Plan: RAG Chatbot for Docusaurus Book

**Feature Branch**: `1-rag-chatbot`
**Created**: 2025-12-16
**Status**: Draft
**Input**: User description: "I want to create a full-stack RAG chatbot system for a Docusaurus-based book. This is Part 2 of my project."

## Technical Context

### Architecture Overview
The RAG chatbot system will be built as a full-stack application with:
- **Frontend**: React-based chat widget embedded in Docusaurus
- **Backend**: FastAPI server handling chat logic and RAG operations
- **Vector Store**: Qdrant for document embeddings and similarity search
- **Database**: Neon Postgres for session management and metadata
- **AI Services**: OpenRouter API with Qwen models for response generation and embedding creation

### Key Technologies
- **Backend Framework**: FastAPI
- **Database**: Neon Serverless Postgres
- **Vector Database**: Qdrant Cloud
- **AI Integration**: OpenRouter API with Qwen models
- **Frontend**: React component for Docusaurus integration
- **Document Processing**: Markdown parsing and text chunking

### System Components
1. **Document Ingestion Pipeline**: Processes Docusaurus markdown files
2. **Embedding Generator**: Creates vector embeddings using Qwen models
3. **RAG Pipeline**: Retrieves and generates responses using book content
4. **Chat Session Manager**: Handles conversation state and history
5. **Frontend Widget**: Embedded chat interface in Docusaurus pages

## Constitution Check

### Compliance with Project Principles
- **Technical Accuracy**: All implementations will follow official documentation for FastAPI, OpenAI, Qdrant, and Docusaurus
- **Clarity for Learning**: Implementation will include clear documentation and examples
- **Practical and Hands-On**: All components will be tested in real environments
- **Consistency**: Code style and documentation will follow project standards

### Success Criteria Alignment
- Backend will be deployable and scalable
- Frontend will integrate seamlessly with Docusaurus
- System will handle book content efficiently
- Performance targets will be met (response times, accuracy)

## Phase 0: Research & Analysis

### Completed Research
- [X] Technology stack evaluation and selection
- [X] Architecture pattern analysis
- [X] API design principles
- [X] Document processing approaches
- [X] Vector database comparison

## Phase 1: System Design

### Backend Architecture
The backend will be built using FastAPI with the following structure:

```
backend/
├── main.py                 # FastAPI application entry point
├── models/                 # Data models and schemas
│   ├── database.py         # Database models
│   ├── chat.py             # Chat-related models
│   └── documents.py        # Document models
├── api/                    # API routes
│   ├── chat.py             # Chat endpoints
│   ├── documents.py        # Document ingestion endpoints
│   └── health.py           # Health check endpoints
├── services/               # Business logic
│   ├── chat_service.py     # Chat session management
│   ├── rag_service.py      # RAG pipeline
│   ├── document_service.py # Document processing
│   └── embedding_service.py # Embedding generation
├── database/               # Database configuration
│   ├── session.py          # Database session
│   └── init.py             # Schema initialization
├── vector_store/           # Vector store operations
│   └── qdrant_client.py    # Qdrant integration
├── config/                 # Configuration
│   └── settings.py         # Settings and environment variables
└── utils/                  # Utility functions
    ├── markdown_parser.py  # Markdown processing
    └── validators.py       # Input validation
```

### Database Schema (Neon Postgres)
Based on the data model defined in `data-model.md`:
- `chat_sessions` table for session management
- `messages` table for conversation history
- `book_content` table for processed documents
- `embedding_vectors` table for vector storage (if needed separately)

### Vector Store Schema (Qdrant)
- Collection: `book_content`
- Vector size: Based on Qwen embedding model (likely 384-1536 dimensions depending on model)
- Payload: source_path, content_hash, chunk_index, metadata

## Phase 2: Implementation Steps

### Step 1: Backend Setup
**Objective**: Set up FastAPI application with database and vector store connections

**Commands**:
```bash
# Create project directory
mkdir rag-chatbot-backend
cd rag-chatbot-backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-dotenv openrouter qdrant-client psycopg2-binary sqlalchemy python-multipart python-jose[cryptography] passlib[bcrypt] python-uuid requests

# Create directory structure
mkdir -p models api services database vector_store config utils
```

**Tasks**:
1. Create `config/settings.py` with environment variable loading
2. Create database models in `models/database.py`
3. Set up database session in `database/session.py`
4. Create initial FastAPI app in `main.py`

### Step 2: Document Ingestion Pipeline
**Objective**: Create system to process Docusaurus markdown files and store them in vector database

**Commands**:
```bash
# Install additional dependencies for document processing
pip install markdown unstructured PyPDF2
```

**Tasks**:
1. Create `utils/markdown_parser.py` for parsing markdown files
2. Create `services/document_service.py` for document processing logic
3. Create `services/embedding_service.py` for embedding generation
4. Create `/api/documents` endpoints for ingestion
5. Implement text chunking with overlap for context preservation

### Step 3: RAG Pipeline Implementation
**Objective**: Build retrieval and generation system using vector search

**Tasks**:
1. Create `vector_store/qdrant_client.py` for vector database operations
2. Implement similarity search in `services/rag_service.py`
3. Add re-ranking logic for better results
4. Create OpenRouter API integration with Qwen models for response generation
5. Implement context injection for RAG responses

### Step 4: Chat Session Management
**Objective**: Implement conversation state management and history

**Tasks**:
1. Create `models/chat.py` for chat-related data models
2. Implement session creation and management in `services/chat_service.py`
3. Create `/api/chat` endpoints for messaging
4. Add WebSocket support for real-time chat (optional)

### Step 5: Frontend Widget Development
**Objective**: Create React component for Docusaurus integration

**Commands**:
```bash
# Navigate to Docusaurus project
cd /path/to/your/docusaurus/project

# Create components directory if it doesn't exist
mkdir -p src/components
```

**Tasks**:
1. Create `src/components/ChatWidget/ChatWidget.jsx` React component
2. Implement floating chat interface with CSS
3. Add API integration for sending/receiving messages
4. Implement "Ask about selected text" functionality
5. Add session persistence using localStorage

### Step 6: API Integration
**Objective**: Connect frontend to backend services

**Tasks**:
1. Implement API client in frontend component
2. Add error handling and loading states
3. Implement session token management
4. Add typing indicators and message streaming

### Step 7: Testing and Validation
**Objective**: Ensure all components work together properly

**Tasks**:
1. Unit tests for backend services
2. Integration tests for API endpoints
3. Frontend component testing
4. End-to-end testing of complete flow

### Step 8: Deployment Preparation
**Objective**: Prepare system for deployment

**Tasks**:
1. Create Dockerfile for backend
2. Update Docusaurus configuration for production
3. Set up environment-specific configurations
4. Create deployment scripts

## Phase 3: Deployment Steps

### Backend Deployment
**To Neon Postgres**:
1. Create Neon project and database
2. Run database migrations using the schema from quickstart.md
3. Set environment variables for production

**To Qdrant Cloud**:
1. Create Qdrant cluster
2. Configure API key and endpoint
3. Set up collection schema

**To Cloud Provider**:
1. Deploy FastAPI application (using Docker or directly)
2. Set up domain and SSL certificate
3. Configure environment variables

### Frontend Integration
1. Build Docusaurus site with chat widget
2. Deploy to GitHub Pages (or preferred hosting)
3. Configure API endpoint in frontend

## Success Criteria Verification

### Technical Validation
- [ ] Backend API responds to requests within 10 seconds (95% of requests)
- [ ] Document ingestion processes all book content successfully
- [ ] Vector search returns relevant results for queries
- [ ] Chat responses are generated based on book content
- [ ] Session management preserves conversation context

### User Experience Validation
- [ ] Chat widget loads within 3 seconds of page load
- [ ] Both "Ask about book content" and "Ask about selected text" modes work
- [ ] Floating chat remains accessible across pages
- [ ] Error handling provides clear feedback to users

## Risk Mitigation

### Technical Risks
- **API Rate Limits**: Implement caching and request throttling
- **Large Document Processing**: Chunk large documents and process in batches
- **Vector Database Costs**: Optimize embedding requests and implement caching
- **Response Latency**: Use streaming responses and optimize queries

### Implementation Risks
- **Scope Creep**: Focus on MVP features first, add enhancements later
- **Integration Complexity**: Test components independently before integration
- **Performance Issues**: Monitor and optimize critical paths

## Next Steps

1. Complete Phase 1 implementation (Backend setup)
2. Implement document ingestion pipeline
3. Build RAG pipeline
4. Create frontend widget
5. Integrate and test full system
6. Deploy to production environment