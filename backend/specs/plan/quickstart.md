# Quickstart Guide: RAG Chatbot for Docusaurus Book

## Prerequisites
- Python 3.9+
- Node.js 16+
- Docusaurus project
- OpenAI API key
- Neon Postgres account
- Qdrant Cloud account

## Setup Steps

### 1. Clone and Initialize Backend
```bash
mkdir rag-chatbot-backend
cd rag-chatbot-backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install fastapi uvicorn python-dotenv openai qdrant-client psycopg2-binary sqlalchemy python-multipart
```

### 2. Environment Variables
Create `.env` file:
```env
OPENAI_API_KEY=your_openai_api_key
NEON_DATABASE_URL=postgresql://user:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require
QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key
SECRET_KEY=your_secret_key
```

### 3. Initialize Database
```bash
# Run database migrations
python -c "
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
import os

load_dotenv()

DATABASE_URL = os.getenv('NEON_DATABASE_URL')
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create tables based on data model
from sqlalchemy import text

with engine.connect() as conn:
    conn.execute(text('''
    CREATE TABLE IF NOT EXISTS chat_sessions (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        user_id UUID,
        session_token VARCHAR(255),
        created_at TIMESTAMP DEFAULT NOW(),
        updated_at TIMESTAMP DEFAULT NOW(),
        metadata JSONB,
        is_active BOOLEAN DEFAULT TRUE
    );
    '''))

    conn.execute(text('''
    CREATE TABLE IF NOT EXISTS messages (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        session_id UUID REFERENCES chat_sessions(id),
        role VARCHAR(20) NOT NULL,
        content TEXT NOT NULL,
        timestamp TIMESTAMP DEFAULT NOW(),
        metadata JSONB,
        parent_message_id UUID REFERENCES messages(id)
    );
    '''))

    conn.execute(text('''
    CREATE TABLE IF NOT EXISTS book_content (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        source_path VARCHAR(500) NOT NULL,
        content_hash VARCHAR(64) NOT NULL,
        chunk_index INTEGER NOT NULL,
        content TEXT NOT NULL,
        metadata JSONB,
        created_at TIMESTAMP DEFAULT NOW(),
        updated_at TIMESTAMP DEFAULT NOW()
    );
    '''))

    conn.commit()
"
```

### 4. Start Backend Server
```bash
uvicorn main:app --reload --port 8000
```

### 5. Integrate with Docusaurus
Add the chat widget to your Docusaurus site by creating a React component in `src/components/ChatWidget`.

### 6. Test the Integration
- Visit your Docusaurus site
- Open the chat widget
- Ask questions about your book content