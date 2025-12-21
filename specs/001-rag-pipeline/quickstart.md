# Quickstart: RAG Pipeline Setup

## Prerequisites
- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant API key and URL

## Setup

1. **Create the backend directory:**
```bash
mkdir backend && cd backend
```

2. **Initialize the project with uv:**
```bash
uv init
```

3. **Install dependencies:**
```bash
uv add requests beautifulsoup4 cohere qdrant-client python-dotenv
```

4. **Set environment variables:**
Create a `.env` file with:
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
```

## Usage

1. **Run the main pipeline:**
```bash
python main.py
```

2. **The pipeline will:**
   - Fetch all URLs from the deployed site
   - Extract text content from each URL
   - Chunk the text appropriately
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant with metadata

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant access
- `SOURCE_URL`: Base URL of the deployed documentation site (default: https://hackathon-create-a-textbook-for-tea.vercel.app/)