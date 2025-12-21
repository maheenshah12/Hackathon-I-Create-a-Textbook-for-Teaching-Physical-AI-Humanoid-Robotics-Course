import os
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    # API Settings
    openrouter_api_key: str = os.getenv("OPENROUTER_API_KEY", "")
    openrouter_base_url: str = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
    qwen_api_key: str = os.getenv("QWEN_API_KEY", "")  # May be the same as OpenRouter key

    # Database Settings
    neon_database_url: str = os.getenv("NEON_DATABASE_URL", "")
    neon_postgres_url: str = os.getenv("NEON_POSTGRES_URL", "")  # Alternative name

    # Vector Database Settings
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection: str = os.getenv("QDRANT_COLLECTION", "project_docs")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", "6333"))

    # Application Settings
    app_name: str = "RAG Chatbot Backend"
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"
    environment: str = os.getenv("ENVIRONMENT", "development")

    # Model Settings
    embedding_model: str = os.getenv("EMBEDDING_MODEL", "text-embedding-ada-002")
    chat_model: str = os.getenv("CHAT_MODEL", "openchat/openchat-7b:free")

    # Vector Settings
    vector_size: int = 1536  # Default size for text-embedding-ada-002 embeddings

    class Config:
        env_file = ".env"


settings = Settings()