#!/usr/bin/env python
"""
Test script to check Qdrant client initialization.
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.settings import settings
from qdrant_client import QdrantClient

def test_qdrant_client():
    print(f"QDRANT_URL: {settings.qdrant_url}")
    print(f"QDRANT_API_KEY: {'***' if settings.qdrant_api_key else 'None'}")
    print(f"Collection name: book_content")

    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False
        )
        print(f"Client type: {type(client)}")
        print(f"Client methods: {[method for method in dir(client) if not method.startswith('_')]}")

        # Check if search method exists
        if hasattr(client, 'search') or hasattr(client, 'query_points'):
            print("OK: search method exists")
        else:
            print("ERROR: search method does NOT exist")

        # Try to get collection info to test connection
        try:
            # This will test if we can actually connect
            collections = client.get_collections()
            print(f"OK: Connected successfully. Available collections: {collections}")
        except Exception as e:
            print(f"ERROR: Connection failed: {str(e)}")

    except Exception as e:
        print(f"ERROR: Client initialization failed: {str(e)}")

if __name__ == "__main__":
    test_qdrant_client()