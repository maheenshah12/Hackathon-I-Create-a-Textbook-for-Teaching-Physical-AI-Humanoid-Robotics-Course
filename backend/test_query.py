#!/usr/bin/env python
"""
Test script to check Qdrant query_points method.
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.settings import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models

def test_query_points():
    print(f"QDRANT_URL: {settings.qdrant_url}")
    print(f"QDRANT_API_KEY: {'***' if settings.qdrant_api_key else 'None'}")

    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False
        )
        print(f"Client type: {type(client)}")

        # Try a simple query with a dummy vector (just to see the return type)
        # Using a simple 4-dim vector for testing
        dummy_vector = [0.1, 0.2, 0.3, 0.4]

        try:
            # Try the old method first to see what happens
            print("Testing old search method...")
            try:
                old_results = client.search(
                    collection_name="book_content",
                    query_vector=dummy_vector,
                    limit=1
                )
                print(f"Old search method returned: {type(old_results)} with {len(old_results) if hasattr(old_results, '__len__') else 'N/A'} items")
            except Exception as e:
                print(f"Old search method failed: {e}")

            # Try the new query_points method
            print("Testing new query_points method...")
            results = client.query_points(
                collection_name="book_content",
                query=dummy_vector,
                limit=1
            )
            print(f"New query_points method returned: {type(results)}")
            print(f"Type of results object: {type(results)}")

            # Check if it's a named tuple, dataclass, or regular object
            print(f"Available attributes: {[attr for attr in dir(results) if not attr.startswith('_')]}")

            if hasattr(results, 'points'):
                print(f"Has 'points' attribute: {type(results.points)}")
                if hasattr(results.points, '__len__'):
                    print(f"Number of points: {len(results.points)}")
                    if len(results.points) > 0:
                        first_point = results.points[0]
                        print(f"First point type: {type(first_point)}")
                        print(f"First point attributes: {[attr for attr in dir(first_point) if not attr.startswith('_')]}")
                        if hasattr(first_point, 'id'):
                            print(f"First point has id: {first_point.id}")
                        if hasattr(first_point, 'score'):
                            print(f"First point has score: {first_point.score}")
                        if hasattr(first_point, 'payload'):
                            print(f"First point has payload: {type(first_point.payload)}")

        except Exception as e:
            print(f"Query failed: {str(e)}")
            import traceback
            traceback.print_exc()

    except Exception as e:
        print(f"Client initialization failed: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_query_points()