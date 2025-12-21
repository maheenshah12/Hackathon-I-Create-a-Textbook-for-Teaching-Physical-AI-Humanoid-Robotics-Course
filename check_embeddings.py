import os
import sys
import time
from dotenv import load_dotenv

# Load environment variables from .env file in root
load_dotenv()

# Add backend to the Python path so we can import the services
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

def check_embeddings():
    """
    Check if embeddings have been created in Qdrant
    """
    print("Checking if embeddings exist in Qdrant...")

    try:
        # Import services after setting up the path
        from backend.vector_store.qdrant_client import qdrant_client

        # Try to get the collection info or count
        try:
            # Get list of all source paths that have been ingested
            all_docs = qdrant_client.get_all_source_paths()
            print(f"Found {len(all_docs)} documents already embedded in Qdrant")

            if len(all_docs) > 0:
                print("Documents found in Qdrant:")
                for doc in all_docs:
                    print(f"  - {doc}")
                return True
            else:
                print("No documents found in Qdrant yet.")
                return False
        except Exception as e:
            print(f"Could not get source paths from Qdrant: {str(e)}")

            # Try an alternative method - check if collection exists and has points
            try:
                collection_info = qdrant_client.client.get_collection(qdrant_client.collection_name)
                print(f"Collection '{qdrant_client.collection_name}' exists with {collection_info.points_count} points")
                return collection_info.points_count > 0
            except Exception as e2:
                print(f"Could not connect to Qdrant or collection doesn't exist: {str(e2)}")
                return False

    except ImportError as e:
        print(f"Could not import Qdrant client: {str(e)}")
        return False
    except Exception as e:
        print(f"Error checking embeddings: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print("Checking embedding status...")

    # Wait a bit to allow any ongoing embedding to potentially complete
    print("Waiting 10 seconds before checking...")
    time.sleep(10)

    # Check if embeddings exist
    has_embeddings = check_embeddings()

    if has_embeddings:
        print("\nSUCCESS: Embeddings have been created in Qdrant!")
        print("Your Physical AI Humanoid Robotics curriculum is ready for use.")
    else:
        print("\nINFO: No embeddings found in Qdrant yet.")
        print("The embedding process may still be running or may need to be restarted after verifying your API key.")

if __name__ == "__main__":
    main()