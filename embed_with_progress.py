import os
import sys
import time
from dotenv import load_dotenv

# Load environment variables from .env file in root
load_dotenv()

# Add backend to the Python path so we can import the services
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

def embed_book_content_with_progress():
    """
    Embed your Docusaurus book content into Qdrant with progress tracking
    """
    print("Starting to embed your book content...")

    # Import services after setting up the path
    from backend.services.document_service import document_service

    # Check if docs directory exists
    docs_dir = "./docs"
    if not os.path.exists(docs_dir):
        print(f"Error: {docs_dir} directory not found!")
        print("Available directories in root:")
        for item in os.listdir('.'):
            if os.path.isdir(item):
                print(f"  - {item}")
        return

    print(f"Processing documents from {docs_dir}...")

    try:
        # Process all markdown files in the docs directory
        print("Starting document processing...")
        result = document_service.process_and_ingest_directory(
            directory_path=docs_dir,
            file_pattern="**/*.md"
        )

        print("Embedding completed successfully!")
        print(f"Processed files: {len(result['processed_files'])}")
        print(f"Total chunks created: {result['total_chunks']}")

        if result['failed_files']:
            print(f"Failed files: {len(result['failed_files'])}")
            for failed in result['failed_files']:
                print(f"  - {failed['file_path']}: {failed['error']}")
        else:
            print("All files processed successfully!")

        # Check how many points are now in the collection
        from backend.vector_store.qdrant_client import qdrant_client
        try:
            collection_info = qdrant_client.client.get_collection(qdrant_client.collection_name)
            print(f"Final collection status: {collection_info.points_count} points in '{qdrant_client.collection_name}' collection")
        except Exception as e:
            print(f"Could not get final collection status: {str(e)}")

        return result
    except Exception as e:
        print(f"Error during embedding: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("Starting embedding process with progress tracking...")
    start_time = time.time()
    embed_book_content_with_progress()
    end_time = time.time()
    print(f"\nEmbedding process completed in {end_time - start_time:.2f} seconds")