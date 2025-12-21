import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env file in root
load_dotenv()

# Add backend to the Python path so we can import the services
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

def embed_book_content():
    """
    Embed your Docusaurus book content into Qdrant
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

        return result
    except Exception as e:
        print(f"Error during embedding: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    embed_book_content()