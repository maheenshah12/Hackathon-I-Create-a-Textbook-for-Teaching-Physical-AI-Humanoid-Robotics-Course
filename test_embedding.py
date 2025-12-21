import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add backend to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend'))

def test_single_embedding():
    """Test a single embedding to verify the setup works"""
    print("Testing single embedding...")

    try:
        from backend.services.llm_service import llm_service

        # Test text
        test_text = "This is a test document for embedding."

        # Generate embedding
        embedding = llm_service.generate_embedding(test_text)

        print(f"SUCCESS: Successfully generated embedding with {len(embedding)} dimensions")
        print(f"First 5 dimensions: {embedding[:5]}")

        return True
    except Exception as e:
        print(f"ERROR: Error generating embedding: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_single_embedding()
    if success:
        print("\nSUCCESS: Embedding test completed successfully!")
        print("The embedding infrastructure is properly configured.")
    else:
        print("\nERROR: Embedding test failed!")