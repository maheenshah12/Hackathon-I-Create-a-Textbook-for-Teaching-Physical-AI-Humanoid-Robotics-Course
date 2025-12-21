import re


def clean_text(text: str) -> str:
    text = re.sub(r"\s+", " ", text).strip()
    return text
import re
from typing import List, Tuple
import markdown


class TextProcessor:
    def __init__(self, chunk_size: int = 1000, overlap: int = 100):
        self.chunk_size = chunk_size
        self.overlap = overlap

    def extract_text_from_markdown(self, markdown_content: str) -> str:
        """
        Extract plain text from markdown content, preserving important structure.

        Args:
            markdown_content: Raw markdown text

        Returns:
            Plain text extracted from markdown
        """
        # Convert markdown to HTML first
        html_content = markdown.markdown(markdown_content)

        # Remove HTML tags to get plain text
        clean_text = re.sub('<[^<]+?>', '', html_content)

        # Clean up extra whitespace
        clean_text = re.sub(r'\n\s*\n', '\n\n', clean_text)
        clean_text = clean_text.strip()

        return clean_text

    def chunk_text(self, text: str, source_path: str) -> List[dict]:
        """
        Split text into overlapping chunks for better context preservation.

        Args:
            text: Text to be chunked
            source_path: Path of the source document

        Returns:
            List of chunks with metadata
        """
        # Split text into sentences to avoid breaking them
        sentences = re.split(r'(?<=[.!?]) +', text)

        chunks = []
        current_chunk = ""
        chunk_index = 0

        for sentence in sentences:
            # If adding this sentence would exceed chunk size
            if len(current_chunk) + len(sentence) > self.chunk_size and current_chunk:
                # Save current chunk
                chunks.append({
                    'content': current_chunk.strip(),
                    'source_path': source_path,
                    'chunk_index': chunk_index,
                    'metadata': {'type': 'text_chunk'}
                })

                # Start new chunk with overlap
                overlap_text = self._get_overlap(current_chunk, self.overlap)
                current_chunk = overlap_text + " " + sentence
                chunk_index += 1
            else:
                current_chunk += " " + sentence if current_chunk else sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'source_path': source_path,
                'chunk_index': chunk_index,
                'metadata': {'type': 'text_chunk'}
            })

        return chunks

    def _get_overlap(self, text: str, overlap_size: int) -> str:
        """
        Get the last 'overlap_size' characters from text, trying to break at word boundaries.

        Args:
            text: Text to extract overlap from
            overlap_size: Number of characters for overlap

        Returns:
            Overlapping text segment
        """
        if len(text) <= overlap_size:
            return text

        # Try to break at word boundary
        overlap_text = text[-overlap_size:]
        last_space = overlap_text.rfind(' ')

        if last_space != -1:
            return overlap_text[last_space:].strip()
        else:
            return overlap_text

    def process_document(self, content: str, source_path: str) -> List[dict]:
        """
        Process a document: extract text and chunk it.

        Args:
            content: Raw document content
            source_path: Path of the source document

        Returns:
            List of processed chunks
        """
        # Extract text from markdown
        text = self.extract_text_from_markdown(content)

        # Chunk the text
        chunks = self.chunk_text(text, source_path)

        return chunks


# Global instance
text_processor = TextProcessor()