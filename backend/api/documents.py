from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from typing import Optional
import os
from services.document_service import get_document_service
from config.settings import settings


router = APIRouter()


@router.post("/documents/ingest")
async def ingest_document(content: str = Form(...), source_path: str = Form(...), metadata: Optional[str] = Form(None)):
    try:
        result = get_document_service().process_and_ingest_document(
            content=content,
            source_path=source_path
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting document: {str(e)}")


@router.post("/documents/ingest-file")
async def ingest_document_file(file: UploadFile = File(...), source_path: Optional[str] = Form(None)):
    try:
        content = await file.read()
        content_str = content.decode('utf-8')

        actual_source_path = source_path or file.filename

        result = get_document_service().process_and_ingest_document(
            content=content_str,
            source_path=actual_source_path
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting document file: {str(e)}")


@router.delete("/documents/{source_path}")
async def delete_document(source_path: str):
    try:
        success = get_document_service().delete_document(source_path)

        if success:
            return {"status": "deleted", "source_path": source_path}
        else:
            raise HTTPException(status_code=500, detail=f"Error deleting document: {source_path}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting document: {str(e)}")


@router.get("/documents")
async def get_ingested_documents():
    try:
        documents = get_document_service().get_all_ingested_documents()
        return {"documents": documents}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving documents: {str(e)}")


@router.post("/documents/ingest-directory")
async def ingest_directory(directory_path: str = Form(...), file_pattern: str = Form("**/*.md")):
    try:
        if not os.path.isdir(directory_path):
            raise HTTPException(status_code=400, detail=f"Directory does not exist: {directory_path}")

        result = get_document_service().process_and_ingest_directory(
            directory_path=directory_path,
            file_pattern=file_pattern
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting directory: {str(e)}")
from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from typing import Optional
import os
from services.document_service import get_document_service
from config.settings import settings


router = APIRouter()


@router.post("/documents/ingest")
async def ingest_document(content: str = Form(...), source_path: str = Form(...), metadata: Optional[str] = Form(None)):
    """
    Ingest a document into the vector store for RAG retrieval.

    Args:
        content: The content of the document
        source_path: The path/filename of the source document
        metadata: Optional metadata about the document (JSON string)
    """
    try:
        result = get_document_service().process_and_ingest_document(
            content=content,
            source_path=source_path
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting document: {str(e)}")


@router.post("/documents/ingest-file")
async def ingest_document_file(file: UploadFile = File(...), source_path: Optional[str] = Form(None)):
    """
    Ingest a document file into the vector store for RAG retrieval.

    Args:
        file: The document file to ingest
        source_path: Optional custom source path (if not provided, uses original filename)
    """
    try:
        # Read file content
        content = await file.read()
        content_str = content.decode('utf-8')

        # Use provided source path or default to filename
        actual_source_path = source_path or file.filename

        result = get_document_service().process_and_ingest_document(
            content=content_str,
            source_path=actual_source_path
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting document file: {str(e)}")


@router.delete("/documents/{source_path}")
async def delete_document(source_path: str):
    """
    Delete a document and all its chunks from the vector store.

    Args:
        source_path: The path of the document to delete
    """
    try:
        success = get_document_service().delete_document(source_path)

        if success:
            return {"status": "deleted", "source_path": source_path}
        else:
            raise HTTPException(status_code=500, detail=f"Error deleting document: {source_path}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting document: {str(e)}")


@router.get("/documents")
async def get_ingested_documents():
    """
    Get a list of all documents that have been ingested into the vector store.
    """
    try:
        documents = get_document_service().get_all_ingested_documents()
        return {"documents": documents}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving documents: {str(e)}")


@router.post("/documents/ingest-directory")
async def ingest_directory(directory_path: str = Form(...), file_pattern: str = Form("**/*.md")):
    """
    Ingest all documents in a directory into the vector store.

    Args:
        directory_path: Path to the directory containing documents
        file_pattern: Pattern to match files (default: all markdown files)
    """
    try:
        if not os.path.isdir(directory_path):
            raise HTTPException(status_code=400, detail=f"Directory does not exist: {directory_path}")

        result = get_document_service().process_and_ingest_directory(
            directory_path=directory_path,
            file_pattern=file_pattern
        )

        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting directory: {str(e)}")