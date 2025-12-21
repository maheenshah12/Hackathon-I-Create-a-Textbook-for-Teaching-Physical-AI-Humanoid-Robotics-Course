# from fastapi import APIRouter, Depends, HTTPException
# from sqlalchemy.orm import Session
# from typing import Optional
# from models.message import MessageRequest, MessageResponse
# from services.rag_service import get_rag_service
# from services.chat_service import chat_service
# from database.session import get_db
# from config.settings import settings


# router = APIRouter()


# @router.post("/chat", response_model=MessageResponse)
# async def chat(request: MessageRequest, db: Session = Depends(get_db)):
#     """
#     Process a chat message and return a response based on book content.
#     If selected_text is provided, respond only based on that text.
#     Otherwise, perform RAG using vector search.
#     """
#     try:
#         # If no conversation ID is provided, create a new conversation
#         if not request.conversation_id:
#             conversation = chat_service.create_conversation(db)
#             conversation_id = conversation.id
#         else:
#             conversation_id = request.conversation_id

#         # Add user message to conversation
#         chat_service.add_message(
#             db=db,
#             conversation_id=conversation_id,
#             role="user",
#             content=request.message,
#             selected_text=request.selected_text
#         )

#         # Generate response using RAG service
#         rag_result = get_rag_service().generate_response(
#             query=request.message,
#             conversation_id=conversation_id,
#             selected_text=request.selected_text
#         )

#         # Add assistant response to conversation
#         chat_service.add_message(
#             db=db,
#             conversation_id=rag_result["conversation_id"],
#             role="assistant",
#             content=rag_result["response"],
#             sources=rag_result["sources"]
#         )

#         return MessageResponse(
#             response=rag_result["response"],
#             conversation_id=rag_result["conversation_id"],
#             sources=rag_result["sources"]
#         )

#     except Exception as e:
#         raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


# @router.get("/conversations/{conversation_id}")
# async def get_conversation(conversation_id: str, db: Session = Depends(get_db)):
#     """
#     Retrieve conversation history
#     """
#     try:
#         conversation = chat_service.get_conversation(db, conversation_id)
#         if not conversation:
#             raise HTTPException(status_code=404, detail="Conversation not found")

#         return conversation
#     except Exception as e:
#         raise HTTPException(status_code=500, detail=f"Error retrieving conversation: {str(e)}")


# @router.delete("/conversations/{conversation_id}")
# async def delete_conversation(conversation_id: str, db: Session = Depends(get_db)):
#     """
#     Delete a conversation
#     """
#     try:
#         # For now, we'll just return a success response
#         # In a real implementation, you'd want to actually delete from the database
#         return {"status": "deleted", "conversation_id": conversation_id}
#     except Exception as e:
#         raise HTTPException(status_code=500, detail=f"Error deleting conversation: {str(e)}")
# ////////////////////////////////////////////////////////////////////////////////////////////////

import logging
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from models.message import MessageRequest, MessageResponse
from services.rag_service import get_rag_service
from services.chat_service import chat_service
from database.session import get_db

# Set up logger for this module
logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/chat", response_model=MessageResponse)
async def chat(request: MessageRequest, db: Session = Depends(get_db)):
    """
    Process a chat message and return a response based on book content.
    If selected_text is provided, respond only based on that text.
    Otherwise, perform RAG using vector search.
    """
    logger.debug(f"CHAT REQUEST RECEIVED: {request.dict()}")

    try:
        # Handle conversation_id
        if not request.conversation_id:
            conversation = chat_service.create_conversation(db)
            conversation_id = conversation.id
            logger.info(f"New conversation created: {conversation_id}")
        else:
            conversation_id = request.conversation_id
            logger.info(f"Continuing conversation: {conversation_id}")

        # Save user message
        chat_service.add_message(
            db=db,
            conversation_id=conversation_id,
            role="user",
            content=request.message,
            selected_text=request.selected_text
        )
        logger.info("User message saved to database")

        # Critical section: Generate response via RAG service
        logger.warning("=== ABOUT TO CALL generate_response() ===")
        rag_result = get_rag_service().generate_response(
            query=request.message,
            conversation_id=conversation_id,
            selected_text=request.selected_text
        )
        logger.warning("=== generate_response() RETURNED SUCCESSFULLY ===")
        logger.debug(f"RAG result keys: {rag_result.keys() if isinstance(rag_result, dict) else type(rag_result)}")

        # Save assistant response
        chat_service.add_message(
            db=db,
            conversation_id=conversation_id,
            role="assistant",
            content=rag_result["response"],
            sources=rag_result.get("sources", [])
        )
        logger.info("Assistant response saved")

        # Return to frontend
        return MessageResponse(
            response=rag_result["response"],
            conversation_id=conversation_id,
            sources=rag_result.get("sources", [])
        )

    except Exception as e:
        logger.error("=== FATAL ERROR IN /chat ENDPOINT ===", exc_info=True)
        logger.error(f"Exception type: {type(e).__name__}")
        logger.error(f"Exception message: {str(e)}")
        logger.error(f"Request data: {request.dict() if 'request' in locals() else 'N/A'}")

        raise HTTPException(
            status_code=500,
            detail="Internal processing error – check server logs"
        )


@router.get("/conversations/{conversation_id}")
async def get_conversation(conversation_id: str, db: Session = Depends(get_db)):
    logger.debug(f"Fetching conversation: {conversation_id}")
    try:
        conversation = chat_service.get_conversation(db, conversation_id)
        if not conversation:
            raise HTTPException(status_code=404, detail="Conversation not found")
        return conversation
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error fetching conversation {conversation_id}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to retrieve conversation")


@router.delete("/conversations/{conversation_id}")
async def delete_conversation(conversation_id: str, db: Session = Depends(get_db)):
    logger.info(f"Delete request for conversation: {conversation_id}")
    try:
        # Placeholder – implement actual deletion if needed
        return {"status": "deleted", "conversation_id": conversation_id}
    except Exception as e:
        logger.error(f"Error deleting conversation {conversation_id}", exc_info=True)
        raise HTTPException(status_code=500, detail="Failed to delete conversation")