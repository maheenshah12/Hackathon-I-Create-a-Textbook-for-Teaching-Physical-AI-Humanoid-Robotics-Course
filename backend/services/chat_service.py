from typing import Dict, Any, Optional, List
from sqlalchemy.orm import Session
from models.database import Conversation, Message
from models.message import MessageModel
from models.conversation import ConversationModel
import uuid
from datetime import datetime


class ChatService:
    def __init__(self):
        pass

    def create_conversation(self, db: Session, metadata: Optional[Dict[str, Any]] = None) -> ConversationModel:
        conversation_id = str(uuid.uuid4())

        db_conversation = Conversation(
            id=conversation_id,
            metadata_=metadata
        )

        db.add(db_conversation)
        db.commit()
        db.refresh(db_conversation)

        return ConversationModel(
            id=db_conversation.id,
            created_at=db_conversation.created_at,
            updated_at=db_conversation.updated_at,
            metadata=db_conversation.metadata_,
            messages=[]
        )

    def get_conversation(self, db: Session, conversation_id: str) -> Optional[ConversationModel]:
        db_conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
        if not db_conversation:
            return None

        messages = []
        for db_message in db_conversation.messages:
            messages.append(MessageModel(
                id=db_message.id,
                conversation_id=db_message.conversation_id,
                role=db_message.role,
                content=db_message.content,
                timestamp=db_message.timestamp,
                sources=db_message.sources,
                selected_text=db_message.selected_text
            ))

        return ConversationModel(
            id=db_conversation.id,
            created_at=db_conversation.created_at,
            updated_at=db_conversation.updated_at,
            metadata=db_conversation.metadata_,
            messages=messages
        )

    def add_message(self, db: Session, conversation_id: str, role: str, content: str,
                    sources: Optional[List[str]] = None, selected_text: Optional[str] = None) -> MessageModel:
        message_id = str(uuid.uuid4())

        db_message = Message(
            id=message_id,
            conversation_id=conversation_id,
            role=role,
            content=content,
            sources=sources,
            selected_text=selected_text
        )

        db.add(db_message)
        db.commit()
        db.refresh(db_message)

        return MessageModel(
            id=db_message.id,
            conversation_id=db_message.conversation_id,
            role=db_message.role,
            content=db_message.content,
            timestamp=db_message.timestamp,
            sources=db_message.sources,
            selected_text=db_message.selected_text
        )

    def update_conversation_metadata(self, db: Session, conversation_id: str, metadata: Dict[str, Any]) -> bool:
        db_conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
        if not db_conversation:
            return False

        db_conversation.metadata_ = metadata
        db.commit()

        return True


# Global instance
chat_service = ChatService()
from typing import Dict, Any, Optional, List
from sqlalchemy.orm import Session
from models.database import Conversation, Message
from models.message import MessageModel
from models.conversation import ConversationModel
from database.session import get_db
from config.settings import settings
import uuid
from datetime import datetime


class ChatService:
    def __init__(self):
        pass

    def create_conversation(self, db: Session, metadata: Optional[Dict[str, Any]] = None) -> ConversationModel:
        """
        Create a new conversation.

        Args:
            db: Database session
            metadata: Optional metadata for the conversation

        Returns:
            Created conversation model
        """
        conversation_id = str(uuid.uuid4())

        db_conversation = Conversation(
            id=conversation_id,
            metadata_=metadata
        )

        db.add(db_conversation)
        db.commit()
        db.refresh(db_conversation)

        return ConversationModel(
            id=db_conversation.id,
            created_at=db_conversation.created_at,
            updated_at=db_conversation.updated_at,
            metadata=db_conversation.metadata_,
            messages=[]
        )

    def get_conversation(self, db: Session, conversation_id: str) -> Optional[ConversationModel]:
        """
        Get a conversation by ID.

        Args:
            db: Database session
            conversation_id: ID of the conversation

        Returns:
            Conversation model or None if not found
        """
        db_conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
        if not db_conversation:
            return None

        messages = []
        for db_message in db_conversation.messages:
            messages.append(MessageModel(
                id=db_message.id,
                conversation_id=db_message.conversation_id,
                role=db_message.role,
                content=db_message.content,
                timestamp=db_message.timestamp,
                sources=db_message.sources,
                selected_text=db_message.selected_text
            ))

        return ConversationModel(
            id=db_conversation.id,
            created_at=db_conversation.created_at,
            updated_at=db_conversation.updated_at,
            metadata=db_conversation.metadata_,
            messages=messages
        )

    def add_message(self, db: Session, conversation_id: str, role: str, content: str,
                    sources: Optional[List[str]] = None, selected_text: Optional[str] = None) -> MessageModel:
        """
        Add a message to a conversation.

        Args:
            db: Database session
            conversation_id: ID of the conversation
            role: Role of the message sender ('user' or 'assistant')
            content: Content of the message
            sources: Optional list of sources for the message
            selected_text: Optional selected text associated with the message

        Returns:
            Created message model
        """
        message_id = str(uuid.uuid4())

        db_message = Message(
            id=message_id,
            conversation_id=conversation_id,
            role=role,
            content=content,
            sources=sources,
            selected_text=selected_text
        )

        db.add(db_message)
        db.commit()
        db.refresh(db_message)

        return MessageModel(
            id=db_message.id,
            conversation_id=db_message.conversation_id,
            role=db_message.role,
            content=db_message.content,
            timestamp=db_message.timestamp,
            sources=db_message.sources,
            selected_text=db_message.selected_text
        )

    def update_conversation_metadata(self, db: Session, conversation_id: str, metadata: Dict[str, Any]) -> bool:
        """
        Update the metadata of a conversation.

        Args:
            db: Database session
            conversation_id: ID of the conversation
            metadata: New metadata to update

        Returns:
            True if successful, False otherwise
        """
        db_conversation = db.query(Conversation).filter(Conversation.id == conversation_id).first()
        if not db_conversation:
            return False

        db_conversation.metadata_ = metadata
        db.commit()

        return True


# Global instance
chat_service = ChatService()