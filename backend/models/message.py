from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class MessageRequest(BaseModel):
    message: str
    conversation_id: Optional[str] = None
    selected_text: Optional[str] = None


class MessageResponse(BaseModel):
    response: str
    conversation_id: str
    sources: Optional[List[str]] = None


class MessageModel(BaseModel):
    id: str
    conversation_id: str
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime
    sources: Optional[List[str]] = None
    selected_text: Optional[str] = None