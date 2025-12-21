from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from .message import MessageModel


class ConversationModel(BaseModel):
    id: str
    created_at: datetime
    updated_at: datetime
    metadata: Optional[dict] = None
    messages: List[MessageModel] = []
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
from .message import MessageModel


class ConversationRequest(BaseModel):
    conversation_id: Optional[str] = None


class ConversationResponse(BaseModel):
    conversation_id: str
    messages: List[MessageModel]
    created_at: datetime
    updated_at: datetime
    metadata: Optional[Dict[str, Any]] = None


class ConversationModel(BaseModel):
    id: str
    created_at: datetime
    updated_at: datetime
    metadata: Optional[Dict[str, Any]] = None
    messages: List[MessageModel] = []