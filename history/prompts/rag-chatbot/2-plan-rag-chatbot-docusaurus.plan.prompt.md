---
id: 2
title: "plan-rag-chatbot-docusaurus"
stage: "plan"
date_iso: "2025-12-16"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "rag-chatbot"
branch: "1-rag-chatbot"
user: "USER"
command: "/sp.plan"
labels: ["plan", "rag", "chatbot", "docusaurus", "architecture"]
links:
  spec: "specs/1-rag-chatbot/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/1-rag-chatbot/plan.md"
  - "specs/1-rag-chatbot/research.md"
  - "specs/1-rag-chatbot/data-model.md"
  - "specs/1-rag-chatbot/quickstart.md"
  - "specs/1-rag-chatbot/contracts/chat-api.yaml"
tests_yaml:
  - "plan validation"
  - "API contract validation"
prompt_text: "## Phase 1: Architecture Design
- Define backend FastAPI service structure
- Define RAG pipeline components
- Define data flow between frontend and backend

## Phase 2: Vector Ingestion
- Extract Docusaurus markdown content
- Chunk book content
- Generate embeddings using Qwen
- Store vectors in Qdrant Cloud

## Phase 3: RAG Query Pipeline
- Implement similarity search in Qdrant
- Assemble context from retrieved chunks
- Generate responses using OpenRouter + Qwen

## Phase 4: Selected-Text Answering
- Detect user-selected text in Docusaurus UI
- Bypass vector retrieval when selected text is present
- Restrict LLM context strictly to selected text

## Phase 5: Frontend Integration
- Create chatbot UI component in Docusaurus
- Connect UI to FastAPI endpoints
- Embed chatbot globally or per-page

## Phase 6: Persistence & Metadata
- Store chat metadata in Neon Postgres
- Track queries and document references

## Phase 7: Deployment
- Deploy FastAPI backend
- Configure environment variables
- Redeploy Docusaurus"

response_text: "Created comprehensive implementation plan for RAG Chatbot feature, including updated technical context with OpenRouter and Qwen models, research findings, data model, quickstart guide, and API contracts. The plan addresses all 7 phases of development with detailed implementation steps."

outcome: "Implementation plan created and validated successfully"
evaluation: "Plan meets all requirements and is ready for task breakdown phase"
---

# Prompt History Record: plan-rag-chatbot-docusaurus

## Summary
Created comprehensive implementation plan for the RAG chatbot feature with OpenRouter and Qwen models.

## Context
User requested implementation plan for RAG chatbot with specific architecture requirements using OpenRouter API and Qwen models.

## Actions Taken
1. Updated existing plan with new requirements (OpenRouter/Qwen instead of OpenAI)
2. Created research.md with technology decisions and rationale
3. Created data-model.md with entity definitions
4. Created quickstart.md with setup instructions
5. Created API contract in contracts/chat-api.yaml

## Results
- Updated plan at specs/1-rag-chatbot/plan.md
- Research document at specs/1-rag-chatbot/research.md
- Data model at specs/1-rag-chatbot/data-model.md
- Quickstart guide at specs/1-rag-chatbot/quickstart.md
- API contract at specs/1-rag-chatbot/contracts/chat-api.yaml

## Next Steps
Plan is ready for task breakdown phase (/sp.tasks)