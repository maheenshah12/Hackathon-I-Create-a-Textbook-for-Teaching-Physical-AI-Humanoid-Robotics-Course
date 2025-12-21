# # ////
# import logging

# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
#     force=True
# )

# logger = logging.getLogger(__name__)
# logger.info("=== BACKEND STARTED WITH FULL DEBUG LOGGING ===")

# # //
# from fastapi import FastAPI
# from fastapi.middleware.cors import CORSMiddleware
# from dotenv import load_dotenv
# from api.chat import router as chat_router
# from api.documents import router as documents_router

# # Load environment variables
# load_dotenv()

# app = FastAPI(
#     title="RAG Chatbot Backend",
#     description="Backend API for RAG chatbot embedded in Docusaurus book",
#     version="1.0.0"
# )

# # Add CORS middleware to allow requests from Docusaurus frontend
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=["*"],  # In production, replace with specific origins
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # Include API routes
# app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
# app.include_router(documents_router, prefix="/api/v1", tags=["documents"])

# @app.get("/")
# async def root():
#     return {"message": "RAG Chatbot Backend API"}

# @app.get("/health")
# async def health_check():
#     return {"status": "healthy"}

# if __name__ == "__main__":
#     import uvicorn
#     uvicorn.run(app, host="0.0.0.0", port=8000)
# ///////////////////////////////
import logging
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import traceback
from dotenv import load_dotenv

from api.chat import router as chat_router
from api.documents import router as documents_router

# Load environment variables
load_dotenv()

# Optional: enable basic debug logging
logging.basicConfig(level=logging.DEBUG)

app = FastAPI(
    title="RAG Chatbot Backend",
    description="Backend API for RAG chatbot embedded in Docusaurus book",
    version="1.0.0"
)

# === GLOBAL EXCEPTION HANDLER - THIS WILL PRINT THE REAL ERROR ===
@app.exception_handler(Exception)
async def debug_exception_handler(request: Request, exc: Exception):
    tb = traceback.format_exc()
    print("\n" + "="*80)
    print("CRITICAL ERROR - FULL TRACEBACK (COPY THIS TO FIX THE BUG)")
    print("="*80)
    print(tb)
    print("="*80 + "\n")
    return JSONResponse(
        status_code=500,
        content={"detail": "Server error - full traceback printed in terminal. Check logs!"}
    )

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your Docusaurus URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(documents_router, prefix="/api/v1", tags=["documents"])

@app.get("/")
async def root():
    return {"message": "RAG Chatbot Backend API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)