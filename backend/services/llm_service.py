# import os
# import requests
# from typing import List, Dict, Any
# from config.settings import settings


# class LLMService:
#     def __init__(self):
#         # Extract the base API URL (remove /chat/completions part if present)
#         base_url = settings.openrouter_base_url
#         if base_url.endswith('/chat/completions'):
#             base_url = base_url.replace('/chat/completions', '')
#         self.base_url = base_url
#         self.api_key = settings.openrouter_api_key
#         self.headers = {
#             "Authorization": f"Bearer {self.api_key}",
#             "Content-Type": "application/json"
#         }

#     def generate_response(self, prompt: str, context: str = "", selected_text: str = None) -> str:
#         """
#         Generate a response using the LLM with the given prompt and context.

#         Args:
#             prompt: The user's question or prompt
#             context: Retrieved context from vector search (or empty if using selected text mode)
#             selected_text: Text selected by user (if in selected-text-only mode)

#         Returns:
#             Generated response from the LLM
#         """
#         # If selected text is provided, use ONLY that as context
#         if selected_text:
#             full_prompt = f"Based ONLY on the following text, answer the question:\n\n{selected_text}\n\nQuestion: {prompt}"
#         else:
#             # Otherwise, use retrieved context if available
#             if context:
#                 full_prompt = f"Based on the following context, answer the question:\n\n{context}\n\nQuestion: {prompt}"
#             else:
#                 full_prompt = prompt

#         payload = {
#             "model": settings.chat_model,
#             "messages": [
#                 {
#                     "role": "user",
#                     "content": full_prompt
#                 }
#             ],
#             "temperature": 0.7
#         }

#         try:
#             response = requests.post(
#                 f"{self.base_url}/chat/completions",
#                 headers=self.headers,
#                 json=payload,
#                 timeout=60  # Add this line
#             )
#             response.raise_for_status()

#             result = response.json()
#             return result['choices'][0]['message']['content']
#         except requests.exceptions.RequestException as e:
#             raise Exception(f"Error calling OpenRouter API: {str(e)}")

#     def generate_embedding(self, text: str) -> List[float]:
#         """
#         Generate an embedding for the given text using the Qwen embedding model.

#         Args:
#             text: Text to generate embedding for

#         Returns:
#             List of float values representing the embedding vector
#         """
#         payload = {
#             "model": settings.embedding_model,
#             "input": text
#         }

#         try:
#             response = requests.post(
#                 f"{self.base_url}/embeddings",
#                 headers=self.headers,
#                 json=payload
#             )
#             response.raise_for_status()

#             result = response.json()
#             return result['data'][0]['embedding']
#         except requests.exceptions.RequestException as e:
#             raise Exception(f"Error calling OpenRouter embeddings API: {str(e)}")


# # Global instance
# llm_service = LLMService()


# /////////////////////////////////////////////////gemini
import os
import time
import requests
from typing import List, Dict, Any
from config.settings import settings

class LLMService:
    def __init__(self):
        # Clean the base URL to ensure it doesn't have double paths
        base_url = settings.openrouter_base_url
        if base_url.endswith('/chat/completions'):
            base_url = base_url.replace('/chat/completions', '')
        
        self.base_url = base_url
        self.api_key = settings.openrouter_api_key
        self.headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "HTTP-Referer": "http://localhost:8000", # Optional but good for OpenRouter
            "X-Title": "Physical AI Textbook Bot"
        }

    def generate_response(self, prompt: str, context: str = "", selected_text: str = None) -> str:
        """
        Generate a response with built-in retry logic and extended timeout.
        """
        # 1. Prepare the prompt based on available context
        if selected_text:
            full_prompt = f"Based ONLY on the following text, answer the question:\n\n{selected_text}\n\nQuestion: {prompt}"
        elif context:
            full_prompt = f"Based on the following context, answer the question:\n\n{context}\n\nQuestion: {prompt}"
        else:
            full_prompt = prompt

        payload = {
            "model": settings.chat_model,
            "messages": [
                {"role": "user", "content": full_prompt}
            ],
            "temperature": 0.7
        }

        # 2. Implementation of Retry Logic
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # We use a 60-second timeout to allow the LLM time to process
                response = requests.post(
                    f"{self.base_url}/chat/completions",
                    headers=self.headers,
                    json=payload,
                    timeout=60 
                )
                response.raise_for_status()
                result = response.json()
                
                if 'choices' in result and len(result['choices']) > 0:
                    return result['choices'][0]['message']['content']
                else:
                    raise Exception("Unexpected response format from OpenRouter")

            except (requests.exceptions.ReadTimeout, requests.exceptions.ConnectTimeout) as e:
                if attempt < max_retries - 1:
                    wait_time = (attempt + 1) * 2  # Exponential-ish backoff (2s, 4s)
                    print(f"⚠️ Timeout on attempt {attempt + 1}. Retrying in {wait_time}s...")
                    time.sleep(wait_time)
                    continue
                raise Exception(f"OpenRouter API timed out after {max_retries} attempts.")
            
            except requests.exceptions.RequestException as e:
                # Handle 404, 401, 500 etc.
                raise Exception(f"Error calling OpenRouter API: {str(e)}")

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate an embedding for the given text.
        """
        payload = {
            "model": settings.embedding_model,
            "input": text
        }

        try:
            response = requests.post(
                f"{self.base_url}/embeddings",
                headers=self.headers,
                json=payload,
                timeout=30 # Embeddings are usually faster
            )
            response.raise_for_status()
            result = response.json()
            return result['data'][0]['embedding']
        except requests.exceptions.RequestException as e:
            raise Exception(f"Error calling OpenRouter embeddings API: {str(e)}")

# Global instance
llm_service = LLMService()












