"""RAG (Retrieval-Augmented Generation) Service for AI-Native Book."""

import logging
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from openai import OpenAI
from config import settings

logger = logging.getLogger(__name__)


class RAGService:
    """Retrieval-Augmented Generation service combining Qdrant and OpenAI."""
    
    def __init__(self):
        """Initialize RAG service with Qdrant and OpenAI clients."""
        try:
            self.qdrant_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
            )
            logger.info("✓ Connected to Qdrant")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            self.qdrant_client = None
        
        try:
            self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
            logger.info("✓ Connected to OpenAI")
        except Exception as e:
            logger.warning(f"OpenAI not configured: {e}")
            self.openai_client = None
        
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.embedding_model = settings.OPENAI_EMBEDDING_MODEL
        self.chat_model = settings.OPENAI_MODEL
    
    def get_embedding(self, text: str) -> Optional[List[float]]:
        """Generate embedding for text using OpenAI."""
        if not self.openai_client:
            logger.warning("OpenAI client not available")
            return None
        
        try:
            response = self.openai_client.embeddings.create(
                input=text,
                model=self.embedding_model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            return None
    
    def search_similar_content(
        self,
        query: str,
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict]:
        """
        Search for similar content in Qdrant based on query.
        
        Args:
            query: User query string
            limit: Maximum number of results to return
            score_threshold: Minimum similarity score
            
        Returns:
            List of similar documents with scores
        """
        if not self.qdrant_client:
            logger.warning("Qdrant client not available")
            return []
        
        try:
            # Generate embedding for query
            query_embedding = self.get_embedding(query)
            if not query_embedding:
                logger.warning("Failed to generate query embedding")
                return []
            
            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                score_threshold=score_threshold,
            )
            
            # Format results
            results = []
            for point in search_results:
                result = {
                    "id": point.id,
                    "score": point.score,
                    "payload": point.payload
                }
                results.append(result)
            
            logger.info(f"Found {len(results)} similar documents for query")
            return results
        
        except Exception as e:
            logger.error(f"Search failed: {e}")
            return []
    
    def generate_response(
        self,
        query: str,
        context: List[Dict],
        system_prompt: Optional[str] = None
    ) -> str:
        """
        Generate response using OpenAI based on query and retrieved context.
        
        Args:
            query: User query
            context: Retrieved context from vector DB
            system_prompt: Optional custom system prompt
            
        Returns:
            Generated response text
        """
        if not self.openai_client:
            return "OpenAI not configured. Please add OPENAI_API_KEY to environment."
        
        try:
            # Format context
            context_text = self._format_context(context)
            
            # Build system prompt
            if not system_prompt:
                system_prompt = """You are a helpful AI assistant for the AI-Native Book. 
Answer questions based on the provided context from the book content. 
Be concise, accurate, and helpful. If the context doesn't contain relevant information, 
say so and provide your best general knowledge answer."""
            
            # Create messages
            messages = [
                {
                    "role": "system",
                    "content": system_prompt
                },
                {
                    "role": "user",
                    "content": f"""Context from the book:
{context_text}

Question: {query}"""
                }
            ]
            
            # Call OpenAI
            response = self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )
            
            answer = response.choices[0].message.content
            logger.info(f"Generated response for query: {query[:50]}...")
            return answer
        
        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            return f"Error generating response: {str(e)}"
    
    def chat(
        self,
        query: str,
        include_sources: bool = True,
        search_limit: int = 5
    ) -> Dict:
        """
        Complete chat flow: search relevant content and generate response.
        
        Args:
            query: User query
            include_sources: Whether to include source information
            search_limit: Number of context documents to retrieve
            
        Returns:
            Dictionary with response and optional sources
        """
        try:
            # Search for relevant content
            search_results = self.search_similar_content(
                query=query,
                limit=search_limit
            )
            
            # Generate response
            response = self.generate_response(query, search_results)
            
            # Build result
            result = {
                "query": query,
                "response": response,
                "sources": []
            }
            
            # Add sources if requested
            if include_sources and search_results:
                result["sources"] = [
                    {
                        "title": doc.get("payload", {}).get("title", "Unknown"),
                        "content": doc.get("payload", {}).get("content", "")[:200],
                        "score": doc.get("score", 0)
                    }
                    for doc in search_results
                ]
            
            return result
        
        except Exception as e:
            logger.error(f"Chat failed: {e}")
            return {
                "query": query,
                "response": f"Error processing query: {str(e)}",
                "sources": []
            }
    
    def _format_context(self, context: List[Dict]) -> str:
        """Format retrieved context for prompt."""
        if not context:
            return "No relevant context found."
        
        formatted = []
        for i, doc in enumerate(context, 1):
            payload = doc.get("payload", {})
            title = payload.get("title", "Unknown")
            content = payload.get("content", "")
            score = doc.get("score", 0)
            
            formatted.append(
                f"[{i}] {title} (relevance: {score:.2f})\n{content}"
            )
        
        return "\n\n".join(formatted)
    
    def health_check(self) -> Dict:
        """Check RAG service health."""
        status = {
            "qdrant": "connected" if self.qdrant_client else "disconnected",
            "openai": "connected" if self.openai_client else "disconnected"
        }
        
        try:
            if self.qdrant_client:
                collections = self.qdrant_client.get_collections()
                status["collections"] = len(collections.collections)
        except Exception as e:
            logger.error(f"Health check failed: {e}")
        
        return status


# Global RAG service instance
rag_service = RAGService()
