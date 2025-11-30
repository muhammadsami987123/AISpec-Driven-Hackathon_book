from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
from datetime import timedelta
import logging

from config import settings
from schemas import SignupRequest, LoginRequest, AuthResponse, ProfileResponse, UserResponse
from auth import hash_password, verify_password, create_access_token, verify_token
from database import create_user, get_user_by_email, user_exists
from rag import rag_service

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="AI-Native Book Backend",
    description="Authentication API with Qdrant integration",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.FRONTEND_URL],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Dependency to verify token
async def verify_auth_token(authorization: Optional[str] = Header(None)):
    """Verify JWT token from Authorization header."""
    if not authorization:
        raise HTTPException(status_code=401, detail="Access token required")
    
    try:
        scheme, token = authorization.split()
        if scheme.lower() != "bearer":
            raise ValueError("Invalid auth scheme")
    except ValueError:
        raise HTTPException(status_code=401, detail="Invalid authorization header")
    
    payload = verify_token(token)
    if not payload:
        raise HTTPException(status_code=403, detail="Invalid or expired token")
    
    return payload


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    from datetime import datetime
    return {
        "status": "ok",
        "timestamp": datetime.utcnow().isoformat()
    }


@app.post("/api/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """Create a new user account."""
    try:
        # Check if user already exists
        if user_exists(request.email):
            raise HTTPException(status_code=409, detail="Email already registered")
        
        # Hash password
        password_hash = hash_password(request.password)
        
        # Create user
        user = create_user(
            full_name=request.fullName,
            email=request.email,
            password_hash=password_hash
        )
        
        # Generate token
        token = create_access_token(
            data={"userId": user.id, "email": user.email},
            expires_delta=timedelta(hours=settings.JWT_EXPIRATION_HOURS)
        )
        
        logger.info(f"User created: {user.email}")
        
        return {
            "message": "User created successfully",
            "user": user.to_response(),
            "token": token
        }
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signup error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/api/auth/login", response_model=AuthResponse)
async def login(request: LoginRequest):
    """Login with email and password."""
    try:
        # Find user
        user = get_user_by_email(request.email)
        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")
        
        # Verify password
        if not verify_password(request.password, user.password_hash):
            raise HTTPException(status_code=401, detail="Invalid email or password")
        
        # Generate token
        token = create_access_token(
            data={"userId": user.id, "email": user.email},
            expires_delta=timedelta(hours=settings.JWT_EXPIRATION_HOURS)
        )
        
        logger.info(f"User logged in: {user.email}")
        
        return {
            "message": "Login successful",
            "user": user.to_response(),
            "token": token
        }
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/api/users/profile", response_model=ProfileResponse)
async def get_profile(payload: dict = Depends(verify_auth_token)):
    """Get authenticated user's profile."""
    try:
        user = get_user_by_email(payload.get("email"))
        if not user:
            raise HTTPException(status_code=404, detail="User not found")
        
        return {"user": user.to_response()}
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Profile fetch error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.put("/api/users/profile")
async def update_profile(request: dict, payload: dict = Depends(verify_auth_token)):
    """Update authenticated user's profile."""
    try:
        user = get_user_by_email(payload.get("email"))
        if not user:
            raise HTTPException(status_code=404, detail="User not found")
        
        # Update user fields
        if "fullName" in request:
            user.full_name = request["fullName"]
        
        # Save updated user (in-memory for now)
        # In production, you'd update the database
        
        return {"user": user.to_response()}
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Profile update error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/api/auth/logout")
async def logout():
    """Logout endpoint (client-side token removal)."""
    return {"message": "Logged out successfully"}


@app.get("/api/rag/health")
async def rag_health():
    """Check RAG service health."""
    return rag_service.health_check()


@app.post("/api/rag/chat")
async def rag_chat(request: dict, payload: dict = Depends(verify_auth_token)):
    """
    Chat endpoint: retrieve relevant content and generate response.
    
    Request body:
    {
        "query": "Your question here",
        "include_sources": true,
        "search_limit": 5
    }
    """
    try:
        query = request.get("query", "").strip()
        if not query:
            raise HTTPException(status_code=400, detail="Query cannot be empty")
        
        include_sources = request.get("include_sources", True)
        search_limit = request.get("search_limit", 5)
        
        # Perform RAG chat
        result = rag_service.chat(
            query=query,
            include_sources=include_sources,
            search_limit=search_limit
        )
        
        logger.info(f"Chat response generated for user: {payload.get('email')}")
        return result
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"RAG chat error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/api/rag/search")
async def rag_search(request: dict, payload: dict = Depends(verify_auth_token)):
    """
    Search endpoint: retrieve relevant content from vector database.
    
    Request body:
    {
        "query": "Your search query",
        "limit": 5,
        "score_threshold": 0.7
    }
    """
    try:
        query = request.get("query", "").strip()
        if not query:
            raise HTTPException(status_code=400, detail="Query cannot be empty")
        
        limit = request.get("limit", 5)
        score_threshold = request.get("score_threshold", 0.7)
        
        # Search
        results = rag_service.search_similar_content(
            query=query,
            limit=limit,
            score_threshold=score_threshold
        )
        
        logger.info(f"Search performed for user: {payload.get('email')}")
        return {
            "query": query,
            "results": results,
            "count": len(results)
        }
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"RAG search error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")



@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "AI-Native Book Backend API",
        "version": "1.0.0",
        "docs": "/docs"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host=settings.HOST,
        port=settings.PORT,
        log_level="info"
    )
