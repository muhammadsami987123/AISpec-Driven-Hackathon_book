from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    PORT: int = 5000
    HOST: str = "0.0.0.0"
    NODE_ENV: str = "development"
    FRONTEND_URL: str = "http://localhost:3000"
    
    # Qdrant Configuration
    QDRANT_URL: str = "https://3462b505-216d-4929-8c71-19abd4ed08d6.europe-west3-0.gcp.cloud.qdrant.io:6333"
    QDRANT_API_KEY: str = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.p-WpDXKqzTWNQWJBMgrbEd2mt0eIYemvR8SPfPcpN7o"
    QDRANT_COLLECTION_NAME: str = "ai-native-book"
    
    # OpenAI Configuration
    OPENAI_API_KEY: Optional[str] = None
    OPENAI_MODEL: str = "gpt-3.5-turbo"
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    
    # JWT Configuration
    JWT_SECRET: str = "your-super-secret-jwt-key-change-in-production"
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_HOURS: int = 168  # 7 days
    
    # Google OAuth (for future)
    GOOGLE_CLIENT_ID: Optional[str] = None
    GOOGLE_CLIENT_SECRET: Optional[str] = None
    
    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
