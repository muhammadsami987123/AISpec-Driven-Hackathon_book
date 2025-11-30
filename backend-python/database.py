from typing import Optional, Dict
from datetime import datetime
import uuid

# In-memory user store (ready to integrate with Qdrant)
users_db: Dict[str, dict] = {}


class User:
    def __init__(self, full_name: str, email: str, password_hash: str, 
                 provider: str = "credentials", image: Optional[str] = None):
        self.id = f"user_{uuid.uuid4().hex[:12]}"
        self.full_name = full_name
        self.email = email
        self.password_hash = password_hash
        self.provider = provider
        self.image = image
        self.created_at = datetime.utcnow().isoformat()
        self.updated_at = datetime.utcnow().isoformat()
    
    def to_dict(self):
        return {
            "id": self.id,
            "fullName": self.full_name,
            "email": self.email,
            "passwordHash": self.password_hash,
            "provider": self.provider,
            "image": self.image,
            "createdAt": self.created_at,
            "updatedAt": self.updated_at,
        }
    
    def to_response(self):
        return {
            "id": self.id,
            "fullName": self.full_name,
            "email": self.email,
            "image": self.image,
            "createdAt": self.created_at,
        }


def create_user(full_name: str, email: str, password_hash: str) -> User:
    """Create and store a new user."""
    user = User(full_name=full_name, email=email, password_hash=password_hash)
    users_db[user.email] = user.to_dict()
    return user


def get_user_by_email(email: str) -> Optional[User]:
    """Get a user by email."""
    user_data = users_db.get(email)
    if not user_data:
        return None
    
    user = User(
        full_name=user_data["fullName"],
        email=user_data["email"],
        password_hash=user_data["passwordHash"],
        provider=user_data.get("provider", "credentials"),
        image=user_data.get("image")
    )
    user.id = user_data["id"]
    user.created_at = user_data["createdAt"]
    user.updated_at = user_data["updatedAt"]
    return user


def user_exists(email: str) -> bool:
    """Check if a user exists by email."""
    return email in users_db
