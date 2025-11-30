from pydantic import BaseModel, EmailStr, field_validator
from typing import Optional

class SignupRequest(BaseModel):
    fullName: str
    email: EmailStr
    password: str
    confirmPassword: str
    
    @field_validator('fullName')
    @classmethod
    def validate_full_name(cls, v):
        if len(v.strip()) < 2:
            raise ValueError('Full name must be at least 2 characters')
        return v.strip()
    
    @field_validator('password')
    @classmethod
    def validate_password(cls, v):
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not any(c.isupper() for c in v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not any(c.isdigit() for c in v):
            raise ValueError('Password must contain at least one number')
        return v
    
    @field_validator('confirmPassword')
    @classmethod
    def validate_confirm_password(cls, v, info):
        if 'password' in info.data and v != info.data['password']:
            raise ValueError('Passwords do not match')
        return v


class LoginRequest(BaseModel):
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    id: str
    fullName: str
    email: str
    image: Optional[str] = None
    createdAt: Optional[str] = None


class AuthResponse(BaseModel):
    message: str
    user: UserResponse
    token: str


class ProfileResponse(BaseModel):
    user: UserResponse
