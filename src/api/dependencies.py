"""API dependencies for authentication and validation."""

from typing import Optional
from fastapi import HTTPException, status, Security
from fastapi.security import APIKeyHeader
from src.config.settings import get_settings

# Security scheme for Swagger UI - API Key in header
api_key_header = APIKeyHeader(name="api-key", auto_error=False)


def verify_api_key(
    api_key: Optional[str] = Security(api_key_header)
) -> None:
    """
    Verify API key if authentication is enabled.
    
    This dependency checks if API_KEY environment variable is set.
    If set, it requires a valid API key in the 'api-key' header.
    If not set, authentication is disabled and all requests are allowed.
    
    Args:
        api_key: API key from 'api-key' header
        
    Raises:
        HTTPException: 401 if API key is invalid or missing when required
        
    Example:
        api-key: your-secret-key-here
    """
    settings = get_settings()
    
    # If API_KEY is not configured, skip authentication
    if not settings.api_key:
        return
    
    # If API_KEY is configured, require authentication
    if not api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing API key. Please provide 'api-key' header.",
        )
    
    # Verify API key
    if api_key != settings.api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid API key",
        )
