"""Logging configuration."""

import logging
import sys
from typing import Optional

from .settings import get_settings


def setup_logging(level: Optional[str] = None) -> None:
    """
    Setup logging configuration for the application.
    
    Args:
        level: Optional logging level override (default from settings)
    """
    settings = get_settings()
    log_level = level or settings.log_level
    
    # Configure root logger
    logging.basicConfig(
        level=getattr(logging, log_level.upper()),
        format=settings.log_format,
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    # Set specific loggers
    logging.getLogger("uvicorn").setLevel(logging.INFO)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger with the specified name.
    
    Args:
        name: Logger name (typically __name__)
        
    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)
