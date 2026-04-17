"""Application settings using Pydantic."""

from typing import Optional, List
from pydantic_settings import BaseSettings
from pydantic import Field
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings."""
    
    # API Settings
    app_name: str = Field("Fleet Route Optimizer API", description="Application name")
    api_host: str = Field("127.0.0.1", description="API host")
    api_port: int = Field(8000, description="API port")
    debug: bool = Field(False, description="Debug mode")
    
    # CORS Settings
    cors_origins: List[str] = Field(
        ["http://localhost:3000", "http://127.0.0.1:3000"],
        description="Allowed CORS origins"
    )
    
    # Solver Settings
    default_time_limit: int = Field(60, description="Default solver time limit in seconds")
    default_solver: str = Field("ortools", description="Default solver (ortools/gurobi)")
    ortools_vehicle_penalty: float = Field(100000.0, description="OR-Tools vehicle penalty weight")
    gurobi_vehicle_penalty: float = Field(1000.0, description="Gurobi vehicle penalty weight")
    default_distance_weight: float = Field(1.0, description="Default distance weight")
    default_mip_gap: float = Field(0.01, description="Default MIP gap for Gurobi")
    
    # Distance Cache Settings
    distance_cache_db: str = Field("distance_cache.db", description="Distance cache database path")
    osrm_base_url: str = Field("http://router.project-osrm.org", description="OSRM API base URL")
    
    # Logging Settings
    log_level: str = Field("INFO", description="Logging level")
    log_format: str = Field(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        description="Log format string"
    )
    
    # Security Settings
    api_key: Optional[str] = Field(None, description="API authentication key (optional)")
    
    # File paths
    inputs_dir: str = Field("inputs", description="Input files directory")
    results_dir: str = Field("results", description="Results output directory")
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
