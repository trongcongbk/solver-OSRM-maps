"""API request/response models."""

from typing import Optional, Dict, List
from pydantic import BaseModel, Field

from .domain import Customer, Vehicle, Depot


class SolverConfig(BaseModel):
    """Solver configuration parameters."""
    time_limit: int = Field(60, description="Time limit in seconds", ge=1, le=3600)
    solver: str = Field("ortools", description="Solver type: 'ortools' or 'gurobi'")
    vehicle_penalty_weight: Optional[float] = Field(None, description="Weight for minimizing vehicles")
    distance_weight: float = Field(1.0, description="Weight for distance minimization")
    mip_gap: float = Field(0.01, description="MIP optimality gap for Gurobi")


class SolveRequest(BaseModel):
    """Request to solve a CVRPTW problem."""
    date: Optional[str] = Field(None, description="Date for the problem (YYYY-MM-DD)")
    depot: Depot = Field(..., description="Depot information")
    vehicles: List[Vehicle] = Field(..., description="List of available vehicles")
    customers: List[Customer] = Field(..., description="List of customers")
    metadata: Optional[Dict] = Field(None, description="Additional metadata")


class SolveResponse(BaseModel):
    """Response containing the solution."""
    date: str = Field(..., description="Date solved for")
    summary: Dict = Field(..., description="Summary statistics")
    routes: List[Dict] = Field(..., description="Detailed routes")
    objective_value: Optional[float] = Field(None, description="Objective function value")


class HealthResponse(BaseModel):
    """Health check response."""
    status: str = Field(..., description="Service status: 'ready' or 'busy'")
    message: Optional[str] = Field(None, description="Additional status message")
