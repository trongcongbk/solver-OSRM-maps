"""Data models for CVRPTW solver."""

from .domain import (
    Location,
    TimeWindow,
    Customer,
    Vehicle,
    Depot,
    ProblemData,
    RouteStop,
    Route,
    Solution,
)
from .api import (
    SolveRequest,
    SolveResponse,
    HealthResponse,
    SolverConfig,
)

__all__ = [
    # Domain models
    "Location",
    "TimeWindow",
    "Customer",
    "Vehicle",
    "Depot",
    "ProblemData",
    "RouteStop",
    "Route",
    "Solution",
    # API models
    "SolveRequest",
    "SolveResponse",
    "HealthResponse",
    "SolverConfig",
]
