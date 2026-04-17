"""Base solver interface for CVRPTW problem."""

from abc import ABC, abstractmethod
from typing import Dict, Optional
from enum import Enum


class SolverType(str, Enum):
    """Supported solver types."""
    ORTOOLS = "ortools"
    GUROBI = "gurobi"


class BaseSolver(ABC):
    """Abstract base class for CVRPTW solvers."""
    
    def __init__(self, data: Dict):
        """
        Initialize the solver with problem data.
        
        Args:
            data: Dictionary containing problem definition
        """
        self.data = data
        self._validate_data()
        self._prepare_data()
    
    @abstractmethod
    def _validate_data(self) -> None:
        """Validate input data structure and constraints."""
        pass
    
    @abstractmethod
    def _prepare_data(self) -> None:
        """Prepare and compute derived data (e.g., distance matrices)."""
        pass
    
    @abstractmethod
    def solve(self, 
              time_limit_seconds: int = 60, 
              log_search: bool = False,
              vehicle_penalty_weight: float = None,
              distance_weight: float = 1.0,
              **kwargs) -> Optional[Dict]:
        """
        Solve the CVRPTW problem.
        
        Args:
            time_limit_seconds: Maximum time for solver
            log_search: Whether to log search progress
            vehicle_penalty_weight: Weight for minimizing number of vehicles
            distance_weight: Weight for distance in objective function
            **kwargs: Additional solver-specific parameters
            
        Returns:
            Dictionary containing solution details or None if no solution found
        """
        pass
    
    @property
    @abstractmethod
    def solver_name(self) -> str:
        """Return the name of the solver."""
        pass
