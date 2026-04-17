"""OR-Tools solver wrapper implementing BaseSolver interface."""

from .ortools_impl import ORToolsSolverImpl
from .base import BaseSolver


class ORToolsSolver(BaseSolver):
    """OR-Tools solver implementation."""
    
    def __init__(self, data: dict):
        """Initialize OR-Tools solver with problem data."""
        # Use refactored implementation
        self._solver = ORToolsSolverImpl(data)
        self.data = data
    
    def _validate_data(self) -> None:
        """Validation is done in the implementation."""
        pass
    
    def _prepare_data(self) -> None:
        """Preparation is done in the implementation."""
        pass
    
    def solve(self, 
              time_limit_seconds: int = 60, 
              log_search: bool = False,
              vehicle_penalty_weight: float = 100000.0,
              distance_weight: float = 1.0,
              **kwargs):
        """Solve using OR-Tools."""
        return self._solver.solve(
            time_limit_seconds=time_limit_seconds,
            log_search=log_search,
            vehicle_penalty_weight=vehicle_penalty_weight,
            distance_weight=distance_weight
        )
    
    @property
    def solver_name(self) -> str:
        """Return solver name."""
        return "OR-Tools"

