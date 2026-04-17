"""Gurobi solver wrapper implementing BaseSolver interface."""

try:
    from .gurobi_impl import GurobiSolverImpl, GUROBI_AVAILABLE
except ImportError:
    GUROBI_AVAILABLE = False
    GurobiSolverImpl = None

from .base import BaseSolver


class GurobiSolver(BaseSolver):
    """Gurobi solver implementation."""
    
    def __init__(self, data: dict):
        """Initialize Gurobi solver with problem data."""
        if not GUROBI_AVAILABLE:
            raise RuntimeError("Gurobi is not available")
        # Use refactored implementation
        self._solver = GurobiSolverImpl(data)
        self.data = data
    
    def _validate_data(self) -> None:
        """Validation is done in the original implementation."""
        pass
    
    def _prepare_data(self) -> None:
        """Preparation is done in the original implementation."""
        pass
    
    def solve(self, 
              time_limit_seconds: int = 60, 
              log_search: bool = False,
              vehicle_penalty_weight: float = 1000.0,
              distance_weight: float = 1.0,
              mip_gap: float = 0.01,
              **kwargs):
        """Solve using Gurobi."""
        return self._solver.solve(
            time_limit_seconds=time_limit_seconds,
            log_search=log_search,
            vehicle_penalty_weight=vehicle_penalty_weight,
            distance_weight=distance_weight,
            mip_gap=mip_gap
        )
    
    @property
    def solver_name(self) -> str:
        """Return solver name."""
        return "Gurobi"
