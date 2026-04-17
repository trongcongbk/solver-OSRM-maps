"""Factory for creating solver instances."""

from typing import Dict
from fastapi import HTTPException

from .base import BaseSolver, SolverType
from .ortools_solver import ORToolsSolver
from .gurobi_solver import GurobiSolver, GUROBI_AVAILABLE
from ...config import get_logger

logger = get_logger(__name__)


class SolverFactory:
    """Factory for creating solver instances based on solver type."""
    
    @staticmethod
    def create(solver_type: str, problem: Dict) -> BaseSolver:
        """
        Create a solver instance.
        
        Args:
            solver_type: Type of solver ('ortools' or 'gurobi')
            problem: Problem data dictionary
            
        Returns:
            Solver instance
            
        Raises:
            HTTPException: If solver type is invalid or unavailable
        """
        solver_type_lower = solver_type.lower()
        
        if solver_type_lower == SolverType.GUROBI:
            if not GUROBI_AVAILABLE:
                raise HTTPException(
                    status_code=400,
                    detail="Gurobi solver not available. Install with: pip install gurobipy and ensure license is configured"
                )
            logger.info("Using Gurobi solver")
            return GurobiSolver(problem)
        
        elif solver_type_lower == SolverType.ORTOOLS:
            logger.info("Using OR-Tools solver")
            return ORToolsSolver(problem)
        
        else:
            raise HTTPException(
                status_code=400,
                detail=f"Unknown solver type: {solver_type}. Valid options: 'ortools', 'gurobi'"
            )


def create_solver(solver_type: str, problem: Dict) -> BaseSolver:
    """
    Convenience function to create a solver instance.
    
    Args:
        solver_type: Type of solver ('ortools' or 'gurobi')
        problem: Problem data dictionary
        
    Returns:
        Solver instance
    """
    return SolverFactory.create(solver_type, problem)
