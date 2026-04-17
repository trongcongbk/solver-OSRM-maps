"""Solver orchestration service."""

import time
import threading
from typing import Dict, Optional

from ..core.solvers import create_solver
from ..config import get_logger, get_settings
from .distance_cache import DistanceCacheService
from .problem_builder import ProblemBuilder

logger = get_logger(__name__)


class SolverService:
    """Service for orchestrating the solving process."""
    
    def __init__(self):
        """Initialize solver service."""
        settings = get_settings()
        self.distance_cache = DistanceCacheService(
            db_path=settings.distance_cache_db,
            osrm_base_url=settings.osrm_base_url
        )
        self.problem_builder = ProblemBuilder()
        self._solver_lock = threading.Lock()
        self._solver_running = False
    
    def is_busy(self) -> bool:
        """Check if solver is currently running."""
        return self._solver_running
    
    def solve(self, 
              payload: dict,
              solver_type: str = "ortools",
              time_limit: int = 60,
              vehicle_penalty_weight: Optional[float] = None,
              distance_weight: float = 1.0,
              mip_gap: float = 0.01) -> Dict:
        """
        Solve a CVRPTW problem from a JSON payload.
        
        Args:
            payload: Problem data in JSON format
            solver_type: Solver to use ('ortools' or 'gurobi')
            time_limit: Time limit in seconds
            vehicle_penalty_weight: Weight for minimizing vehicles
            distance_weight: Weight for distance minimization
            mip_gap: MIP gap for Gurobi
            
        Returns:
            Solution dictionary
            
        Raises:
            ValueError: If solver is busy or problem has no active customers
        """
        # Check if solver is already running
        if not self._solver_lock.acquire(blocking=False):
            raise ValueError("Solver is already running. Try again later.")
        
        try:
            self._solver_running = True
            start_time = time.time()
            
            # Infer date from payload
            solved_date = self.problem_builder.infer_date_from_payload(payload) or "unknown"
            logger.info(f"========== SOLVING FOR DATE: {solved_date} ==========")
            
            # Build problem from payload
            problem = self.problem_builder.build_from_payload(payload, solved_date)
            if not problem:
                logger.info(f"No active customers on {solved_date}")
                return {"status": "no_active_customers", "date": solved_date}
            
            # Fetch real distances and travel times from cache
            logger.info("Fetching distances and travel times from cache...")

            distance_matrix, time_matrix_morning, time_matrix_afternoon, time_matrix_evening = (
                self.distance_cache.populate_matrix_all_times(problem['locations'])
            )
            
            # Replace problem matrices with real-world data
            problem['distance_matrix'] = distance_matrix
            
            # Build time matrix based on delivery time windows
            problem['time_matrix'] = self._build_time_matrix(
                problem, distance_matrix, time_matrix_morning, 
                time_matrix_afternoon, time_matrix_evening
            )
            
            # Remove obsolete vehicle_speed parameter
            problem.pop('vehicle_speed', None)
            
            # Set default vehicle penalty weight based on solver
            if vehicle_penalty_weight is None:
                settings = get_settings()
                vehicle_penalty_weight = (
                    settings.ortools_vehicle_penalty if solver_type == 'ortools' 
                    else settings.gurobi_vehicle_penalty
                )
            
            # Create solver and solve
            logger.info(f"Creating {solver_type} solver...")
            solver = create_solver(solver_type, problem)
            
            solve_params = {
                'time_limit_seconds': int(time_limit),
                'log_search': False,
                'vehicle_penalty_weight': vehicle_penalty_weight,
                'distance_weight': distance_weight
            }
            
            if solver_type == 'gurobi':
                solve_params['mip_gap'] = mip_gap
            
            logger.info("Starting optimization...")
            solution = solver.solve(**solve_params)
            
            if not solution:
                return {"status": "no_solution_found", "date": solved_date}
            
            # Check for errors in solution
            if solution.get('status') == 'error':
                return solution
            
            # Calculate execution time
            elapsed_time = time.time() - start_time
            logger.info(f"========== COMPLETED IN {elapsed_time:.2f}s ==========")
            
            # Enrich solution with customer information
            routes_enriched = self.problem_builder.enrich_solution_routes(
                solution, problem, payload, solved_date
            )
            
            result = {
                'date': solved_date,
                'summary': {
                    'num_vehicles_used': solution['num_vehicles_used'],
                    'total_distance_km': solution['total_distance'],
                    'total_load': solution['total_load'],
                    'average_saturation_pct': solution['average_saturation_pct']
                },
                'routes': routes_enriched,
                'objective_value': solution.get('objective_value'),
                'solver': solver_type,
                'execution_time_seconds': round(elapsed_time, 2)
            }
            
            return result
            
        finally:
            self._solver_running = False
            self._solver_lock.release()
    
    def _build_time_matrix(self, problem: Dict, distance_matrix, 
                          time_matrix_morning, time_matrix_afternoon, 
                          time_matrix_evening) -> list:
        """
        Build time matrix based on delivery time windows with traffic patterns.
        
        Args:
            problem: Problem data
            distance_matrix: Distance matrix
            time_matrix_morning: Morning travel times
            time_matrix_afternoon: Afternoon travel times
            time_matrix_evening: Evening travel times
            
        Returns:
            Time matrix scaled by 100
        """
        demands = problem['demands']
        depot = problem['depot']
        time_windows = problem['time_windows']
        n = len(distance_matrix)
        time_matrix_scaled = [[0] * n for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                if j == depot:
                    # Returning to depot: use afternoon baseline
                    travel_time_min = time_matrix_afternoon[i][j]
                    service_time_min = 0
                else:
                    # Delivering to customer: select time matrix based on delivery time window
                    tw_start_min = time_windows[j][0] if j < len(time_windows) else 480
                    
                    # Select appropriate traffic pattern based on time window
                    # Morning (before 12:00), Afternoon (12:00-18:00), Evening (after 18:00)
                    if tw_start_min < 720:  # Before 12:00
                        travel_time_min = time_matrix_morning[i][j]
                    elif tw_start_min < 1080:  # 12:00-18:00
                        travel_time_min = time_matrix_afternoon[i][j]
                    else:  # After 18:00
                        travel_time_min = time_matrix_evening[i][j]
                    
                    # Calculate service time: 10 min base + 2 min per unit
                    units = demands[j] if j < len(demands) else 0
                    service_time_min = 5 + (0.015 * units)
                
                # Total time = travel + service (scaled by 100 for solver precision)
                total_time_min = travel_time_min + service_time_min
                time_matrix_scaled[i][j] = int(total_time_min * 100)
        
        return time_matrix_scaled
