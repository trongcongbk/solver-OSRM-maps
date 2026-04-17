"""
Gurobi CVRPTW Solver Implementation

Capacitated Vehicle Routing Problem with Time Windows solver
using Gurobi MILP optimization.
"""

import logging
import time
import math
from typing import Dict, List, Optional, Tuple

try:
    import gurobipy as gp
    from gurobipy import GRB, quicksum
    GUROBI_AVAILABLE = True
except ImportError:
    GUROBI_AVAILABLE = False
    logging.warning("Gurobi not available. Install with: pip install gurobipy")

from ...utils.distance_calculator import haversine_distance, euclidean_distance
from ...utils.time_formatter import minutes_to_time, format_time_minutes

logger = logging.getLogger(__name__)


class GurobiSolverImpl:
    """Gurobi MILP implementation of CVRPTW solver."""
    
    def __init__(self, problem_data: Dict):
        """
        Initialize Gurobi solver with problem data.
        
        Args:
            problem_data: Dictionary containing problem definition
        """
        if not GUROBI_AVAILABLE:
            raise RuntimeError("Gurobi is not installed. Install with: pip install gurobipy")
        
        self.problem_data = problem_data
        self._validate_data()
        self._prepare_data()
    
    def _validate_data(self):
        """Validate input data."""
        required_keys = ['locations', 'demands', 'time_windows', 
                        'vehicle_capacities', 'num_vehicles']
        for key in required_keys:
            if key not in self.problem_data:
                raise ValueError(f"Missing required key: {key}")
        
        n_locations = len(self.problem_data['locations'])
        if len(self.problem_data['demands']) != n_locations:
            raise ValueError("Demands length must match locations length")
        if len(self.problem_data['time_windows']) != n_locations:
            raise ValueError("Time windows length must match locations length")
    
    def _prepare_data(self):
        """Prepare and compute derived data."""
        # Set defaults
        self.problem_data.setdefault('depot', 0)
        self.problem_data.setdefault('service_time', 0)
        
        # Compute distance matrix if not provided
        if 'distance_matrix' not in self.problem_data:
            self.problem_data['distance_matrix'] = self._compute_distance_matrix()
        
        # Compute time matrix if not provided
        if 'time_matrix' not in self.problem_data:
            self.problem_data['time_matrix'] = self._compute_time_matrix()
    
    def _compute_distance_matrix(self) -> List[List[float]]:
        """Compute distance matrix between all locations."""
        locations = self.problem_data['locations']
        n = len(locations)
        distance_matrix = [[0.0] * n for _ in range(n)]
        
        use_latlon = self.problem_data.get('coord_type', '').lower() == 'latlon'
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    distance_matrix[i][j] = 0.0
                else:
                    if use_latlon:
                        distance_matrix[i][j] = haversine_distance(
                            locations[i], locations[j]
                        )
                    else:
                        distance_matrix[i][j] = euclidean_distance(
                            locations[i], locations[j]
                        )
        
        return distance_matrix
    
    def _compute_time_matrix(self) -> List[List[int]]:
        """Compute time matrix based on distance and speed (scaled by 100)."""
        distance_matrix = self.problem_data['distance_matrix']
        speed = self.problem_data.get('vehicle_speed', 1.0)
        service_time = self.problem_data.get('service_time', 0)
        
        n = len(distance_matrix)
        time_matrix = [[0] * n for _ in range(n)]
        
        for i in range(n):
            for j in range(n):
                travel_time = (distance_matrix[i][j] / speed) if speed > 0 else 0
                total_time = travel_time + (service_time if j != self.problem_data['depot'] else 0)
                time_matrix[i][j] = int(total_time * 100)
        
        return time_matrix
    
    def solve(
        self,
        time_limit_seconds: int = 60,
        log_search: bool = False,
        vehicle_penalty_weight: float = 1000.0,
        distance_weight: float = 1.0,
        mip_gap: float = 0.01
    ) -> Optional[Dict]:
        """
        Solve CVRPTW problem using Gurobi MILP.
        
        Args:
            time_limit_seconds: Maximum time for solver
            log_search: Whether to log search progress
            vehicle_penalty_weight: Weight for minimizing number of vehicles
            distance_weight: Weight for distance minimization
            mip_gap: Relative MIP optimality gap
            
        Returns:
            Solution dictionary or None if no solution found
        """
        logger.info(
            f"Starting Gurobi solver: locations={len(self.problem_data['locations'])}, "
            f"vehicles={self.problem_data['num_vehicles']}"
        )
        
        n = len(self.problem_data['locations'])
        depot = self.problem_data['depot']
        vehicles = list(range(self.problem_data['num_vehicles']))
        customers = [i for i in range(n) if i != depot]
        
        distance_matrix = self.problem_data['distance_matrix']
        time_matrix = self.problem_data['time_matrix']
        demands = self.problem_data['demands']
        time_windows = self.problem_data['time_windows']
        capacities = self.problem_data['vehicle_capacities']
        
        # Validate time windows feasibility
        logger.info("Validating time windows feasibility...")
        for i in customers:
            travel_from_depot = time_matrix[depot][i]
            travel_to_depot = time_matrix[i][depot]
            earliest_arrival = time_windows[depot][0] * 100 + travel_from_depot
            latest_departure = time_windows[depot][1] * 100 - travel_to_depot
            
            if earliest_arrival > time_windows[i][1] * 100:
                logger.error(
                    f"Customer {i}: Cannot reach within time window. "
                    f"Earliest arrival: {earliest_arrival/100:.2f}, "
                    f"Latest allowed: {time_windows[i][1]}"
                )
            if latest_departure < time_windows[i][0] * 100:
                logger.error(
                    f"Customer {i}: Cannot return to depot in time. "
                    f"Latest departure: {latest_departure/100:.2f}, "
                    f"Earliest required: {time_windows[i][0]}"
                )
        
        try:
            # Create model
            model = gp.Model("CVRPTW")
            
            # Set parameters
            model.Params.TimeLimit = time_limit_seconds
            model.Params.OutputFlag = 1 if log_search else 0
            model.Params.MIPGap = mip_gap
            
            logger.info(
                f"Building model with n={n}, vehicles={len(vehicles)}, "
                f"customers={len(customers)}"
            )
            
            # Decision variables
            x = {}  # x[i,j,k] = 1 if vehicle k travels from i to j
            for k in vehicles:
                for i in range(n):
                    for j in range(n):
                        if i != j:
                            x[i, j, k] = model.addVar(
                                vtype=GRB.BINARY,
                                name=f'x_{i}_{j}_{k}'
                            )
            
            # u[i,k] = arrival time at location i for vehicle k (scaled by 100)
            u = {}
            max_time_bound = (
                max(tw[1] for tw in time_windows) * 100 + 
                max(max(row) for row in time_matrix)
            )
            for k in vehicles:
                for i in range(n):
                    u[i, k] = model.addVar(
                        vtype=GRB.CONTINUOUS,
                        lb=0,
                        ub=max_time_bound,
                        name=f'u_{i}_{k}'
                    )
            
            # y[k] = 1 if vehicle k is used
            y = {}
            for k in vehicles:
                y[k] = model.addVar(vtype=GRB.BINARY, name=f'y_{k}')
            
            # z[i,k] = 1 if customer i is visited by vehicle k
            z = {}
            for k in vehicles:
                for i in customers:
                    z[i, k] = model.addVar(vtype=GRB.BINARY, name=f'z_{i}_{k}')
            
            # w[i] = 1 if customer i is NOT served (dropped)
            w = {}
            for i in customers:
                w[i] = model.addVar(vtype=GRB.BINARY, name=f'w_{i}')
            
            model.update()
            
            # Objective: minimize total distance + vehicle penalty + unserved penalty
            unserved_penalty = 10000000000.0  # Same as OR-Tools
            obj = (
                gp.quicksum(
                    distance_matrix[i][j] * x[i, j, k] * distance_weight
                    for k in vehicles
                    for i in range(n)
                    for j in range(n)
                    if i != j
                ) +
                gp.quicksum(y[k] * vehicle_penalty_weight for k in vehicles) +
                gp.quicksum(w[i] * unserved_penalty for i in customers)
            )
            
            model.setObjective(obj, GRB.MINIMIZE)
            
            logger.info(
                f"Objective weights: vehicle_penalty={vehicle_penalty_weight}, "
                f"distance_weight={distance_weight}, "
                f"unserved_penalty={unserved_penalty}, mip_gap={mip_gap}"
            )
            
            # Constraints
            
            # 1. Each customer is either visited exactly once OR marked as unserved
            for i in customers:
                model.addConstr(
                    gp.quicksum(z[i, k] for k in vehicles) + w[i] == 1,
                    name=f'visit_{i}'
                )
            
            # 1b. Link z variables to actual visits
            for k in vehicles:
                for i in customers:
                    model.addConstr(
                        z[i, k] == gp.quicksum(x[j, i, k] for j in range(n) if j != i),
                        name=f'visit_link_{i}_{k}'
                    )
            
            # 2. Flow conservation: if vehicle enters a node, it must leave
            for k in vehicles:
                for i in customers:
                    model.addConstr(
                        gp.quicksum(x[i, j, k] for j in range(n) if j != i) ==
                        gp.quicksum(x[j, i, k] for j in range(n) if j != i),
                        name=f'flow_{i}_{k}'
                    )
            
            # 3. Each vehicle starts and ends at depot (if used)
            for k in vehicles:
                model.addConstr(
                    gp.quicksum(x[depot, j, k] for j in customers) == y[k],
                    name=f'start_{k}'
                )
                model.addConstr(
                    gp.quicksum(x[i, depot, k] for i in customers) == y[k],
                    name=f'end_{k}'
                )
            
            # 4. Vehicle usage indicator
            for k in vehicles:
                model.addConstr(
                    gp.quicksum(z[i, k] for i in customers) <= len(customers) * y[k],
                    name=f'vehicle_used_{k}'
                )
            
            # 5. Capacity constraints
            for k in vehicles:
                model.addConstr(
                    gp.quicksum(demands[i] * z[i, k] for i in customers) <= capacities[k],
                    name=f'capacity_{k}'
                )
            
            # 6. Time window constraints
            max_tw = max(tw[1] for tw in time_windows) * 100
            max_travel = max(max(row) for row in time_matrix) if time_matrix else 0
            M = int(max_tw - min(tw[0] for tw in time_windows) * 100 + max_travel + 1000)
            logger.info(f"Using Big-M={M} (max_tw={max_tw}, max_travel={max_travel})")
            
            # Time window enforcement for customers
            for k in vehicles:
                for i in customers:
                    model.addConstr(
                        u[i, k] >= time_windows[i][0] * 100 - M * (1 - z[i, k]),
                        name=f'tw_lower_{i}_{k}'
                    )
                    model.addConstr(
                        u[i, k] <= time_windows[i][1] * 100 + M * (1 - z[i, k]),
                        name=f'tw_upper_{i}_{k}'
                    )
            
            # Time window enforcement for depot
            for k in vehicles:
                model.addConstr(
                    u[depot, k] >= time_windows[depot][0] * 100 - M * (1 - y[k]),
                    name=f'tw_lower_depot_{k}'
                )
                model.addConstr(
                    u[depot, k] <= time_windows[depot][1] * 100 + M * (1 - y[k]),
                    name=f'tw_upper_depot_{k}'
                )
            
            # Add callback for intermediate statistics
            last_stats_time = [0.0]
            start_time = [time.time()]
            
            def stats_callback(model, where):
                """Gurobi callback to report intermediate statistics every 5 seconds."""
                if where == GRB.Callback.MIPSOL:
                    current_time = time.time()
                    if current_time - last_stats_time[0] >= 5.0:
                        last_stats_time[0] = current_time
                        elapsed = current_time - start_time[0]
                        
                        try:
                            vehicles_used = sum(
                                1 for k in vehicles 
                                if model.cbGetSolution(y[k]) > 0.5
                            )
                            
                            customers_served = len(set(
                                i for i in customers
                                for k in vehicles
                                if model.cbGetSolution(z[i, k]) > 0.5
                            ))
                            
                            total_distance = sum(
                                distance_matrix[i][j] / 100.0
                                for i in range(n)
                                for j in range(n)
                                if i != j
                                for k in vehicles
                                if model.cbGetSolution(x[i, j, k]) > 0.5
                            )
                            
                            total_load = sum(
                                demands[i]
                                for i in customers
                                for k in vehicles
                                if model.cbGetSolution(z[i, k]) > 0.5
                            )
                            
                            total_trips = sum(
                                1 for k in vehicles
                                for i in customers
                                if model.cbGetSolution(x[i, depot, k]) > 0.5
                            )
                            
                            avg_distance = (
                                total_distance / vehicles_used 
                                if vehicles_used > 0 
                                else 0.0
                            )
                            total_capacity = sum(capacities)
                            avg_saturation = (
                                (total_load / total_capacity * 100) 
                                if total_capacity > 0 
                                else 0.0
                            )
                            
                            logger.info(
                                f"[{elapsed:.0f}s] Intermediate: "
                                f"Vehicles={vehicles_used}/{len(vehicles)}, "
                                f"Trips={total_trips}, "
                                f"Customers={customers_served}/{len(customers)}, "
                                f"Avg Distance={avg_distance:.1f}km, "
                                f"Avg Saturation={avg_saturation:.1f}%"
                            )
                        except Exception:
                            pass
            
            logger.info("Starting Gurobi optimization...")
            model.optimize(stats_callback)
            
            # Check solution status
            if model.Status == GRB.OPTIMAL or model.Status == GRB.TIME_LIMIT:
                if model.SolCount > 0:
                    logger.info(
                        f"Solution found! Status: {model.Status}, "
                        f"Objective: {model.ObjVal:.2f}"
                    )
                    solution = self._extract_solution(
                        model, x, u, y, z, w, vehicles, n, depot
                    )
                    
                    if solution:
                        self._log_solution_summary(solution, capacities)
                    
                    return solution
                else:
                    logger.warning("No feasible solution found within time limit")
                    return None
            elif model.Status == GRB.INFEASIBLE:
                logger.error("Model is infeasible")
                self._log_infeasibility_details(model, n, depot, vehicles)
                return None
            else:
                logger.warning(f"Optimization ended with status {model.Status}")
                return None
                
        except gp.GurobiError as e:
            error_msg = str(e)
            logger.error(f"Gurobi error: {error_msg}")
            return {
                'status': 'error',
                'error_type': 'gurobi_error',
                'message': error_msg
            }
        except Exception as e:
            logger.exception(f"Error during optimization: {e}")
            return None
    
    def _log_infeasibility_details(self, model, n, depot, vehicles):
        """Log details about model infeasibility."""
        time_windows = self.problem_data['time_windows']
        demands = self.problem_data['demands']
        capacities = self.problem_data['vehicle_capacities']
        time_matrix = self.problem_data['time_matrix']
        
        logger.error(f"Problem details: n={n}, depot={depot}, vehicles={len(vehicles)}")
        logger.error(f"Time windows (scaled): {[(tw[0]*100, tw[1]*100) for tw in time_windows]}")
        logger.error(f"Demands: {demands}")
        logger.error(f"Capacities: {capacities}")
        
        # Compute IIS
        try:
            model.computeIIS()
            iis_file = "gurobi_infeasible.ilp"
            model.write(iis_file)
            logger.error(f"IIS written to {iis_file}")
            
            logger.error("Constraints in IIS:")
            for c in model.getConstrs():
                if c.IISConstr:
                    logger.error(f"  - {c.ConstrName}")
        except Exception as e:
            logger.error(f"Could not compute IIS: {e}")
    
    def _log_solution_summary(self, solution, capacities):
        """Log solution summary statistics."""
        total_capacity = sum(capacities)
        logger.info(
            f"  Vehicles used: {solution['num_vehicles_used']}/"
            f"{solution['total_vehicles_available']}"
        )
        logger.info(f"  Total trips: {solution['total_trips']}")
        logger.info(f"  Average trips per vehicle: {solution['avg_trips_per_vehicle']}")
        logger.info(f"  Total distance: {solution['total_distance_formatted']}")
        logger.info(
            f"  Average distance per vehicle: "
            f"{solution['avg_distance_per_vehicle_formatted']}"
        )
        logger.info(
            f"  Total load: {solution['total_load']}/{total_capacity} units "
            f"({solution['average_saturation_pct']:.1f}% saturation)"
        )
        logger.info(
            f"  Average travel time per vehicle: "
            f"{int(solution['avg_travel_time_per_vehicle_minutes'])} min "
            f"(excluding service)"
        )
        logger.info(
            f"  Average total time per vehicle: "
            f"{int(solution['avg_duration_per_vehicle_minutes'])} min "
            f"(including service)"
        )
        
        customers_served = solution.get('customers_served', 0)
        customers_total = solution.get('customers_total', 0)
        logger.info(f"  Customers served: {customers_served}/{customers_total}")
        
        dropped = solution.get('dropped_customers', [])
        if dropped:
            logger.warning(f"WARNING: {len(dropped)} customers could not be served:")
            for dc in dropped:
                logger.warning(
                    f"  - Location {dc['location']}: demand={dc['demand']}, "
                    f"TW={dc['time_window_formatted']}"
                )
    
    def _extract_solution(
        self, 
        model, 
        x, 
        u, 
        y, 
        z, 
        w, 
        vehicles, 
        n, 
        depot
    ) -> Dict:
        """Extract solution from Gurobi model."""
        routes = []
        total_distance = 0.0
        total_load = 0
        vehicles_used = 0
        dropped_customers = []
        
        distance_matrix = self.problem_data['distance_matrix']
        time_matrix = self.problem_data.get('time_matrix', distance_matrix)
        demands = self.problem_data['demands']
        time_windows = self.problem_data['time_windows']
        capacities = self.problem_data['vehicle_capacities']
        customers = [idx for idx in range(n) if idx != depot]
        
        # Debug logging
        logger.info("Active arcs in solution:")
        for k in vehicles:
            for i in range(n):
                for j in range(n):
                    if i != j and (i, j, k) in x and x[i, j, k].X > 0.5:
                        logger.info(f"  Vehicle {k}: {i} -> {j} (x={x[i, j, k].X:.2f})")
        
        logger.info("Customer assignments (z variables):")
        for k in vehicles:
            for i in customers:
                if (i, k) in z and z[i, k].X > 0.5:
                    logger.info(
                        f"  Customer {i} assigned to vehicle {k} (z={z[i, k].X:.2f})"
                    )
        
        # Check for dropped customers
        logger.info("Checking for unserved customers (w variables):")
        for i in customers:
            if i in w and w[i].X > 0.5:
                dropped_customers.append({
                    'location': i,
                    'demand': demands[i],
                    'time_window': time_windows[i],
                    'time_window_formatted': (
                        f"{minutes_to_time(time_windows[i][0])} - "
                        f"{minutes_to_time(time_windows[i][1])}"
                    )
                })
                logger.warning(f"  Customer {i} was NOT served (dropped)")
        
        if dropped_customers:
            logger.warning(
                f"Total customers dropped: {len(dropped_customers)} out of {len(customers)}"
            )
        else:
            logger.info("All customers were served successfully!")
        
        # Extract routes for each vehicle
        for k in vehicles:
            if y[k].X > 0.5:
                vehicles_used += 1
                
                # Reconstruct route
                route_indices = self._reconstruct_route(x, n, depot, k)
                
                # Calculate route statistics
                route_distance = sum(
                    distance_matrix[route_indices[i]][route_indices[i+1]]
                    for i in range(len(route_indices)-1)
                )
                route_load = sum(demands[idx] for idx in route_indices if idx != depot)
                
                # Calculate times
                travel_time_minutes = sum(
                    time_matrix[route_indices[i]][route_indices[i+1]]
                    for i in range(len(route_indices)-1)
                ) / 100.0
                
                num_customers = len([idx for idx in route_indices if idx != depot])
                service_time_minutes = sum(
                    5 + (0.015 * demands[idx])
                    for idx in route_indices
                    if idx != depot
                )
                
                route_duration_minutes = travel_time_minutes + service_time_minutes
                
                # Build detailed route with time info
                route_details = self._build_route_details(
                    route_indices, depot, distance_matrix, time_matrix
                )
                
                saturation_pct = (
                    round((route_load / capacities[k]) * 100, 1)
                    if capacities[k] > 0
                    else 0
                )
                
                routes.append({
                    'vehicle_id': k,
                    'route': route_details,
                    'distance': round(route_distance, 2),
                    'distance_km': round(route_distance, 2),
                    'distance_formatted': f"{route_distance:.2f} km",
                    'load': route_load,
                    'load_units': route_load,
                    'load_formatted': f"{route_load} units",
                    'capacity': capacities[k],
                    'saturation_pct': saturation_pct,
                    'duration_minutes': round(route_duration_minutes, 2),
                    'duration_formatted': format_time_minutes(route_duration_minutes),
                    'duration_hours': round(route_duration_minutes / 60.0, 2),
                    'travel_time_minutes': round(travel_time_minutes, 2),
                    'travel_time_hours': round(travel_time_minutes / 60.0, 2),
                    'travel_time_formatted': format_time_minutes(travel_time_minutes),
                    'service_time_minutes': round(service_time_minutes, 2),
                    'service_time_hours': round(service_time_minutes / 60.0, 2),
                    'service_time_formatted': format_time_minutes(service_time_minutes),
                    'num_customers': num_customers
                })
                
                total_distance += route_distance
                total_load += route_load
        
        # Build solution summary
        return self._build_solution_summary(
            routes, vehicles_used, total_distance, total_load,
            dropped_customers, model.ObjVal, customers, n, depot
        )
    
    def _reconstruct_route(self, x, n, depot, k):
        """Reconstruct route for vehicle k from solution variables."""
        route_indices = [depot]
        current = depot
        visited = {depot}
        
        while True:
            next_node = None
            for j in range(n):
                if j != current and j not in visited:
                    if (current, j, k) in x and x[current, j, k].X > 0.5:
                        next_node = j
                        break
            
            if next_node is None:
                # Check if returning to depot
                if (current, depot, k) in x and x[current, depot, k].X > 0.5:
                    route_indices.append(depot)
                break
            
            route_indices.append(next_node)
            visited.add(next_node)
            current = next_node
        
        return route_indices
    
    def _build_route_details(
        self, 
        route_indices, 
        depot, 
        distance_matrix, 
        time_matrix
    ):
        """Build detailed route information with times and loads."""
        route_details = []
        demands = self.problem_data['demands']
        time_windows = self.problem_data['time_windows']
        
        current_time = time_windows[depot][0] * 100  # Start at depot opening time
        
        for i, idx in enumerate(route_indices):
            # Calculate segment distance
            segment_distance = 0.0
            if i > 0:
                prev_idx = route_indices[i-1]
                segment_distance = distance_matrix[prev_idx][idx]
                travel_time = time_matrix[prev_idx][idx]
                current_time += travel_time
                
                # Add service time at previous location
                if prev_idx != depot:
                    service_at_prev = 5 + (0.015 * demands[prev_idx])
                    current_time += service_at_prev * 100
            
            arrival_time_minutes = current_time / 100.0
            
            # Respect time window lower bound
            tw_lower = time_windows[idx][0]
            if arrival_time_minutes < tw_lower:
                arrival_time_minutes = tw_lower
                current_time = tw_lower * 100
            
            # Calculate loads
            load_before = sum(
                demands[route_indices[j]]
                for j in range(i, len(route_indices))
                if route_indices[j] != depot
            )
            load_after = sum(
                demands[route_indices[j]]
                for j in range(i+1, len(route_indices))
                if route_indices[j] != depot
            )
            
            tw = time_windows[idx]
            time_window_formatted = (
                f"{minutes_to_time(tw[0])} - {minutes_to_time(tw[1])}"
            )
            
            route_details.append({
                'location': idx,
                'arrival_time': round(arrival_time_minutes, 2),
                'time_formatted': minutes_to_time(arrival_time_minutes),
                'time_window': time_windows[idx],
                'time_window_formatted': time_window_formatted,
                'demand': demands[idx],
                'load_before': load_before,
                'load_after': load_after,
                'segment_distance': round(segment_distance, 2),
                'segment_distance_formatted': f"{segment_distance:.2f} km"
            })
        
        return route_details
    
    def _build_solution_summary(
        self,
        routes,
        vehicles_used,
        total_distance,
        total_load,
        dropped_customers,
        objective_value,
        customers,
        n,
        depot
    ):
        """Build comprehensive solution summary."""
        capacities = self.problem_data['vehicle_capacities']
        
        total_capacity = sum(r['capacity'] for r in routes) if routes else 0
        avg_saturation = (
            (total_load / total_capacity * 100)
            if total_capacity > 0
            else 0
        )
        
        total_trips = len(routes)
        avg_trips_per_vehicle = (
            total_trips / vehicles_used
            if vehicles_used > 0
            else 0
        )
        avg_distance_per_vehicle = (
            total_distance / vehicles_used
            if vehicles_used > 0
            else 0
        )
        
        total_duration_minutes = sum(r['duration_minutes'] for r in routes)
        total_travel_time_minutes = sum(r['travel_time_minutes'] for r in routes)
        total_service_time_minutes = sum(r['service_time_minutes'] for r in routes)
        
        avg_duration_per_vehicle_minutes = (
            total_duration_minutes / vehicles_used
            if vehicles_used > 0
            else 0
        )
        avg_travel_time_per_vehicle_minutes = (
            total_travel_time_minutes / vehicles_used
            if vehicles_used > 0
            else 0
        )
        avg_service_time_per_vehicle_minutes = (
            total_service_time_minutes / vehicles_used
            if vehicles_used > 0
            else 0
        )
        
        # Count customers served
        served_customers = set()
        for route in routes:
            for stop in route['route']:
                if stop['location'] != depot:
                    served_customers.add(stop['location'])
        
        customers_served = len(served_customers)
        customers_total = len(customers)
        
        return {
            'status': 'success',
            'num_vehicles_used': vehicles_used,
            'total_vehicles_available': self.problem_data['num_vehicles'],
            'total_trips': total_trips,
            'avg_trips_per_vehicle': round(avg_trips_per_vehicle, 2),
            'total_distance': round(total_distance, 2),
            'total_distance_km': round(total_distance, 2),
            'total_distance_formatted': f"{total_distance:.2f} km",
            'avg_distance_per_vehicle_km': round(avg_distance_per_vehicle, 2),
            'avg_distance_per_vehicle_formatted': f"{avg_distance_per_vehicle:.2f} km",
            'total_load': total_load,
            'average_saturation_pct': round(avg_saturation, 1),
            'total_duration_minutes': round(total_duration_minutes, 1),
            'total_duration_hours': round(total_duration_minutes / 60.0, 2),
            'total_duration_formatted': format_time_minutes(total_duration_minutes),
            'total_travel_time_minutes': round(total_travel_time_minutes, 1),
            'total_travel_time_hours': round(total_travel_time_minutes / 60.0, 2),
            'total_travel_time_formatted': format_time_minutes(total_travel_time_minutes),
            'total_service_time_minutes': round(total_service_time_minutes, 1),
            'total_service_time_hours': round(total_service_time_minutes / 60.0, 2),
            'total_service_time_formatted': format_time_minutes(total_service_time_minutes),
            'avg_duration_per_vehicle_minutes': round(avg_duration_per_vehicle_minutes, 1),
            'avg_duration_per_vehicle_hours': round(avg_duration_per_vehicle_minutes / 60.0, 2),
            'avg_duration_per_vehicle_formatted': format_time_minutes(avg_duration_per_vehicle_minutes),
            'avg_travel_time_per_vehicle_minutes': round(avg_travel_time_per_vehicle_minutes, 1),
            'avg_travel_time_per_vehicle_hours': round(avg_travel_time_per_vehicle_minutes / 60.0, 2),
            'avg_travel_time_per_vehicle_formatted': format_time_minutes(avg_travel_time_per_vehicle_minutes),
            'avg_service_time_per_vehicle_minutes': round(avg_service_time_per_vehicle_minutes, 1),
            'avg_service_time_per_vehicle_hours': round(avg_service_time_per_vehicle_minutes / 60.0, 2),
            'avg_service_time_per_vehicle_formatted': format_time_minutes(avg_service_time_per_vehicle_minutes),
            'routes': routes,
            'customers_served': customers_served,
            'customers_total': customers_total,
            'dropped_customers': dropped_customers,
            'objective_value': round(objective_value, 2),
            'solver': 'gurobi'
        }
