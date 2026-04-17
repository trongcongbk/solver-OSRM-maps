"""
OR-Tools CVRPTW Solver Implementation

Capacitated Vehicle Routing Problem with Time Windows solver
using Google OR-Tools constraint programming.
"""

import math
import time
import threading
import logging
from typing import List, Dict, Optional

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

from ...utils.distance_calculator import haversine_distance
from ...utils.time_formatter import minutes_to_time, round_to_5_minutes

logger = logging.getLogger(__name__)


class ORToolsSolverImpl:
    """OR-Tools implementation of CVRPTW solver."""
    
    def __init__(self, problem_data: Dict):
        """
        Initialize OR-Tools solver with problem data.
        
        Args:
            problem_data: Dictionary containing problem definition
        """
        self.problem_data = problem_data
        self._validate_data()
        self._prepare_data()
        self._manager = None
        self._routing = None
    
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
        self.problem_data.setdefault('vehicle_speed', 1.0)
        
        # Compute distance matrix (only if not provided)
        if 'distance_matrix' not in self.problem_data:
            self.problem_data['distance_matrix'] = self._compute_distance_matrix()
        
        # Compute time matrix (only if not provided)
        if 'time_matrix' not in self.problem_data:
            self.problem_data['time_matrix'] = self._compute_time_matrix()
    
    def _compute_distance_matrix(self) -> List[List[float]]:
        """Compute distance matrix between all locations."""
        locations = self.problem_data['locations']
        n = len(locations)
        distance_matrix = [[0.0] * n for _ in range(n)]
        
        use_latlon = (
            self.problem_data.get('coord_type', '').lower() == 'latlon' 
            if isinstance(self.problem_data.get('coord_type', ''), str) 
            else bool(self.problem_data.get('use_haversine', False))
        )
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    distance_matrix[i][j] = 0.0
                    continue
                
                loc1 = locations[i]
                loc2 = locations[j]
                
                if use_latlon:
                    distance = haversine_distance(loc1, loc2)
                else:
                    # Euclidean distance
                    try:
                        distance = math.sqrt((loc2[0] - loc1[0])**2 + (loc2[1] - loc1[1])**2)
                    except Exception:
                        distance = 0.0
                
                distance_matrix[i][j] = distance
        
        return distance_matrix
    
    def _compute_time_matrix(self) -> List[List[int]]:
        """
        Compute time matrix (travel time + service time).
        
        Service time is calculated dynamically as:
        - Depot: 0 minutes
        - Customer: 10 minutes (fixed) + 2 minutes per unit
        """
        distance_matrix = self.problem_data['distance_matrix']
        speed = self.problem_data['vehicle_speed']
        demands = self.problem_data['demands']
        depot = self.problem_data['depot']
        n = len(distance_matrix)
        
        # Convert to integer time units (scaled by 100 for precision)
        time_matrix = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                travel_time = int((distance_matrix[i][j] / speed) * 100)
                
                # Dynamic service time: 10 min + 2 min per unit
                if j != depot:
                    units = demands[j] if j < len(demands) else 0
                    service_time_min = 5 + (0.015 * units)
                    service_time_scaled = int(service_time_min * 100)
                else:
                    service_time_scaled = 0
                
                time_matrix[i][j] = travel_time + service_time_scaled
        
        return time_matrix
    
    def _create_distance_callback(self, manager):
        """Create callback to get distance between nodes."""
        distance_matrix = self.problem_data['distance_matrix']
        
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(distance_matrix[from_node][to_node] * 100)  # Scale for precision
        
        return distance_callback
    
    def _create_demand_callback(self, manager):
        """Create callback to get demand at each node."""
        demands = self.problem_data['demands']
        
        def demand_callback(from_index):
            from_node = manager.IndexToNode(from_index)
            return demands[from_node]
        
        return demand_callback

    def _create_time_callback(self, manager):
        """Create callback to get total time (Travel Time OSRM + Service Time)."""
        time_matrix = self.problem_data['time_matrix']
        demands = self.problem_data['demands']
        depot = self.problem_data['depot']

        def time_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)

            # Thời gian lái xe thuần túy từ OSRM
            travel_time = time_matrix[from_node][to_node]

            # CỘNG THÊM thời gian phục vụ tại điểm xuất phát
            if from_node != depot:
                units = demands[from_node] if from_node < len(demands) else 0
                service_time_scaled = int((5 + 0.015 * units) * 100)
            else:
                service_time_scaled = 0

            return travel_time + service_time_scaled

        return time_callback
    
    def solve(
        self,
        time_limit_seconds: int = 30,
        log_search: bool = False,
        vehicle_penalty_weight: float = 100000.0,
        distance_weight: float = 1.0,
        **kwargs
    ) -> Optional[Dict]:
        """
        Solve CVRPTW problem using OR-Tools.
        
        Args:
            time_limit_seconds: Maximum time for solver
            log_search: Whether to log search progress
            vehicle_penalty_weight: Weight for minimizing number of vehicles
            distance_weight: Weight for distance minimization
            **kwargs: Additional parameters (ignored)
            
        Returns:
            Solution dictionary or None if no solution found
        """
        logger.info(
            f"Preparing OR-Tools model: locations={len(self.problem_data['distance_matrix'])}, "
            f"vehicles={self.problem_data['num_vehicles']}, depot={self.problem_data.get('depot', 0)}"
        )
        
        # Create routing index manager
        manager = pywrapcp.RoutingIndexManager(
            len(self.problem_data['distance_matrix']),
            self.problem_data['num_vehicles'],
            self.problem_data['depot']
        )
        
        # Create routing model
        routing = pywrapcp.RoutingModel(manager)
        
        # Register distance callback
        distance_callback = self._create_distance_callback(manager)
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        
        # Apply distance weight to arc costs
        if distance_weight != 1.0:
            def weighted_distance_callback(from_index, to_index):
                return int(distance_callback(from_index, to_index) * distance_weight)
            weighted_transit_callback_index = routing.RegisterTransitCallback(weighted_distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(weighted_transit_callback_index)
        else:
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Add capacity constraint
        demand_callback = self._create_demand_callback(manager)
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,  # null capacity slack
            self.problem_data['vehicle_capacities'],
            True,  # start cumul to zero
            'Capacity'
        )
        
        # Add time window constraint
        time_callback = self._create_time_callback(manager)
        time_callback_index = routing.RegisterTransitCallback(time_callback)
        
        # Get maximum time window end (scaled)
        max_time = int(max(tw[1] for tw in self.problem_data['time_windows']) * 100)
        
        routing.AddDimension(
            time_callback_index,
            max_time,  # allow waiting time
            max_time,  # maximum time per vehicle
            False,  # don't force start cumul to zero
            'Time'
        )
        
        # Add time window constraints for each location
        time_dimension = routing.GetDimensionOrDie('Time')
        for location_idx, time_window in enumerate(self.problem_data['time_windows']):
            if location_idx == self.problem_data['depot']:
                continue
            index = manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(
                int(time_window[0] * 100),
                int(time_window[1] * 100)
            )
        
        # Add time window constraints for depot
        depot_idx = self.problem_data['depot']
        for vehicle_id in range(self.problem_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(
                int(self.problem_data['time_windows'][depot_idx][0] * 100),
                int(self.problem_data['time_windows'][depot_idx][1] * 100)
            )
        
        # Set up objective: minimize number of vehicles and total distance
        routing.SetFixedCostOfAllVehicles(int(vehicle_penalty_weight))
        
        logger.info(
            f"Objective weights: vehicle_penalty={vehicle_penalty_weight}, "
            f"distance_weight={distance_weight}"
        )
        
        # Allow dropping nodes with very high penalty
        penalty = 10000000000  # Extremely high to force visiting all nodes
        for node in range(1, len(self.problem_data['distance_matrix'])):
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)
        
        # Set search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = time_limit_seconds
        search_parameters.log_search = log_search
        
        # Create monitoring thread for progress reporting
        solving = [True]
        start_time = time.time()
        num_customers = len(self.problem_data['demands']) - 1
        num_vehicles = self.problem_data['num_vehicles']
        
        def monitor_progress():
            """Monitor thread to report progress every 5 seconds."""
            last_report = start_time
            while solving[0]:
                time.sleep(1)
                current_time = time.time()
                if current_time - last_report >= 5.0:
                    last_report = current_time
                    elapsed = current_time - start_time
                    logger.info(
                        f"[{elapsed:.0f}s] OR-Tools optimizing: "
                        f"{num_customers} customers, {num_vehicles} vehicles available "
                        f"(intermediate stats not available with OR-Tools)"
                    )
        
        # Start monitoring thread
        monitor_thread = threading.Thread(target=monitor_progress, daemon=True)
        monitor_thread.start()
        
        # Solve the problem
        logger.info(
            f"Starting OR-Tools solver (time limit: {time_limit_seconds}s, "
            f"locations: {len(self.problem_data['locations'])}, "
            f"vehicles: {self.problem_data['num_vehicles']})..."
        )
        solution = routing.SolveWithParameters(search_parameters)
        
        # Stop monitoring
        solving[0] = False
        monitor_thread.join(timeout=1.0)
        
        if solution:
            obj_value = solution.ObjectiveValue()
            logger.info(f"✓ Solution found - Objective: {obj_value:,.0f}")
            return self._extract_solution(manager, routing, solution)
        else:
            logger.warning("No solution found")
            return None
    
    def _extract_solution(self, manager, routing, solution) -> Dict:
        """Extract solution details from OR-Tools solution."""
        total_distance = 0
        total_load = 0
        routes = []
        num_vehicles_used = 0

        # Thêm dòng này nếu chưa có để định nghĩa depot_idx
        depot_idx = self.problem_data.get('depot', 0)

        capacity_dimension = routing.GetDimensionOrDie('Capacity')
        time_dimension = routing.GetDimensionOrDie('Time')
        
        for vehicle_id in range(self.problem_data['num_vehicles']):
            index = routing.Start(vehicle_id)
            route = []
            route_distance = 0
            is_first_arc = True
            segment_distances = []
            
            # First pass: collect all stops to calculate total load
            temp_route = []
            temp_index = index
            while not routing.IsEnd(temp_index):
                node_index = manager.IndexToNode(temp_index)
                demand = (
                    self.problem_data['demands'][node_index] 
                    if node_index < len(self.problem_data['demands']) 
                    else 0
                )
                temp_route.append((node_index, demand))
                temp_index = solution.Value(routing.NextVar(temp_index))
            
            # Calculate total load
            total_route_load = sum(d for _, d in temp_route)
            
            # Second pass: build route with delivery model
            current_load = total_route_load
            
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                time_var = time_dimension.CumulVar(index)
                time_minutes = solution.Value(time_var) / 100.0
                
                demand = (
                    self.problem_data['demands'][node_index] 
                    if node_index < len(self.problem_data['demands']) 
                    else 0
                )
                
                time_window = self.problem_data['time_windows'][node_index]
                time_window_start = time_window[0]
                time_window_end = time_window[1]
                
                # Delivery model: load decreases after delivery
                if node_index == self.problem_data['depot']:
                    load_before = 0
                    load_after = total_route_load
                else:
                    load_before = current_load
                    load_after = current_load - demand
                    current_load = load_after
                
                time_rounded = round_to_5_minutes(time_minutes)
                
                route.append({
                    'location': node_index,
                    'load_before': load_before,
                    'load_after': load_after,
                    'time': time_rounded,
                    'time_formatted': minutes_to_time(time_minutes),
                    'time_window': time_window,
                    'time_window_formatted': (
                        f"{minutes_to_time(time_window_start)} - "
                        f"{minutes_to_time(time_window_end)}"
                    ),
                    'segment_distance': 0.0,
                    'segment_distance_formatted': "0.00 km"
                })
                
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                arc_cost = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
                
                # Subtract fixed cost from first arc
                if is_first_arc:
                    fixed_cost_scaled = int(routing.GetFixedCostOfVehicle(vehicle_id))
                    arc_cost -= fixed_cost_scaled
                    is_first_arc = False
                
                route_distance += arc_cost
                segment_distances.append(arc_cost)
            
            # Add final depot stop
            node_index = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            time_minutes = solution.Value(time_var) / 100.0
            time_rounded = round_to_5_minutes(time_minutes)
            time_window = self.problem_data['time_windows'][node_index]
            
            route.append({
                'location': node_index,
                'load_before': current_load,
                'load_after': 0,
                'time': time_rounded,
                'time_formatted': minutes_to_time(time_minutes),
                'time_window': time_window,
                'time_window_formatted': (
                    f"{minutes_to_time(time_window[0])} - "
                    f"{minutes_to_time(time_window[1])}"
                ),
                'segment_distance': 0.0,
                'segment_distance_formatted': "0.00 km"
            })
            
            # Update segment distances
            # =================================================================
            # ĐỒNG BỘ HÓA TOÀN DIỆN: KHOẢNG CÁCH, JIT VÀ TIMELINE (FIXED)
            # =================================================================

            # 1. Khởi tạo Service Time, Distance và Actual Travel Time cho từng điểm
            for i in range(len(route)):
                stop = route[i]

                # Tính khoảng cách từ điểm trước đó
                if i > 0 and i - 1 < len(segment_distances):
                    stop['segment_distance'] = round(segment_distances[i - 1] / 100.0, 2)
                    stop['segment_distance_formatted'] = f"{stop['segment_distance']:.2f} km"
                else:
                    stop['segment_distance'] = 0.0
                    stop['segment_distance_formatted'] = "0.00 km"

                # Tính Service Time (Thời gian bốc dỡ)
                if stop['location'] != depot_idx:
                    units = stop['load_before'] - stop['load_after']
                    stop['service_time'] = round(5 + (0.015 * units))
                else:
                    stop['service_time'] = 0

                # Tính Travel Time thực tế từ OSRM (Scale down 100)
                # Tính sớm ở đây để Bước 2 (JIT) không bị KeyError
                if i < len(route) - 1:
                    from_loc = route[i]['location']
                    to_loc = route[i + 1]['location']
                    raw_transit = self.problem_data['time_matrix'][from_loc][to_loc] / 100.0
                    stop['actual_travel_time'] = round(raw_transit)

            # 2. Tính JIT Departure (Tối ưu để KHÔNG còn khoảng trắng ở điểm đầu)
            if len(route) > 2:
                # Giờ OR-Tools sắp xếp (Arrival dự kiến)
                arrival_first_cust_raw = route[1]['time']
                # Cửa sổ thời gian mở cửa của khách hàng đầu tiên
                tw_start_first = route[1]['time_window'][0]

                # Ép xe đến đúng lúc khách mở cửa hoặc lúc OR-Tools tính (tùy cái nào muộn hơn)
                # Điều này giúp xóa bỏ khoảng Wait Time ở điểm đầu
                target_arrival = max(arrival_first_cust_raw, tw_start_first)

                # Giờ xuất phát = Giờ mục tiêu - Thời gian chạy xe từ kho
                travel_to_first = route[0].get('actual_travel_time', 0)
                jit_departure = target_arrival - travel_to_first

                route[0]['time'] = jit_departure
                route[0]['time_formatted'] = minutes_to_time(jit_departure)

            # 3. Forward Pass (Domino thuận): Xây dựng Segments và cập nhật giờ đến chuẩn
            segments = []
            current_time = route[0]['time']

            for i in range(len(route) - 1):
                stop = route[i]
                next_stop = route[i + 1]

                # Xe rời điểm hiện tại sau khi bốc dỡ xong
                departure_time = current_time + stop['service_time']
                # Thời gian lái xe đến điểm tiếp theo
                travel_time = stop.get('actual_travel_time', 0)
                arrival_next_raw = departure_time + travel_time

                # Ghi nhận chặng Di chuyển (Travel Segment)
                segments.append({
                    'type': 'travel',
                    'from_location': stop['location'],
                    'to_location': next_stop['location'],
                    'start_time': departure_time,
                    'end_time': arrival_next_raw,
                    'duration_minutes': travel_time,
                    'distance_km': next_stop['segment_distance']
                })

                # Xử lý thời gian chờ (Wait Time) nếu đến trước giờ khách mở cửa
                tw_start = next_stop['time_window'][0]
                arrival_next = arrival_next_raw
                if next_stop['location'] != depot_idx and arrival_next_raw < tw_start:
                    wait_time = tw_start - arrival_next_raw
                    segments.append({
                        'type': 'wait',
                        'location': next_stop['location'],
                        'start_time': arrival_next_raw,
                        'end_time': tw_start,
                        'duration_minutes': wait_time
                    })
                    arrival_next = tw_start  # Xe bắt đầu làm việc từ lúc mở cửa

                # KHÓA ĐỒNG BỘ: Cập nhật biến time cho Frontend vẽ Timeline
                next_stop['time'] = arrival_next
                next_stop['time_formatted'] = minutes_to_time(arrival_next)

                # Ghi nhận chặng Phục vụ (Service Segment)
                if next_stop['service_time'] > 0:
                    segments.append({
                        'type': 'service',
                        'location': next_stop['location'],
                        'start_time': arrival_next,
                        'end_time': arrival_next + next_stop['service_time'],
                        'duration_minutes': next_stop['service_time'],
                        'units': next_stop['load_before'] - next_stop['load_after']
                    })

                current_time = arrival_next  # Mốc thời gian tịnh tiến
            # =================================================================

            # Build timeline segments
            segments = self._build_timeline_segments(route, segment_distances)
            
            # Only count routes with customers
            if len(route) > 2:
                num_vehicles_used += 1
                route_distance = route_distance / 100.0
                route_load = total_route_load
                
                vehicle_capacity = self.problem_data['vehicle_capacities'][vehicle_id]
                saturation = (
                    (route_load / vehicle_capacity * 100) 
                    if vehicle_capacity > 0 
                    else 0
                )
                
                route_start_time = route[0]['time']
                route_end_time = route[-1]['time']
                route_duration_minutes = route_end_time - route_start_time
                
                # Calculate service time
                #service_time_minutes = 0
                num_customers = sum(1 for stop in route if stop['location'] != self.problem_data['depot'])

                # CỘNG DỒN 100% TỪ SEGMENTS VỪA VẼ (Single Source of Truth)
                service_time_minutes = sum(s['duration_minutes'] for s in segments if s['type'] == 'service')
                travel_time_minutes = sum(s['duration_minutes'] for s in segments if s['type'] == 'travel')
                
                routes.append({
                    'vehicle_id': vehicle_id,
                    'route': route,
                    'segments': segments,
                    'distance': route_distance,
                    'distance_km': route_distance,
                    'distance_formatted': f"{route_distance:.2f} km",
                    'load': route_load,
                    'load_units': route_load,
                    'load_formatted': f"{route_load} units",
                    'capacity': vehicle_capacity,
                    'saturation_pct': saturation,
                    'duration_minutes': route_duration_minutes,
                    'duration_formatted': (
                        f"{int(route_duration_minutes // 60)}h "
                        f"{int(route_duration_minutes % 60)}m"
                    ),
                    'duration_hours': round(route_duration_minutes / 60.0, 2),
                    'travel_time_minutes': travel_time_minutes,
                    'travel_time_hours': travel_time_minutes / 60.0,
                    'travel_time_formatted': (
                        f"{int(travel_time_minutes // 60)}h "
                        f"{int(travel_time_minutes % 60)}m"
                    ),
                    'service_time_minutes': service_time_minutes,
                    'service_time_hours': service_time_minutes / 60.0,
                    'service_time_formatted': (
                        f"{int(service_time_minutes // 60)}h "
                        f"{int(service_time_minutes % 60)}m"
                    ),
                    'num_customers': num_customers
                })
                
                total_distance += route_distance
                total_load += route_load
        
        # Calculate overall metrics
        return self._build_solution_summary(
            routes, num_vehicles_used, total_distance, total_load,
            solution.ObjectiveValue() / 100.0
        )

    def _build_timeline_segments(
            self,
            route: List[Dict],
            segment_distances: List[float]
    ) -> List[Dict]:
        """Xây dựng Timeline theo nguyên lý Domino tịnh tiến (Không âm)."""
        segments = []
        depot_idx = self.problem_data['depot']

        # Bắt đầu từ giờ xuất phát của Depot (đã được JIT Departure tính toán)
        current_time = route[0]['time']

        for i in range(len(route) - 1):
            from_stop = route[i]
            to_stop = route[i + 1]

            segment_distance_km = 0.0
            if i < len(segment_distances):
                segment_distance_km = segment_distances[i] / 100.0

            # 1. LẤY THỜI GIAN DI CHUYỂN OSRM (Không dính líu đến Service Time)
            raw_transit = self.problem_data['time_matrix'][from_stop['location']][to_stop['location']] / 100.0
            actual_travel_time = round(raw_transit)

            # 2. TÍNH THỜI GIAN PHỤC VỤ (Tại điểm đến)
            service_time_minutes = 0
            units_delivered = 0
            if to_stop['location'] != depot_idx:
                units_delivered = to_stop['load_before'] - to_stop['load_after']
                service_time_minutes = round(5 + (0.015 * units_delivered))

            # 3. CỘNG DỒN DOMINO (Đảm bảo thời gian luôn tiến lên phía trước)
            arrival_time = current_time + actual_travel_time

            tw_start = to_stop['time_window'][0]
            wait_time = max(0, tw_start - arrival_time) if to_stop['location'] != depot_idx else 0

            start_service_time = arrival_time + wait_time
            end_service_time = start_service_time + service_time_minutes

            # 4. GHI NHẬN SEGMENT TRAVEL (Màu tím)
            segments.append({
                'type': 'travel',
                'from_location': from_stop['location'],
                'to_location': to_stop['location'],
                'start_time': current_time,
                'end_time': arrival_time,
                'duration_minutes': actual_travel_time,
                'distance_km': round(segment_distance_km, 2)
            })

            # 5. GHI NHẬN SEGMENT SERVICE (Màu hồng - nếu có)
            if service_time_minutes > 0:
                segments.append({
                    'type': 'service',
                    'location': to_stop['location'],
                    'start_time': start_service_time,
                    'end_time': end_service_time,
                    'duration_minutes': service_time_minutes,
                    'units': units_delivered
                })

            # 6. KHÓA ĐỒNG BỘ: Ép thẻ text hiển thị đúng mốc giờ của biểu đồ vẽ ra
            to_stop['time'] = end_service_time
            to_stop['time_formatted'] = minutes_to_time(end_service_time)

            # Dịch chuyển mốc thời gian sang chặng tiếp theo
            current_time = end_service_time

        return segments
    
    def _build_solution_summary(
        self,
        routes: List[Dict],
        num_vehicles_used: int,
        total_distance: float,
        total_load: int,
        objective_value: float
    ) -> Dict:
        """Build comprehensive solution summary."""
        total_capacity = sum(r['capacity'] for r in routes) if routes else 0
        avg_saturation = (
            (total_load / total_capacity * 100) 
            if total_capacity > 0 
            else 0
        )
        
        total_trips = len(routes)
        avg_trips_per_vehicle = (
            total_trips / num_vehicles_used 
            if num_vehicles_used > 0 
            else 0
        )
        avg_distance_per_vehicle = (
            total_distance / num_vehicles_used 
            if num_vehicles_used > 0 
            else 0
        )
        
        total_duration_minutes = sum(r['duration_minutes'] for r in routes)
        total_travel_time_minutes = sum(r['travel_time_minutes'] for r in routes)
        total_service_time_minutes = sum(r['service_time_minutes'] for r in routes)
        
        avg_duration_per_vehicle_minutes = (
            total_duration_minutes / num_vehicles_used 
            if num_vehicles_used > 0 
            else 0
        )
        avg_travel_time_per_vehicle_minutes = (
            total_travel_time_minutes / num_vehicles_used 
            if num_vehicles_used > 0 
            else 0
        )
        avg_service_time_per_vehicle_minutes = (
            total_service_time_minutes / num_vehicles_used 
            if num_vehicles_used > 0 
            else 0
        )
        
        # Count customers served
        served_customers = set()
        depot = self.problem_data['depot']
        for route in routes:
            for stop in route['route']:
                if stop['location'] != depot:
                    served_customers.add(stop['location'])
        
        customers_served = len(served_customers)
        customers_total = len(self.problem_data['demands']) - 1
        
        # Log summary
        logger.info(f"  Vehicles used: {num_vehicles_used}/{self.problem_data['num_vehicles']}")
        logger.info(f"  Total trips: {total_trips}")
        logger.info(f"  Average trips per vehicle: {avg_trips_per_vehicle:.1f}")
        logger.info(f"  Total distance: {total_distance:.2f} km")
        logger.info(f"  Average distance per vehicle: {avg_distance_per_vehicle:.2f} km")
        logger.info(
            f"  Total load: {total_load}/{total_capacity} units "
            f"({avg_saturation:.1f}% saturation)"
        )
        logger.info(
            f"  Average travel time per vehicle: {avg_travel_time_per_vehicle_minutes:.0f} min "
            f"(excluding service)"
        )
        logger.info(
            f"  Average total time per vehicle: {avg_duration_per_vehicle_minutes:.0f} min "
            f"(including service)"
        )
        logger.info(f"  Customers served: {customers_served}/{customers_total}")
        
        return {
            'status': 'success',
            'num_vehicles_used': num_vehicles_used,
            'total_vehicles_available': self.problem_data['num_vehicles'],
            'total_trips': total_trips,
            'avg_trips_per_vehicle': round(avg_trips_per_vehicle, 2),
            'total_distance': total_distance,
            'total_distance_km': round(total_distance, 2),
            'total_distance_formatted': f"{total_distance:.2f} km",
            'avg_distance_per_vehicle_km': round(avg_distance_per_vehicle, 2),
            'avg_distance_per_vehicle_formatted': f"{avg_distance_per_vehicle:.2f} km",
            'total_load': total_load,
            'average_saturation_pct': round(avg_saturation, 1),
            'total_duration_minutes': round(total_duration_minutes, 1),
            'total_duration_hours': round(total_duration_minutes / 60.0, 2),
            'total_duration_formatted': (
                f"{int(total_duration_minutes // 60)}h "
                f"{int(total_duration_minutes % 60)}m"
            ),
            'total_travel_time_minutes': round(total_travel_time_minutes, 1),
            'total_travel_time_hours': round(total_travel_time_minutes / 60.0, 2),
            'total_travel_time_formatted': (
                f"{int(total_travel_time_minutes // 60)}h "
                f"{int(total_travel_time_minutes % 60)}m"
            ),
            'total_service_time_minutes': round(total_service_time_minutes, 1),
            'total_service_time_hours': round(total_service_time_minutes / 60.0, 2),
            'total_service_time_formatted': (
                f"{int(total_service_time_minutes // 60)}h "
                f"{int(total_service_time_minutes % 60)}m"
            ),
            'avg_duration_per_vehicle_minutes': round(avg_duration_per_vehicle_minutes, 1),
            'avg_duration_per_vehicle_hours': round(avg_duration_per_vehicle_minutes / 60.0, 2),
            'avg_duration_per_vehicle_formatted': (
                f"{int(avg_duration_per_vehicle_minutes // 60)}h "
                f"{int(avg_duration_per_vehicle_minutes % 60)}m"
            ),
            'avg_travel_time_per_vehicle_minutes': round(avg_travel_time_per_vehicle_minutes, 1),
            'avg_travel_time_per_vehicle_hours': round(avg_travel_time_per_vehicle_minutes / 60.0, 2),
            'avg_travel_time_per_vehicle_formatted': (
                f"{int(avg_travel_time_per_vehicle_minutes // 60)}h "
                f"{int(avg_travel_time_per_vehicle_minutes % 60)}m"
            ),
            'avg_service_time_per_vehicle_minutes': round(avg_service_time_per_vehicle_minutes, 1),
            'avg_service_time_per_vehicle_hours': round(avg_service_time_per_vehicle_minutes / 60.0, 2),
            'avg_service_time_per_vehicle_formatted': (
                f"{int(avg_service_time_per_vehicle_minutes // 60)}h "
                f"{int(avg_service_time_per_vehicle_minutes % 60)}m"
            ),
            'routes': routes,
            'customers_served': customers_served,
            'customers_total': customers_total,
            'objective_value': objective_value
        }
