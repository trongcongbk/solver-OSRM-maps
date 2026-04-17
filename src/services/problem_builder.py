"""Problem builder service to construct solver input from JSON payloads."""

from typing import Dict, Optional, List, Tuple


class ProblemBuilder:
    """Builds solver input from various JSON formats."""
    
    @staticmethod
    def build_from_payload(payload: dict, date: str, speed_kmph: float = 40.0) -> Optional[Dict]:
        """
        Build solver input for a single date from JSON payload.
        
        Args:
            payload: JSON data containing depot, vehicles, customers
            date: Date string (YYYY-MM-DD)
            speed_kmph: Vehicle speed in km/h (default: 40.0)
            
        Returns:
            Solver data dictionary or None if no active customers
        """
        depot = payload.get('depot', {}).get('location', [])
        vehicles = payload.get('vehicles', [])
        customers = payload.get('customers', [])
        
        # Build list of active customers for the date
        active_customers = []
        for c in customers:
            # Support two formats:
            # - aggregated file: 'demands_units' is a dict keyed by date
            # - per-day file: 'demand_units' is a scalar for that day
            d = 0
            if 'demands_units' in c:
                d = c.get('demands_units', {}).get(date, 0)
            elif 'demand_units' in c:
                d = c.get('demand_units', 0)
            
            if d and d > 0:
                # Gather time window for date
                tw = None
                if 'time_windows' in c:
                    tw = c.get('time_windows', {}).get(date)
                if not tw and 'time_window' in c:
                    tw = c.get('time_window')
                if not tw:
                    # Fallback to full day
                    tw = {'start_min': 240, 'end_min': 1260, 'start_hhmm': '04:00', 'end_hhmm': '21:00'}
                
                active_customers.append({
                    'id': c.get('id'),
                    'name': c.get('name'),
                    'location': c.get('location'),
                    'demand': d,
                    'time_window': (tw.get('start_min', 240), tw.get('end_min', 1260)),
                    'service_time_min': c.get('service_time_min', 15)
                })
        
        # If no customers active on this date, return None
        if len(active_customers) == 0:
            return None
        
        # Build locations: depot first, then customers
        locations = [tuple(depot)] + [tuple(c['location']) for c in active_customers]
        
        # Demands: depot has 0
        demands = [0] + [int(c['demand']) for c in active_customers]
        
        # Time windows in minutes
        time_windows = [(0, 24*60)]  # Placeholder for depot
        time_windows += [c['time_window'] for c in active_customers]
        
        # Service time: choose max or default 15
        service_time = max([c.get('service_time_min', 15) for c in active_customers] + [15])
        
        # Vehicle capacities and number
        vehicle_capacities = [int(v.get('capacity_units', 0)) for v in vehicles]
        num_vehicles = len(vehicle_capacities)
        
        # Determine depot time window
        depot_tw = payload.get('metadata', {}).get('depot_time_window')
        if not depot_tw:
            # Use global vehicle window min/max
            starts = [v.get('time_window', {}).get('start_min', 240) for v in vehicles]
            ends = [v.get('time_window', {}).get('end_min', 1260) for v in vehicles]
            if starts and ends:
                depot_tw = {'start_min': min(starts), 'end_min': max(ends)}
            else:
                depot_tw = {'start_min': 240, 'end_min': 1260}
        
        # Set depot time window
        time_windows[0] = (depot_tw['start_min'], depot_tw['end_min'])
        
        # Build solver data
        solver_data = {
            'locations': locations,
            'demands': demands,
            'time_windows': time_windows,
            'vehicle_capacities': vehicle_capacities,
            'num_vehicles': num_vehicles,
            'depot': 0,
            'service_time': service_time,
            'vehicle_speed': (speed_kmph / 60.0),  # Convert km/h to km/min
            'coord_type': 'latlon'
        }
        
        return solver_data
    
    @staticmethod
    def infer_date_from_payload(payload: dict) -> Optional[str]:
        """
        Infer date from payload.
        
        Preference order: top-level 'date', metadata.date, metadata.date_range[0]
        
        Args:
            payload: JSON payload
            
        Returns:
            Date string or None
        """
        if not isinstance(payload, dict):
            return None
        if 'date' in payload:
            return payload.get('date')
        md = payload.get('metadata', {})
        if isinstance(md, dict) and 'date' in md:
            return md.get('date')
        if isinstance(md, dict) and 'date_range' in md:
            date_range = md.get('date_range')
            if isinstance(date_range, list) and len(date_range) >= 1:
                return date_range[0]
        return None
    
    @staticmethod
    def enrich_solution_routes(solution: Dict, problem: Dict, payload: dict, solved_date: str) -> List[Dict]:
        """
        Enrich solution routes with customer information.
        
        Args:
            solution: Raw solver solution
            problem: Problem data used for solving
            payload: Original request payload
            solved_date: Date that was solved
            
        Returns:
            List of enriched routes
        """
        locations = problem['locations']
        
        # Rebuild active_customers by matching coordinates
        active_customers = []
        for loc_idx in range(1, len(locations)):
            coord = tuple(locations[loc_idx])
            match = None
            for c in payload.get('customers', []):
                if tuple(c.get('location', ())) == coord:
                    # Find demand for this date
                    d = 0
                    if 'demands_units' in c:
                        d = c.get('demands_units', {}).get(solved_date, 0)
                    elif 'demand_units' in c:
                        d = c.get('demand_units', 0)
                    if d and d > 0:
                        match = c
                        break
            if match:
                active_customers.append(match)
            else:
                active_customers.append({'id': None, 'name': None, 'location': coord})
        
        # Transform routes
        routes_out = []
        for r in solution['routes']:
            new_r = r.copy()
            new_route = []
            for stop in r['route']:
                stop_location_idx = stop.get('location')
                
                if stop_location_idx == 0:
                    # Depot stop
                    loc_info = {'type': 'depot', 'index': 0, 'location': locations[0]}
                else:
                    # Customer stop
                    customer_idx = stop_location_idx - 1
                    if customer_idx < len(active_customers):
                        cust = active_customers[customer_idx]
                        loc_info = {
                            'type': 'customer',
                            'index': stop_location_idx,
                            'customer_id': cust.get('id'),
                            'customer_name': cust.get('name'),
                            'location': locations[stop_location_idx] if stop_location_idx < len(locations) else None
                        }
                    else:
                        loc_info = {
                            'type': 'customer',
                            'index': stop_location_idx,
                            'customer_id': None,
                            'customer_name': None,
                            'location': locations[stop_location_idx] if stop_location_idx < len(locations) else None
                        }
                
                new_stop = stop.copy()
                new_stop['location_info'] = loc_info
                new_route.append(new_stop)
            
            new_r['route'] = new_route
            routes_out.append(new_r)
        
        return routes_out
