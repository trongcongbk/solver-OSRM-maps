"""Domain models for CVRPTW problem representation."""

from typing import List, Dict, Optional, Tuple
from pydantic import BaseModel, Field


class Location(BaseModel):
    """Geographic location with coordinates."""
    latitude: float = Field(..., description="Latitude in degrees")
    longitude: float = Field(..., description="Longitude in degrees")

    def as_tuple(self) -> Tuple[float, float]:
        """Return location as (lat, lon) tuple."""
        return (self.latitude, self.longitude)


class TimeWindow(BaseModel):
    """Time window constraint for a location."""
    start_min: int = Field(..., description="Start time in minutes from midnight")
    end_min: int = Field(..., description="End time in minutes from midnight")
    start_hhmm: Optional[str] = Field(None, description="Start time in HH:MM format")
    end_hhmm: Optional[str] = Field(None, description="End time in HH:MM format")


class Customer(BaseModel):
    """Customer with location, demand, and time window."""
    id: str = Field(..., description="Unique customer identifier")
    name: Optional[str] = Field(None, description="Customer name")
    location: Location = Field(..., description="Customer location")
    demand_units: Optional[int] = Field(None, description="Demand for single day")
    demands_units: Optional[Dict[str, int]] = Field(None, description="Demand per date")
    time_window: Optional[TimeWindow] = Field(None, description="Time window for single day")
    time_windows: Optional[Dict[str, TimeWindow]] = Field(None, description="Time windows per date")
    service_time_min: int = Field(15, description="Service time in minutes")


class Vehicle(BaseModel):
    """Vehicle with capacity and time window."""
    id: str = Field(..., description="Unique vehicle identifier")
    capacity_units: int = Field(..., description="Vehicle capacity in units")
    time_window: Optional[TimeWindow] = Field(None, description="Vehicle availability window")


class Depot(BaseModel):
    """Depot location where vehicles start and end."""
    location: Location = Field(..., description="Depot location")
    time_window: Optional[TimeWindow] = Field(None, description="Depot operating hours")


class ProblemData(BaseModel):
    """Complete CVRPTW problem data."""
    locations: List[Tuple[float, float]] = Field(..., description="List of (lat, lon) tuples")
    demands: List[int] = Field(..., description="Demand at each location")
    time_windows: List[Tuple[int, int]] = Field(..., description="Time window (start, end) for each location")
    vehicle_capacities: List[int] = Field(..., description="Capacity of each vehicle")
    num_vehicles: int = Field(..., description="Number of available vehicles")
    depot: int = Field(0, description="Index of depot location")
    service_time: int = Field(15, description="Service time in minutes")
    coord_type: str = Field("latlon", description="Coordinate type: 'latlon' or 'euclidean'")
    distance_matrix: Optional[List[List[float]]] = Field(None, description="Precomputed distance matrix")
    time_matrix: Optional[List[List[int]]] = Field(None, description="Precomputed time matrix (scaled by 100)")


class RouteStop(BaseModel):
    """A stop in a vehicle route."""
    location: int = Field(..., description="Location index")
    location_info: Optional[Dict] = Field(None, description="Location details (depot/customer info)")
    arrival_time: float = Field(..., description="Arrival time in minutes from midnight")
    time_formatted: str = Field(..., description="Arrival time in HH:MM format")
    time_window: Tuple[int, int] = Field(..., description="Time window (start, end)")
    time_window_formatted: str = Field(..., description="Time window in HH:MM format")
    demand: int = Field(0, description="Demand at this location")
    load_before: int = Field(..., description="Load before serving this location")
    load_after: int = Field(..., description="Load after serving this location")
    segment_distance: float = Field(0.0, description="Distance to this stop from previous")
    segment_distance_formatted: str = Field("0.00 km", description="Formatted segment distance")


class Route(BaseModel):
    """A vehicle route with stops and metrics."""
    vehicle_id: int = Field(..., description="Vehicle identifier")
    route: List[RouteStop] = Field(..., description="Ordered list of stops")
    distance: float = Field(..., description="Total route distance in km")
    distance_km: float = Field(..., description="Total route distance in km (explicit)")
    distance_formatted: str = Field(..., description="Formatted distance")
    load: int = Field(..., description="Total load delivered")
    load_units: int = Field(..., description="Total load in units (explicit)")
    load_formatted: str = Field(..., description="Formatted load")
    capacity: int = Field(..., description="Vehicle capacity")
    saturation_pct: float = Field(..., description="Capacity utilization percentage")
    duration_minutes: float = Field(..., description="Total route duration in minutes")
    duration_formatted: str = Field(..., description="Formatted duration")
    duration_hours: float = Field(..., description="Total route duration in hours")
    travel_time_minutes: float = Field(..., description="Travel time in minutes")
    travel_time_hours: float = Field(..., description="Travel time in hours")
    travel_time_formatted: str = Field(..., description="Formatted travel time")
    service_time_minutes: float = Field(..., description="Service time in minutes")
    service_time_hours: float = Field(..., description="Service time in hours")
    service_time_formatted: str = Field(..., description="Formatted service time")
    num_customers: int = Field(..., description="Number of customers served")


class Solution(BaseModel):
    """Complete solution to a CVRPTW problem."""
    status: str = Field("success", description="Solution status")
    num_vehicles_used: int = Field(..., description="Number of vehicles used")
    total_vehicles_available: int = Field(..., description="Total vehicles available")
    total_trips: int = Field(..., description="Total number of trips")
    avg_trips_per_vehicle: float = Field(..., description="Average trips per vehicle")
    total_distance: float = Field(..., description="Total distance traveled")
    total_distance_km: float = Field(..., description="Total distance in km")
    total_distance_formatted: str = Field(..., description="Formatted total distance")
    avg_distance_per_vehicle_km: float = Field(..., description="Average distance per vehicle")
    avg_distance_per_vehicle_formatted: str = Field(..., description="Formatted average distance")
    total_load: int = Field(..., description="Total load delivered")
    average_saturation_pct: float = Field(..., description="Average vehicle capacity utilization")
    total_duration_minutes: float = Field(..., description="Total duration in minutes")
    total_duration_hours: float = Field(..., description="Total duration in hours")
    total_duration_formatted: str = Field(..., description="Formatted total duration")
    total_travel_time_minutes: float = Field(..., description="Total travel time in minutes")
    total_travel_time_hours: float = Field(..., description="Total travel time in hours")
    total_travel_time_formatted: str = Field(..., description="Formatted total travel time")
    total_service_time_minutes: float = Field(..., description="Total service time in minutes")
    total_service_time_hours: float = Field(..., description="Total service time in hours")
    total_service_time_formatted: str = Field(..., description="Formatted total service time")
    avg_duration_per_vehicle_minutes: float = Field(..., description="Average duration per vehicle")
    avg_duration_per_vehicle_hours: float = Field(..., description="Average duration per vehicle in hours")
    avg_duration_per_vehicle_formatted: str = Field(..., description="Formatted average duration")
    avg_travel_time_per_vehicle_minutes: float = Field(..., description="Average travel time per vehicle")
    avg_travel_time_per_vehicle_hours: float = Field(..., description="Average travel time per vehicle in hours")
    avg_travel_time_per_vehicle_formatted: str = Field(..., description="Formatted average travel time")
    avg_service_time_per_vehicle_minutes: float = Field(..., description="Average service time per vehicle")
    avg_service_time_per_vehicle_hours: float = Field(..., description="Average service time per vehicle in hours")
    avg_service_time_per_vehicle_formatted: str = Field(..., description="Formatted average service time")
    routes: List[Route] = Field(..., description="List of vehicle routes")
    customers_served: int = Field(..., description="Number of customers served")
    customers_total: int = Field(..., description="Total number of customers")
    dropped_customers: Optional[List[Dict]] = Field(None, description="Customers not served")
    objective_value: Optional[float] = Field(None, description="Objective function value")
    solver: Optional[str] = Field(None, description="Solver used (ortools/gurobi)")
    error_type: Optional[str] = Field(None, description="Error type if status is error")
    message: Optional[str] = Field(None, description="Error message if status is error")
