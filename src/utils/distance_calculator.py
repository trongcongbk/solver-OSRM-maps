"""Distance calculation utilities."""

import math
from typing import Tuple


def haversine_distance(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    """
    Calculate haversine distance between two coordinates in kilometers.
    
    Args:
        coord1: First coordinate as (latitude, longitude) in degrees
        coord2: Second coordinate as (latitude, longitude) in degrees
        
    Returns:
        Distance in kilometers
    """
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    
    return 6371.0 * c  # Earth radius in km


def euclidean_distance(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    """
    Calculate Euclidean distance between two coordinates.
    
    Args:
        coord1: First coordinate as (x, y)
        coord2: Second coordinate as (x, y)
        
    Returns:
        Euclidean distance in the same units as the input coordinates
    """
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)
