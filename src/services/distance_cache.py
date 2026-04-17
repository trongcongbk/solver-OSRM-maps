"""
Distance and travel time cache service using SQLite and OSRM routing service.
Stores real distances (km) and travel times (morning/afternoon/evening) for all location pairs.
"""
import sqlite3
import hashlib
import urllib.request
import urllib.error
import json
import time
from typing import List, Tuple, Optional, Dict

from ..config import get_logger
from ..utils import haversine_distance

logger = get_logger(__name__)


class DistanceCacheService:
    """Manages a SQLite cache of distances and travel times between locations."""
    
    def __init__(self, db_path: str = "distance_cache.db", osrm_base_url: str = "http://router.project-osrm.org"):
        """
        Initialize the distance cache service.
        
        Args:
            db_path: Path to SQLite database file
            osrm_base_url: Base URL for OSRM routing API
        """
        self.db_path = db_path
        self.osrm_base_url = osrm_base_url
        self._init_db()
    
    def _init_db(self):
        """Initialize the SQLite database with required tables."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Table: locations - stores unique locations with a hash as key
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS locations (
                location_hash TEXT PRIMARY KEY,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Table: distances - stores distance and travel times between location pairs
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS distances (
                from_hash TEXT NOT NULL,
                to_hash TEXT NOT NULL,
                distance_km REAL NOT NULL,
                time_morning_min REAL NOT NULL,
                time_afternoon_min REAL NOT NULL,
                time_evening_min REAL NOT NULL,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                PRIMARY KEY (from_hash, to_hash),
                FOREIGN KEY (from_hash) REFERENCES locations(location_hash),
                FOREIGN KEY (to_hash) REFERENCES locations(location_hash)
            )
        """)
        
        conn.commit()
        conn.close()
        logger.info(f"Distance cache database initialized at {self.db_path}")
    
    @staticmethod
    def _location_hash(lat: float, lon: float) -> str:
        """Create a unique hash for a location (rounded to 6 decimals for ~0.1m precision)."""
        lat_round = round(lat, 6)
        lon_round = round(lon, 6)
        return hashlib.md5(f"{lat_round},{lon_round}".encode()).hexdigest()[:16]
    
    def _get_or_create_location(self, lat: float, lon: float) -> str:
        """Get location hash, creating entry if it doesn't exist."""
        loc_hash = self._location_hash(lat, lon)
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute("SELECT location_hash FROM locations WHERE location_hash = ?", (loc_hash,))
        if cursor.fetchone() is None:
            cursor.execute(
                "INSERT INTO locations (location_hash, latitude, longitude) VALUES (?, ?, ?)",
                (loc_hash, lat, lon)
            )
            conn.commit()
            logger.debug(f"Added new location: {lat},{lon} -> {loc_hash}")
        
        conn.close()
        return loc_hash
    
    def _fetch_from_osrm(self, from_lat: float, from_lon: float, 
                         to_lat: float, to_lon: float) -> Optional[Tuple[float, float]]:
        """
        Fetch real distance and travel time from OSRM routing service.
        
        Returns:
            (distance_km, duration_min) or None if request fails
        """
        # OSRM route API: /route/v1/{profile}/{coordinates}
        # coordinates: lon,lat;lon,lat (note: OSRM uses lon,lat order!)
        url = f"{self.osrm_base_url}/route/v1/driving/{from_lon},{from_lat};{to_lon},{to_lat}?overview=false"
        
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=10) as response:
                data = json.loads(response.read().decode('utf-8'))
                
                if data.get('code') == 'Ok' and data.get('routes'):
                    route = data['routes'][0]
                    distance_m = route['distance']  # meters
                    duration_s = route['duration']  # seconds
                    
                    distance_km = distance_m / 1000.0
                    duration_min = duration_s / 60.0
                    
                    logger.debug(f"OSRM: {from_lat},{from_lon} -> {to_lat},{to_lon}: {distance_km:.2f}km, {duration_min:.1f}min")
                    return (distance_km, duration_min)
                else:
                    logger.warning(f"OSRM returned non-Ok code: {data.get('code')}")
                    return None
                    
        except urllib.error.URLError as e:
            logger.error(f"OSRM request failed: {e}")
            return None
        except Exception as e:
            logger.error(f"Error fetching from OSRM: {e}")
            return None
    
    def get_distance_and_time(self, from_lat: float, from_lon: float,
                              to_lat: float, to_lon: float,
                              time_of_day: str = "afternoon") -> Tuple[float, float]:
        """
        Get distance (km) and travel time (min) between two locations.
        Uses cache if available, otherwise fetches from OSRM and caches result.
        
        Args:
            from_lat, from_lon: Origin coordinates
            to_lat, to_lon: Destination coordinates
            time_of_day: 'morning', 'afternoon', or 'evening' (default: 'afternoon')
        
        Returns:
            (distance_km, travel_time_min)
        """
        # Same location -> zero distance/time
        if abs(from_lat - to_lat) < 1e-6 and abs(from_lon - to_lon) < 1e-6:
            return (0.0, 0.0)
        
        from_hash = self._get_or_create_location(from_lat, from_lon)
        to_hash = self._get_or_create_location(to_lat, to_lon)
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Check cache
        cursor.execute("""
            SELECT distance_km, time_morning_min, time_afternoon_min, time_evening_min
            FROM distances
            WHERE from_hash = ? AND to_hash = ?
        """, (from_hash, to_hash))
        
        row = cursor.fetchone()
        
        if row:
            # Cache hit
            distance_km, time_morning, time_afternoon, time_evening = row
            time_map = {
                'morning': time_morning,
                'afternoon': time_afternoon,
                'evening': time_evening
            }
            travel_time = time_map.get(time_of_day, time_afternoon)
            conn.close()
            return (distance_km, travel_time)
        
        # Cache miss - fetch from OSRM
        logger.info(f"Cache miss for {from_lat},{from_lon} -> {to_lat},{to_lon}, fetching from OSRM...")
        result = self._fetch_from_osrm(from_lat, from_lon, to_lat, to_lon)
        
        if result is None:
            # Fallback: use Haversine distance and estimate time (no traffic data)
            distance_km = haversine_distance((from_lat, from_lon), (to_lat, to_lon))
            # Estimate: 40 km/h average speed
            travel_time = distance_km / 40.0 * 60.0  # minutes
            logger.warning(f"OSRM failed, using Haversine fallback: {distance_km:.2f}km, {travel_time:.1f}min")
        else:
            distance_km, base_time = result
            # Estimate traffic variations: morning +15%, afternoon baseline, evening +10%
            travel_time = base_time
            time_morning = base_time * 1.15
            time_afternoon = base_time
            time_evening = base_time * 1.10
        
        # For fallback case, use same time for all periods
        if result is None:
            time_morning = time_afternoon = time_evening = travel_time
        
        # Store in cache (both directions)
        cursor.execute("""
            INSERT OR REPLACE INTO distances 
            (from_hash, to_hash, distance_km, time_morning_min, time_afternoon_min, time_evening_min)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (from_hash, to_hash, distance_km, time_morning, time_afternoon, time_evening))
        
        # Also store reverse direction (symmetric for car routing)
        #cursor.execute("""
            #INSERT OR REPLACE INTO distances
            #(from_hash, to_hash, distance_km, time_morning_min, time_afternoon_min, time_evening_min)
            #VALUES (?, ?, ?, ?, ?, ?)
        #""", (to_hash, from_hash, distance_km, time_morning, time_afternoon, time_evening))
        
        conn.commit()
        conn.close()
        
        # Rate limiting: small delay to avoid overwhelming OSRM
        if result is not None:
            time.sleep(0.1)
        
        time_map = {
            'morning': time_morning,
            'afternoon': time_afternoon,
            'evening': time_evening
        }
        travel_time_final = time_map.get(time_of_day, time_afternoon)
        
        return (distance_km, travel_time_final)
    
    def populate_matrix_all_times(self, locations: List[Tuple[float, float]]) -> Tuple[List[List[float]], List[List[float]], List[List[float]], List[List[float]]]:
        """
        Populate distance and all three time matrices (morning/afternoon/evening) for a list of locations.
        Fetches missing entries from OSRM and caches them.
        
        Args:
            locations: List of (latitude, longitude) tuples
        
        Returns:
            (distance_matrix, time_matrix_morning, time_matrix_afternoon, time_matrix_evening) - all as 2D lists in minutes
        """
        # Thêm dòng này ngay trước khi gọi hàm
        logger.info(f"Dữ liệu locations thô trước khi đưa vào tính toán: {locations}")
        valid_locations = []
        for idx, loc in enumerate(locations):
            # Kiểm tra xem loc có tồn tại và có đúng 2 phần tử (lat, lon) không
            if not loc or len(loc) != 2:
                logger.warning(f"Dữ liệu đầu vào bị lỗi tại index {idx}: {loc}. Đang bỏ qua tọa độ này.")
                continue
            valid_locations.append(loc)
    
        # Gán lại mảng đã lọc và đếm lại số lượng
        locations = valid_locations
        n = len(locations)
        distance_matrix = [[0.0] * n for _ in range(n)]
        time_matrix_morning = [[0.0] * n for _ in range(n)]
        time_matrix_afternoon = [[0.0] * n for _ in range(n)]
        time_matrix_evening = [[0.0] * n for _ in range(n)]
        
        total_pairs = n * (n - 1)  # Exclude diagonal
        cache_hits = 0
        osrm_calls = 0
        
        logger.info(f"Populating matrices for {n} locations ({total_pairs} pairs, all time periods)...")
        
        for i in range(n):
            for j in range(n):
                if i == j:
                    distance_matrix[i][j] = 0.0
                    time_matrix_morning[i][j] = 0.0
                    time_matrix_afternoon[i][j] = 0.0
                    time_matrix_evening[i][j] = 0.0
                else:
                    lat1, lon1 = locations[i]
                    lat2, lon2 = locations[j]
                    
                    # Check if already in cache
                    from_hash = self._location_hash(lat1, lon1)
                    to_hash = self._location_hash(lat2, lon2)
                    
                    conn = sqlite3.connect(self.db_path)
                    cursor = conn.cursor()
                    cursor.execute("""
                        SELECT distance_km, time_morning_min, time_afternoon_min, time_evening_min
                        FROM distances WHERE from_hash = ? AND to_hash = ?
                    """, (from_hash, to_hash))
                    row = cursor.fetchone()
                    conn.close()
                    
                    if row:
                        # Cache hit
                        cache_hits += 1
                        distance_matrix[i][j] = row[0]
                        time_matrix_morning[i][j] = row[1]
                        time_matrix_afternoon[i][j] = row[2]
                        time_matrix_evening[i][j] = row[3]
                    else:
                        # Cache miss - need OSRM call
                        osrm_calls += 1
                        dist_km, _ = self.get_distance_and_time(
                            lat1, lon1, lat2, lon2, time_of_day='afternoon'
                        )
                        
                        # Now retrieve from cache (just populated)
                        conn = sqlite3.connect(self.db_path)
                        cursor = conn.cursor()
                        cursor.execute("""
                            SELECT distance_km, time_morning_min, time_afternoon_min, time_evening_min
                            FROM distances WHERE from_hash = ? AND to_hash = ?
                        """, (from_hash, to_hash))
                        row = cursor.fetchone()
                        conn.close()
                        
                        if row:
                            distance_matrix[i][j] = row[0]
                            time_matrix_morning[i][j] = row[1]
                            time_matrix_afternoon[i][j] = row[2]
                            time_matrix_evening[i][j] = row[3]
                        else:
                            # Fallback (should not happen)
                            distance_matrix[i][j] = dist_km
                            time_matrix_morning[i][j] = dist_km / 40.0 * 60.0 * 1.15
                            time_matrix_afternoon[i][j] = dist_km / 40.0 * 60.0
                            time_matrix_evening[i][j] = dist_km / 40.0 * 60.0 * 1.10
        
        cache_hit_rate = (cache_hits / total_pairs * 100) if total_pairs > 0 else 0
        logger.info(f"✓ Matrix ready - Cache: {cache_hits}/{total_pairs} hits ({cache_hit_rate:.1f}%), OSRM calls: {osrm_calls}")
        return (distance_matrix, time_matrix_morning, time_matrix_afternoon, time_matrix_evening)
