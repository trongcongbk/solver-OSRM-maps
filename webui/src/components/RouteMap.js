import React, { useEffect, useRef, useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap, CircleMarker, Tooltip } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './RouteMap.css';

// Fix Leaflet default marker icons
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

// Custom icons
const depotIcon = new L.Icon({
  iconUrl: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNDgiIGhlaWdodD0iNDgiIHZpZXdCb3g9IjAgMCA0OCA0OCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48Y2lyY2xlIGN4PSIyNCIgY3k9IjI0IiByPSIxOCIgZmlsbD0iIzY2N2VlYSIgc3Ryb2tlPSJ3aGl0ZSIgc3Ryb2tlLXdpZHRoPSI0Ii8+PHRleHQgeD0iMjQiIHk9IjMxIiBmb250LXNpemU9IjIyIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSJ3aGl0ZSI+8J+PrTwvdGV4dD48L3N2Zz4=',
  iconSize: [48, 48],
  iconAnchor: [24, 24],
  popupAnchor: [0, -24],
});

const depotReturnIcon = new L.Icon({
  iconUrl: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNDgiIGhlaWdodD0iNDgiIHZpZXdCb3g9IjAgMCA0OCA0OCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48Y2lyY2xlIGN4PSIyNCIgY3k9IjI0IiByPSIxOCIgZmlsbD0iIzQ1NWE2NCIgc3Ryb2tlPSJ3aGl0ZSIgc3Ryb2tlLXdpZHRoPSI0Ii8+PHRleHQgeD0iMjQiIHk9IjMxIiBmb250LXNpemU9IjIyIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSJ3aGl0ZSI+8J+PrTwvdGV4dD48L3N2Zz4=',
  iconSize: [48, 48],
  iconAnchor: [24, 24],
  popupAnchor: [0, -24],
});

const customerIcon = new L.Icon({
  iconUrl: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNDgiIGhlaWdodD0iNDgiIHZpZXdCb3g9IjAgMCA0OCA0OCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48Y2lyY2xlIGN4PSIyNCIgY3k9IjI0IiByPSIxNiIgZmlsbD0iIzc2NGJhMiIgc3Ryb2tlPSJ3aGl0ZSIgc3Ryb2tlLXdpZHRoPSI0Ii8+PHRleHQgeD0iMjQiIHk9IjMxIiBmb250LXNpemU9IjIwIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBmaWxsPSJ3aGl0ZSI+8J+TpjwvdGV4dD48L3N2Zz4=',
  iconSize: [48, 48],
  iconAnchor: [24, 24],
  popupAnchor: [0, -24],
});

function FitBounds({ locations }) {
  const map = useMap();
  
  useEffect(() => {
    if (locations && locations.length > 0) {
      const bounds = L.latLngBounds(locations);
      map.fitBounds(bounds, { padding: [50, 50] });
    }
  }, [locations, map]);
  
  return null;
}

// Fetch OSRM route between consecutive points
async function fetchOSRMRoute(locations) {
  if (locations.length < 2) return [];
  
  try {
    // Build coordinates string: lon,lat;lon,lat;...
    const coords = locations.map(loc => `${loc[1]},${loc[0]}`).join(';');
    const url = `https://router.project-osrm.org/route/v1/driving/${coords}?overview=full&geometries=geojson`;
    
    const response = await fetch(url);
    const data = await response.json();
    
    if (data.code === 'Ok' && data.routes && data.routes.length > 0) {
      // OSRM returns coordinates as [lon, lat], convert to [lat, lon] for Leaflet
      const geometry = data.routes[0].geometry.coordinates;
      return geometry.map(coord => [coord[1], coord[0]]);
    }
  } catch (error) {
    console.error('OSRM routing error:', error);
  }
  
  // Fallback to straight lines
  return locations;
}

function RouteMap({ route, onClose }) {
  const mapRef = useRef(null);
  const [pathCoordinates, setPathCoordinates] = useState([]);
  const [loading, setLoading] = useState(true);

  // Extract locations from route - use location array [lat, lon]
  const locations = route.route
    .map(stop => {
      const loc = stop.location_info?.location;
      // Leaflet expects [lat, lon] format
      return loc && Array.isArray(loc) && loc.length === 2 ? loc : null;
    })
    .filter(loc => loc !== null);

  // Fetch OSRM route on mount
  useEffect(() => {
    async function loadRoute() {
      setLoading(true);
      const osrmPath = await fetchOSRMRoute(locations);
      setPathCoordinates(osrmPath);
      setLoading(false);
    }
    
    if (locations.length > 0) {
      loadRoute();
    }
  }, [route]);

  // Get depot and customer stops
  const depotStops = route.route.filter(stop => stop.location_info?.type === 'depot');
  const customerStops = route.route.filter(stop => stop.location_info?.type === 'customer');

  return (
    <div className="route-map-overlay">
      <div className="route-map-container">
        <div className="map-header">
          <h2>🗺️ Route Map - Vehicle {route.vehicle_id + 1} (Capacity: {route.capacity} units)</h2>
          <button className="close-btn" onClick={onClose}>✕</button>
        </div>

        <div className="map-info">
          <div className="info-item">
            <span className="info-label">Distance:</span>
            <span className="info-value">{route.distance_formatted}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Duration:</span>
            <span className="info-value">{route.duration_formatted}</span>
          </div>
          <div className="info-item">
            <span className="info-label">Stops:</span>
            <span className="info-value">{route.num_customers} customers</span>
          </div>
          <div className="info-item">
            <span className="info-label">Load:</span>
            <span className="info-value">{route.load_formatted}</span>
          </div>
        </div>

        {/* Two-column layout: Route Details on left, Map on right */}
        <div className="map-content-wrapper">
          {/* Route Details - left side */}
          <div className="route-stops-list">
            <h3>Route Details</h3>
            <div className="stops-timeline">
              {route.route.map((stop, index) => (
                <div key={index} className="stop-item">
                  <div className="stop-number">{index + 1}</div>
                  <div className="stop-details">
                    <div className="stop-title">
                      {stop.location_info?.type === 'depot' ? '🏭 Depot' : `📦 ${stop.location_info?.customer_name || stop.location_info?.customer_id || 'Customer'}`}
                    </div>
                    <div className="stop-info">
                      <span>⏰ {stop.time_formatted}</span>
                      <span>📦 {stop.load_before} → {stop.load_after} units</span>
                    </div>
                    {stop.location_info?.type !== 'depot' && (
                      <div className="stop-window">
                        Window: {stop.time_window_formatted}
                      </div>
                    )}
                    {index > 0 && stop.segment_distance > 0 && (
                      <div className="stop-distance">
                        📍 {stop.segment_distance_formatted} from previous stop
                      </div>
                    )}
                  </div>
                </div>
              ))}
              {/* Total distance summary */}
              <div className="distance-summary">
                <strong>Total Distance:</strong> {route.distance_formatted}
              </div>
            </div>
          </div>

          {/* Map - right side */}
          <div className="map-wrapper">
            {loading && <div className="map-loading">Loading route...</div>}
            <MapContainer
              center={locations[0] || [37.96, 13.58]}
            zoom={10}
            style={{ height: '100%', width: '100%' }}
            ref={mapRef}
          >
            <TileLayer
              attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            
            <FitBounds locations={locations} />

            {/* Draw route line */}
            {pathCoordinates.length > 1 && (
              <>
                <Polyline 
                  positions={pathCoordinates} 
                  color="#667eea" 
                  weight={4}
                  opacity={0.6}
                />
                
                {/* Add connector lines from actual locations to OSRM snap points */}
                {locations.length > 0 && pathCoordinates.length > 0 && (
                  <>
                    {/* Start connector: first location to first OSRM point */}
                    <Polyline
                      positions={[locations[0], pathCoordinates[0]]}
                      color="#667eea"
                      weight={2}
                      opacity={0.4}
                      dashArray="5, 5"
                    />
                    {/* End connector: last location to last OSRM point */}
                    <Polyline
                      positions={[locations[locations.length - 1], pathCoordinates[pathCoordinates.length - 1]]}
                      color="#667eea"
                      weight={2}
                      opacity={0.4}
                      dashArray="5, 5"
                    />
                  </>
                )}
              </>
            )}

            {/* Segment badges - numbered circles on route segments */}
            {!loading && pathCoordinates.length > 1 && locations.length > 1 && locations.map((loc, index) => {
              if (index === 0) return null; // Skip first location
              
              // Find the closest point on the OSRM path to this segment
              // We need to find where in pathCoordinates is the transition from previous stop to current stop
              const prevLoc = locations[index - 1];
              
              // Find indices in pathCoordinates that are closest to prevLoc and loc
              let startIdx = 0;
              let endIdx = pathCoordinates.length - 1;
              let minDistStart = Infinity;
              let minDistEnd = Infinity;
              
              pathCoordinates.forEach((coord, i) => {
                const distToPrev = Math.sqrt(Math.pow(coord[0] - prevLoc[0], 2) + Math.pow(coord[1] - prevLoc[1], 2));
                const distToCurr = Math.sqrt(Math.pow(coord[0] - loc[0], 2) + Math.pow(coord[1] - loc[1], 2));
                
                if (distToPrev < minDistStart) {
                  minDistStart = distToPrev;
                  startIdx = i;
                }
                if (distToCurr < minDistEnd) {
                  minDistEnd = distToCurr;
                  endIdx = i;
                }
              });
              
              // Get the midpoint index of the segment in pathCoordinates
              const midIdx = Math.floor((startIdx + endIdx) / 2);
              const badgePosition = pathCoordinates[midIdx] || [loc[0], loc[1]];
              
              return (
                <CircleMarker
                  key={`segment-${index}`}
                  center={badgePosition}
                  radius={12}
                  pathOptions={{
                    fillColor: '#FF6B6B',
                    fillOpacity: 0.9,
                    color: 'white',
                    weight: 2
                  }}
                >
                  <Tooltip permanent direction="center" className="segment-tooltip">
                    <span className="segment-number">{index}</span>
                  </Tooltip>
                </CircleMarker>
              );
            })}

            {/* All markers */}
            {route.route.map((stop, index) => {
              const coords = stop.location_info?.location;
              if (!coords || !Array.isArray(coords) || coords.length !== 2) {
                console.warn(`Stop ${index} has invalid location:`, stop);
                return null;
              }
              
              const isDepot = stop.location_info?.type === 'depot';
              
              // Check if this is a return depot (same location as first depot but at the end)
              const isReturnDepot = isDepot && index > 0 && index === route.route.length - 1;
              
              // Offset return depot marker slightly to avoid overlap
              let markerCoords = coords;
              if (isReturnDepot) {
                // Check if start and end depot are at same location
                const startDepot = route.route[0].location_info?.location;
                const sameLocation = startDepot && 
                  Math.abs(coords[0] - startDepot[0]) < 0.0001 && 
                  Math.abs(coords[1] - startDepot[1]) < 0.0001;
                
                if (sameLocation) {
                  // Offset by ~0.002 degrees (about 200 meters) for better visibility
                  markerCoords = [coords[0] + 0.002, coords[1] + 0.002];
                }
              }
              
              return (
                <Marker 
                  key={`stop-${index}`} 
                  position={markerCoords}
                  icon={isReturnDepot ? depotReturnIcon : (isDepot ? depotIcon : customerIcon)}
                >
                  <Popup>
                    <div className="popup-content">
                      {isDepot ? (
                        <>
                          <h3>🏭 Depot</h3>
                          <p><strong>Time:</strong> {stop.time_formatted}</p>
                          <p><strong>Load:</strong> {stop.load_before} → {stop.load_after} units</p>
                        </>
                      ) : (
                        <>
                          <h3>📦 Stop #{index}</h3>
                          <p><strong>Customer:</strong> {stop.location_info?.customer_name || stop.location_info?.customer_id || 'Unknown'}</p>
                          <p><strong>Arrival:</strong> {stop.time_formatted}</p>
                          <p><strong>Time Window:</strong> {stop.time_window_formatted}</p>
                          <p><strong>Delivery:</strong> {stop.load_before - stop.load_after} units</p>
                          <p><strong>Remaining Load:</strong> {stop.load_after} units</p>
                        </>
                      )}
                    </div>
                  </Popup>
                </Marker>
              );
            })}
          </MapContainer>
        </div>
        </div> {/* close map-content-wrapper */}
      </div>
    </div>
  );
}

export default RouteMap;
