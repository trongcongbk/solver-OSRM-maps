import React from 'react';
import './RoutesList.css';

function RoutesList({routes, onRouteClick}) {
    if (!routes || routes.length === 0) {
        return null;
    }

    return (
        <div className="routes-list">
            <h2>🗺️ Routes ({routes.length})</h2>
            <div className="routes-grid">
                {routes.map((route, index) => (
                    <div
                        key={index}
                        className="route-card"
                        onClick={() => onRouteClick(route)}
                    >
                        <div className="route-header">
                            <h3>Vehicle {route.vehicle_id + 1}</h3>
                            <span className="route-badge">Trip {index + 1}</span>
                        </div>

                        <div className="route-stats">
                            <div className="route-stat">
                                <span className="stat-icon">📍</span>
                                <span className="stat-text">{route.distance_formatted}</span>
                            </div>
                            <div className="route-stat">
                                <span className="stat-icon">📦</span>
                                <span className="stat-text">{route.load_formatted}</span>
                            </div>
                            <div className="route-stat">
                                <span className="stat-icon">⏱️</span>
                                <span className="stat-text">{route.duration_formatted}</span>
                            </div>
                            <div className="route-stat">
                                <span className="stat-icon">🎯</span>
                                <span className="stat-text">{route.num_customers} stops</span>
                            </div>
                        </div>

                        <div className="route-details">
                            <div className="detail-row">
                                <span>Saturation:</span>
                                <span className="detail-value">{route.saturation_pct?.toFixed(1)}%</span>
                            </div>
                            <div className="detail-row">
                                <span>Travel Time:</span>
                                <span className="detail-value">{route.travel_time_formatted}</span>
                            </div>
                            <div className="detail-row">
                                <span>Service Time:</span>
                                <span className="detail-value">{route.service_time_formatted}</span>
                            </div>
                        </div>

                        <div className="route-timeline">
                            <div className="timeline-point">
                                <span className="timeline-icon">🏭</span>
                                <span className="timeline-time">{route.route[0]?.time_formatted}</span>
                            </div>
                            <div className="timeline-arrow">→</div>
                            <div className="timeline-point">
                                <span className="timeline-icon">🏁</span>
                                <span
                                    className="timeline-time">{route.route[route.route.length - 1]?.time_formatted}</span>
                            </div>
                        </div>

                        <button className="view-map-btn">
                            🗺️ View on Map
                        </button>
                    </div>
                ))}
            </div>
        </div>
    );
}

export default RoutesList;
