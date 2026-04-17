import React from 'react';
import './SolutionSummary.css';

function SolutionSummary({ solution }) {
  if (!solution || solution.status !== 'success') {
    return null;
  }

  const stats = [
    {
      label: 'Vehicles Used',
      value: `${solution.num_vehicles_used} / ${solution.total_vehicles_available}`,
      icon: '🚚'
    },
    {
      label: 'Total Trips',
      value: solution.total_trips,
      icon: '🔄'
    },
    {
      label: 'Total Distance',
      value: solution.total_distance_formatted,
      icon: '📍'
    },
    {
      label: 'Avg Distance/Vehicle',
      value: solution.avg_distance_per_vehicle_formatted,
      icon: '📏'
    },
    {
      label: 'Vehicle Saturation',
      value: `${solution.average_saturation_pct}%`,
      icon: '📦'
    },
    {
      label: 'Total Duration',
      value: solution.total_duration_formatted,
      icon: '⏱️'
    },
    {
      label: 'Travel Time (avg)',
      value: solution.avg_travel_time_per_vehicle_formatted,
      icon: '🚗'
    },
    {
      label: 'Service Time (avg)',
      value: solution.avg_service_time_per_vehicle_formatted,
      icon: '📤'
    }
  ];

  return (
    <div className="solution-summary">
      <h2>📊 Solution Summary</h2>
      <div className="stats-grid">
        {stats.map((stat, index) => (
          <div key={index} className="stat-card">
            <div className="stat-icon">{stat.icon}</div>
            <div className="stat-content">
              <div className="stat-label">{stat.label}</div>
              <div className="stat-value">{stat.value}</div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default SolutionSummary;
