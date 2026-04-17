import React from 'react';
import './Timeline.css';

function Timeline({ routes }) {
  const timeInfo = React.useMemo(() => {
    if (!routes || routes.length === 0) return { minTime: 0, maxTime: 0, duration: 0 };

    let minTime = Infinity;
    let maxTime = -Infinity;

    routes.forEach(routeData => {
      // SỬ DỤNG MẢNG SEGMENTS TỪ BACKEND
      const segments = routeData.segments || [];
      segments.forEach(s => {
        minTime = Math.min(minTime, s.start_time);
        maxTime = Math.max(maxTime, s.end_time);
      });
    });

    if (minTime === Infinity) return { minTime: 0, maxTime: 0, duration: 0 };

    const ROUND_MINS = 15;
    minTime = Math.floor(minTime / ROUND_MINS) * ROUND_MINS;
    maxTime = Math.ceil(maxTime / ROUND_MINS) * ROUND_MINS + 15;

    return { minTime, maxTime, duration: maxTime - minTime };
  }, [routes]);

  const timeMarkers = React.useMemo(() => {
    if (timeInfo.duration === 0) return [];
    const markers = [];
    let step = timeInfo.duration <= 120 ? 15 : (timeInfo.duration <= 360 ? 30 : 60);

    for (let m = timeInfo.minTime; m <= timeInfo.maxTime; m += step) {
      markers.push({
        position: ((m - timeInfo.minTime) / timeInfo.duration) * 100,
        label: `${Math.floor(m / 60).toString().padStart(2, '0')}:${(m % 60).toString().padStart(2, '0')}`
      });
    }
    return markers;
  }, [timeInfo]);

  return (
    <div className="timeline">
      <h3>Timeline</h3>
      <div className="time-axis">
        {timeMarkers.map((marker, idx) => (
          <div key={idx} className="time-marker" style={{ left: `${marker.position}%` }}>
            <div className="marker-line"></div>
            <div className="marker-label">{marker.label}</div>
          </div>
        ))}
      </div>

      <div className="timeline-content">
        {routes.map((routeData, routeIdx) => (
          <div key={routeIdx} className="vehicle-timeline-row">
            <div className="vehicle-header">
              <strong>{routeData.actual_vehicle_id || `Vehicle ${routeData.vehicle_id + 1}`}</strong>
              <span className="vehicle-stats">
                {routeData.num_customers} stops • {routeData.distance_formatted}
              </span>
            </div>

            <div className="timeline-bar-container">
              {/* VẼ TRỰC TIẾP TỪ SEGMENTS - KHÔNG TỰ TÍNH TOÁN LẠI */}
              {(routeData.segments || []).map((seg, segIdx) => {
                const left = ((seg.start_time - timeInfo.minTime) / timeInfo.duration) * 100;
                const width = ((seg.end_time - seg.start_time) / timeInfo.duration) * 100;

                if (width <= 0) return null;

                return (
                  <div
                    key={segIdx}
                    className={`timeline-segment ${seg.type}`}
                    style={{ left: `${left}%`, width: `${width}%` }}
                    title={`${seg.type}: ${Math.round(seg.duration_minutes)} min`}
                  >
                    <div className="segment-info">
                      {seg.type === 'travel' && (
                        <>
                          <span>{seg.distance_km?.toFixed(1)} km</span>
                          <span>{Math.round(seg.duration_minutes)} min</span>
                          <span>→ {seg.to_location === 0 ? (routeData.route[0].location_info.customer_name || 'Nhà máy') : (routeData.route.find(r => r.location === seg.to_location)?.location_info.customer_name || `Client ${seg.to_location}`)}</span>
                        </>
                      )}
                      {seg.type === 'service' && (
                        <>
                          <span>{Math.round(seg.duration_minutes)} min service</span>
                          {/*<span>{routeData.route.find(r => r.location === seg.location)?.location_info?.customer_name || `Client ${seg.location}`}</span>
                          <span>Client {seg.location}</span>*/}
                        </>
                      )}
                      {seg.type === 'wait' && <span>Wait {Math.round(seg.duration_minutes)}m</span>}
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default Timeline;