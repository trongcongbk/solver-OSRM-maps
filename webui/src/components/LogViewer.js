import React from 'react';
import './LogViewer.css';

function LogViewer({ logs, isVisible }) {
  if (!isVisible) return null;

  return (
    <div className="log-viewer">
      <div className="log-header">
        <h3>📊 Progress Logs</h3>
      </div>
      <div className="log-content">
        {logs.length === 0 ? (
          <div className="log-empty">Waiting for logs...</div>
        ) : (
          logs.map((log, index) => (
            <div key={index} className="log-entry">
              {log}
            </div>
          ))
        )}
      </div>
    </div>
  );
}

export default LogViewer;
