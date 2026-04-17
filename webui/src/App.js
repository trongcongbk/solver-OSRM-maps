import React, { useState, useRef, useEffect } from 'react';
import axios from 'axios';
import FileUpload from './components/FileUpload';
import SolutionSummary from './components/SolutionSummary';
import RoutesList from './components/RoutesList';
import RouteMap from './components/RouteMap';
import LogViewer from './components/LogViewer';
import SelectionModal from './components/SelectionModal';
import Timeline from './components/Timeline';
import './App.css';

function App() {
  const [jsonData, setJsonData] = useState(null);
  const [originalData, setOriginalData] = useState(null);
  const [selectedVehicleIndices, setSelectedVehicleIndices] = useState(null);
  const [selectedCustomerIndices, setSelectedCustomerIndices] = useState(null);
  const [showSelectionModal, setShowSelectionModal] = useState(false);
  const [solution, setSolution] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedRoute, setSelectedRoute] = useState(null);
  const [logs, setLogs] = useState([]);
  const [apiUrl] = useState('http://localhost:8000');
  const [timeLimit, setTimeLimit] = useState(60);
  const [solver, setSolver] = useState('ortools');
  const [vehiclePenaltyWeight, setVehiclePenaltyWeight] = useState(null); // null = use default
  const [distanceWeight, setDistanceWeight] = useState(1.0);
  const [mipGap, setMipGap] = useState(0.01);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const eventSourceRef = useRef(null);
  const logContentRef = useRef(null);

  // Auto-scroll logs to bottom
  useEffect(() => {
    if (logContentRef.current) {
      const logContent = logContentRef.current.querySelector('.log-content');
      if (logContent) {
        logContent.scrollTop = logContent.scrollHeight;
      }
    }
  }, [logs]);

  const handleFileLoad = (data) => {
    setOriginalData(data);
    setSelectedVehicleIndices(null); // Reset selections on new file
    setSelectedCustomerIndices(null);
    setShowSelectionModal(true);
    setSolution(null);
    setError(null);
    setSelectedRoute(null);
    setLogs([]);
  };

  const handleSelectionConfirm = (filteredData, vehicleIndices, customerIndices) => {
    setJsonData(filteredData);
    setSelectedVehicleIndices(vehicleIndices);
    setSelectedCustomerIndices(customerIndices);
    setShowSelectionModal(false);
  };

  const handleSelectionClose = () => {
    setShowSelectionModal(false);
    // Don't reset originalData - keep it for re-selection
  };

  const handleSolve = async () => {
    if (!jsonData) {
      setError('Please load a JSON file first');
      return;
    }

    setLoading(true);
    setError(null);
    setSolution(null);
    setLogs([]);

    try {
      // Extract date from JSON data
      const date = jsonData.metadata?.date || jsonData.date || '2025-11-07';

      // Close existing EventSource if any
      if (eventSourceRef.current) {
        eventSourceRef.current.close();
      }

      // Use EventSource for SSE
      const params = new URLSearchParams({
        time_limit: timeLimit,
        solver: solver,
        distance_weight: distanceWeight,
        mip_gap: mipGap
      });
      
      // Add vehicle_penalty_weight only if user has set it (not null)
      if (vehiclePenaltyWeight !== null) {
        params.append('vehicle_penalty_weight', vehiclePenaltyWeight);
      }
      
      const url = `${apiUrl}/solve-stream?${params.toString()}`;
      
      // EventSource doesn't support POST, so we use fetch with streaming
      const response = await fetch(url, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(jsonData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            const data = JSON.parse(line.slice(6));
            
            if (data.type === 'log') {
              setLogs(prev => [...prev, data.message]);
            } else if (data.type === 'result') {
              setSolution(data.data);
            } else if (data.type === 'error') {
              setError(data.message);
            }
          }
        }
      }

    } catch (err) {
      setError(err.message || 'Failed to solve');
      console.error('Error solving:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleRouteClick = (route) => {
    setSelectedRoute(route);
  };

  const handleCloseMap = () => {
    setSelectedRoute(null);
  };

  return (
    <div className="app">
      <header className="app-header">
        <h1>🚚 Fleet Route Optimizer</h1>
        <p>Capacitated Vehicle Routing with Time Windows • Real-world Routing • Traffic-aware Optimization</p>
      </header>

      <main className="app-main">
        <div className="control-panel">
          <FileUpload onFileLoad={handleFileLoad} />
          
          {jsonData && (
            <div className="selection-summary">
              <h3>📋 Current Selection</h3>
              <div className="summary-stats">
                <div className="stat-item">
                  <span className="stat-icon">🚚</span>
                  <span className="stat-value">{jsonData.vehicles?.length || 0}</span>
                  <span className="stat-label">Vehicles</span>
                </div>
                <div className="stat-item">
                  <span className="stat-icon">📦</span>
                  <span className="stat-value">{jsonData.customers?.length || 0}</span>
                  <span className="stat-label">Customers</span>
                </div>
              </div>
              <button 
                className="btn-reselect" 
                onClick={() => setShowSelectionModal(true)}
                disabled={loading || !originalData}
              >
                🔄 Change Selection
              </button>
            </div>
          )}
          
          {jsonData && (
            <div className="file-info">
              <h3>✓ File loaded</h3>
              <p>Customers: {jsonData.customers?.length || 0}</p>
              <p>Fleet: {jsonData.vehicles?.length || jsonData.fleet?.vehicles?.length || 0} vehicles</p>
            </div>
          )}

          <div className="time-limit-input">
            <label htmlFor="timeLimit">Time Limit (seconds):</label>
            <input
              id="timeLimit"
              type="number"
              min="1"
              max="3600"
              value={timeLimit}
              onChange={(e) => setTimeLimit(parseInt(e.target.value) || 60)}
              disabled={loading}
            />
          </div>

          <div className="solver-selector">
            <label>Solver:</label>
            <div className="solver-options">
              <label className="radio-option">
                <input
                  type="radio"
                  name="solver"
                  value="ortools"
                  checked={solver === 'ortools'}
                  onChange={(e) => setSolver(e.target.value)}
                  disabled={loading}
                />
                <span>OR-Tools</span>
              </label>
              <label className="radio-option">
                <input
                  type="radio"
                  name="solver"
                  value="gurobi"
                  checked={solver === 'gurobi'}
                  onChange={(e) => setSolver(e.target.value)}
                  disabled={loading}
                />
                <span>Gurobi</span>
              </label>
            </div>
          </div>

          <div className="advanced-params">
            <button 
              className="toggle-advanced"
              onClick={() => setShowAdvanced(!showAdvanced)}
              type="button"
            >
              {showAdvanced ? '▼' : '▶'} Advanced Parameters
            </button>
            
            {showAdvanced && (
              <div className="advanced-inputs">
                <div className="param-group">
                  <label htmlFor="vehiclePenalty">
                    Vehicle Penalty Weight:
                    <span className="param-hint">
                      Higher = fewer vehicles (default: {solver === 'ortools' ? '100000' : '1000'})
                    </span>
                  </label>
                  <input
                    id="vehiclePenalty"
                    type="number"
                    min="0"
                    step="1000"
                    placeholder={solver === 'ortools' ? '100000' : '1000'}
                    value={vehiclePenaltyWeight || ''}
                    onChange={(e) => setVehiclePenaltyWeight(e.target.value ? parseFloat(e.target.value) : null)}
                    disabled={loading}
                  />
                </div>

                <div className="param-group">
                  <label htmlFor="distanceWeight">
                    Distance Weight:
                    <span className="param-hint">
                      Higher = shorter routes (default: 1.0)
                    </span>
                  </label>
                  <input
                    id="distanceWeight"
                    type="number"
                    min="0.1"
                    max="10"
                    step="0.1"
                    value={distanceWeight}
                    onChange={(e) => setDistanceWeight(parseFloat(e.target.value) || 1.0)}
                    disabled={loading}
                  />
                </div>

                {solver === 'gurobi' && (
                  <div className="param-group">
                    <label htmlFor="mipGap">
                      MIP Gap (Gurobi only):
                      <span className="param-hint">
                        Optimality gap, 0.01 = 1% (lower = better solution, slower)
                      </span>
                    </label>
                    <input
                      id="mipGap"
                      type="number"
                      min="0.001"
                      max="0.5"
                      step="0.01"
                      value={mipGap}
                      onChange={(e) => setMipGap(parseFloat(e.target.value) || 0.01)}
                      disabled={loading}
                    />
                  </div>
                )}
              </div>
            )}
          </div>

          <button
            className="solve-button"
            onClick={handleSolve}
            disabled={!jsonData || loading}
          >
            {loading ? '⏳ Solving...' : '🚀 Solve'}
          </button>

          {error && (
            <div className="error-message">
              <strong>Error:</strong> {error}
            </div>
          )}
        </div>

        {(loading || logs.length > 0) && (
          <div ref={logContentRef}>
            <LogViewer logs={logs} isVisible={true} />
          </div>
        )}

        {solution && (
          <div className="results-panel">
            <SolutionSummary solution={solution} />
            <Timeline routes={solution.routes} />
            <RoutesList 
              routes={solution.routes} 
              onRouteClick={handleRouteClick}
            />
          </div>
        )}

        {selectedRoute && (
          <RouteMap 
            route={selectedRoute} 
            onClose={handleCloseMap}
          />
        )}

        {showSelectionModal && originalData && (
          <SelectionModal
            data={originalData}
            initialVehicleIndices={selectedVehicleIndices}
            initialCustomerIndices={selectedCustomerIndices}
            onConfirm={handleSelectionConfirm}
            onClose={handleSelectionClose}
          />
        )}
      </main>
    </div>
  );
}

export default App;
