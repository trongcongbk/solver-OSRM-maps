import React, { useState, useEffect } from 'react';
import './SelectionModal.css';

function SelectionModal({ data, initialVehicleIndices, initialCustomerIndices, onConfirm, onClose }) {
  const [selectedVehicles, setSelectedVehicles] = useState(new Set());
  const [selectedCustomers, setSelectedCustomers] = useState(new Set());
  const [selectAllVehicles, setSelectAllVehicles] = useState(true);
  const [selectAllCustomers, setSelectAllCustomers] = useState(true);

  // Initialize with provided selections or all selected
  useEffect(() => {
    if (data) {
      // If we have initial indices, use those; otherwise select all
      if (initialVehicleIndices && initialVehicleIndices.length > 0) {
        setSelectedVehicles(new Set(initialVehicleIndices));
        setSelectAllVehicles(initialVehicleIndices.length === data.vehicles.length);
      } else {
        setSelectedVehicles(new Set(data.vehicles.map((_, idx) => idx)));
        setSelectAllVehicles(true);
      }
      
      if (initialCustomerIndices && initialCustomerIndices.length > 0) {
        setSelectedCustomers(new Set(initialCustomerIndices));
        setSelectAllCustomers(initialCustomerIndices.length === data.customers.length);
      } else {
        setSelectedCustomers(new Set(data.customers.map((_, idx) => idx)));
        setSelectAllCustomers(true);
      }
    }
  }, [data, initialVehicleIndices, initialCustomerIndices]);

  const toggleVehicle = (index) => {
    const newSet = new Set(selectedVehicles);
    if (newSet.has(index)) {
      newSet.delete(index);
    } else {
      newSet.add(index);
    }
    setSelectedVehicles(newSet);
    setSelectAllVehicles(newSet.size === data.vehicles.length);
  };

  const toggleCustomer = (index) => {
    const newSet = new Set(selectedCustomers);
    if (newSet.has(index)) {
      newSet.delete(index);
    } else {
      newSet.add(index);
    }
    setSelectedCustomers(newSet);
    setSelectAllCustomers(newSet.size === data.customers.length);
  };

  const toggleAllVehicles = () => {
    if (selectAllVehicles) {
      setSelectedVehicles(new Set());
      setSelectAllVehicles(false);
    } else {
      setSelectedVehicles(new Set(data.vehicles.map((_, idx) => idx)));
      setSelectAllVehicles(true);
    }
  };

  const toggleAllCustomers = () => {
    if (selectAllCustomers) {
      setSelectedCustomers(new Set());
      setSelectAllCustomers(false);
    } else {
      setSelectedCustomers(new Set(data.customers.map((_, idx) => idx)));
      setSelectAllCustomers(true);
    }
  };

  const handleConfirm = () => {
    const filteredData = {
      ...data,
      vehicles: data.vehicles.filter((_, idx) => selectedVehicles.has(idx)),
      customers: data.customers.filter((_, idx) => selectedCustomers.has(idx))
    };
    
    // Pass the selected indices back to parent
    const vehicleIndices = Array.from(selectedVehicles);
    const customerIndices = Array.from(selectedCustomers);
    
    onConfirm(filteredData, vehicleIndices, customerIndices);
  };

  if (!data) return null;

  return (
    <div className="selection-modal-overlay">
      <div className="selection-modal">
        <div className="modal-header">
          <h2>📋 Select Vehicles and Customers</h2>
          <button className="close-btn" onClick={onClose}>✕</button>
        </div>

        <div className="modal-body">
          <div className="selection-section">
            <div className="section-header">
              <h3>🚚 Vehicles ({selectedVehicles.size}/{data.vehicles.length})</h3>
              <label className="select-all">
                <input
                  type="checkbox"
                  checked={selectAllVehicles}
                  onChange={toggleAllVehicles}
                />
                Select All
              </label>
            </div>
            <div className="items-list">
              {data.vehicles.map((vehicle, index) => (
                <div key={index} className="item-card">
                  <label className="item-checkbox">
                    <input
                      type="checkbox"
                      checked={selectedVehicles.has(index)}
                      onChange={() => toggleVehicle(index)}
                    />
                    <div className="item-details">
                      <div className="item-title">{vehicle.id || `Vehicle ${index + 1}`}</div>
                      <div className="item-info">
                        <span>Type: {vehicle.type}</span>
                        <span>Capacity: {vehicle.capacity_units} units</span>
                      </div>
                      <div className="item-info">
                        <span>Time: {Math.floor(vehicle.time_window.start_min / 60)}:{(vehicle.time_window.start_min % 60).toString().padStart(2, '0')} - {Math.floor(vehicle.time_window.end_min / 60)}:{(vehicle.time_window.end_min % 60).toString().padStart(2, '0')}</span>
                      </div>
                    </div>
                  </label>
                </div>
              ))}
            </div>
          </div>

          <div className="selection-section">
            <div className="section-header">
              <h3>📦 Customers ({selectedCustomers.size}/{data.customers.length})</h3>
              <label className="select-all">
                <input
                  type="checkbox"
                  checked={selectAllCustomers}
                  onChange={toggleAllCustomers}
                />
                Select All
              </label>
            </div>
            <div className="items-list">
              {data.customers.map((customer, index) => (
                <div key={index} className="item-card">
                  <label className="item-checkbox">
                    <input
                      type="checkbox"
                      checked={selectedCustomers.has(index)}
                      onChange={() => toggleCustomer(index)}
                    />
                    <div className="item-details">
                      <div className="item-title">{customer.name || customer.id || `Customer ${index + 1}`}</div>
                      <div className="item-info">
                        <span>ID: {customer.id}</span>
                        <span>Demand: {customer.demand_units} units</span>
                      </div>
                      <div className="item-info">
                        <span>Location: [{customer.location[0].toFixed(4)}, {customer.location[1].toFixed(4)}]</span>
                      </div>
                      <div className="item-info">
                        <span>Time: {Math.floor(customer.time_window.start_min / 60)}:{(customer.time_window.start_min % 60).toString().padStart(2, '0')} - {Math.floor(customer.time_window.end_min / 60)}:{(customer.time_window.end_min % 60).toString().padStart(2, '0')}</span>
                      </div>
                    </div>
                  </label>
                </div>
              ))}
            </div>
          </div>
        </div>

        <div className="modal-footer">
          <button className="btn-cancel" onClick={onClose}>Cancel</button>
          <button 
            className="btn-confirm" 
            onClick={handleConfirm}
            disabled={selectedVehicles.size === 0 || selectedCustomers.size === 0}
          >
            Confirm Selection
          </button>
        </div>
      </div>
    </div>
  );
}

export default SelectionModal;
