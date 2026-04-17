import React, { useRef } from 'react';
import * as XLSX from 'xlsx'; // Import thư viện xử lý Excel
import './FileUpload.css';

function FileUpload({ onFileLoad }) {
  // Chỉ cần 2 Ref này là đủ
  const jsonInputRef = useRef(null);
  const excelInputRef = useRef(null);

  // ================= XỬ LÝ FILE JSON =================
  const handleJsonChange = (event) => {
    const file = event.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const json = JSON.parse(e.target.result);
        onFileLoad(json);
      } catch (error) {
        alert('Invalid JSON file: ' + error.message);
      }
    };
    reader.readAsText(file);
    event.target.value = '';
  };

// ================= XỬ LÝ FILE EXCEL =================
  const handleExcelChange = (event) => {
    const file = event.target.files[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const data = new Uint8Array(e.target.result);
        const workbook = XLSX.read(data, { type: 'array' });

        const depotSheet = workbook.Sheets['Depot'];
        const vehiclesSheet = workbook.Sheets['Vehicles'];
        const customersSheet = workbook.Sheets['Customers'];

        if (!depotSheet || !vehiclesSheet || !customersSheet) {
          throw new Error('File Excel phải có đủ 3 sheet tên là: Depot, Vehicles, Customers');
        }

        // 1. Đọc dữ liệu phẳng từ Excel
        const rawDepot = XLSX.utils.sheet_to_json(depotSheet)[0] || {};
        const rawVehicles = XLSX.utils.sheet_to_json(vehiclesSheet);
        const rawCustomers = XLSX.utils.sheet_to_json(customersSheet);

        // 2. MAPPING DỮ LIỆU: Ép kiểu phẳng của Excel thành kiểu Lồng (Nested) của JSON

        // Tạo Depot
        const formattedDepot = {
          location: [parseFloat(rawDepot.lat), parseFloat(rawDepot.lon)],
          name: rawDepot.name || "Tổng kho"
        };
        console.log("Depot Name bốc được từ Excel:", formattedDepot.name);

        // Tạo Vehicles
        const formattedVehicles = rawVehicles.map(v => ({
          id: String(v.id), // Ép kiểu string như "TAV01"
          type: v.type || "Xe_Tai",
          capacity_units: parseInt(v.capacity_units) || 0,
          start_location: [parseFloat(v.lat), parseFloat(v.lon)],
          time_window: {
            start_min: parseInt(v.start_min) || 0,
            end_min: parseInt(v.end_min) || 1440
          }
        }));

        // Tạo Customers
        const formattedCustomers = rawCustomers.map(c => ({
          id: c.id, // Có thể là số 1001 hoặc string
          name: c.name || `Khách_${c.id}`,
          location: [parseFloat(c.lat), parseFloat(c.lon)],
          demand_units: parseInt(c.demand_units) || 0,
          time_window: {
            start_min: parseInt(c.start_min) || 0,
            end_min: parseInt(c.end_min) || 1440
          },
          // MỚI THÊM: Đọc cột service_time_min, nếu Excel để trống thì mặc định là 15
          service_time_min: c.service_time_min !== undefined ? parseInt(c.service_time_min) : 5
        }));

        // 3. Đóng gói Payload giống hệt cấu trúc JSON gốc
        const payload = {
          depot: formattedDepot,
          vehicles: formattedVehicles,
          customers: formattedCustomers
        };

        // Gửi lên Component cha
        onFileLoad(payload);

      } catch (error) {
        alert('Lỗi đọc file Excel: ' + error.message);
      }
    };
    reader.readAsArrayBuffer(file);
    event.target.value = '';
  };

  // ================= GỌI NÚT BẤM =================
  const handleDownloadExamples = async () => {
    try {
      const response = await fetch('http://localhost:8000/download-examples');
      if (!response.ok) {
        throw new Error('Failed to download examples');
      }

      const blob = await response.blob();
      const url = window.URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = 'cvrptw_examples.zip';
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      window.URL.revokeObjectURL(url);
    } catch (error) {
      alert('Error downloading examples: ' + error.message);
    }
  };

  return (
    <div className="file-upload">
      {/* 1. Thẻ Input ẩn dành cho JSON */}
      <input
        ref={jsonInputRef}
        type="file"
        accept=".json"
        onChange={handleJsonChange}
        style={{ display: 'none' }}
      />

      {/* 2. Thẻ Input ẩn dành cho EXCEL (Phần bạn bị thiếu) */}
      <input
        ref={excelInputRef}
        type="file"
        accept=".xlsx, .xls"
        onChange={handleExcelChange}
        style={{ display: 'none' }}
      />

      {/* Bọc các nút vào container để CSS nhận diện hiển thị nằm ngang */}
      <div className="upload-buttons-container">
        <button className="upload-button" onClick={() => jsonInputRef.current?.click()}>
          📁 Load JSON File
        </button>

        <button className="upload-button excel-btn" onClick={() => excelInputRef.current?.click()}>
          📊 Load Excel
        </button>

        <button className="download-example" onClick={handleDownloadExamples}>
          📥 Download Examples
        </button>
      </div>

      <p className="upload-hint">Upload a problem file (JSON or Excel format) with customers, vehicles, and time windows</p>
    </div>
  );
}

export default FileUpload;