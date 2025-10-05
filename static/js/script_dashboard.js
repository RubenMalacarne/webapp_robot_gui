// Monitor configuration - Start empty, charts added dynamically
const monitorConfigs = [];

// Available data field configurations - from Inspector telemetry
const availableFields = {
    // Winch Right telemetry
    'winch_right_rope_force': { label: 'WR Rope Force', unit: 'N', color: '#ff6b6b', category: 'Winch Right' },
    'winch_right_rope_length': { label: 'WR Rope Length', unit: 'm', color: '#ff8585', category: 'Winch Right' },
    'winch_right_rope_velocity': { label: 'WR Rope Velocity', unit: 'm/s', color: '#ff9f9f', category: 'Winch Right' },
    'winch_right_current': { label: 'WR Current', unit: 'A', color: '#ffb8b8', category: 'Winch Right' },
    
    // Winch Left telemetry
    'winch_left_rope_force': { label: 'WL Rope Force', unit: 'N', color: '#4ecdc4', category: 'Winch Left' },
    'winch_left_rope_length': { label: 'WL Rope Length', unit: 'm', color: '#6ed7d0', category: 'Winch Left' },
    'winch_left_rope_velocity': { label: 'WL Rope Velocity', unit: 'm/s', color: '#8ee1db', category: 'Winch Left' },
    'winch_left_current': { label: 'WL Current', unit: 'A', color: '#aeebe6', category: 'Winch Left' },
    
    // Rope IMU
    'rope_imu_angular_velocity_x': { label: 'Rope IMU AngVel X', unit: 'rad/s', color: '#95e1d3', category: 'Rope IMU' },
    'rope_imu_angular_velocity_y': { label: 'Rope IMU AngVel Y', unit: 'rad/s', color: '#a8e6cf', category: 'Rope IMU' },
    'rope_imu_angular_velocity_z': { label: 'Rope IMU AngVel Z', unit: 'rad/s', color: '#bbebdb', category: 'Rope IMU' },
    'rope_imu_rpy_roll': { label: 'Rope IMU Roll', unit: 'rad', color: '#95d5e1', category: 'Rope IMU' },
    'rope_imu_rpy_pitch': { label: 'Rope IMU Pitch', unit: 'rad', color: '#a8dfe6', category: 'Rope IMU' },
    'rope_imu_rpy_yaw': { label: 'Rope IMU Yaw', unit: 'rad', color: '#bbe9eb', category: 'Rope IMU' },
    'rope_imu_acceleration_x': { label: 'Rope IMU Accel X', unit: 'm/s²', color: '#c7f0f2', category: 'Rope IMU' },
    'rope_imu_acceleration_y': { label: 'Rope IMU Accel Y', unit: 'm/s²', color: '#d4f4f5', category: 'Rope IMU' },
    'rope_imu_acceleration_z': { label: 'Rope IMU Accel Z', unit: 'm/s²', color: '#e1f8f9', category: 'Rope IMU' },
    
    // Body IMU
    'body_imu_angular_velocity_x': { label: 'Body IMU AngVel X', unit: 'rad/s', color: '#f38181', category: 'Body IMU' },
    'body_imu_angular_velocity_y': { label: 'Body IMU AngVel Y', unit: 'rad/s', color: '#f59b9b', category: 'Body IMU' },
    'body_imu_angular_velocity_z': { label: 'Body IMU AngVel Z', unit: 'rad/s', color: '#f7b5b5', category: 'Body IMU' },
    'body_imu_rpy_roll': { label: 'Body IMU Roll', unit: 'rad', color: '#aa96da', category: 'Body IMU' },
    'body_imu_rpy_pitch': { label: 'Body IMU Pitch', unit: 'rad', color: '#baa8e0', category: 'Body IMU' },
    'body_imu_rpy_yaw': { label: 'Body IMU Yaw', unit: 'rad', color: '#cabae6', category: 'Body IMU' },
    'body_imu_acceleration_x': { label: 'Body IMU Accel X', unit: 'm/s²', color: '#dacdec', category: 'Body IMU' },
    'body_imu_acceleration_y': { label: 'Body IMU Accel Y', unit: 'm/s²', color: '#e8dff1', category: 'Body IMU' },
    'body_imu_acceleration_z': { label: 'Body IMU Accel Z', unit: 'm/s²', color: '#f6f1f7', category: 'Body IMU' }
};

// Chart counter for dynamic chart creation
let chartCounter = 0;

// Data storage
const maxDataPoints = 1000;
let monitorData = {};
let monitorStats = {};
let charts = {};
let elements = {};
let isPaused = false;

// Storage for all telemetry data from inspector
let telemetryData = {
    winch_right: {},
    winch_left: {},
    rope_imu: {},
    body_imu: {}
};

// Initialize data structures for each monitor
monitorConfigs.forEach(config => {
    monitorData[config.id] = [];
    monitorStats[config.id] = { sum: 0, count: 0, peak: 0 };
});

// Function to create a monitor section
function createMonitorSection(config) {
    return `
    <div class="monitor-section ${config.theme}" data-chart-id="${config.id}">
        <div class="section-title">
        <span class="chart-title-text" id="${config.id}-title">${config.label}</span>
        <button class="rename-btn" onclick="renameChart('${config.id}')" title="Rename chart">✏️</button>
        </div>
        
        <div class="chart-container">
        <canvas id="${config.id}Chart"></canvas>
        </div>

        <div class="status-info">
        <div class="connection-status">
            <div class="status-indicator" id="${config.id}-status"></div>
            <span>${config.label.replace(' Monitor', ' Sensor')}</span>
        </div>
        <span>Samples: <span id="${config.id}-samples">0</span></span>
        </div>
    </div>
    `;
}

// Generate monitor sections dynamically
function initializeMonitors() {
    const container = document.getElementById('monitors-container');
    container.innerHTML = monitorConfigs.map(createMonitorSection).join('');
    
    // Cache DOM elements for each monitor
    monitorConfigs.forEach(config => {
    elements[config.id] = {
        status: document.getElementById(`${config.id}-status`),
        samples: document.getElementById(`${config.id}-samples`)
    };
    });
}

// Data storage - keeping legacy variable names for compatibility
let voltageData = [];
let currentData = [];
let timeLabels = [];

// Statistics tracking - keeping legacy variable names for compatibility  
let voltageStats = { sum: 0, count: 0, peak: 0 };
let currentStats = { sum: 0, count: 0, peak: 0 };

// Chart configuration template
const chartConfig = {
    type: 'line',
    options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    scales: {
        x: {
        type: 'linear',
        position: 'bottom',
        grid: {
            color: 'rgba(255, 255, 255, 0.1)'
        },
        ticks: {
            color: '#888',
            callback: function(value) {
            return value.toFixed(1) + 's';
            }
        }
        },
        y: {
        grid: {
            color: 'rgba(255, 255, 255, 0.1)'
        },
        ticks: {
            color: '#888'
        }
        }
    },
    plugins: {
        legend: {
        display: true,  // Show legend when multiple signals
        position: 'top',
        labels: {
            color: '#ccc',
            font: {
                size: 11
            },
            usePointStyle: true
        }
        }
    },
    elements: {
        point: {
        radius: 0
        },
        line: {
        tension: 0.1
        }
    }
    }
};

// Initialize charts for each monitor
function initializeCharts() {
    monitorConfigs.forEach(config => {
    charts[config.id] = new Chart(document.getElementById(`${config.id}Chart`), {
        ...chartConfig,
        data: {
        datasets: [{
            label: config.label,
            data: [],
            borderColor: config.color,
            backgroundColor: config.backgroundColor,
            borderWidth: 2,
            fill: true
        }]
        }
    });
    });
}

// Legacy chart variables for compatibility
let voltageChart, currentChart;

// Generic update statistics function
function updateStats(value, type) {
    const config = monitorConfigs.find(c => c.id === type);
    if (!config) return;
    
    const stats = monitorStats[type];
    stats.sum += value;
    stats.count++;
    stats.peak = Math.max(stats.peak, Math.abs(value));
    
    if (elements[type] && elements[type].samples) {
        elements[type].samples.textContent = stats.count.toString();
    }
    
    // Update legacy stats for compatibility
    if (type === 'voltage') {
    voltageStats = stats;
    } else if (type === 'current') {
    currentStats = stats;
    }
}

// Add data point
function addDataPoint(timestamp, voltage, current) {
    if (isPaused) return;

    const timeInSeconds = timestamp / 1000;
    
    // Add new data
    voltageData.push({ x: timeInSeconds, y: voltage });
    currentData.push({ x: timeInSeconds, y: current });
    
    // Remove old data if exceeding max points
    if (voltageData.length > maxDataPoints) {
    voltageData.shift();
    currentData.shift();
    }
    
    // Update charts using the new chart structure
    charts.voltage.data.datasets[0].data = voltageData;
    charts.current.data.datasets[0].data = currentData;
    
    charts.voltage.update('none');
    charts.current.update('none');
    
    // Update legacy chart references for compatibility
    voltageChart = charts.voltage;
    currentChart = charts.current;
    
    // Update statistics
    updateStats(voltage, 'voltage');
    updateStats(current, 'current');
    
    // Update connection status indicators
    updateConnectionStatus();
}

// Update connection status
function updateConnectionStatus() {
    monitorConfigs.forEach(config => {
    elements[config.id].status.classList.add('connected');
    });
}

// Initialize everything when page loads
document.addEventListener('DOMContentLoaded', function() {
    initializeMonitors();
    initializeCharts();
    
    // Initialize drag and drop
    initializeDragAndDrop();
    
    // Initialize sidebar controls
    initializeSidebarControls();
    
    // Initialize color pickers for field icons
    initializeColorPickers();
    
    // Update active charts list
    updateActiveChartsList();
});

// Socket.IO Handle
const socket = io();

socket.on('connect', function() {
    console.log('Connected to server');
    updateConnectionStatus();
    // Register this client as dashboard
    socket.emit('register', { node: 'dashboard_gui' });
});

socket.on('disconnect', function() {
    console.log('Disconnected from server');
    monitorConfigs.forEach(config => {
    if (elements[config.id]) {
        elements[config.id].status.classList.remove('connected');
    }
    });
});

// Handle winch telemetry data
socket.on('winch_telemetry', function(data) {
    const timestamp = Date.now();
    
    // Update Winch Right data
    if (data.winch_right) {
        telemetryData.winch_right = data.winch_right;
        updateChartData('winch_right_rope_force', data.winch_right.rope_force, timestamp);
        updateChartData('winch_right_rope_length', data.winch_right.rope_length, timestamp);
        updateChartData('winch_right_rope_velocity', data.winch_right.rope_velocity, timestamp);
        updateChartData('winch_right_current', data.winch_right.current, timestamp);
    }
    
    // Update Winch Left data
    if (data.winch_left) {
        telemetryData.winch_left = data.winch_left;
        updateChartData('winch_left_rope_force', data.winch_left.rope_force, timestamp);
        updateChartData('winch_left_rope_length', data.winch_left.rope_length, timestamp);
        updateChartData('winch_left_rope_velocity', data.winch_left.rope_velocity, timestamp);
        updateChartData('winch_left_current', data.winch_left.current, timestamp);
    }
});

// Handle alpine body telemetry data
socket.on('alpine_body_telemetry', function(data) {
    const timestamp = Date.now();
    
    // Update Rope IMU data
    if (data.rope_imu_angular_velocity) {
        telemetryData.rope_imu.angular_velocity = data.rope_imu_angular_velocity;
        updateChartData('rope_imu_angular_velocity_x', data.rope_imu_angular_velocity.x, timestamp);
        updateChartData('rope_imu_angular_velocity_y', data.rope_imu_angular_velocity.y, timestamp);
        updateChartData('rope_imu_angular_velocity_z', data.rope_imu_angular_velocity.z, timestamp);
    }
    
    if (data.rope_imu_rpy) {
        telemetryData.rope_imu.rpy = data.rope_imu_rpy;
        updateChartData('rope_imu_rpy_roll', data.rope_imu_rpy.x, timestamp);
        updateChartData('rope_imu_rpy_pitch', data.rope_imu_rpy.y, timestamp);
        updateChartData('rope_imu_rpy_yaw', data.rope_imu_rpy.z, timestamp);
    }
    
    if (data.rope_imu_acceleration) {
        telemetryData.rope_imu.acceleration = data.rope_imu_acceleration;
        updateChartData('rope_imu_acceleration_x', data.rope_imu_acceleration.x, timestamp);
        updateChartData('rope_imu_acceleration_y', data.rope_imu_acceleration.y, timestamp);
        updateChartData('rope_imu_acceleration_z', data.rope_imu_acceleration.z, timestamp);
    }
    
    // Update Body IMU data
    if (data.body_imu_angular_velocity) {
        telemetryData.body_imu.angular_velocity = data.body_imu_angular_velocity;
        updateChartData('body_imu_angular_velocity_x', data.body_imu_angular_velocity.x, timestamp);
        updateChartData('body_imu_angular_velocity_y', data.body_imu_angular_velocity.y, timestamp);
        updateChartData('body_imu_angular_velocity_z', data.body_imu_angular_velocity.z, timestamp);
    }
    
    if (data.body_imu_rpy) {
        telemetryData.body_imu.rpy = data.body_imu_rpy;
        updateChartData('body_imu_rpy_roll', data.body_imu_rpy.x, timestamp);
        updateChartData('body_imu_rpy_pitch', data.body_imu_rpy.y, timestamp);
        updateChartData('body_imu_rpy_yaw', data.body_imu_rpy.z, timestamp);
    }
    
    if (data.body_imu_acceleration) {
        telemetryData.body_imu.acceleration = data.body_imu_acceleration;
        updateChartData('body_imu_acceleration_x', data.body_imu_acceleration.x, timestamp);
        updateChartData('body_imu_acceleration_y', data.body_imu_acceleration.y, timestamp);
        updateChartData('body_imu_acceleration_z', data.body_imu_acceleration.z, timestamp);
    }
});

// Function to update chart data for a specific field
function updateChartData(fieldName, value, timestamp) {
    if (isPaused) return;
    
    const timeInSeconds = timestamp / 1000;
    
    // Find charts that have this field assigned
    monitorConfigs.forEach(config => {
        if (config.assignedFields && config.assignedFields.includes(fieldName) && charts[config.id]) {
            // Get or initialize data storage for this chart
            if (!monitorData[config.id]) {
                monitorData[config.id] = {};
            }
            
            if (!monitorData[config.id][fieldName]) {
                monitorData[config.id][fieldName] = [];
            }
            
            // Add new data point
            monitorData[config.id][fieldName].push({ x: timeInSeconds, y: value });
            
            // Remove old data if exceeding max points
            if (monitorData[config.id][fieldName].length > maxDataPoints) {
                monitorData[config.id][fieldName].shift();
            }
            
            // Update the corresponding dataset in the chart
            const datasetIndex = config.assignedFields.indexOf(fieldName);
            if (datasetIndex !== -1 && charts[config.id].data.datasets[datasetIndex]) {
                charts[config.id].data.datasets[datasetIndex].data = monitorData[config.id][fieldName];
                charts[config.id].update('none');
            }
            
            // Update statistics (use first field for stats)
            if (datasetIndex === 0) {
                updateStats(value, config.id);
            }
            
            // Update connection status
            if (elements[config.id] && elements[config.id].status) {
                elements[config.id].status.classList.add('connected');
            }
        }
    });
}

// Control buttons
document.getElementById('pause-btn')?.addEventListener('click', function() {
    isPaused = !isPaused;
    this.textContent = isPaused ? 'Resume' : 'Pause';
    this.classList.toggle('active', isPaused);
});

// Drag and Drop Functionality
function initializeDragAndDrop() {
    const dataFields = document.querySelectorAll('.data-field');
    const monitorSections = document.querySelectorAll('.monitor-section');
    
    // Setup draggable fields
    dataFields.forEach(field => {
        field.addEventListener('dragstart', handleDragStart);
        field.addEventListener('dragend', handleDragEnd);
    });
    
    // Setup drop zones (monitor sections)
    monitorSections.forEach(section => {
        section.addEventListener('dragover', handleDragOver);
        section.addEventListener('drop', handleDrop);
        section.addEventListener('dragleave', handleDragLeave);
        section.addEventListener('dragenter', handleDragEnter);
    });
}

let draggedField = null;

function handleDragStart(e) {
    draggedField = this.dataset.field;
    this.classList.add('dragging');
    e.dataTransfer.effectAllowed = 'copy';
    e.dataTransfer.setData('text/html', this.innerHTML);
}

function handleDragEnd(e) {
    this.classList.remove('dragging');
}

function handleDragOver(e) {
    if (e.preventDefault) {
        e.preventDefault();
    }
    e.dataTransfer.dropEffect = 'copy';
    return false;
}

function handleDragEnter(e) {
    this.classList.add('drop-target');
}

function handleDragLeave(e) {
    if (e.target.classList.contains('monitor-section')) {
        this.classList.remove('drop-target');
    }
}

function handleDrop(e) {
    if (e.stopPropagation) {
        e.stopPropagation();
    }
    
    this.classList.remove('drop-target');
    
    if (draggedField) {
        // Find the monitor ID from the section
        const chartCanvas = this.querySelector('canvas');
        if (chartCanvas) {
            const chartId = chartCanvas.id.replace('Chart', '');
            assignFieldToChart(draggedField, chartId);
        }
    }
    
    return false;
}

function assignFieldToChart(fieldName, chartId) {
    const field = availableFields[fieldName];
    if (!field) return;
    
    console.log(`Assigning ${fieldName} to chart ${chartId}`);
    
    // Update the chart configuration
    if (charts[chartId]) {
        const configIndex = monitorConfigs.findIndex(c => c.id === chartId);
        if (configIndex === -1) return;
        
        const config = monitorConfigs[configIndex];
        
        // Check if chart already has signals assigned
        if (!config.assignedFields) {
            config.assignedFields = [];
        }
        
        // Check if this field is already assigned
        if (config.assignedFields.includes(fieldName)) {
            showNotification(`${field.label} already assigned to this chart!`, 'warning');
            return;
        }
        
        // Check unit compatibility if chart already has signals
        if (config.assignedFields.length > 0) {
            const firstField = availableFields[config.assignedFields[0]];
            if (firstField.unit !== field.unit) {
                showNotification(
                    `❌ Unit mismatch!\n${field.label} (${field.unit}) cannot be added to chart with ${firstField.unit}`,
                    'error'
                );
                return;
            }
        }
        
        // Add field to assigned fields
        config.assignedFields.push(fieldName);
        
        // Update chart unit if first signal
        if (config.assignedFields.length === 1) {
            config.unit = field.unit;
            config.label = `${field.label}`;
        } else {
            // Update label to show multiple signals
            config.label = `Mixed (${field.unit})`;
        }
        
        // Initialize data storage for this field
        if (!monitorData[chartId]) {
            monitorData[chartId] = {};
        }
        monitorData[chartId][fieldName] = [];
        
        // Add new dataset to chart
        charts[chartId].data.datasets.push({
            label: field.label,
            data: [],
            borderColor: field.color,
            backgroundColor: 'transparent',  // No fill under line
            borderWidth: 2,
            fill: false,  // Explicitly no fill
            pointRadius: 0
        });
        
        charts[chartId].update();
        
        // Update title
        const titleElement = document.querySelector(`#${chartId}Chart`)?.closest('.monitor-section')?.querySelector('.chart-title-text');
        if (titleElement && config.assignedFields.length === 1) {
            titleElement.textContent = `${field.label}`;
        }
        
        // Update active charts list
        updateActiveChartsList();
        
        // Show success feedback
        showNotification(`✅ ${field.label} added to chart!`, 'success');
    }
}

function showNotification(message, type = 'success') {
    // Create a simple notification
    const notification = document.createElement('div');
    
    let bgColor = 'rgba(0, 255, 136, 0.9)';
    let textColor = '#000';
    
    if (type === 'warning') {
        bgColor = 'rgba(255, 204, 0, 0.9)';
        textColor = '#000';
    } else if (type === 'error') {
        bgColor = 'rgba(255, 68, 68, 0.9)';
        textColor = '#fff';
    }
    
    notification.style.cssText = `
        position: fixed;
        top: 20px;
        right: 20px;
        background: ${bgColor};
        color: ${textColor};
        padding: 12px 20px;
        border-radius: 6px;
        font-size: 14px;
        z-index: 9999;
        animation: slideIn 0.3s ease;
        white-space: pre-line;
        max-width: 400px;
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
    `;
    notification.textContent = message;
    document.body.appendChild(notification);
    
    const duration = type === 'error' ? 3000 : 2000;
    
    setTimeout(() => {
        notification.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => notification.remove(), 300);
    }, duration);
}

// Sidebar Controls
function initializeSidebarControls() {
    const addChartBtn = document.getElementById('add-chart-btn');
    const removeChartBtn = document.getElementById('remove-chart-btn');
    
    if (addChartBtn) {
        addChartBtn.addEventListener('click', addNewChart);
    }
    
    if (removeChartBtn) {
        removeChartBtn.addEventListener('click', removeLastChart);
    }
}

function addNewChart() {
    chartCounter++;
    const newChartId = `chart${chartCounter}`;
    
    const newConfig = {
        id: newChartId,
        label: `Chart ${chartCounter}`,
        unit: 'units',
        theme: 'voltage-theme',
        color: getRandomColor(),
        backgroundColor: 'rgba(150, 150, 200, 0.1)'
    };
    
    monitorConfigs.push(newConfig);
    monitorData[newChartId] = [];
    monitorStats[newChartId] = { sum: 0, count: 0, peak: 0 };
    
    // Create and append the new monitor section
    const container = document.getElementById('monitors-container');
    const sectionHTML = createMonitorSection(newConfig);
    const tempDiv = document.createElement('div');
    tempDiv.innerHTML = sectionHTML;
    const newSection = tempDiv.firstElementChild;
    container.appendChild(newSection);
    
    // Cache elements
    elements[newChartId] = {
        status: document.getElementById(`${newChartId}-status`),
        samples: document.getElementById(`${newChartId}-samples`)
    };
    
    // Initialize the chart
    charts[newChartId] = new Chart(document.getElementById(`${newChartId}Chart`), {
        ...chartConfig,
        data: {
            datasets: []  // Start with no datasets, they'll be added via drag-drop
        }
    });
    
    // Enable drag and drop for the new section
    const newSectionElement = document.querySelector(`#${newChartId}Chart`).closest('.monitor-section');
    if (newSectionElement) {
        newSectionElement.addEventListener('dragover', handleDragOver);
        newSectionElement.addEventListener('drop', handleDrop);
        newSectionElement.addEventListener('dragleave', handleDragLeave);
        newSectionElement.addEventListener('dragenter', handleDragEnter);
    }
    
    // Update active charts list
    updateActiveChartsList();
    
    showNotification('New chart added!');
}

function removeLastChart() {
    if (monitorConfigs.length === 0) {
        showNotification('No charts to remove!');
        return;
    }
    
    const lastConfig = monitorConfigs[monitorConfigs.length - 1];
    const chartId = lastConfig.id;
    
    // Remove from DOM
    const chartElement = document.getElementById(`${chartId}Chart`);
    if (chartElement) {
        const section = chartElement.closest('.monitor-section');
        if (section) {
            section.remove();
        }
    }
    
    // Destroy chart
    if (charts[chartId]) {
        charts[chartId].destroy();
        delete charts[chartId];
    }
    
    // Clean up data structures
    delete monitorData[chartId];
    delete monitorStats[chartId];
    delete elements[chartId];
    monitorConfigs.pop();
    
    // Update active charts list
    updateActiveChartsList();
    
    showNotification('Chart removed!');
}

function updateActiveChartsList() {
    const listElement = document.getElementById('active-charts-list');
    if (!listElement) return;
    
    if (monitorConfigs.length === 0) {
        listElement.innerHTML = '<p style="color: #888; font-size: 12px; text-align: center; font-style: italic;">No charts yet. Click "Add New Chart" to start!</p>';
        return;
    }
    
    listElement.innerHTML = monitorConfigs.map(config => {
        const fieldsInfo = config.assignedFields && config.assignedFields.length > 0
            ? config.assignedFields.map(fieldKey => {
                const field = availableFields[fieldKey];
                return `<span class="signal-tag" data-chart-id="${config.id}" data-field-key="${fieldKey}" style="background: ${field.color}20; color: ${field.color}; border: 1px solid ${field.color};">
                    ${field.label}
                    <button class="remove-signal-btn" onclick="removeSignalFromChart('${config.id}', '${fieldKey}'); event.stopPropagation();">×</button>
                </span>`;
              }).join('')
            : '<span style="color: #888; font-size: 11px;">No signals</span>';
        
        return `
            <div class="chart-info-item">
                <span class="chart-badge" style="background: ${config.color};"></span>
                <div style="flex: 1;">
                    <div style="font-weight: 500;">${config.label}</div>
                    <div class="signal-tags-container" style="margin-top: 4px; display: flex; flex-wrap: wrap; gap: 4px;">
                        ${fieldsInfo}
                    </div>
                </div>
            </div>
        `;
    }).join('');
}

function getRandomColor() {
    const colors = ['#ff6b6b', '#4ecdc4', '#95e1d3', '#f38181', '#aa96da', '#fcbad3', '#a8e6cf', '#ffd3b6'];
    return colors[Math.floor(Math.random() * colors.length)];
}

// Rename chart function
function renameChart(chartId) {
    const currentLabel = monitorConfigs.find(c => c.id === chartId)?.label || 'Chart';
    const newName = prompt('Enter new chart name:', currentLabel);
    
    if (newName && newName.trim() !== '') {
        const configIndex = monitorConfigs.findIndex(c => c.id === chartId);
        if (configIndex !== -1) {
            monitorConfigs[configIndex].label = newName.trim();
            
            // Update title in DOM
            const titleElement = document.getElementById(`${chartId}-title`);
            if (titleElement) {
                titleElement.textContent = newName.trim();
            }
            
            // Update chart label
            if (charts[chartId]) {
                charts[chartId].data.datasets[0].label = newName.trim();
                charts[chartId].update();
            }
            
            // Update active charts list
            updateActiveChartsList();
            
            showNotification(`Chart renamed to "${newName.trim()}"!`, 'success');
        }
    }
}

// Remove signal from chart
function removeSignalFromChart(chartId, fieldKey) {
    const config = monitorConfigs.find(c => c.id === chartId);
    if (!config || !config.assignedFields) return;
    
    // Remove field from assignedFields
    const fieldIndex = config.assignedFields.indexOf(fieldKey);
    if (fieldIndex === -1) return;
    
    config.assignedFields.splice(fieldIndex, 1);
    
    // Update chart - remove the dataset
    if (charts[chartId]) {
        const chart = charts[chartId];
        const datasetIndex = chart.data.datasets.findIndex(ds => ds.fieldKey === fieldKey);
        
        if (datasetIndex !== -1) {
            chart.data.datasets.splice(datasetIndex, 1);
            chart.update();
        }
        
        // If no more signals, hide legend
        if (config.assignedFields.length === 0) {
            chart.options.plugins.legend.display = false;
            chart.update();
        } else if (config.assignedFields.length === 1) {
            // If only one signal left, optionally hide legend
            chart.options.plugins.legend.display = false;
            chart.update();
        }
    }
    
    // Update active charts list
    updateActiveChartsList();
    
    const fieldLabel = availableFields[fieldKey]?.label || 'Signal';
    showNotification(`"${fieldLabel}" removed from chart!`, 'success');
}

// Initialize color pickers for data fields
function initializeColorPickers() {
    const fieldIcons = document.querySelectorAll('.field-icon');
    
    fieldIcons.forEach(icon => {
        icon.style.cursor = 'pointer';
        icon.title = 'Click to change color';
        
        icon.addEventListener('click', function(e) {
            e.stopPropagation(); // Prevent drag start
            
            const fieldElement = this.closest('.data-field');
            const fieldName = fieldElement.dataset.field;
            
            changeFieldColor(fieldName, this);
        });
    });
}

// Change color of a data field
function changeFieldColor(fieldName, iconElement) {
    const field = availableFields[fieldName];
    if (!field) return;
    
    // Create color input
    const colorInput = document.createElement('input');
    colorInput.type = 'color';
    colorInput.value = field.color;
    
    colorInput.addEventListener('change', function() {
        const newColor = this.value;
        
        // Update field color
        availableFields[fieldName].color = newColor;
        
        // Update icon
        iconElement.style.background = newColor;
        
        // Update all charts using this field
        monitorConfigs.forEach(config => {
            if (config.assignedFields && config.assignedFields.includes(fieldName)) {
                const datasetIndex = config.assignedFields.indexOf(fieldName);
                if (charts[config.id] && charts[config.id].data.datasets[datasetIndex]) {
                    charts[config.id].data.datasets[datasetIndex].borderColor = newColor;
                    charts[config.id].update();
                }
            }
        });
        
        // Update active charts list
        updateActiveChartsList();
        
        showNotification(`🎨 Color updated for ${field.label}!`, 'success');
    });
    
    // Trigger click to open color picker
    colorInput.click();
}

// Add CSS animations
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from {
            transform: translateX(400px);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }
    
    @keyframes slideOut {
        from {
            transform: translateX(0);
            opacity: 1;
        }
        to {
            transform: translateX(400px);
            opacity: 0;
        }
    }
`;
document.head.appendChild(style);

