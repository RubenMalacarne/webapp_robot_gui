const socket = io();

socket.on('winch_telemetry', function(data) {
    console.log("Telemetria aggiornata:", data);

    if (data.winch_right) {
        document.getElementById('winch-right-rope-force').textContent = data.winch_right.rope_force.toFixed(2);
        document.getElementById('winch-right-rope-length').textContent = data.winch_right.rope_length.toFixed(2);
        document.getElementById('winch-right-rope-velocity').textContent = data.winch_right.rope_velocity.toFixed(2);
        document.getElementById('winch-right-current').textContent = data.winch_right.current.toFixed(2) + " A";
        document.getElementById('winch-right-brake-status').textContent = data.winch_right.brake_status;
        document.getElementById('winch-right-connected').textContent = data.winch_right.connected;
    }

    if (data.winch_left) {
        document.getElementById('winch-left-rope-force').textContent = data.winch_left.rope_force.toFixed(2);
        document.getElementById('winch-left-rope-length').textContent = data.winch_left.rope_length.toFixed(2);
        document.getElementById('winch-left-rope-velocity').textContent = data.winch_left.rope_velocity.toFixed(2);
        document.getElementById('winch-left-current').textContent = data.winch_left.current.toFixed(2) + " A";
        document.getElementById('winch-left-brake-status').textContent = data.winch_left.brake_status;
        document.getElementById('winch-left-connected').textContent = data.winch_left.connected;
    }
});

socket.emit('register', { node: 'inspector_gui' });

let lastUpdate = Date.now();


// Listener per dati telemetria winch
socket.on('winch_telemetry', (data) => {
    console.log('Ricevuti dati telemetria winch:', data);
    if (data.winch_right) {
        updateWinchTelemetry('right', data.winch_right);
    }
    if (data.winch_left) {
        updateWinchTelemetry('left', data.winch_left);
    }
    lastUpdate = Date.now();
});

// Listener per dati telemetria Alpine Body
socket.on('alpine_body_telemetry', (data) => {
    console.log('Ricevuti dati telemetria Alpine Body:', data);
    updateAlpineBodyTelemetry(data);
    lastUpdate = Date.now();
});

// Listener per eventi di connessione Socket.IO
socket.on('connect', () => {
    console.log('✅ Inspector connesso al server Socket.IO');
    updateConnectionStatus(true);
});

socket.on('disconnect', () => {
    console.log('❌ Inspector disconnesso dal server Socket.IO');
    updateConnectionStatus(false);
});

socket.on('connect_error', (error) => {
    console.error('Errore connessione Socket.IO:', error);
    updateConnectionStatus(false);
});


function updateWinchTelemetry(side, telemetryData) {
    const sectionTitle = side === 'right' ? 'Winch Right' : 'Winch Left';
    const section = findSectionByTitle(sectionTitle);
    
    if (!section) {
        console.warn(`Sezione non trovata: ${sectionTitle}`);
        return;
    }
    const fieldMappings = {
        'rope_force': 'Rope force',
        'rope_length': 'Rope length',
        'rope_velocity': 'Rope velocity',
        'current': 'Current',
        'brake_status': 'Brake Status'
    };
    
    Object.entries(fieldMappings).forEach(([dataKey, labelText]) => {
        if (telemetryData[dataKey] !== undefined) {
            const valueElement = findValueElementInSection(section, labelText);
            if (valueElement) {
                let displayValue = telemetryData[dataKey];
                if (dataKey === 'current') {
                    displayValue = `${displayValue} A`;
                } else if (dataKey === 'brake_status') {
                    displayValue = displayValue ? 'True' : 'False';
                    valueElement.style.color = displayValue === 'True' ? '#00ff88' : '#ff4444';
                }
                
                valueElement.textContent = displayValue;
                console.log(`✅ Aggiornato ${sectionTitle} - ${labelText}: ${displayValue}`);
            } else {
                console.warn(`❌ Elemento valore non trovato per ${labelText} in ${sectionTitle}`);
            }
        }
    });
}

function updateAlpineBodyTelemetry(data) {
    console.log('Aggiornamento Alpine Body telemetry:', data);
    if (data.rope_imu_orientation) {
        const element = document.getElementById('rope-imu-orientation');
        if (element) {
            element.textContent = `${data.rope_imu_orientation.x.toFixed(3)}, ${data.rope_imu_orientation.y.toFixed(3)}, ${data.rope_imu_orientation.z.toFixed(3)}, ${data.rope_imu_orientation.w.toFixed(3)}`;
        }
    }
    
    if (data.rope_imu_angular_velocity) {
        const element = document.getElementById('rope-imu-angular-velocity');
        if (element) {
            element.textContent = `${data.rope_imu_angular_velocity.x.toFixed(3)}, ${data.rope_imu_angular_velocity.y.toFixed(3)}, ${data.rope_imu_angular_velocity.z.toFixed(3)}`;
        }
    }
    
    if (data.rope_imu_rpy) {
        const element = document.getElementById('rope-imu-rpy');
        if (element) {
            element.textContent = `${data.rope_imu_rpy.x.toFixed(3)}, ${data.rope_imu_rpy.y.toFixed(3)}, ${data.rope_imu_rpy.z.toFixed(3)}`;
        }
    }
    
    if (data.rope_imu_rpy_d) {
        const element = document.getElementById('rope-imu-rpy-d');
        if (element) {
            element.textContent = `${data.rope_imu_rpy_d.x.toFixed(3)}, ${data.rope_imu_rpy_d.y.toFixed(3)}, ${data.rope_imu_rpy_d.z.toFixed(3)}`;
        }
    }
    
    if (data.rope_imu_acceleration) {
        const element = document.getElementById('rope-imu-acceleration');
        if (element) {
            element.textContent = `${data.rope_imu_acceleration.x.toFixed(3)}, ${data.rope_imu_acceleration.y.toFixed(3)}, ${data.rope_imu_acceleration.z.toFixed(3)}`;
        }
    }
    if (data.body_imu_orientation) {
        const element = document.getElementById('body-imu-orientation');
        if (element) {
            element.textContent = `${data.body_imu_orientation.x.toFixed(3)}, ${data.body_imu_orientation.y.toFixed(3)}, ${data.body_imu_orientation.z.toFixed(3)}, ${data.body_imu_orientation.w.toFixed(3)}`;
        }
    }
    
    if (data.body_imu_angular_velocity) {
        const element = document.getElementById('body-imu-angular-velocity');
        if (element) {
            element.textContent = `${data.body_imu_angular_velocity.x.toFixed(3)}, ${data.body_imu_angular_velocity.y.toFixed(3)}, ${data.body_imu_angular_velocity.z.toFixed(3)}`;
        }
    }
    
    if (data.body_imu_rpy) {
        const element = document.getElementById('body-imu-rpy');
        if (element) {
            element.textContent = `${data.body_imu_rpy.x.toFixed(3)}, ${data.body_imu_rpy.y.toFixed(3)}, ${data.body_imu_rpy.z.toFixed(3)}`;
        }
    }
    
    if (data.body_imu_rpy_derivatives) {
        const element = document.getElementById('body-imu-rpy-derivatives');
        if (element) {
            element.textContent = `${data.body_imu_rpy_derivatives.x.toFixed(3)}, ${data.body_imu_rpy_derivatives.y.toFixed(3)}, ${data.body_imu_rpy_derivatives.z.toFixed(3)}`;
        }
    }
    
    if (data.body_imu_acceleration) {
        const element = document.getElementById('body-imu-acceleration');
        if (element) {
            element.textContent = `${data.body_imu_acceleration.x.toFixed(3)}, ${data.body_imu_acceleration.y.toFixed(3)}, ${data.body_imu_acceleration.z.toFixed(3)}`;
        }
    }
}

function findSectionByTitle(titleText) {
    const sections = document.querySelectorAll('.data-section');
    for (let section of sections) {
        const titleElement = section.querySelector('.data-title');
        if (titleElement && titleElement.textContent === titleText) {
            return section;
        }
    }
    return null;
}

function findValueElementInSection(section, labelText) {
    const items = section.querySelectorAll('.data-item');
    for (let item of items) {
        const label = item.querySelector('.data-label');
        if (label && label.textContent === labelText) {
            return item.querySelector('.data-value');
        }
    }
    return null;
}

function findValueElementInGroup(group, labelText) {
    const items = group.querySelectorAll('.data-item');
    for (let item of items) {
        const label = item.querySelector('.data-label');
        if (label && label.textContent === labelText) {
            return item.querySelector('.data-value');
        }
    }
    return null;
}

function updateConnectionStatus(connected) {
    const indicators = document.querySelectorAll('.status-indicator');
    indicators.forEach(indicator => {
        if (!connected) {
            indicator.classList.remove('status-connected', 'status-warning');
            indicator.classList.add('status-disconnected');
        }
    });
}

function requestDataUpdate() {
    socket.emit('request_inspector_data', { 
        timestamp: Date.now(),
        node: 'inspector_gui'
    });
}

document.addEventListener('DOMContentLoaded', () => {
    console.log('🔧 Inspector GUI inizializzato');
    console.log('Sezioni presenti:', document.querySelectorAll('.data-section').length);
    console.log('Gruppi dati presenti:', document.querySelectorAll('.data-group').length);
    console.log('Elementi dati presenti:', document.querySelectorAll('.data-item').length);
    
    const sections = document.querySelectorAll('.data-section');
    sections.forEach((section, index) => {
        const title = section.querySelector('.data-title')?.textContent;
        console.log(`📋 Sezione ${index + 1}: ${title}`);
        
        const groups = section.querySelectorAll('.data-group');
        groups.forEach((group, groupIndex) => {
            const groupTitle = group.querySelector('.group-title')?.textContent;
            console.log(`  📁 Gruppo ${groupIndex + 1}: ${groupTitle}`);
        });
    });
    setTimeout(() => {
        requestDataUpdate();
    }, 1000);

    setInterval(requestDataUpdate, 3000);
});

window.addEventListener('error', (event) => {
    console.error('Errore JavaScript Inspector:', event.error);
});

window.addEventListener('beforeunload', () => {
    if (socket) {
        socket.disconnect();
    }
});