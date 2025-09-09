// Inizializza Socket.IO
const socket = io();

// Registra questo client
socket.emit('register', { node: 'inspector_gui' });

// Socket.IO Listeners per ricevere dati dal backend
socket.on('robot_status', (data) => {
    console.log('Ricevuti dati status robot:', data);
    
    // Aggiorna stato ESP32
    if (data.esp32_status !== undefined) {
        updateStatusValue('esp32_status', data.esp32_status ? 'Connected' : 'Disconnected', 
                         data.esp32_status ? 'status-connected' : 'status-disconnected');
    }
    
    // Aggiorna forza del segnale
    if (data.signal_strength !== undefined) {
        updateStatusValue('signal_strength', data.signal_strength + '%', 
                         data.signal_strength > 80 ? 'status-connected' : 
                         data.signal_strength > 50 ? 'status-warning' : 'status-disconnected');
    }
    
    // Aggiorna batteria
    if (data.battery !== undefined) {
        updateStatusValue('battery', data.battery + '%', 
                         data.battery > 50 ? 'status-connected' : 
                         data.battery > 20 ? 'status-warning' : 'status-disconnected');
    }
    
    // Aggiorna batteria tank
    if (data.battery_tank !== undefined) {
        updateStatusValue('battery_tank', data.battery_tank + '%', 
                         data.battery_tank > 50 ? 'status-connected' : 
                         data.battery_tank > 20 ? 'status-warning' : 'status-disconnected');
    }
    
    lastUpdate = Date.now();
});

socket.on('winch_status', (data) => {
    console.log('Ricevuti dati winch:', data);
    
    // Aggiorna stato winch destro
    if (data.winch_dx !== undefined) {
        updateWinchStatus('Winch_DX', data.winch_dx.connected);
    }
    
    // Aggiorna stato winch sinistro
    if (data.winch_sx !== undefined) {
        updateWinchStatus('Winch_SX', data.winch_sx.connected);
    }
    
    lastUpdate = Date.now();
});

socket.on('control_values', (data) => {
    console.log('Ricevuti valori di controllo:', data);
    
    // Aggiorna i valori di controllo
    const valueElements = document.querySelectorAll('#val_1');
    if (data.winch_sx_values && valueElements.length > 0) {
        valueElements.forEach((element, index) => {
            if (data.winch_sx_values[index] !== undefined) {
                element.textContent = data.winch_sx_values[index];
            }
        });
    }
    
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

// Funzioni utility per aggiornare l'UI
function updateStatusValue(fieldName, value, statusClass) {
    const element = findElementByLabel(fieldName);
    
    if (element) {
        // Trova il contenuto testuale (escludendo l'indicatore)
        const textContent = element.childNodes[0];
        if (textContent && textContent.nodeType === Node.TEXT_NODE) {
            textContent.textContent = value;
        } else {
            // Se non c'è un nodo di testo, crea il contenuto
            element.innerHTML = `${value}<span class="status-indicator ${statusClass}"></span>`;
            return;
        }
        
        // Aggiorna l'indicatore di stato
        const indicator = element.querySelector('.status-indicator');
        if (indicator) {
            indicator.className = `status-indicator ${statusClass}`;
        }
    } else {
        console.warn(`Elemento non trovato per: ${fieldName}`);
    }
}

function findElementByLabel(labelText) {
    const labels = document.querySelectorAll('.data-label');
    for (let label of labels) {
        const labelContent = label.textContent.toLowerCase().trim();
        const searchText = labelText.toLowerCase().replace('_', ' ').replace('_', ' ');
        
        // Cerca corrispondenze più precise
        if (labelContent.includes(searchText) || 
            labelContent === searchText ||
            (labelText === 'esp32_status' && labelContent.includes('esp32')) ||
            (labelText === 'signal_strength' && labelContent.includes('signal')) ||
            (labelText === 'battery_tank' && labelContent.includes('battery_tank'))) {
            return label.nextElementSibling;
        }
    }
    return null;
}

function updateWinchStatus(winchName, connected) {
    const winchGroups = document.querySelectorAll('.data-group');
    for (let group of winchGroups) {
        const titleElement = group.querySelector('.group-title');
        if (titleElement && titleElement.textContent === winchName) {
            const valueElement = group.querySelector('.data-value');
            if (valueElement) {
                valueElement.textContent = connected ? 'True' : 'False';
                valueElement.style.color = connected ? '#00ff88' : '#ff4444';
            }
            break;
        }
    }
}

function updateConnectionStatus(connected) {
    // Aggiorna tutti gli indicatori di connessione se presenti
    const indicators = document.querySelectorAll('.status-indicator');
    indicators.forEach(indicator => {
        if (!connected) {
            indicator.classList.remove('status-connected', 'status-warning');
            indicator.classList.add('status-disconnected');
        }
    });
}

// Funzione per inviare richieste di dati (opzionale)
function requestDataUpdate() {
    socket.emit('request_inspector_data', { 
        timestamp: Date.now(),
        node: 'inspector_gui'
    });
}

// Simulazione dati (mantenuta per testing quando non ci sono dati reali)
function simulateData() {
    time += 0.1;
    
    // Simula solo se non ci sono stati aggiornamenti recenti dai socket
    const timeSinceLastUpdate = Date.now() - lastUpdate;
    if (timeSinceLastUpdate > 5000) { // 5 secondi senza aggiornamenti
        console.log('Nessun dato reale ricevuto, uso simulazione...');
        
        // Simula valori di stato
        const simulatedStatus = {
            esp32_status: Math.random() > 0.1, // 90% connected
            signal_strength: Math.floor(85 + Math.sin(time * 0.1) * 15),
            battery: Math.floor(90 + Math.sin(time * 0.05) * 10),
            battery_tank: Math.floor(95 + Math.cos(time * 0.08) * 5)
        };
        
        // Simula valori di controllo
        const simulatedControl = {
            winch_sx_values: [
                Math.floor(1024 + Math.sin(time) * 200),
                Math.floor(512 + Math.cos(time * 1.2) * 100),
                Math.floor(256 + Math.sin(time * 0.8) * 150)
            ]
        };
        
        // Emula eventi socket
        socket.emit('robot_status', simulatedStatus);
        socket.emit('control_values', simulatedControl);
    }
}

// Inizializzazione
document.addEventListener('DOMContentLoaded', () => {
    console.log('🔧 Inspector GUI inizializzato');
    console.log('Elementi presenti:', document.querySelectorAll('.data-item').length);
    
    // Aggiungi attributi data-field per identificazione più facile
    const labelMappings = {
        'esp32_status': 'esp32_status',
        'Signal Strength': 'signal_strength', 
        'Battery': 'battery',
        'Battery_tank': 'battery_tank'
    };
    
    Object.entries(labelMappings).forEach(([labelText, fieldName]) => {
        const element = findElementByLabel(labelText);
        if (element) {
            element.setAttribute('data-field', fieldName);
            console.log(`✅ Elemento trovato per ${labelText}:`, element);
        } else {
            console.warn(`❌ Elemento NON trovato per ${labelText}`);
        }
    });
    
    // Richiedi dati iniziali
    setTimeout(() => {
        requestDataUpdate();
    }, 1000);
    
    // Avvia simulazione come fallback
    setInterval(simulateData, 1000);
    
    // Richiedi aggiornamenti periodici
    setInterval(requestDataUpdate, 5000);
});

// Gestione errori globali
window.addEventListener('error', (event) => {
    console.error('Errore JavaScript Inspector:', event.error);
});

// Cleanup on page unload
window.addEventListener('beforeunload', () => {
    if (socket) {
        socket.disconnect();
    }
});