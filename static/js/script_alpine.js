
        const crosshair       = document.getElementById('crosshair');
        const coordinates     = document.getElementById('coordinates');
        const joystickInfo    = document.getElementById('joystick-info');
        const statusIndicator = document.getElementById('status');
        const container       = document.getElementById('container');
        const lineLeft        = document.getElementById('lineLeft');
        const lineRight       = document.getElementById('lineRight');
        const crosshairLineLeft  = document.getElementById('crosshairLineLeft');
        const crosshairLineRight = document.getElementById('crosshairLineRight');
        const distanceVector  = document.getElementById('distanceVector');
        const distanceLabel   = document.getElementById('distanceLabel');
        const jumpIndicator   = document.getElementById('jumpIndicator');
        
        // Inizializza Socket.IO
        const socket = io();
        
        // Registra questo client
        socket.emit('register', { node: 'alpine_gui' });
        
        // Listener jump indicator
        socket.on('jump_indicator', (data) => {
            const isJumping = data.jumping || false;
            
            if (isJumping) {
                // Attiva l'indicatore jump
                jumpIndicator.classList.add('active');
                console.log('🦘 Jump indicator ATTIVATO!');
                
            } else {
                // Disattiva l'indicatore jump
                jumpIndicator.classList.remove('active');
                console.log('🦘 Jump indicator disattivato');
            }
        });

        // Listener per feedback dai pulsanti (opzionale)
        socket.on('button_start_stop', (data) => {
            console.log('Feedback Start/Stop:', data);
        });
        
        socket.on('button_calibration', (data) => {
            console.log('Feedback Calibration:', data);
        });
        
        socket.on('button_initialization', (data) => {
            console.log('Feedback Initialization:', data);
        });

        
        // Listener per ricevere comandi robot da ROS2 tramite Socket.IO
        socket.on('robot_cmd', (data) => {
            const throttle = data.throttle || 0.0;
            const yaw = data.yaw || 0.0;
            
            // Converte throttle e yaw in coordinate per il crosshair
            // Assumendo che throttle sia l'asse Y e yaw sia l'asse X
            const crosshairX = yaw * joystickRange;        // Yaw -> movimento orizzontale
            const crosshairY = -throttle * joystickRange;  // Throttle -> movimento verticale (invertito)
            
            // Aggiorna lo stato corrente
            currentState = { 
                x: crosshairX, 
                y: crosshairY, 
                robot_x: 0,
                robot_y: 0,
                throttle: throttle,
                yaw: yaw
            };

            // Aggiorna la UI
            coordinates.textContent = `X: ${Math.round(crosshairX)}, Y: ${Math.round(crosshairY)}`;
            joystickInfo.innerHTML = `Throttle: ${throttle.toFixed(3)}<br>Yaw: ${yaw.toFixed(3)}`;
            
            // Aggiorna la geometria
            updateGeometry(crosshairX, crosshairY, currentState.robot_x, currentState.robot_y);
            
            // Aggiorna indicatore di connessione
            const now = Date.now();
            statusIndicator.classList.toggle('connected', now - lastUpdate < 2000);
            lastUpdate = now;
        });
        
        /* ---------- CONFIG ---------- */
        // Configurazione Socket.IO per joystick
        let lastUpdate = Date.now();
        let currentState = { x: 0, y: 0, robot_x: 0, robot_y: 0 };
        const joystickRange = 150; // Range di movimento del crosshair in pixel

        // Controllo pulsanti
        let isStarted = false;
        const startStopBtn = document.getElementById('startStop');
        // Pulsanti test che si alternano (solo uno attivo alla volta)
        const testButtons = [
            document.getElementById('testSingleJump'),
            document.getElementById('testMultiJump'),
            document.getElementById('discreteJump'),
            document.getElementById('optiJump')
        ];
        let activeTestButton = null;
        
        // update gui robot state
        function updateGeometry(crosshairX, crosshairY, robotX, robotY) {
            const containerRect = container.getBoundingClientRect();
            const containerWidth = containerRect.width;
            const containerHeight = containerRect.height;

            // Conversion factor: pixels per meter (adjust based on your setup)
            const pixelsPerMeter = 100; // Example: 100 pixels = 1 meter

            // Arrow triangle dimensions (match your CSS exactly!)
            const arrowHeight = 35; // height from CSS (border-bottom: 35px solid)
            const tipOffsetY = arrowHeight / 2; // vertical offset from center to tip (upward)

            // Robot's center position
            const robotCenterX = containerWidth / 2 + robotX;
            const robotCenterY = containerHeight / 2 + robotY;

            // Arrow tip exact position
            const arrowTipX = robotCenterX;
            const arrowTipY = robotCenterY - tipOffsetY;

            // Position the purple arrow (CSS triangle visual)
            const robot = document.querySelector('.robot');
            robot.style.left = robotCenterX + 'px';
            robot.style.top = (robotCenterY - tipOffsetY) + 'px';  // Arrow top-left CSS position adjustment

            // Position the crosshair exactly at the arrow tip by default
            const crosshair = document.getElementById('crosshair');
            crosshair.style.left = (arrowTipX + crosshairX) + 'px';
            crosshair.style.top = (arrowTipY + crosshairY) + 'px';
            crosshair.style.transform = `translate(-50%, -50%)`;

            // Vector from arrow tip to displaced crosshair (useful if joystick moves crosshair)
            const deltaX = crosshairX / pixelsPerMeter; // Convert to meters
            const deltaY = crosshairY / pixelsPerMeter; // Convert to meters
            const vectorDistance = Math.sqrt(deltaX**2 + deltaY**2); // Distance in meters
            const vectorAngle = Math.atan2(deltaY, deltaX) * 180 / Math.PI;

            if (vectorDistance > 0.05) { // Threshold in meters
                distanceVector.style.display = 'block';
                distanceVector.style.left = arrowTipX + 'px';
                distanceVector.style.top = arrowTipY + 'px';
                distanceVector.style.width = vectorDistance * pixelsPerMeter + 'px'; // Convert back to pixels
                distanceVector.style.transform = `rotate(${vectorAngle}deg)`;

                const labelX = arrowTipX + crosshairX * 0.5;
                const labelY = arrowTipY + crosshairY * 0.5 - 20;

                distanceLabel.style.display = 'block';
                distanceLabel.style.left = labelX + 'px';
                distanceLabel.style.top = labelY + 'px';
                distanceLabel.textContent = `ΔX: ${deltaX.toFixed(2)} m, ΔY: ${deltaY.toFixed(2)} m, Distance: ${vectorDistance.toFixed(2)} m`;
            } else {
                distanceVector.style.display = 'none';
                distanceLabel.style.display = 'none';
            }
        }
        
        function updateDashedLine(line, startX, startY, endX, endY) {
            const dx = endX - startX;
            const dy = endY - startY;
            const distance = Math.hypot(dx, dy);
            const angle = Math.atan2(dy, dx) * 180 / Math.PI;

            line.style.left   = startX + 'px';
            line.style.top    = startY + 'px';
            line.style.width  = distance + 'px';
            line.style.transform = `rotate(${angle}deg)`;
        }
        
        /* ---------- POLLING SOSTITUITO CON SOCKET.IO ---------- */
        // Polling rimosso - ora i dati joystick arrivano tramite Socket.IO
        // async function pollState() { ... }
        
        /* ---------- INIT & RESIZE ---------- */
        window.addEventListener('load', () => {
            // Prima di ricevere dati reali, si parte con (0,0)
            updateGeometry(0, 0, 0, 0);
        });

        window.addEventListener('resize', () => {
            // Ricostruisce le linee se la finestra cambia
            updateGeometry(currentState.x, currentState.y, currentState.robot_x, currentState.robot_y);
        });

        /* ---------- BUTTON HANDLERS ---------- */
        // Start/Stop toggle button
        startStopBtn.addEventListener('click', function() {
            isStarted = !isStarted;
            if (isStarted) {
                this.textContent = 'Stop';
                this.className = 'control-button stop';
                console.log('Sistema avviato');
            } else {
                this.textContent = 'Start';
                this.className = 'control-button start';
                console.log('Sistema fermato');
            }
            // Invia stato tramite Socket.IO
            socket.emit('button_start_stop', { is_started: isStarted });
        });

        // Main control buttons
        document.getElementById('calibration').addEventListener('click', function() {
            console.log('Calibration avviata');
            // Invia comando tramite Socket.IO
            socket.emit('button_calibration', { triggered: true });
        });

        document.getElementById('initialization').addEventListener('click', function() {
            console.log('Initialization avviata');
            // Invia comando tramite Socket.IO
            socket.emit('button_initialization', { triggered: true });
        });

        // Test buttons che si alternano (solo uno attivo alla volta)
        testButtons.forEach((button) => {
            button.addEventListener('click', function() {
                console.log(`${this.textContent} selezionato`);
                
                // Se c'è già un pulsante attivo, riabilitalo
                if (activeTestButton && activeTestButton !== this) {
                    activeTestButton.classList.remove('disabled');
                    activeTestButton.style.pointerEvents = 'auto';
                    activeTestButton.style.opacity = '1';
                    
                    // Disattiva il pulsante precedente tramite Socket.IO
                    const prevButtonEvent = getButtonEventName(activeTestButton.id);
                    if (prevButtonEvent) {
                        socket.emit(prevButtonEvent, { is_active: false });
                    }
                }
                
                // Disabilita il pulsante corrente
                this.classList.add('disabled');
                this.style.pointerEvents = 'none';
                this.style.opacity = '0.4';
                
                // Imposta come pulsante attivo
                activeTestButton = this;
                
                // Invia attivazione tramite Socket.IO
                const buttonEvent = getButtonEventName(this.id);
                if (buttonEvent) {
                    socket.emit(buttonEvent, { is_active: true });
                }
                
                // Qui aggiungi la logica specifica per ogni test
                switch(this.id) {
                    case 'testSingleJump':
                        console.log('Modalità: Test Single Jump attivata');
                        break;
                    case 'testMultiJump':
                        console.log('Modalità: Test Multi Jump attivata');
                        break;
                    case 'discreteJump':
                        console.log('Modalità: Discrete Jump attivata');
                        break;
                    case 'optiJump':
                        console.log('Modalità: Opti Jump attivata');
                        break;
                }
            });
        });
        
        // Funzione helper per mappare gli ID dei pulsanti agli eventi Socket.IO
        function getButtonEventName(buttonId) {
            const eventMap = {
                'testSingleJump': 'button_test_single_jump',
                'testMultiJump': 'button_test_multi_jump',
                'discreteJump': 'button_discrete_jump',
                'optiJump': 'button_opti_jump'
            };
            return eventMap[buttonId];
        }