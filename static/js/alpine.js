
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
        
        /* ---------- CONFIG ---------- */
        const STORE_NAME = 'joy';  // Cambiato da 'crosshair' a 'joy' per leggere i dati del joystick
        const STATE_URL  = `/state/${STORE_NAME}`;
        const UPDATE_URL = `/update/${STORE_NAME}`;
        let lastUpdate = Date.now();
        let currentState = { x: 0, y: 0, robot_x: 0, robot_y: 0 };
        
        // Joystick configuration
        const joystickRange = 150; // Range di movimento del crosshair in pixel (fisso)

        // Controllo pulsanti
        let isStarted = false;
        const startStopBtn = document.getElementById('startStop');
        const residualSlider = document.getElementById('residualSlider');
        const residualValue = document.getElementById('residualValue');
        
        // Pulsanti test che si alternano (solo uno attivo alla volta)
        const testButtons = [
            document.getElementById('testSingleJump'),
            document.getElementById('testMultiJump'),
            document.getElementById('discreteJump'),
            document.getElementById('optiJump')
        ];
        let activeTestButton = null;

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
        
        /* ---------- POLLING ---------- */
        async function pollState() {
            while (true) {
                try {
                    const res  = await fetch(STATE_URL);
                    const data = await res.json();

                    const axes = Array.isArray(data.axes) ? data.axes : [];
                    const buttons = Array.isArray(data.buttons) ? data.buttons : [];

                    // Leggi gli assi del joystick (axis 0 = X, axis 1 = Y)
                    const axis0 = axes[0] ?? 0; // X axis
                    const axis1 = axes[1] ?? 0; // Y axis
                    
                    // Converte i valori degli assi (-1 a +1) in coordinate pixel
                    const crosshairX = axis0 * joystickRange;
                    const crosshairY = axis1 * joystickRange;
                    
                    // Gestione del bottone 0 per il jump
                    const button0 = buttons[0] ?? 0;
                    
                    // Aggiorna l'indicatore del jump
                    jumpIndicator.classList.toggle('active', button0 === 1);
                    
                    if (button0 === 1) {
                        console.log('Jump button pressed!');
                        // Qui puoi aggiungere la logica per il jump
                    }

                    // Aggiorna lo stato corrente
                    currentState = { 
                        x: crosshairX, 
                        y: crosshairY, 
                        robot_x: 0, // Il JOY_STATE non ha robot_x/robot_y, li impostiamo a 0
                        robot_y: 0,
                        axis0: axis0,
                        axis1: axis1,
                        button0: button0
                    };

                    coordinates.textContent =
                        `X: ${Math.round(crosshairX)}, Y: ${Math.round(crosshairY)}`;

                    joystickInfo.innerHTML =
                        `Axis 0: ${axis0.toFixed(3)}<br>Axis 1: ${axis1.toFixed(3)}<br>Button 0: ${button0}`;

                    updateGeometry(crosshairX, crosshairY, currentState.robot_x, currentState.robot_y);

                    const now = Date.now();
                    statusIndicator.classList.toggle('connected', now - lastUpdate < 1000);
                    lastUpdate = now;
                } catch (e) {
                    console.error('Errore connessione:', e);
                    statusIndicator.classList.remove('connected');
                }

                await new Promise(r => setTimeout(r, 50)); // 20 fps circa
            }
        }

        /* ---------- BUTTON HANDLERS ---------- */
        // Residual Jump Slider
        residualSlider.addEventListener('input', function() {
            residualValue.textContent = this.value;
            console.log(`Residual Jump: ${this.value}`);
        });
        
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
        });

        // Main control buttons
        document.getElementById('calibration').addEventListener('click', function() {
            console.log('Calibration avviata');
            // Qui aggiungi la logica per calibration
        });

        document.getElementById('initialization').addEventListener('click', function() {
            console.log('Initialization avviata');
            // Qui aggiungi la logica per initialization
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
                }
                
                // Disabilita il pulsante corrente
                this.classList.add('disabled');
                this.style.pointerEvents = 'none';
                this.style.opacity = '0.4';
                
                // Imposta come pulsante attivo
                activeTestButton = this;
                
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
        
        /* ---------- INIT & RESIZE ---------- */
        window.addEventListener('load', () => {
            // Prima di ricevere dati reali, si parte con (0,0)
            updateGeometry(0, 0, 0, 0);
            pollState();                   // avvia il loop una sola volta
        });

        window.addEventListener('resize', () => {
            // Ricostruisce le linee se la finestra cambia
            updateGeometry(currentState.x, currentState.y, currentState.robot_x, currentState.robot_y);
        });