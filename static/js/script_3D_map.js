
let scene, camera, renderer, pointCloud, controls;
let socket = null;
let isConnected = false;
let pointsData = [];
let animationId;

//setup 3D map
function initThree() {
    const container = document.getElementById('three-container');

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);

    // Camera
    camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
    camera.position.set(10, 10, 10);
    camera.lookAt(0, 0, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    // Simple orbit controls (manual implementation since OrbitControls not available)
    setupControls();

    // Lights
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 5);
    scene.add(directionalLight);

    // Grid helper
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    scene.add(gridHelper);

    // Axes helper
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    // Start render loop
    animate();

    // Handle window resize
    window.addEventListener('resize', onWindowResize);
}

// mouse controls for camera
function setupControls() {
    let isMouseDown = false;
    let mouseX = 0, mouseY = 0;
    let targetX = 0, targetY = 0;
    let windowHalfX = window.innerWidth / 2;
    let windowHalfY = window.innerHeight / 2;

    const container = document.getElementById('three-container');

    container.addEventListener('mousedown', (event) => {
        isMouseDown = true;
        mouseX = event.clientX - windowHalfX;
        mouseY = event.clientY - windowHalfY;
    });

    container.addEventListener('mouseup', () => {
        isMouseDown = false;
    });

    container.addEventListener('mousemove', (event) => {
        if (isMouseDown) {
            targetX = (event.clientX - windowHalfX - mouseX) * 0.005;
            targetY = (event.clientY - windowHalfY - mouseY) * 0.005;

            // Rotate camera around the origin
            const radius = camera.position.length();
            camera.position.x = radius * Math.sin(targetX) * Math.cos(targetY);
            camera.position.z = radius * Math.cos(targetX) * Math.cos(targetY);
            camera.position.y = radius * Math.sin(targetY);
            camera.lookAt(0, 0, 0);
        }
    });

    container.addEventListener('wheel', (event) => {
        const scale = event.deltaY > 0 ? 1.1 : 0.9;
        camera.position.multiplyScalar(scale);
        event.preventDefault();
    });
}

function onWindowResize() {
    const container = document.getElementById('three-container');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

// Animation loop
function animate() {
    animationId = requestAnimationFrame(animate);
    renderer.render(scene, camera);
    updateFPS();
}

// Create or update point cloud
function updatePointCloud() {
    if (pointCloud) {
        scene.remove(pointCloud);
    }

    if (pointsData.length === 0) return;

    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(pointsData.length * 3);
    const colors = new Float32Array(pointsData.length * 3);

    const density = parseFloat(document.getElementById('density-slider').value);
    // Uso un colore fisso verde per i punti
    const customColor = new THREE.Color(0x00ff88);

    let visiblePoints = 0;
    for (let i = 0; i < pointsData.length; i++) {
        if (Math.random() > density) continue;

        const point = pointsData[i];
        positions[visiblePoints * 3] = point.x;
        positions[visiblePoints * 3 + 1] = point.y;
        positions[visiblePoints * 3 + 2] = point.z;

        // Colore fisso verde
        colors[visiblePoints * 3] = customColor.r;
        colors[visiblePoints * 3 + 1] = customColor.g;
        colors[visiblePoints * 3 + 2] = customColor.b;

        visiblePoints++;
    }

    // Trim arrays to actual size
    const trimmedPositions = positions.slice(0, visiblePoints * 3);
    const trimmedColors = colors.slice(0, visiblePoints * 3);

    geometry.setAttribute('position', new THREE.BufferAttribute(trimmedPositions, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(trimmedColors, 3));

    const material = new THREE.PointsMaterial({
        size: parseFloat(document.getElementById('size-slider').value),
        vertexColors: true,
        opacity: parseFloat(document.getElementById('opacity-slider').value),
        transparent: true,
        sizeAttenuation: false
    });

    pointCloud = new THREE.Points(geometry, material);
    scene.add(pointCloud);

    document.getElementById('point-count').textContent = visiblePoints;
}

// Generate sample data EXAMPLE
function generateSampleData() {
    pointsData = [];
    const numPoints = 5000;

    for (let i = 0; i < numPoints; i++) {
        const theta = Math.random() * Math.PI * 2;
        const phi = Math.random() * Math.PI;
        const radius = 5 + Math.random() * 10;

        pointsData.push({
            x: radius * Math.sin(phi) * Math.cos(theta) + (Math.random() - 0.5) * 2,
            y: radius * Math.cos(phi) + (Math.random() - 0.5) * 2,
            z: radius * Math.sin(phi) * Math.sin(theta) + (Math.random() - 0.5) * 2,
            intensity: Math.random()
        });
    }

    updatePointCloud();
}

// Socket.IO functions
function connectSocket() {
    const serverUrl = document.getElementById('server-url').value;

    try {
        socket = io(serverUrl);

        socket.on('connect', () => {
            isConnected = true;
            document.getElementById('connection-status').textContent = 'Connected';
            document.getElementById('connection-dot').classList.add('connected');
            console.log('Connected to server');
            
            // Register this client as a 3D map viewer
            socket.emit('register', {'node': '3d_map_viewer'});
        });

        socket.on('disconnect', () => {
            isConnected = false;
            document.getElementById('connection-status').textContent = 'Disconnected';
            document.getElementById('connection-dot').classList.remove('connected');
            console.log('Disconnected from server');
        });

        socket.on('point_cloud', (data) => {
            console.log('Received point cloud data:', data);
            if (data && data.point) {
                pointsData = data.point;
                updatePointCloud();
                console.log(`Point cloud updated with ${pointsData.length} points`);
            }
        });


        socket.on('connect_error', (error) => {
            console.error('Connection error:', error);
            document.getElementById('connection-status').textContent = 'Error';
        });

    } catch (error) {
        console.error('Failed to connect:', error);
        document.getElementById('connection-status').textContent = 'Error';
    }
}

function disconnectSocket() {
    if (socket) {
        socket.disconnect();
        socket = null;
    }
    isConnected = false;
    document.getElementById('connection-status').textContent = 'Disconnected';
    document.getElementById('connection-dot').classList.remove('connected');
}

// FPS counter
let lastTime = performance.now();
let frameCount = 0;

function updateFPS() {
    frameCount++;
    const currentTime = performance.now();

    if (currentTime - lastTime >= 1000) {
        document.getElementById('fps-counter').textContent = frameCount;
        frameCount = 0;
        lastTime = currentTime;

        // Update memory usage (approximation)
        const memoryMB = (pointsData.length * 24 / 1024 / 1024).toFixed(1);
        document.getElementById('memory-usage').textContent = memoryMB + ' MB';
    }
}

// Event listeners
document.addEventListener('DOMContentLoaded', () => {
    initThree();

    // Sliders
    document.getElementById('density-slider').addEventListener('input', (e) => {
        document.getElementById('density-value').textContent = e.target.value;
        updatePointCloud();
    });

    document.getElementById('size-slider').addEventListener('input', (e) => {
        document.getElementById('size-value').textContent = e.target.value;
        updatePointCloud();
    });

    document.getElementById('opacity-slider').addEventListener('input', (e) => {
        document.getElementById('opacity-value').textContent = e.target.value;
        updatePointCloud();
    });

    document.getElementById('height-filter').addEventListener('input', (e) => {
        document.getElementById('height-value').textContent = e.target.value;
    });

    document.getElementById('update-rate').addEventListener('input', (e) => {
        document.getElementById('rate-value').textContent = e.target.value;
    });

    // Buttons
    document.getElementById('connect-btn').addEventListener('click', connectSocket);
    document.getElementById('disconnect-btn').addEventListener('click', disconnectSocket);
    document.getElementById('clear-points').addEventListener('click', () => {
        pointsData = [];
        updatePointCloud();
    });
    document.getElementById('generate-sample').addEventListener('click', generateSampleData);

    document.getElementById('apply-filters').addEventListener('click', () => {
        const minDist = parseFloat(document.getElementById('min-distance').value);
        const maxDist = parseFloat(document.getElementById('max-distance').value);
        const heightFilter = parseFloat(document.getElementById('height-filter').value);

        pointsData = pointsData.filter(point => {
            const dist = Math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            return dist >= minDist && dist <= maxDist && Math.abs(point.y - heightFilter) < 5;
        });

        updatePointCloud();
    });

    document.getElementById('reset-filters').addEventListener('click', () => {
        document.getElementById('min-distance').value = 0.1;
        document.getElementById('max-distance').value = 100;
        document.getElementById('height-filter').value = 0;
        document.getElementById('height-value').textContent = '0';
    });

    document.getElementById('reset-camera').addEventListener('click', () => {
        camera.position.set(10, 10, 10);
        camera.lookAt(0, 0, 0);
    });

    document.getElementById('save-screenshot').addEventListener('click', () => {
        const link = document.createElement('a');
        link.download = 'pointcloud_screenshot.png';
        link.href = renderer.domElement.toDataURL();
        link.click();
    });

    // Generate initial sample data
    generateSampleData();
});