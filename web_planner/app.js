let scene, camera, renderer, controls;
let obstaclesGroup, treeGroup, pathGroup, markersGroup;
let uavModel = null;
let treeNodes = [];
let finalPath = null;
let isPlanning = false;
let animationFrameId = null;
let flightAnimationId = null;

window.addEventListener('DOMContentLoaded', () => {
    initUI();
    generateRandomObstacles(15);
    initThreeJS();
    resetSimulator();
});

function initUI() {
    const sliders = [
        { id: 'maxYaw', valId: 'maxYawVal', suffix: '°' },
        { id: 'maxPitch', valId: 'maxPitchVal', suffix: '°' },
        { id: 'safetyBuffer', valId: 'safetyBufferVal', suffix: 'm' },
        { id: 'obstacleCount', valId: 'obstacleCountVal', suffix: '' }
    ];
    sliders.forEach(s => {
        const sliderEl = document.getElementById(s.id);
        const valEl = document.getElementById(s.valId);
        sliderEl.addEventListener('input', () => {
            valEl.textContent = sliderEl.value + s.suffix;
            if (s.id === 'safetyBuffer') {
                updateObstaclesBuffer(parseFloat(sliderEl.value));
            } else if (s.id === 'obstacleCount') {
                generateRandomObstacles(parseInt(sliderEl.value));
                updateObstaclesBuffer(parseFloat(document.getElementById('safetyBuffer').value));
                resetSimulator();
            }
        });
    });
    document.getElementById('btnRandomizeObs').addEventListener('click', () => {
        const count = parseInt(document.getElementById('obstacleCount').value);
        generateRandomObstacles(count);
        updateObstaclesBuffer(parseFloat(document.getElementById('safetyBuffer').value));
        resetSimulator();
    });
    document.getElementById('btnRun').addEventListener('click', runPlannerInstant);
    document.getElementById('btnStep').addEventListener('click', runPlannerStepped);
    document.getElementById('btnSimulate').addEventListener('click', simulateFlight);
    document.getElementById('btnClear').addEventListener('click', resetSimulator);
}

function initThreeJS() {
    const container = document.getElementById('threejs-canvas');
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x050608);
    scene.fog = new THREE.FogExp2(0x050608, 0.007);

    camera = new THREE.PerspectiveCamera(45, width / height, 1, 1000);
    camera.position.set(-30, 45, 110);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 - 0.01;
    controls.minDistance = 10;
    controls.maxDistance = 300;

    obstaclesGroup = new THREE.Group();
    treeGroup = new THREE.Group();
    pathGroup = new THREE.Group();
    markersGroup = new THREE.Group();

    scene.add(obstaclesGroup);
    scene.add(treeGroup);
    scene.add(pathGroup);
    scene.add(markersGroup);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.4);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.6);
    dirLight1.position.set(100, 150, 50);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0x00f0ff, 0.3);
    dirLight2.position.set(-100, 50, -50);
    scene.add(dirLight2);

    const gridHelper = new THREE.GridHelper(100, 20, 0x00f0ff, 0x1f2430);
    gridHelper.position.set(50, 0, 50);
    scene.add(gridHelper);

    const boundsBox = new THREE.BoxHelper(new THREE.Mesh(
        new THREE.BoxGeometry(100, 30, 100),
        new THREE.MeshBasicMaterial()
    ), 0x334155);
    boundsBox.position.set(50, 15, 50);
    scene.add(boundsBox);

    updateObstaclesBuffer(parseFloat(document.getElementById('safetyBuffer').value));

    window.addEventListener('resize', () => {
        const w = container.clientWidth;
        const h = container.clientHeight;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    });

    function animate() {
        requestAnimationFrame(animate);
        controls.update();
        renderer.render(scene, camera);
    }
    animate();
}

function updateObstaclesBuffer(bufferSize) {
    while(obstaclesGroup.children.length > 0) {
        obstaclesGroup.remove(obstaclesGroup.children[0]);
    }
    OBSTACLES.forEach((obs) => {
        const w = obs.xMax - obs.xMin;
        const d = obs.yMax - obs.yMin;
        const h = obs.zMax - obs.zMin;
        const cx = obs.xMin + w / 2;
        const cy = obs.yMin + d / 2;
        const cz = obs.zMin + h / 2;

        const geom = new THREE.BoxGeometry(w, h, d);
        const mat = new THREE.MeshPhongMaterial({
            color: 0xe11d48,
            transparent: true,
            opacity: 0.5,
            shininess: 30
        });
        const mesh = new THREE.Mesh(geom, mat);
        mesh.position.set(cx, cz, cy);
        obstaclesGroup.add(mesh);

        const bufGeom = new THREE.BoxGeometry(w + 2 * bufferSize, h + 2 * bufferSize, d + 2 * bufferSize);
        const edges = new THREE.EdgesGeometry(bufGeom);
        const line = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({
            color: 0xf59e0b,
            transparent: true,
            opacity: 0.25,
            linewidth: 1
        }));
        line.position.set(cx, cz, cy);
        obstaclesGroup.add(line);
    });
}

function generateRandomObstacles(count) {
    OBSTACLES.length = 0; // Clear in-place
    const startX = parseFloat(document.getElementById('startX').value);
    const startY = parseFloat(document.getElementById('startY').value);
    const startZ = parseFloat(document.getElementById('startZ').value);
    const goalX = parseFloat(document.getElementById('goalX').value);
    const goalY = parseFloat(document.getElementById('goalY').value);
    const goalZ = parseFloat(document.getElementById('goalZ').value);

    const startBuf = 10.0;
    const goalBuf = 10.0;

    let attempts = 0;
    while (OBSTACLES.length < count && attempts < 1000) {
        attempts++;
        const w = 6.0 + Math.random() * 12.0;
        const d = 6.0 + Math.random() * 12.0;
        const h = 8.0 + Math.random() * 20.0;

        const cx = w / 2 + Math.random() * (X_MAX - w);
        const cy = d / 2 + Math.random() * (Y_MAX - d);

        const xMin = cx - w / 2;
        const xMax = cx + w / 2;
        const yMin = cy - d / 2;
        const yMax = cy + d / 2;
        const zMin = 0.0;
        const zMax = h;

        // Check if start overlaps
        const startOverlap = (
            (xMin - startBuf) <= startX && startX <= (xMax + startBuf) &&
            (yMin - startBuf) <= startY && startY <= (yMax + startBuf) &&
            (zMin - startBuf) <= startZ && startZ <= (zMax + startBuf)
        );

        // Check if goal overlaps
        const goalOverlap = (
            (xMin - goalBuf) <= goalX && goalX <= (xMax + goalBuf) &&
            (yMin - goalBuf) <= goalY && goalY <= (yMax + goalBuf) &&
            (zMin - goalBuf) <= goalZ && goalZ <= (zMax + goalBuf)
        );

        if (!startOverlap && !goalOverlap) {
            OBSTACLES.push({ xMin, yMin, zMin, xMax, yMax, zMax });
        }
    }
}

function getPlannerConfig() {
    return {
        start: new Node(
            parseFloat(document.getElementById('startX').value),
            parseFloat(document.getElementById('startY').value),
            parseFloat(document.getElementById('startZ').value),
            mathRadians(parseFloat(document.getElementById('startYaw').value))
        ),
        goal: new Node(
            parseFloat(document.getElementById('goalX').value),
            parseFloat(document.getElementById('goalY').value),
            parseFloat(document.getElementById('goalZ').value)
        ),
        maxYaw: mathRadians(parseFloat(document.getElementById('maxYaw').value)),
        maxPitch: mathRadians(parseFloat(document.getElementById('maxPitch').value)),
        stepSize: parseFloat(document.getElementById('stepSize').value),
        goalBias: parseFloat(document.getElementById('goalBias').value),
        goalThreshold: parseFloat(document.getElementById('goalThreshold').value),
        maxIter: parseInt(document.getElementById('maxIter').value),
        safetyBuffer: parseFloat(document.getElementById('safetyBuffer').value)
    };
}

function drawStartGoalMarkers(start, goal) {
    while(markersGroup.children.length > 0) {
        markersGroup.remove(markersGroup.children[0]);
    }
    const startGeom = new THREE.SphereGeometry(1.2, 16, 16);
    const startMat = new THREE.MeshPhongMaterial({ color: 0x22c55e, emissive: 0x052e16 });
    const startMesh = new THREE.Mesh(startGeom, startMat);
    startMesh.position.set(start.x, start.z, start.y);
    markersGroup.add(startMesh);

    const goalGeom = new THREE.SphereGeometry(1.5, 16, 16);
    const goalMat = new THREE.MeshPhongMaterial({ color: 0xeab308, emissive: 0x422006 });
    const goalMesh = new THREE.Mesh(goalGeom, goalMat);
    goalMesh.position.set(goal.x, goal.z, goal.y);
    markersGroup.add(goalMesh);

    const threshGeom = new THREE.SphereGeometry(parseFloat(document.getElementById('goalThreshold').value), 16, 8);
    const threshWire = new THREE.LineSegments(
        new THREE.EdgesGeometry(threshGeom),
        new THREE.LineBasicMaterial({ color: 0xeab308, transparent: true, opacity: 0.15 })
    );
    threshWire.position.set(goal.x, goal.z, goal.y);
    markersGroup.add(threshWire);
}

function resetSimulator() {
    isPlanning = false;
    if (animationFrameId) cancelAnimationFrame(animationFrameId);
    if (flightAnimationId) cancelAnimationFrame(flightAnimationId);

    while(treeGroup.children.length > 0) treeGroup.remove(treeGroup.children[0]);
    while(pathGroup.children.length > 0) pathGroup.remove(pathGroup.children[0]);
    if (uavModel) {
        scene.remove(uavModel);
        uavModel = null;
    }
    treeNodes = [];
    finalPath = null;

    updateStatus('READY', 'text-ready');
    document.getElementById('costVal').textContent = '0.00 m';
    document.getElementById('waypointsVal').textContent = '0';
    document.getElementById('treeNodesVal').textContent = '0';
    document.getElementById('timeVal').textContent = '0 ms';
    document.getElementById('efficiencyVal').textContent = '0.0%';

    const tbody = document.querySelector('#waypointTable tbody');
    tbody.innerHTML = '<tr><td colspan="8" class="empty-table-msg">Run planner to generate waypoints</td></tr>';
    document.getElementById('btnSimulate').disabled = true;

    const conf = getPlannerConfig();
    drawStartGoalMarkers(conf.start, conf.goal);
}

function runPlannerInstant() {
    resetSimulator();
    updateStatus('PLANNING...', 'text-planning');
    const conf = getPlannerConfig();
    const startTime = performance.now();

    if (!isNodeValid(conf.start, OBSTACLES, conf.safetyBuffer)) {
        updateStatus('FAILED (START IN OBS)', 'text-failed');
        alert('Error: Start position resides inside an obstacle or its safety buffer!');
        return;
    }

    treeNodes = [conf.start];
    let bestGoalNode = null;
    let minGoalCost = Infinity;

    for (let i = 0; i < conf.maxIter; i++) {
        const rnd = getRandomNode(conf.goal, conf.goalBias);
        const nearestId = getNearestNodeId(treeNodes, rnd);
        const nearest = treeNodes[nearestId];

        const candidates = generatePrimitives(nearest, conf.stepSize, conf.maxYaw, conf.maxPitch);
        let bestCandidate = null;
        let minDistToRnd = Infinity;

        for (const cand of candidates) {
            if (checkCollision(nearest, cand.candidate, OBSTACLES, conf.safetyBuffer)) {
                const d = hitungJarak(cand.candidate, rnd);
                if (d < minDistToRnd) {
                    minDistToRnd = d;
                    bestCandidate = cand.candidate;
                }
            }
        }
        if (bestCandidate === null) continue;
        treeNodes.push(bestCandidate);
        drawTreeBranch(nearest, bestCandidate);

        if (hitungJarak(bestCandidate, conf.goal) <= conf.goalThreshold) {
            if (checkCollision(bestCandidate, conf.goal, OBSTACLES, conf.safetyBuffer)) {
                const cost = bestCandidate.cost + hitungJarak(bestCandidate, conf.goal);
                if (cost < minGoalCost) {
                    minGoalCost = cost;
                    bestGoalNode = new Node(conf.goal.x, conf.goal.y, conf.goal.z, bestCandidate.psi);
                    bestGoalNode.parent = bestCandidate;
                    bestGoalNode.cost = cost;
                }
            }
        }
    }

    const elapsed = performance.now() - startTime;
    if (bestGoalNode) {
        treeNodes.push(bestGoalNode);
        finalPath = extractPath(bestGoalNode);
        drawOptimalPath(finalPath);
        populateResults(elapsed);
    } else {
        updateStatus('FAILED (NO PATH)', 'text-failed');
        document.getElementById('treeNodesVal').textContent = treeNodes.length;
        document.getElementById('timeVal').textContent = Math.round(elapsed) + ' ms';
    }
}

function runPlannerStepped() {
    resetSimulator();
    isPlanning = true;
    updateStatus('EXPANDING...', 'text-planning');
    const conf = getPlannerConfig();
    const startTime = performance.now();

    if (!isNodeValid(conf.start, OBSTACLES, conf.safetyBuffer)) {
        updateStatus('FAILED (START IN OBS)', 'text-failed');
        alert('Error: Start position resides inside an obstacle or its safety buffer!');
        return;
    }

    treeNodes = [conf.start];
    let bestGoalNode = null;
    let minGoalCost = Infinity;
    let iter = 0;
    const batchSize = 15;

    function step() {
        if (!isPlanning) return;
        for (let b = 0; b < batchSize; b++) {
            if (iter >= conf.maxIter) break;

            const rnd = getRandomNode(conf.goal, conf.goalBias);
            const nearestId = getNearestNodeId(treeNodes, rnd);
            const nearest = treeNodes[nearestId];

            const candidates = generatePrimitives(nearest, conf.stepSize, conf.maxYaw, conf.maxPitch);
            let bestCandidate = null;
            let minDistToRnd = Infinity;

            for (const cand of candidates) {
                if (checkCollision(nearest, cand.candidate, OBSTACLES, conf.safetyBuffer)) {
                    const d = hitungJarak(cand.candidate, rnd);
                    if (d < minDistToRnd) {
                        minDistToRnd = d;
                        bestCandidate = cand.candidate;
                    }
                }
            }
            if (bestCandidate !== null) {
                treeNodes.push(bestCandidate);
                drawTreeBranch(nearest, bestCandidate);

                if (hitungJarak(bestCandidate, conf.goal) <= conf.goalThreshold) {
                    if (checkCollision(bestCandidate, conf.goal, OBSTACLES, conf.safetyBuffer)) {
                        const cost = bestCandidate.cost + hitungJarak(bestCandidate, conf.goal);
                        if (cost < minGoalCost) {
                            minGoalCost = cost;
                            bestGoalNode = new Node(conf.goal.x, conf.goal.y, conf.goal.z, bestCandidate.psi);
                            bestGoalNode.parent = bestCandidate;
                            bestGoalNode.cost = cost;
                        }
                    }
                }
            }
            iter++;
        }
        document.getElementById('treeNodesVal').textContent = treeNodes.length;

        if (iter < conf.maxIter) {
            animationFrameId = requestAnimationFrame(step);
        } else {
            const elapsed = performance.now() - startTime;
            if (bestGoalNode) {
                treeNodes.push(bestGoalNode);
                finalPath = extractPath(bestGoalNode);
                drawOptimalPath(finalPath);
                populateResults(elapsed);
            } else {
                updateStatus('FAILED (NO PATH)', 'text-failed');
                document.getElementById('timeVal').textContent = Math.round(elapsed) + ' ms';
            }
        }
    }
    step();
}

// Draw a single branch line of RRT tree
function drawTreeBranch(fromNode, toNode) {
    const points = [];
    points.push(new THREE.Vector3(fromNode.x, fromNode.z, fromNode.y));
    points.push(new THREE.Vector3(toNode.x, toNode.z, toNode.y));
    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({
        color: 0x93c5fd,
        transparent: true,
        opacity: 0.35
    }));
    treeGroup.add(line);
}

// Draw final neon blue optimal path
function drawOptimalPath(path) {
    const points = [];
    path.forEach(node => {
        points.push(new THREE.Vector3(node.x, node.z, node.y));
    });
    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({
        color: 0x00f0ff,
        linewidth: 3
    }));
    pathGroup.add(line);

    try {
        const curve = new THREE.CatmullRomCurve3(points);
        const tubeGeom = new THREE.TubeGeometry(curve, 64, 0.4, 8, false);
        const tubeMat = new THREE.MeshBasicMaterial({
            color: 0x0072ff,
            transparent: true,
            opacity: 0.25,
            wireframe: false
        });
        const tube = new THREE.Mesh(tubeGeom, tubeMat);
        pathGroup.add(tube);
    } catch(e) {}
}

// Generate Stylized 3D Airplane Mesh Model
function createUavModel() {
    const uav = new THREE.Group();
    const bodyGeom = new THREE.CylinderGeometry(0.2, 0.2, 2.0, 8);
    bodyGeom.rotateX(Math.PI / 2);
    const bodyMat = new THREE.MeshPhongMaterial({
        color: 0x00f0ff,
        emissive: 0x0f172a,
        shininess: 100
    });
    const body = new THREE.Mesh(bodyGeom, bodyMat);
    uav.add(body);

    const wingGeom = new THREE.BoxGeometry(3.5, 0.05, 0.5);
    const wing = new THREE.Mesh(wingGeom, bodyMat);
    wing.position.set(0, 0, 0.2);
    uav.add(wing);

    const tailGeom = new THREE.BoxGeometry(1.0, 0.03, 0.3);
    const tail = new THREE.Mesh(tailGeom, bodyMat);
    tail.position.set(0, 0, -0.8);
    uav.add(tail);

    const vTailGeom = new THREE.BoxGeometry(0.03, 0.5, 0.3);
    const vTail = new THREE.Mesh(vTailGeom, bodyMat);
    vTail.position.set(0, 0.25, -0.8);
    uav.add(vTail);

    return uav;
}

// Animate UAV flight along path waypoints
function simulateFlight() {
    if (!finalPath || finalPath.length < 2) return;
    if (flightAnimationId) cancelAnimationFrame(flightAnimationId);

    if (!uavModel) {
        uavModel = createUavModel();
        scene.add(uavModel);
    }
    let currentSegmentIndex = 0;
    let t = 0.0;
    const speed = 0.04;

    function animateFlight() {
        if (currentSegmentIndex >= finalPath.length - 1) {
            uavModel.position.set(
                finalPath[finalPath.length - 1].x,
                finalPath[finalPath.length - 1].z,
                finalPath[finalPath.length - 1].y
            );
            return;
        }
        const nodeA = finalPath[currentSegmentIndex];
        const nodeB = finalPath[currentSegmentIndex + 1];

        const px = nodeA.x + (nodeB.x - nodeA.x) * t;
        const py = nodeA.y + (nodeB.y - nodeA.y) * t;
        const pz = nodeA.z + (nodeB.z - nodeA.z) * t;
        uavModel.position.set(px, pz, py);

        let psiA = nodeA.psi;
        let psiB = nodeB.psi;
        let diff = psiB - psiA;
        diff = ((diff + Math.PI) % (2 * Math.PI)) - Math.PI;
        const currentPsi = psiA + diff * t;

        const dx = nodeB.x - nodeA.x;
        const dy = nodeB.y - nodeA.y;
        const dz = nodeB.z - nodeA.z;
        const d2d = Math.sqrt(dx * dx + dy * dy);
        const pitch = Math.atan2(dz, d2d);
        const roll = -diff * 0.8;

        uavModel.rotation.set(pitch, currentPsi, roll, 'YXZ');
        t += speed;
        if (t >= 1.0) {
            t = 0.0;
            currentSegmentIndex++;
        }
        flightAnimationId = requestAnimationFrame(animateFlight);
    }
    animateFlight();
}

// Display results metrics in Stats and Waypoints Table
function populateResults(elapsedTime) {
    updateStatus('SUCCESS', 'text-success');
    const totalNodes = treeNodes.length;
    const pathCost = finalPath[finalPath.length - 1].cost;
    const waypointsCount = finalPath.length;

    const start = finalPath[0];
    const goal = finalPath[finalPath.length - 1];
    const straightDist = Math.sqrt(
        Math.pow(goal.x - start.x, 2) +
        Math.pow(goal.y - start.y, 2) +
        Math.pow(goal.z - start.z, 2)
    );
    const efficiency = (straightDist / pathCost) * 100;

    document.getElementById('costVal').textContent = pathCost.toFixed(2) + ' m';
    document.getElementById('waypointsVal').textContent = waypointsCount;
    document.getElementById('treeNodesVal').textContent = totalNodes;
    document.getElementById('timeVal').textContent = Math.round(elapsedTime) + ' ms';
    document.getElementById('efficiencyVal').textContent = efficiency.toFixed(1) + '%';

    const tbody = document.querySelector('#waypointTable tbody');
    tbody.innerHTML = '';

    for (let idx = 0; idx < finalPath.length; idx++) {
        const nd = finalPath[idx];
        let dYawDeg = 0.0;
        let dPitchDeg = 0.0;
        let dDist = 0.0;

        if (idx > 0) {
            const prev = finalPath[idx - 1];
            let dyaw = mathDegrees(nd.psi) - mathDegrees(prev.psi);
            dYawDeg = ((dyaw + 180) % 360) - 180;

            const dx = nd.x - prev.x;
            const dy = nd.y - prev.y;
            const dz = nd.z - prev.z;
            const d2d = Math.sqrt(dx * dx + dy * dy);
            dPitchDeg = d2d > 1e-6 ? mathDegrees(Math.atan2(dz, d2d)) : 0.0;
            dDist = hitungJarak(prev, nd);
        }

        const tr = document.createElement('tr');
        const tag = idx === 0 ? ' <span style=\'color:#22c55e\'>(Start)</span>' : (idx === finalPath.length - 1 ? ' <span style=\'color:#eab308\'>(Goal)</span>' : '');
        
        // Use standard concatenation instead of template literals with backticks to prevent JSON issues
        tr.innerHTML = '<td>' + (idx + 1) + tag + '</td>' +
                       '<td>' + nd.x.toFixed(2) + '</td>' +
                       '<td>' + nd.y.toFixed(2) + '</td>' +
                       '<td>' + nd.z.toFixed(2) + '</td>' +
                       '<td>' + mathDegrees(nd.psi).toFixed(1) + '°</td>' +
                       '<td style=\'color:' + (dYawDeg > 0.1 ? '#bd5eff' : (dYawDeg < -0.1 ? '#00f0ff' : 'inherit')) + '\'>' + (dYawDeg > 0 ? '+' : '') + dYawDeg.toFixed(1) + '°</td>' +
                       '<td>' + (dPitchDeg > 0 ? '+' : '') + dPitchDeg.toFixed(1) + '°</td>' +
                       '<td>' + dDist.toFixed(2) + ' m</td>';

        tbody.appendChild(tr);
    }
    document.getElementById('btnSimulate').disabled = false;
}

function updateStatus(text, className) {
    const statusVal = document.getElementById('statusVal');
    statusVal.textContent = text;
    statusVal.className = 'stat-value';
    statusVal.classList.add(className);
}

function mathRadians(degrees) {
    return degrees * (Math.PI / 180);
}

function mathDegrees(radians) {
    return radians * (180 / Math.PI);
}
