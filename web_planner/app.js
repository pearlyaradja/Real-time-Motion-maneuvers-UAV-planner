let scene, camera, renderer, controls;
let obstaclesGroup, treeGroup, pathGroup, markersGroup;
let uavModels = [];
let treeNodes = [];
let finalPaths = [];
let finalPathUavIndex = []; // finalPaths[i] belongs to UAV number finalPathUavIndex[i] (0-based)
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
        { id: 'trimSpeed', valId: 'trimSpeedVal', suffix: ' m/s' },
        { id: 'maxYaw', valId: 'maxYawVal', suffix: '°' },
        { id: 'maxPitch', valId: 'maxPitchVal', suffix: '°' },
        { id: 'safetyBuffer', valId: 'safetyBufferVal', suffix: 'm' },
        { id: 'obstacleCount', valId: 'obstacleCountVal', suffix: '' },
        { id: 'uavCount', valId: 'uavCountVal', suffix: '' },
        { id: 'flightSpeed', valId: 'flightSpeedVal', suffix: 'x' }
    ];
    sliders.forEach(s => {
        const sliderEl = document.getElementById(s.id);
        const valEl = document.getElementById(s.valId);
        sliderEl.addEventListener('input', () => {
            valEl.textContent = sliderEl.value + s.suffix;
            if (s.id === 'trimSpeed') {
                Dynamics.AIRCRAFT.Vtrim = parseFloat(sliderEl.value);
                resetSimulator();
            } else if (s.id === 'safetyBuffer') {
                updateObstaclesBuffer(parseFloat(sliderEl.value));
            } else if (s.id === 'obstacleCount') {
                generateRandomObstacles(parseInt(sliderEl.value));
                updateObstaclesBuffer(parseFloat(document.getElementById('safetyBuffer').value));
                resetSimulator();
            } else if (s.id === 'uavCount') {
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

    document.getElementById('chkHideTree').addEventListener('change', (e) => {
        treeGroup.visible = !e.target.checked;
    });

    Dynamics.AIRCRAFT.Vtrim = parseFloat(document.getElementById('trimSpeed').value);
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
    OBSTACLES.length = 0;
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

        const startOverlap = (
            (xMin - startBuf) <= startX && startX <= (xMax + startBuf) &&
            (yMin - startBuf) <= startY && startY <= (yMax + startBuf) &&
            (zMin - startBuf) <= startZ && startZ <= (zMax + startBuf)
        );

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

function drawStartGoalMarkers(start, goal, uavCount = 1) {
    while(markersGroup.children.length > 0) {
        markersGroup.remove(markersGroup.children[0]);
    }
    const pathColors = [0x00f0ff, 0xbd5eff, 0x39ff14, 0xff0055, 0xeab308, 0xff7700];
    const spacing = 3.5;

    for (let i = 0; i < uavCount; i++) {
        const offset = (i - (uavCount - 1) / 2) * spacing;
        const start_i = { x: start.x, y: start.y + offset, z: start.z };
        const goal_i = { x: goal.x, y: goal.y + offset, z: goal.z };
        const color = pathColors[i % pathColors.length];

        const startGeom = new THREE.SphereGeometry(1.0, 16, 16);
        const startMat = new THREE.MeshPhongMaterial({ color: color, emissive: 0x052e16 });
        const startMesh = new THREE.Mesh(startGeom, startMat);
        startMesh.position.set(start_i.x, start_i.z, start_i.y);
        markersGroup.add(startMesh);

        const goalGeom = new THREE.SphereGeometry(1.2, 16, 16);
        const goalMat = new THREE.MeshPhongMaterial({ color: color, emissive: 0x422006 });
        const goalMesh = new THREE.Mesh(goalGeom, goalMat);
        goalMesh.position.set(goal_i.x, goal_i.z, goal_i.y);
        markersGroup.add(goalMesh);

        const threshGeom = new THREE.SphereGeometry(parseFloat(document.getElementById('goalThreshold').value), 16, 8);
        const threshWire = new THREE.LineSegments(
            new THREE.EdgesGeometry(threshGeom),
            new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: 0.12 })
        );
        threshWire.position.set(goal_i.x, goal_i.z, goal_i.y);
        markersGroup.add(threshWire);
    }
}

function resetSimulator() {
    isPlanning = false;
    resetStallCounter();
    if (animationFrameId) cancelAnimationFrame(animationFrameId);
    if (flightAnimationId) cancelAnimationFrame(flightAnimationId);

    while(treeGroup.children.length > 0) treeGroup.remove(treeGroup.children[0]);
    while(pathGroup.children.length > 0) pathGroup.remove(pathGroup.children[0]);
    uavModels.forEach(model => {
        scene.remove(model);
    });
    uavModels = [];
    treeNodes = [];
    finalPaths = [];
    finalPathUavIndex = [];
    finalPath = null;

    treeGroup.visible = !document.getElementById('chkHideTree').checked;

    updateStatus('READY', 'text-ready');
    document.getElementById('costVal').textContent = '0.00 m';
    document.getElementById('waypointsVal').textContent = '0';
    document.getElementById('treeNodesVal').textContent = '0';
    document.getElementById('timeVal').textContent = '0 ms';
    document.getElementById('efficiencyVal').textContent = '0.0%';
    document.getElementById('stallRejectsVal').textContent = '0';

    const tbody = document.querySelector('#waypointTable tbody');
    tbody.innerHTML = '<tr><td colspan="9" class="empty-table-msg">Run planner to generate waypoints</td></tr>';
    document.getElementById('btnSimulate').disabled = true;

    const uavSelectContainer = document.getElementById('uavSelectContainer');
    if (uavSelectContainer) {
        uavSelectContainer.style.display = 'none';
    }

    const conf = getPlannerConfig();
    const uavCount = parseInt(document.getElementById('uavCount').value);
    drawStartGoalMarkers(conf.start, conf.goal, uavCount);
}

/**
 * Shared RRT expansion loop, batched via requestAnimationFrame so the
 * browser tab never freezes and the tree/status update live while the
 * search runs -- regardless of whether it was triggered from "Run
 * Planner" (large batchSize, finishes fast) or "Animate Tree" (small
 * batchSize, deliberately slow so you can watch it grow).
 */
function runPlannerBatched(batchSize) {
    isPlanning = true;
    updateStatus('SEARCHING... 0%', 'text-planning');

    const conf = getPlannerConfig();
    const uavCount = parseInt(document.getElementById('uavCount').value);
    const startTime = performance.now();

    const trees = [];
    const bestGoalNodes = [];
    const minGoalCosts = [];
    const iters = [];
    // Only an UAV's ACTUAL flown path (once found) is used to keep later
    // UAVs clear of it. Using the whole raw search tree here (as before)
    // meant every failed/unused exploration branch counted as an
    // "obstacle" for the next UAV, which made it nearly impossible for
    // UAV 2+ to find any path at all.
    const solvedPaths = [];
    const pathColors = [0x00f0ff, 0xbd5eff, 0x39ff14, 0xff0055, 0xeab308, 0xff7700];
    const spacing = 3.5;
    const minSeparation = 2.5;
    const goalNodes = [];

    for (let i = 0; i < uavCount; i++) {
        const offset = (i - (uavCount - 1) / 2) * spacing;
        const start_i = new Node(conf.start.x, conf.start.y + offset, conf.start.z, conf.start.psi);
        const goal_i = new Node(conf.goal.x, conf.goal.y + offset, conf.goal.z);
        goalNodes.push(goal_i);

        if (!isNodeValid(start_i, OBSTACLES, conf.safetyBuffer)) {
            alert(`Warning: UAV ${i + 1} start position resides inside an obstacle or safety buffer!`);
        }

        trees.push([start_i]);
        bestGoalNodes.push(null);
        minGoalCosts.push(Infinity);
        iters.push(0);
        solvedPaths.push(null);
    }

    function step() {
        if (!isPlanning) return;

        let activeTreesCount = 0;

        for (let uavIdx = 0; uavIdx < uavCount; uavIdx++) {
            const localTreeNodes = trees[uavIdx];
            const color = pathColors[uavIdx % pathColors.length];
            const goal_i = goalNodes[uavIdx];

            if (!isNodeValid(localTreeNodes[0], OBSTACLES, conf.safetyBuffer)) {
                continue;
            }

            if (iters[uavIdx] < conf.maxIter) {
                activeTreesCount++;
                // Only prior UAVs that have ALREADY found a real path
                // constrain this one -- computed once per UAV per frame,
                // not per-candidate, to keep this cheap.
                const priorSolvedPaths = solvedPaths.slice(0, uavIdx).filter(p => p !== null);

                // Ekspansi satu batch node untuk UAV saat ini
                for (let b = 0; b < batchSize; b++) {
                    if (iters[uavIdx] >= conf.maxIter) break;

                    const rnd = getRandomNode(goal_i, conf.goalBias);

                    // Retry up to 5 nearest nodes (paper Sect. 6.2 / Algorithm
                    // 2) instead of giving up on this sample after a single
                    // collision. The nearest node first tries the analytic
                    // steer (paper's Steer(), Eq. 3); if THAT nearest node's
                    // steer collides, we additionally try the fixed agile
                    // maneuver library from that same node before moving on
                    // -- mirroring the paper's aggressive-turn-around trigger.
                    const kNearestIds = getKNearestNodeIds(localTreeNodes, rnd, 5);
                    let bestCandidate = null;
                    let usedNearest = null;

                    for (let ni = 0; ni < kNearestIds.length; ni++) {
                        const nearest = localTreeNodes[kNearestIds[ni]];
                        const analytic = steerAnalytic(nearest, rnd);
                        if (analytic &&
                            checkCollision(nearest, analytic, OBSTACLES, conf.safetyBuffer, analytic.subTrajectory) &&
                            checkUavCollision(nearest, analytic, priorSolvedPaths, minSeparation)) {
                            bestCandidate = analytic;
                            usedNearest = nearest;
                            break;
                        }

                        if (ni === 0) {
                            const fallbackCandidates = generatePrimitives(nearest, conf.stepSize, conf.maxYaw, conf.maxPitch);
                            let fb = null, fbBestD = Infinity;
                            for (const cand of fallbackCandidates) {
                                if (checkCollision(nearest, cand.candidate, OBSTACLES, conf.safetyBuffer, cand.candidate.subTrajectory) &&
                                    checkUavCollision(nearest, cand.candidate, priorSolvedPaths, minSeparation)) {
                                    const d = hitungJarak(cand.candidate, rnd);
                                    if (d < fbBestD) { fbBestD = d; fb = cand.candidate; }
                                }
                            }
                            if (fb) {
                                bestCandidate = fb;
                                usedNearest = nearest;
                                break;
                            }
                        }
                    }

                    if (bestCandidate !== null) {
                        const nearest = usedNearest;
                        localTreeNodes.push(bestCandidate);
                        drawTreeBranch(nearest, bestCandidate, color);

                        if (hitungJarak(bestCandidate, goal_i) <= conf.goalThreshold) {
                            if (checkCollision(bestCandidate, goal_i, OBSTACLES, conf.safetyBuffer) &&
                                checkUavCollision(bestCandidate, goal_i, priorSolvedPaths, minSeparation)) {
                                const cost = bestCandidate.cost + hitungJarak(bestCandidate, goal_i);
                                if (cost < minGoalCosts[uavIdx]) {
                                    minGoalCosts[uavIdx] = cost;
                                    const goalNode = new Node(goal_i.x, goal_i.y, goal_i.z, bestCandidate.psi);
                                    goalNode.parent = bestCandidate;
                                    goalNode.cost = cost;
                                    bestGoalNodes[uavIdx] = goalNode;
                                    solvedPaths[uavIdx] = extractPath(goalNode);
                                }
                            }
                        }
                    }
                    iters[uavIdx]++;
                }
            }
        }

        // Live progress feedback -- this is what was missing before.
        const totalNodes = trees.reduce((sum, t) => sum + t.length, 0);
        const totalItersDone = iters.reduce((sum, n) => sum + n, 0);
        const totalBudget = uavCount * conf.maxIter;
        const pct = totalBudget > 0 ? Math.min(100, Math.round(100 * totalItersDone / totalBudget)) : 100;
        document.getElementById('treeNodesVal').textContent = totalNodes;
        document.getElementById('stallRejectsVal').textContent = stallRejectCount;
        updateStatus(`SEARCHING... ${pct}%`, 'text-planning');

        if (activeTreesCount > 0) {
            animationFrameId = requestAnimationFrame(step);
        } else {
            const elapsed = performance.now() - startTime;
            let pathsFoundCount = 0;

            for (let uavIdx = 0; uavIdx < uavCount; uavIdx++) {
                const goalNode = bestGoalNodes[uavIdx];
                if (goalNode) {
                    trees[uavIdx].push(goalNode);
                    const path = extractPath(goalNode);
                    finalPaths.push(path);
                    finalPathUavIndex.push(uavIdx);

                    const color = pathColors[uavIdx % pathColors.length];
                    drawOptimalPath(path, color);
                    pathsFoundCount++;
                }
            }

            document.getElementById('stallRejectsVal').textContent = stallRejectCount;
            if (pathsFoundCount > 0) {
                treeGroup.visible = !document.getElementById('chkHideTree').checked;
                populateResults(elapsed, totalNodes, pathsFoundCount, uavCount);
            } else {
                updateStatus('FAILED (NO PATH)', 'text-failed');
                document.getElementById('timeVal').textContent = Math.round(elapsed) + ' ms';
            }
        }
    }
    step();
}

// "Run Planner": big batch per animation frame -- finishes fast, but still
// yields to the browser every frame so the tree visibly grows and the
// status bar shows live progress instead of freezing the tab.
function runPlannerInstant() {
    resetSimulator();
    runPlannerBatched(60);
}

// "Animate Tree": small batch per animation frame -- deliberately slow so
// you can watch each expansion happen, useful for explaining the
// algorithm.
function runPlannerStepped() {
    resetSimulator();
    runPlannerBatched(10);
}

// Menggambar garis cabang RRT dengan warna unik masing-masing jalur UAV
function drawTreeBranch(fromNode, toNode, color = 0x93c5fd) {
    const points = [
        new THREE.Vector3(fromNode.x, fromNode.z, fromNode.y),
        new THREE.Vector3(toNode.x, toNode.z, toNode.y)
    ];
    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({
        color: color,
        transparent: true,
        opacity: 0.25
    }));
    treeGroup.add(line);
}

function drawOptimalPath(path, color = 0x00f0ff) {
    const points = [];
    path.forEach(node => {
        points.push(new THREE.Vector3(node.x, node.z, node.y));
    });
    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({
        color: color,
        linewidth: 3
    }));
    pathGroup.add(line);

    try {
        const curve = new THREE.CatmullRomCurve3(points);
        const tubeGeom = new THREE.TubeGeometry(curve, 64, 0.4, 8, false);
        const tubeMat = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.2
        });
        const mesh = new THREE.Mesh(tubeGeom, tubeMat);
        pathGroup.add(mesh);
    } catch(e) {}
}

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

function simulateFlight() {
    if (finalPaths.length === 0) return;
    if (flightAnimationId) cancelAnimationFrame(flightAnimationId);

    uavModels.forEach(model => scene.remove(model));
    uavModels = [];

    const uavCount = parseInt(document.getElementById('uavCount').value);

    for (let i = 0; i < uavCount; i++) {
        const uav = createUavModel();
        if (i > 0) {
            const colors = [0xbd5eff, 0x39ff14, 0xff0055, 0xeab308, 0xff7700];
            const uavColor = colors[(i - 1) % colors.length];
            uav.children.forEach(mesh => {
                if (mesh.material) mesh.material.color.setHex(uavColor);
            });
        }
        uav.visible = false;
        scene.add(uav);
        uavModels.push(uav);
    }

    const flightCurves = finalPaths.map(path => {
        const points = path.map(node => new THREE.Vector3(node.x, node.z, node.y));
        return new THREE.CatmullRomCurve3(points);
    });

    let u = 0.0;
    const speedMultiplier = parseFloat(document.getElementById('flightSpeed').value);
    const speed = 0.001 * speedMultiplier;
    const delayOffset = finalPaths.length > 1 ? 0.0 : 0.08;

    const prevYaws = new Array(uavCount).fill(0);
    const isFirstFrames = new Array(uavCount).fill(true);

    function animateFlight() {
        u += speed;

        const lastUavProgress = u - (uavCount - 1) * delayOffset;
        if (lastUavProgress > 1.0) {
            u = 0.0;
            isFirstFrames.fill(true);
        }

        for (let i = 0; i < uavCount; i++) {
            const uav = uavModels[i];
            const u_i = u - i * delayOffset;

            if (u_i >= 0.0 && u_i <= 1.0) {
                uav.visible = true;
                const curve = flightCurves[i % flightCurves.length];
                const pos = curve.getPointAt(u_i);
                
                uav.position.set(pos.x, pos.y, pos.z);

                const tangent = curve.getTangentAt(u_i).normalize();
                const target = new THREE.Vector3(pos.x, pos.y, pos.z).add(tangent);
                uav.lookAt(target);

                const currentYaw = Math.atan2(tangent.x, tangent.z);
                if (isFirstFrames[i]) {
                    prevYaws[i] = currentYaw;
                    isFirstFrames[i] = false;
                }

                let yawDiff = currentYaw - prevYaws[i];
                yawDiff = ((yawDiff + Math.PI) % (2 * Math.PI)) - Math.PI;

                const bankAngle = Math.max(-0.6, Math.min(0.6, yawDiff * 12.0));
                uav.rotateOnAxis(new THREE.Vector3(0, 0, 1), bankAngle);

                prevYaws[i] = currentYaw;
            } else {
                uav.visible = false;
            }
        }
        flightAnimationId = requestAnimationFrame(animateFlight);
    }
    animateFlight();
}

function populateResults(elapsedTime, totalNodes, pathsFoundCount, uavCountTotal) {
    if (pathsFoundCount !== undefined && uavCountTotal !== undefined && pathsFoundCount < uavCountTotal) {
        updateStatus(`PARTIAL (${pathsFoundCount}/${uavCountTotal} UAVs)`, 'text-planning');
    } else {
        updateStatus('SUCCESS', 'text-success');
    }
    if (totalNodes === undefined) totalNodes = treeNodes.length;
    document.getElementById('treeNodesVal').textContent = totalNodes;
    document.getElementById('timeVal').textContent = Math.round(elapsedTime) + ' ms';

    const uavSelectContainer = document.getElementById('uavSelectContainer');
    const uavPathSelect = document.getElementById('uavPathSelect');
    
    if (uavSelectContainer && uavPathSelect) {
        if (finalPaths.length > 1) {
            uavPathSelect.innerHTML = '';
            const colorNames = ["Cyan", "Purple", "Green", "Pink", "Yellow", "Orange"];
            finalPaths.forEach((path, idx) => {
                const trueUavNumber = (finalPathUavIndex[idx] !== undefined ? finalPathUavIndex[idx] : idx) + 1;
                const colorIdx = (finalPathUavIndex[idx] !== undefined ? finalPathUavIndex[idx] : idx) % colorNames.length;
                const option = document.createElement('option');
                option.value = idx;
                option.textContent = `UAV ${trueUavNumber} (${colorNames[colorIdx]})`;
                uavPathSelect.appendChild(option);
            });
            uavSelectContainer.style.display = 'flex';
            uavPathSelect.onchange = () => displayPathStatsAndTable(parseInt(uavPathSelect.value));
        } else {
            uavSelectContainer.style.display = 'none';
        }
    }
    displayPathStatsAndTable(0);
    document.getElementById('btnSimulate').disabled = false;
}

function displayPathStatsAndTable(pathIndex) {
    if (finalPaths.length === 0 || !finalPaths[pathIndex]) return;
    const finalPath = finalPaths[pathIndex];
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
    document.getElementById('efficiencyVal').textContent = efficiency.toFixed(1) + '%';

    const tbody = document.querySelector('#waypointTable tbody');
    tbody.innerHTML = '';

    for (let idx = 0; idx < finalPath.length; idx++) {
        const nd = finalPath[idx];
        let dYawDeg = 0.0, dPitchDeg = 0.0, dDist = 0.0;

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
        const tag = idx === 0 ? ' <span style="color:#22c55e">(Start)</span>' : (idx === finalPath.length - 1 ? ' <span style="color:#eab308">(Goal)</span>' : '');
        const primLabel = nd.primitiveName || (idx === 0 ? '—' : '(direct)');

        tr.innerHTML = `<td>${idx + 1}${tag}</td>` +
                       `<td>${nd.x.toFixed(2)}</td>` +
                       `<td>${nd.y.toFixed(2)}</td>` +
                       `<td>${nd.z.toFixed(2)}</td>` +
                       `<td>${mathDegrees(nd.psi).toFixed(1)}°</td>` +
                       `<td style="color:${dYawDeg > 0.1 ? '#bd5eff' : (dYawDeg < -0.1 ? '#00f0ff' : 'inherit')}">${dYawDeg > 0 ? '+' : ''}${dYawDeg.toFixed(1)}°</td>` +
                       `<td>${dPitchDeg > 0 ? '+' : ''}${dPitchDeg.toFixed(1)}°</td>` +
                       `<td>${dDist.toFixed(2)} m</td>` +
                       `<td style="color:#94a3b8; font-size:11px;">${primLabel}</td>`;
        tbody.appendChild(tr);
    }
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