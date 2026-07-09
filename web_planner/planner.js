const X_MAX = 100.0;
const Y_MAX = 100.0;
const Z_MAX = 30.0;

class Node {
    constructor(x, y, z, psi = 0.0) {
        this.x = parseFloat(x);
        this.y = parseFloat(y);
        this.z = parseFloat(z);
        this.psi = parseFloat(psi); // heading in radians
        this.parent = null;
        this.cost = 0.0;
    }

    pos() {
        return [this.x, this.y, this.z];
    }
}

// 3D Obstacles list (dynamically generated)
const OBSTACLES = [];

function hitungJarak(nodeA, nodeB) {
    return Math.sqrt(
        Math.pow(nodeA.x - nodeB.x, 2) +
        Math.pow(nodeA.y - nodeB.y, 2) +
        Math.pow(nodeA.z - nodeB.z, 2)
    );
}

function getRandomNode(goal, goalSampleRate) {
    if (Math.random() < goalSampleRate) {
        return new Node(goal.x, goal.y, goal.z);
    }
    return new Node(
        Math.random() * X_MAX,
        Math.random() * Y_MAX,
        Math.random() * Z_MAX
    );
}

function getNearestNodeId(nodeList, rndNode) {
    let minId = 0;
    let minDist = Infinity;
    for (let i = 0; i < nodeList.length; i++) {
        const d = hitungJarak(nodeList[i], rndNode);
        if (d < minDist) {
            minDist = d;
            minId = i;
        }
    }
    return minId;
}

function steer(fromNode, toNode, stepSize, maxYaw, maxPitch) {
    const dx = toNode.x - fromNode.x;
    const dy = toNode.y - fromNode.y;
    const dz = toNode.z - fromNode.z;
    const dist3d = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (dist3d < 1e-6) {
        const n = new Node(toNode.x, toNode.y, toNode.z, fromNode.psi);
        n.parent = fromNode;
        n.cost = fromNode.cost;
        return n;
    }

    const desiredPsi = Math.atan2(dy, dx);
    const dist2d = Math.sqrt(dx * dx + dy * dy);
    const desiredPitch = dist2d > 1e-6 ? Math.atan2(dz, dist2d) : Math.sign(dz) * (Math.PI / 2);

    // Yaw change limit
    let yawError = desiredPsi - fromNode.psi;
    yawError = ((yawError + Math.PI) % (2 * Math.PI)) - Math.PI;

    let actualYawChange = yawError;
    if (Math.abs(yawError) > maxYaw) {
        actualYawChange = Math.sign(yawError) * maxYaw;
    }

    let newPsi = fromNode.psi + actualYawChange;
    newPsi = ((newPsi + Math.PI) % (2 * Math.PI)) - Math.PI;

    // Pitch change limit
    let newPitch = desiredPitch;
    if (Math.abs(desiredPitch) > maxPitch) {
        newPitch = Math.sign(desiredPitch) * maxPitch;
    }

    const actualStep = Math.min(stepSize, dist3d);
    let newX = fromNode.x + actualStep * Math.cos(newPitch) * Math.cos(newPsi);
    let newY = fromNode.y + actualStep * Math.cos(newPitch) * Math.sin(newPsi);
    let newZ = fromNode.z + actualStep * Math.sin(newPitch);

    newX = Math.max(0.0, Math.min(X_MAX, newX));
    newY = Math.max(0.0, Math.min(Y_MAX, newY));
    newZ = Math.max(0.0, Math.min(Z_MAX, newZ));

    const newNode = new Node(newX, newY, newZ, newPsi);
    newNode.parent = fromNode;
    newNode.cost = fromNode.cost + hitungJarak(fromNode, newNode);
    return newNode;
}

function generatePrimitives(fromNode, stepSize, maxYaw, maxPitch) {
    const candidates = [];
    const maneuverRatios = [
        { name: "Steady Cruise Straight",       yawMult: 0.0,  pitchMult: 0.0 },
        { name: "Standard Left Turn",          yawMult: -0.6, pitchMult: 0.0 },
        { name: "Standard Right Turn",          yawMult: 0.6,  pitchMult: 0.0 },
        { name: "Standard Gentle Climb",        yawMult: 0.0,  pitchMult: 0.5 },
        { name: "Standard Gentle Descent",      yawMult: 0.0,  pitchMult: -0.5 },
        
        // Agile maneuvers
        { name: "Knife-Edge Sharp Turn Left",   yawMult: -1.7, pitchMult: 0.0 },
        { name: "Knife-Edge Sharp Turn Right",  yawMult: 1.7,  pitchMult: 0.0 },
        { name: "Agile Climb (Aggressive)",     yawMult: 0.0,  pitchMult: 1.6 },
        { name: "Agile Dive (Aggressive)",      yawMult: 0.0,  pitchMult: -1.6 },
        { name: "Agile Climbing Left Turn",     yawMult: -1.1, pitchMult: 0.8 },
        { name: "Agile Climbing Right Turn",    yawMult: 1.1,  pitchMult: 0.8 }
    ];

    for (const ratio of maneuverRatios) {
        const dyaw = ratio.yawMult * maxYaw;
        const dpitch = ratio.pitchMult * maxPitch;

        let newPsi = fromNode.psi + dyaw;
        newPsi = ((newPsi + Math.PI) % (2 * Math.PI)) - Math.PI;

        let newX = fromNode.x + stepSize * Math.cos(dpitch) * Math.cos(newPsi);
        let newY = fromNode.y + stepSize * Math.cos(dpitch) * Math.sin(newPsi);
        let newZ = fromNode.z + stepSize * Math.sin(dpitch);

        newX = Math.max(0.0, Math.min(X_MAX, newX));
        newY = Math.max(0.0, Math.min(Y_MAX, newY));
        newZ = Math.max(0.0, Math.min(Z_MAX, newZ));

        const candidate = new Node(newX, newY, newZ, newPsi);
        candidate.parent = fromNode;
        const dist = Math.sqrt(
            Math.pow(newX - fromNode.x, 2) +
            Math.pow(newY - fromNode.y, 2) +
            Math.pow(newZ - fromNode.z, 2)
        );
        candidate.cost = fromNode.cost + dist;
        candidates.push({ candidate, name: ratio.name });
    }
    return candidates;
}

function pointInInflatedBox(x, y, z, box, buffer) {
    return (
        (box.xMin - buffer) <= x && x <= (box.xMax + buffer) &&
        (box.yMin - buffer) <= y && y <= (box.yMax + buffer) &&
        (box.zMin - buffer) <= z && z <= (box.zMax + buffer)
    );
}

// Tabrakan dengan daftar rintangan 3D
function checkCollision(fromNode, toNode, obstacles, buffer) {
    const dist = hitungJarak(fromNode, toNode);
    const steps = Math.max(2, Math.floor(dist / 0.5));

    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const x = fromNode.x + (toNode.x - fromNode.x) * t;
        const y = fromNode.y + (toNode.y - fromNode.y) * t;
        const z = fromNode.z + (toNode.z - fromNode.z) * t;
        for (const box of obstacles) {
            if (pointInInflatedBox(x, y, z, box, buffer)) {
                return false;
            }
        }
    }
    return true;
}

function isNodeValid(node, obstacles, buffer) {
    for (const box of obstacles) {
        if (pointInInflatedBox(node.x, node.y, node.z, box, buffer)) {
            return false;
        }
    }
    return true;
}

// Memeriksa apakah segmen baru terlalu dekat dengan jalur UAV lain yang sudah ada
function checkUavCollision(fromNode, toNode, existingPaths, minSeparation) {
    const dist = hitungJarak(fromNode, toNode);
    const steps = Math.max(2, Math.floor(dist / 0.5));

    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const x = fromNode.x + (toNode.x - fromNode.x) * t;
        const y = fromNode.y + (toNode.y - fromNode.y) * t;
        const z = fromNode.z + (toNode.z - fromNode.z) * t;

        for (const path of existingPaths) {
            for (const otherNode of path) {
                const d = Math.sqrt(
                    Math.pow(x - otherNode.x, 2) +
                    Math.pow(y - otherNode.y, 2) +
                    Math.pow(z - otherNode.z, 2)
                );
                if (d < minSeparation) {
                    return false; // Terlalu dekat dengan jalur UAV lain (tabrakan)
                }
            }
        }
    }
    return true; // Aman
}

function extractPath(goalNode) {
    const path = [];
    let current = goalNode;
    while (current !== null) {
        path.push(current);
        current = current.parent;
    }
    path.reverse();
    return path;
}