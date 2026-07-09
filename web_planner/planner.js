const X_MAX = 100.0;
const Y_MAX = 100.0;
const Z_MAX = 30.0;

/**
 * Node now carries the FULL 12-state 6-DOF state, not just position+yaw.
 * The constructor signature stays (x, y, z, psi, extra) so existing call
 * sites like `new Node(start.x, start.y, start.z, start.psi)` in app.js
 * keep working unchanged -- any state not provided defaults to a steady,
 * wings-level trim condition at the configured cruise airspeed.
 */
class Node {
    constructor(x, y, z, psi = 0.0, extra = {}) {
        this.x = parseFloat(x);
        this.y = parseFloat(y);
        this.z = parseFloat(z);
        this.psi = parseFloat(psi);

        this.phi = extra.phi || 0.0;      // roll angle (rad)
        this.theta = extra.theta || 0.0;  // pitch angle (rad)
        this.u = extra.u !== undefined ? extra.u : Dynamics.AIRCRAFT.Vtrim;
        this.v = extra.v || 0.0;
        this.w = extra.w || 0.0;
        this.p = extra.p || 0.0;
        this.q = extra.q || 0.0;
        this.r = extra.r || 0.0;

        this.parent = null;
        this.cost = 0.0;

        // Bookkeeping added by the dynamics-based planner (all optional /
        // null for nodes that were never produced by simulateControlSet,
        // e.g. the user-defined start and goal nodes).
        this.controls = extra.controls || null;
        this.primitiveName = extra.primitiveName || null;
        this.subTrajectory = extra.subTrajectory || null;
        this.stalled = extra.stalled || false;
    }

    /** Convert to the plain state object Dynamics.* functions expect. */
    toDynState() {
        return {
            u: this.u, v: this.v, w: this.w,
            p: this.p, q: this.q, r: this.r,
            phi: this.phi, theta: this.theta, psi: this.psi,
            x: this.x, y: this.y, z: this.z
        };
    }

    pos() {
        return [this.x, this.y, this.z];
    }
}

// 3D Obstacles list (dynamically generated)
const OBSTACLES = [];

// Incremented every time a candidate primitive is rejected for stalling
// during its integration. Exposed so app.js can surface it in the UI as
// proof that the 6-DOF feasibility check is actually doing something.
let stallRejectCount = 0;
function resetStallCounter() { stallRejectCount = 0; }

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

/**
 * Return up to `k` node indices from `nodeList`, sorted nearest-to-farthest
 * from `rndNode`. This is what powers the "try the next-nearest node if the
 * closest one collides" retry logic in Extend Tree (paper Sect. 6.2 /
 * Algorithm 2), instead of giving up on a sample after a single failure.
 */
function getKNearestNodeIds(nodeList, rndNode, k) {
    const withDist = nodeList.map((node, idx) => ({ idx, d: hitungJarak(node, rndNode) }));
    withDist.sort((a, b) => a.d - b.d);
    return withDist.slice(0, k).map(e => e.idx);
}

/**
 * Analytically solve the level-flight TRIM condition at the configured
 * cruise airspeed: find (alpha, delta_e) that simultaneously satisfy
 * L = W (lift equals weight) and Cm = 0 (zero net pitching moment), then
 * back out the throttle that balances drag at that condition. This is a
 * much smaller version of what the paper does with GPOPS-II trajectory
 * optimization (Sect. 4.1) -- exact here because the level-flight case
 * reduces to two linear equations, whereas the paper solves the full
 * nonlinear 6-DOF optimal control problem for many more trim conditions
 * (turns, climbs, hover).
 */
function solveLevelTrim(P = AIRCRAFT_REF()) {
    const V = P.Vtrim;
    const qbar = 0.5 * P.rho * V * V;
    const W = P.mass * P.g;
    const CL_req = W / (qbar * P.S);

    // [CLalpha  CLde ] [alpha]   [CL_req - CL0]
    // [Cmalpha  Cmde ] [de   ] = [-Cm0        ]
    const a11 = P.CLalpha, a12 = P.CLde, b1 = CL_req - P.CL0;
    const a21 = P.Cmalpha, a22 = P.Cmde, b2 = -P.Cm0;
    const det = a11 * a22 - a12 * a21;
    const alpha = (b1 * a22 - a12 * b2) / det;
    const delta_e = (a11 * b2 - b1 * a21) / det;

    const CD = P.CD0 + P.k * CL_req * CL_req;
    const D = qbar * P.S * CD;
    const throttle = clamp(D / P.thrustMax, 0, 1);

    return { alpha, delta_e, throttle };
}
function AIRCRAFT_REF() { return Dynamics.AIRCRAFT; }

/**
 * Generalize solveLevelTrim() to a STEADY, COORDINATED TURN with a
 * simultaneous climb/descent rate: given a desired yaw rate (psidot) and
 * climb rate (zdot) at airspeed V, solve analytically for the control
 * inputs (and bank angle) that hold that condition steady, the same way
 * the paper solves a grid of 116 trim primitives (Sect. 4.1) -- except we
 * solve it exactly, on demand, for whatever (psidot, zdot) the geometry
 * asks for, rather than rounding to the nearest of a precomputed grid.
 *
 * Steady-turn body rates (phidot = thetadot = 0) reduce to the standard
 * closed-form relations used here:
 *   phi   = atan(V * psidot / g)              (coordinated-turn bank angle)
 *   p     = -psidot * sin(theta)
 *   q     =  psidot * sin(phi) * cos(theta)
 *   r     =  psidot * cos(phi) * cos(theta)
 * Longitudinal trim (alpha, delta_e) is solved the same way as
 * solveLevelTrim, but for the load-factor-scaled CL the bank angle
 * demands. Lateral trim (delta_a, delta_r) is then solved so that the
 * roll and yaw moments are zero at those body rates (assuming zero
 * sideslip, i.e. a coordinated turn).
 *
 * Returns { delta_a, delta_e, delta_r, throttle, phi, feasible }.
 * feasible=false means this (psidot, zdot) combination is outside the
 * aircraft's trim-able envelope (would need more lift or more thrust than
 * physically available) -- reject it before even integrating.
 */
function solveTurnTrim(psidot, zdot, V, P = AIRCRAFT_REF()) {
    const g = P.g;
    const phi = Math.atan2(V * psidot, g);
    const gamma = Math.asin(clamp(zdot / V, -0.9, 0.9)); // flight path angle

    if (Math.abs(phi) > 70 * Math.PI / 180) {
        return { feasible: false };
    }

    const n = 1 / Math.max(0.15, Math.cos(phi)); // load factor from banking
    const W = P.mass * g;
    const qbar = 0.5 * P.rho * V * V;
    const CL_req = (n * W * Math.cos(gamma)) / (qbar * P.S);

    // Longitudinal trim: same 2x2 linear solve as solveLevelTrim, scaled by CL_req
    const a11 = P.CLalpha, a12 = P.CLde, b1 = CL_req - P.CL0;
    const a21 = P.Cmalpha, a22 = P.Cmde, b2 = -P.Cm0;
    const det = a11 * a22 - a12 * a21;
    const alpha = (b1 * a22 - a12 * b2) / det;
    const delta_e = (a11 * b2 - b1 * a21) / det;

    const CL_max = P.CL0 + P.CLalpha * P.stallAlpha + Math.abs(P.CLde) * P.deltaEMax;
    if (CL_req > CL_max || Math.abs(delta_e) > P.deltaEMax * 1.5) {
        return { feasible: false };
    }

    const theta = alpha + gamma;
    const p = -psidot * Math.sin(theta);
    const q = psidot * Math.sin(phi) * Math.cos(theta);
    const r = psidot * Math.cos(phi) * Math.cos(theta);

    // Lateral trim: solve delta_a, delta_r so Cl = 0 and Cn = 0 at (p, r), beta = 0
    const bRef = P.b / (2 * V);
    const lat_b1 = -P.Clp * (p * bRef);
    const lat_b2 = -P.Cnr * (r * bRef);
    const m11 = P.Clda, m12 = P.Cldr;
    const m21 = P.Cnda, m22 = P.Cndr;
    const latDet = m11 * m22 - m12 * m21;
    const delta_a = (lat_b1 * m22 - m12 * lat_b2) / latDet;
    const delta_r = (m11 * lat_b2 - lat_b1 * m21) / latDet;

    if (Math.abs(delta_a) > P.deltaAMax || Math.abs(delta_r) > P.deltaRMax) {
        return { feasible: false };
    }

    const CD = P.CD0 + P.k * CL_req * CL_req;
    const D = qbar * P.S * CD;
    const throttleNeeded = D / P.thrustMax;
    if (throttleNeeded > 1.05) {
        return { feasible: false }; // not enough thrust available to hold this trim
    }

    return {
        delta_a, delta_e, delta_r, throttle: clamp(throttleNeeded, 0, 1),
        phi, theta, p, q, r, feasible: true
    };
}

/**
 * Fast, CLOSED-FORM circular-arc propagation of a trim condition -- no
 * numerical integration. This is what the paper's Steer() actually costs
 * during planning (Sect. 6.3): pure geometry. The nonlinear 6-DOF model is
 * still used, but only ANALYTICALLY inside solveTurnTrim's feasibility
 * check above (O(1) linear algebra, not a simulation) -- mirroring the
 * paper's architecture, where the expensive part (validating a maneuver
 * against the aircraft's real dynamics) happens once per candidate as a
 * closed-form check, and the full nonlinear simulation is reserved for
 * building the maneuver space, not for searching it. An earlier version
 * of this planner ran a full RK4 integration (dozens of physics substeps)
 * for every single candidate, on every retry -- which is why it needed
 * thousands of iterations and multiple seconds to converge, the opposite
 * of "real-time". This version is O(1) per candidate.
 */
function propagateTrimKinematic(fromNode, psidot, zdot, duration, V) {
    const newPsi = fromNode.psi + psidot * duration;
    let newX, newY;
    if (Math.abs(psidot) < 1e-4) {
        newX = fromNode.x + V * Math.cos(fromNode.psi) * duration;
        newY = fromNode.y + V * Math.sin(fromNode.psi) * duration;
    } else {
        const r = V / psidot;
        newX = fromNode.x + r * (Math.sin(newPsi) - Math.sin(fromNode.psi));
        newY = fromNode.y - r * (Math.cos(newPsi) - Math.cos(fromNode.psi));
    }
    return { x: newX, y: newY, z: fromNode.z + zdot * duration, psi: newPsi };
}

/** Cheap sampled points along the same closed-form arc, for collision checking. */
function sampleArcTrajectory(fromNode, psidot, zdot, duration, V, numSamples = 10) {
    const points = [];
    const r = Math.abs(psidot) < 1e-4 ? null : V / psidot;
    for (let i = 0; i <= numSamples; i++) {
        const t = (duration * i) / numSamples;
        const psi_t = fromNode.psi + psidot * t;
        let x, y;
        if (r === null) {
            x = fromNode.x + V * Math.cos(fromNode.psi) * t;
            y = fromNode.y + V * Math.sin(fromNode.psi) * t;
        } else {
            x = fromNode.x + r * (Math.sin(psi_t) - Math.sin(fromNode.psi));
            y = fromNode.y - r * (Math.cos(psi_t) - Math.cos(fromNode.psi));
        }
        points.push({ x, y, z: fromNode.z + zdot * t });
    }
    return points;
}

const MAX_BANK_FOR_STEER = 45 * Math.PI / 180;
const MAX_CLIMB_RATE_FOR_STEER = 4.0; // m/s, a reasonable ceiling for a small UAV

function clampSteerDuration(t) {
    return Math.min(2.2, Math.max(0.3, t));
}

/**
 * The dynamics-aware equivalent of the paper's analytic Steer() (Sect. 6.3,
 * Eq. 3): given the node being steered away from and a target point, work
 * out the circular-arc geometry connecting them, derive the (yaw rate,
 * climb rate, coasting time) that would trace that arc, solve the trim
 * condition for that maneuver analytically, and integrate it via the 6-DOF
 * dynamics for exactly that duration.
 *
 * Unlike generatePrimitives() (which tries a fixed library of candidates
 * and keeps whichever ends up closest), this computes ONE primitive aimed
 * directly at the target -- so, as in the paper, growing the trim library
 * doesn't cost more planning time; there IS no library here, just a solve.
 *
 * Returns the candidate Node, or null if the required maneuver is outside
 * the trim-able envelope (too tight a turn, too steep a climb) or the
 * aircraft would stall/leave the airspace during it -- the caller is
 * expected to fall back to generatePrimitives()'s agile maneuvers in that
 * case, the same way the paper falls back to the aggressive turn-around.
 */
function steerAnalytic(fromNode, target) {
    const V = Math.max(1.0, Math.sqrt(fromNode.u * fromNode.u + fromNode.v * fromNode.v + fromNode.w * fromNode.w));
    const psidotMax = (Dynamics.AIRCRAFT.g * Math.tan(MAX_BANK_FOR_STEER)) / V;

    const dx = target.x - fromNode.x;
    const dy = target.y - fromNode.y;
    const dz = target.z - fromNode.z;
    const d2d = Math.sqrt(dx * dx + dy * dy);

    // Beyond this relative bearing, the circular-arc chord formula below
    // degenerates: sin(thetaL) -> 0 as thetaL -> +-180 deg, which drives
    // the "ideal" psidot toward ZERO (straight line) even though what's
    // actually needed is a hard turn -- exactly backwards. (Verified: a
    // target directly behind produced a straight segment moving AWAY from
    // it.) The paper avoids this by rounding to the nearest of a discrete
    // yaw-rate grid, which naturally saturates to the max available rate
    // in these cases; we replicate that by explicitly commanding a
    // max-rate turn toward the target whenever the bearing is this wide.
    const REORIENT_THRESHOLD = 100 * Math.PI / 180;

    let psidot, duration;
    if (d2d < 0.5) {
        // Target is nearly straight above/below -- steer has no meaningful
        // heading to aim for; treat as a short wings-level segment.
        psidot = 0;
        duration = clampSteerDuration(Math.max(0.5, Math.abs(dz) / V));
    } else {
        let thetaL = Math.atan2(dy, dx) - fromNode.psi;
        thetaL = ((thetaL + Math.PI) % (2 * Math.PI)) - Math.PI;

        if (Math.abs(thetaL) > REORIENT_THRESHOLD) {
            psidot = Math.sign(thetaL) * psidotMax;
            duration = clampSteerDuration(Math.abs(thetaL) / psidotMax);
        } else if (Math.abs(thetaL) < 1e-3) {
            psidot = 0;
            duration = clampSteerDuration(d2d / V);
        } else {
            const r = d2d / (2 * Math.sin(thetaL));
            const L = d2d * thetaL / Math.sin(thetaL);
            psidot = clamp(V / r, -psidotMax, psidotMax);
            duration = clampSteerDuration(Math.abs(L) / V);
        }
    }

    psidot = clamp(psidot, -psidotMax, psidotMax);
    const zdot = clamp(dz / duration, -MAX_CLIMB_RATE_FOR_STEER, MAX_CLIMB_RATE_FOR_STEER);

    // Feasibility is validated analytically (O(1) linear algebra) -- this
    // is the ONLY place the nonlinear aircraft model is consulted per
    // candidate. Rejecting here plays the same role the paper's stall/
    // envelope check does, just without ever needing to simulate anything.
    const trim = solveTurnTrim(psidot, zdot, V);
    if (!trim.feasible) {
        stallRejectCount++;
        return null;
    }

    const s = propagateTrimKinematic(fromNode, psidot, zdot, duration, V);
    if (s.x < 0 || s.x > X_MAX || s.y < 0 || s.y > Y_MAX || s.z < 0 || s.z > Z_MAX) return null;

    const psidotDeg = (psidot * 180 / Math.PI).toFixed(1);
    const candidate = new Node(s.x, s.y, s.z, s.psi, {
        phi: trim.phi, theta: trim.theta,
        u: V, v: 0, w: 0,
        p: trim.p, q: trim.q, r: trim.r,
        controls: { delta_a: trim.delta_a, delta_e: trim.delta_e, delta_r: trim.delta_r, throttle: trim.throttle },
        primitiveName: `Analytic Trim (ψ̇=${psidotDeg}°/s)`,
        subTrajectory: sampleArcTrajectory(fromNode, psidot, zdot, duration, V)
    });
    candidate.parent = fromNode;
    candidate.cost = fromNode.cost + hitungJarak(fromNode, candidate);
    return candidate;
}

/**
 * Fixed control-input sets, loosely analogous to the paper's trim + agile
 * maneuver primitives. Unlike the previous geometric version, these are
 * INTEGRATED through the 6-DOF equations of motion in dynamics.js rather
 * than applied as direct yaw/pitch offsets. "Steady Cruise Straight" uses
 * the analytically-solved level-flight trim above; every other primitive
 * is defined as an offset around that trim baseline, so turns/climbs/
 * dives are physically consistent perturbations rather than arbitrary
 * numbers. A primitive that stalls the aircraft along the way is rejected
 * (see generatePrimitives below).
 *
 * These are still NOT the output of an offline trajectory optimization
 * like GPOPS-II in the paper (that would be the natural Tier-3 follow-up:
 * solve trim conditions for a whole grid of yaw/climb rates, as the paper
 * does with 116 primitives). Over short RRT expansion steps the aircraft
 * is mid-transient (e.g. phugoid response) rather than fully settled into
 * a new steady state -- treat these as a reasonable, honestly-labeled
 * starting point, not validated flight data.
 */
function buildControlPrimitives() {
    const trim = solveLevelTrim();
    const DE = trim.delta_e;
    const TH = trim.throttle;

    return [
        { name: "Steady Cruise Straight",      delta_a: 0.0,   delta_e: DE,        delta_r: 0.0,   throttle: TH },
        { name: "Standard Left Turn",          delta_a: -0.12, delta_e: DE - 0.02, delta_r: -0.03, throttle: TH + 0.05 },
        { name: "Standard Right Turn",         delta_a: 0.12,  delta_e: DE - 0.02, delta_r: 0.03,  throttle: TH + 0.05 },
        { name: "Standard Gentle Climb",       delta_a: 0.0,   delta_e: DE + 0.05, delta_r: 0.0,   throttle: TH + 0.15 },
        { name: "Standard Gentle Descent",     delta_a: 0.0,   delta_e: DE - 0.05, delta_r: 0.0,   throttle: Math.max(0, TH - 0.03) },
        { name: "Knife-Edge Sharp Turn Left",  delta_a: -0.30, delta_e: DE,        delta_r: -0.12, throttle: TH + 0.10 },
        { name: "Knife-Edge Sharp Turn Right", delta_a: 0.30,  delta_e: DE,        delta_r: 0.12,  throttle: TH + 0.10 },
        { name: "Agile Climb (Aggressive)",    delta_a: 0.0,   delta_e: DE + 0.20, delta_r: 0.0,   throttle: TH + 0.40 },
        { name: "Agile Dive (Aggressive)",     delta_a: 0.0,   delta_e: DE - 0.15, delta_r: 0.0,   throttle: Math.max(0, TH - 0.04) },
        { name: "Agile Climbing Left Turn",    delta_a: -0.20, delta_e: DE + 0.12, delta_r: -0.06, throttle: TH + 0.25 },
        { name: "Agile Climbing Right Turn",   delta_a: 0.20,  delta_e: DE + 0.12, delta_r: 0.06,  throttle: TH + 0.25 },
    ];
}

function clampDuration(t) {
    return Math.min(3.0, Math.max(0.3, t));
}

/**
 * Generate successor candidates by INTEGRATING the 6-DOF dynamics for each
 * control primitive over a short fixed duration, instead of geometrically
 * projecting a step from yaw/pitch multipliers.
 *
 * stepDuration is derived from stepSize / current airspeed, so the
 * "Step Size" UI slider keeps roughly the same physical meaning as before
 * (how far, in meters, one tree expansion tends to reach).
 *
 * maxYaw / maxPitch are still accepted (for signature compatibility with
 * app.js) but are no longer used to directly set the new heading -- the
 * dynamics integration determines the resulting heading/attitude on its
 * own, constrained by the aircraft's actual control authority.
 */
function generatePrimitives(fromNode, stepSize, maxYaw, maxPitch) {
    const candidates = [];
    const fromState = fromNode.toDynState();
    const V = Math.max(1.0, Math.sqrt(fromNode.u * fromNode.u + fromNode.v * fromNode.v + fromNode.w * fromNode.w));
    const stepDuration = clampDuration(stepSize / V);
    const controlPrimitives = buildControlPrimitives(); // tracks the current Trim Airspeed slider

    for (const prim of controlPrimitives) {
        const controls = {
            delta_a: prim.delta_a,
            delta_e: prim.delta_e,
            delta_r: prim.delta_r,
            throttle: prim.throttle
        };

        const sim = Dynamics.simulateControlSet(fromState, controls, stepDuration);

        if (!sim.feasible) {
            stallRejectCount++;
            continue; // aircraft stalled at some point during this maneuver
        }

        const s = sim.finalState;
        if (s.x < 0 || s.x > X_MAX || s.y < 0 || s.y > Y_MAX || s.z < 0 || s.z > Z_MAX) {
            continue; // left the airspace box
        }

        const candidate = new Node(s.x, s.y, s.z, s.psi, {
            phi: s.phi, theta: s.theta,
            u: s.u, v: s.v, w: s.w,
            p: s.p, q: s.q, r: s.r,
            controls, primitiveName: prim.name,
            subTrajectory: sim.trajectory
        });
        candidate.parent = fromNode;
        candidate.cost = fromNode.cost + hitungJarak(fromNode, candidate);
        candidates.push({ candidate, name: prim.name });
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

function linearSamplePoints(fromNode, toNode) {
    const dist = hitungJarak(fromNode, toNode);
    const steps = Math.max(2, Math.floor(dist / 0.5));
    const points = [];
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        points.push({
            x: fromNode.x + (toNode.x - fromNode.x) * t,
            y: fromNode.y + (toNode.y - fromNode.y) * t,
            z: fromNode.z + (toNode.z - fromNode.z) * t
        });
    }
    return points;
}

/**
 * Collision check against the 3D obstacle list.
 *
 * If `trajectory` is supplied (the sub-sampled {x,y,z} points produced by
 * simulateControlSet for a dynamics-integrated candidate), those exact
 * points are checked -- this is more faithful than a straight line when
 * the maneuver curves. If omitted (e.g. checking a direct connection to
 * the goal, which is not itself dynamically integrated), it falls back to
 * linear interpolation between fromNode and toNode, same as before.
 */
function checkCollision(fromNode, toNode, obstacles, buffer, trajectory = null) {
    const points = trajectory || linearSamplePoints(fromNode, toNode);
    for (const pt of points) {
        for (const box of obstacles) {
            if (pointInInflatedBox(pt.x, pt.y, pt.z, box, buffer)) {
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