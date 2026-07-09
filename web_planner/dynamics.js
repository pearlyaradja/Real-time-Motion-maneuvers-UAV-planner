/* dynamics.js
 * ============================================================================
 * Simplified 6-DOF rigid-body flight dynamics for a small fixed-wing UAV.
 *
 * This is a REDUCED-ORDER model: linear aerodynamic stability derivatives
 * are used instead of the blade-element-momentum + slipstream model from
 * Levin, Nahon & Paranjape, "Real-time motion planning with a fixed-wing
 * UAV using an agile maneuver space" (Autonomous Robots, 2019). Building
 * that exact model (thruster BEMT, slipstream velocity field, component
 * aerodynamic breakdown, GPOPS-II trajectory optimization) is a research
 * project on its own and is out of scope for a browser-based demo.
 *
 * What THIS module gives you, which the previous geometric version did
 * NOT have, is the property that actually matters for "6-DOF": every
 * candidate motion primitive is produced by numerically integrating the
 * aircraft's equations of motion under a fixed control input, so a
 * primitive that would stall the aircraft (or otherwise leave the flight
 * envelope) is detected and rejected -- exactly the role the "maneuver
 * space" plays in the paper, just with a lighter-weight aerodynamic model.
 *
 * STATE VECTOR (12 states), matching the paper's convention:
 *   u, v, w        body-axis velocities          (m/s)
 *   p, q, r        body-axis angular rates       (rad/s)
 *   phi,theta,psi  Euler angles: roll,pitch,yaw  (rad)
 *   x, y, z        inertial position, z = ALTITUDE UP (m)
 *
 * FRAME NOTE: the rest of this codebase (Three.js scene, obstacle boxes,
 * X_MAX/Y_MAX/Z_MAX bounds) treats +z as altitude UP. The body-axis force
 * and moment equations below are the standard aerospace equations, which
 * are frame-convention-agnostic. Only the translational kinematics (how
 * body velocity maps to inertial position rate) needs a sign flip on the
 * vertical channel, which is handled once, in stateDerivative().
 * ========================================================================= */

const AIRCRAFT = {
    mass: 0.6,              // kg (paper's reference aircraft: 0.576 kg)
    g: 9.81,                // m/s^2

    // Principal moments of inertia, kg*m^2 (rough estimate for a ~0.9 m
    // wingspan foam aerobatic UAV; products of inertia Ixz neglected).
    Ixx: 0.0165,
    Iyy: 0.0250,
    Izz: 0.0415,

    S: 0.25,                 // wing reference area, m^2
    b: 0.90,                  // wingspan, m
    c: 0.28,                  // mean aerodynamic chord, m
    rho: 1.225,               // air density, kg/m^3 (sea level)

    // Longitudinal stability & control derivatives (per radian)
    CL0: 0.25, CLalpha: 5.0, CLde: 0.4, CLq: 4.0,
    CD0: 0.035, k: 0.045,
    Cm0: 0.02, Cmalpha: -1.0, Cmde: -1.2, Cmq: -10.0,

    // Lateral-directional stability & control derivatives (per radian)
    CYbeta: -0.4,
    Clbeta: -0.05, Clp: -0.45, Clda: 0.12, Cldr: 0.01,
    Cnbeta: 0.06, Cnr: -0.09, Cndr: -0.06, Cnda: -0.015,

    // Actuator / envelope limits
    deltaAMax: 25 * Math.PI / 180,
    deltaEMax: 25 * Math.PI / 180,
    deltaRMax: 25 * Math.PI / 180,
    thrustMax: 9.0,            // N
    stallAlpha: 16 * Math.PI / 180,
    minSpeed: 3.0,               // m/s floor used only to avoid /0 in coefficients
    Vtrim: 7.0                   // m/s cruise reference speed (matches the paper)
};

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

/** Aerodynamic + propulsive forces and moments, resolved in body axes. */
function aeroForcesMoments(state, controls, P = AIRCRAFT) {
    const { u, v, w, p, q, r } = state;
    const V = Math.max(P.minSpeed, Math.sqrt(u * u + v * v + w * w));
    const alpha = Math.atan2(w, u);
    const beta = Math.asin(clamp(v / V, -1, 1));
    const qbar = 0.5 * P.rho * V * V;

    const CL = P.CL0 + P.CLalpha * alpha + P.CLde * controls.delta_e + P.CLq * (q * P.c / (2 * V));
    const CD = P.CD0 + P.k * CL * CL;
    const CY = P.CYbeta * beta;

    const Cl = P.Clbeta * beta + P.Clp * (p * P.b / (2 * V)) + P.Clda * controls.delta_a + P.Cldr * controls.delta_r;
    const Cm = P.Cm0 + P.Cmalpha * alpha + P.Cmde * controls.delta_e + P.Cmq * (q * P.c / (2 * V));
    const Cn = P.Cnbeta * beta + P.Cnr * (r * P.b / (2 * V)) + P.Cndr * controls.delta_r + P.Cnda * controls.delta_a;

    const L = qbar * P.S * CL;
    const D = qbar * P.S * CD;
    const Y = qbar * P.S * CY;

    // stability axes -> body axes
    const Fx_aero = -D * Math.cos(alpha) + L * Math.sin(alpha);
    const Fz_aero = -D * Math.sin(alpha) - L * Math.cos(alpha);
    const thrust = clamp(controls.throttle, 0, 1) * P.thrustMax;

    const Fx = Fx_aero + thrust;
    const Fy = Y;
    const Fz = Fz_aero;

    const l = qbar * P.S * P.b * Cl;
    const m = qbar * P.S * P.c * Cm;
    const n = qbar * P.S * P.b * Cn;

    return { Fx, Fy, Fz, l, m, n, alpha, beta, V, stalled: Math.abs(alpha) > P.stallAlpha };
}

/** Full 12-state time derivative (standard rigid-body 6-DOF equations). */
function stateDerivative(state, controls, P = AIRCRAFT) {
    const { u, v, w, p, q, r, phi, theta, psi } = state;
    const fm = aeroForcesMoments(state, controls, P);
    const m = P.mass;

    // gravity resolved into body axes
    const gx = -P.g * Math.sin(theta);
    const gy = P.g * Math.cos(theta) * Math.sin(phi);
    const gz = P.g * Math.cos(theta) * Math.cos(phi);

    // translational (Newton, body-axis form with rotating-frame terms)
    const udot = r * v - q * w + fm.Fx / m + gx;
    const vdot = p * w - r * u + fm.Fy / m + gy;
    const wdot = q * u - p * v + fm.Fz / m + gz;

    // rotational (Euler's equations, products of inertia neglected)
    const pdot = ((P.Iyy - P.Izz) * q * r + fm.l) / P.Ixx;
    const qdot = ((P.Izz - P.Ixx) * p * r + fm.m) / P.Iyy;
    const rdot = ((P.Ixx - P.Iyy) * p * q + fm.n) / P.Izz;

    // Euler angle kinematics
    const phidot = p + q * Math.sin(phi) * Math.tan(theta) + r * Math.cos(phi) * Math.tan(theta);
    const thetadot = q * Math.cos(phi) - r * Math.sin(phi);
    const cosTheta = Math.abs(Math.cos(theta)) > 1e-6 ? Math.cos(theta) : 1e-6;
    const psidot = (q * Math.sin(phi) + r * Math.cos(phi)) / cosTheta;

    // body -> inertial velocity (ZYX Euler sequence); zdotN is "NED-down"
    const cph = Math.cos(phi), sph = Math.sin(phi);
    const cth = Math.cos(theta), sth = Math.sin(theta);
    const cps = Math.cos(psi), sps = Math.sin(psi);

    const xdotN = cth * cps * u + (sph * sth * cps - cph * sps) * v + (cph * sth * cps + sph * sps) * w;
    const ydotN = cth * sps * u + (sph * sth * sps + cph * cps) * v + (cph * sth * sps - sph * cps) * w;
    const zdotN = -sth * u + sph * cth * v + cph * cth * w; // positive = descending

    return {
        u: udot, v: vdot, w: wdot,
        p: pdot, q: qdot, r: rdot,
        phi: phidot, theta: thetadot, psi: psidot,
        x: xdotN, y: ydotN, z: -zdotN,  // world z is altitude-up -> flip once here
        stalled: fm.stalled, alpha: fm.alpha, beta: fm.beta, V: fm.V
    };
}

const STATE_KEYS = ['u', 'v', 'w', 'p', 'q', 'r', 'phi', 'theta', 'psi', 'x', 'y', 'z'];

function addScaled(state, deriv, h) {
    const out = {};
    for (const k of STATE_KEYS) out[k] = state[k] + h * deriv[k];
    return out;
}

/** One classical RK4 integration step of duration dt (seconds). */
function rk4Step(state, controls, dt, P = AIRCRAFT) {
    const k1 = stateDerivative(state, controls, P);
    const k2 = stateDerivative(addScaled(state, k1, dt / 2), controls, P);
    const k3 = stateDerivative(addScaled(state, k2, dt / 2), controls, P);
    const k4 = stateDerivative(addScaled(state, k3, dt), controls, P);

    const next = {};
    for (const k of STATE_KEYS) {
        next[k] = state[k] + (dt / 6) * (k1[k] + 2 * k2[k] + 2 * k3[k] + k4[k]);
    }
    return { state: next, alpha: k1.alpha, beta: k1.beta, V: k1.V, stalled: k1.stalled };
}

/**
 * Simulate a FIXED control input over `duration` seconds, sub-stepped by
 * `dtSub`, via RK4. Returns:
 *   - finalState: the resulting 12-state node
 *   - trajectory: sampled {x,y,z} points along the way, for higher-fidelity
 *                 collision checking than a straight-line segment
 *   - feasible:   false if the aircraft stalled at any point during the
 *                 maneuver (this is the "dynamic feasibility" rejection)
 */
function simulateControlSet(fromState, controls, duration, dtSub = 0.05, P = AIRCRAFT) {
    let state = { ...fromState };
    const trajectory = [{ x: state.x, y: state.y, z: state.z }];
    let feasible = true;
    const steps = Math.max(1, Math.round(duration / dtSub));

    for (let i = 0; i < steps; i++) {
        const result = rk4Step(state, controls, dtSub, P);
        state = result.state;
        if (result.stalled) feasible = false;
        trajectory.push({ x: state.x, y: state.y, z: state.z });
    }

    return { finalState: state, trajectory, feasible };
}

/**
 * Applying a fixed "trim" control input from a DIFFERENT starting attitude
 * doesn't work for roll: unlike pitch (which has a natural restoring
 * moment toward a trim alpha via Cmalpha), roll has no aerodynamic
 * "spring" pulling the aircraft toward a particular bank angle -- the
 * steady-turn aileron/rudder deflections from solveTurnTrim only HOLD a
 * bank once you're already there; they provide almost no authority to
 * roll INTO that bank from wings-level. (Verified: applying them directly
 * from phi=0 produced barely 1 degree of bank instead of the intended
 * ~26 degrees.)
 *
 * This mirrors exactly why the paper pairs its feedforward motion
 * primitives with a separate feedback controller (Sect. 5) rather than
 * expecting feedforward alone to work. Here, a minimal proportional-
 * derivative loop on roll angle/rate is layered on top of the feedforward
 * aileron trim -- just enough active control to actually establish and
 * hold the commanded bank, while pitch/rudder/throttle are left as pure
 * feedforward (pitch has enough natural static stability via Cmalpha that
 * closing an explicit loop on it was found to cause oscillation instead
 * of helping -- verified experimentally, not assumed).
 */
function simulateTrimTrackingManeuver(fromState, target, duration, dtSub = 0.05, P = AIRCRAFT) {
    const Kp_phi = 4.0, Kd_p = 0.8;
    let state = { ...fromState };
    const trajectory = [{ x: state.x, y: state.y, z: state.z }];
    let feasible = true;
    const steps = Math.max(1, Math.round(duration / dtSub));

    for (let i = 0; i < steps; i++) {
        const phiErr = target.phi - state.phi;
        const delta_a = clamp(target.delta_a + Kp_phi * phiErr - Kd_p * state.p, -P.deltaAMax, P.deltaAMax);
        const delta_e = clamp(target.delta_e, -P.deltaEMax, P.deltaEMax);
        const delta_r = clamp(target.delta_r, -P.deltaRMax, P.deltaRMax);
        const throttle = clamp(target.throttle, 0, 1);

        const result = rk4Step(state, { delta_a, delta_e, delta_r, throttle }, dtSub, P);
        state = result.state;
        if (result.stalled) feasible = false;
        trajectory.push({ x: state.x, y: state.y, z: state.z });
    }

    return { finalState: state, trajectory, feasible };
}

// Plain <script> usage (no bundler/module system in this project), so we
// attach everything to window, the same way three.min.js / OrbitControls.js
// already do in this codebase.
window.Dynamics = {
    AIRCRAFT, aeroForcesMoments, stateDerivative, rk4Step, simulateControlSet,
    simulateTrimTrackingManeuver, clamp
};