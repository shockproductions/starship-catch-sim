/* =====================================================================
   STARSHIP TOWER-CATCH SIMULATION — physics, sensors, GNC & fail-safes
   Pure JS, no DOM. Runs in the browser (window.CatchSim) and in Node.
   World frame: x = East, y = North, z = Up  (meters, origin = catch point)
   Body frame:  X = nose (long axis), Y = pin side, Z = belly (heat shield)
   Body coordinates are measured from the geometric center of the ship.
   ===================================================================== */
(function (global) {
'use strict';

const G0 = 9.80665;
const DEG = Math.PI / 180;

/* ---------------- vector / quaternion helpers ---------------- */
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = a => Math.hypot(a[0], a[1], a[2]);
const norm = a => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const clampLen = (a, m) => { const l = len(a); return l > m ? scl(a, m / l) : a; };

const qmul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
const qconj = q => [q[0], -q[1], -q[2], -q[3]];
const qnormalize = q => { const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return [q[0] / l, q[1] / l, q[2] / l, q[3] / l]; };
function qrot(q, v) { // body -> world
  const w = q[0], x = q[1], y = q[2], z = q[3];
  const tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}
const qrotInv = (q, v) => qrot(qconj(q), v);
function qFromAxes(X, Y, Z) {
  const m00 = X[0], m10 = X[1], m20 = X[2], m01 = Y[0], m11 = Y[1], m21 = Y[2], m02 = Z[0], m12 = Z[1], m22 = Z[2];
  const tr = m00 + m11 + m22; let w, x, y, z, s;
  if (tr > 0) { s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
  else if (m00 > m11 && m00 > m22) { s = Math.sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s; }
  else if (m11 > m22) { s = Math.sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s; }
  else { s = Math.sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s; }
  return qnormalize([w, x, y, z]);
}
function qToRotVec(q) {
  let w = q[0], x = q[1], y = q[2], z = q[3];
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  const s = Math.hypot(x, y, z);
  if (s < 1e-9) return [2 * x, 2 * y, 2 * z];
  const a = 2 * Math.atan2(s, w);
  return [x / s * a, y / s * a, z / s * a];
}
function qFromRotVec(r) {
  const a = len(r);
  if (a < 1e-12) return qnormalize([1, r[0] / 2, r[1] / 2, r[2] / 2]);
  const s = Math.sin(a / 2) / a;
  return [Math.cos(a / 2), r[0] * s, r[1] * s, r[2] * s];
}

/* ---------------- seeded RNG ---------------- */
function mulberry32(a) {
  return function () { a |= 0; a = a + 0x6D2B79F5 | 0; let t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
}
class RNG {
  constructor(seed) { this.r = mulberry32(seed >>> 0); this.spare = null; }
  u() { return this.r(); }
  n() {
    if (this.spare !== null) { const s = this.spare; this.spare = null; return s; }
    let u, v, s; do { u = this.r() * 2 - 1; v = this.r() * 2 - 1; s = u * u + v * v; } while (s >= 1 || s === 0);
    const m = Math.sqrt(-2 * Math.log(s) / s); this.spare = v * m; return u * m;
  }
}

/* ---------------- vehicle & site constants ---------------- */
const VEH = {
  L: 50, R: 4.5, X_TAIL: -25, X_NOSE: 25, X_GIMBAL: -23,
  // body aero: force coefficient * reference area per body axis (crossflow model)
  CXA: 0.55 * 63.6, CYA: 0.55 * 450, CZA: 1.05 * 450,
  CF: 1.2,              // flap normal-force coefficient
  flaps: [              // name, body x, body y, area
    { id: 'FL', x: 17, y: -6.0, A: 22 }, { id: 'FR', x: 17, y: 6.0, A: 22 },
    { id: 'AL', x: -20, y: -6.5, A: 40 }, { id: 'AR', x: -20, y: 6.5, A: 40 }],
  X_PIN: 13, PIN_R0: 4.6, PIN_R1: 6.3,
  T_NOM: 2.3e6, ISP: 330, GIMBAL_MAX: 15 * DEG, GIMBAL_RATE: 25 * DEG,
  RCS: [0.30e6, 0.8e6, 0.8e6],
  FLAP_RATE: 0.7,       // exposure fraction per second
};
const SITE = {
  tower: { x0: -28.5, x1: -19.5, y0: -4.5, y1: 4.5, h: 146 },
  armPivotX: -19.5, armPivotY: 5.75, armLen: 33, armOpen: 40 * DEG, armHalfW: 0.55, armThick: 2.4,
  coastX: 25,                 // ocean east of this line
  abort: [420, 40],            // divert / splashdown zone (offshore)
  catchPt: [0, 0],
};

/* ---------------- default parameters (all live-editable) ---------------- */
const DEFAULTS = {
  seed: 1,
  dispersion: 1.0,           // master multiplier on initial-condition / model dispersions
  // environment
  windSpeed: 7,  windDir: 110, windVeer: 25, gust: 2.0, rhoScale: 1.0,
  // vehicle
  dryMass: 100, propMass: 11, cdScale: 1.0, cmOffset: -1.0,
  // engines
  startMean: 2.4, startSigma: 0.15, startFailProb: 3, thrustSigma: 1.5, minThrottle: 0.40,
  inflightFailRate: 0,       // % per minute per running engine
  // initial state
  startAlt: 10000, startDownrange: 2600, startCrossrange: 700,
  // guidance & control
  ignAlt: 620, adaptiveIgn: true, attBW: 1.4, glideGain: 0.25, glideTiltMax: 16,
  maxTilt: 25, holdOffset: 10, railWindow: 4, flipKick: 52, armHeight: 90, catchClear: 8.0, gateSink: 3.0, termRate: 1.0,
  minEnginesFlip: 2, allowEngineOutCatch: false,
  // go / no-go
  windLimit: 16, propReserve: 1.0,
  // sensors / nav
  gpsSigma: 0.6, imuNoise: 1.0, attNoise: 0.15,
};

/* ===================================================================== */
class Sim {
  constructor(params) {
    this.P = Object.assign({}, DEFAULTS, params || {});
    this.reset(this.P.seed);
  }

  log(msg, level) { this.events.push({ t: this.t, msg, level: level || 'info' }); }

  /* ------------------------------------------------------------- */
  reset(seed) {
    const P = this.P;
    this.seed = seed >>> 0; P.seed = this.seed;
    const rd = new RNG(this.seed * 2654435761 + 17);
    this.rn = new RNG(this.seed * 7919 + 13);
    const sd = P.dispersion;
    this.disp = {
      cd: 1 + rd.n() * 0.07 * sd,
      cm: rd.n() * 0.35 * sd,
      mass: rd.n() * 1.5 * sd,
      thrust: [0, 1, 2].map(() => 1 + rd.n() * P.thrustSigma / 100),
      windFcSpd: rd.n() * 0.15 * sd,
      windFcDir: rd.n() * 8 * sd,
      p0: [rd.n() * 250 * sd, rd.n() * 250 * sd, rd.n() * 120 * sd],
      v0: [rd.n() * 6 * sd, rd.n() * 6 * sd, rd.n() * 4 * sd],
      att0: [rd.n() * 3 * DEG * sd, rd.n() * 3 * DEG * sd, rd.n() * 3 * DEG * sd],
    };
    const d = this.disp;
    this.t = 0;
    this.events = [];
    this.samples = [];
    this.sampleT = 0;
    this.trail = [];
    this.done = false; this.frozen = false; this.outcome = null;
    this.prop = P.propMass * 1000;
    this.dryActual = P.dryMass * 1000 + d.mass * 1000;

    const p0 = [P.startDownrange + d.p0[0], P.startCrossrange + d.p0[1], P.startAlt + d.p0[2]];
    const hTo = norm([-p0[0], -p0[1], 0]);
    this.p = p0;
    this.v = [hTo[0] * 55 + d.v0[0], hTo[1] * 55 + d.v0[1], -110 + d.v0[2]];
    // belly-flop attitude: nose along heading, belly down
    const Z = [0, 0, -1], X = scl(hTo, -1), Y = cross(Z, X);
    this.q = qmul(qFromAxes(X, Y, Z), qFromRotVec(d.att0));
    this.w = [0, 0, 0];

    this.engines = [0, 1, 2].map(i => ({ id: i, st: 'off', t: 0, dur: 0, fail: false, thr: 0, cmd: 0, T: 0, scale: d.thrust[i] }));
    this.flaps = VEH.flaps.map(() => ({ e: 0.5, cmd: 0.5, stuck: false }));
    this.gimbal = [0, 0]; this.gimbalCmd = [0, 0];
    this.rcs = [0, 0, 0]; this.rcsCmd = [0, 0, 0];
    this.arms = { a: 1, cmd: 1, fault: false };
    this.armLoad = 0; this.armContact = false;
    this.gust = [0, 0, 0]; this.discreteGust = null;
    this.windTrue = [0, 0, 0];
    this.forces = { aero: [0, 0, 0], thrust: [0, 0, 0] };
    this.aTrue = [0, 0, 0]; this.gload = 0;
    this.stats = { maxG: 0, maxQ: 0, catchErr: null, minPropAtCatch: null };

    // navigation filter state
    this.nav = {
      p: this.p.slice(), v: this.v.slice(), q: this.q.slice(), w: [0, 0, 0],
      ab: [0, 0, 0], att: [0, 0, 0], nextGps: 0, dropUntil: -1, lastGps: 0, gpsOk: true,
    };

    // controller state
    this.ctrl = {
      phase: 'BELLY-FLOP', mode: 'NOMINAL', abortReason: null,
      heading: scl(hTo, -1), tIgn: null, tPhase: 0, integ: [0, 0, 0],
      go: false, pollName: '', gateDone: false, poll1: false, abortPt: null, divTerm: false, tgo: 0, checks: {}, engAnomaly: false, loadT: 0,
      hIgnEff: P.ignAlt, target: null, attErr: 0, tauCmd: [0, 0, 0],
      locT: 0, divertStage: 0, catchSettle: 0, transOkT: 0, catchX: undefined,
    };
    this.stepCount = 0;
    this.log(`Mission start — seed ${this.seed}. Entry interface complete, belly-flop established at ${(p0[2] / 1000).toFixed(1)} km.`, 'phase');
  }

  /* ---------------- geometry helpers ---------------- */
  cm() { return this.P.cmOffset + this.disp.cm; }                  // true CoM (body x)
  mass() { return this.dryActual + this.prop; }
  massEst() { return this.P.dryMass * 1000 + this.prop; }         // what GNC believes
  inertia(m) { return [m * 4.5 * 4.5 * 0.8, m * (50 * 50 / 12 + 5), m * (50 * 50 / 12 + 5)]; }
  holdZ() { return this.P.armHeight + this.P.catchClear - (VEH.X_PIN - this.P.cmOffset); }   // gate: pins catchClear above rails
  splashZ() { return 0.4 + (this.P.cmOffset - VEH.X_TAIL); }
  holdPoint() { return [SITE.catchPt[0] + this.P.holdOffset, SITE.catchPt[1], this.holdZ()]; }
  catchPoint() { return [SITE.catchPt[0], SITE.catchPt[1], this.holdZ()]; }
  pinContactZ() { return this.P.armHeight - 0.4 - (VEH.X_PIN - this.P.cmOffset); }
  bodyPoint(xb, yb, zb) { return add(this.p, qrot(this.q, [xb - this.cm(), yb, zb])); }
  axis() { return qrot(this.q, [1, 0, 0]); }
  rho(z) { return 1.225 * Math.exp(-Math.max(0, z) / 8500) * this.P.rhoScale; }
  tailZ() { return this.bodyPoint(VEH.X_TAIL, 0, 0)[2]; }

  windMean(z, fc) {
    const P = this.P;
    let spd = P.windSpeed, dir = P.windDir;
    if (fc) { spd *= 1 + this.disp.windFcSpd; dir += this.disp.windFcDir; }
    const h = Math.max(z, 2);
    let f = h < 1000 ? Math.pow(h / 10, 0.14) : Math.pow(100, 0.14) * (1 + (h - 1000) / 9000);
    dir += P.windVeer * clamp(h / 10000, 0, 1.2);
    const s = spd * f, a = dir * DEG;
    return [-Math.sin(a) * s, -Math.cos(a) * s, 0]; // dir = where it blows FROM
  }

  armSegs() {
    const th = this.arms.a * SITE.armOpen, out = [];
    for (const sg of [-1, 1]) {
      const px = SITE.armPivotX, py = sg * SITE.armPivotY;
      const dx = Math.cos(th), dy = sg * Math.sin(th);
      out.push({ px, py, dx, dy, ex: px + dx * SITE.armLen, ey: py + dy * SITE.armLen, sg });
    }
    return out;
  }

  /* ---------------- engines ---------------- */
  startEngine(i) {
    const e = this.engines[i], P = this.P;
    if (e.st === 'running' || e.st === 'starting' || e.st === 'failed') return;
    e.st = 'starting'; e.t = 0;
    e.dur = Math.max(0.9, P.startMean + this.rn.n() * P.startSigma);
    e.fail = this.rn.u() < P.startFailProb / 100;
    e.thr = 0;
  }
  shutdownEngine(i, why) {
    const e = this.engines[i];
    if (e.st === 'running' || e.st === 'starting') { e.st = 'shutdown'; e.thr = 0; e.T = 0; if (why) this.log(`Raptor ${i + 1} shutdown — ${why}`); }
  }
  failEngine(i, why) {
    const e = this.engines[i];
    if (e.st === 'failed') return;
    const was = e.st;
    e.st = 'failed'; e.thr = 0; e.T = 0;
    if (was === 'running' || was === 'starting') this.ctrl.engAnomaly = true;
    this.log(`Raptor ${i + 1} FAILURE — ${why || 'injected'}`, 'warn');
  }
  nRunning() { return this.engines.filter(e => e.st === 'running').length; }
  nLit() { return this.engines.filter(e => e.st === 'running' || e.st === 'starting').length; }

  /* ---------------- failure injection API ---------------- */
  inject(kind, arg) {
    switch (kind) {
      case 'engine': this.failEngine(arg, 'injected fault'); break;
      case 'flap': { const f = this.flaps[arg]; f.stuck = !f.stuck; this.log(`Flap ${VEH.flaps[arg].id} actuator ${f.stuck ? 'JAMMED' : 'restored'}`, f.stuck ? 'warn' : 'info'); break; }
      case 'gust': this.discreteGust = { t0: this.t, dur: 5, amp: arg || 14, dir: this.rn.u() * 2 * Math.PI }; this.log(`Discrete gust injected (${(arg || 14)} m/s)`, 'warn'); break;
      case 'gps': this.nav.dropUntil = this.t + (arg || 10); this.log(`GPS dropout for ${arg || 10} s`, 'warn'); break;
      case 'arms': this.arms.fault = !this.arms.fault; this.log(`Tower arm hydraulics ${this.arms.fault ? 'FAULT' : 'restored'}`, this.arms.fault ? 'warn' : 'info'); break;
    }
  }

  /* ---------------- aerodynamics (true or nominal model) ---------------- */
  aero(vrelWorld, q, rho, cdScale, cm, flapE, w) {
    const vb = qrotInv(q, vrelWorld);
    const k = 0.5 * rho * cdScale;
    const Fb = [-k * VEH.CXA * Math.abs(vb[0]) * vb[0], -k * VEH.CYA * Math.abs(vb[1]) * vb[1], -k * VEH.CZA * Math.abs(vb[2]) * vb[2]];
    // center of pressure at geometric center (x=0) -> arm from CoM
    let tau = cross([0 - cm, 0, 0], Fb);
    // aerodynamic rotational damping
    const vm = len(vb);
    const cdmp = 0.5 * rho * vm * 450 * 50 * 50 / 12 * 0.25;
    tau = add(tau, [-0.5 * rho * vm * 60 * w[0], -cdmp * w[1], -cdmp * w[2]]);
    const flapF = [], flapTau1 = [];
    let F = Fb.slice();
    for (let i = 0; i < 4; i++) {
      const fl = VEH.flaps[i];
      const f1 = -k * VEH.CF * fl.A * Math.abs(vb[2]) * vb[2]; // per unit effective exposure
      const ax = -k * 0.08 * fl.A * Math.abs(vb[0]) * vb[0];
      const r = [fl.x - cm, fl.y, 0];
      const t1 = cross(r, [ax, 0, f1]);
      flapTau1.push(t1);
      const eff = 0.1 + 0.9 * (flapE ? flapE[i] : 0.5);
      const Fi = [ax * eff, 0, f1 * eff];
      flapF.push(Fi);
      F = add(F, Fi);
      tau = add(tau, scl(t1, eff));
    }
    return { Fb: F, tau, flapTau1, vb, qbar: k / cdScale * vm * vm, bodyTau: tau };
  }

  /* ================= PHYSICS STEP ================= */
  step(dt) {
    if (this.frozen) { this.t += dt; return; }
    const P = this.P, rn = this.rn;
    this.stepCount++;
    // ---- controller @ 50 Hz
    if (this.stepCount % 10 === 1) this.control(0.02);

    // ---- wind (mean + Gauss-Markov gusts + discrete gust)
    const z = this.p[2];
    const tauG = 2.5, sg = P.gust * (z < 400 ? 1 : 0.75);
    const a = Math.exp(-dt / tauG), bq = sg * Math.sqrt(1 - a * a);
    this.gust = [this.gust[0] * a + bq * rn.n(), this.gust[1] * a + bq * rn.n(), this.gust[2] * a + 0.4 * bq * rn.n()];
    let wind = add(this.windMean(z, false), this.gust);
    if (this.discreteGust) {
      const g = this.discreteGust, tt = (this.t - g.t0) / g.dur;
      if (tt > 1) this.discreteGust = null;
      else { const m = g.amp * 0.5 * (1 - Math.cos(2 * Math.PI * tt)); wind = add(wind, [Math.cos(g.dir) * m, Math.sin(g.dir) * m, 0]); }
    }
    this.windTrue = wind;

    // ---- actuators
    for (const f of this.flaps) if (!f.stuck) f.e += clamp(f.cmd - f.e, -VEH.FLAP_RATE * dt, VEH.FLAP_RATE * dt);
    for (let i = 0; i < 2; i++) this.gimbal[i] += clamp(this.gimbalCmd[i] - this.gimbal[i], -VEH.GIMBAL_RATE * dt, VEH.GIMBAL_RATE * dt);
    const rl = Math.min(1, dt / 0.05);
    for (let i = 0; i < 3; i++) this.rcs[i] += (this.rcsCmd[i] - this.rcs[i]) * rl;
    if (!this.arms.fault) this.arms.a += clamp(this.arms.cmd - this.arms.a, -0.45 * dt, 0.45 * dt);

    // ---- engines
    let T = 0, mdot = 0;
    for (const e of this.engines) {
      if (e.st === 'starting') {
        e.t += dt;
        if (e.t >= e.dur) {
          if (e.fail) { e.st = 'failed'; this.log(`Raptor ${e.id + 1} FAILED TO START (spin-up ${e.dur.toFixed(2)} s)`, 'warn'); }
          else { e.st = 'running'; e.thr = P.minThrottle * 0.6; this.log(`Raptor ${e.id + 1} at mainstage (+${e.dur.toFixed(2)} s)`); }
        }
      } else if (e.st === 'running') {
        e.thr += (clamp(e.cmd, P.minThrottle, 1) - e.thr) * Math.min(1, dt / 0.18);
        if (P.inflightFailRate > 0 && rn.u() < P.inflightFailRate / 100 / 60 * dt) this.failEngine(e.id, 'in-flight anomaly');
        if (this.prop <= 0) this.shutdownEngine(e.id, 'propellant depletion');
      }
      e.T = e.st === 'running' ? VEH.T_NOM * e.scale * e.thr * (1 + 0.004 * rn.n()) : 0;
      T += e.T; mdot += e.T / (VEH.ISP * G0);
    }
    this.prop = Math.max(0, this.prop - mdot * dt);

    // ---- forces & torques (body)
    const m = this.mass(), cm = this.cm();
    const vrel = sub(this.v, wind);
    const rho = this.rho(z);
    const ae = this.aero(vrel, this.q, rho, P.cdScale * this.disp.cd, cm, this.flaps.map(f => f.e), this.w);
    let Fb = ae.Fb.slice(), tau = ae.tau.slice();
    this.qbar = ae.qbar;
    const tdir = norm([1, Math.tan(this.gimbal[0]), Math.tan(this.gimbal[1])]);
    const Ft = scl(tdir, T);
    Fb = add(Fb, Ft);
    tau = add(tau, cross([VEH.X_GIMBAL - cm, 0, 0], Ft));
    tau = add(tau, this.rcs);
    let Fw = qrot(this.q, Fb);
    this.forces.aero = qrot(this.q, ae.Fb); this.forces.thrust = qrot(this.q, Ft); this.thrustTotal = T;

    // ---- contacts with tower arms
    const wW = qrot(this.q, this.w);
    this.armLoad = 0;
    const segs = this.armSegs();
    const zA = P.armHeight;
    const addPointForce = (pt, F) => { Fw = add(Fw, F); tau = add(tau, qrotInv(this.q, cross(sub(pt, this.p), F))); };
    const ptVel = pt => add(this.v, cross(wW, sub(pt, this.p)));
    const X = this.axis();
    // body cylinder vs arm (horizontal push)
    if (Math.abs(X[2]) > 0.2) {
      const zc = zA - SITE.armThick / 2;
      const s = cm + (zc - this.p[2]) / X[2];
      if (s > VEH.X_TAIL && s < VEH.X_NOSE) {
        const c = this.bodyPoint(s, 0, 0);
        for (const sgm of segs) {
          const cl = closestOnSeg(c[0], c[1], sgm);
          const dx = c[0] - cl[0], dy = c[1] - cl[1], d = Math.hypot(dx, dy) || 1e-6;
          const pen = VEH.R + SITE.armHalfW - d;
          if (pen > 0) {
            const n = [dx / d, dy / d, 0];
            const vn = dot(ptVel(c), n);
            if (!this.armContact && vn < -4.5 && !this.done) this.crash(`Arm strike at ${(-vn).toFixed(1)} m/s`);
            this.armContact = true;
            const fmag = Math.max(0, 4e6 * pen - 6e5 * vn);
            addPointForce(c, scl(n, fmag));
          }
        }
      }
    }
    // catch pins vs arm tops (vertical support)
    for (const side of [-1, 1]) {
      for (const r of [4.9, 5.6, 6.2]) {
        const pt = this.bodyPoint(VEH.X_PIN, side * r, 0);
        for (const sgm of segs) {
          const cl = closestOnSeg(pt[0], pt[1], sgm);
          if (Math.hypot(pt[0] - cl[0], pt[1] - cl[1]) > SITE.armHalfW) continue;
          const pen = zA - pt[2];
          if (pen > 0 && pen < 1.6) {
            const pv = ptVel(pt);
            const Fz = Math.max(0, 1.2e7 * pen - 1.1e6 * pv[2]);
            const Fh = clampLen([-4e5 * pv[0], -4e5 * pv[1], 0], 0.6 * Fz);
            addPointForce(pt, [Fh[0], Fh[1], Fz]);
            this.armLoad += Fz;
          }
        }
      }
    }

    // ---- integrate translational
    Fw = add(Fw, [0, 0, -m * G0]);
    const acc = scl(Fw, 1 / m);
    this.aTrue = acc;
    const fsp = sub(acc, [0, 0, -G0]);
    this.gload = len(fsp) / G0;
    if (this.t > 1) this.stats.maxG = Math.max(this.stats.maxG, this.gload);
    this.stats.maxQ = Math.max(this.stats.maxQ, this.qbar);
    this.v = add(this.v, scl(acc, dt));
    this.p = add(this.p, scl(this.v, dt));

    // ---- integrate rotational (body frame, Euler's equations)
    const I = this.inertia(m);
    const Iw = [I[0] * this.w[0], I[1] * this.w[1], I[2] * this.w[2]];
    const gyro = cross(this.w, Iw);
    for (let i = 0; i < 3; i++) this.w[i] += (tau[i] - gyro[i]) / I[i] * dt;
    const dq = qmul(this.q, [0, this.w[0], this.w[1], this.w[2]]);
    this.q = qnormalize([this.q[0] + 0.5 * dq[0] * dt, this.q[1] + 0.5 * dq[1] * dt, this.q[2] + 0.5 * dq[2] * dt, this.q[3] + 0.5 * dq[3] * dt]);

    // ---- sensors / navigation filter
    this.navStep(dt, fsp);

    this.t += dt;
    this.checkCollisions();
    if (this.t >= this.sampleT) { this.sample(); this.sampleT += 0.1; }
  }

  /* ---------------- navigation: IMU propagation + GPS / radar-alt correction ---------------- */
  navStep(dt, fsp) {
    const n = this.nav, P = this.P, rn = this.rn;
    const sa = 0.05 * P.imuNoise;
    n.ab = add(scl(n.ab, 1 - dt / 200), [rn.n() * 0.004 * P.imuNoise * Math.sqrt(dt), rn.n() * 0.004 * P.imuNoise * Math.sqrt(dt), rn.n() * 0.004 * P.imuNoise * Math.sqrt(dt)]);
    const fm = [fsp[0] + n.ab[0] + rn.n() * sa, fsp[1] + n.ab[1] + rn.n() * sa, fsp[2] + n.ab[2] + rn.n() * sa];
    n.v = add(n.v, scl([fm[0], fm[1], fm[2] - G0], dt));
    n.p = add(n.p, scl(n.v, dt));
    n.gpsOk = this.t >= n.dropUntil;
    if (this.t >= n.nextGps) {
      n.nextGps = this.t + 0.1;
      if (n.gpsOk) {
        const s = P.gpsSigma;
        const zp = [this.p[0] + rn.n() * s, this.p[1] + rn.n() * s, this.p[2] + rn.n() * s * 1.6];
        const zv = [this.v[0] + rn.n() * 0.05, this.v[1] + rn.n() * 0.05, this.v[2] + rn.n() * 0.08];
        for (let i = 0; i < 3; i++) { n.p[i] += 0.3 * (zp[i] - n.p[i]); n.v[i] += 0.25 * (zv[i] - n.v[i]); }
        n.lastGps = this.t;
      }
      // radar altimeter below 800 m AGL (sea / pad)
      if (this.p[2] < 800) {
        const za = this.p[2] + rn.n() * (0.15 + 0.002 * this.p[2]);
        n.p[2] += 0.35 * (za - n.p[2]);
      }
    }
    // attitude estimate: star tracker / INS with slowly wandering error
    const ta = Math.exp(-dt / 20), ba = P.attNoise * DEG * Math.sqrt(1 - ta * ta);
    n.att = [n.att[0] * ta + ba * rn.n(), n.att[1] * ta + ba * rn.n(), n.att[2] * ta + ba * rn.n()];
    n.q = qmul(this.q, qFromRotVec(n.att));
    const gs = 0.002 * P.imuNoise;
    n.w = [this.w[0] + rn.n() * gs, this.w[1] + rn.n() * gs, this.w[2] + rn.n() * gs];
  }

  /* ---------------- collisions / touchdown ---------------- */
  checkCollisions() {
    if (this.done && this.outcome && this.outcome.kind !== 'success') return;
    const X = this.axis();
    const tw = SITE.tower, R = VEH.R;
    for (let s = VEH.X_TAIL; s <= VEH.X_NOSE; s += 5) {
      const c = this.bodyPoint(s, 0, 0);
      if (c[2] < tw.h && c[2] > 0) {
        const dx = Math.max(tw.x0 - c[0], 0, c[0] - tw.x1), dy = Math.max(tw.y0 - c[1], 0, c[1] - tw.y1);
        if (Math.hypot(dx, dy) < R) { this.crash('Tower strike'); return; }
      }
    }
    if (this.ctrl.phase === 'CAUGHT') return;
    const tail = this.bodyPoint(VEH.X_TAIL, 0, 0), nose = this.bodyPoint(VEH.X_NOSE, 0, 0);
    const rz = R * Math.sqrt(Math.max(0, 1 - X[2] * X[2]));
    const low = Math.min(tail[2], nose[2]) - rz;
    if (low <= 0) {
      const sp = len(this.v), tilt = Math.acos(clamp(X[2], -1, 1)) / DEG;
      const sea = this.p[0] > SITE.coastX;
      if (-this.v[2] < 3.5 && Math.hypot(this.v[0], this.v[1]) < 5 && tilt < 15) {
        if (sea) this.finish('divert', `Soft splashdown at divert zone — vehicle safed (${sp.toFixed(1)} m/s)`);
        else this.finish('divert', `Emergency landing on pad apron (${sp.toFixed(1)} m/s)`);
        this.frozen = true;
      } else {
        this.crash(sea ? `Water impact at ${sp.toFixed(0)} m/s` : `Ground impact at ${sp.toFixed(0)} m/s`);
      }
    }
  }
  crash(msg) {
    if (this.frozen) return;
    this.log(`VEHICLE LOST — ${msg}`, 'crit');
    this.ctrl.phase = 'LOST';
    this.finish('lost', msg);
    this.frozen = true;
    for (const e of this.engines) { if (e.st === 'running' || e.st === 'starting') e.st = 'shutdown'; e.T = 0; }
  }
  finish(kind, msg) {
    if (this.done) return;
    this.done = true;
    const hp = SITE.catchPt;
    this.outcome = { kind, msg, t: this.t, seed: this.seed, x: this.p[0], y: this.p[1], err: Math.hypot(this.p[0] - hp[0], this.p[1] - hp[1]), prop: this.prop / 1000, maxG: this.stats.maxG };
    if (kind === 'success') this.log(`CATCH CONFIRMED — ${msg}`, 'good');
    else if (kind === 'divert') this.log(msg, 'warn');
  }

  /* ---------------- telemetry sampling (10 Hz) ---------------- */
  sample() {
    const X = this.axis(), c = this.ctrl;
    const vh = Math.hypot(this.v[0], this.v[1]);
    const tgt = c.target || this.holdPoint();
    const wm = this.windMean(this.p[2], false);
    this.samples.push({
      t: this.t, alt: this.p[2], agl: this.tailZ(), vz: this.v[2], vh, spd: len(this.v),
      qbar: this.qbar || 0, T: this.thrustTotal || 0, W: this.mass() * G0,
      thr: this.engines.map(e => e.st === 'running' ? e.thr * 100 : 0),
      flaps: this.flaps.map(f => f.e * 100),
      tilt: Math.acos(clamp(X[2], -1, 1)) / DEG, attErr: c.attErr / DEG,
      gimbal: [this.gimbal[0] / DEG, this.gimbal[1] / DEG],
      windMean: Math.hypot(wm[0], wm[1]), windTot: len(this.windTrue),
      tgtErr: Math.hypot(tgt[0] - this.p[0], tgt[1] - this.p[1]),
      navErr: len(sub(this.nav.p, this.p)),
      prop: this.prop / 1000, propNeed: c.propNeedCatch ? c.propNeedCatch / 1000 : null, g: this.gload, x: this.p[0], y: this.p[1], phase: c.phase,
    });
    if (this.samples.length > 12000) this.samples.splice(0, 2000);
    this.trail.push([this.p[0], this.p[1], this.p[2], c.phase]);
    if (this.trail.length > 8000) this.trail.splice(0, 1000);
  }

  setPhase(ph, msg, level) {
    if (this.ctrl.phase === ph) return;
    this.ctrl.phase = ph; this.ctrl.tPhase = this.t; this.ctrl.integ = [0, 0, 0];
    this.log(msg || ph, level || 'phase');
  }
  abort(reason) {
    const c = this.ctrl;
    if (c.mode === 'DIVERT') return;
    c.mode = 'DIVERT'; c.abortReason = reason; c.divTerm = false;
    // before the burn there is time to retarget the glide far offshore; after it, splash at the offshore aim point
    const early = ['BELLY-FLOP', 'IGNITION'].includes(c.phase);
    const aim = this.holdPoint();
    c.abortPt = early ? SITE.abort.slice() : [Math.max(aim[0], SITE.coastX) + 35, aim[1]];
    this.arms.cmd = 1;
    this.log(`ABORT — ${reason}. ${early ? 'Retargeting to offshore divert zone.' : 'Wave-off: hoverslam into the water at the offshore aim point.'}`, 'crit');
    if (['LANDING BURN', 'TOWER DIVERT', 'CATCH'].includes(c.phase)) this.setPhase('DIVERT', 'Divert — tower protected, hoverslam to water', 'warn');
  }

  /* ---------------- ignition altitude (with adaptive fail-safe) ---------------- */
  // minimum safe relight altitude: spin-up + flip + braking at 80 % of two-engine authority + gate height
  requiredIgnAlt(vDown) {
    const P = this.P, m = this.massEst();
    const aDec = Math.max(2, (2 * VEH.T_NOM * 0.95) / m - G0) * 0.8;
    const tStart = P.startMean + 2.5 * P.startSigma;
    vDown = Math.min(vDown, 60);                         // evaluated for the near-ground terminal sink rate
    const vAfter = 0.95 * vDown;                         // the flip mostly brakes horizontally; sink rate survives it
    return vDown * (tStart + 3.2) + vAfter * vAfter / (2 * aDec) + this.holdZ() + 30;
  }
  // propellant for the rest of the hoverslam: Δv = |v| + gravity loss over tgo + lateral divert + terminal descent
  propForBurn(v, tgo, lateral, m) {
    const dv = len(v) + G0 * tgo + (lateral > 0 ? 2.2 * lateral / Math.max(3, tgo) : 0) + G0 * 4.5;
    return m * (1 - Math.exp(-dv / (VEH.ISP * G0)));
  }

  /* ================= GUIDANCE, NAVIGATION & CONTROL (50 Hz) ================= */
  control(dt) {
    const P = this.P, c = this.ctrl, n = this.nav;
    if (this.frozen) return;
    const p = n.p, v = n.v, q = n.q, w = n.w;
    const m = this.massEst(), cmN = P.cmOffset;
    const X = qrot(q, [1, 0, 0]), Yb = qrot(q, [0, 1, 0]);
    const windFc = this.windMean(p[2], true);
    const vrel = sub(v, windFc);
    const rho = 1.225 * Math.exp(-Math.max(0, p[2]) / 8500);
    const aeN = this.aero(vrel, q, rho, 1, cmN, this.flaps.map(f => f.cmd), w);
    const aAero = scl(qrot(q, aeN.Fb), 1 / m);
    const nRun = this.nRunning(), nLit = this.nLit();
    const up = [0, 0, 1];
    let Xd = null, Yh = null, Zd = null, maxRate = 12 * DEG, wn = P.attBW * 0.55, aT = null;
    let flapBias = [0.5, 0.5, 0.5, 0.5];
    const aim = this.holdPoint(), cp0 = this.catchPoint();
    const catchP = [clamp(p[0], cp0[0] - P.railWindow, cp0[0] + P.railWindow), cp0[1], cp0[2]];   // rails run E-W along the approach
    const towerC = [(SITE.tower.x0 + SITE.tower.x1) / 2, 0];
    const powered = ['FLIP', 'LANDING BURN', 'TOWER DIVERT', 'CATCH', 'DIVERT'].includes(c.phase);

    // ---------- in-flight monitors (fail-safes) ----------
    const nearTower = ['TOWER DIVERT', 'CATCH'].includes(c.phase) && this.armLoad < 0.2 * m * G0;
    if (!n.gpsOk && nearTower && !c.committed && this.t - n.lastGps > 1.5) this.abort('Navigation degraded (GNSS loss) during tower approach');
    if (c.engAnomaly && !P.allowEngineOutCatch && nearTower && !c.committed) this.abort('Engine anomaly during tower approach');
    if (this.arms.fault && nearTower && !c.committed) this.abort('Tower reports chopstick hydraulics fault');
    if (c.phase === 'TOWER DIVERT' && c.mode === 'NOMINAL') {
      // stopping point at full lateral braking authority must not pass the catch point toward the tower
      const aStop = 0.6 * G0 * Math.tan(P.maxTilt * DEG), vh = Math.hypot(v[0], v[1]);
      const stop = [p[0] + v[0] * vh / (2 * aStop), p[1] + v[1] * vh / (2 * aStop)];
      const tx = (SITE.tower.x0 + SITE.tower.x1) / 2;
      if (Math.hypot(stop[0] - tx, stop[1]) < 10.5 && p[2] < 240) this.abort('Predicted lateral overshoot toward tower');
    }
    const towerWindMeas = len(this.windMean(P.armHeight, false)) + len(this.gust) * 0.7;
    c.towerWind = towerWindMeas;
    if (c.mode === 'NOMINAL' && !c.go && ['LANDING BURN', 'FLIP'].includes(c.phase)) {
      const pf = [p[0] + v[0] * 3, p[1] + v[1] * 3];
      const dT = Math.min(Math.hypot(p[0] - towerC[0], p[1]), segPointDist(p[0], p[1], pf[0], pf[1], towerC[0], towerC[1]));
      if (dT < 34 && p[2] < 175 && c.phase !== 'FLIP') this.abort('Predicted tower keep-out breach without GO');
    }
    // engine-out recovery: relight a standby engine when thrust is lost / insufficient in powered flight
    if (powered && c.phase !== 'FLIP' && this.prop > 300) {
      const sb = this.engines.find(e => e.st === 'shutdown');
      c.satT = (c.thrNeed || 0) > nLit * 0.98 ? (c.satT || 0) + dt : 0;
      if (sb && (nLit === 0 || c.satT > 0.8)) { c.satT = 0; this.log(`Insufficient thrust — emergency relight of Raptor ${sb.id + 1}`, 'warn'); this.startEngine(sb.id); }
    }
    if (this.prop <= 0 && nRun === 0 && c.phase !== 'CAUGHT' && !c.propOut) { c.propOut = true; this.log('Propellant depleted', 'crit'); }

    switch (c.phase) {
      /* ---------------- BELLY-FLOP GLIDE ---------------- */
      case 'BELLY-FLOP': {
        const tgt = c.mode === 'DIVERT' ? [c.abortPt[0], c.abortPt[1], 0] : aim;
        c.target = tgt;
        const hReq = this.requiredIgnAlt(Math.max(0, -v[2]));
        c.hReq = hReq;
        c.hIgnEff = P.adaptiveIgn ? Math.max(P.ignAlt, hReq) : P.ignAlt;
        if (P.adaptiveIgn && hReq > P.ignAlt && !c.adaptLogged && p[2] < hReq + 300) { c.adaptLogged = true; this.log(`Adaptive relight: raising relight altitude ${P.ignAlt.toFixed(0)} → ${hReq.toFixed(0)} m`, 'warn'); }
        // catch poll #1 (pre-relight): weather, navigation, tower, propellant — last chance to retarget the glide offshore
        if (c.mode === 'NOMINAL' && !c.poll1 && p[2] < c.hIgnEff + 2200) {
          c.poll1 = true;
          const need = this.propForBurn(v, 12, P.holdOffset, m) + P.propReserve * 1000;
          const ck = {
            [`Tower winds < ${P.windLimit} m/s`]: towerWindMeas < P.windLimit,
            'Navigation': n.gpsOk,
            'Tower ready': !this.arms.fault,
            'Propellant for catch': this.prop > need,
          };
          c.checks = ck; c.pollName = 'POLL 1 · PRE-RELIGHT';
          const bad = Object.entries(ck).filter(([k, ok]) => !ok).map(([k]) => k);
          if (bad.length) this.abort(`Pre-relight catch poll NO-GO (${bad.join(', ')})`);
          else this.log('Catch poll 1 (pre-relight): GO to continue toward offshore aim point', 'good');
        }
        if (p[2] <= c.hIgnEff) {
          for (let i = 0; i < 3; i++) this.startEngine(i);
          c.tIgn = this.t;
          this.setPhase('IGNITION', `Raptor relight command at ${p[2].toFixed(0)} m — 3 engines spinning up`);
          break;
        }
        // E-guidance (min-energy, final position + final velocity constraint) to the ignition gate
        const tgo = Math.max(2.5, (p[2] - c.hIgnEff) / Math.max(15, -v[2]));
        // tail-first arrival speed, cancelled by the flip's thrust kick; aim long by the spin-up drift
        const vArr = P.flipKick * 112000 / m, lead = vArr * (P.startMean + 0.12);   // flip Δv ∝ 1/mass
        // the ship drifts with the air mass through spin-up + flip (~1.5 s equivalent): aim upwind using the forecast
        const wl = this.windMean(300, true);
        const rf = [tgt[0] + c.heading[0] * lead - 1.5 * wl[0], tgt[1] + c.heading[1] * lead - 1.5 * wl[1]];
        const vf = [-c.heading[0] * vArr, -c.heading[1] * vArr];
        let ah = [6 * (rf[0] - p[0]) / (tgo * tgo) - (4 * v[0] + 2 * vf[0]) / tgo,
                  6 * (rf[1] - p[1]) / (tgo * tgo) - (4 * v[1] + 2 * vf[1]) / tgo, 0];
        ah = scl(ah, P.glideGain / 0.25);
        ah = clampLen(ah, G0 * Math.tan(P.glideTiltMax * DEG));
        c.ahCmd = ah;
        const F = norm([ah[0], ah[1], G0]);
        Zd = scl(F, -1);
        const hh = [c.heading[0], c.heading[1], 0];
        Xd = norm(sub(hh, scl(Zd, dot(hh, Zd))));
        Yh = cross(Zd, Xd);
        maxRate = 8 * DEG; wn = P.attBW * 0.6;
        break;
      }
      /* ---------------- ENGINE START ---------------- */
      case 'IGNITION': {
        const hh = norm([c.heading[0], c.heading[1], 0]);
        const pitchUp = clamp((this.t - c.tIgn) / 2.5, 0, 1) * 25 * DEG;
        // keep steering toward the flip-start aim point while the engines spin up (cross-track via bank)
        const tgtI = c.mode === 'DIVERT' ? [c.abortPt[0], c.abortPt[1], 0] : aim;
        const vArrI = P.flipKick * 112000 / m, tsp = Math.max(0.8, P.startMean - (this.t - c.tIgn));
        const wlI = this.windMean(300, true);
        const rfI = [tgtI[0] + c.heading[0] * vArrI * 0.25 - 1.5 * wlI[0], tgtI[1] + c.heading[1] * vArrI * 0.25 - 1.5 * wlI[1]];
        const vfI = [-c.heading[0] * vArrI, -c.heading[1] * vArrI];
        let ahI = [6 * (rfI[0] - p[0] - v[0] * tsp) / (tsp * tsp) + 2 * (v[0] - vfI[0]) / tsp, 6 * (rfI[1] - p[1] - v[1] * tsp) / (tsp * tsp) + 2 * (v[1] - vfI[1]) / tsp, 0];
        const side = norm(cross(up, hh));
        const aCross = clamp(dot(ahI, side), -G0 * Math.tan(P.glideTiltMax * DEG), G0 * Math.tan(P.glideTiltMax * DEG));
        Zd = norm(add(scl(up, -G0), scl(side, -aCross)));
        Xd = norm(add(scl(hh, Math.cos(pitchUp)), scl(up, Math.sin(pitchUp))));
        Xd = norm(sub(Xd, scl(Zd, dot(Xd, Zd))));
        Yh = cross(Zd, Xd);
        maxRate = 15 * DEG; wn = P.attBW * 0.8;
        flapBias = [0.15, 0.15, 0.9, 0.9];
        const tLimit = P.startMean + 3 * P.startSigma + 1.2;
        if (nRun >= 1 && this.thrustTotal > 0.25 * VEH.T_NOM) this.setPhase('FLIP', 'Flip maneuver — gimbal + flaps rotating to vertical');
        if (this.t - c.tIgn > tLimit) {
          if (nRun === 0) { if (!c.noEng) { c.noEng = true; this.log('NO ENGINES AT MAINSTAGE — vehicle cannot be saved', 'crit'); } }
          else this.setPhase('FLIP', 'Flip maneuver (late)');
        }
        break;
      }
      /* ---------------- POWERED: FLIP / LANDING BURN / TOWER DIVERT / CATCH / DIVERT ---------------- */
      default: {
        if (c.phase === 'CAUGHT') {
          for (const e of this.engines) e.cmd = 0;
          this.gimbalCmd = [0, 0]; this.rcsCmd = [0, 0, 0];
          this.flaps.forEach(f => f.cmd = 0.3);
          c.catchSettle += dt;
          if (c.catchSettle > 4 && !this.done && this.armLoad > 0.8 * this.mass() * G0) {
            this.finish('success', `ship secured on chopsticks — pins ${this.stats.catchRail.toFixed(1)} m along the rails, ${this.stats.catchErr.toFixed(2)} m off centreline, ${(this.prop / 1000).toFixed(2)} t propellant remaining`);
          }
          return;
        }
        if (c.phase === 'LOST') return;
        // relight result check
        if (c.tIgn !== null && !c.relightChecked && this.t - c.tIgn > P.startMean + 3 * P.startSigma + 1.0) {
          c.relightChecked = true;
          if (nRun < P.minEnginesFlip) { if (nRun > 0) this.abort(`Only ${nRun} of 3 engines relit (minimum ${P.minEnginesFlip})`); }
          else if (nRun < 3) { c.engAnomaly = true; this.log(`${nRun} of 3 engines at mainstage — engine-out flagged`, 'warn'); }
          else this.log('All 3 Raptors healthy at mainstage', 'good');
        }

        let tiltLim = P.maxTilt;
        const ec = Math.hypot(catchP[0] - p[0], catchP[1] - p[1]);
        if (c.phase === 'CATCH' || (c.mode === 'DIVERT' && c.divTerm)) {
          // ---- terminal descent: precise horizontal hold + rate-limited sink into the arms / water
          // terminal: hold the along-rail position captured at the gate (rails tolerate it); centre across the arms
          if (c.phase === 'CATCH' && c.catchX === undefined) c.catchX = clamp(p[0], cp0[0] - 3, cp0[0] + 3);
          const tp = c.phase === 'CATCH' ? [c.catchX, cp0[1], cp0[2]] : [c.abortPt[0], c.abortPt[1], 0];
          const zc = c.phase === 'CATCH' ? this.pinContactZ() - 1.0 : this.splashZ() - 3;
          // over open water a divert just nulls its drift; near the coast it still pushes offshore
          const eh = c.phase === 'CATCH' || p[0] < SITE.coastX + 25 ? [tp[0] - p[0], tp[1] - p[1]] : [0, 0];
          c.hInt = c.hInt || [0, 0];
          // conditional integration (anti-windup): only once converged near the centreline
          if (c.phase === 'CATCH' && Math.hypot(eh[0], eh[1]) < 2 && Math.hypot(v[0], v[1]) < 1) c.hInt = [clamp(c.hInt[0] + eh[0] * dt, -5, 5), clamp(c.hInt[1] + eh[1] * dt, -5, 5)];
          tiltLim = 10;   // single-engine gimbal authority: gentle PD (wn ~0.6 rad/s) + slow integral
          let ah = [0.35 * eh[0] - 1.1 * v[0] + 0.06 * c.hInt[0], 0.35 * eh[1] - 1.1 * v[1] + 0.06 * c.hInt[1]];
          const amax = G0 * Math.tan(tiltLim * DEG); const hm = Math.hypot(ah[0], ah[1]); if (hm > amax) ah = [ah[0] * amax / hm, ah[1] * amax / hm];
          const vzd = -clamp(0.4 * (p[2] - zc), P.termRate, c.phase === 'CATCH' ? 2.5 : 4);
          const az = 2.2 * (vzd - v[2]);
          aT = [ah[0] - aAero[0], ah[1] - aAero[1], Math.max(0.3 * G0, az + G0 - aAero[2])];
          c.target = tp; c.tgo = Math.max(0, (p[2] - zc) / Math.max(0.5, -v[2]));
        } else {
          // ---- hoverslam: E-guidance to a gate point with a final sink-rate constraint
          let rf, vf;
          if (c.mode === 'DIVERT') {
            // stay above the tower only if it lies between the ship and the splash point
            const ab = [c.abortPt[0] - p[0], c.abortPt[1] - p[1]], tw = [towerC[0] - p[0], towerC[1] - p[1]];
            const dTw = Math.hypot(tw[0], tw[1]);
            const nearT = c.nearT = p[2] > 120 && (dTw < 16 || ((ab[0] * tw[0] + ab[1] * tw[1]) > 0 && segPointDist(p[0], p[1], c.abortPt[0], c.abortPt[1], towerC[0], towerC[1]) < 18));
            rf = [c.abortPt[0], c.abortPt[1], nearT ? Math.max(p[2], 170) : this.splashZ() + 15]; vf = [0, 0, nearT ? 0 : -3]; tiltLim = Math.max(P.maxTilt, 25);
          }
          else if (c.phase === 'TOWER DIVERT' || c.go) { rf = catchP; vf = [0, 0, -P.gateSink]; }
          else { rf = aim; vf = [0, 0, -P.gateSink]; }
          // safely offshore the splash point no longer matters: just null the drift, vertical braking gets the thrust
          if (c.mode === 'DIVERT' && p[0] > SITE.coastX + 25 && !c.nearT && Math.hypot(rf[0] - p[0], rf[1] - p[1]) > 15) {
            const tq = Math.min(c.tgo || 5, 6) * 0.6;
            rf = [p[0] + v[0] * tq, p[1] + v[1] * tq, rf[2]]; tiltLim = 12;
          }
          c.target = rf;
          aT = this.eGuidance(p, v, rf, vf, aAero, c.phase === 'FLIP' ? 10 : tiltLim, c.mode === 'DIVERT' ? 20 : 40);
          // gate reached -> terminal descent
          if (c.phase !== 'FLIP' && !c.nearT && (c.tgo < 1.5 || p[2] < rf[2] + 6)) {
            if (c.mode === 'DIVERT') { if (!c.divTerm) { c.divTerm = true; this.log('Divert gate reached — terminal descent to the water', 'warn'); } }
            else if (c.phase === 'TOWER DIVERT') {
              if (Math.abs(p[0] - cp0[0]) > 7 || Math.abs(p[1] - cp0[1]) > 4) this.abort(`Arrived at catch gate ${ec.toFixed(1)} m off-centre`);
              else { this.setPhase('CATCH', 'Catch gate: single engine, chopsticks closing, terminal sink into the arms'); this.arms.cmd = 0; c.hInt = [0, 0]; }
            }
          }
        }
        if (c.phase !== 'FLIP' && nRun > 0) {
          const aMax = nRun * VEH.T_NOM * 0.97 / m, mag = len(aT);
          if (mag > aMax) {
            // vertical first, but always keep a lateral share (otherwise a braking-saturated ship drifts into the tower)
            const hm = Math.hypot(aT[0], aT[1]), hMin = Math.min(hm, aMax * Math.sin(14 * DEG));
            const az = Math.min(aT[2], Math.sqrt(aMax * aMax - hMin * hMin)), hAllow = Math.sqrt(Math.max(0, aMax * aMax - az * az));
            if (hm > hAllow) { aT = [aT[0] * hAllow / hm, aT[1] * hAllow / hm, az]; }
          }
        }
        Xd = norm(aT);
        if (P.trimComp && c.phase !== 'FLIP' && (this.thrustTotal || 0) > 1e5) {
          // steer the THRUST vector, not the body axis: remove the current gimbal trim from the attitude target
          // only the slow trim (low-passed gimbal, τ≈3 s), never the active attitude-control deflection
          c.gTrim = c.gTrim || [0, 0];
          c.gTrim = [c.gTrim[0] + (this.gimbal[0] - c.gTrim[0]) * dt / 3, c.gTrim[1] + (this.gimbal[1] - c.gTrim[1]) * dt / 3];
          const tdw = qrot(q, norm([1, Math.tan(c.gTrim[0]), Math.tan(c.gTrim[1])]));
          Xd = norm(add(Xd, sub(X, tdw)));
        }
        if (c.phase === 'FLIP') { Yh = Yb; maxRate = 30 * DEG; wn = P.attBW; flapBias = [0.1, 0.1, 1, 1]; }
        else { Yh = [0, Yb[1] >= 0 ? 1 : -1, 0]; // gain-schedule the attitude loop on available gimbal authority (∝ thrust): avoids gimbal-saturation limit cycles
          const auth = clamp((this.thrustTotal || VEH.T_NOM) / 1.6e6, 0.45, 1.3);
          maxRate = (c.phase === 'CATCH' ? 12 : 22) * DEG * Math.min(1, auth); wn = P.attBW * auth; flapBias = [0.2, 0.2, 0.55, 0.55]; }

        // ---- engine count & throttle: fewest engines that can fly the demand without saturating
        const need = m * len(aT);
        c.thrNeed = need / VEH.T_NOM;
        const tilt = Math.acos(clamp(dot(X, Xd), -1, 1));
        c.attErr = tilt;
        if (c.phase === 'FLIP' && X[2] > 0.9 && this.t - c.tPhase > 1.0) this.setPhase(c.mode === 'DIVERT' ? 'DIVERT' : c.go ? 'TOWER DIVERT' : 'LANDING BURN', c.go ? 'Flip complete — hoverslam diverting into the tower' : 'Flip complete — hoverslam toward offshore aim point');
        if (c.phase !== 'FLIP' && nRun >= 2) {
          // keep two engines for authority through the divert; single engine only for the terminal sink (or if overbraking)
          const overbrake = c.phase === 'TOWER DIVERT' ? 0.8 : 1.03;   // tolerate mild overbraking in the divert to keep 2-engine gimbal authority
          const tooMuch = (['CATCH'].includes(c.phase) || c.divTerm) && need < (nRun - 1) * VEH.T_NOM * 0.75 || need < nRun * VEH.T_NOM * P.minThrottle * overbrake;
          if (tooMuch && this.t - (c.lastShut || 0) > 0.6) {
            const idx = this.engines.map((e, i) => e.st === 'running' ? i : -1).filter(i => i >= 0).pop();
            c.lastShut = this.t;
            this.shutdownEngine(idx, nRun === 3 ? 'flip complete, burn on 2' : 'final descent on single engine');
          }
        }
        let thr = nRun > 0 ? need / (nRun * VEH.T_NOM) : 1;
        if (c.phase === 'FLIP' && nRun > 0) thr = Math.max(thr, 3 * P.minThrottle / nRun);   // engine-out: keep the flip's braking kick
        for (const e of this.engines) e.cmd = clamp(thr, P.minThrottle, 1);
        if (c.phase !== 'FLIP' && tilt > 50 * DEG) { c.locT += dt; if (c.locT > 2 && !c.locLogged) { c.locLogged = true; this.log('LOSS OF ATTITUDE CONTROL', 'crit'); } } else c.locT = 0;

        // ---- propellant bookkeeping (for GO gate + UI)
        c.propNeedCatch = c.phase === 'CATCH'
          ? m * (1 - Math.exp(-(G0 * Math.max(0, p[2] - this.pinContactZ()) / Math.max(0.8, P.termRate) + 5) / (VEH.ISP * G0)))
          : this.propForBurn(v, c.tgo || 5, ec, m);
        c.propNeedCatch += P.propReserve * 1000;

        // ---- divert gate: single GO/NO-GO decision shortly after the flip
        if (c.phase === 'LANDING BURN' && c.mode === 'NOMINAL' && !c.gateDone && this.t - c.tPhase > 0.3 && !this.engines.some(e => e.st === 'starting')) this.divertGate(p, v, X, w, m, ec);

        // ---- catch phase monitors & load transfer
        if (c.phase === 'CATCH') {
          const ecT = Math.hypot(SITE.catchPt[0] - this.p[0], SITE.catchPt[1] - this.p[1]);
          const pinAbove = this.p[2] - this.pinContactZ();
          if (this.arms.fault && pinAbove < 8 && this.arms.a > 0.3 && !c.committed) this.abort('Chopsticks failed to close');
          // envelope: pins may land anywhere along the rails (x), but must be centred between the arms (y)
          const ex = this.p[0] - SITE.catchPt[0], ey = this.p[1] - SITE.catchPt[1];
          // commit point: once the pins are within 6 m of the rails there is no wave-off — complete the catch
          if (!c.committed && pinAbove < 6) { c.committed = true; this.log('Commit point — pins 6 m above the rails, catch committed', 'phase'); }
          if (!c.committed && (ex < -10 || ex > 9 || Math.abs(ey) > 3) && this.armLoad < 0.2 * m * G0) this.abort(`Off catch envelope (rail ${ex.toFixed(1)} m, cross ${ey.toFixed(1)} m)`);
          if (pinAbove < -3.0 && this.armLoad < 0.1 * m * G0) this.abort('Pins below arm rails without load — missed catch');
          if (this.arms.a < 0.03 && this.armLoad > 0.04 * this.mass() * G0) c.loadT += dt; else c.loadT = 0;
          if (c.loadT > 0.15 && c.mode === 'NOMINAL') {
            this.stats.catchErr = Math.abs(this.p[1] - SITE.catchPt[1]); this.stats.catchRail = this.p[0] - SITE.catchPt[0]; this.stats.catchVz = this.v[2];
            for (let i = 0; i < 3; i++) this.shutdownEngine(i);
            this.setPhase('CAUGHT', `Pin contact at ${(-this.v[2]).toFixed(1)} m/s — engine cutoff, load transferring to tower`, 'good');
            c.catchSettle = 0;
            return;
          }
        }
        if (c.mode === 'DIVERT' && c.phase !== 'DIVERT' && c.phase !== 'FLIP') this.setPhase('DIVERT', 'Divert — tower protected, hoverslam to water', 'warn');
      }
    }

    // ---------------- attitude control ----------------
    if (!Xd) return;
    if (!Zd) { const Yp = norm(sub(Yh, scl(Xd, dot(Yh, Xd)))); Zd = cross(Xd, Yp); }
    const Yd = cross(Zd, Xd);
    const qd = qFromAxes(Xd, Yd, Zd);
    const e = qToRotVec(qmul(qconj(q), qd));     // body-frame error
    if (c.phase === 'BELLY-FLOP' || c.phase === 'IGNITION') c.attErr = len(e);
    const zeta = 0.9, kp = wn * wn, kd = 2 * zeta * wn;
    let wdes = clampLen(scl(e, kp / kd), maxRate);
    const I = this.inertia(m);
    if (c.phase === 'BELLY-FLOP') c.integ = add(c.integ, scl(e, dt * 0.15));
    c.integ = clampLen(c.integ, 0.2);
    const alpha = [kd * (wdes[0] - w[0]) + kp * c.integ[0], kd * (wdes[1] - w[1]) + kp * c.integ[1], kd * (wdes[2] - w[2]) + kp * c.integ[2]];
    const gyro = cross(w, [I[0] * w[0], I[1] * w[1], I[2] * w[2]]);
    const tauCmd = [I[0] * alpha[0] + gyro[0], I[1] * alpha[1] + gyro[1], I[2] * alpha[2] + gyro[2]];
    c.tauCmd = tauCmd;
    // feedforward: subtract modelled aero torque (body only, flaps handled in allocation)
    const aeBody = this.aero(vrel, q, rho, 1, cmN, [0, 0, 0, 0], w);
    let tauBodyAero = aeBody.tau;
    for (let i = 0; i < 4; i++) tauBodyAero = sub(tauBodyAero, scl(aeBody.flapTau1[i], 0.1));
    let need = sub(tauCmd, tauBodyAero);

    // 1) engine gimbal (pitch / yaw)
    const Tsum = this.thrustTotal || 0;
    const Lx = VEH.X_GIMBAL - cmN;
    if (Tsum > 1e5) {
      const tg = Math.tan(VEH.GIMBAL_MAX);
      const gb = clamp(need[1] / (-Lx * Tsum), -tg, tg), ga = clamp(need[2] / (Lx * Tsum), -tg, tg);
      this.gimbalCmd = [Math.atan(ga), Math.atan(gb)];
      need = sub(need, [0, -Lx * Tsum * gb, Lx * Tsum * ga]);
    } else this.gimbalCmd = [0, 0];

    // 2) flaps — regularized least-squares allocation over 4 surfaces
    const A = aeBody.flapTau1.map(t => scl(t, 0.9));
    let S = 0; for (const a of A) S = Math.max(S, len(a));
    let eCmd = flapBias.slice();
    if (S > 2e4) {
      const b = scl(need, 1 / S), An = A.map(a => scl(a, 1 / S));
      const lam = 0.08;
      const M = [], r = [];
      for (let i = 0; i < 4; i++) {
        M.push([]);
        for (let j = 0; j < 4; j++) M[i].push(dot(An[i], An[j]) + (i === j ? lam : 0));
        r.push(dot(An[i], b) + lam * (flapBias[i] - 0.5) + dot(An[i], [0, 0, 0]));
      }
      // solve for deviation around 0.5 centered so bias pulls toward flapBias
      const base = flapBias.map(() => 0.5);
      const bAdj = sub(b, An.reduce((s, a, i) => add(s, scl(a, base[i])), [0, 0, 0]));
      for (let i = 0; i < 4; i++) r[i] = dot(An[i], bAdj) + lam * (flapBias[i] - base[i]);
      const x = solve4(M, r);
      eCmd = x.map((d, i) => clamp(base[i] + d, 0, 1));
      let ach = [0, 0, 0];
      for (let i = 0; i < 4; i++) ach = add(ach, scl(A[i], this.flaps[i].e));
      need = sub(need, ach);
    }
    this.flaps.forEach((f, i) => f.cmd = eCmd[i]);

    // 3) RCS takes the residual
    this.rcsCmd = [clamp(need[0], -VEH.RCS[0], VEH.RCS[0]), clamp(need[1], -VEH.RCS[1], VEH.RCS[1]), clamp(need[2], -VEH.RCS[2], VEH.RCS[2])];
  }

  /* E-guidance: a = 6(rf - r)/tgo² - (4v + 2vf)/tgo. Choosing tgo from a constant-deceleration stop of the
     vertical channel makes the vertical command a constant decel (v - vf)/tgo — a true hoverslam (no hover). */
  eGuidance(p, v, rf, vf, aAero, tiltLim, tgoMax) {
    const c = this.ctrl;
    const dz = p[2] - rf[2], vd = -v[2], vfd = -vf[2];
    const tgoV = (dz > 0.3 && vd + vfd > 1) ? 2 * dz / (vd + vfd) : 3 + Math.abs(dz) / 2;
    // lateral: braking-distance-aware velocity law (robust to the ~1 s thrust-vector lag)
    const e = [rf[0] - p[0], rf[1] - p[1]], d = Math.hypot(e[0], e[1]) || 1e-6;
    const aBr = this.lateralBrake(tiltLim);
    const tLat = this.lateralTime(d, (v[0] * e[0] + v[1] * e[1]) / d, aBr);
    // couple the channels: the vertical hoverslam stretches if the lateral divert needs longer
    // couple the channels (lateral finishes ~2 s before the vertical gate) — except an over-water divert, which just hoverslams
    const overWater = c.mode === 'DIVERT' && p[0] > SITE.coastX + 12 && !c.nearT;
    const tgo = clamp(overWater || c.nearT ? tgoV : Math.max(tgoV, tLat + 2.0), 0.8, tgoMax);
    c.tgo = tgo; c.tLat = tLat;
    const az = 6 * (rf[2] - p[2]) / (tgo * tgo) - (4 * v[2] + 2 * vf[2]) / tgo;
    const vSq = Math.sqrt(2 * aBr * d), vmag = Math.min(22, 0.3 * d, vSq);   // linear zone: wn≈0.42, ζ≈0.7 with the velocity loop
    let vh = [e[0] / d * vmag, e[1] / d * vmag];
    // feed-forward the braking profile's own deceleration so the loop doesn't need a big tracking error to brake
    const vAl = (v[0] * e[0] + v[1] * e[1]) / d;
    const ff = vmag === vSq && vAl > 0.5 && d > 4 ? -aBr * Math.min(1, vAl / Math.max(0.5, vSq)) : 0;
    if ((!c.go || c.mode === 'DIVERT') && p[2] < 175) {          // tower keep-out repulsion when not cleared in
      const tx = (SITE.tower.x0 + SITE.tower.x1) / 2, dx = p[0] - tx, dy = p[1], dt = Math.hypot(dx, dy) || 1;
      if (dt < 60) { const k = (60 - dt) * 0.25; vh = [vh[0] + dx / dt * k, vh[1] + dy / dt * k]; }
      const vm = Math.hypot(vh[0], vh[1]); if (vm > 14) vh = [vh[0] * 14 / vm, vh[1] * 14 / vm];
    }
    const a = [0.6 * (vh[0] - v[0]) + ff * e[0] / d, 0.6 * (vh[1] - v[1]) + ff * e[1] / d, az];
    let aT = [a[0] - aAero[0], a[1] - aAero[1], a[2] + G0 - aAero[2]];
    if (aT[2] < 0.3 * G0) aT[2] = 0.3 * G0;
    const hmag = Math.hypot(aT[0], aT[1]), lim = aT[2] * Math.tan(tiltLim * DEG);
    if (hmag > lim) { aT[0] *= lim / hmag; aT[1] *= lim / hmag; }
    return aT;
  }
  lateralBrake(tiltLim) { return 0.42 * G0 * Math.tan(tiltLim * DEG); }
  // time for a rest-to-rest lateral move of d metres at braking accel aBr, credited for velocity already toward the target
  lateralTime(d, v0, a) {
    // exact minimum time (bang-bang, ±a) to travel d and arrive at rest, from initial speed v0 along the line
    let t;
    if (v0 < 0) t = -v0 / a + 2 * Math.sqrt((d + v0 * v0 / (2 * a)) / a);               // first stop, then rest-to-rest
    else if (v0 * v0 / (2 * a) <= d) t = (-v0 + 2 * Math.sqrt(v0 * v0 / 2 + a * d)) / a;  // accelerate then brake
    else { const over = v0 * v0 / (2 * a) - d; t = v0 / a + 2 * Math.sqrt(over / a); }      // overshoot and come back
    return t + 1.0;                                                                          // attitude-lag allowance
  }

  // Single divert gate in the landing burn: GO → shift the hoverslam target from offshore into the tower
  divertGate(p, v, X, w, m, ec) {
    const P = this.P, c = this.ctrl;
    c.gateDone = true;
    const cz = this.catchPoint()[2], dz = p[2] - cz, vd = -v[2];
    const tgoV = dz > 0 && vd + P.gateSink > 1 ? 2 * dz / (vd + P.gateSink) : 3;
    const tx = clamp(p[0], SITE.catchPt[0] - P.railWindow, SITE.catchPt[0] + P.railWindow), ty = SITE.catchPt[1];
    const tLat = this.lateralTime(ec, ((tx - p[0]) * v[0] + (ty - p[1]) * v[1]) / Math.max(1e-6, ec), this.lateralBrake(P.maxTilt));
    const tgo = Math.max(tgoV, tLat);
    const need = this.propForBurn(v, tgo, ec, m) + P.propReserve * 1000;
    c.propNeedCatch = need;
    const ck = {
      'Engines healthy': this.nRunning() >= P.minEnginesFlip && ((!c.engAnomaly && !this.engines.some(e => e.st === 'failed')) || P.allowEngineOutCatch),
      'Navigation': this.nav.gpsOk,
      [`Tower winds < ${P.windLimit} m/s`]: c.towerWind < P.windLimit,
      'Tower ready': !this.arms.fault && this.arms.a > 0.95,
      'Divert time ≤ burn time': tLat + 2.0 < tgoV * 1.6,
      'Propellant margin': this.prop > need,
      'Attitude control': c.locT === 0 && len(w) < 45 * DEG,
    };
    c.checks = ck; c.pollName = 'POLL 2 · DIVERT GATE';
    const bad = Object.entries(ck).filter(([k, ok]) => !ok).map(([k]) => k);
    if (bad.length) { this.abort(`Divert gate NO-GO (${bad.join(', ')})`); return; }
    c.go = true;
    if (c.phase === 'FLIP') { this.log(`GO FOR CATCH — hoverslam retargeted ${ec.toFixed(0)} m into the tower (tgo ${tgo.toFixed(1)} s, ${((this.prop - need) / 1000 + P.propReserve).toFixed(1)} t margin)`, 'good'); return; }
    this.setPhase('TOWER DIVERT', `GO FOR CATCH — diverting ${ec.toFixed(0)} m into the tower (tgo ${tgo.toFixed(1)} s, ${((this.prop - need) / 1000 + P.propReserve).toFixed(1)} t margin)`, 'good');
  }
}

function closestOnSeg(x, y, s) {
  const t = clamp((x - s.px) * s.dx + (y - s.py) * s.dy, 0, SITE.armLen);
  return [s.px + s.dx * t, s.py + s.dy * t];
}
function segPointDist(ax, ay, bx, by, px, py) {
  const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy || 1e-9;
  const t = clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0, 1);
  return Math.hypot(ax + dx * t - px, ay + dy * t - py);
}
function solve4(M, r) {
  const n = 4, a = M.map((row, i) => row.concat([r[i]]));
  for (let i = 0; i < n; i++) {
    let mx = i; for (let k = i + 1; k < n; k++) if (Math.abs(a[k][i]) > Math.abs(a[mx][i])) mx = k;
    [a[i], a[mx]] = [a[mx], a[i]];
    const d = a[i][i] || 1e-9;
    for (let k = i + 1; k < n; k++) { const f = a[k][i] / d; for (let j = i; j <= n; j++) a[k][j] -= f * a[i][j]; }
  }
  const x = [0, 0, 0, 0];
  for (let i = n - 1; i >= 0; i--) { let s = a[i][n]; for (let j = i + 1; j < n; j++) s -= a[i][j] * x[j]; x[i] = s / (a[i][i] || 1e-9); }
  return x;
}

const CatchSim = { Sim, DEFAULTS, VEH, SITE, G0, DEG, qrot, qFromAxes };
if (typeof module !== 'undefined' && module.exports) module.exports = CatchSim;
global.CatchSim = CatchSim;
})(typeof window !== 'undefined' ? window : globalThis);
