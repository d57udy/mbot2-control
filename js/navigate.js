// Map-based navigation: scan into the grid map, plan with A*, drive in short
// gyro legs, re-measure before every straight and replan when blocked.
//
// Pose updates: if the PoseTracker is attached to this bus (pose.bus === bus)
// its listener applies every successful turn/straight and the Navigator does
// not; otherwise the Navigator applies its own moves. Either way each move is
// applied exactly once. Straight legs are driven with streamed `drive`
// commands (js/motion.js, args.leg = true) that the tracker does not see, so
// the Navigator always applies the measured leg distance itself; the app must
// skip its own drive integration for commands with args.leg.
//
// Crashes and stalls mark the contact in the map, flag the pose as uncertain,
// rescan and relocalize. Every scan with a known map is matched against it
// first (js/localize.js, loaded lazily; skipped if missing) and integrated at
// the corrected pose.
//
// Anchors: the first scan of a map (taken at the start) is kept as the
// reference scan. A later scan within ANCHOR_CM of an anchor is matched
// against that scan alone (scan to scan), not against the accumulated map,
// which carries the odometry drift it was built with; a confident anchor
// match sets x, y and the heading (gyro drift) fully. goHome ends with such a
// check at the start and drives a short correction leg if needed.
//
// Map update gating (v0.8): a scan is written into the map only after its
// match was accepted, at the corrected pose. A rejected scan goes to a small
// pending buffer (pendingScans) and is retried after the next accepted scan;
// it never paints a second copy of a door at a drifted pose. Over unknown
// ground (exploring) a scan with too little mapped structure to match is
// integrated at the odometry pose. While the map is frozen (goHome, or
// mapFrozen = true) only localization runs.
//
// Pose uncertainty (unc: sigma xy cm, heading deg) grows with distance,
// turning and time (gyro drift) and sets the match window (3 sigma, clamped)
// and the fusion: a match with covariance corrects each well-determined
// component (heading, cross-track, along-track) Kalman-style; a door or
// corridor that leaves along-track open still fixes heading and cross-track.
//
// Heading (js/heading.js): with a sampler, every sample goes through one
// HeadingEstimator (gyro bias at standstill, gyrodometry per turn, leg and
// sweep, wall snap per sweep against the room axes); the pose heading comes
// from those segments and snaps, not from raw gyro reads.

import { makeCommand } from './bus.js';
import { planPath, simplifyPath, pathToMoves, DEFAULT_INFLATE_CM } from './planner.js';
import { L_SUSPECT, GridMap } from './gridmap.js';
import { driveLeg, turnInPlace } from './motion.js';
import { sweepScan, resampleSweep } from './scan.js';
import { HeadingEstimator } from './heading.js';

const NO_ECHO_CM = 300;
const FRONT_CM = 10;       // robot centre to front bumper
const CONTACT_HALF_CM = 8; // half width of the marked contact
const CONTACT_CLEAR_CM = 9; // a leg clears contacts within this of its centre track (robot radius)
const CONTACT_L = 2;       // one confirming hit per contact cell when the map has no contact layer
const MAX_CRASHES = 4;
const BACKOFF_CM = 10;     // reverse this far when every heading from here is blocked
const MAX_BACKOFFS = 2;
const FOOTPRINT_CM = 12;   // robot radius plus margin, for the known-cells check beside a leg
const MIN_BLIND_LEG_CM = 12; // after a scan here, at most this far along an unseen edge
const EDGE_CM = 15;        // unknown cells this close to obstacle evidence count as an unseen edge
const MAX_FIX_CM = 30;     // largest position correction a routine scan match may apply
const MAX_FIX_DEG = 30;    // largest heading correction (without a gyro) a routine match may apply
const MATCH_BIN_DEG = 5;   // sweep points are resampled to this for matching
const ANCHOR_CM = 60;      // match against an anchor scan within this distance
const ANCHOR_MIN_CONF = 0.5; // an anchor match below this falls back to the map
const MIN_VALID_SWEEP = 20; // a sweep needs this many readings in range to be matched
const MIN_VALID_STEP = 4;   // a step scan (8 to 12 readings) this many
const PENDING_MAX = 3;      // rejected scans kept for a retry
const PENDING_TRIES = 2;    // retries per pending scan
const SLIP_FRAC = 0.05;     // odometry sigma per cm driven
const TURN_FRAC = 0.02;     // heading sigma per degree turned
const DRIFT_DEG_MIN = 2;    // heading sigma growth per minute (gyro drift)
const UNC_MIN = { xy: 2, th: 0.7 };
const WIN_MIN = { xy: 15, th: 10 }, WIN_MAX = { xy: 80, th: 45 };
const KNOWN_FRAC = 0.3;     // below this share of known cells around the robot it is exploring
const SNAP_SIGMA_DEG = 2;   // heading sigma after an applied wall snap
const MAP_HEAD_STEP = 3;    // a map match moves the heading at most this much per scan ...
const MAP_HEAD_MOVE_CM = 10; // ... and only after this much driving since the last accepted map match
const WRITE_TOL = { cm: 6, deg: 3 }; // a scan is written only where the fused pose agrees with its match
const SINGLE_MAX = { xy: 10, th: 5 }; // single readings are written only within this pose sigma
const ANCHOR_UNC = { xy: 10, th: 10 }; // odometry sigma floor when fusing an anchor match (it carries no drift)
const HOME_TOL_CM = 3;     // home check: drive a correction leg beyond this residual
const HOME_FIXES = 2;      // at most this many correction legs

let localizer; // undefined = not tried, null = unavailable
async function loadLocalizer() {
  if (localizer === undefined) {
    try {
      const m = await import('./localize.js');
      localizer = typeof m.matchScan === 'function' ? m : null;
    } catch {
      localizer = null;
    }
  }
  return localizer;
}

function abortError(msg = 'navigation aborted') {
  const e = new Error(msg);
  e.name = 'AbortError';
  return e;
}

class Cancelled extends Error {}

const normDeg = (a) => { const r = ((((a + 180) % 360) + 360) % 360) - 180; return r === -180 ? 180 : r; };
const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const pathLen = (p) => p.slice(1).reduce((s, q, i) => s + dist(p[i], q), 0);

export class Navigator {
  constructor({ bus, map, pose, scan, onEvent, steps = 12, safetyCm = 20, inflateCm = DEFAULT_INFLATE_CM, maxLegCm = 40,
    legsPerScan = Infinity, rescanCm = 120, taskScanCm = 50, settleMs, useYaw = false, beamDeg = 25, maxRangeCm = 250,
    heading, sweepLatencyMs, sampler, sample = sampler, sweepSample, legMode = sample ? 'drive' : 'straight', turnMode = sample ? 'gyro' : 'blocking', legRpm = 40, scanMode = sample ? 'sweep' : 'step', sweepDegS = 45, motionOpts, localize = true, localizer, minMatchConfidence, odomWeight = 0.4, stopAtCm = 15 }) {
    // sampler (alias sample): js/motion.js sampler for legs and sweeps; with one,
    // scans default to 'sweep' = continuous rotation, else 'step' = stop-and-measure
    this.scanMode = scanMode;
    // heading: a HeadingEstimator to share, or false for none; the samplers
    // are wrapped through it (also when set later, e.g. by the app)
    this.heading = heading === false ? null : heading ?? new HeadingEstimator({ beamDeg });
    this.sweepLatencyMs = sweepLatencyMs; // sensor latency for sweeps (undefined: scan.js default; 0 in a simulator)
    this.sweepSample = sweepSample; // optional lighter sampler for sweeps and turns (distance + yaw)
    // turnMode 'gyro': closed-loop turn on yaw with drive frames (turnInPlace);
    // 'blocking': mbot2.turn. Scans happen on unknown cells ahead, blocked or
    // shortened legs, crashes, an uncertain pose, and after rescanCm of driving
    // (thin obstacles such as table legs are only seen when a beam hits them);
    // legsPerScan adds scans every n legs (off by default).
    this.turnMode = turnMode;
    this.rescanCm = rescanCm;
    this.taskScanCm = taskScanCm; // goTo/goHome scan first ('task-start') after moving this far since the last scan
    this.droveSinceScan = 0;
    this.spinSign = 1; // learned if clockwise wheel commands turn the robot counterclockwise
    this.turnSign = 1; // learned if mbot2.turn(+deg) turns counterclockwise (needs the gyro)
    this.legCoast = { cm: 0 }; // learned stop distance of drive legs (driveLeg aims this far short)
    this.sweepDegS = sweepDegS;
    Object.assign(this, { bus, map, pose, scanFn: scan, onEvent, steps, safetyCm, inflateCm, maxLegCm, legsPerScan, settleMs, useYaw, beamDeg, maxRangeCm });
    // legMode 'drive': driveLeg with sensor polling (default with a sampler);
    // 'straight': blocking gyro straight (default without one, because a
    // time-based leg estimate is worse than the robot's own straight())
    // localizer: a { matchScan, fusePose } object instead of ./localize.js (tests).
    // odomWeight: trust in odometry for routine map matches (the match moves the
    // pose by confidence * (1 - odomWeight)); after a crash it drops to 0.1,
    // for anchor matches to 0.
    // stopAtCm: in-leg ultrasonic stop; below safetyCm, which already shortens the leg.
    Object.assign(this, { legMode, legRpm, sample, motionOpts, localize, localizer, minMatchConfidence, odomWeight, stopAtCm });
    this.lastPath = null;
    this.goal = null;
    this.busy = false;
    this.poseUncertain = false;
    this.contacts = []; // crash contact points, for reference; the map owns their state
    this.anchors = [];  // [{ pose, points, map }]: anchors[0] is the reference scan at the start
    this.lastFix = null; // the last relocalize result, for the home check
    this.pendingScans = []; // [{ pose, points, reason, tries }]: rejected scans waiting for a retry
    this.unc = { ...UNC_MIN }; // pose sigma (cm, deg) since the last accepted match
    this.uncT = null;
    this._mapFrozen = false;
    this.frozenTasks = 0;
  }

  // Map frozen: scans only localize, nothing is written (also while goHome runs).
  get mapFrozen() { return this._mapFrozen; }
  set mapFrozen(v) { this.freeze(() => { this._mapFrozen = !!v; }); }
  get frozen() { return this._mapFrozen || this.frozenTasks > 0; }
  freeze(change) {
    const before = this.frozen;
    change();
    if (this.frozen !== before) this.emit({ type: 'map-frozen', frozen: this.frozen });
  }

  clockNow() { return this.sample?.clock?.now?.() ?? performance.now(); }

  // Heading sigma grows with time at the gyro drift rate.
  ageUnc() {
    const now = this.clockNow();
    if (this.uncT != null) this.unc.th += (DRIFT_DEG_MIN * Math.max(0, now - this.uncT)) / 60000;
    this.uncT = now;
  }

  // Odometry uncertainty of a leg (cm) or a turn (deg).
  growUnc({ cm = 0, deg = 0 } = {}) {
    this.ageUnc();
    if (deg) this.unc.th = Math.hypot(this.unc.th, TURN_FRAC * Math.abs(deg));
    if (cm) this.unc.xy = Math.hypot(this.unc.xy, SLIP_FRAC * Math.abs(cm), Math.abs(cm) * Math.sin((this.unc.th * Math.PI) / 180));
  }

  // Match window: 3 sigma, clamped; at least 60 cm / 30 deg after a crash.
  matchWindow() {
    this.ageUnc();
    const c = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
    let xy = c(3 * this.unc.xy, WIN_MIN.xy, WIN_MAX.xy), th = c(3 * this.unc.th, WIN_MIN.th, WIN_MAX.th);
    if (this.poseUncertain) { xy = Math.max(xy, 60); th = Math.max(th, 30); }
    return { xyWindowCm: Math.round(xy), angWindowDeg: Math.round(th) };
  }

  // Share of observed cells within a metre: below KNOWN_FRAC the robot explores.
  knownAround(p = this.pose.pose) {
    let known = 0, total = 0;
    for (let dy = -100; dy <= 100; dy += 20) {
      for (let dx = -100; dx <= 100; dx += 20) {
        if (dx * dx + dy * dy > 10000) continue;
        total++;
        if (!this.unknownAt(p.x + dx, p.y + dy)) known++;
      }
    }
    return known / total;
  }

  // Samplers pass through the heading estimator (corrected yaw in .yaw).
  get sample() { return this._sample; }
  set sample(v) { this._sample = this.wrapSampler(v); }
  get sweepSample() { return this._sweepSample; }
  set sweepSample(v) { this._sweepSample = this.wrapSampler(v); }
  wrapSampler(v) {
    if (typeof v !== 'function' || !this.heading || v.wrappedBy === this.heading) return v;
    const w = this.heading.wrap(v);
    w.wrappedBy = this.heading;
    return w;
  }

  // The estimator owns the heading while samples flow through it.
  get headingOwned() { return !!(this.heading && (this._sample || this._sweepSample)); }

  // Forget the anchor scans (call when the map is cleared or replaced, e.g. a
  // loaded map: anchors belong to the frame of the session that took them).
  resetAnchors() { this.anchors = []; }

  // Everything tied to the current map frame: anchors, pending scans, pose
  // sigma, and the heading estimator's room axes and bias (map cleared or loaded).
  resetLocalization() {
    this.anchors = [];
    this.pendingScans = [];
    this.unc = { ...UNC_MIN };
    this.heading?.reset();
  }

  // The first scan of a map becomes the reference anchor.
  addAnchor(pose, points, mapWasEmpty) {
    if (mapWasEmpty) this.anchors = [];
    if (this.anchors.length) return;
    const beams = (points ?? []).filter((p) => p.cm != null);
    if (beams.length >= 8) this.anchors.push({ pose: { ...pose }, points: beams, map: null });
  }

  nearestAnchor(p) {
    let best = null;
    for (const a of this.anchors) if (dist(a.pose, p) <= ANCHOR_CM && (!best || dist(a.pose, p) < dist(best.pose, p))) best = a;
    return best;
  }

  // A small map of the anchor scan alone, for scan-to-scan matching.
  anchorMap(a) {
    if (!a.map) {
      a.map = new GridMap({ cellCm: this.map.cellCm, sizeCm: this.map.sizeCm });
      a.map.integrateScan(a.pose, a.points, { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, robotRadiusCm: 0 });
    }
    return a.map;
  }

  emit(ev) { try { this.onEvent?.(ev); } catch { /* UI errors must not break navigation */ } }

  get selfPose() { return this.pose.bus !== this.bus; }

  // Runs one public task: one stamped makeCommand, abort -> stop + AbortError,
  // stop generation or unexpected errors -> { ok: false, note }.
  async task(name, signal, fn) {
    const mk = this.bus.stamped ? this.bus.stamped() : makeCommand;
    this.busy = true;
    try {
      return await fn(mk);
    } catch (e) {
      if (e?.name === 'AbortError') {
        await this.bus.stop('agent');
        throw e;
      }
      const note = e instanceof Cancelled ? 'cancelled by stop' : `${name} failed: ${e?.message ?? e}`;
      this.emit({ type: 'error', note });
      return { ok: false, reached: false, pose: this.pose.pose, note };
    } finally {
      this.busy = false;
    }
  }

  checkAbort(signal) { if (signal?.aborted) throw abortError(); }

  // Submits one command; rejects at once on abort. Failed commands come back
  // as results, except stop-generation cancellation which ends the task.
  async cmd(mk, signal, cmd, args) {
    this.checkAbort(signal);
    const p = this.bus.submit(mk(cmd, args, 'agent'));
    let r;
    if (!signal) r = await p;
    else {
      r = await new Promise((resolve, reject) => {
        const onAbort = () => reject(abortError());
        signal.addEventListener('abort', onAbort, { once: true });
        p.then(resolve, reject).finally(() => signal.removeEventListener('abort', onAbort));
      });
    }
    if (!r.ok && /cancelled by stop/.test(r.error ?? '')) throw new Cancelled(r.error);
    return r;
  }

  async scanHere({ signal } = {}) {
    return this.task('scan', signal, async (mk) => { await this.ensureYawRef(mk, signal); return this.doScan(mk, signal, 'request'); });
  }

  // Integrates each point as it arrives so the map view updates live.
  async doScan(mk, signal, reason = 'request') {
    // the scan pose is matched against the map: take the heading from the gyro first
    await this.correctYaw(mk, signal);
    if (this.scanMode === 'sweep' && (this.sweepSample ?? this.sample)) return this.doSweep(mk, signal, reason);
    let at = this.pose.pose;
    const empty = !this.mapKnown();
    const opts = { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: Math.max(this.beamDeg, 360 / this.steps) };
    // with a known map, match first and integrate at the corrected pose
    const loc = this.localize && this.mapKnown() ? (this.localizer ?? await loadLocalizer()) : null;
    const onPoint = loc || this.frozen ? null : (p) => this.map.integrateScan(at, [p], opts);
    let res;
    // one map scan for all points: confirmation needs hits from two scans
    this.map.beginScan?.();
    try {
      try {
        res = await this.scanFn(this.bus, { steps: this.steps, signal, makeCommand: mk, onPoint, settleMs: this.settleMs });
      } catch (e) {
        if (e?.name === 'AbortError') throw e;
        if (/cancelled by stop/.test(e?.message ?? '')) throw new Cancelled(e.message);
        throw e;
      }
      if (loc) {
        const r = this.relocalize(loc, at, res.points, { minValid: MIN_VALID_STEP });
        at = r.pose;
        if (this.gate(r, at, res.points, reason)) this.map.integrateScan(at, res.points, opts);
      }
    } finally {
      this.map.endScan?.();
    }
    if (!this.frozen) this.addAnchor(at, res.points, empty);
    // scan() turns a full circle; with steps that do not divide 360 the bus
    // rounds each turn, so apply the residue when the tracker is not attached
    if (this.selfPose) this.pose.applyTurn(Math.round(360 / this.steps) * this.steps - 360);
    await this.correctYaw(mk, signal);
    this.scannedAt = at;
    this.droveSinceScan = 0;
    this.emit({ type: 'scan', reason, pose: at, points: res.points, method: 'step' });
    return { ok: true, points: res.points };
  }

  // Continuous rotation scan: dense points relative to the heading at the start.
  // The sweep uses drive frames, which no pose listener counts, so its net
  // rotation (turnedDeg) is applied here in every wiring.
  async doSweep(mk, signal, reason = 'request') {
    let at = this.pose.pose;
    const empty = !this.mapKnown();
    const loc = this.localize && !empty ? (this.localizer ?? await loadLocalizer()) : null;
    let res;
    const hm = this.heading?.mark();
    try {
      res = await sweepScan(this.bus, { sample: this.sweepSample ?? this.sample, makeCommand: mk, signal, speedDegS: this.sweepDegS,
        ...(this.sweepLatencyMs != null ? { latencyMs: this.sweepLatencyMs } : {}) });
    } catch (e) {
      // a bus stop surfaces as AbortError from sweepScan; only the signal is a real abort
      if (e?.name === 'AbortError' && !signal?.aborted) throw new Cancelled(e.message);
      if (e?.name === 'AbortError') throw e;
      if (/cancelled by stop/.test(e?.message ?? '')) throw new Cancelled(e.message);
      throw e;
    }
    const mapPoints = resampleSweep(res.points, 5);
    // net rotation of the sweep (gyrodometry), then the wall snap against the
    // room axes, before matching so the match starts from the corrected heading
    const seg = hm ? this.heading.segmentSince(hm, 'sweep') : null;
    const turned = normDeg(Number.isFinite(seg?.deg) ? seg.deg : res.turnedDeg ?? 0);
    if (hm) {
      // the reference scan sets the room axes; later sweeps only snap here
      const snap = this.heading.sweep(res.points, at.heading, { trusted: empty });
      this.emit({ ...snap, wallList: undefined });
      if (snap.applied && Number.isFinite(snap.correctionDeg)) {
        this.setPose({ ...this.pose.pose, heading: normDeg(this.pose.pose.heading + snap.correctionDeg) });
        at = this.pose.pose;
        this.unc.th = Math.max(UNC_MIN.th, Math.min(this.unc.th, SNAP_SIGMA_DEG));
      }
    }
    let write = !this.frozen;
    if (loc) {
      const r = this.relocalize(loc, at, resampleSweep(res.points, MATCH_BIN_DEG), { minValid: MIN_VALID_SWEEP });
      at = r.pose;
      write = this.gate(r, at, mapPoints, reason);
      // a sweep fixed against an anchor has a trustworthy heading: it may
      // set the room axes while they are not set yet
      const fix = this.lastFix;
      if (hm && this.heading.axisDeg == null && fix?.applied && fix.ref === 'anchor' && (fix.confidence ?? 0) >= 0.5) {
        this.emit({ ...this.heading.sweep(res.points, at.heading, { trusted: true }), wallList: undefined });
      }
    }
    if (write) this.map.integrateScan(at, mapPoints, { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: this.beamDeg });
    if (!this.frozen) this.addAnchor(at, mapPoints, empty);
    this.pose.applyTurn(turned); // relocalize() already moved the estimate if it matched
    await this.correctYaw(mk, signal);
    this.scannedAt = at;
    this.droveSinceScan = 0;
    this.emit({ type: 'scan', reason, pose: at, points: mapPoints, method: 'sweep', samples: res.samples, turnedDeg: turned });
    return { ok: true, points: mapPoints, method: 'sweep' };
  }

  // Movement since the last scan: the larger of the leg path driven by this
  // navigator and the straight-line distance from the scan pose (which also
  // covers joystick or button moves the navigator did not drive). Infinity
  // if there was never a scan.
  movedSinceScan() {
    if (!this.scannedAt) return Infinity;
    return Math.max(this.droveSinceScan ?? 0, dist(this.scannedAt, this.pose.pose));
  }

  mapKnown() { return (this.map.stats?.().knownM2 ?? 0) > 0.3; }

  // Decides whether a matched scan may be written into the map (and retries
  // pending scans after an accepted one). Returns true to integrate at pose.
  gate(r, pose, points, reason) {
    if (r.outcome === 'accepted') {
      if (this.frozen) return false;
      this.retryPending();
      return true;
    }
    // too little mapped structure to match: fine to map at the odometry pose
    // while exploring new ground, never next to known structure
    if (r.outcome === 'uninformative' && this.knownAround(pose) < KNOWN_FRAC) return !this.frozen;
    if (!this.frozen) {
      this.pendingScans.push({ pose: { ...pose }, points, reason, tries: 0, unc: { ...this.unc } });
      if (this.pendingScans.length > PENDING_MAX) this.pendingScans.shift();
    }
    this.emit({ type: 'scan-pending', reason: r.why ?? r.outcome, pose, pending: this.pendingScans.length, frozen: this.frozen });
    return false;
  }

  // Pending scans are matched again (wider window) against the map that now
  // holds a newly accepted scan; accepted ones are written at their corrected pose.
  retryPending() {
    const loc = this.localizer ?? localizer;
    if (!this.pendingScans.length || !loc?.matchScan) return;
    const keep = [];
    for (const p of this.pendingScans) {
      const win = { xyWindowCm: Math.min(WIN_MAX.xy, Math.max(WIN_MIN.xy, 4.5 * p.unc.xy)), angWindowDeg: Math.min(WIN_MAX.th, Math.max(WIN_MIN.th, 4.5 * p.unc.th)) };
      let m = null;
      try { m = loc.matchScan(this.map, p.pose, p.points, { ...win, maxRangeCm: this.maxRangeCm, minValid: MIN_VALID_STEP }); } catch { m = null; }
      const ok = m?.pose && (m.axes ? m.axes.heading && m.axes.cross : (m.confidence ?? 0) >= 0.5);
      if (ok) {
        this.map.integrateScan(m.pose, p.points, { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: this.beamDeg });
        this.emit({ type: 'scan-accepted', pending: true, correction: { dx: m.pose.x - p.pose.x, dy: m.pose.y - p.pose.y, dh: normDeg(m.pose.heading - p.pose.heading) }, confidence: m.confidence, cov: m.cov ?? null });
      } else if (++p.tries < PENDING_TRIES) keep.push(p);
    }
    this.pendingScans = keep;
  }

  // Scan matching around the odometry pose, window from the pose uncertainty.
  // Near an anchor, scan to scan against it first (it carries no drift);
  // otherwise against the map. A match with a covariance (js/localize.js)
  // corrects each well-determined component Kalman-style; a plain
  // { pose, confidence } (test localizers) goes through fusePose.
  // Returns { outcome: 'accepted' | 'rejected' | 'uninformative', pose, why }.
  relocalize(loc, guess, points, { minValid = MIN_VALID_STEP } = {}) {
    const beams = (points ?? []).filter((p) => p.cm != null && p.cm > 0 && p.cm < Math.min(NO_ECHO_CM, this.maxRangeCm));
    this.lastFix = null;
    const out = (outcome, why, pose = this.pose.pose) => ({ outcome, why, pose });
    if (beams.length < Math.min(minValid, 4)) return out('uninformative', 'few readings', guess);
    try {
      const wide = this.poseUncertain;
      const win = this.matchWindow();
      const opts = { ...win, maxRangeCm: this.maxRangeCm, minValid, priorCm: Math.max(5, this.unc.xy), priorDeg: Math.max(2, this.unc.th) };
      const anchor = this.nearestAnchor(guess);
      let m = null, ref = 'map';
      if (anchor) {
        m = loc.matchScan(this.anchorMap(anchor), guess, beams, { ...opts, xyWindowCm: Math.max(40, win.xyWindowCm), angWindowDeg: Math.max(20, win.angWindowDeg) });
        if (m?.pose && (m.axes ? m.axes.heading && m.axes.cross : (m.confidence ?? 0) >= ANCHOR_MIN_CONF)) ref = 'anchor';
        else m = null;
      }
      if (!m) m = loc.matchScan(this.map, guess, beams, opts);
      if (!m?.pose) return out('rejected', 'no match', guess);
      // the anchor scan carries no drift: trust it over odometry (sigma floor)
      if (ref === 'anchor') this.unc = { xy: Math.max(this.unc.xy, ANCHOR_UNC.xy), th: Math.max(this.unc.th, ANCHOR_UNC.th) };
      const fused = m.cov ? this.fuseCov(guess, m, ref) : this.fusePlain(loc, guess, m, wide);
      const { pose: p, applied } = fused;
      const correction = { dx: p.x - guess.x, dy: p.y - guess.y, dh: normDeg(p.heading - guess.heading) };
      if (applied) {
        this.setPose(p);
        this.poseUncertain = false;
        if (ref === 'map') this.lastMapFixAt = { x: p.x, y: p.y };
      }
      // written only where the fused pose agrees with the scan's own best fit
      // (along an open corridor axis only the cross-track part counts)
      let off = Math.hypot(p.x - m.pose.x, p.y - m.pose.y);
      if (m.cov && !m.axes?.along) {
        const t = (m.cov.axisDeg * Math.PI) / 180;
        off = Math.abs((p.x - m.pose.x) * Math.cos(t) - (p.y - m.pose.y) * Math.sin(t));
      }
      const agrees = !m.cov || (off <= WRITE_TOL.cm && Math.abs(normDeg(p.heading - m.pose.heading)) <= WRITE_TOL.deg);
      this.lastFix = { applied, confidence: m.confidence ?? 0, ref, correction, cov: m.cov ?? null };
      this.emit({ type: 'localized', correction, confidence: m.confidence, applied, source: applied ? 'scan' : 'odom', ref, reason: fused.why ?? null, cov: m.cov ?? null, axes: fused.axes ?? null, window: win, guess, match: m.pose, pose: this.pose.pose });
      if (applied) this.emit({ type: 'scan-accepted', correction, confidence: m.confidence ?? 0, cov: m.cov ?? null, axes: fused.axes ?? null, ref });
      if (applied && !agrees) return out('rejected', 'pose corrected, but not yet where the scan fits', this.pose.pose);
      return applied ? out('accepted', null) : out(fused.uninformative ? 'uninformative' : 'rejected', fused.why, guess);
    } catch (e) {
      this.emit({ type: 'localized', error: e?.message ?? String(e), confidence: 0, applied: false, source: 'odom' });
      return out('rejected', e?.message ?? String(e), guess);
    }
  }

  // Kalman-style fusion of a match with covariance: heading if determined,
  // position along the determined axes (cross-track only when along-track is
  // open, as at a door or in a corridor). Updates the pose uncertainty.
  fuseCov(g, m, ref = 'map') {
    const c = m.cov, axes = m.axes ?? {};
    if (m.reason) return { pose: g, applied: false, uninformative: true, why: m.reason, axes };
    if (!axes.heading && !axes.cross) return { pose: g, applied: false, why: `ambiguous (sigma ${c.major}/${c.minor} cm, ${c.sigmaDeg} deg)`, axes };
    // innovation check: a determined match far outside the combined sigma
    // means odometry drifted more than its sigma claims (the field case);
    // raise the odometry sigma to the disagreement instead of averaging
    // (position only: a wrong map in the heading would feed itself, see below)
    const r0 = Math.hypot(m.pose.x - g.x, m.pose.y - g.y);
    if (axes.cross && r0 > 2 * Math.hypot(this.unc.xy, c.minor)) this.unc.xy = r0;
    // Heading: the gyro pipeline (heading.js) and anchor scans own heading
    // drift. A map built during that drift agrees with it, and scans written
    // there at a matched heading pull the map along (field-like divergence in
    // the sim: 11, 18, 22 deg). So map matches move the heading by a Kalman
    // step capped at MAP_HEAD_STEP, and only after the robot moved since the
    // last accepted map match (a rescan on the spot brings no new evidence).
    // With the heading estimator active (gyrodometry and wall snaps), map
    // matches do not touch the heading at all.
    let heading = g.heading;
    const moved = !this.lastMapFixAt || dist(this.lastMapFixAt, g) >= MAP_HEAD_MOVE_CM;
    if (axes.heading && (ref === 'anchor' || (moved && !this.headingOwned))) {
      const k = this.unc.th ** 2 / (this.unc.th ** 2 + c.sigmaDeg ** 2);
      let dh = k * normDeg(m.pose.heading - g.heading);
      if (ref !== 'anchor') dh = Math.max(-MAP_HEAD_STEP, Math.min(MAP_HEAD_STEP, dh));
      heading = normDeg(g.heading + dh);
      this.unc.th = Math.max(UNC_MIN.th, Math.sqrt((1 - k) * this.unc.th ** 2));
    }
    let x = g.x, y = g.y;
    if (axes.cross) {
      // match covariance; an undetermined major axis gets a huge variance
      const t = (c.axisDeg * Math.PI) / 180, ex = Math.sin(t), ey = Math.cos(t);  // major axis
      const vMaj = axes.along ? c.major ** 2 : 1e6, vMin = c.minor ** 2;
      const mxx = vMaj * ex * ex + vMin * ey * ey, myy = vMaj * ey * ey + vMin * ex * ex, mxy = (vMaj - vMin) * ex * ey;
      const o = this.unc.xy ** 2;
      // K = O (O + M)^-1 with O = o I
      const a = o + mxx, b = mxy, d = o + myy, det = a * d - b * b;
      const kxx = (o * d) / det, kxy = (-o * b) / det, kyy = (o * a) / det;
      const rx = m.pose.x - g.x, ry = m.pose.y - g.y;
      x = g.x + kxx * rx + kxy * ry;
      y = g.y + kxy * rx + kyy * ry;
      // posterior (I - K) O: keep the larger of its diagonal
      this.unc.xy = Math.max(UNC_MIN.xy, Math.sqrt(Math.max((1 - kxx) * o, (1 - kyy) * o)));
    }
    return { pose: { x, y, heading }, applied: true, axes };
  }

  // A match without covariance (test localizers): fusePose with odomWeight,
  // routine corrections capped at MAX_FIX_CM / MAX_FIX_DEG.
  fusePlain(loc, g, m, wide) {
    const minConfidence = this.minMatchConfidence;
    const yawDeg = this.useYaw && this.pose.yawRef != null ? g.heading : undefined;
    const fused = loc.fusePose
      ? loc.fusePose(g, m, { yawDeg, odomWeight: wide ? 0.1 : this.odomWeight, ...(minConfidence != null ? { minConfidence } : {}) })
      : { pose: m.pose, source: (m.confidence ?? 0) >= (minConfidence ?? 0.5) ? 'scan' : 'odom' };
    const p = fused?.pose;
    if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return { pose: g, applied: false, why: 'no pose' };
    const pose = { x: p.x, y: p.y, heading: yawDeg ?? p.heading ?? g.heading };
    let applied = fused.source !== 'odom', why = fused.reason ?? (applied ? null : 'rejected by fusePose');
    const dxy = Math.hypot(pose.x - g.x, pose.y - g.y), dh = normDeg(pose.heading - g.heading);
    if (applied && !wide && dxy > MAX_FIX_CM) { applied = false; why = `position jump ${Math.round(dxy)} cm`; }
    if (applied && !wide && Math.abs(dh) > MAX_FIX_DEG) { applied = false; why = `heading jump ${Math.round(dh)} deg`; }
    if (applied) this.unc = { ...UNC_MIN };
    return { pose: applied ? pose : g, applied, why };
  }

  // Moves the estimate without resetting the trail; keeps later gyro
  // corrections consistent with the new heading.
  setPose({ x, y, heading }) {
    const t = this.pose;
    const dh = normDeg(heading - t.heading);
    t.x = x; t.y = y; t.heading = normDeg(heading);
    if (t.yawRef != null) t.yawRef = normDeg(t.yawRef - dh);
    t.mark?.();
  }

  // Marks a crash contact in front of the robot (after any back-off), once
  // per crash. With map.markContact (gridmap contact layer) the cells are
  // flagged, not given hit evidence, and the map expires them; re-marking is
  // idempotent. Without it, each point gets one confirming hit and nothing
  // re-adds it later (re-adding before every plan made a false crash a
  // permanent obstacle in the field).
  addContact(aheadCm) {
    const { x, y, heading } = this.pose.pose;
    const h = (heading * Math.PI) / 180, fx = Math.sin(h), fy = Math.cos(h);
    const step = this.map.cellCm / 2;
    const m = this.map;
    const points = [];
    for (let l = -CONTACT_HALF_CM; l <= CONTACT_HALF_CM + 1e-9; l += step) {
      points.push({ x: x + fx * aheadCm + fy * l, y: y + fy * aheadCm - fx * l });
    }
    if (typeof m.markContact === 'function') {
      for (const c of points) m.markContact(c.x, c.y);
    } else if (typeof m.add === 'function' && typeof m.index === 'function') {
      const seen = new Set();
      for (const c of points) {
        const k = m.index(c.x, c.y);
        if (k >= 0 && !seen.has(k)) { seen.add(k); m.add(k, CONTACT_L); }
      }
      m.touch?.();
    }
    this.contacts.push(...points);
    return points;
  }

  // Forward reading at the current heading, integrated into the map.
  async readAhead(mk, signal) {
    const r = await this.cmd(mk, signal, 'read', { sensor: 'distance' });
    const cm = r.ok && Number.isFinite(Number(r.value)) ? Number(r.value) : null;
    if (cm != null && this.mayWriteSingle()) this.map.integrateScan(this.pose.pose, [{ angle: 0, cm }], { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm });
    return cm;
  }

  // Single readings (before and after legs) are not matched: they go into the
  // map only while the pose is well known and the map is not frozen (without
  // localization there is nothing to wait for).
  mayWriteSingle() { return !this.frozen && (!this.localize || (this.unc.xy <= SINGLE_MAX.xy && this.unc.th <= SINGLE_MAX.th)); }

  // Raw gyro reads would undo the estimator's bias correction: while it owns
  // the heading this only runs when forced (blocking turns measure with it),
  // and then reads the corrected yaw from the sampler.
  async correctYaw(mk, signal, { force = false } = {}) {
    if (!this.useYaw) return;
    if (this.headingOwned) {
      if (!force) return;
      const smp = this._sweepSample ?? this._sample;
      const v = Number((await smp())?.yaw);
      if (Number.isFinite(v)) this.pose.correctHeading(v);
      return;
    }
    const r = await this.cmd(mk, signal, 'read', { sensor: 'yaw' });
    if (r.ok) this.pose.correctHeading(Number(r.value));
  }

  // The gyro reference must be taken while the heading estimate is still
  // right, i.e. before the first motion of a task. Taken lazily after a turn
  // it would bake that turn's error into every later yaw correction.
  async ensureYawRef(mk, signal) {
    if (this.useYaw && this.pose.yawRef == null) await this.correctYaw(mk, signal, { force: true });
  }

  // Returns { ok, crash?, result }. Gyro mode turns on yaw feedback and falls
  // back to the blocking turn when no yaw or encoder is available.
  async turn(mk, signal, deg) {
    deg = Math.round(normDeg(deg));
    if (!deg) return { ok: true };
    const sample = this.sweepSample ?? this.sample;
    if (this.turnMode === 'gyro' && sample) {
      this.checkAbort(signal);
      const hm = this.heading?.mark();
      const r = await turnInPlace(this.bus, { deg, sample, makeCommand: mk, signal, spinSign: this.spinSign, opts: this.turnOpts });
      if (r.reason !== 'nosensor') {
        if (r.reversed) this.emit({ type: 'warning', note: 'spin direction reversed: wheel commands turned the robot the other way; corrected' });
        this.spinSign = r.spinSign;
        // drive frames are not seen by the tracker: apply the measured rotation
        // (gyrodometry from the heading estimator when there is one)
        const seg = hm ? this.heading.segmentSince(hm, 'turn') : null;
        const turned = Number.isFinite(seg?.deg) ? seg.deg : r.achievedDeg;
        this.pose.applyTurn(turned);
        this.growUnc({ deg: turned });
        if (seg?.slip) this.emit({ type: 'heading-slip', ...seg });
        this.emit({ type: 'turn', target: deg, achieved: r.achievedDeg, applied: Math.round(turned * 10) / 10, source: seg?.source ?? 'gyro', reason: r.reason, detail: r.detail, passes: r.passes, reversed: r.reversed, mode: 'gyro' });
        if (r.reason === 'aborted') {
          this.checkAbort(signal);
          throw new Cancelled(r.note ?? 'cancelled by stop');
        }
        if (r.reason === 'error') return { ok: false, result: r };
        await this.correctYaw(mk, signal);
        return { ok: true, crash: r.reason === 'crash' || r.reason === 'stall', result: r };
      }
    }
    // Blocking mbot2.turn. With the gyro on, the achieved rotation is measured;
    // if it went the other way (the field log: alternating -125/+104/-122
    // turns), the sign of later blocking turns is flipped.
    const gyro = this.useYaw && this.pose.yawRef != null;
    if (gyro) await this.correctYaw(mk, signal, { force: true });
    const h0 = this.pose.heading;
    const r = await this.cmd(mk, signal, 'turn', { deg: deg * this.turnSign, wait: true });
    if (!r.ok) return { ok: false };
    if (this.selfPose) this.pose.applyTurn(deg);
    this.growUnc({ deg });
    await this.correctYaw(mk, signal, { force: true });
    const achieved = gyro ? normDeg(this.pose.heading - h0) : deg;
    if (gyro && Math.abs(deg) >= 20 && Math.sign(achieved) === -Math.sign(deg) && Math.abs(achieved) > Math.abs(deg) / 2) {
      this.turnSign = -this.turnSign;
      this.emit({ type: 'warning', note: `turn(${deg}) rotated ${Math.round(achieved)} deg: blocking turns now use the opposite sign` });
    }
    this.emit({ type: 'turn', target: deg, achieved: Math.round(achieved * 10) / 10, reason: 'done', mode: 'blocking' });
    return { ok: true };
  }

  async straight(mk, signal, cm) {
    const r = await this.cmd(mk, signal, 'straight', { cm, wait: true });
    if (!r.ok) return false;
    if (this.selfPose) this.pose.applyStraight(cm);
    this.growUnc({ cm });
    await this.correctYaw(mk, signal);
    return true;
  }

  // Reverses cm with the gyro straight (the bus allows backward moves without
  // the obstacle guard). The pose follows through the tracker or here.
  async backOff(mk, signal, cm) {
    const before = this.pose.pose;
    const ok = await this.straight(mk, signal, -Math.abs(cm));
    this.emit({ type: 'backoff', cm: Math.abs(cm), ok, from: before, pose: this.pose.pose });
    return { ok };
  }

  // One straight leg. Returns the driveLeg result (reason, droveCm, ...).
  async leg(mk, signal, cm) {
    if (this.legMode === 'straight') {
      const ok = await this.straight(mk, signal, cm);
      return { ok, reason: ok ? 'done' : 'error', droveCm: ok ? cm : 0, note: ok ? undefined : 'straight failed' };
    }
    this.checkAbort(signal);
    const from = this.pose.pose;
    const hm = this.heading?.mark();
    const r = await driveLeg(this.bus, {
      cm, speed: this.legRpm, makeCommand: mk, signal, sample: this.sample, stopAtCm: this.stopAtCm, opts: this.motionOpts, coast: this.legCoast,
    });
    // the drive commands are not seen by the tracker: apply the measured leg here
    if (r.droveCm) {
      // gyrodometry from the heading estimator (encoders unless the gyro
      // disagrees by more than 2 deg); else the gyro's rotation when there is one
      const seg = hm ? this.heading.segmentSince(hm, 'leg') : null;
      const dh = Number.isFinite(seg?.deg) ? seg.deg : r.yawDelta ?? r.encHeading ?? 0;
      if (seg?.slip) this.emit({ type: 'heading-slip', ...seg });
      if (dh) this.pose.applyTurn(dh / 2);
      this.pose.applyStraight(r.droveCm);
      this.growUnc({ cm: r.droveCm });
      if (dh) this.pose.applyTurn(dh / 2);
    }
    if (r.slip) this.emit({ type: 'warning', kind: 'slip', note: `wheel slip: wheels disagree by ${Math.round(r.slip.encHeading)} deg, gyro ${Math.round(r.slip.yawDelta)} deg; not a crash`, slip: r.slip });
    if (r.reason === 'aborted') {
      this.checkAbort(signal);
      throw new Cancelled(r.note ?? 'cancelled by stop');
    }
    // The robot itself passed here without a crash: contacts on its track
    // were wrong (or the obstacle moved). Only this clears a contact; the
    // ultrasonic cannot, since it sees through low obstacles.
    if ((r.reason === 'done' || r.reason === 'obstacle') && r.droveCm > 1) {
      const n = this.map.clearContactsAlong?.([from, this.pose.pose], CONTACT_CLEAR_CM) ?? 0;
      if (n) this.emit({ type: 'contacts-cleared', count: n, from, to: this.pose.pose });
    }
    const last = r.samples?.at(-1);
    if (last?.distanceCm != null && this.mayWriteSingle()) {
      this.map.integrateScan(this.pose.pose, [{ angle: 0, cm: last.distanceCm }], { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm });
    }
    if (r.reason !== 'error') await this.correctYaw(mk, signal);
    return r;
  }

  // Enough known cells within 40 cm to plan the first leg?
  knowsSurroundings() {
    const { x, y } = this.pose.pose;
    let known = 0, total = 0;
    for (let dy = -40; dy <= 40; dy += 10) {
      for (let dx = -40; dx <= 40; dx += 10) {
        if (dx * dx + dy * dy > 1600) continue;
        total++;
        if (this.map.cell(x + dx, y + dy) !== 'unknown') known++;
      }
    }
    return known >= total * 0.5;
  }

  // True if the first maxLegCm of the path crosses unknown cells.
  unknownAhead(path) {
    let left = this.maxLegCm;
    for (let i = 1; i < path.length && left > 0; i++) {
      const a = path[i - 1], b = path[i], d = dist(a, b), seg = Math.min(d, left);
      for (let t = this.map.cellCm; t <= seg; t += this.map.cellCm) {
        if (this.unknownAt(a.x + ((b.x - a.x) * t) / d, a.y + ((b.y - a.y) * t) / d)) return true;
      }
      left -= d;
    }
    return false;
  }

  // Unknown in the sense of "never observed": suspect cells (weak hit
  // evidence, which map.cell() reports as unknown) count as evidence.
  unknownAt(x, y) { return this.map.kind ? this.map.kind(x, y) === 'unknown' : this.map.cell(x, y) === 'unknown'; }

  // How far the robot can drive from a toward b (at most cm) before unknown
  // cells beside its footprint (FOOTPRINT_CM each side) lie next to obstacle
  // evidence (EDGE_CM). That is an obstacle edge seen only at a grazing angle
  // (the explore near-miss along the chair); the ultrasonic only looks ahead,
  // so a leg must not run along it blind. Unknown cells in open space do not
  // count, which keeps extra scans rare.
  knownReach(a, b, cm) {
    const d = dist(a, b) || 1, ux = (b.x - a.x) / d, uy = (b.y - a.y) / d;
    const step = this.map.cellCm / 2;
    for (let t = step; t <= cm; t += step) {
      const x = a.x + ux * t, y = a.y + uy * t;
      for (const l of [-FOOTPRINT_CM, -FOOTPRINT_CM / 2, 0, FOOTPRINT_CM / 2, FOOTPRINT_CM]) {
        const px = x + uy * l, py = y - ux * l;
        if (this.unknownAt(px, py) && this.map.clearance(px, py, L_SUSPECT) <= EDGE_CM) return Math.max(0, t - step);
      }
    }
    return cm;
  }

  plan(goal) {
    const opts = { inflateCm: this.inflateCm, allowUnknown: true };
    const raw = planPath(this.map, this.pose.pose, goal, opts);
    return raw ? simplifyPath(raw, this.map, opts) : null;
  }

  async goTo(goal, { signal, tolCm = 10, maxLegs = 12 } = {}) {
    return this.task('goTo', signal, (mk) => this.doGoTo(mk, signal, goal, { tolCm, maxLegs }));
  }

  async doGoTo(mk, signal, goal, { tolCm = 10, maxLegs = 12 } = {}) {
    goal = { x: Number(goal.x), y: Number(goal.y) };
    if (!Number.isFinite(goal.x) || !Number.isFinite(goal.y)) return { ok: false, reached: false, pose: this.pose.pose, legs: 0, note: 'invalid goal' };
    this.goal = goal;
    // a scan just taken here (e.g. scanHere before goTo) counts for this task
    let legs = 0, sinceScan = 0, blocked = 0, backoffs = 0, crashes = 0, scannedHere = this.movedSinceScan() < 5;
    const done = (ok, reached, note) => {
      const r = { ok, reached, pose: this.pose.pose, legs, note };
      if (reached) this.emit({ type: 'arrived', ...r, goal });
      return r;
    };
    // Scans (and with them relocalization) only when they add something:
    // unknown cells ahead, a blocked or shortened leg, a crash, an uncertain
    // pose, or every legsPerScan legs if that is set.
    const rescan = async (reason) => { await this.doScan(mk, signal, reason); sinceScan = 0; scannedHere = true; };
    const crashed = async (where, r) => {
      crashes++;
      if (where === 'leg') this.addContact((r.backedCm ?? 0) + FRONT_CM);
      this.poseUncertain = true;
      this.unc = { xy: Math.max(this.unc.xy, 20), th: Math.max(this.unc.th, 10) };
      this.emit({ type: 'crash', reason: r.detail ?? r.reason, during: where, details: r.details ?? null, droveCm: r.droveCm, backedCm: r.backedCm, pose: this.pose.pose });
      if (crashes >= MAX_CRASHES) return false;
      this.emit({ type: 'replan', reason: 'crash' });
      await rescan('crash');
      return true;
    };
    await this.ensureYawRef(mk, signal);
    // one scan at the start of a task when the map around the robot is thin
    // or the robot moved since the last scan (thin obstacles such as table legs
    // are only seen when a beam happens to hit them)
    if (!this.knowsSurroundings() || this.movedSinceScan() > this.taskScanCm) await rescan('task-start');
    for (let attempt = 0; legs < maxLegs && attempt < maxLegs * 3; attempt++) {
      this.checkAbort(signal);
      if (this.poseUncertain && !scannedHere) { await rescan('pose uncertain'); continue; }
      const here = this.pose.pose;
      if (dist(here, goal) <= tolCm) return done(true, true, `arrived within ${Math.round(dist(here, goal))} cm`);
      const path = this.plan(goal);
      if (!path) {
        if (!scannedHere) { this.emit({ type: 'replan', reason: 'no path' }); await rescan('no path'); continue; }
        return done(false, false, 'no path to the goal');
      }
      this.lastPath = path;
      this.emit({ type: 'plan', path, goal });
      let route = path;
      if (pathLen(path) <= tolCm || path.length < 2) {
        const d = dist(here, goal);
        if (d <= tolCm) return done(true, true, 'arrived');
        // the path ends at the goal cell's centre, a little short of the goal: finish directly
        if (dist(path.at(-1), goal) <= this.map.cellCm) route = [here, goal];
        else {
          // the goal was snapped to the nearest reachable cell and we are there
          if (!scannedHere) { await rescan('goal looks blocked'); continue; }
          return done(false, false, `goal blocked; stopped ${Math.round(d)} cm away`);
        }
      }
      if (!scannedHere && this.unknownAhead(path)) {
        await rescan('unknown cells ahead');
        continue;
      }
      const m = pathToMoves(here, route, { maxSegCm: this.maxLegCm })[0];
      if (!m) return done(false, false, 'no move');
      const t = await this.turn(mk, signal, m.turnDeg);
      if (!t.ok) return done(false, false, 'turn failed');
      if (t.crash) {
        if (!(await crashed('turn', t.result))) return done(false, false, `gave up after ${crashes} collisions`);
        continue;
      }
      const reading = await this.readAhead(mk, signal);
      const room = reading == null ? 0 : reading >= NO_ECHO_CM ? Infinity : reading - this.safetyCm;
      // stop at the edge of the known area; past it, scan again from closer
      const reach = this.knownReach(here, m.to, m.cm);
      const blind = reach < m.cm;
      const cap = blind ? Math.max(reach - 3, scannedHere ? Math.min(MIN_BLIND_LEG_CM, m.cm) : 0) : m.cm;
      if (blind && cap <= 5 && !scannedHere) { await rescan('unseen obstacle edge beside the leg'); continue; }
      const cm = Math.floor(Math.min(m.cm, room, cap));
      if (cm <= 5) {
        blocked++;
        this.emit({ type: 'blocked', readingCm: reading, pose: this.pose.pose });
        // Replanning from the same spot after a scan here gives the same
        // blocked heading (field: 5 'blocked' in one second, then give up).
        // Back off first, scan from there, and give up only after that failed.
        if (scannedHere) {
          if (backoffs >= MAX_BACKOFFS) return done(false, false, `blocked: ${reading ?? '?'} cm ahead`);
          backoffs++;
          const b = await this.backOff(mk, signal, BACKOFF_CM);
          if (!b.ok) return done(false, false, `blocked: ${reading ?? '?'} cm ahead; back-off failed`);
          await rescan('blocked, backed off');
        } else {
          if (blocked > 4) return done(false, false, `blocked: ${reading ?? '?'} cm ahead`);
          await rescan('blocked');
        }
        this.emit({ type: 'replan', reason: 'blocked' });
        continue;
      }
      const shortened = cm < m.cm - 5;
      const lr = await this.leg(mk, signal, cm);
      if (lr.reason === 'error') return done(false, false, `straight failed: ${lr.note ?? ''}`.trim());
      legs++;
      sinceScan++;
      this.droveSinceScan += Math.abs(lr.droveCm ?? 0);
      scannedHere = false;
      this.emit({ type: 'leg', leg: legs, turnDeg: m.turnDeg, cm, droveCm: lr.droveCm, reason: lr.reason, detail: lr.detail ?? null,
        overshootCm: lr.overshootCm ?? null, coastCm: this.legCoast.cm,
        readingCm: reading, pose: this.pose.pose, samples: lr.samples ?? null });
      if (lr.reason === 'crash' || lr.reason === 'stall') {
        if (!(await crashed('leg', lr))) return done(false, false, `gave up after ${crashes} collisions`);
        continue;
      }
      if (lr.reason === 'obstacle') this.emit({ type: 'blocked', readingCm: lr.samples?.at(-1)?.distanceCm, pose: this.pose.pose });
      if (dist(this.pose.pose, goal) <= tolCm) continue;
      if (lr.reason === 'obstacle' || shortened) await rescan(lr.reason === 'obstacle' ? 'obstacle during leg' : blind ? 'unseen obstacle edge beside the leg' : 'leg shortened');
      else if (sinceScan >= this.legsPerScan) await rescan('periodic');
      else if (this.droveSinceScan >= this.rescanCm) await rescan('distance since last scan');
    }
    const d = dist(this.pose.pose, goal);
    return done(d <= tolCm, d <= tolCm, d <= tolCm ? 'arrived' : `gave up after ${legs} legs, ${Math.round(d)} cm from the goal`);
  }

  // Frontier exploration: drive to the nearest reachable frontier (by path
  // length), scan there, repeat until none are left or maxMoves is used.
  async explore({ signal, maxMoves = 6, minCells = 4 } = {}) {
    return this.task('explore', signal, async (mk) => {
      await this.ensureYawRef(mk, signal);
      if (!this.knowsSurroundings()) await this.doScan(mk, signal, 'unknown surroundings');
      const tried = [];
      let moves = 0;
      const candidates = () => this.map.frontiers({ minCells })
        .filter((f) => dist(f, this.pose.pose) > 30 && !tried.some((t) => dist(t, f) < 30));
      while (moves < maxMoves) {
        this.checkAbort(signal);
        const ranked = candidates().slice(0, 12)
          .map((f) => ({ f, path: this.plan(f) }))
          .filter((c) => c.path)
          .map((c) => ({ ...c, len: pathLen(c.path) }))
          .sort((a, b) => a.len - b.len);
        if (!ranked.length) break;
        const target = ranked[0].f;
        tried.push(target);
        this.emit({ type: 'plan', path: ranked[0].path, goal: target, frontier: true });
        const r = await this.doGoTo(mk, signal, target, { tolCm: 20, maxLegs: 4 });
        moves++;
        if (!r.reached && !r.ok && /straight failed|turn failed/.test(r.note ?? '')) {
          return { ok: false, moves, frontiersLeft: candidates().length, pose: this.pose.pose, note: r.note };
        }
        if (!this.scannedAt || dist(this.scannedAt, this.pose.pose) > 10) await this.doScan(mk, signal, 'frontier reached');
      }
      const left = candidates().length;
      return { ok: true, moves, frontiersLeft: left, pose: this.pose.pose, note: left ? `${left} frontiers left` : 'no frontiers left' };
    });
  }

  // The way home runs over mapped ground: the map is frozen (localization only).
  async goHome({ signal, tolCm = 10 } = {}) {
    return this.task('goHome', signal, async (mk) => {
      this.freeze(() => { this.frozenTasks++; });
      try {
        const r = await this.doGoTo(mk, signal, { x: 0, y: 0 }, { tolCm, maxLegs: 12 });
        if (!r.reached) return r;
        const check = await this.homeCheck(mk, signal);
        // face the start heading; one retry if the turn ended far off (field: -150 for -168)
        await this.turn(mk, signal, -this.pose.pose.heading);
        if (Math.abs(normDeg(this.pose.pose.heading)) > 5) await this.turn(mk, signal, -this.pose.pose.heading);
        return { ...r, pose: this.pose.pose, note: 'home', homeCheck: check };
      } finally {
        this.freeze(() => { this.frozenTasks--; });
      }
    });
  }

  // At the start: scan, match against the reference scan (via relocalize,
  // the start anchor is the nearest), and drive a short correction leg while
  // the residual exceeds HOME_TOL_CM, at most HOME_FIXES times. Emits
  // { type: 'home-check', attempt, residualCm, confidence, applied, ref }.
  async homeCheck(mk, signal) {
    const ref = this.anchors[0];
    if (!this.localize || !ref || Math.hypot(ref.pose.x, ref.pose.y) > 1) return null;
    let last = null;
    for (let attempt = 0; ; attempt++) {
      await this.doScan(mk, signal, 'home-check');
      const fix = this.lastFix;
      const residualCm = Math.round(dist(this.pose.pose, ref.pose) * 10) / 10;
      last = { type: 'home-check', attempt, residualCm, confidence: fix?.confidence ?? 0, applied: !!fix?.applied, ref: fix?.ref ?? null, pose: this.pose.pose };
      this.emit(last);
      // drive only on a trusted fix against the reference
      if (residualCm <= HOME_TOL_CM || attempt >= HOME_FIXES || !(fix?.applied && fix.ref === 'anchor')) break;
      const m = pathToMoves(this.pose.pose, [this.pose.pose, ref.pose], { maxSegCm: 100 })[0];
      if (!m) break;
      const t = await this.turn(mk, signal, m.turnDeg);
      if (!t.ok || t.crash) break;
      const lr = await this.leg(mk, signal, Math.max(1, Math.round(m.cm)));
      if (lr.reason !== 'done') break;
    }
    return last;
  }

  describe() {
    const p = this.pose.pose;
    return `Pose: x ${Math.round(p.x)} cm, y ${Math.round(p.y)} cm, heading ${Math.round(p.heading)}° (x right, y forward of the start, clockwise +). `
      + this.map.describe(p);
  }
}
