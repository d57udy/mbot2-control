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

import { makeCommand } from './bus.js';
import { planPath, simplifyPath, pathToMoves, DEFAULT_INFLATE_CM } from './planner.js';
import { driveLeg } from './motion.js';
import { sweepScan, resampleSweep } from './scan.js';

const NO_ECHO_CM = 300;
const FRONT_CM = 10;       // robot centre to front bumper
const CONTACT_HALF_CM = 8; // half width of the marked contact
const CONTACT_L = 4;       // log-odds added per contact cell (clamped by the map)
const MAX_CRASHES = 4;

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
    legsPerScan = 2, settleMs, useYaw = false, beamDeg = 16, maxRangeCm = 250,
    sampler, sample = sampler, legMode = sample ? 'drive' : 'straight', legRpm = 40, scanMode = sample ? 'sweep' : 'step', sweepDegS = 45, motionOpts, localize = true, localizer, minMatchConfidence, odomWeight = 0.7, stopAtCm = 15 }) {
    // sampler (alias sample): js/motion.js sampler for legs and sweeps; with one,
    // scans default to 'sweep' = continuous rotation, else 'step' = stop-and-measure
    this.scanMode = scanMode;
    this.sweepDegS = sweepDegS;
    Object.assign(this, { bus, map, pose, scanFn: scan, onEvent, steps, safetyCm, inflateCm, maxLegCm, legsPerScan, settleMs, useYaw, beamDeg, maxRangeCm });
    // legMode 'drive': driveLeg with sensor polling (default with a sampler);
    // 'straight': blocking gyro straight (default without one, because a
    // time-based leg estimate is worse than the robot's own straight())
    // localizer: a { matchScan, fusePose } object instead of ./localize.js (tests).
    // odomWeight: trust in odometry for routine scans; after a crash it drops to 0.1.
    // stopAtCm: in-leg ultrasonic stop; below safetyCm, which already shortens the leg.
    Object.assign(this, { legMode, legRpm, sample, motionOpts, localize, localizer, minMatchConfidence, odomWeight, stopAtCm });
    this.lastPath = null;
    this.goal = null;
    this.busy = false;
    this.poseUncertain = false;
    this.contacts = []; // map points where the robot hit something; kept occupied
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
    return this.task('scan', signal, (mk) => this.doScan(mk, signal));
  }

  // Integrates each point as it arrives so the map view updates live.
  async doScan(mk, signal) {
    if (this.scanMode === 'sweep' && this.sample) return this.doSweep(mk, signal);
    let at = this.pose.pose;
    const opts = { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: Math.max(this.beamDeg, 360 / this.steps) };
    // with a known map, match first and integrate at the corrected pose
    const loc = this.localize && this.mapKnown() ? (this.localizer ?? await loadLocalizer()) : null;
    const onPoint = loc ? null : (p) => this.map.integrateScan(at, [p], opts);
    let res;
    try {
      res = await this.scanFn(this.bus, { steps: this.steps, signal, makeCommand: mk, onPoint, settleMs: this.settleMs });
    } catch (e) {
      if (e?.name === 'AbortError') throw e;
      if (/cancelled by stop/.test(e?.message ?? '')) throw new Cancelled(e.message);
      throw e;
    }
    if (loc) {
      at = this.relocalize(loc, at, res.points) ?? at;
      this.map.integrateScan(at, res.points, opts);
    }
    this.markContacts();
    // scan() turns a full circle; with steps that do not divide 360 the bus
    // rounds each turn, so apply the residue when the tracker is not attached
    if (this.selfPose) this.pose.applyTurn(Math.round(360 / this.steps) * this.steps - 360);
    this.scannedAt = at;
    this.emit({ type: 'scan', pose: at, points: res.points });
    return { ok: true, points: res.points };
  }

  // Continuous rotation scan: dense points relative to the heading at the start.
  // The sweep uses drive frames, which no pose listener counts, so its net
  // rotation (turnedDeg) is applied here in every wiring.
  async doSweep(mk, signal) {
    let at = this.pose.pose;
    const loc = this.localize && this.mapKnown() ? (this.localizer ?? await loadLocalizer()) : null;
    let res;
    try {
      res = await sweepScan(this.bus, { sample: this.sample, makeCommand: mk, signal, speedDegS: this.sweepDegS });
    } catch (e) {
      // a bus stop surfaces as AbortError from sweepScan; only the signal is a real abort
      if (e?.name === 'AbortError' && !signal?.aborted) throw new Cancelled(e.message);
      if (e?.name === 'AbortError') throw e;
      if (/cancelled by stop/.test(e?.message ?? '')) throw new Cancelled(e.message);
      throw e;
    }
    const mapPoints = resampleSweep(res.points, 5);
    if (loc) at = this.relocalize(loc, at, resampleSweep(res.points, 10)) ?? at;
    this.map.integrateScan(at, mapPoints, { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: this.beamDeg });
    this.markContacts();
    this.pose.applyTurn(res.turnedDeg ?? 0); // relocalize() already moved the estimate if it matched
    this.scannedAt = at;
    this.emit({ type: 'scan', pose: at, points: mapPoints, method: 'sweep', samples: res.samples });
    return { ok: true, points: mapPoints, method: 'sweep' };
  }

  mapKnown() { return (this.map.stats?.().knownM2 ?? 0) > 0.3; }

  // Scan matching around the odometry pose (wider when uncertain), fused with
  // odometry. Returns the new pose, or null if nothing was applied.
  relocalize(loc, guess, points) {
    const beams = (points ?? []).filter((p) => p.cm != null && p.cm > 0 && p.cm < NO_ECHO_CM);
    if (beams.length < 4) return null;
    try {
      const wide = this.poseUncertain;
      const m = loc.matchScan(this.map, guess, beams, wide ? { xyWindowCm: 60, angWindowDeg: 30 } : {});
      if (!m?.pose) return null;
      const minConfidence = this.minMatchConfidence;
      const fused = loc.fusePose
        ? loc.fusePose(guess, m, { odomWeight: wide ? 0.1 : this.odomWeight, ...(minConfidence != null ? { minConfidence } : {}) })
        : { pose: m.pose, source: (m.confidence ?? 0) >= (minConfidence ?? 0.5) ? 'scan' : 'odom' };
      const p = fused?.pose;
      if (!p || !Number.isFinite(p.x) || !Number.isFinite(p.y)) return null;
      const correction = { dx: p.x - guess.x, dy: p.y - guess.y, dHeading: normDeg((p.heading ?? guess.heading) - guess.heading) };
      const used = fused.source !== 'odom';
      if (used) {
        this.setPose({ x: p.x, y: p.y, heading: p.heading ?? guess.heading });
        this.poseUncertain = false;
      }
      this.emit({ type: 'localized', correction, confidence: m.confidence, source: fused.source ?? (used ? 'scan' : 'odom'), pose: this.pose.pose });
      return used ? this.pose.pose : null;
    } catch (e) {
      this.emit({ type: 'localized', error: e?.message ?? String(e), confidence: 0, source: 'odom' });
      return null;
    }
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

  // Marks a contact in front of the robot (after any back-off) and remembers
  // it, because the ultrasonic may not see what was hit and free cones from
  // later scans would erase it.
  addContact(aheadCm) {
    const { x, y, heading } = this.pose.pose;
    const h = (heading * Math.PI) / 180, fx = Math.sin(h), fy = Math.cos(h);
    const step = this.map.cellCm / 2;
    for (let l = -CONTACT_HALF_CM; l <= CONTACT_HALF_CM + 1e-9; l += step) {
      this.contacts.push({ x: x + fx * aheadCm + fy * l, y: y + fy * aheadCm - fx * l });
    }
    this.markContacts();
  }

  markContacts() {
    if (!this.contacts.length) return;
    const m = this.map;
    if (typeof m.add === 'function' && typeof m.index === 'function') {
      for (const c of this.contacts) { const k = m.index(c.x, c.y); if (k >= 0) m.add(k, CONTACT_L); }
      m.touch?.();
    } else {
      // fallback: a narrow beam at each contact point
      for (const c of this.contacts) {
        const p = this.pose.pose;
        const ang = normDeg((Math.atan2(c.x - p.x, c.y - p.y) * 180) / Math.PI - p.heading);
        m.integrateScan(p, [{ angle: ang, cm: Math.hypot(c.x - p.x, c.y - p.y) }], { beamDeg: 4, sensorOffsetCm: 0, robotRadiusCm: 0 });
      }
    }
  }

  // Forward reading at the current heading, integrated into the map.
  async readAhead(mk, signal) {
    const r = await this.cmd(mk, signal, 'read', { sensor: 'distance' });
    const cm = r.ok && Number.isFinite(Number(r.value)) ? Number(r.value) : null;
    if (cm != null) this.map.integrateScan(this.pose.pose, [{ angle: 0, cm }], { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm });
    return cm;
  }

  async correctYaw(mk, signal) {
    if (!this.useYaw) return;
    const r = await this.cmd(mk, signal, 'read', { sensor: 'yaw' });
    if (r.ok) this.pose.correctHeading(Number(r.value));
  }

  async turn(mk, signal, deg) {
    deg = Math.round(normDeg(deg));
    if (!deg) return true;
    const r = await this.cmd(mk, signal, 'turn', { deg, wait: true });
    if (!r.ok) return false;
    if (this.selfPose) this.pose.applyTurn(deg);
    await this.correctYaw(mk, signal);
    return true;
  }

  async straight(mk, signal, cm) {
    const r = await this.cmd(mk, signal, 'straight', { cm, wait: true });
    if (!r.ok) return false;
    if (this.selfPose) this.pose.applyStraight(cm);
    await this.correctYaw(mk, signal);
    return true;
  }

  // One straight leg. Returns the driveLeg result (reason, droveCm, ...).
  async leg(mk, signal, cm) {
    if (this.legMode === 'straight') {
      const ok = await this.straight(mk, signal, cm);
      return { ok, reason: ok ? 'done' : 'error', droveCm: ok ? cm : 0, note: ok ? undefined : 'straight failed' };
    }
    this.checkAbort(signal);
    const r = await driveLeg(this.bus, {
      cm, speed: this.legRpm, makeCommand: mk, signal, sample: this.sample, stopAtCm: this.stopAtCm, opts: this.motionOpts,
    });
    // the drive commands are not seen by the tracker: apply the measured leg here
    if (r.droveCm) {
      const dh = r.encHeading ?? 0;
      if (dh) this.pose.applyTurn(dh / 2);
      this.pose.applyStraight(r.droveCm);
      if (dh) this.pose.applyTurn(dh / 2);
    }
    if (r.reason === 'aborted') {
      this.checkAbort(signal);
      throw new Cancelled(r.note ?? 'cancelled by stop');
    }
    const last = r.samples?.at(-1);
    if (last?.distanceCm != null) {
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
        if (this.map.cell(a.x + ((b.x - a.x) * t) / d, a.y + ((b.y - a.y) * t) / d) === 'unknown') return true;
      }
      left -= d;
    }
    return false;
  }

  plan(goal) {
    this.markContacts();
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
    let legs = 0, sinceScan = 0, blocked = 0, crashes = 0, scannedHere = false;
    const done = (ok, reached, note) => {
      const r = { ok, reached, pose: this.pose.pose, legs, note };
      if (reached) this.emit({ type: 'arrived', ...r, goal });
      return r;
    };
    const rescan = async () => { await this.doScan(mk, signal); sinceScan = 0; scannedHere = true; };
    if (!this.knowsSurroundings()) await rescan();
    for (let attempt = 0; legs < maxLegs && attempt < maxLegs * 3; attempt++) {
      this.checkAbort(signal);
      const here = this.pose.pose;
      if (dist(here, goal) <= tolCm) return done(true, true, `arrived within ${Math.round(dist(here, goal))} cm`);
      const path = this.plan(goal);
      if (!path) {
        if (!scannedHere) { this.emit({ type: 'replan', reason: 'no path' }); await rescan(); continue; }
        return done(false, false, 'no path to the goal');
      }
      this.lastPath = path;
      this.emit({ type: 'plan', path, goal });
      if (pathLen(path) <= tolCm || path.length < 2) {
        // the goal was snapped to the nearest reachable cell and we are there
        const d = dist(here, goal);
        return done(d <= tolCm, d <= tolCm, d <= tolCm ? 'arrived' : `goal blocked; stopped ${Math.round(d)} cm away`);
      }
      if (!scannedHere && this.unknownAhead(path)) {
        await rescan();
        continue;
      }
      const m = pathToMoves(here, path, { maxSegCm: this.maxLegCm })[0];
      if (!m) return done(false, false, 'no move');
      if (!(await this.turn(mk, signal, m.turnDeg))) return done(false, false, 'turn failed');
      const reading = await this.readAhead(mk, signal);
      const room = reading == null ? 0 : reading >= NO_ECHO_CM ? Infinity : reading - this.safetyCm;
      const cm = Math.floor(Math.min(m.cm, room));
      if (cm <= 5) {
        blocked++;
        this.emit({ type: 'blocked', readingCm: reading, pose: this.pose.pose });
        if (blocked > 4) return done(false, false, `blocked: ${reading ?? '?'} cm ahead`);
        if (!scannedHere) await rescan();
        this.emit({ type: 'replan', reason: 'blocked' });
        continue;
      }
      const lr = await this.leg(mk, signal, cm);
      if (lr.reason === 'error') return done(false, false, `straight failed: ${lr.note ?? ''}`.trim());
      legs++;
      sinceScan++;
      scannedHere = false;
      this.emit({ type: 'leg', leg: legs, turnDeg: m.turnDeg, cm, droveCm: lr.droveCm, reason: lr.reason, readingCm: reading, pose: this.pose.pose });
      if (lr.reason === 'crash' || lr.reason === 'stall') {
        crashes++;
        this.addContact((lr.backedCm ?? 0) + FRONT_CM);
        this.poseUncertain = true;
        this.emit({ type: 'crash', reason: lr.detail ?? lr.reason, droveCm: lr.droveCm, backedCm: lr.backedCm, pose: this.pose.pose });
        if (crashes >= MAX_CRASHES) return done(false, false, `gave up after ${crashes} collisions`);
        this.emit({ type: 'replan', reason: 'crash' });
        await rescan();
        continue;
      }
      if (lr.reason === 'obstacle') this.emit({ type: 'blocked', readingCm: lr.samples?.at(-1)?.distanceCm, pose: this.pose.pose });
      if (sinceScan >= this.legsPerScan && dist(this.pose.pose, goal) > tolCm) await rescan();
    }
    const d = dist(this.pose.pose, goal);
    return done(d <= tolCm, d <= tolCm, d <= tolCm ? 'arrived' : `gave up after ${legs} legs, ${Math.round(d)} cm from the goal`);
  }

  // Frontier exploration: drive to the nearest reachable frontier (by path
  // length), scan there, repeat until none are left or maxMoves is used.
  async explore({ signal, maxMoves = 6, minCells = 4 } = {}) {
    return this.task('explore', signal, async (mk) => {
      if (!this.knowsSurroundings()) await this.doScan(mk, signal);
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
        if (!this.scannedAt || dist(this.scannedAt, this.pose.pose) > 10) await this.doScan(mk, signal);
      }
      const left = candidates().length;
      return { ok: true, moves, frontiersLeft: left, pose: this.pose.pose, note: left ? `${left} frontiers left` : 'no frontiers left' };
    });
  }

  async goHome({ signal, tolCm = 10 } = {}) {
    return this.task('goHome', signal, async (mk) => {
      const r = await this.doGoTo(mk, signal, { x: 0, y: 0 }, { tolCm, maxLegs: 12 });
      if (!r.reached) return r;
      await this.turn(mk, signal, -this.pose.pose.heading);
      return { ...r, pose: this.pose.pose, note: 'home' };
    });
  }

  describe() {
    const p = this.pose.pose;
    return `Pose: x ${Math.round(p.x)} cm, y ${Math.round(p.y)} cm, heading ${Math.round(p.heading)}° (x right, y forward of the start, clockwise +). `
      + this.map.describe(p);
  }
}
