// Map-based navigation: scan into the grid map, plan with A*, drive in short
// gyro legs, re-measure before every straight and replan when blocked.
//
// Pose updates: if the PoseTracker is attached to this bus (pose.bus === bus)
// its listener applies every successful turn/straight and the Navigator does
// not; otherwise the Navigator applies its own moves. Either way each move is
// applied exactly once.

import { makeCommand } from './bus.js';
import { planPath, simplifyPath, pathToMoves } from './planner.js';

const NO_ECHO_CM = 300;

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
  constructor({ bus, map, pose, scan, onEvent, steps = 12, safetyCm = 20, inflateCm = 14, maxLegCm = 40,
    legsPerScan = 2, settleMs, useYaw = false, beamDeg = 16, maxRangeCm = 250 }) {
    Object.assign(this, { bus, map, pose, scanFn: scan, onEvent, steps, safetyCm, inflateCm, maxLegCm, legsPerScan, settleMs, useYaw, beamDeg, maxRangeCm });
    this.lastPath = null;
    this.goal = null;
    this.busy = false;
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
    const at = this.pose.pose;
    const opts = { beamDeg: this.beamDeg, maxRangeCm: this.maxRangeCm, freeBeamDeg: Math.max(this.beamDeg, 360 / this.steps) };
    const onPoint = (p) => this.map.integrateScan(at, [p], opts);
    let res;
    try {
      res = await this.scanFn(this.bus, { steps: this.steps, signal, makeCommand: mk, onPoint, settleMs: this.settleMs });
    } catch (e) {
      if (e?.name === 'AbortError') throw e;
      if (/cancelled by stop/.test(e?.message ?? '')) throw new Cancelled(e.message);
      throw e;
    }
    // scan() turns a full circle; with steps that do not divide 360 the bus
    // rounds each turn, so apply the residue when the tracker is not attached
    if (this.selfPose) this.pose.applyTurn(Math.round(360 / this.steps) * this.steps - 360);
    this.scannedAt = at;
    this.emit({ type: 'scan', pose: at, points: res.points });
    return { ok: true, points: res.points };
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
    let legs = 0, sinceScan = 0, blocked = 0, scannedHere = false;
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
      if (!(await this.straight(mk, signal, cm))) return done(false, false, 'straight failed');
      legs++;
      sinceScan++;
      scannedHere = false;
      this.emit({ type: 'leg', leg: legs, turnDeg: m.turnDeg, cm, readingCm: reading, pose: this.pose.pose });
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
