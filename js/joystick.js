// Virtual joystick: one finger, returns x/y in [-1, 1] (y up = +1).
// Calls onMove while held (on every pointer move) and onRelease when let go.

export class Joystick {
  constructor(el, { onStart, onMove, onRelease }) {
    this.el = el;
    this.knob = el.querySelector('.knob');
    this.onStart = onStart;
    this.onMove = onMove;
    this.onRelease = onRelease;
    this.pointerId = null;
    this.value = { x: 0, y: 0 };

    el.addEventListener('pointerdown', (e) => this.start(e));
    el.addEventListener('pointermove', (e) => this.move(e));
    el.addEventListener('pointerup', (e) => this.end(e));
    el.addEventListener('pointercancel', (e) => this.end(e));
    el.addEventListener('lostpointercapture', (e) => this.end(e));
    el.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  start(e) {
    if (this.pointerId !== null) return;
    e.preventDefault();
    this.pointerId = e.pointerId;
    try { this.el.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    this.el.classList.add('active');
    this.onStart?.();
    this.move(e);
  }

  move(e) {
    if (e.pointerId !== this.pointerId) return;
    const r = this.el.getBoundingClientRect();
    const radius = r.width / 2;
    let dx = (e.clientX - (r.left + radius)) / radius;
    let dy = (e.clientY - (r.top + radius)) / radius;
    const len = Math.hypot(dx, dy);
    if (len > 1) { dx /= len; dy /= len; }
    this.value = { x: dx, y: -dy };
    this.knob.style.transform = `translate(${dx * radius * 0.7}px, ${dy * radius * 0.7}px)`;
    this.onMove?.(this.value);
  }

  end(e) {
    if (e.pointerId !== this.pointerId) return;
    this.pointerId = null;
    this.value = { x: 0, y: 0 };
    this.knob.style.transform = '';
    this.el.classList.remove('active');
    this.onRelease?.();
  }

  // Forces release, e.g. when the page is hidden.
  reset() {
    if (this.pointerId === null) return;
    this.end({ pointerId: this.pointerId });
  }
}
