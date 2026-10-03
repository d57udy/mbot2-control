// Top-down drawing of the simulator room, obstacles and robot.
// Optional `rays`: { x, y, heading, points } draws the last scan from its pose.

import { SIM_ROBOT } from './robot-sim.js';

export function drawSim(canvas, state, room, obstacles = [], { rays } = {}) {
  const c = canvas;
  const s = state;
  const ctx = c.getContext('2d');
  const k = c.width / room.w;
  const css = getComputedStyle(document.documentElement);
  ctx.clearRect(0, 0, c.width, c.height);
  ctx.strokeStyle = css.getPropertyValue('--border');
  ctx.lineWidth = 4;
  ctx.strokeRect(2, 2, c.width - 4, c.height - 4);

  // furniture
  ctx.font = `${Math.max(10, Math.round(6 * k))}px system-ui`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  for (const o of obstacles) {
    ctx.fillStyle = css.getPropertyValue('--border');
    ctx.beginPath();
    if (o.kind === 'circle') ctx.arc(o.x * k, o.y * k, o.r * k, 0, Math.PI * 2);
    else ctx.rect(o.x * k, o.y * k, o.w * k, o.h * k);
    ctx.fill();
    if (o.label && (o.kind !== 'circle' || o.r >= 10)) {
      ctx.fillStyle = css.getPropertyValue('--muted');
      const [lx, ly] = o.kind === 'circle' ? [o.x, o.y] : [o.x + o.w / 2, o.y + o.h / 2];
      ctx.fillText(o.label, lx * k, ly * k);
    }
  }
  ctx.textAlign = 'left';
  ctx.textBaseline = 'alphabetic';

  // last scan, rays from the sensor position at scan time
  if (rays?.points?.length) {
    ctx.strokeStyle = css.getPropertyValue('--accent');
    ctx.fillStyle = css.getPropertyValue('--accent');
    ctx.lineWidth = 1.5;
    for (const p of rays.points) {
      if (p.cm == null) continue;
      const a = ((rays.heading + p.angle) * Math.PI) / 180;
      const ox = rays.x + Math.cos(a) * SIM_ROBOT.sensorCm, oy = rays.y + Math.sin(a) * SIM_ROBOT.sensorCm;
      const ex = ox + Math.cos(a) * p.cm, ey = oy + Math.sin(a) * p.cm;
      ctx.globalAlpha = 0.35;
      ctx.beginPath(); ctx.moveTo(ox * k, oy * k); ctx.lineTo(ex * k, ey * k); ctx.stroke();
      ctx.globalAlpha = 1;
      if (p.cm < 300) { ctx.beginPath(); ctx.arc(ex * k, ey * k, 3, 0, Math.PI * 2); ctx.fill(); }
    }
  }

  // robot, about 17 x 13 cm with the sensor and eyes at the front (+x)
  ctx.save();
  ctx.translate(s.x * k, s.y * k);
  ctx.rotate((s.heading * Math.PI) / 180);
  ctx.scale(k / 2, k / 2); // shapes below are in half-centimetres
  ctx.fillStyle = css.getPropertyValue('--accent');
  ctx.fillRect(-20, -13, 34, 26);
  // five back LEDs along the rear edge
  s.leds.forEach((rgb, i) => {
    ctx.fillStyle = `rgb(${rgb.join(',')})`;
    ctx.beginPath(); ctx.arc(-20, -10 + i * 5, 2.5, 0, Math.PI * 2); ctx.fill();
  });
  // eyes at the front
  s.eyes.forEach((bri, i) => {
    ctx.fillStyle = `rgba(80,160,255,${0.15 + (bri / 100) * 0.85})`;
    ctx.beginPath(); ctx.arc(14, i === 0 ? -6 : 6, 4, 0, Math.PI * 2); ctx.fill();
  });
  ctx.restore();
  if (s.label) {
    ctx.fillStyle = css.getPropertyValue('--text');
    ctx.font = '24px system-ui';
    ctx.fillText(s.label, 12, 32);
  }
}
