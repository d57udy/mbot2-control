// Polar plot of a scan: robot at the centre, forward = up, clockwise positive.
// Colours come from the page's CSS custom properties (light and dark mode).

export function drawRadar(canvas, points = [], { highlight = [], maxCm = 200 } = {}) {
  const ctx = canvas.getContext('2d');
  const css = getComputedStyle(document.documentElement);
  const col = (name, fallback) => css.getPropertyValue(name).trim() || fallback;
  const accent = col('--accent', '#2a7'), border = col('--border', '#888'), text = col('--text', '#222');
  const muted = col('--muted', '#888'), stopC = col('--stop', '#d33'), ok = col('--ok', '#2a2');
  const { width: w, height: h } = canvas;
  const cx = w / 2, cy = h / 2;
  const R = Math.min(w, h) / 2 - Math.max(6, Math.min(w, h) * 0.04);
  const k = R / maxCm;
  const font = Math.max(9, Math.round(Math.min(w, h) / 28));
  // canvas angle for a scan angle: 0 = up, clockwise positive
  const rad = (a) => ((a - 90) * Math.PI) / 180;

  ctx.clearRect(0, 0, w, h);

  // open sectors
  ctx.save();
  ctx.globalAlpha = 0.18;
  ctx.fillStyle = ok;
  for (const o of highlight) {
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    if (o.widthDeg >= 360) ctx.arc(cx, cy, R, 0, Math.PI * 2);
    else ctx.arc(cx, cy, R, rad(o.angle - o.widthDeg / 2), rad(o.angle + o.widthDeg / 2));
    ctx.closePath();
    ctx.fill();
  }
  ctx.restore();

  // range rings and labels
  ctx.strokeStyle = border;
  ctx.fillStyle = muted;
  ctx.lineWidth = 1;
  ctx.font = `${font}px system-ui`;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'bottom';
  for (let cm = 50; cm <= maxCm; cm += 50) {
    ctx.beginPath(); ctx.arc(cx, cy, cm * k, 0, Math.PI * 2); ctx.stroke();
    ctx.fillText(`${cm}`, cx + 3, cy - cm * k - 1);
  }
  // forward tick
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - R); ctx.setLineDash([3, 4]); ctx.stroke(); ctx.setLineDash([]);

  // filled outline of the measured free space
  const valid = points.filter((p) => p.cm != null);
  if (valid.length > 2) {
    ctx.save();
    ctx.globalAlpha = 0.12;
    ctx.fillStyle = accent;
    ctx.beginPath();
    [...valid].sort((a, b) => ((a.angle + 360) % 360) - ((b.angle + 360) % 360)).forEach((p, i) => {
      const r = Math.min(p.cm, maxCm) * k;
      ctx[i ? 'lineTo' : 'moveTo'](cx + Math.cos(rad(p.angle)) * r, cy + Math.sin(rad(p.angle)) * r);
    });
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // rays and echo dots; beyond-range readings end in an open marker at the rim
  const dot = Math.max(2.5, R / 40);
  ctx.lineWidth = Math.max(1, R / 120);
  for (const p of points) {
    const a = rad(p.angle);
    const ux = Math.cos(a), uy = Math.sin(a);
    if (p.cm == null) {
      ctx.fillStyle = muted;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('?', cx + ux * R * 0.5, cy + uy * R * 0.5);
      continue;
    }
    const far = p.cm > maxCm;
    const r = Math.min(p.cm, maxCm) * k;
    ctx.strokeStyle = accent;
    ctx.globalAlpha = 0.5;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + ux * r, cy + uy * r); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(cx + ux * r, cy + uy * r, dot, 0, Math.PI * 2);
    if (far) { ctx.strokeStyle = muted; ctx.stroke(); } else { ctx.fillStyle = p.cm < 30 ? stopC : accent; ctx.fill(); }
  }

  // robot
  const rb = Math.max(4, R / 14);
  ctx.fillStyle = text;
  ctx.beginPath();
  ctx.moveTo(cx, cy - rb * 1.4); ctx.lineTo(cx + rb, cy + rb); ctx.lineTo(cx - rb, cy + rb); ctx.closePath();
  ctx.fill();

  // best opening label
  if (highlight[0]) {
    const o = highlight[0];
    ctx.fillStyle = text;
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(`open ${o.angle > 0 ? '+' : ''}${o.angle}°`, 4, 4);
  }
}
