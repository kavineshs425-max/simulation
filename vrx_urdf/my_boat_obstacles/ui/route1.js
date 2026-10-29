'use strict';
// ====================================================================== PLANNER 1 map overlay
// Route planner 1's COMPILED map (route_one_planner / map_compiler): where the
// boat may NOT go right now and why (too shallow at this water level, structure,
// bridge too low, active no-go, closure, obstruction), and the canal graph it
// will route on (blue = passable, red dashed = blocked, with the reason).
// Water and land themselves are already on the map, so they stay transparent.
// Hover: the reason under the cursor, and the canal stretch's attributes.

const R1 = { info: null, img: null, raw: null, version: -1 };
const R1_REASON_TEXT = { uncharted: 'not charted as water', too_shallow: 'too shallow at the current water level',
  land: 'land / canal bank', obstruction: 'obstruction (not enough water over it)', structure: 'structure',
  low_bridge: 'bridge deck too low for the air draft', no_go: 'no-go area (in force now)', closure: 'closure' };

function r1On(id) { const el = document.getElementById(id); return !el || el.checked; }
function r1Sub(pts, s0, s1) {                  // the part of a polyline between arc lengths s0 and s1
  const out = []; let s = 0;
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1], L = Math.hypot(bx - ax, by - ay);
    const a = Math.max(s0, s), b = Math.min(s1, s + L);
    if (b > a) {
      const p = (t) => [ax + (bx - ax) * (t - s) / (L || 1), ay + (by - ay) * (t - s) / (L || 1)];
      if (!out.length) out.push(p(a));
      out.push(p(b));
    }
    s += L;
  }
  return out;
}
function r1CanalPts(name) {
  if (typeof ENV !== 'undefined' && ENV.draft) { const c = ENV.draft.canals.find((o) => o.name === name); if (c) return c.points; }
  if (typeof VM !== 'undefined' && VM.data && VM.data.features) { const f = VM.data.features.find((g) => g.class === 'CANALS' && g.properties.name === name); if (f) return f.geometry.coordinates; }
  return null;
}

function r1Draw() {
  const I = R1.info; if (!I || !I.ok || !r1On('showr1')) return;
  const g = I.grid;
  if (R1.img && R1.img.complete) {
    const [sx, sy] = w2s(g.x0, g.y0 + g.ny * g.res);
    ctx.save(); ctx.imageSmoothingEnabled = false;
    ctx.drawImage(R1.img, sx, sy, g.nx * g.res * view.scale, g.ny * g.res * view.scale); ctx.restore();
  }
  if (!r1On('r1graph') || !I.graph) return;
  I.graph.edges.forEach((e) => {
    const pts = r1CanalPts(e.canal); if (!pts) return;
    const seg = r1Sub(pts, e.s0, e.s1); if (seg.length < 2) return;
    path(seg); ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.lineWidth = Math.max(3, 2.2 * Math.min(view.scale, 3)); ctx.setLineDash(e.passable ? [] : [10, 7]);
    ctx.strokeStyle = e.passable ? 'rgba(21,101,192,.55)' : 'rgba(198,40,40,.9)'; ctx.stroke(); ctx.setLineDash([]); ctx.lineCap = 'butt';
    if (!e.passable) { const m = seg[Math.floor(seg.length / 2)]; vmChip(`BLOCKED: ${(e.blocked_by || '?').replace('_', ' ')}`, m[0], m[1], '#c62828'); }
  });
  const col = { junction: '#fbc02d', mouth: '#2e7d32', dead_end: '#6d4c41' };
  I.graph.nodes.forEach((n) => {
    const [sx, sy] = w2s(n.x, n.y);
    ctx.beginPath(); ctx.arc(sx, sy, 6.5, 0, 2 * Math.PI); ctx.fillStyle = col[n.kind] || '#999'; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke();
  });
}

// hover text for the vector-map box (vmap.js calls this)
function r1Explain(wx, wy) {
  const I = R1.info; if (!I || !I.ok || !r1On('showr1')) return '';
  let html = '';
  const g = I.grid, i = Math.floor((wy - g.y0) / g.res), j = Math.floor((wx - g.x0) / g.res);
  if (R1.raw && i >= 0 && j >= 0 && i < g.ny && j < g.nx) {
    const code = R1.raw[i * g.nx + j], name = (I.summary && I.summary.reasons) ? I.summary.reasons[code] : String(code);
    if (name !== 'free' && name !== 'uncharted' && name !== 'land') html += `<div class="vmbad">Planner 1: NOT ALLOWED here -- ${R1_REASON_TEXT[name] || name}</div>`;
    else if (name === 'free') html += `<div class="vmok">Planner 1: allowed</div>`;
  }
  if (r1On('r1graph') && I.graph) {
    let best = null;
    I.graph.edges.forEach((e) => {
      const pts = r1CanalPts(e.canal); if (!pts) return;
      const seg = r1Sub(pts, e.s0, e.s1); if (seg.length < 2) return;
      const d = polyDist(wx, wy, seg); if (d <= e.width / 2 && (!best || d < best.d)) best = { e, d };
    });
    if (best) {
      const e = best.e;
      html += `<div class="vmh" style="margin-top:6px"><b>Planner 1 canal stretch</b></div><div class="vmn">${e.id}</div>` +
        (e.passable ? '<div class="vmok">passable</div>' : `<div class="vmbad">BLOCKED: ${R1_REASON_TEXT[e.blocked_by] || e.blocked_by}</div>`) +
        `<table><tr><td>length</td><td>${e.length.toFixed(0)} m</td></tr><tr><td>time</td><td>${e.time_s.toFixed(0)} s</td></tr>` +
        `<tr><td>best depth across</td><td>${e.depth_min == null ? '?' : e.depth_min.toFixed(2) + ' m'}</td></tr>` +
        `<tr><td>clearance (min)</td><td>${e.clearance_min.toFixed(1)} m</td></tr><tr><td>speed limit</td><td>${e.max_speed_mps.toFixed(2)} m/s</td></tr>` +
        `<tr><td>traffic</td><td>${e.one_way ? 'ONE WAY' : 'two-way'}${e.overtaking ? '' : ', no overtaking'}</td></tr></table>`;
    }
  }
  return html;
}

function r1Summary() {
  const I = R1.info, el = $('r1sum'); if (!el) return;
  if (!I || !I.ok) { el.textContent = I && I.message ? I.message : 'route planner 1 not running'; return; }
  const s = I.summary || {}, b = (s.graph && s.graph.blocked_edges) || [];
  el.textContent = (s.version ? `v${s.version} (${s.why}) · ` : '') + `compiled ${s.compile_s} s · water ${s.water_level_m >= 0 ? '+' : ''}${s.water_level_m} m · ${s.time_of_day} · ` +
    `needs ${s.need_depth_m} m · no-go now: ${(s.no_go_active || []).join(', ') || 'none'}` + (b.length ? ` · BLOCKED: ${b.join(', ')}` : ' · all canals passable') +
    ((s.closures || []).length ? ` · closures: ${s.closures.join(', ')}` : '');
}
async function r1Load() {
  try {
    const I = await (await fetch('/route1/info')).json();
    const changed = I.version !== R1.version; R1.info = I; R1.version = I.version;
    if (I.ok && changed) {
      const img = new Image(); img.onload = () => { R1.img = img; draw(); }; img.src = '/route1/reason.png?v=' + I.version;
      const b = await fetch('/route1/reason.bin?v=' + I.version); if (b.ok) R1.raw = new Uint8Array(await b.arrayBuffer());
    }
    r1Summary(); if (changed) draw();
  } catch (e) { /* page keeps the last one */ }
}
['showr1', 'r1graph'].forEach((id) => { const el = document.getElementById(id); if (el) el.addEventListener('change', () => draw()); });
r1Load();
setInterval(r1Load, 3000);
