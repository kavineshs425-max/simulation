'use strict';
// ====================================================================== VECTOR MAP overlay
// Draws the vector map's INFORMATION layer on the SIMU map (GET /vmap, local
// metres): depth areas (judged against the boat's draft at the current water
// level), restricted / no-go areas, speed limits, caution areas, obstructions,
// and each canal's rules (one-way / keep-starboard arrows, depth, speed,
// overtaking). The physical features are the Environment layer (env.js).
// Legend switches: vector map on/off, depth, rules & limits, labels.
// Hover anything for its class, attributes and what it means for the boat.

const VM = { data: null, hover: null };
const VM_COL = { deep: 'rgba(120,180,230,0.10)', shallow: 'rgba(40,110,200,0.28)', noentry: '#d32f2f', nowake: '#ef6c00',
  speed: '#7b1fa2', caution: '#f57c00', rule: '#0d47a1', danger: '#c62828' };
const vmOn = (id) => { const el = document.getElementById(id); return !el || el.checked; };

function vmHatch(colour, gap) {
  const c = document.createElement('canvas'); c.width = c.height = gap;
  const g = c.getContext('2d'); g.strokeStyle = colour; g.lineWidth = 1.6;
  g.beginPath(); g.moveTo(0, gap); g.lineTo(gap, 0); g.moveTo(-gap / 2, gap / 2); g.lineTo(gap / 2, -gap / 2);
  g.moveTo(gap / 2, gap * 1.5); g.lineTo(gap * 1.5, gap / 2); g.stroke();
  return ctx.createPattern(c, 'repeat');
}
let VM_PAT = null;
function vmPatterns() {
  if (!VM_PAT) VM_PAT = { noentry: vmHatch('rgba(211,47,47,.55)', 10), nowake: vmHatch('rgba(239,108,0,.45)', 12),
    shallow: vmHatch('rgba(198,40,40,.55)', 7) };
  return VM_PAT;
}
function vmRing(f) { return f.geometry.type === 'Polygon' ? f.geometry.coordinates[0] : null; }
function vmCentroid(ring) { let x = 0, y = 0; const n = ring.length - 1 || 1; for (let i = 0; i < n; i++) { x += ring[i][0]; y += ring[i][1]; } return [x / n, y / n]; }
function vmCanalPoints(f) {                  // the LIVE canal (Environment draft) when it exists
  if (typeof ENV !== 'undefined' && ENV.draft) { const c = ENV.draft.canals.find((o) => o.name === f.properties.name); if (c && c.points.length >= 2) return c.points; }
  return f.geometry.coordinates;
}
function vmCanalWidth(f) {
  if (typeof ENV !== 'undefined' && ENV.draft) { const c = ENV.draft.canals.find((o) => o.name === f.properties.name); if (c) return c.width; }
  return f.properties.width || 10;
}

// a small road-sign style speed limit: white disc, red ring, the number
function vmSpeedSign(x, y, kn) {
  const [sx, sy] = w2s(x, y), r = 11;
  ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI); ctx.fillStyle = '#fff'; ctx.fill();
  ctx.lineWidth = 3; ctx.strokeStyle = '#d32f2f'; ctx.stroke();
  ctx.fillStyle = '#111'; ctx.font = '800 10px Inter, system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(String(+kn.toFixed(1)).replace(/\.0$/, ''), sx, sy + 0.5);
}
function vmTriangle(x, y, colour) {
  const [sx, sy] = w2s(x, y);
  ctx.beginPath(); ctx.moveTo(sx, sy - 10); ctx.lineTo(sx + 9, sy + 6); ctx.lineTo(sx - 9, sy + 6); ctx.closePath();
  ctx.fillStyle = '#fff8e1'; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = colour; ctx.stroke();
  ctx.fillStyle = colour; ctx.font = '800 10px Inter, system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('!', sx, sy + 1);
}
function vmChevron(x, y, h, colour, size) {
  const [sx, sy] = w2s(x, y), a = -h, s = size || 7;
  ctx.beginPath(); ctx.moveTo(sx + s * Math.cos(a), sy + s * Math.sin(a));
  ctx.lineTo(sx + s * Math.cos(a + 2.5), sy + s * Math.sin(a + 2.5));
  ctx.moveTo(sx + s * Math.cos(a), sy + s * Math.sin(a));
  ctx.lineTo(sx + s * Math.cos(a - 2.5), sy + s * Math.sin(a - 2.5));
  ctx.lineWidth = 2.2; ctx.strokeStyle = colour; ctx.stroke();
}
// points every `step` metres along a polyline: [x, y, heading]
function vmAlong(pts, step, offset) {
  const out = []; let carry = step / 2;
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1], L = Math.hypot(bx - ax, by - ay), h = Math.atan2(by - ay, bx - ax);
    for (let s = carry; s < L; s += step) {
      const nx = -Math.sin(h), ny = Math.cos(h);
      out.push([ax + (bx - ax) * s / L + nx * (offset || 0), ay + (by - ay) * s / L + ny * (offset || 0), h]);
    }
    carry = (carry - L) % step; if (carry < 0) carry += step;
  }
  return out;
}
function vmMid(pts) {                       // the point half way along a polyline
  let L = 0; for (let i = 0; i + 1 < pts.length; i++) L += Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]);
  let s = L / 2;
  for (let i = 0; i + 1 < pts.length; i++) {
    const l = Math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]);
    if (s <= l) return [pts[i][0] + (pts[i + 1][0] - pts[i][0]) * s / (l || 1), pts[i][1] + (pts[i + 1][1] - pts[i][1]) * s / (l || 1)];
    s -= l;
  }
  return pts[0];
}
function vmChip(text, x, y, bg) {
  const [sx, sy] = w2s(x, y);
  ctx.font = '700 10.5px Inter, system-ui'; const tw = ctx.measureText(text).width;
  const box = [sx - tw / 2 - 6, sy - 8, sx + tw / 2 + 6, sy + 8];
  if (labelBoxes.some((b) => box[0] < b[2] && box[2] > b[0] && box[1] < b[3] && box[3] > b[1])) return;
  labelBoxes.push(box);
  ctx.fillStyle = bg; roundRect(box[0], box[1], tw + 12, 16, 5); ctx.fill();
  ctx.fillStyle = '#fff'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(text, sx, sy + 0.5);
}

function vmapDraw() {
  const D = VM.data; if (!D || !D.ok || !vmOn('showvmap')) return;
  const P = vmPatterns(), depthOn = vmOn('vmdepth'), rulesOn = vmOn('vmrules'), labelsOn = vmOn('vmlabels') && view.scale > 0.3;
  const draft = D.boat_draft, need = draft + D.ukc_m;
  const labels = [];
  D.features.forEach((f) => {
    const p = f.properties, ring = vmRing(f), hov = VM.hover === f;
    if (f.class === 'DEPARE' && depthOn && ring) {
      path(ring, true);
      if (p.navigable === false) { ctx.fillStyle = VM_COL.shallow; ctx.fill(); ctx.fillStyle = P.shallow; ctx.fill(); }
      else { ctx.fillStyle = p.effective_depth < 2.0 ? VM_COL.shallow : VM_COL.deep; ctx.fill(); }
      ctx.setLineDash([2, 3]); ctx.lineWidth = hov ? 2.5 : 1.1; ctx.strokeStyle = p.navigable === false ? VM_COL.danger : 'rgba(13,71,161,.55)'; ctx.stroke(); ctx.setLineDash([]);
      const [cx, cy] = vmCentroid(ring);
      if (labelsOn) labels.push(() => vmChip(`${p.drval1.toFixed(1)}${p.drval2 != null ? '–' + p.drval2.toFixed(1) : ''} m${p.navigable === false ? ' · TOO SHALLOW' : ''}`, cx, cy, p.navigable === false ? VM_COL.danger : '#1565c0'));
    }
    if (f.class === 'RESARE' && rulesOn && ring) {
      const no = p.restriction === 'entry_prohibited';
      path(ring, true); ctx.fillStyle = no ? P.noentry : P.nowake; ctx.fill();
      ctx.lineWidth = hov ? 3.5 : 2.2; ctx.strokeStyle = no ? VM_COL.noentry : VM_COL.nowake; ctx.stroke();
      const [cx, cy] = vmCentroid(ring);
      if (labelsOn) labels.push(() => vmChip(no ? `NO ENTRY · ${p.reason || ''}${p.active && p.active !== 'all year' ? ' · ' + p.active : ''}` : p.restriction.replace('_', ' ').toUpperCase(), cx, cy, no ? VM_COL.noentry : VM_COL.nowake));
    }
    if (f.class === 'SPDLIM' && rulesOn && ring) {
      path(ring, true); ctx.setLineDash([9, 5]); ctx.lineWidth = hov ? 3 : 1.6; ctx.strokeStyle = VM_COL.speed; ctx.stroke(); ctx.setLineDash([]);
      const [x0, y0] = ring[0], [x2, y2] = ring[2] || ring[0];
      labels.push(() => vmSpeedSign(Math.min(x0, x2) + 5 / Math.max(view.scale, 0.3) + 4, Math.max(y0, y2) - 5 / Math.max(view.scale, 0.3) - 4, p.max_speed_kn));
    }
    if (f.class === 'CTNARE' && rulesOn) {
      let at;
      if (ring) { path(ring, true); ctx.setLineDash([5, 4]); ctx.lineWidth = hov ? 3 : 1.8; ctx.strokeStyle = VM_COL.caution; ctx.stroke(); ctx.setLineDash([]); at = vmCentroid(ring); }
      else at = f.geometry.coordinates;
      labels.push(() => { vmTriangle(at[0], at[1], VM_COL.caution); if (labelsOn && hov) vmChip(p.text, at[0], at[1] - 14 / view.scale, VM_COL.caution); });
    }
    if (f.class === 'OBSTRN' && rulesOn) {
      const at = f.geometry.type === 'Point' ? f.geometry.coordinates : vmCentroid(ring), [sx, sy] = w2s(at[0], at[1]);
      labels.push(() => {
        ctx.beginPath(); ctx.arc(sx, sy, 8, 0, 2 * Math.PI); ctx.setLineDash([2, 2]); ctx.lineWidth = 2; ctx.strokeStyle = VM_COL.danger; ctx.stroke(); ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(sx - 4, sy - 4); ctx.lineTo(sx + 4, sy + 4); ctx.moveTo(sx + 4, sy - 4); ctx.lineTo(sx - 4, sy + 4); ctx.stroke();
        if (labelsOn) vmChip(`${p.valsou != null ? p.valsou.toFixed(1) + ' m · ' : ''}${(p.text || 'obstruction').split(',')[0]}`, at[0], at[1] - 16 / view.scale, VM_COL.danger);
      });
    }
    if (f.class === 'CANALS' && rulesOn) {
      const pts = vmCanalPoints(f), w = vmCanalWidth(f), step = Math.max(28, 70 / view.scale);
      if (p.traffic === 'one_way') vmAlong(pts, step, 0).forEach(([x, y, h]) => vmChevron(x, y, h, VM_COL.rule, 8));
      else if (p.keep === 'starboard') {
        // right-hand traffic: going along the points you keep to the RIGHT (starboard) side
        vmAlong(pts, step, -w / 4).forEach(([x, y, h]) => vmChevron(x, y, h, 'rgba(13,71,161,.8)', 6));
        vmAlong(pts, step, +w / 4).forEach(([x, y, h]) => vmChevron(x, y, h + Math.PI, 'rgba(13,71,161,.8)', 6));
      }
      if (p.navigable === false) { path(pts); ctx.setLineDash([6, 4]); ctx.lineWidth = 3; ctx.strokeStyle = VM_COL.danger; ctx.stroke(); ctx.setLineDash([]); }
      if (hov) { path(pts); ctx.lineWidth = Math.max(w * view.scale, 3); ctx.strokeStyle = 'rgba(13,71,161,.15)'; ctx.lineCap = 'round'; ctx.stroke(); ctx.lineCap = 'butt'; }
      const mid = vmMid(pts);
      if (labelsOn) labels.push(() => vmChip(`${p.depth != null ? p.depth.toFixed(1) + ' m · ' : ''}${p.traffic === 'one_way' ? 'ONE WAY' : 'two-way'}${p.max_speed_kn ? ' · ' + p.max_speed_kn + ' kn' : ''}${p.overtaking === false ? ' · no overtaking' : ''}`,
        mid[0], mid[1], p.navigable === false ? VM_COL.danger : VM_COL.rule));
    }
    if (f.class === 'HRBBSN' && labelsOn && p.depth != null) {
      const [x, y] = f.geometry.coordinates; labels.push(() => vmChip(`basin · ${p.depth.toFixed(1)} m`, x, y + 6, '#1565c0'));
    }
    if (f.class === 'BRIDGE' && labelsOn) {
      const s = (typeof ENV !== 'undefined' && ENV.draft) ? ENV.draft.structures.find((o) => o.name === p.name) : null, o = s || Object.assign({ x: f.geometry.coordinates[0], y: f.geometry.coordinates[1] }, p);
      const clr = o.clearance, ok = clr >= (VM.data.air_draft || 2.5);
      labels.push(() => vmChip(`clearance ${(+clr).toFixed(1)} m`, o.x, o.y - 26 / view.scale - 6, ok ? '#455a64' : VM_COL.danger));
    }
  });
  labels.forEach((fn) => fn());           // symbols + chips on top of every fill
}

// ---------------------------------------------------------------- hover
function vmHit(wx, wy) {
  const D = VM.data; if (!D || !D.ok || !vmOn('showvmap')) return null;
  const tol = 10 / view.scale, pri = { OBSTRN: 0, CTNARE: 1, RESARE: 2, BRIDGE: 3, CANALS: 4, HRBBSN: 5, SPDLIM: 7, DEPARE: 6 };
  const hits = D.features.filter((f) => {
    if (!(f.class in pri)) return false;
    if ((f.class === 'DEPARE') && !vmOn('vmdepth')) return false;
    if (['RESARE', 'SPDLIM', 'CTNARE', 'OBSTRN', 'CANALS'].includes(f.class) && !vmOn('vmrules')) return false;
    const g = f.geometry;
    if (g.type === 'Point') return Math.hypot(g.coordinates[0] - wx, g.coordinates[1] - wy) < Math.max(tol, f.class === 'BRIDGE' ? 18 : 0);
    if (g.type === 'LineString') return polyDist(wx, wy, vmCanalPoints(f)) <= Math.max(vmCanalWidth(f) / 2, tol);
    return pointInPoly(wx, wy, g.coordinates[0]);
  });
  // same class (a shoal inside a lake's depth area): the SMALLER, more specific one wins
  const area = (f) => { const r = vmRing(f); if (!r) return 0; let s = 0; for (let i = 0; i + 1 < r.length; i++) s += r[i][0] * r[i + 1][1] - r[i + 1][0] * r[i][1]; return Math.abs(s) / 2; };
  hits.sort((a, b) => pri[a.class] - pri[b.class] || area(a) - area(b));
  return hits[0] || null;
}
function vmExplain(f) {
  const p = f.properties, D = VM.data, need = (D.boat_draft + D.ukc_m).toFixed(1);
  const skip = new Set(['class', 'name', 'description', 'effective_depth', 'navigable']);
  const rows = Object.entries(p).filter(([k]) => !skip.has(k)).map(([k, v]) => `<tr><td>${k}</td><td>${typeof v === 'number' ? +v.toFixed(3) : v}</td></tr>`).join('');
  let verdict = '';
  if (p.navigable === false) verdict = `<div class="vmbad">TOO SHALLOW for this boat: ${p.effective_depth.toFixed(2)} m of water &lt; ${need} m needed (draft ${D.boat_draft} + clearance ${D.ukc_m})</div>`;
  else if (p.navigable === true) verdict = `<div class="vmok">navigable: ${p.effective_depth.toFixed(2)} m of water &ge; ${need} m needed</div>`;
  if (f.class === 'RESARE' && p.restriction === 'entry_prohibited') verdict += '<div class="vmbad">NO-GO: the route planner must not enter</div>';
  if (f.class === 'CANALS' && p.traffic === 'one_way') verdict += '<div class="vmwarn">one way: only in the direction of its points (the arrows)</div>';
  return `<div class="vmh"><b>${f.class}</b> &middot; ${f.label}</div><div class="vmn">${p.name}</div>` +
    (p.description ? `<div class="vmd">${p.description}</div>` : '') + verdict + (rows ? `<table>${rows}</table>` : '');
}
function vmapHover(px, py, wx, wy) {
  const f = vmHit(wx, wy), box = $('vminfo');
  if (f !== VM.hover) { VM.hover = f; draw(); }
  const extra = (typeof r1Explain === 'function') ? r1Explain(wx, wy) : '';      // route planner 1 (route1.js)
  if (!f && !extra) { box.style.display = 'none'; return; }
  box.innerHTML = (f ? vmExplain(f) : '') + extra; box.style.display = 'block';
  const r = $('mapwrap').getBoundingClientRect(), bw = box.offsetWidth, bh = box.offsetHeight;
  box.style.left = Math.min(px + 18, r.width - bw - 10) + 'px'; box.style.top = Math.min(py + 14, r.height - bh - 10) + 'px';
}
function vmapInfo() {
  const D = VM.data, el = $('vmsum'); if (!el) return;
  if (!D || !D.ok) { el.textContent = D && D.message ? D.message : 'no vector map'; return; }
  const n = (c) => D.features.filter((f) => f.class === c).length;
  el.textContent = `${D.features.length} features · water level ${D.water_level_m >= 0 ? '+' : ''}${D.water_level_m} m · needs ${(D.boat_draft + D.ukc_m).toFixed(1)} m depth · ${n('RESARE')} restricted · ${n('SPDLIM')} speed zones`;
}
async function vmapLoad() {
  try { VM.data = await (await fetch('/vmap')).json(); } catch (e) { VM.data = null; }
  if (VM.data && VM.data.ok && typeof ENV !== 'undefined' && ENV.draft) VM.data.air_draft = ENV.draft.settings.boat_air_draft;
  vmapInfo(); draw();
}
['showvmap', 'vmdepth', 'vmrules', 'vmlabels'].forEach((id) => { const el = document.getElementById(id); if (el) el.addEventListener('change', () => { if (id === 'showvmap') $('vmsub').classList.toggle('dim', !el.checked); draw(); }); });
vmapLoad();
setInterval(vmapLoad, 15000);                // the map changes only on Update / reload: a slow refresh is enough
