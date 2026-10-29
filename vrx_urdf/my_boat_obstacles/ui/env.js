'use strict';
// ====================================================================== ENVIRONMENT editor
// The Environment tab of the SIMU page: canals, basins, land regions and fixed
// structures (docks, bridges, walls, piles, nets). Loaded after index.html's own
// script and uses its globals (ctx, view, w2s, s2w, draw, say, post, deep, ...).
//
// TERRAIN is drawn here with the SAME rule the server rasterises:
//     solid = (land U canal banks) - (canal water U basins)
// (a canvas stroke with round caps/joins IS "distance to the polyline <= w/2").
// STRUCTURE footprints come from the server (/env/geometry) in each structure's
// local frame, so the page never re-implements their geometry: it only moves
// and rotates them while you drag.

const ENV = {
  active: false, types: null, applied: null, draft: null, saved: null, file: '',
  sel: null, editPts: false, place: null, geo: {}, checks: { error: null, warnings: [] },
  drag: null,
};
const ENV_SECS = { canals: 'Canals', basins: 'Basins', land: 'Land regions', structures: 'Structures' };

// ---------------------------------------------------------------- model helpers
function envItems() {
  const d = ENV.draft; if (!d) return [];
  return [...d.canals.map((o) => ['canals', o]), ...d.basins.map((o) => ['basins', o]),
          ...d.land.map((o) => ['land', o]), ...d.structures.map((o) => ['structures', o])];
}
function envFind(name) { const it = envItems().find(([, o]) => o.name === name); return it ? { sec: it[0], o: it[1] } : null; }
function envUnique(base) {
  base = base.replace(/[^a-z0-9_]+/gi, '_');
  for (let i = 1; ; i++) { const n = `${base}_${i}`; if (!envFind(n)) return n; }
}
const envSame = (a, b) => JSON.stringify(a) === JSON.stringify(b);
function envGeoKey(s) { const k = deep(s); delete k.x; delete k.y; delete k.heading_deg; delete k.name; return JSON.stringify(k); }

function polyDist(x, y, pts) {
  let best = Infinity;
  for (let i = 0; i + 1 < pts.length; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1], dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
    const t = L2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / L2));
    best = Math.min(best, Math.hypot(x - (ax + t * dx), y - (ay + t * dy)));
  }
  return best;
}
// as environment.canal_water_line: an OPEN end runs on past the banks, unless it is a junction
function envWaterLine(c) {
  const pts = c.points.map((p) => [p[0], p[1]]), ext = c.bank_width + c.width / 2 + 1;
  const inOther = (p) => ENV.draft.canals.some((o) => o.name !== c.name && polyDist(p[0], p[1], o.points) <= o.width / 2 + 1e-6);
  const extend = (p, q) => { const dx = p[0] - q[0], dy = p[1] - q[1], n = Math.hypot(dx, dy) || 1; return [p[0] + dx / n * ext, p[1] + dy / n * ext]; };
  if (pts.length >= 2) {
    if (c.open_start !== false && !inOther(pts[0])) pts[0] = extend(pts[0], pts[1]);
    if (c.open_end !== false && !inOther(pts[pts.length - 1])) pts[pts.length - 1] = extend(pts[pts.length - 1], pts[pts.length - 2]);
  }
  return pts;
}
function basinPoly(b) {
  const h = rad(b.heading_deg || 0), c = Math.cos(h), s = Math.sin(h), L = b.length / 2, W2 = b.width / 2;
  return [[L, W2], [-L, W2], [-L, -W2], [L, -W2]].map(([dx, dy]) => [b.x + c * dx - s * dy, b.y + s * dx + c * dy]);
}
function sToWorld(s, x, y) { const h = rad(s.heading_deg || 0), c = Math.cos(h), n = Math.sin(h); return [s.x + c * x - n * y, s.y + n * x + c * y]; }
function partPoly(s, p) {
  if (p.kind === 'cyl') { const r = p.l / 2, out = []; for (let k = 0; k < 12; k++) { const a = k * Math.PI / 6; out.push(sToWorld(s, p.x + r * Math.cos(a), p.y + r * Math.sin(a))); } return out; }
  const c = Math.cos(p.yaw || 0), n = Math.sin(p.yaw || 0);
  return [[p.l / 2, p.w / 2], [-p.l / 2, p.w / 2], [-p.l / 2, -p.w / 2], [p.l / 2, -p.w / 2]]
    .map(([dx, dy]) => sToWorld(s, p.x + c * dx - n * dy, p.y + n * dx + c * dy));
}
const rgb = (c, a) => `rgba(${Math.round(c[0] * 255)},${Math.round(c[1] * 255)},${Math.round(c[2] * 255)},${a == null ? 1 : a})`;

// ---------------------------------------------------------------- drawing
let envOff = null, envWater = null;
// the lake map's WATER as an alpha mask: the server only builds terrain where the
// world is water, so the page clips to it too (what you see = what Gazebo gets)
function envWaterMask() {
  if (envWater || !mapImg || !mapImg.complete || !mapImg.naturalWidth) return envWater;
  const c = document.createElement('canvas'); c.width = mapImg.naturalWidth; c.height = mapImg.naturalHeight;
  const g = c.getContext('2d'); g.drawImage(mapImg, 0, 0);
  const im = g.getImageData(0, 0, c.width, c.height), px = im.data;
  for (let i = 0; i < px.length; i += 4) {
    const water = Math.abs(px[i] - 168) < 12 && Math.abs(px[i + 1] - 205) < 12 && Math.abs(px[i + 2] - 232) < 12;
    px[i + 3] = water ? 255 : 0;
  }
  g.putImageData(im, 0, 0); envWater = c; return c;
}
function envDraw() {
  const d = ENV.draft; if (!d) return;
  // --- terrain, on its own layer so water can be CUT out of it
  if (!envOff) envOff = document.createElement('canvas');
  if (envOff.width !== canvas.width || envOff.height !== canvas.height) { envOff.width = canvas.width; envOff.height = canvas.height; }
  const g = envOff.getContext('2d');
  g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, envOff.width, envOff.height);
  g.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  const tc = (d.settings && d.settings.terrain_colour) || [0.42, 0.5, 0.3];
  g.fillStyle = g.strokeStyle = rgb(tc);
  const line = (pts, w) => {
    g.beginPath(); pts.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? g.lineTo(sx, sy) : g.moveTo(sx, sy); });
    g.lineWidth = Math.max(w * view.scale, 1); g.lineCap = 'round'; g.lineJoin = 'round'; g.stroke();
  };
  const poly = (pts) => { g.beginPath(); pts.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? g.lineTo(sx, sy) : g.moveTo(sx, sy); }); g.closePath(); g.fill(); };
  d.land.forEach((l) => { if (l.points.length >= 3) poly(l.points); });
  d.canals.forEach((c) => { if (c.points.length >= 2 && c.bank_width > 0) line(c.points, c.width + 2 * c.bank_width); });
  g.globalCompositeOperation = 'destination-out';
  d.canals.forEach((c) => { if (c.points.length >= 2) line(envWaterLine(c), c.width); });
  d.basins.forEach((b) => poly(basinPoly(b)));
  const wm = envWaterMask();
  if (wm && MAP) {
    const [sx, sy] = w2s(MAP.x0, MAP.y0 + MAP.h * MAP.res);
    g.globalCompositeOperation = 'destination-in'; g.imageSmoothingEnabled = false;
    g.drawImage(wm, sx, sy, MAP.w * MAP.res * view.scale, MAP.h * MAP.res * view.scale);
  }
  g.globalCompositeOperation = 'source-over';
  ctx.save(); ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.globalAlpha = 0.92; ctx.drawImage(envOff, 0, 0); ctx.restore();

  // --- structures
  d.structures.forEach((s) => {
    const G = ENV.geo[s.name]; if (!G || !G.data || G.data.error) return;
    G.data.parts.forEach((p) => {
      path(partPoly(s, p), true);
      if (p.visual) { ctx.fillStyle = rgb(p.colour || [0.6, 0.6, 0.6], p.z0 >= 2.5 ? 0.55 : 1); ctx.fill();
        ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(21,32,43,.55)'; ctx.stroke(); }
      else { ctx.setLineDash([4, 3]); ctx.lineWidth = 2; ctx.strokeStyle = '#d93025'; ctx.stroke(); ctx.setLineDash([]); }   // the hidden net
    });
    if (ENV.active || view.scale > 1.5) G.data.berths.forEach((b) => {
      const [x, y] = sToWorld(s, b.x, b.y), h = b.yaw + rad(s.heading_deg || 0);
      path([[x - 2.2 * Math.cos(h), y - 2.2 * Math.sin(h)], [x + 2.2 * Math.cos(h), y + 2.2 * Math.sin(h)]]);
      ctx.lineWidth = 2; ctx.strokeStyle = '#1e8e3e'; ctx.stroke();
      arrowHead(x, y, x + 2.2 * Math.cos(h), y + 2.2 * Math.sin(h), '#1e8e3e');
    });
  });
}
function envDrawOverlay() {           // on top of everything, only while editing the environment
  if (!ENV.active || !ENV.draft) return;
  const d = ENV.draft;
  d.canals.forEach((c) => {
    const s = ENV.sel === c.name; path(c.points); ctx.setLineDash([6, 5]);
    ctx.lineWidth = s ? 2.2 : 1; ctx.strokeStyle = s ? COL.sel : 'rgba(21,32,43,.45)'; ctx.stroke(); ctx.setLineDash([]);
  });
  envItems().forEach(([sec, o]) => {
    const s = ENV.sel === o.name;
    if (sec === 'land') { path(o.points, true); ctx.lineWidth = s ? 2.4 : 1; ctx.setLineDash(s ? [] : [3, 4]); ctx.strokeStyle = s ? COL.sel : 'rgba(21,32,43,.4)'; ctx.stroke(); ctx.setLineDash([]); }
    if (sec === 'basins' && s) { path(basinPoly(o), true); ctx.lineWidth = 2.4; ctx.strokeStyle = COL.sel; ctx.stroke(); }
    if (sec === 'structures' && s) {
      const G = ENV.geo[o.name];
      if (G && G.data && !G.data.error) G.data.parts.forEach((p) => { path(partPoly(o, p), true); ctx.lineWidth = 2.4; ctx.strokeStyle = COL.sel; ctx.stroke(); });
    }
    const at = sec === 'canals' ? o.points[Math.floor((o.points.length - 1) / 2)] || [0, 0] : sec === 'land' ? o.points[0] || [0, 0] : [o.x, o.y];
    if (view.scale > 0.35 || s) label(o.name, at[0], at[1], s ? COL.sel : '#3b2f14', s);
    if (s && (sec === 'canals' || sec === 'land')) o.points.forEach(([x, y], i) => {
      const [sx, sy] = w2s(x, y); ctx.beginPath(); ctx.arc(sx, sy, 6.5, 0, 2 * Math.PI); ctx.fillStyle = '#fff'; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = COL.sel; ctx.stroke();
      ctx.fillStyle = COL.sel; ctx.font = '700 9px Inter, system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(String(i + 1), sx, sy + 0.5);
    });
  });
  if (ENV.place && hoverWorld) {
    const [x, y] = w2s(hoverWorld[0], hoverWorld[1]); ctx.beginPath(); ctx.arc(x, y, 7, 0, 2 * Math.PI); ctx.strokeStyle = COL.sel; ctx.lineWidth = 2; ctx.stroke();
  }
}

// ---------------------------------------------------------------- hit test + mouse
function envHit(wx, wy) {
  const tol = 9 / view.scale, d = ENV.draft;
  const sel = ENV.sel && envFind(ENV.sel);
  if (sel && (sel.sec === 'canals' || sel.sec === 'land')) {
    const i = sel.o.points.findIndex(([x, y]) => Math.hypot(x - wx, y - wy) < tol);
    if (i >= 0) return { name: sel.o.name, vertex: i };
  }
  for (const s of [...d.structures].reverse()) {
    const G = ENV.geo[s.name];
    if (G && G.data && !G.data.error && G.data.parts.some((p) => pointInPoly(wx, wy, partPoly(s, p)))) return { name: s.name };
    if (Math.hypot(wx - s.x, wy - s.y) < tol) return { name: s.name };
  }
  for (const b of [...d.basins].reverse()) if (pointInPoly(wx, wy, basinPoly(b))) return { name: b.name };
  for (const c of [...d.canals].reverse()) if (polyDist(wx, wy, c.points) <= Math.max(c.width / 2, tol)) return { name: c.name };
  for (const l of [...d.land].reverse()) if (l.points.length >= 3 && pointInPoly(wx, wy, l.points)) return { name: l.name };
  return null;
}
// -> true when the event was used; index.html pans otherwise
function envMouseDown(e, wx, wy) {
  const d = ENV.draft; if (!d) return false;
  if (ENV.place) {
    if (e.button === 2) { envSetPlace(null); return true; }
    envAdd(ENV.place, wx, wy); return true;
  }
  const s = ENV.sel && envFind(ENV.sel);
  if (ENV.editPts && s) {
    if (e.button === 2) { s.o.points.pop(); envChanged(); return true; }
    const h = envHit(wx, wy);
    if (h && h.name === s.o.name && h.vertex != null) { ENV.drag = { vertex: h.vertex }; return true; }
    s.o.points.push([+wx.toFixed(2), +wy.toFixed(2)]); envChanged(); return true;
  }
  if (e.button !== 0) return false;
  const h = envHit(wx, wy);
  if (!h) return false;
  envSelect(h.name);
  const it = envFind(h.name);
  if (h.vertex != null) { ENV.drag = { vertex: h.vertex }; return true; }
  if (it.sec === 'structures' || it.sec === 'basins') { ENV.drag = { move: true, dx: it.o.x - wx, dy: it.o.y - wy }; return true; }
  // canals / land: the body moves only with Shift (they are big: a stray drag must not move the world)
  if (e.shiftKey) { ENV.drag = { shift: true, wx, wy, pts: deep(it.o.points) }; return true; }
  return false;
}
function envMouseMove(wx, wy) {
  if (!ENV.drag) return false;
  const it = envFind(ENV.sel); if (!it) { ENV.drag = null; return false; }
  const D = ENV.drag;
  if (D.vertex != null) it.o.points[D.vertex] = [+wx.toFixed(2), +wy.toFixed(2)];
  else if (D.move) { it.o.x = +(wx + D.dx).toFixed(2); it.o.y = +(wy + D.dy).toFixed(2); }
  else if (D.shift) { const dx = wx - D.wx, dy = wy - D.wy; it.o.points = D.pts.map(([x, y]) => [+(x + dx).toFixed(2), +(y + dy).toFixed(2)]); }
  canvas.classList.add('drag'); envChanged(true); return true;
}
function envMouseUp() { const was = !!ENV.drag; ENV.drag = null; if (was) envChanged(); return was; }
function envKey(e) {
  if (!ENV.active) return false;
  if (e.key === 'Escape' || e.key === 'Enter') {
    if (ENV.place) envSetPlace(null); else if (ENV.editPts) envSetEditPts(false); else if (e.key === 'Escape') envSelect(null);
    return true;
  }
  if ((e.key === 'Delete' || e.key === 'Backspace') && ENV.sel && !ENV.editPts) { envDelete(); return true; }
  return false;
}

// ---------------------------------------------------------------- add / delete / select
function envAdd(kind, x, y) {
  const d = ENV.draft, X = +x.toFixed(2), Y = +y.toFixed(2);
  let o;
  if (kind === 'canal') { o = Object.assign({ name: envUnique('canal') }, deep(ENV.types.canal), { points: [[X, Y]] }); d.canals.push(o); }
  else if (kind === 'land') { o = { name: envUnique('land'), points: [[X, Y]] }; d.land.push(o); }
  else if (kind === 'basin') { o = { name: envUnique('basin'), x: X, y: Y, heading_deg: 0, length: 40, width: 30 }; d.basins.push(o); }
  else {
    o = { name: envUnique(kind), type: kind, x: X, y: Y, heading_deg: 0 };
    Object.entries(ENV.types.types[kind]).forEach(([k, v]) => { o[k] = v.default; });
    d.structures.push(o);
  }
  envSetPlace(null); envSelect(o.name);
  if (kind === 'canal' || kind === 'land') envSetEditPts(true);
  say(`added '${o.name}'` + (kind === 'canal' || kind === 'land' ? ' -- keep clicking to add points, Esc when done' : ' -- press Update to build it'), 'ok');
  envChanged();
}
function envDelete() {
  const it = envFind(ENV.sel); if (!it) return;
  ENV.draft[it.sec] = ENV.draft[it.sec].filter((o) => o.name !== it.o.name);
  say(`removed '${it.o.name}' -- press Update to take it out of the sim`, 'warn');
  envSelect(null); envChanged();
}
function envSelect(name) { if (ENV.editPts && name !== ENV.sel) ENV.editPts = false; ENV.sel = name; envPanel(); draw(); }
function envSetPlace(kind) {
  ENV.place = kind; if (kind) { ENV.editPts = false; ENV.sel = null; }
  canvas.classList.toggle('place', !!kind); envMode(); envPanel(); draw();
}
function envSetEditPts(on) { ENV.editPts = on; canvas.classList.toggle('wp', on); envMode(); envPanel(); draw(); }
function envMode() {
  const m = $('mode');
  if (!ENV.active) return;
  if (ENV.place) { m.innerHTML = `Placing <b>${envKindLabel(ENV.place)}</b> &middot; click the map &middot; Esc cancels`; m.style.display = 'block'; }
  else if (ENV.editPts && ENV.sel) { m.innerHTML = `Editing the points of <b>${ENV.sel}</b> &middot; click adds &middot; drag moves &middot; right-click removes the last &middot; Esc when done`; m.style.display = 'block'; }
  else m.style.display = 'none';
}
const envKindLabel = (k) => ({ canal: 'Canal', basin: 'Basin', land: 'Land region' }[k] || (ENV.types && ENV.types.labels[k]) || k);

// ---------------------------------------------------------------- geometry + checks from the server
let envGeoTimer = null, envCheckTimer = null, envCheckSeq = 0;
function envFetchGeometry() {
  clearTimeout(envGeoTimer);
  envGeoTimer = setTimeout(async () => {
    const need = ENV.draft.structures.filter((s) => !ENV.geo[s.name] || ENV.geo[s.name].key !== envGeoKey(s));
    if (!need.length) return;
    try {
      const r = await post('/env/geometry', { settings: ENV.draft.settings, structures: need });
      need.forEach((s) => { ENV.geo[s.name] = { key: envGeoKey(s), data: r[s.name] }; if (r[s.name] && r[s.name].error) say(r[s.name].error, 'bad'); });
      draw(); if (ENV.active) envPanel();
    } catch (e) { /* retried on the next edit */ }
  }, 120);
}
function envScheduleCheck() {
  clearTimeout(envCheckTimer);
  envCheckTimer = setTimeout(async () => {
    const seq = ++envCheckSeq;
    try {
      const r = await post('/env/check', { doc: ENV.draft }); if (seq !== envCheckSeq) return;
      const before = new Set(ENV.checks.warnings.map((w) => w.text));
      ENV.checks = { error: r.ok ? null : r.error, warnings: r.warnings || [], boxes: r.terrain_boxes, berths: r.berths || [] };
      if (ENV.checks.error) say('environment: ' + ENV.checks.error, 'bad');
      ENV.checks.warnings.filter((w) => !before.has(w.text)).forEach((w) => say(w.text, 'warn'));
      refreshBadges(); if (ENV.active) envPanel();
    } catch (e) { /* advisory */ }
  }, 600);
}
function envChanged(light) {
  envFetchGeometry(); if (!light) { envScheduleCheck(); envPanel(); } else envPanelValues();
  refreshBadges(); draw();
}
function envBadges() {
  const b = [];
  if (!ENV.draft) return b;
  if (!envSame(ENV.draft, ENV.applied)) b.push('<span class="badge warn">environment not applied</span>');
  if (!envSame(ENV.draft, ENV.saved)) b.push('<span class="badge warn">environment not saved</span>');
  if (ENV.checks.error) b.push('<span class="badge bad">environment error</span>');
  else if (ENV.checks.warnings.length) b.push(`<span class="badge bad" title="${ENV.checks.warnings.map((w) => w.text).join('\n')}">&#9888; env ${ENV.checks.warnings.length}</span>`);
  return b;
}

// ---------------------------------------------------------------- panel
function envPanel() {
  const el = $('envpanel'); if (!el || !ENV.active) return;
  if (!ENV.draft) { el.innerHTML = '<div class="note">waiting for sim_environment ...</div>'; return; }
  const it = ENV.sel && envFind(ENV.sel);
  if (!it) { el.innerHTML = envSceneHtml(); envBindScene(); return; }
  el.innerHTML = envItemHtml(it); envBindItem(it); envPanelValues();
}
function envSceneHtml() {
  const d = ENV.draft, n = envItems().length;
  const rows = Object.entries(ENV_SECS).map(([sec, title]) => d[sec].map((o) =>
    `<div class="item" data-n="${o.name}"><span class="dot" style="background:${sec === 'structures' ? rgb(ENV.types.colours[o.type] || [0.5, 0.5, 0.5]) : sec === 'land' ? '#6b7f4c' : '#5aa0d8'}"></span>` +
    `<span class="nm">${o.name}</span><span class="md">${sec === 'structures' ? envKindLabel(o.type) : title.replace(/s$/, '').toLowerCase()}</span></div>`).join('')).join('');
  const add = ['canal', 'basin', 'land', ...Object.keys(ENV.types.types)];
  const desc = { canal: 'centreline + width, banks', basin: 'water cut out of land', land: 'solid ground polygon',
    pontoon: 'walkway + finger berths', jetty: 'pier on piles', bridge: 'deck, clearance, piers', wall: 'quay wall / breakwater',
    pile_row: 'mooring piles', net: 'floats + hidden net' };
  return `<div class="sec"><span>Environment</span><small>${n} items${ENV.checks.boxes != null ? ' · ' + ENV.checks.boxes + ' terrain boxes' : ''}</small></div>
    <div class="list">${rows || '<div class="note" style="padding:6px">empty -- add a canal or a structure below</div>'}</div>
    <div class="sec"><span>Add</span><small>pick one, then click the map</small></div>
    <div class="pal">${add.map((k) => `<div class="pcard${ENV.place === k ? ' on' : ''}" data-k="${k}"><span class="sw" style="background:${k === 'canal' || k === 'basin' ? '#5aa0d8' : k === 'land' ? '#6b7f4c' : rgb(ENV.types.colours[k])}"></span>
      <div style="min-width:0"><div class="pn">${envKindLabel(k)}</div><div class="ps">${desc[k] || ''}</div></div></div>`).join('')}</div>
    <div class="note">Land is solid, canal water and basins cut through it, banks rise along canals in open water.
      Drag structures and basins to move them; canals and land move by their points (Shift-drag moves the whole shape).</div>`;
}
function envBindScene() {
  $('envpanel').querySelectorAll('.item').forEach((d) => d.onclick = () => {
    const it = envFind(d.dataset.n); envSelect(d.dataset.n);
    const at = it.o.points ? it.o.points[0] : [it.o.x, it.o.y]; view.cx = at[0]; view.cy = at[1]; if (view.scale < 1.5) view.scale = 2.5; draw();
  });
  $('envpanel').querySelectorAll('.pcard').forEach((c) => c.onclick = () => envSetPlace(ENV.place === c.dataset.k ? null : c.dataset.k));
}
function envField(key, label, unit, step, extra) {
  return `<div class="field"><span class="lbl">${label}</span><input data-k="${key}" type="number" step="${step || 0.5}" ${extra || ''}><span class="unit">${unit || ''}</span></div>`;
}
function envItemHtml(it) {
  const o = it.o, sec = it.sec, mine = ENV.checks.warnings.filter((w) => (w.names || []).includes(o.name));
  let h = `<div class="head"><button class="iconbtn" id="eback" title="Back (Esc)">&#8249;</button>
      <div class="t"><h2>${o.name}</h2><div class="chips"><span class="chip">${sec === 'structures' ? envKindLabel(o.type) : sec.replace(/s$/, '')}</span>
      ${envSame(envFind(o.name) && o, ENV.applied && (ENV.applied[sec] || []).find((a) => a.name === o.name)) ? '' : '<span class="chip st new">changed · press Update</span>'}</div></div>
      <button class="iconbtn danger" id="edel" title="Delete (Del)">&#128465;</button></div>` + mine.map((w) => `<div class="warnbox">&#9888; ${w.text}</div>`).join('');
  if (sec === 'structures' || sec === 'basins') {
    h += `<div class="card"><h3>Position <small>world metres</small></h3><div class="pair">${envField('x', 'x', 'E')}${envField('y', 'y', 'N')}</div></div>
      <div class="card"><h3>Orientation <small>drag the compass &middot; Shift snaps 15&deg;</small></h3>
      <div id="dialrow"><canvas id="edial" width="256" height="256" style="width:128px;height:128px;cursor:grab;touch-action:none"></canvas>
      <div id="orimeta"><div class="field"><input data-k="heading_deg" type="number" step="1" style="padding-left:10px;padding-right:24px"><span class="unit">&deg;</span></div>
      <div class="note" id="ebearing">&nbsp;</div></div></div></div>`;
  }
  if (sec === 'basins') h += `<div class="card"><h3>Size</h3><div class="pair">${envField('length', 'L', 'm')}${envField('width', 'W', 'm')}</div></div>`;
  if (sec === 'canals') {
    h += `<div class="card"><h3>Canal</h3><div class="pair">${envField('width', 'W', 'm', 0.5, 'title="water width"')}${envField('bank_width', 'B', 'm', 0.5, 'title="bank width each side"')}</div>
      <div class="note">W = water width &middot; B = bank on each side (where no land region covers it)</div>
      <div class="tools" style="margin-top:8px"><label class="switch"><input type="checkbox" data-b="open_start"> open start</label>
      <label class="switch"><input type="checkbox" data-b="open_end"> open end</label></div>
      <div class="note">An end that is not open is a dead end (a backwater). An end inside another canal is a junction.</div></div>`;
  }
  if (sec === 'canals' || sec === 'land') {
    h += `<div class="card"><h3>Points <small>${o.points.length} point${o.points.length === 1 ? '' : 's'}</small></h3>
      <div class="tools"><button class="btn small${ENV.editPts ? ' on' : ''}" id="epts">${ENV.editPts ? 'Done (Esc)' : 'Edit points'}</button>
      <button class="btn small ghost" id="erev" title="reverse the point order">Reverse</button></div>
      <div class="note">Click the map to add a point at the end &middot; drag a point to move it &middot; right-click removes the last.</div></div>`;
  }
  if (sec === 'structures') {
    const sch = ENV.types.types[o.type];
    h += `<div class="card"><h3>${envKindLabel(o.type)}</h3><div style="display:grid;grid-template-columns:1fr;gap:6px">` +
      Object.entries(sch).map(([k, v]) => typeof v.default === 'string'
        ? `<div class="field"><span class="lbl" style="position:static;margin-right:8px;min-width:110px">${v.label}</span><select data-s="${k}" style="flex:1;padding:6px;border:1px solid var(--line2);border-radius:8px">${['left', 'right', 'both'].map((x) => `<option>${x}</option>`).join('')}</select></div>`
        : `<div style="display:grid;grid-template-columns:1fr 110px;align-items:center;gap:8px"><span class="note" style="margin:0">${v.label}</span>` +
          `<div class="field"><input data-k="${k}" type="number" step="${Number.isInteger(v.default) ? 1 : 0.1}" min="${v.min}" max="${v.max}" style="padding-left:10px"><span class="unit">${v.unit}</span></div></div>`).join('') + '</div></div>';
    const G = ENV.geo[o.name];
    if (G && G.data && G.data.berths && G.data.berths.length) {
      h += `<div class="card"><h3>Berths <small>docking pose = boat centre, bow out</small></h3>` + G.data.berths.map((b) => {
        const [x, y] = sToWorld(o, b.x, b.y), hd = wrapDeg(deg(b.yaw) + (o.heading_deg || 0));
        return `<div class="note" style="margin:2px 0">${b.name}: (${x.toFixed(1)}, ${y.toFixed(1)}) heading ${hd.toFixed(0)}&deg; &middot; ${b.width.toFixed(1)} m wide</div>`;
      }).join('') + '</div>';
    }
    if (o.type === 'bridge') h += `<div class="note">The deck blocks navigation only if its clearance is below the boat's air draft (${(ENV.draft.settings.boat_air_draft || 2.5)} m); piers and abutments always do.</div>`;
    if (o.type === 'net') h += `<div class="note">The floats are visible and can be passed through; the net under them is INVISIBLE (the lidar cannot see it) and stops the boat.</div>`;
  }
  return h;
}
function envBindItem(it) {
  const o = it.o, el = $('envpanel');
  $('eback').onclick = () => envSelect(null);
  $('edel').onclick = envDelete;
  el.querySelectorAll('input[data-k]').forEach((inp) => inp.addEventListener('change', () => {
    const v = +inp.value; if (!isFinite(v)) return;
    const k = inp.dataset.k;
    o[k] = (it.sec === 'structures' && ENV.types.types[o.type][k] && Number.isInteger(ENV.types.types[o.type][k].default)) ? Math.round(v) : +v.toFixed(3);
    if (k === 'heading_deg') o[k] = +wrapDeg(v).toFixed(1);
    envChanged();
  }));
  el.querySelectorAll('input[data-b]').forEach((inp) => inp.addEventListener('change', () => { o[inp.dataset.b] = inp.checked; envChanged(); }));
  el.querySelectorAll('select[data-s]').forEach((sel) => sel.addEventListener('change', () => { o[sel.dataset.s] = sel.value; envChanged(); }));
  if ($('epts')) $('epts').onclick = () => envSetEditPts(!ENV.editPts);
  if ($('erev')) $('erev').onclick = () => { o.points.reverse(); if (it.sec === 'canals') { const s = o.open_start; o.open_start = o.open_end; o.open_end = s; } envChanged(); };
  const dial = $('edial');
  if (dial) {
    let dragging = false;
    const set = (e) => { const r = dial.getBoundingClientRect(), x = e.clientX - r.left - r.width / 2, y = e.clientY - r.top - r.height / 2;
      let hd = deg(Math.atan2(-y, x)); if (e.shiftKey) hd = Math.round(hd / 15) * 15; o.heading_deg = +wrapDeg(hd).toFixed(1); envChanged(true); };
    dial.addEventListener('pointerdown', (e) => { dragging = true; try { dial.setPointerCapture(e.pointerId); } catch (_) {} set(e); });
    dial.addEventListener('pointermove', (e) => { if (dragging) set(e); });
    dial.addEventListener('pointerup', () => { dragging = false; envChanged(); });
  }
}
function envPanelValues() {          // refresh the numbers without rebuilding (keeps focus while dragging)
  const it = ENV.sel && envFind(ENV.sel), el = $('envpanel'); if (!it || !el) return;
  el.querySelectorAll('input[data-k]').forEach((inp) => { if (document.activeElement !== inp && it.o[inp.dataset.k] != null) inp.value = it.o[inp.dataset.k]; });
  el.querySelectorAll('input[data-b]').forEach((inp) => { inp.checked = it.o[inp.dataset.b] !== false; });
  el.querySelectorAll('select[data-s]').forEach((s) => { s.value = it.o[s.dataset.s]; });
  if ($('edial')) { drawDial(it.o.heading_deg || 0, 'edial');
    const b = ((90 - (it.o.heading_deg || 0)) % 360 + 360) % 360;
    $('ebearing').innerHTML = `compass <b>${b.toFixed(0)}&deg;</b><br>heading_deg ${(+it.o.heading_deg || 0).toFixed(0)} (0 = east)`; }
}

// ---------------------------------------------------------------- tabs + buttons
function envSetTab(on) {
  ENV.active = on;
  document.querySelectorAll('#tabs button').forEach((b) => b.classList.toggle('on', (b.dataset.tab === 'environment') === on));
  $('scene').style.display = 'none'; $('sel').style.display = 'none'; $('envpanel').style.display = on ? '' : 'none';
  if (on) { if (typeof select === 'function') select(null); envPanel(); envMode(); }
  else { ENV.place = null; ENV.editPts = false; canvas.classList.remove('place', 'wp'); $('mode').style.display = 'none'; refreshPanel(); }
  $('bsave').title = on ? 'Write the environment to its config file' : 'Write the obstacles to config/obstacles.yaml';
  $('bupdate').title = on ? 'Rebuild the changed environment in the simulation' : 'Apply every change to the running simulation';
  draw();
}
async function envUpdate() {
  if (ENV.editPts) envSetEditPts(false); if (ENV.place) envSetPlace(null);
  const bad = ENV.draft.canals.filter((c) => c.points.length < 2).map((c) => `'${c.name}' needs at least 2 points`)
    .concat(ENV.draft.land.filter((l) => l.points.length < 3).map((l) => `'${l.name}' needs at least 3 points`));
  if (bad.length) { bad.forEach((b) => say(b, 'bad')); return; }
  busy(true); say('building the environment in Gazebo ...', '');
  try {
    const r = await post('/env/update', { doc: ENV.draft });
    say(r.message || (r.ok ? 'applied' : 'failed'), r.ok ? 'ok' : 'bad'); (r.warnings || []).forEach((w) => say(w, 'warn'));
    if (r.ok) { ENV.applied = deep(ENV.draft); lastCheckKey = ''; scheduleCheck(); envCall('vmapLoad'); }   // obstacle checks now see the new banks
  } catch (e) { say('environment update failed: ' + e, 'bad'); }
  busy(false); refreshBadges(); envPanel();
}
async function envSave() {
  busy(true);
  try {
    const r = await post('/env/save', { doc: ENV.draft }); say(r.message, r.ok ? 'ok' : 'bad');
    if (r.ok) { ENV.saved = deep(ENV.draft); if (!envSame(ENV.draft, ENV.applied)) say('saved to the file but NOT applied to the sim -- press Update', 'warn'); }
  } catch (e) { say('save failed: ' + e, 'bad'); }
  busy(false); refreshBadges();
}
function envClear() { ENV.draft = deep(ENV.applied); ENV.sel = null; ENV.editPts = false; ENV.place = null; say('environment changes cleared', 'ok'); envChanged(); }

async function envInit() {
  try { ENV.types = await (await fetch('/env/types')).json(); } catch (e) { return; }
  for (let k = 0; k < 60; k++) {
    try {
      const j = await (await fetch('/env/config')).json();
      if (j.ok) {
        ENV.applied = j.doc; ENV.draft = deep(j.doc); ENV.saved = deep(j.doc); ENV.file = j.config_file;
        say(`loaded the environment (${j.config_file.split('/').pop()})`, 'ok');
        envFetchGeometry(); envScheduleCheck(); envPanel(); refreshBadges(); draw(); return;
      }
    } catch (e) { /* not up yet */ }
    await new Promise((res) => setTimeout(res, 2000));
  }
  say('no environment: sim_environment is not running (moving_obstacles.launch.py environment:=...)', 'warn');
}
document.querySelectorAll('#tabs button').forEach((b) => b.onclick = () => envSetTab(b.dataset.tab === 'environment'));
envInit();
