'use strict';
// ====================================================================== ROUTE goal tools
// RViz-style goal tools on the SIMU map, for route planner 1 (route_server):
//   SINGLE GOAL mode  press = position, drag = heading (like RViz's 2D Goal Pose),
//                     release = sent at once
//   WAYPOINTS mode    each press-drag-release adds a waypoint (with heading if you
//                     dragged); right-click removes the last; Send sends them all,
//                     the last one is the goal
//   Go to berth       the route ends at a pontoon berth's approach point
//   Cancel            stops the mission
// The planned route is drawn coloured by speed (red slow .. green cruise), with
// its corridor (toggle), its waypoints, and the boat's progress; the status line
// says what the route server is doing -- and WHY when it cannot.

const RG = {
  mode: 'single', tool: false, draft: [], press: null, hover: null,
  route: null, status: null, berths: [], corridor: true,
  p2: null, p2on: true, signals: [],        // route planner 2 (route_two_planner)
};
const RG_STATE_COL = { active: '#1e7d34', arrived: '#1565c0', planning: '#a15c00', replanning: '#a15c00',
  failed: '#c62828', cancelled: '#6b7a8a', idle: '#6b7a8a' };

function rgEl(html) { const d = document.createElement('div'); d.innerHTML = html.trim(); return d.firstChild; }
function rgBuild() {
  const bar = rgEl(`<div id="rgbar" class="float">
    <div class="rgrow"><b>Route</b>
      <div class="rgseg"><button data-m="single" class="on" title="One goal, sent as soon as you place it (like RViz 2D Goal Pose)">Single goal</button>
        <button data-m="waypoints" title="Several waypoints, then Send; the last is the goal">Waypoints</button></div></div>
    <div class="rgrow">
      <button class="btn small" id="rgtool" title="Then press on the map; drag to give a heading">&#127919; Set goal</button>
      <button class="btn small" id="rgsend" style="display:none" title="Send the waypoints (the last is the goal)">Send &#9654;</button>
      <button class="btn small ghost" id="rgundo" style="display:none" title="Remove the last waypoint (or right-click)">Undo</button>
      <button class="btn small ghost" id="rgclear" title="Cancel the mission and clear its plan (and any unsent waypoints) from the map">Clear</button>
      <button class="btn small" id="rgcancel" title="Stop the current mission">&#10005; Cancel</button></div>
    <div class="rgrow"><select id="rgberth" title="Route to a pontoon berth's approach point"><option value="">go to berth...</option></select>
      <label class="switch"><input type="checkbox" id="rgcorr" checked> corridor</label>
      <label class="switch" title="Route planner 2: the path modified for traffic, and why"><input type="checkbox" id="rgp2" checked> planner 2</label></div>
    <div id="rgstat">no route yet</div>
    <div id="rgp2stat" style="display:none"></div></div>`);
  $('mapwrap').appendChild(bar);
  const css = document.createElement('style');
  css.textContent = `#rgbar{top:64px;left:14px;padding:8px 10px;font-size:12.5px;display:flex;flex-direction:column;gap:6px;max-width:380px}
    #rgbar .rgrow{display:flex;align-items:center;gap:6px;flex-wrap:wrap} #rgbar b{font-weight:800;margin-right:4px}
    .rgseg{display:inline-grid;grid-template-columns:1fr 1fr;border:1px solid var(--line2);border-radius:8px;overflow:hidden}
    .rgseg button{border:0;background:#fff;padding:5px 9px;font:inherit;font-size:12.5px;cursor:pointer;color:var(--ink2)}
    .rgseg button+button{border-left:1px solid var(--line2)} .rgseg button.on{background:var(--ink);color:#fff;font-weight:600}
    #rgbar select{font:inherit;font-size:12.5px;padding:4px 6px;border:1px solid var(--line2);border-radius:7px;max-width:200px}
    #rgstat{color:var(--ink2);font-variant-numeric:tabular-nums;line-height:1.35} #rgstat .st{font-weight:800}
    #rgtool.on{background:var(--sel);border-color:var(--sel);color:#fff}`;
  document.head.appendChild(css);
  bar.querySelectorAll('.rgseg button').forEach((b) => b.onclick = () => rgSetMode(b.dataset.m));
  $('rgtool').onclick = () => rgSetTool(!RG.tool);
  $('rgsend').onclick = rgSend;
  $('rgundo').onclick = () => { RG.draft.pop(); rgUpdate(); };
  $('rgclear').onclick = async () => {
    RG.draft = []; rgUpdate();
    const S = RG.status;
    if (S && ['active', 'planning', 'replanning'].includes(S.state)) {   // the plan goes once the cancel lands
      const r = await post('/route1/cancel'); say(r.message, r.ok ? 'ok' : 'bad');
    }
  };
  $('rgcancel').onclick = async () => { const r = await post('/route1/cancel'); say(r.message, r.ok ? 'ok' : 'bad'); };
  $('rgberth').onchange = async () => {
    const b = $('rgberth').value; if (!b) return;
    const r = await post('/route1/goal', { mode: 'single', berth: b }); say(r.message, r.ok ? 'ok' : 'bad'); $('rgberth').value = '';
  };
  $('rgcorr').onchange = () => { RG.corridor = $('rgcorr').checked; draw(); };
  $('rgp2').onchange = () => { RG.p2on = $('rgp2').checked; rgP2Line(); draw(); };
}
function rgSetMode(m) {
  RG.mode = m; RG.draft = [];
  document.querySelectorAll('#rgbar .rgseg button').forEach((b) => b.classList.toggle('on', b.dataset.m === m));
  $('rgtool').innerHTML = m === 'single' ? '&#127919; Set goal' : '&#10133; Add waypoints';
  $('rgsend').style.display = $('rgundo').style.display = m === 'waypoints' ? '' : 'none';
  rgUpdate();
}
function rgSetTool(on) {
  RG.tool = on; RG.press = null; $('rgtool').classList.toggle('on', on); canvas.classList.toggle('place', on);
  const md = $('mode');
  if (on) { md.innerHTML = RG.mode === 'single' ? 'Click the GOAL on the map &middot; drag to set its heading &middot; Esc cancels'
    : 'Click WAYPOINTS in order &middot; drag for a heading &middot; right-click removes the last &middot; Send when done &middot; Esc stops adding';
    md.style.display = 'block'; } else md.style.display = 'none';
  draw();
}
function rgUpdate() { $('rgsend').disabled = !RG.draft.length; $('rgsend').innerHTML = `Send ${RG.draft.length || ''} &#9654;`; draw(); }
async function rgSend() {
  if (!RG.draft.length) return;
  const r = await post('/route1/goal', { mode: RG.mode, waypoints: RG.draft.map((w) => [w.x, w.y, w.h]) });
  say(r.message, r.ok ? 'ok' : 'bad');
  if (r.ok) { RG.draft = []; rgSetTool(false); rgUpdate(); }
}

// ---------------------------------------------------------------- mouse (called first by index.html)
function rgMouseDown(e, wx, wy) {
  if (!RG.tool) return false;
  if (e.button === 2) { if (RG.mode === 'waypoints') { RG.draft.pop(); rgUpdate(); } return true; }
  if (e.button !== 0) return true;
  RG.press = { x: wx, y: wy, px: e.offsetX, py: e.offsetY }; RG.hover = [wx, wy]; draw(); return true;
}
function rgMouseMove(wx, wy) { if (!RG.tool) return false; RG.hover = [wx, wy]; if (RG.press) draw(); return !!RG.press; }
function rgMouseUp(e) {
  if (!RG.tool || !RG.press) return false;
  const p = RG.press, r = canvas.getBoundingClientRect(), dx = e.clientX - r.left - p.px, dy = e.clientY - r.top - p.py;
  const h = Math.hypot(dx, dy) >= 10 ? Math.atan2(-dy, dx) : null;            // screen y is down
  const wp = { x: +p.x.toFixed(2), y: +p.y.toFixed(2), h: h == null ? null : +h.toFixed(3) };
  RG.press = null;
  if (RG.mode === 'single') {
    post('/route1/goal', { mode: 'single', waypoints: [[wp.x, wp.y, wp.h]] }).then((res) => say(res.message, res.ok ? 'ok' : 'bad'));
    rgSetTool(false);
  } else { RG.draft.push(wp); rgUpdate(); }
  return true;
}
function rgKey(e) {
  if (e.key === 'Escape' && RG.tool) { rgSetTool(false); return true; }
  if (e.key === 'Enter' && RG.tool && RG.mode === 'waypoints') { rgSend(); return true; }
  return false;
}

// ---------------------------------------------------------------- drawing (above everything else)
function rgArrow(x, y, h, col, len) {
  const L = (len || 34) / view.scale, ex = x + L * Math.cos(h), ey = y + L * Math.sin(h);
  path([[x, y], [ex, ey]]); ctx.lineWidth = 2.5; ctx.strokeStyle = col; ctx.stroke(); arrowHead(x, y, ex, ey, col);
}
function rgPin(x, y, label, col, big) {
  const [sx, sy] = w2s(x, y), r = big ? 10 : 8;
  ctx.beginPath(); ctx.arc(sx, sy, r, 0, 2 * Math.PI); ctx.fillStyle = col; ctx.fill(); ctx.lineWidth = 2; ctx.strokeStyle = '#fff'; ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.font = `800 ${big ? 10 : 9.5}px Inter, system-ui`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(label, sx, sy + 0.5);
}
function rgSpeedCol(v, vmax) { const t = Math.max(0, Math.min(1, v / Math.max(vmax, 0.05))); return `hsl(${Math.round(120 * t)},85%,42%)`; }
function rgDraw() {
  const S0 = RG.status;
  // a CANCELLED mission's plan is not drawn (whoever cancelled it; also after a page reload)
  const R = (RG.route && S0 && S0.state === 'cancelled' && S0.route_id === RG.route.route_id) ? null : RG.route;
  if (R && R.ok) {
    const n = R.x.length, vmax = Math.max(...R.v, 0.1);
    if (RG.corridor) {                                        // the room planner 2 may use
      const L = [], Rr = [];
      for (let k = 0; k < n; k++) {
        const a = R.alpha[k]; L.push([R.x[k] - Math.sin(a) * R.l_max[k], R.y[k] + Math.cos(a) * R.l_max[k]]);
        Rr.push([R.x[k] - Math.sin(a) * R.l_min[k], R.y[k] + Math.cos(a) * R.l_min[k]]);
      }
      path(L.concat(Rr.reverse()), true); ctx.fillStyle = 'rgba(26,102,224,.10)'; ctx.fill();
      ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.strokeStyle = 'rgba(26,102,224,.55)'; ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.lineCap = 'round';
    for (let k = 0; k + 1 < n; k++) {                         // the path, coloured by planned speed
      path([[R.x[k], R.y[k]], [R.x[k + 1], R.y[k + 1]]]); ctx.lineWidth = 4.5; ctx.strokeStyle = rgSpeedCol(R.v[k], vmax); ctx.stroke();
    }
    ctx.lineCap = 'butt';
    R.waypoints.forEach((w, i) => {
      const goal = i === R.waypoints.length - 1;
      if (w[2] != null) rgArrow(w[0], w[1], w[2], goal ? '#d84315' : '#f9a825');
      rgPin(w[0], w[1], goal ? 'G' : String(i + 1), goal ? '#d84315' : '#f9a825', goal);
    });
    const S = RG.status;
    if (S && S.route_id === R.route_id && ['active', 'replanning'].includes(S.state)) {   // progress along it
      let k = 0; while (k + 1 < n && R.s[k + 1] <= S.s) k++;
      const [sx, sy] = w2s(R.x[k], R.y[k]);
      ctx.beginPath(); ctx.arc(sx, sy, 6, 0, 2 * Math.PI); ctx.fillStyle = '#fff'; ctx.fill(); ctx.lineWidth = 3; ctx.strokeStyle = '#1e7d34'; ctx.stroke();
    }
  }
  rgDrawP2();
  // the waypoints being placed (not sent yet)
  const start = live.boat ? [live.boat.x, live.boat.y] : null;
  const pts = (start ? [start] : []).concat(RG.draft.map((w) => [w.x, w.y]));
  if (RG.press) pts.push([RG.press.x, RG.press.y]);
  if (pts.length > 1) { path(pts); ctx.setLineDash([7, 6]); ctx.lineWidth = 2; ctx.strokeStyle = 'rgba(245,124,0,.9)'; ctx.stroke(); ctx.setLineDash([]); }
  RG.draft.forEach((w, i) => { if (w.h != null) rgArrow(w.x, w.y, w.h, '#f57c00'); rgPin(w.x, w.y, String(i + 1), '#f57c00'); });
  if (RG.press && RG.hover) {
    const [hx, hy] = RG.hover, dx = hx - RG.press.x, dy = hy - RG.press.y;
    if (Math.hypot(dx, dy) * view.scale >= 10) rgArrow(RG.press.x, RG.press.y, Math.atan2(dy, dx), '#f57c00', Math.hypot(dx, dy) * view.scale);
    rgPin(RG.press.x, RG.press.y, RG.mode === 'single' ? 'G' : String(RG.draft.length + 1), '#f57c00', RG.mode === 'single');
  }
}
const P2_DEC_COL = { give_way: '#c62828', stand_on: '#1565c0', avoid: '#a15c00', monitor: '#6b7a8a' };
function rgDrawP2() {
  const P = RG.p2;
  if (!RG.p2on || !P || !P.active) return;
  if (P.override && live.boat) {                            // a preemptor took over: show where the bow is held
    const b = live.boat, a = b.heading + (P.speed < 0 ? Math.PI : 0);
    rgArrow(b.x, b.y, a, '#c62828', 60);
    return;
  }
  if (!P.path || P.path.length < 2) return;
  path(P.path); ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(142,36,170,.9)'; ctx.setLineDash([10, 5]); ctx.stroke(); ctx.setLineDash([]);
}
function rgP2Line() {
  const el = $('rgp2stat'), P = RG.p2; if (!el) return;
  el.style.display = RG.p2on ? '' : 'none';
  if (!RG.p2on) return;
  if (!P) { el.innerHTML = '<span style="color:var(--muted)">planner 2: not running (ros2 launch route_two_planner route_two.launch.py)</span>'; return; }
  if (!P.active) { el.innerHTML = `<b style="color:#8e24aa">Planner 2</b> idle &middot; ${P.why}`; return; }
  let t = `<b style="color:#8e24aa">Planner 2</b> &middot; <b>${P.owner}</b> &middot; ${P.speed.toFixed(2)} m/s &middot; ` +
    `${P.rulebook} &middot; l ${P.l.toFixed(1)} m${P.override ? ' &middot; <b style="color:#c62828">OVERRIDE</b>' : ''}` +
    `<br><span style="color:var(--muted)">${P.binding}</span>`;
  (P.contacts || []).forEach((c) => {
    t += `<br><span style="color:${P2_DEC_COL[c.decision] || '#333'}">&#9679; #${c.id} ${c.decision.replace('_', ' ')}</span> ` +
      `${c.encounter.replace('_', ' ')} &middot; DCPA ${c.dcpa.toFixed(1)} m${c.tcpa >= 0 ? ' in ' + c.tcpa.toFixed(0) + ' s' : ''}` +
      `${c.rule ? ' &middot; <i>' + c.rule + '</i>' : ''}${c.note ? '<br>&nbsp;&nbsp;' + c.note : ''}`;
  });
  (P.behaviours || []).filter((b) => b.requested && !b.ready && b.not_ready).forEach((b) => {
    t += `<br><span style="color:#c62828">${b.name}: wanted to act and could not</span> &middot; ${b.not_ready}`;
  });
  const REQ_COL = { sent: '#a15c00', accepted: '#1565c0', done: '#1e7d34', rejected: '#c62828', unanswered: '#c62828' };
  (P.requests || []).forEach((r) => {
    t += `<br>&#8679; <b>${r.type}</b> to route 1 &middot; <span style="color:${REQ_COL[r.state] || '#333'}">${r.state}` +
      `${r.action ? ' ' + r.action : ''}</span>${r.reason ? ' &middot; ' + r.reason : ''}`;
  });
  if (RG.signals.length) t += `<br>&#128227; ${RG.signals.slice(-3).join(' &middot; ')}`;
  el.innerHTML = t;
}
function rgStatusLine() {
  const S = RG.status, R = RG.route, el = $('rgstat'); if (!el) return;
  if (!S) { el.textContent = 'route server: no status (ros2 launch route_one_bringup route_one.launch.py)'; return; }
  const col = RG_STATE_COL[S.state] || '#333';
  let txt = `<span class="st" style="color:${col}">${S.state.toUpperCase()}</span>`;
  if (['active', 'replanning'].includes(S.state)) {
    txt += ` &middot; ${S.distance_remaining.toFixed(0)} m to go &middot; ${(S.eta_s / 60).toFixed(1)} min &middot; ${S.piece}` +
      (S.waypoints_total > 1 ? ` &middot; next wp ${S.next_waypoint + 1}/${S.waypoints_total}` : '') +
      ` &middot; ${S.speed_planned.toFixed(2)} m/s planned &middot; off-path ${S.cross_track.toFixed(1)} m`;
  }
  if (S.message && S.state !== 'active') txt += `<br>${S.message}`;
  if (R && R.ok) txt += `<br><span style="color:var(--muted)">route ${R.route_id} (${R.why}): ${R.length_m.toFixed(0)} m, ${(R.time_s / 60).toFixed(1)} min, ` +
    `${R.pieces.map((p) => p.kind === 'canal' ? p.name : 'water').join(' &rsaquo; ')}${R.lead_in ? ' &middot; lead-in' : ''}</span>`;
  el.innerHTML = txt;
}
async function rgLoadBerths() {
  try {
    const d = await (await fetch('/route1/berths')).json(), sel = $('rgberth');
    if (!d.berths || !d.berths.length) return;
    RG.berths = d.berths;
    sel.innerHTML = '<option value="">go to berth...</option>' + d.berths.map((b) => `<option value="${b.name}">${b.name} (${b.width.toFixed(1)} m wide)</option>`).join('');
  } catch (e) { /* later */ }
}
function rgAttach() {
  if (!window.SIMU_ES) { setTimeout(rgAttach, 300); return; }
  window.SIMU_ES.addEventListener('route', (ev) => { try { RG.route = JSON.parse(ev.data); } catch (e) { return; } rgStatusLine(); draw(); });
  window.SIMU_ES.addEventListener('route2', (ev) => {
    try { RG.p2 = JSON.parse(ev.data); } catch (e) { return; }
    const sig = RG.p2.last_signal;
    if (sig && sig !== RG.lastSig) { RG.lastSig = sig; RG.signals.push(sig.replace('|', ' (') + ')'); RG.signals = RG.signals.slice(-6); }
    rgP2Line(); draw();
  });
  window.SIMU_ES.addEventListener('route_status', (ev) => {
    const prev = RG.status && RG.status.state;
    try { RG.status = JSON.parse(ev.data); } catch (e) { return; }
    if (prev && prev !== RG.status.state && ['arrived', 'failed', 'cancelled'].includes(RG.status.state))
      say(`route ${RG.status.state}: ${RG.status.message}`, RG.status.state === 'failed' ? 'bad' : 'ok');
    rgStatusLine(); draw();
  });
}
rgBuild(); rgAttach(); rgLoadBerths(); setInterval(rgLoadBerths, 15000);
