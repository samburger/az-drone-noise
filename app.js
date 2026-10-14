// App wiring for the MK30 drone noise simulator (plain script, no modules).
(function () {
  'use strict';

  var Noise = window.Noise;
  var Sim = window.Sim;
  var KT = Sim.KT, FT = Sim.FT;

  // ---------------------------------------------------------------- DOM refs
  var $ = function (id) { return document.getElementById(id); };
  var elLocation = $('locationPanel');
  var elSettings = $('settings');
  var elMapWrap = $('mapWrap');
  var homesCanvas = $('homesCanvas');
  var fxCanvas = $('fxCanvas');
  var playBtn = $('playBtn');
  var speedSel = $('speedSel');
  var scrub = $('scrub');
  var elClock = $('clock');
  var elAirborne = $('airborne');
  var chart = $('chart');
  var elError = $('error');

  // ------------------------------------------------------------ coordinates
  var PAD = [41.4140625, -72.8474375];
  var R_EARTH = 6371008.8;
  var DEG = Math.PI / 180;
  var COS0 = Math.cos(PAD[0] * DEG);

  function toXY(lat, lon) {
    return [R_EARTH * COS0 * (lon - PAD[1]) * DEG, R_EARTH * (lat - PAD[0]) * DEG];
  }
  function toLatLng(x, y) {
    return [PAD[0] + y / (R_EARTH * DEG), PAD[1] + x / (R_EARTH * COS0 * DEG)];
  }

  // Thresholds / colours shared by bubbles, chips and chart.
  var COL_AMB = '#f6c343', COL_55 = '#f28c28', COL_65 = '#d7263d', COL_LOW = '#b8c2cc';
  function levelColor(db, ambient) {
    if (!isFinite(db) || db < ambient) return COL_LOW;
    if (db >= 65) return COL_65;
    if (db >= 55) return COL_55;
    return COL_AMB;
  }

  // ------------------------------------------------------------------ homes
  var HOMES = window.HOMES;
  var homesError = (typeof HOMES === 'undefined' || !HOMES) ? 'data/homes.js not found — run: uv run scripts/bake_homes.py'
    : !HOMES.w ? 'data/homes.js is out of date (no population weights) — run: uv run scripts/bake_homes.py' : null;
  var homesMissing = homesError !== null;
  var nHomes = homesMissing ? 0 : HOMES.count;
  var homesXY = new Float64Array(nHomes * 2);
  var homesLL = new Float64Array(nHomes * 2);
  var homesMerc = new Float64Array(nHomes * 2); // normalised Web Mercator [0,1]
  var homesDist = new Float64Array(nHomes);
  var homesW = new Float64Array(nHomes); // residents per home (2020 Census, est.)
  (function initHomes() {
    if (homesMissing) return;
    var d = HOMES.d, o = HOMES.origin, sc = HOMES.scale;
    for (var i = 0; i < nHomes; i++) {
      homesW[i] = HOMES.w[i] / HOMES.wScale;
      var lat = o[0] + d[2 * i] / sc;
      var lon = o[1] + d[2 * i + 1] / sc;
      homesLL[2 * i] = lat;
      homesLL[2 * i + 1] = lon;
      var xy = toXY(lat, lon);
      homesXY[2 * i] = xy[0];
      homesXY[2 * i + 1] = xy[1];
      homesDist[i] = Math.hypot(xy[0], xy[1]);
      homesMerc[2 * i] = (lon + 180) / 360;
      homesMerc[2 * i + 1] = 0.5 - Math.log(Math.tan(Math.PI / 4 + lat * DEG / 2)) / (2 * Math.PI);
    }
  })();

  // ----------------------------------------------------------------- PARAMS
  var PARAMS = [
    { key: 'deliveriesPerDay', group: 'Operations', label: 'Deliveries per day', unit: '', min: 10, max: 2000, step: 10, def: 1000, note: 'Amazon application', type: 'range' },
    { key: 'opsStart', group: 'Operations', label: 'Ops start', unit: '', def: '06:00', note: 'application', type: 'time' },
    { key: 'opsEnd', group: 'Operations', label: 'Ops end', unit: '', def: '22:30', note: 'application', type: 'time' },
    { key: 'serviceRadiusMi', group: 'Operations', label: 'Service radius', unit: 'mi', min: 1, max: 7.5, step: 0.5, def: 7.5, note: 'MK30 max range', type: 'range' },
    { key: 'outboundAltFt', group: 'Flight', label: 'Outbound cruise altitude', unit: 'ft', min: 150, max: 400, step: 5, def: 200, note: 'FAA', type: 'range' },
    { key: 'inboundAltFt', group: 'Flight', label: 'Return cruise altitude', unit: 'ft', min: 150, max: 400, step: 5, def: 345, note: 'FAA', type: 'range' },
    { key: 'cruiseSpeedKt', group: 'Flight', label: 'Cruise speed', unit: 'kt', min: 30, max: 64, step: 0.1, def: 58.3, note: 'FAA; 64 kt = 73 mph max', type: 'range' },
    { key: 'deliveryHoverS', group: 'Flight', label: 'Drop hover time', unit: 's', min: 2, max: 60, step: 1, def: 4, note: 'FAA', type: 'range' },
    { key: 'cruiseL100', group: 'Noise source', label: 'Cruise noise, Lmax @100 ft', unit: 'dBA', min: 45, max: 85, step: 0.1, def: 62.2, note: 'calibrated to FAA SEL 63.7 @200 ft', type: 'range' },
    { key: 'hoverL100', group: 'Noise source', label: 'Hover/vertical noise, Lmax @100 ft', unit: 'dBA', min: 50, max: 90, step: 0.1, def: 68.0, note: 'calibrated to FAA takeoff/delivery mics', type: 'range' },
    { key: 'absorptionDbPerKm', group: 'Environment', label: 'Air absorption', unit: 'dB/km', min: 0, max: 15, step: 0.5, def: 5, note: '—', type: 'range' },
    { key: 'groundG', group: 'Environment', label: 'Ground', unit: '', def: 1, note: 'ISO 9613-2 ground factor', type: 'select',
      options: [[1, 'Soft lawns'], [0.5, 'Mixed'], [0, 'Hard/paved']] },
    { key: 'ambientDb', group: 'Environment', label: 'Background ambient', unit: 'dBA', min: 30, max: 65, step: 1, def: 45, note: 'quiet suburban daytime; Pendleton field ambient 43–48', type: 'range' },
    { key: 'seed', group: 'Operations', label: 'Random seed', unit: '', def: 1, note: '—', type: 'int' }
  ];
  var GROUPS = ['Operations', 'Flight', 'Noise source', 'Environment'];

  var vals = {};
  var inputs = {}; // key -> array of input elements
  PARAMS.forEach(function (pr) { vals[pr.key] = pr.def; });

  function parseHM(s) {
    var m = /^(\d{1,2}):(\d{2})/.exec(s || '');
    if (!m) return NaN;
    return (+m[1]) * 3600 + (+m[2]) * 60;
  }
  function fmtHM(t) {
    var s = Math.max(0, Math.floor(t));
    var h = Math.floor(s / 3600) % 24, m = Math.floor((s % 3600) / 60);
    return (h < 10 ? '0' : '') + h + ':' + (m < 10 ? '0' : '') + m;
  }

  function buildP() {
    var p = Object.assign({}, Sim.DEFAULTS || {}, Noise.DEFAULTS_SOURCE || {});
    PARAMS.forEach(function (pr) {
      if (pr.type !== 'time') p[pr.key] = vals[pr.key];
    });
    p.opsStartS = parseHM(vals.opsStart);
    p.opsEndS = parseHM(vals.opsEnd);
    return p;
  }

  function el(tag, attrs, text) {
    var e = document.createElement(tag);
    if (attrs) for (var k in attrs) e.setAttribute(k, attrs[k]);
    if (text != null) e.textContent = text;
    return e;
  }

  function renderSettings() {
    elSettings.innerHTML = '';
    GROUPS.forEach(function (g) {
      var det = el('details', { open: '' });
      det.appendChild(el('summary', null, g));
      PARAMS.filter(function (pr) { return pr.group === g; }).forEach(function (pr) {
        var row = el('div', { 'class': 'param' });
        var lab = el('label', { 'for': 'p_' + pr.key }, pr.label + (pr.unit ? ' (' + pr.unit + ')' : ''));
        lab.style.fontWeight = '600';
        row.appendChild(lab);
        var ctrl = el('div');
        ctrl.style.gridColumn = '1 / -1';
        ctrl.style.display = 'flex';
        ctrl.style.gap = '6px';
        ctrl.style.alignItems = 'center';
        var list = [];
        if (pr.type === 'range') {
          var rng = el('input', { type: 'range', min: pr.min, max: pr.max, step: pr.step, id: 'p_' + pr.key });
          rng.style.flex = '1';
          var num = el('input', { type: 'number', min: pr.min, max: pr.max, step: pr.step });
          num.style.width = '5.5em';
          rng.value = num.value = pr.def;
          rng.addEventListener('input', function () {
            num.value = rng.value;
            vals[pr.key] = parseFloat(rng.value);
            scheduleRebuild();
          });
          num.addEventListener('input', function () {
            var v = parseFloat(num.value);
            if (!isFinite(v)) return;
            v = Math.min(pr.max, Math.max(pr.min, v));
            rng.value = v;
            vals[pr.key] = v;
            scheduleRebuild();
          });
          num.addEventListener('change', function () { num.value = vals[pr.key]; });
          ctrl.appendChild(rng);
          ctrl.appendChild(num);
          list.push(rng, num);
        } else if (pr.type === 'time') {
          var ti = el('input', { type: 'time', id: 'p_' + pr.key, step: 60 });
          ti.value = pr.def;
          ti.addEventListener('input', function () {
            if (!isFinite(parseHM(ti.value))) return;
            vals[pr.key] = ti.value;
            scheduleRebuild();
          });
          ctrl.appendChild(ti);
          list.push(ti);
        } else if (pr.type === 'select') {
          var sel = el('select', { id: 'p_' + pr.key });
          pr.options.forEach(function (o) {
            var opt = el('option', { value: o[0] }, o[1] + ' (G = ' + o[0] + ')');
            sel.appendChild(opt);
          });
          sel.value = String(pr.def);
          sel.addEventListener('change', function () {
            vals[pr.key] = parseFloat(sel.value);
            scheduleRebuild();
          });
          ctrl.appendChild(sel);
          list.push(sel);
        } else if (pr.type === 'int') {
          var ni = el('input', { type: 'number', step: 1, id: 'p_' + pr.key });
          ni.style.width = '7em';
          ni.value = pr.def;
          ni.addEventListener('input', function () {
            var v = parseInt(ni.value, 10);
            if (!isFinite(v)) return;
            vals[pr.key] = v;
            scheduleRebuild();
          });
          ni.addEventListener('change', function () { ni.value = vals[pr.key]; });
          ctrl.appendChild(ni);
          list.push(ni);
        }
        row.appendChild(ctrl);
        var note = el('div', { 'class': 'note' }, 'Default ' + pr.def + (pr.unit ? ' ' + pr.unit : '') + ' — ' + pr.note);
        note.style.gridColumn = '1 / -1';
        row.appendChild(note);
        inputs[pr.key] = list;
        det.appendChild(row);
      });
      elSettings.appendChild(det);
    });
  }

  function resetDefaults() {
    PARAMS.forEach(function (pr) {
      vals[pr.key] = pr.def;
      (inputs[pr.key] || []).forEach(function (inp) { inp.value = String(pr.def); });
    });
    clearTimeout(rebuildTimer);
    rebuild();
  }

  // -------------------------------------------------------------- app state
  var sched = null;
  var p = buildP();
  var tSim = 0;
  var playing = false;
  var receiver = null; // {lat, lon, x, y}
  var receiverMarker = null;
  var analysis = null;
  var ripples = new Map();
  var rebuildTimer = null;

  function showError(msg) {
    if (!elError) return;
    elError.textContent = msg;
    elError.style.display = 'block';
  }
  function hideError() {
    if (!elError) return;
    elError.textContent = '';
    elError.style.display = 'none';
  }

  // -------------------------------------------------------------------- map
  var map = L.map('map', { zoomAnimation: false, preferCanvas: true });
  L.tileLayer('https://{s}.tile.openstreetmap.fr/hot/{z}/{x}/{y}.png', {
    subdomains: 'abc',
    maxZoom: 19,
    attribution: '© OpenStreetMap contributors, tiles HOT/OSM France'
  }).addTo(map);
  var serviceCircle = L.circle(PAD, {
    radius: vals.serviceRadiusMi * 1609.344,
    color: '#1f6feb', weight: 1.5, dashArray: '6 6', fill: false, interactive: false
  }).addTo(map);
  L.circleMarker(PAD, {
    radius: 7, color: '#111', weight: 2, fillColor: '#ff9900', fillOpacity: 1, bubblingMouseEvents: false
  }).bindPopup('Amazon BDL3 drone pad (C573+J2)').addTo(map);
  map.fitBounds(L.latLng(PAD).toBounds(vals.serviceRadiusMi * 1609.344 * 2));

  // ---------------------------------------------------------------- canvases
  var dpr = 1;
  var hctx = homesCanvas.getContext('2d');
  var fctx = fxCanvas.getContext('2d');

  function sizeCanvases() {
    dpr = window.devicePixelRatio || 1;
    var s = map.getSize();
    [homesCanvas, fxCanvas].forEach(function (c) {
      c.width = Math.max(1, Math.round(s.x * dpr));
      c.height = Math.max(1, Math.round(s.y * dpr));
      c.style.width = s.x + 'px';
      c.style.height = s.y + 'px';
    });
  }

  function drawHomes() {
    var s = map.getSize();
    hctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    hctx.clearRect(0, 0, s.x, s.y);
    if (!nHomes) return;
    var zoom = map.getZoom();
    var scale = 256 * Math.pow(2, zoom);
    var tl = map.containerPointToLayerPoint([0, 0]).add(map.getPixelOrigin());
    var rad = vals.serviceRadiusMi * 1609.344;
    var w = s.x, h = s.y;
    for (var pass = 0; pass < 2; pass++) {
      var inside = pass === 1;
      hctx.fillStyle = inside ? 'rgba(60,60,60,0.35)' : 'rgba(60,60,60,0.08)';
      for (var i = 0; i < nHomes; i++) {
        if ((homesDist[i] <= rad) !== inside) continue;
        var px = homesMerc[2 * i] * scale - tl.x;
        if (px < -2 || px > w + 2) continue;
        var py = homesMerc[2 * i + 1] * scale - tl.y;
        if (py < -2 || py > h + 2) continue;
        hctx.fillRect(px - 0.75, py - 0.75, 1.5, 1.5);
      }
    }
  }

  map.on('move zoom resize', function (e) {
    if (e.type === 'resize') sizeCanvases();
    drawHomes();
  });

  // ---------------------------------------------------------------- rebuild
  function scheduleRebuild() {
    clearTimeout(rebuildTimer);
    rebuildTimer = setTimeout(function () { rebuildTimer = null; rebuild(); }, 300);
  }

  function rebuild() {
    p = buildP();
    serviceCircle.setRadius(vals.serviceRadiusMi * 1609.344);
    drawHomes();
    if (!homesMissing) {
      var res = Sim.buildSchedule(p, homesXY, homesW);
      if (res.error) {
        showError(res.error);
      } else {
        hideError();
        sched = res;
        ripples.clear();
      }
    }
    if (sched) {
      tSim = Math.min(sched.windowEnd, Math.max(sched.windowStart, tSim));
      scrub.min = sched.windowStart;
      scrub.max = sched.windowEnd;
      scrub.step = 1;
      scrub.value = tSim;
    }
    if (receiver) runAnalysis();
    refreshPanel();
  }

  // ----------------------------------------------------------------- panels
  function fmtDb(v) { return v != null && isFinite(v) ? v.toFixed(1) + ' dBA' : '—'; }
  function fmtNum(v, d) { return v != null && isFinite(v) ? v.toFixed(d) : '—'; }
  function row(label, value) {
    return '<tr><td style="padding:2px 8px 2px 0;color:#444">' + label + '</td><td style="text-align:right;font-weight:600;white-space:nowrap">' + value + '</td></tr>';
  }

  function refreshPanel() {
    if (receiver) renderLocation(); else renderOverview();
  }

  function renderOverview() {
    var v = vals.cruiseSpeedKt * KT;
    var outZ = vals.outboundAltFt * FT, inZ = vals.inboundAltFt * FT;
    var outPass = Noise.passMetrics(outZ, v, 0, p);
    var inPass = Noise.passMetrics(inZ, v, 0, p);
    var outCor = Noise.footprintRadius(outZ, 1, p.ambientDb, p);
    var inCor = Noise.footprintRadius(inZ, 1, p.ambientDb, p);
    var hz = 13 * FT;
    var h50 = Noise.levelAt(0, 0, hz, 0, 50 * FT, 0, p);
    var h100 = Noise.levelAt(0, 0, hz, 0, 100 * FT, 0, p);
    var html = '<h3 style="margin:8px 0 4px">Overview</h3>' +
      '<p style="margin:0 0 6px;font-size:12px;color:#6e7781">Click the map to analyse noise at a location.</p>' +
      '<table style="width:100%;border-collapse:collapse;font-size:13px">' +
      row('Homes in radius', sched ? sched.candidates.length.toLocaleString() : '—') +
      row('Residents in radius (2020 Census, est.)', sched ? Math.round(sched.residents).toLocaleString() : '—') +
      row('Flights in period', sched ? sched.flights.length.toLocaleString() : '—') +
      row('Peak airborne', sched ? String(sched.peakAirborne) : '—') +
      row('Mean one-way distance', sched ? fmtNum(sched.meanDistM / 1000, 2) + ' km' : '—') +
      '</table>' +
      '<h4 style="margin:10px 0 4px">Single pass (ground level, directly under track)</h4>' +
      '<table style="width:100%;border-collapse:collapse;font-size:13px">' +
      '<tr><th colspan="2" style="text-align:left;padding-top:4px">Outbound @ ' + vals.outboundAltFt + ' ft, ' + vals.cruiseSpeedKt + ' kt</th></tr>' +
      row('Lmax', fmtDb(outPass.lmax)) +
      row('SEL', fmtDb(outPass.sel)) +
      row('Audible corridor (≥ ambient)', outCor > 0 ? '±' + Math.round(outCor) + ' m' : 'none') +
      '<tr><th colspan="2" style="text-align:left;padding-top:4px">Return @ ' + vals.inboundAltFt + ' ft, ' + vals.cruiseSpeedKt + ' kt</th></tr>' +
      row('Lmax', fmtDb(inPass.lmax)) +
      row('SEL', fmtDb(inPass.sel)) +
      row('Audible corridor (≥ ambient)', inCor > 0 ? '±' + Math.round(inCor) + ' m' : 'none') +
      '<tr><th colspan="2" style="text-align:left;padding-top:4px">Delivery hover @ 13 ft</th></tr>' +
      row('Level at 50 ft away', fmtDb(h50)) +
      row('Level at 100 ft away', fmtDb(h100)) +
      '</table>';
    elLocation.innerHTML = html;
  }

  function renderLocation() {
    var dist = Math.hypot(receiver.x, receiver.y);
    var inside = dist <= vals.serviceRadiusMi * 1609.344;
    var st = analysis ? analysis.stats : {};
    var html = '<h3 style="margin:8px 0 4px">Location</h3>' +
      '<div style="font-size:12px;color:#6e7781;margin-bottom:6px">' + receiver.lat.toFixed(5) + ', ' + receiver.lon.toFixed(5) + '</div>' +
      '<table style="width:100%;border-collapse:collapse;font-size:13px">' +
      row('Distance to pad', fmtNum(dist / 1000, 2) + ' km') +
      row('Service radius', inside ? 'inside' : 'outside') +
      '</table>' +
      '<div style="margin:10px 0;padding:8px;border:1px solid #ddd;border-radius:6px">' +
      '<div style="font-size:12px;color:#444">Live level now (drones only)</div>' +
      '<div style="display:flex;align-items:center;gap:8px">' +
      '<span id="liveChip" style="display:inline-block;width:16px;height:16px;border-radius:50%;background:' + COL_LOW + '"></span>' +
      '<span id="liveDrone" style="font-size:28px;font-weight:700">—</span></div>' +
      '<div style="font-size:12px;color:#444">With ambient: <b id="liveTotal">—</b></div>' +
      '</div>' +
      '<table style="width:100%;border-collapse:collapse;font-size:13px">' +
      row('Loudest drone event (Lmax)', fmtDb(st.lmaxOverall)) +
      row('Audible events (Lmax ≥ ambient)', fmtNum(st.eventsAudible, 0)) +
      row('Events ≥ 55 dBA', fmtNum(st.n55, 0)) +
      row('Events ≥ 65 dBA', fmtNum(st.n65, 0)) +
      row('Minutes above ambient', fmtNum(st.minutesAboveAmbient, 1)) +
      row('Leq over ops hours (drones only)', fmtDb(st.leqOps)) +
      row('Leq incl. ambient', fmtDb(st.leqOpsWithAmbient)) +
      row('DNL (drones only; FAA significance threshold 65)', fmtDb(st.dnl)) +
      '</table>' +
      '<button id="clearBtn" type="button" style="margin-top:8px">Clear location</button>';
    elLocation.innerHTML = html;
    $('clearBtn').addEventListener('click', clearReceiver);
  }

  // --------------------------------------------------------------- receiver
  function runAnalysis() {
    analysis = sched ? Sim.analyzeReceiver(sched, receiver.x, receiver.y, p) : null;
    renderChartStatic();
  }

  function setReceiver(lat, lon, label) {
    var xy = toXY(lat, lon);
    receiver = { lat: lat, lon: lon, x: xy[0], y: xy[1], label: label || null };
    if (receiverMarker) receiverMarker.setLatLng([lat, lon]);
    else receiverMarker = L.marker([lat, lon], { interactive: false }).addTo(map);
    chart.style.display = 'block';
    runAnalysis();
    renderLocation();
  }

  function clearReceiver() {
    receiver = null;
    analysis = null;
    if (receiverMarker) { map.removeLayer(receiverMarker); receiverMarker = null; }
    chart.style.display = 'none';
    renderOverview();
  }

  map.on('click', function (e) {
    setReceiver(e.latlng.lat, e.latlng.lng);
  });

  // ------------------------------------------------------------------ chart
  var chartBars = document.createElement('canvas');
  var CH_H = 110, CH_MIN = 20, CH_MAX = 95;
  chart.style.width = '100%';
  chart.style.height = CH_H + 'px';
  chart.style.display = 'none';
  chart.style.cursor = 'pointer';

  function chartY(db) { return CH_H - (db - CH_MIN) / (CH_MAX - CH_MIN) * CH_H; }

  function renderChartStatic() {
    var w = chart.clientWidth;
    if (!w) return;
    var cd = window.devicePixelRatio || 1;
    chart.width = chartBars.width = Math.round(w * cd);
    chart.height = chartBars.height = Math.round(CH_H * cd);
    var c = chartBars.getContext('2d');
    c.setTransform(cd, 0, 0, cd, 0, 0);
    c.clearRect(0, 0, w, CH_H);
    c.fillStyle = '#fafbfc';
    c.fillRect(0, 0, w, CH_H);
    if (!analysis || !sched) return;
    var amb = p.ambientDb;
    var energy = analysis.energy;
    var W = energy.length;
    var cols = Math.floor(w);
    for (var i = 0; i < cols; i++) {
      var s0 = Math.floor(i * W / cols), s1 = Math.max(s0 + 1, Math.floor((i + 1) * W / cols));
      var mx = 0;
      for (var s = s0; s < s1 && s < W; s++) if (energy[s] > mx) mx = energy[s];
      if (mx <= 0) continue;
      var db = 10 * Math.log10(mx);
      if (db <= CH_MIN) continue;
      var y = Math.max(0, chartY(db));
      c.fillStyle = levelColor(db, amb);
      c.fillRect(i, y, 1, CH_H - y);
    }
    // Gridlines at 55 / 65.
    c.lineWidth = 1;
    c.strokeStyle = 'rgba(0,0,0,0.25)';
    c.fillStyle = '#6e7781';
    c.font = '10px sans-serif';
    [55, 65].forEach(function (g) {
      var gy = Math.round(chartY(g)) + 0.5;
      c.beginPath(); c.moveTo(0, gy); c.lineTo(w, gy); c.stroke();
      c.fillText(String(g), 2, gy - 2);
    });
    // Dashed ambient line.
    var ay = Math.round(chartY(amb)) + 0.5;
    c.setLineDash([4, 3]);
    c.strokeStyle = '#1f6feb';
    c.beginPath(); c.moveTo(0, ay); c.lineTo(w, ay); c.stroke();
    c.setLineDash([]);
    c.fillStyle = '#1f6feb';
    c.fillText('ambient ' + amb, w - 62, ay - 2);
    // Hour ticks every 3 h.
    c.fillStyle = '#6e7781';
    var ws = sched.windowStart, we = sched.windowEnd;
    for (var hh = Math.ceil(ws / 10800) * 10800; hh <= we; hh += 10800) {
      var hx = (hh - ws) / (we - ws) * w;
      c.fillRect(hx, CH_H - 4, 1, 4);
      c.fillText(fmtHM(hh), Math.min(w - 28, hx + 2), CH_H - 2);
    }
  }

  function drawChartFrame() {
    if (!receiver || !sched || !chart.width) return;
    var cd = chart.width / Math.max(1, chart.clientWidth);
    var c = chart.getContext('2d');
    c.setTransform(1, 0, 0, 1, 0, 0);
    c.clearRect(0, 0, chart.width, chart.height);
    c.drawImage(chartBars, 0, 0);
    var w = chart.width / cd;
    var x = (tSim - sched.windowStart) / (sched.windowEnd - sched.windowStart) * w;
    c.setTransform(cd, 0, 0, cd, 0, 0);
    c.fillStyle = '#111';
    c.fillRect(Math.round(x) - 0.5, 0, 1.5, CH_H);
  }

  chart.addEventListener('click', function (e) {
    if (!sched) return;
    var rect = chart.getBoundingClientRect();
    var frac = Math.min(1, Math.max(0, (e.clientX - rect.left) / rect.width));
    tSim = sched.windowStart + frac * (sched.windowEnd - sched.windowStart);
    scrub.value = tSim;
  });

  window.addEventListener('resize', function () { if (receiver) renderChartStatic(); });

  // --------------------------------------------------------------- playback
  if (speedSel && !speedSel.options.length) {
    [60, 120, 300, 600, 1200, 3600].forEach(function (s) {
      var o = el('option', { value: s }, s + '×');
      if (s === 600) o.selected = true;
      speedSel.appendChild(o);
    });
  }

  function setPlaying(on) {
    playing = on;
    playBtn.textContent = on ? '❚❚' : '▶';
    playBtn.title = on ? 'Pause' : 'Play';
  }

  playBtn.addEventListener('click', function () {
    if (!sched) return;
    if (!playing && tSim >= sched.windowEnd) tSim = sched.windowStart;
    setPlaying(!playing);
  });

  scrub.addEventListener('input', function () {
    if (!sched) return;
    tSim = parseFloat(scrub.value);
  });

  // ------------------------------------------------------------------- fx
  var PHASE_COLOR = {
    takeoff: '#1f6feb', climbOut: '#1f6feb', cruiseOut: '#1f6feb',
    approach: '#e36209', descent: '#e36209', dwell: '#e36209', ascent: '#e36209',
    climbIn: '#6e7781', cruiseIn: '#6e7781', landingApproach: '#6e7781', landing: '#6e7781'
  };
  var RIPPLE_EVERY = 4, RIPPLE_LIFE = 1.5;
  function easeOut(x) { return 1 - Math.pow(1 - x, 3); }

  function activeFlights() {
    var out = [];
    if (!sched) return out;
    var fl = sched.flights;
    for (var i = 0; i < fl.length; i++) {
      var f = fl[i];
      if (f.launch > tSim) break; // sorted by launch
      if (tSim < f.launch + f.duration) out.push(f);
    }
    return out;
  }

  function drawFx(wall, active) {
    var s = map.getSize();
    fctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    fctx.clearRect(0, 0, s.x, s.y);
    var zoom = map.getZoom();
    var amb = p.ambientDb;
    var padPt = map.latLngToContainerPoint(PAD);
    var recPt = receiver ? map.latLngToContainerPoint([receiver.lat, receiver.lon]) : null;
    var items = [];
    var drone = [];
    var levels = [];
    var seen = new Set();

    for (var i = 0; i < active.length; i++) {
      var f = active[i];
      var tRel = tSim - f.launch;
      var st = Sim.stateAt(f, tRel);
      if (!st) continue;
      var ll = toLatLng(st.x, st.y);
      var pt = map.latLngToContainerPoint(ll);
      var mpp = 40075016.686 * Math.cos(ll[0] * DEG) / Math.pow(2, zoom + 8);
      var rAmb = Noise.footprintRadius(st.z, st.f, amb, p) / mpp;
      var r55 = Noise.footprintRadius(st.z, st.f, 55, p) / mpp;
      var r65 = Noise.footprintRadius(st.z, st.f, 65, p) / mpp;
      var inbound = tRel >= f.legSplit;
      items.push({ f: f, st: st, pt: pt, rAmb: rAmb, r55: r55, r65: r65, inbound: inbound });
      seen.add(f.id);

      // Ripple emission on wall-clock time.
      var rp = ripples.get(f.id);
      if (!rp) { rp = { next: wall + Math.random() * 0.5, rings: [] }; ripples.set(f.id, rp); }
      if (wall >= rp.next) {
        rp.rings.push({ bornWall: wall, targetRadiusPx: Math.max(4, rAmb) });
        rp.next = wall + RIPPLE_EVERY;
      }
      rp.rings = rp.rings.filter(function (r) { return wall - r.bornWall < RIPPLE_LIFE; });

      if (receiver) {
        var L = Noise.levelAt(st.x, st.y, st.z, st.f, receiver.x, receiver.y, p);
        levels.push(L);
        if (L >= amb) drone.push(pt);
      }
    }
    ripples.forEach(function (_, id) { if (!seen.has(id)) ripples.delete(id); });

    // Bubbles.
    var it, k;
    for (k = 0; k < items.length; k++) {
      it = items[k];
      fctx.fillStyle = 'rgba(246,195,67,0.10)';
      fctx.beginPath(); fctx.arc(it.pt.x, it.pt.y, Math.max(4, it.rAmb), 0, 2 * Math.PI); fctx.fill();
      if (it.r55 > 0) {
        fctx.fillStyle = 'rgba(242,140,40,0.20)';
        fctx.beginPath(); fctx.arc(it.pt.x, it.pt.y, it.r55, 0, 2 * Math.PI); fctx.fill();
      }
      if (it.r65 > 0) {
        fctx.fillStyle = 'rgba(215,38,61,0.30)';
        fctx.beginPath(); fctx.arc(it.pt.x, it.pt.y, it.r65, 0, 2 * Math.PI); fctx.fill();
      }
    }

    // Routes and target homes.
    fctx.lineWidth = 1;
    fctx.strokeStyle = 'rgba(0,0,0,0.15)';
    for (k = 0; k < items.length; k++) {
      it = items[k];
      var hp = map.latLngToContainerPoint([homesLL[2 * it.f.homeIdx], homesLL[2 * it.f.homeIdx + 1]]);
      fctx.beginPath(); fctx.moveTo(padPt.x, padPt.y); fctx.lineTo(hp.x, hp.y); fctx.stroke();
      fctx.beginPath(); fctx.arc(hp.x, hp.y, 3, 0, 2 * Math.PI); fctx.stroke();
    }

    // Ripples.
    fctx.lineWidth = 1.5;
    for (k = 0; k < items.length; k++) {
      it = items[k];
      var rings = ripples.get(it.f.id).rings;
      for (var j = 0; j < rings.length; j++) {
        var age = (wall - rings[j].bornWall) / RIPPLE_LIFE;
        if (age < 0 || age >= 1) continue;
        fctx.strokeStyle = 'rgba(215,120,40,' + (0.6 * (1 - age)).toFixed(3) + ')';
        fctx.beginPath();
        fctx.arc(it.pt.x, it.pt.y, rings[j].targetRadiusPx * easeOut(age), 0, 2 * Math.PI);
        fctx.stroke();
      }
    }

    // Receiver lines.
    if (recPt) {
      fctx.lineWidth = 1;
      fctx.strokeStyle = 'rgba(215,38,61,0.5)';
      for (k = 0; k < drone.length; k++) {
        fctx.beginPath(); fctx.moveTo(recPt.x, recPt.y); fctx.lineTo(drone[k].x, drone[k].y); fctx.stroke();
      }
    }

    // Drones.
    fctx.lineWidth = 2;
    for (k = 0; k < items.length; k++) {
      it = items[k];
      var col = PHASE_COLOR[it.st.phase] || '#1f6feb';
      var hx = it.inbound ? -it.f.ux : it.f.ux;
      var hy = it.inbound ? -it.f.uy : it.f.uy;
      fctx.strokeStyle = col;
      fctx.beginPath();
      fctx.moveTo(it.pt.x, it.pt.y);
      fctx.lineTo(it.pt.x + hx * 9, it.pt.y - hy * 9);
      fctx.stroke();
      fctx.fillStyle = col;
      fctx.beginPath(); fctx.arc(it.pt.x, it.pt.y, 4, 0, 2 * Math.PI); fctx.fill();
    }

    return levels;
  }

  function updateLive(levels) {
    var dEl = $('liveDrone');
    if (!dEl) return;
    var amb = p.ambientDb;
    var L = Noise.sumDb(levels);
    var tot = 10 * Math.log10((isFinite(L) ? Math.pow(10, L / 10) : 0) + Math.pow(10, amb / 10));
    dEl.textContent = L < 20 ? '< 20 dBA' : fmtDb(L);
    $('liveTotal').textContent = fmtDb(tot);
    $('liveChip').style.background = levelColor(L, amb);
  }

  // ------------------------------------------------------------------ loop
  var lastWall = performance.now();
  function frame(now) {
    var dt = Math.min(0.25, Math.max(0, (now - lastWall) / 1000));
    lastWall = now;
    if (sched) {
      if (playing) {
        tSim += dt * (parseFloat(speedSel.value) || 600);
        if (tSim >= sched.windowEnd) { tSim = sched.windowEnd; setPlaying(false); }
      }
      if (document.activeElement !== scrub) scrub.value = tSim;
    }
    var active = activeFlights();
    var levels = drawFx(now / 1000, active);
    elClock.textContent = sched ? fmtHM(tSim) : '--:--';
    elAirborne.textContent = String(active.length);
    if (receiver) { updateLive(levels); drawChartFrame(); }
    requestAnimationFrame(frame);
  }

  // ------------------------------------------------------------------ start
  renderSettings();
  var resetBtn = $('resetBtn');
  if (!resetBtn) {
    resetBtn = el('button', { id: 'resetBtn', type: 'button' }, 'Reset to FAA/Amazon defaults');
    elSettings.parentNode.insertBefore(resetBtn, elSettings.nextSibling);
  }
  resetBtn.addEventListener('click', resetDefaults);

  setPlaying(false);
  sizeCanvases();
  if (homesMissing) {
    showError(homesError);
    playBtn.disabled = true;
    scrub.disabled = true;
  }
  rebuild();
  if (sched) tSim = sched.windowStart, scrub.value = tSim;
  requestAnimationFrame(frame);
})();
