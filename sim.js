// Flight profile (FAA Appendix E Table 1), schedule and receiver analysis.
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./noise.js'));
  else root.Sim = factory(root.Noise);
})(typeof self !== 'undefined' ? self : this, function (Noise) {
  'use strict';
  var KT = 0.514444, FT = 0.3048;

  var DEFAULTS = Object.assign({
    deliveriesPerDay: 1000, opsStartS: 21600, opsEndS: 81000, serviceRadiusMi: 7.5,
    outboundAltFt: 200, inboundAltFt: 345, cruiseSpeedKt: 58.3, deliveryHoverS: 4,
    ambientDb: 45, seed: 1
  }, Noise.DEFAULTS_SOURCE);

  function mulberry32(seed) {
    var a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0;
      var t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function buildFlight(id, homeIdx, hx, hy, launch, p) {
    var D = Math.hypot(hx, hy);
    var ux = D > 0 ? hx / D : 1, uy = D > 0 ? hy / D : 0;
    var v = p.cruiseSpeedKt * KT;
    var k = Math.min(1, D / (35 * v));
    var cruiseLen = Math.max(0, D - 35 * v * k);
    var cruiseT = cruiseLen / v;
    var outAlt = p.outboundAltFt * FT, inAlt = p.inboundAltFt * FT;
    var spec = [
      ['takeoff', 15, 0, 115 * FT],
      ['climbOut', 40, 115 * FT, outAlt],
      ['cruiseOut', cruiseT, outAlt, outAlt],
      ['approach', 30, outAlt, 140 * FT],
      ['descent', 27, 140 * FT, 13 * FT],
      ['dwell', p.deliveryHoverS, 13 * FT, 13 * FT],
      ['ascent', 30, 13 * FT, 197 * FT],
      ['climbIn', 40, 197 * FT, inAlt],
      ['cruiseIn', cruiseT, inAlt, inAlt],
      ['landingApproach', 30, inAlt, 197 * FT],
      ['landing', 35, 197 * FT, 0]
    ];
    var phases = [], t = 0, legSplit = 0;
    for (var i = 0; i < spec.length; i++) {
      var s = spec[i];
      phases.push({ name: s[0], t0: t, t1: t + s[1], z0: s[2], z1: s[3] });
      t += s[1];
      if (s[0] === 'dwell') legSplit = t;
    }
    return { id: id, homeIdx: homeIdx, hx: hx, hy: hy, dist: D, ux: ux, uy: uy, v: v, k: k,
      launch: launch, duration: t, phases: phases, legSplit: legSplit };
  }

  function stateAt(fl, tRel) {
    if (!(tRel >= 0 && tRel < fl.duration)) return null;
    var ph = fl.phases, P = null;
    for (var i = 0; i < ph.length; i++) {
      if (ph[i].t0 <= tRel && tRel < ph[i].t1) { P = ph[i]; break; }
    }
    if (!P) return null;
    var tau = P.t1 > P.t0 ? (tRel - P.t0) / (P.t1 - P.t0) : 0;
    var z = P.z0 + (P.z1 - P.z0) * tau;
    var v = fl.v, k = fl.k, D = fl.dist, s, f;
    switch (P.name) {
      case 'takeoff': s = 0; f = 0; break;
      case 'climbOut': s = 20 * v * k * tau * tau; f = k * tau; break;
      case 'cruiseOut': s = 20 * v * k + v * (tRel - P.t0); f = 1; break;
      case 'approach': s = D - 15 * v * k + 15 * v * k * (2 * tau - tau * tau); f = k * (1 - tau); break;
      case 'descent': case 'dwell': case 'ascent': s = D; f = 0; break;
      case 'climbIn': s = D - 20 * v * k * tau * tau; f = k * tau; break;
      case 'cruiseIn': s = D - 20 * v * k - v * (tRel - P.t0); f = 1; break;
      case 'landingApproach': s = 15 * v * k - 15 * v * k * (2 * tau - tau * tau); f = k * (1 - tau); break;
      default: s = 0; f = 0;
    }
    return { x: fl.ux * s, y: fl.uy * s, z: z, f: f, phase: P.name };
  }

  function buildSchedule(p, homesXY, homesW) {
    var W = p.opsEndS - p.opsStartS;
    if (W < 3600) return { error: 'Operating window must be at least 1 hour' };
    var rMax = p.serviceRadiusMi * 1609.344, cand = [], cum = [], total = 0;
    for (var i = 0; i < homesXY.length / 2; i++) {
      var d = Math.hypot(homesXY[2 * i], homesXY[2 * i + 1]);
      if (d >= 50 && d <= rMax && homesW[i] > 0) { cand.push(i); total += homesW[i]; cum.push(total); }
    }
    if (!cand.length) return { error: 'No homes inside the service radius' };
    var rng = mulberry32(p.seed), flights = [], distSum = 0;
    for (var j = 0; j < p.deliveriesPerDay; j++) {
      var u = rng() * total, lo = 0, hi = cand.length - 1;
      while (lo < hi) { var mid = (lo + hi) >> 1; if (cum[mid] > u) hi = mid; else lo = mid + 1; }
      var idx = cand[lo];
      var hx = homesXY[2 * idx], hy = homesXY[2 * idx + 1];
      var fl = buildFlight(0, idx, hx, hy, 0, p);
      fl.launch = p.opsStartS + rng() * Math.max(0, W - fl.duration);
      flights.push(fl);
      distSum += fl.dist;
    }
    flights.sort(function (a, b) { return a.launch - b.launch; });
    var ev = [];
    for (var m = 0; m < flights.length; m++) {
      flights[m].id = m;
      ev.push([flights[m].launch, 1], [flights[m].launch + flights[m].duration, -1]);
    }
    ev.sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
    var cur = 0, peak = 0;
    for (var q = 0; q < ev.length; q++) { cur += ev[q][1]; if (cur > peak) peak = cur; }
    return { flights: flights, windowStart: p.opsStartS, windowEnd: p.opsEndS, candidates: cand,
      residents: total, peakAirborne: peak, meanDistM: flights.length ? distSum / flights.length : 0 };
  }

  function segDist(px, py, bx, by) {
    var L2 = bx * bx + by * by;
    var t = L2 > 0 ? Math.max(0, Math.min(1, (px * bx + py * by) / L2)) : 0;
    return Math.hypot(px - bx * t, py - by * t);
  }

  function toDb(e) { return e > 0 ? 10 * Math.log10(e) : -Infinity; }

  function analyzeReceiver(sched, rx, ry, p) {
    var ws = sched.windowStart, W = sched.windowEnd - ws;
    var energy = new Float64Array(W);
    var Rcut = Math.min(5000, Noise.footprintRadius(40, 0, 20, p));
    var events = [];
    for (var i = 0; i < sched.flights.length; i++) {
      var fl = sched.flights[i];
      if (segDist(rx, ry, fl.hx, fl.hy) > Rcut) continue;
      var legs = { out: { lmax: -Infinity, e: 0, tPeak: 0 }, in: { lmax: -Infinity, e: 0, tPeak: 0 } };
      var end = fl.launch + fl.duration;
      for (var s = Math.ceil(fl.launch); s < end; s++) {
        var tRel = s - fl.launch, st = stateAt(fl, tRel);
        if (!st) continue;
        var L = Noise.levelAt(st.x, st.y, st.z, st.f, rx, ry, p);
        var e = Math.pow(10, L / 10);
        var bi = s - ws;
        if (bi >= 0 && bi < W) energy[bi] += e;
        var g = tRel < fl.legSplit ? legs.out : legs.in;
        g.e += e;
        if (L > g.lmax) { g.lmax = L; g.tPeak = s; }
      }
      ['out', 'in'].forEach(function (name) {
        var g = legs[name];
        if (g.lmax >= 20) events.push({ flightId: fl.id, leg: name, lmax: g.lmax, sel: toDb(g.e), tPeak: g.tPeak });
      });
    }
    var lmaxOverall = null, aud = 0, n55 = 0, n65 = 0;
    for (var j = 0; j < events.length; j++) {
      var lm = events[j].lmax;
      if (lmaxOverall === null || lm > lmaxOverall) lmaxOverall = lm;
      if (lm >= p.ambientDb) aud++;
      if (lm >= 55) n55++;
      if (lm >= 65) n65++;
    }
    var ambE = Math.pow(10, p.ambientDb / 10), above = 0, sum = 0, wsum = 0;
    for (var k = 0; k < W; k++) {
      var en = energy[k];
      if (en <= 0) continue;
      if (en >= ambE) above++;
      sum += en;
      var sod = ((ws + k) % 86400 + 86400) % 86400;
      wsum += (sod < 25200 || sod >= 79200 ? 10 : 1) * en;
    }
    var leqOps = toDb(sum / W);
    return { energy: energy, events: events, stats: {
      lmaxOverall: lmaxOverall, eventsAudible: aud, n55: n55, n65: n65,
      minutesAboveAmbient: above / 60, leqOps: leqOps,
      leqOpsWithAmbient: 10 * Math.log10((leqOps === -Infinity ? 0 : Math.pow(10, leqOps / 10)) + ambE),
      dnl: toDb(wsum / 86400) } };
  }

  return { KT: KT, FT: FT, DEFAULTS: DEFAULTS, mulberry32: mulberry32, buildFlight: buildFlight,
    stateAt: stateAt, buildSchedule: buildSchedule, analyzeReceiver: analyzeReceiver };
});
