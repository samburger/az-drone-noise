// Validates the noise model, flight profile, schedule and receiver analysis
// against FAA Appendix E measurements. Run: node scripts/validate.js
'use strict';
global.window = {};
require('../data/homes.js');
const Noise = require('../noise.js');
const Sim = require('../sim.js');

const HOMES = global.window.HOMES;
const P = Object.assign({}, Sim.DEFAULTS);
const FT = Sim.FT, KT = Sim.KT;
const R = 6371008.8;

let fails = 0;
function row(name, ok, expected, got) {
  if (!ok) fails++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name.padEnd(42)} expected ${String(expected).padEnd(18)} got ${got}`);
}
const f1 = (x) => (Number.isFinite(x) ? x.toFixed(1) : String(x));
function near(name, got, exp, tol) {
  row(name, Math.abs(got - exp) <= tol, `${exp} ± ${tol}`, f1(got));
}

// En route
const v = P.cruiseSpeedKt * KT;
near('En route SEL 200 ft overhead', Noise.passMetrics(200 * FT, v, 0, P).sel, 63.7, 1.0);
near('En route SEL 345 ft overhead', Noise.passMetrics(345 * FT, v, 0, P).sel, 60.9, 1.0);
near('En route Lmax 122 ft alt, 17.5 ft lateral', Noise.passMetrics(122 * FT, v, 17.5 * FT, P).lmax, 60.1, 2.0);

// Flight replay helpers
const flight = Sim.buildFlight(0, 0, 3000, 0, 0, P);
const phase = (n) => flight.phases.find((ph) => ph.name === n);
function replay(t0, t1, mx, my) {
  let lmax = -Infinity, e = 0;
  const n = Math.round((t1 - t0) / 0.1);
  for (let i = 0; i < n; i++) {
    const st = Sim.stateAt(flight, t0 + i * 0.1);
    if (!st) continue;
    const L = Noise.levelAt(st.x, st.y, st.z, st.f, mx, my, P);
    if (L > lmax) lmax = L;
    e += Math.pow(10, L / 10) * 0.1;
  }
  return { lmax, sel: 10 * Math.log10(e) };
}

// Takeoff
const tkEnd = phase('climbOut').t1 + 150;
const takeoff = [
  ['under', 20, 81.8, 90.5], ['under', 75, 70.6, 81.5], ['under', 195, 64.7, 76.5],
  ['under', 372, 63.6, 72.4], ['under', 772, 58.6, 67.0],
  ['behind', 79, 69.9, 80.0], ['behind', 176, 61.4, 73.5], ['behind', 375, 54.0, 67.5],
  ['lateral', 76, 70.6, 82.0], ['lateral', 175, 62.9, 75.5], ['lateral', 375, 55.6, 68.2],
];
for (const [pos, d, lm, sel] of takeoff) {
  const dm = d * FT;
  const [mx, my] = pos === 'under' ? [dm, 0] : pos === 'behind' ? [-dm, 0] : [0, dm];
  const r = replay(0, tkEnd, mx, my);
  near(`Takeoff ${pos} ${d} ft Lmax`, r.lmax, lm, 2.5);
  near(`Takeoff ${pos} ${d} ft SEL`, r.sel, sel, 2.5);
}

// Delivery
const dStart = phase('approach').t0 - 60, dEnd = phase('climbIn').t1 + 60;
const delivery = [
  ['lateral', 25, 82.8], ['lateral', 76, 71.3], ['lateral', 175, 63.7], ['lateral', 375, 55.6],
  ['behind', 19, 82.5], ['behind', 79, 69.0], ['behind', 176, 62.0], ['behind', 375, 55.4],
];
for (const [pos, d, lm] of delivery) {
  const dm = d * FT;
  const [mx, my] = pos === 'lateral' ? [3000, dm] : [3000 + dm, 0];
  near(`Delivery ${pos} ${d} ft Lmax`, replay(dStart, dEnd, mx, my).lmax, lm, 3.5);
}

// Schedule
const [lat0, lon0] = HOMES.origin;
const cosLat0 = Math.cos(lat0 * Math.PI / 180);
const homesXY = new Float64Array(HOMES.count * 2);
for (let i = 0; i < HOMES.count; i++) {
  const dlat = HOMES.d[2 * i] / HOMES.scale, dlon = HOMES.d[2 * i + 1] / HOMES.scale;
  homesXY[2 * i] = R * cosLat0 * dlon * Math.PI / 180;
  homesXY[2 * i + 1] = R * dlat * Math.PI / 180;
}
const homesW = new Float64Array(HOMES.count);
for (let i = 0; i < HOMES.count; i++) homesW[i] = HOMES.w[i] / HOMES.wScale;
const sched = Sim.buildSchedule(P, homesXY, homesW);
if (sched.error) {
  row('Schedule builds', false, 'no error', sched.error);
} else {
  row('Schedule flight count', sched.flights.length === 1000, 1000, sched.flights.length);
  const early = sched.flights.filter((f) => f.launch < 21600).length;
  row('All launches >= 21600', early === 0, '0 violations', early);
  const late = sched.flights.filter((f) => f.launch + f.duration > 81000).length;
  row('All landings <= 81000', late === 0, '0 violations', late);
  row('Peak airborne in [5,40]', sched.peakAirborne >= 5 && sched.peakAirborne <= 40, '5..40', sched.peakAirborne);
  row('Residents in radius in [150k,450k]', sched.residents >= 150000 && sched.residents <= 450000,
    '150000..450000', Math.round(sched.residents));
  const s2 = Sim.buildSchedule(P, homesXY, homesW);
  const same = !s2.error && s2.flights.length === sched.flights.length &&
    s2.flights.every((f, i) => f.launch === sched.flights[i].launch && f.homeIdx === sched.flights[i].homeIdx);
  row('Same seed reproducible', same, 'identical', same ? 'identical' : 'differs');

  // Receiver
  const near300 = Sim.analyzeReceiver(sched, 300, 0, P).stats;
  row('Receiver (300,0) eventsAudible > 0', near300.eventsAudible > 0, '> 0', near300.eventsAudible);
  row('Receiver (300,0) n55 > 0', near300.n55 > 0, '> 0', near300.n55);
  row('Receiver (300,0) dnl finite', Number.isFinite(near300.dnl), 'finite', f1(near300.dnl));
  const far = Sim.analyzeReceiver(sched, 20000, 0, P);
  row('Receiver (20000,0) 0 events', far.events.length === 0, 0, far.events.length);
  row('Receiver (20000,0) leqOps -Infinity', far.stats.leqOps === -Infinity, '-Infinity', far.stats.leqOps);
}

// Weighted destination sampling (synthetic: weights 1:3:0, zero-weight home must never be picked)
{
  const sw = Sim.buildSchedule(Object.assign({}, P, { deliveriesPerDay: 4000 }),
    Float64Array.of(1000, 0, 0, 1000, 0, -2000), Float64Array.of(1, 3, 0));
  const share0 = sw.flights.filter((f) => f.homeIdx === 0).length / sw.flights.length;
  const n2 = sw.flights.filter((f) => f.homeIdx === 2).length;
  row('Weighted pick share, weight 1 of 4', Math.abs(share0 - 0.25) <= 0.03, '0.25 ± 0.03', share0.toFixed(3));
  row('Zero-weight home never picked', n2 === 0, 0, n2);
}

console.log(fails ? `\n${fails} check(s) FAILED` : '\nAll checks passed');
process.exit(fails ? 1 : 0);
