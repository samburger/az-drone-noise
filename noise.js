// MK30 drone noise propagation model (calibrated to FAA Appendix E measurements).
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.Noise = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';
  var REF_M = 30.48, RX_H = 1.5, MIN_R = 1;
  var DEFAULTS_SOURCE = { cruiseL100: 62.2, hoverL100: 68.0, absorptionDbPerKm: 5, groundG: 1 };

  function levelAt(sx, sy, sz, f, rx, ry, p) {
    var dx = sx - rx, dy = sy - ry, dz = sz - RX_H;
    var r = Math.max(MIN_R, Math.sqrt(dx * dx + dy * dy + dz * dz));
    var L100 = p.hoverL100 + (p.cruiseL100 - p.hoverL100) * f;
    var hm = (sz + RX_H) / 2;
    var Agr = p.groundG * Math.max(0, 4.8 - (2 * hm / r) * (17 + 300 / r));
    return L100 - 20 * Math.log10(r / REF_M) - p.absorptionDbPerKm / 1000 * (r - REF_M) - Agr;
  }

  function sumDb(levels) {
    var e = 0;
    for (var i = 0; i < levels.length; i++) e += Math.pow(10, levels[i] / 10);
    return e > 0 ? 10 * Math.log10(e) : -Infinity;
  }

  function footprintRadius(sz, f, thresholdDb, p) {
    if (levelAt(0, 0, sz, f, 0, 0, p) < thresholdDb) return 0;
    var lo = 0, hi = 20000;
    for (var i = 0; i < 40; i++) {
      var mid = (lo + hi) / 2;
      if (levelAt(0, 0, sz, f, mid, 0, p) >= thresholdDb) lo = mid; else hi = mid;
    }
    return lo;
  }

  function passMetrics(sz, speedMs, lateralM, p) {
    var step = speedMs * 0.1, lmax = -Infinity, e = 0;
    for (var x = -5000; x <= 5000; x += step) {
      var L = levelAt(x, 0, sz, 1, 0, lateralM, p);
      if (L > lmax) lmax = L;
      e += Math.pow(10, L / 10) * 0.1;
    }
    return { lmax: lmax, sel: e > 0 ? 10 * Math.log10(e) : -Infinity };
  }

  return { REF_M: REF_M, RX_H: RX_H, MIN_R: MIN_R, levelAt: levelAt, sumDb: sumDb,
    footprintRadius: footprintRadius, passMetrics: passMetrics, DEFAULTS_SOURCE: DEFAULTS_SOURCE };
});
