/* Passing-lane alert logic. Pure JS, no DOM. Works in browser (window.PassingLogic) and Node (module.exports). */
(function (root) {
  'use strict';
  var LAT0 = 36.3, KX = Math.cos(LAT0 * Math.PI / 180) * 111320, KY = 110574;
  var MILE = 1609.344;
  var CFG = {
    lateralMax: 40,        // m: must be within this of the road corridor
    bearingTol: 35,        // deg: heading vs road direction
    warnMax: 1650,         // m: advance warning window (start of lane ahead)
    warnMin: 300,          // m: too close for an advance warning -> wait for "starts now"
    startAt: 30,           // m: "starts now" when this close to the start
    lateJoin: 150,         // m: joined road more than this past the start -> no "starts now"
    endWarn: 402,          // m: quarter mile before end
    shortLane: 500,        // m: lanes shorter than this get one combined call
    minSpeedForGpsHeading: 2, // m/s
    minMoveForHeading: 12  // m between fixes to compute heading ourselves
  };
  function xy(lon, lat) { return [(lon + 92.4) * KX, (lat - LAT0) * KY]; }
  function bearingDeg(dx, dy) { var b = Math.atan2(dx, dy) * 180 / Math.PI; return (b + 360) % 360; }
  function angDiff(a, b) { var d = Math.abs(a - b) % 360; return d > 180 ? 360 - d : d; }

  function prepare(geojson) {
    return geojson.features.map(function (f) {
      var p = f.properties, pts = p.corridor.map(function (c) { return xy(c[0], c[1]); });
      var cum = [0];
      for (var i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
      var g = f.geometry.coordinates, a = xy(g[0][0], g[0][1]), b = xy(g[g.length - 1][0], g[g.length - 1][1]);
      return { id: p.id, route: p.route, county: p.county, length: p.length_m, startOff: p.startOff, endOff: p.endOff,
               pts: pts, cum: cum, ends: [a, b] };
    });
  }

  // nearest point on corridor polyline: {d, s, brg}
  function project(seg, P) {
    var best = { d: Infinity, s: 0, brg: 0 }, pts = seg.pts;
    for (var i = 1; i < pts.length; i++) {
      var A = pts[i - 1], B = pts[i], dx = B[0] - A[0], dy = B[1] - A[1], L2 = dx * dx + dy * dy;
      if (L2 === 0) continue;
      var t = ((P[0] - A[0]) * dx + (P[1] - A[1]) * dy) / L2; t = t < 0 ? 0 : t > 1 ? 1 : t;
      var qx = A[0] + t * dx, qy = A[1] + t * dy, d = Math.hypot(P[0] - qx, P[1] - qy);
      if (d < best.d) best = { d: d, s: seg.cum[i - 1] + t * Math.sqrt(L2), brg: bearingDeg(dx, dy) };
    }
    return best;
  }

  function Tracker(geojson, opts) {
    this.segs = prepare(geojson);
    this.cfg = Object.assign({}, CFG, opts || {});
    this.reset();
  }
  Tracker.prototype.reset = function () {
    this.state = {}; this.last = null; this.heading = null; this.log = [];
  };
  Tracker.prototype._heading = function (fix, P) {
    var c = this.cfg, moved = this.last ? Math.hypot(P[0] - this.last[0], P[1] - this.last[1]) : 0;
    var gpsOk = fix.heading != null && !isNaN(fix.heading) && fix.heading >= 0 &&
                (fix.speed == null || isNaN(fix.speed) || fix.speed >= c.minSpeedForGpsHeading);
    if (gpsOk) this.heading = fix.heading;
    else if (this.last && moved >= c.minMoveForHeading) this.heading = bearingDeg(P[0] - this.last[0], P[1] - this.last[1]);
    if (!this.last || moved >= c.minMoveForHeading) this.last = P;
    return this.heading;
  };

  // returns {events:[{type,text,seg}], banner:{...}}
  Tracker.prototype.update = function (fix) {
    var c = this.cfg, P = xy(fix.lon, fix.lat), h = this._heading(fix, P), events = [], ahead = null, inLane = null;
    for (var i = 0; i < this.segs.length; i++) {
      var seg = this.segs[i];
      if (h == null) break;
      var pr = project(seg, P);
      if (pr.d > c.lateralMax) continue;
      var diff = angDiff(h, pr.brg), toStart, toEnd;
      if (diff <= c.bearingTol) { toStart = seg.startOff - pr.s; toEnd = seg.endOff - pr.s; }
      else if (diff >= 180 - c.bearingTol) { toStart = pr.s - seg.endOff; toEnd = pr.s - seg.startOff; }
      else continue;
      var st = this.state[seg.id] || (this.state[seg.id] = { warned: false, started: false, ending: false });
      var short = seg.length < c.shortLane;
      if (!st.warned && toStart > c.warnMin && toStart <= c.warnMax) {
        st.warned = true;
        events.push({ type: 'warn', seg: seg.id, text: toStart >= 1200 ? 'Passing lane in one mile' : 'Passing lane in half a mile' });
      }
      if (!st.started && toStart <= c.startAt && toEnd > 0) {
        st.started = true; st.warned = true;
        if (toStart >= -c.lateJoin) events.push({ type: 'start', seg: seg.id, text: short ? 'Short passing lane starts now' : 'Passing lane starts now' });
        if (short) st.ending = true;
      }
      if (st.started && !st.ending && toEnd <= c.endWarn && toEnd > 0) {
        st.ending = true;
        events.push({ type: 'end', seg: seg.id, text: 'Passing lane ends in a quarter mile' });
      }
      if (toStart > 0 && (!ahead || toStart < ahead.dist)) ahead = { seg: seg, dist: toStart };
      if (toStart <= 0 && toEnd > 0) inLane = { seg: seg, remaining: toEnd };
    }
    // fallback: straight-line distance to nearest lane end point
    var nearest = null;
    for (var j = 0; j < this.segs.length; j++) for (var k = 0; k < 2; k++) {
      var e = this.segs[j].ends[k], d = Math.hypot(e[0] - P[0], e[1] - P[1]);
      if (!nearest || d < nearest.dist) nearest = { seg: this.segs[j], dist: d };
    }
    for (var m = 0; m < events.length; m++) this.log.push(Object.assign({ t: fix.t }, events[m]));
    return { events: events, heading: h, banner: { inLane: inLane, ahead: ahead, nearest: nearest } };
  };

  var api = { Tracker: Tracker, xy: xy, MILE: MILE, CFG: CFG, angDiff: angDiff };
  if (typeof module !== 'undefined' && module.exports) module.exports = api; else root.PassingLogic = api;
})(this);
