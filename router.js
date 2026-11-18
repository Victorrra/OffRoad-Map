/* OffRoad Maps — offline ATV router
 * Reads bulgaria-routing.bin (built by map/build-routing.py) and finds routes with A*,
 * entirely on the device. Exposes window.OffroadRouter.
 */
(function(global){
  'use strict';

  // Road classes (must match map/build-routing.py): speed km/h, group for instructions
  var CLASSES = {
    1:  { kmh: 30, group: 'track' },   // track grade1
    2:  { kmh: 25, group: 'track' },
    3:  { kmh: 18, group: 'track' },
    4:  { kmh: 12, group: 'track' },
    5:  { kmh: 8,  group: 'track' },   // grade5 — rough terrain
    6:  { kmh: 18, group: 'track' },   // track, grade unknown
    7:  { kmh: 6,  group: 'path' },
    8:  { kmh: 20, group: 'paved' },   // service
    9:  { kmh: 25, group: 'paved' },   // residential
    10: { kmh: 35, group: 'paved' },   // unclassified
    11: { kmh: 45, group: 'paved' },   // tertiary
    12: { kmh: 55, group: 'paved' },   // secondary
    13: { kmh: 60, group: 'paved' }    // primary
  };
  var MAX_KMH = 60;
  var GROUP_LABEL = { track: 'черен път', path: 'пътека', paved: 'асфалтов път' };
  var R_EARTH = 6371008.8;
  var RAD = Math.PI / 180;

  function haversine(lat1, lon1, lat2, lon2){
    var p1 = lat1 * RAD, p2 = lat2 * RAD, dp = p2 - p1, dl = (lon2 - lon1) * RAD;
    var a = Math.sin(dp / 2) * Math.sin(dp / 2) + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) * Math.sin(dl / 2);
    return 2 * R_EARTH * Math.asin(Math.min(1, Math.sqrt(a)));
  }
  function bearing(lat1, lon1, lat2, lon2){
    var p1 = lat1 * RAD, p2 = lat2 * RAD, dl = (lon2 - lon1) * RAD;
    var y = Math.sin(dl) * Math.cos(p2);
    var x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return (Math.atan2(y, x) / RAD + 360) % 360;
  }

  // Distance from point P to segment AB, in metres (local flat projection — fine for short segments)
  function projectToSegment(plat, plon, alat, alon, blat, blon){
    var kx = Math.cos(plat * RAD) * 111320, ky = 110540;
    var ax = (alon - plon) * kx, ay = (alat - plat) * ky;
    var bx = (blon - plon) * kx, by = (blat - plat) * ky;
    var dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    var t = len2 > 0 ? Math.max(0, Math.min(1, -(ax * dx + ay * dy) / len2)) : 0;
    var x = ax + t * dx, y = ay + t * dy;
    return { t: t, dist: Math.sqrt(x * x + y * y) };
  }

  // ---------- Min-heap of (priority, node) ----------
  function Heap(cap){ this.k = new Float64Array(cap); this.v = new Uint32Array(cap); this.n = 0; }
  Heap.prototype.push = function(key, val){
    if(this.n === this.k.length){
      var k2 = new Float64Array(this.n * 2), v2 = new Uint32Array(this.n * 2);
      k2.set(this.k); v2.set(this.v); this.k = k2; this.v = v2;
    }
    var i = this.n++, k = this.k, v = this.v;
    while(i > 0){
      var p = (i - 1) >> 1;
      if(k[p] <= key) break;
      k[i] = k[p]; v[i] = v[p]; i = p;
    }
    k[i] = key; v[i] = val;
  };
  Heap.prototype.pop = function(){   // returns node; priority in this.lastKey
    var k = this.k, v = this.v, top = v[0];
    this.lastKey = k[0];
    var n = --this.n, key = k[n], val = v[n], i = 0;
    while(true){
      var l = 2 * i + 1;
      if(l >= n) break;
      var r = l + 1, c = (r < n && k[r] < k[l]) ? r : l;
      if(k[c] >= key) break;
      k[i] = k[c]; v[i] = v[c]; i = c;
    }
    k[i] = key; v[i] = val;
    return top;
  };

  // ---------- Router ----------
  function Router(buffer){
    var dv = new DataView(buffer);
    var magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
    if(magic !== 'ORRG') throw new Error('Not a routing file');
    var version = dv.getUint32(4, true);
    if(version !== 1) throw new Error('Unsupported routing file version ' + version);
    var n = this.nodeCount = dv.getUint32(8, true);
    var m = this.edgeCount = dv.getUint32(12, true);
    var g = dv.getUint32(16, true);
    var off = 32;
    this.nodes = new Int32Array(buffer, off, 2 * n); off += 8 * n;
    this.ea = new Uint32Array(buffer, off, m); off += 4 * m;
    this.eb = new Uint32Array(buffer, off, m); off += 4 * m;
    this.elen = new Float32Array(buffer, off, m); off += 4 * m;
    this.egeo = new Uint32Array(buffer, off, m + 1); off += 4 * (m + 1);
    this.geom = new Int32Array(buffer, off, 2 * g); off += 8 * g;
    this.ecls = new Uint8Array(buffer, off, m); off += m;
    this.eflags = new Uint8Array(buffer, off, m);
    this._buildAdjacency();
    this._buildComponents();
    this._buildGrid();
  }

  Router.prototype._buildAdjacency = function(){
    var n = this.nodeCount, m = this.edgeCount, ea = this.ea, eb = this.eb, fl = this.eflags;
    var start = new Uint32Array(n + 1), deg = new Uint16Array(n);
    for(var e = 0; e < m; e++){
      start[ea[e] + 1]++;
      if(!(fl[e] & 1)) start[eb[e] + 1]++;
      deg[ea[e]]++; deg[eb[e]]++;
    }
    for(var i = 0; i < n; i++) start[i + 1] += start[i];
    var fill = start.slice(0, n), adjEdge = new Uint32Array(start[n]), adjRev = new Uint8Array(start[n]);
    for(e = 0; e < m; e++){
      var p = fill[ea[e]]++; adjEdge[p] = e; adjRev[p] = 0;
      if(!(fl[e] & 1)){ p = fill[eb[e]]++; adjEdge[p] = e; adjRev[p] = 1; }
    }
    this.adjStart = start; this.adjEdge = adjEdge; this.adjRev = adjRev; this.degree = deg;
  };

  // Undirected connected components, so snapping can avoid tiny islands of disconnected ways
  Router.prototype._buildComponents = function(){
    var n = this.nodeCount, parent = new Int32Array(n);
    for(var i = 0; i < n; i++) parent[i] = i;
    function find(x){ while(parent[x] !== x){ parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    for(var e = 0; e < this.edgeCount; e++){
      var a = find(this.ea[e]), b = find(this.eb[e]);
      if(a !== b) parent[a] = b;
    }
    var comp = new Int32Array(n), size = new Uint32Array(n), best = 0;
    for(i = 0; i < n; i++){ comp[i] = find(i); size[comp[i]]++; }
    for(i = 0; i < n; i++) if(size[i] > size[best]) best = i;
    this.comp = comp; this.compSize = size; this.mainComp = best;
  };

  // Uniform grid over edge segments for nearest-way lookups
  Router.prototype._buildGrid = function(){
    var nodes = this.nodes, n = this.nodeCount, m = this.edgeCount;
    var minLat = Infinity, maxLat = -Infinity, minLon = Infinity, maxLon = -Infinity;
    for(var i = 0; i < n; i++){
      var la = nodes[2 * i], lo = nodes[2 * i + 1];
      if(la < minLat) minLat = la; if(la > maxLat) maxLat = la;
      if(lo < minLon) minLon = lo; if(lo > maxLon) maxLon = lo;
    }
    var geom = this.geom;
    for(i = 0; i < geom.length; i += 2){
      if(geom[i] < minLat) minLat = geom[i]; if(geom[i] > maxLat) maxLat = geom[i];
      if(geom[i + 1] < minLon) minLon = geom[i + 1]; if(geom[i + 1] > maxLon) maxLon = geom[i + 1];
    }
    var cell = 5000;                          // 0.005° ≈ 400–550 m
    while(((maxLat - minLat) / cell + 1) * ((maxLon - minLon) / cell + 1) > 4e6) cell *= 2;
    var rows = Math.floor((maxLat - minLat) / cell) + 1, cols = Math.floor((maxLon - minLon) / cell) + 1;
    this.grid = { minLat: minLat, minLon: minLon, cell: cell, rows: rows, cols: cols };

    var cells = rows * cols, count = new Uint32Array(cells + 1), stamp = new Int32Array(cells).fill(-1);
    var self = this;
    function eachCell(e, fn){
      var pts = self._edgeMicro(e);
      for(var j = 0; j + 3 < pts.length; j += 2){
        var r0 = Math.floor((Math.min(pts[j], pts[j + 2]) - minLat) / cell), r1 = Math.floor((Math.max(pts[j], pts[j + 2]) - minLat) / cell);
        var c0 = Math.floor((Math.min(pts[j + 1], pts[j + 3]) - minLon) / cell), c1 = Math.floor((Math.max(pts[j + 1], pts[j + 3]) - minLon) / cell);
        for(var r = r0; r <= r1; r++) for(var c = c0; c <= c1; c++){
          var k = r * cols + c;
          if(stamp[k] !== e){ stamp[k] = e; fn(k); }
        }
      }
    }
    for(var e = 0; e < m; e++) eachCell(e, function(k){ count[k + 1]++; });
    for(i = 0; i < cells; i++) count[i + 1] += count[i];
    var fill = count.slice(0, cells), items = new Uint32Array(count[cells]);
    stamp.fill(-1);
    for(e = 0; e < m; e++) eachCell(e, function(k){ items[fill[k]++] = e; });
    this.gridStart = count; this.gridItems = items;
  };

  // Edge points start → end as flat [lat, lon, lat, lon, ...] in micro-degrees
  Router.prototype._edgeMicro = function(e){
    var a = this.ea[e], b = this.eb[e], g0 = this.egeo[e], g1 = this.egeo[e + 1];
    var out = new Array(4 + 2 * (g1 - g0));
    out[0] = this.nodes[2 * a]; out[1] = this.nodes[2 * a + 1];
    for(var i = g0, j = 2; i < g1; i++, j += 2){ out[j] = this.geom[2 * i]; out[j + 1] = this.geom[2 * i + 1]; }
    out[out.length - 2] = this.nodes[2 * b]; out[out.length - 1] = this.nodes[2 * b + 1];
    return out;
  };
  // Edge points as [[lat, lon], ...] in degrees
  Router.prototype.edgePoints = function(e){
    var p = this._edgeMicro(e), out = [];
    for(var i = 0; i < p.length; i += 2) out.push([p[i] / 1e6, p[i + 1] / 1e6]);
    return out;
  };
  Router.prototype.nodeLatLng = function(v){ return [this.nodes[2 * v] / 1e6, this.nodes[2 * v + 1] / 1e6]; };

  // Nearest point on the network. onlyComp: restrict to one connected component.
  Router.prototype.snap = function(lat, lon, maxM, onlyComp){
    var G = this.grid, ml = lat * 1e6, mo = lon * 1e6;
    var r = Math.floor((ml - G.minLat) / G.cell), c = Math.floor((mo - G.minLon) / G.cell);
    var cellM = G.cell / 1e6 * 110540 * Math.cos(lat * RAD);   // conservative cell size in metres
    var radius = Math.max(1, Math.ceil(maxM / cellM));
    var best = null, seen = {};
    for(var rr = r - radius; rr <= r + radius; rr++){
      if(rr < 0 || rr >= G.rows) continue;
      for(var cc = c - radius; cc <= c + radius; cc++){
        if(cc < 0 || cc >= G.cols) continue;
        var k = rr * G.cols + cc;
        for(var i = this.gridStart[k]; i < this.gridStart[k + 1]; i++){
          var e = this.gridItems[i];
          if(seen[e]) continue;
          seen[e] = 1;
          if(onlyComp != null && this.comp[this.ea[e]] !== onlyComp) continue;
          var pts = this.edgePoints(e);
          for(var s = 0; s + 1 < pts.length; s++){
            var pr = projectToSegment(lat, lon, pts[s][0], pts[s][1], pts[s + 1][0], pts[s + 1][1]);
            if(pr.dist <= maxM && (!best || pr.dist < best.dist)){
              best = { e: e, seg: s, t: pr.t, dist: pr.dist, pts: pts };
            }
          }
        }
      }
    }
    if(!best) return null;
    // Position along the edge, scaled to the stored edge length
    var along = 0, total = 0, P = best.pts;
    for(s = 0; s + 1 < P.length; s++){
      var d = haversine(P[s][0], P[s][1], P[s + 1][0], P[s + 1][1]);
      if(s < best.seg) along += d; else if(s === best.seg) along += d * best.t;
      total += d;
    }
    var scale = total > 0 ? this.elen[best.e] / total : 1;
    var A = P[best.seg], B = P[best.seg + 1];
    best.lat = A[0] + (B[0] - A[0]) * best.t;
    best.lon = A[1] + (B[1] - A[1]) * best.t;
    best.toA = along * scale;
    best.toB = (total - along) * scale;
    best.comp = this.comp[this.ea[best.e]];
    return best;
  };

  // pavedFactor: how much more "expensive" asphalt is than its driving time.
  // 1 = fastest route; OFFROAD_PAVED_FACTOR = prefer dirt even when it is slower.
  var OFFROAD_PAVED_FACTOR = 3.5;
  Router.prototype._costPerM = function(cls, pavedFactor){
    var c = CLASSES[cls] || CLASSES[6];
    var spm = 3.6 / c.kmh;                     // seconds per metre
    return c.group === 'paved' ? spm * pavedFactor : spm;
  };

  /* Finds a route. opts: { mode: 'offroad' (default) | 'fast', maxSnapM: number }
   * Returns { points, cum, distance, duration, steps, km: {track, path, paved} } or { error } */
  Router.prototype.route = function(fromLat, fromLon, toLat, toLon, opts){
    opts = opts || {};
    var prefer = opts.mode === 'fast' ? 1 : OFFROAD_PAVED_FACTOR;
    var maxSnap = opts.maxSnapM || 1500;
    var s = this.snap(fromLat, fromLon, maxSnap), t = this.snap(toLat, toLon, maxSnap);
    if(!s) return { error: 'start-far' };
    if(!t) return { error: 'end-far' };
    if(s.comp !== t.comp){
      // One end is on an isolated piece of network — move both onto the main network
      var s2 = s.comp === this.mainComp ? s : this.snap(fromLat, fromLon, maxSnap * 2, this.mainComp);
      var t2 = t.comp === this.mainComp ? t : this.snap(toLat, toLon, maxSnap * 2, this.mainComp);
      if(!s2 || !t2) return { error: 'disconnected' };
      s = s2; t = t2;
    }

    var n = this.nodeCount, ea = this.ea, eb = this.eb, elen = this.elen, ecls = this.ecls, fl = this.eflags;
    var adjStart = this.adjStart, adjEdge = this.adjEdge, adjRev = this.adjRev, nodes = this.nodes;
    var gScore = new Float64Array(n).fill(Infinity);
    var prevEdge = new Int32Array(n).fill(-1), prevRev = new Uint8Array(n), closed = new Uint8Array(n);
    var START = -2;
    var hPerM = 3.6 / MAX_KMH;
    function h(v){ return haversine(nodes[2 * v] / 1e6, nodes[2 * v + 1] / 1e6, t.lat, t.lon) * hPerM; }

    var heap = new Heap(1024);
    var cS = this._costPerM(ecls[s.e], prefer), cT = this._costPerM(ecls[t.e], prefer);
    // A point sitting on an edge end is that vertex itself — one-way rules don't apply to 0 m
    var sOneway = (fl[s.e] & 1) && s.toA >= 1, tOneway = (fl[t.e] & 1) && t.toB >= 1;
    // From the start point we can drive to the edge's end (B), and to its start (A) unless one-way
    var sa = ea[s.e], sb = eb[s.e];
    gScore[sb] = s.toB * cS; prevEdge[sb] = START; prevRev[sb] = 0; heap.push(gScore[sb] + h(sb), sb);
    if(!sOneway && s.toA * cS < gScore[sa]){ gScore[sa] = s.toA * cS; prevEdge[sa] = START; prevRev[sa] = 1; heap.push(gScore[sa] + h(sa), sa); }

    var best = Infinity, bestEnd = -1, direct = false;
    if(s.e === t.e){
      var dAlong = t.toA - s.toA;
      if(dAlong >= 0 || !sOneway){ best = Math.abs(dAlong) * cS; direct = true; }
    }
    var ta = ea[t.e], tb = eb[t.e];

    while(heap.n > 0){
      var u = heap.pop();
      if(heap.lastKey >= best) break;
      if(closed[u]) continue;
      closed[u] = 1;
      var gu = gScore[u];
      if(u === ta && gu + t.toA * cT < best){ best = gu + t.toA * cT; bestEnd = ta; direct = false; }
      if(u === tb && !tOneway && gu + t.toB * cT < best){ best = gu + t.toB * cT; bestEnd = tb; direct = false; }
      for(var p = adjStart[u]; p < adjStart[u + 1]; p++){
        var e = adjEdge[p], rev = adjRev[p], v = rev ? ea[e] : eb[e];
        if(closed[v]) continue;
        var g = gu + elen[e] * this._costPerM(ecls[e], prefer);
        if(g < gScore[v]){ gScore[v] = g; prevEdge[v] = e; prevRev[v] = rev; heap.push(g + h(v), v); }
      }
    }
    if(best === Infinity) return { error: 'no-route' };
    return this._assemble(s, t, direct, bestEnd, prevEdge, prevRev);
  };

  // Builds the polyline, distances and turn-by-turn steps for the found path
  Router.prototype._assemble = function(s, t, direct, endNode, prevEdge, prevRev){
    var self = this, points = [], legs = [];   // legs: {cls, from, to} point index ranges
    function addPts(list, cls){
      var from = Math.max(0, points.length - 1);
      for(var i = 0; i < list.length; i++){
        var q = list[i], last = points[points.length - 1];
        if(last && Math.abs(last[0] - q[0]) < 1e-9 && Math.abs(last[1] - q[1]) < 1e-9) continue;
        points.push(q);
      }
      if(points.length - 1 > from) legs.push({ cls: cls, from: from, to: points.length - 1 });
    }
    var S = [s.lat, s.lon], T = [t.lat, t.lon];
    if(direct){
      var P = s.pts, fwd = t.toA >= s.toA, mid = [];
      if(fwd){ for(var i = s.seg + 1; i <= t.seg; i++) mid.push(P[i]); }
      else { for(i = s.seg; i > t.seg; i--) mid.push(P[i]); }
      addPts([S].concat(mid, [T]), this.ecls[s.e]);
    } else {
      // Walk back from the end node to the start edge
      var chain = [], v = endNode;
      while(prevEdge[v] !== -2){
        var e = prevEdge[v], rev = prevRev[v];
        chain.push({ e: e, rev: rev });
        v = rev ? this.eb[e] : this.ea[e];
      }
      chain.reverse();
      var startRev = prevRev[v];     // 1: left the start point towards A
      var Ps = s.pts, first = [S];
      if(startRev){ for(i = s.seg; i >= 0; i--) first.push(Ps[i]); }
      else { for(i = s.seg + 1; i < Ps.length; i++) first.push(Ps[i]); }
      addPts(first, this.ecls[s.e]);
      chain.forEach(function(c){
        var pts = self.edgePoints(c.e);
        if(c.rev) pts.reverse();
        addPts(pts, self.ecls[c.e]);
      });
      var Pt = t.pts, last = [];
      if(endNode === this.ea[t.e]){ for(i = 0; i <= t.seg; i++) last.push(Pt[i]); }
      else { for(i = Pt.length - 1; i > t.seg; i--) last.push(Pt[i]); }
      last.push(T);
      addPts(last, this.ecls[t.e]);
    }
    if(points.length < 2) points.push(T);

    var cum = [0];
    for(i = 1; i < points.length; i++) cum.push(cum[i - 1] + haversine(points[i - 1][0], points[i - 1][1], points[i][0], points[i][1]));
    var km = { track: 0, path: 0, paved: 0 }, duration = 0;
    legs.forEach(function(L){
      var d = cum[L.to] - cum[L.from], c = CLASSES[L.cls] || CLASSES[6];
      km[c.group] += d / 1000;
      duration += d / (c.kmh / 3.6);
    });
    var route = { points: points, cum: cum, distance: cum[cum.length - 1], duration: duration, km: km, legs: legs };
    route.steps = this._instructions(route);
    return route;
  };

  function pointAtDistance(route, idx, metres, dir){
    // walks from point idx along the route (dir = +1 forward, -1 backward) about `metres`
    var P = route.points, cum = route.cum, target = cum[idx] + dir * metres, j = idx;
    while(j + dir >= 0 && j + dir < P.length && (dir > 0 ? cum[j] < target : cum[j] > target)) j += dir;
    return P[j];
  }

  Router.prototype._instructions = function(route){
    var steps = [], legs = route.legs, P = route.points, self = this;
    for(var i = 0; i + 1 < legs.length; i++){
      var at = legs[i].to, here = P[at];
      var gIn = (CLASSES[legs[i].cls] || CLASSES[6]).group, gOut = (CLASSES[legs[i + 1].cls] || CLASSES[6]).group;
      var a = pointAtDistance(route, at, 20, -1), b = pointAtDistance(route, at, 20, +1);
      var turn = ((bearing(here[0], here[1], b[0], b[1]) - bearing(a[0], a[1], here[0], here[1]) + 540) % 360) - 180;
      var junction = self._isJunctionAt(here);
      var abs = Math.abs(turn), kind;
      if(abs < 30){
        if(gIn === gOut) continue;            // just a new way segment — nothing to say
        kind = 'continue';
      } else {
        if(!junction) continue;               // the road only bends, no choice to make
        kind = abs < 60 ? 'slight' : abs < 140 ? 'turn' : 'sharp';
      }
      steps.push(makeStep(kind, turn, gOut, at, route.cum[at]));
    }
    steps.push({ kind: 'arrive', idx: P.length - 1, dist: route.distance, arrow: '🏁', text: 'Пристигане', speech: 'Пристигнахте' });
    // Drop instructions that come right after each other (< 15 m) — keep the later one
    return steps.filter(function(st, k){ return k + 1 >= steps.length || steps[k + 1].dist - st.dist > 15; });
  };

  Router.prototype._isJunctionAt = function(latlng){
    var sn = this.snap(latlng[0], latlng[1], 3);
    if(!sn) return false;
    // the point is a vertex if it sits on an edge end
    var v = sn.toA < 1 ? this.ea[sn.e] : sn.toB < 1 ? this.eb[sn.e] : -1;
    return v >= 0 && this.degree[v] >= 3;
  };

  function makeStep(kind, turn, group, idx, dist){
    var left = turn < 0, label = GROUP_LABEL[group];
    var verb, arrow;
    if(kind === 'continue'){ verb = 'Продължи'; arrow = '↑'; }
    else if(kind === 'slight'){ verb = left ? 'Леко наляво' : 'Леко надясно'; arrow = left ? '↖' : '↗'; }
    else if(kind === 'turn'){ verb = left ? 'Завий наляво' : 'Завий надясно'; arrow = left ? '←' : '→'; }
    else { verb = left ? 'Остро наляво' : 'Остро надясно'; arrow = left ? '↙' : '↘'; }
    var text = verb + ' по ' + label;
    return { kind: kind, idx: idx, dist: dist, arrow: arrow, text: text, speech: text };
  }

  /* Where is the rider on the route? hint: last known point index (keeps the search local).
   * Returns { idx, along (m from start), off (m from the line) } */
  Router.locate = function(route, lat, lon, hint){
    var P = route.points, from = 0, to = P.length - 1;
    if(hint != null){ from = Math.max(0, hint - 10); to = Math.min(P.length - 1, hint + 400); }
    var best = null;
    for(var pass = 0; pass < 2 && !best; pass++){
      for(var i = from; i < to; i++){
        var pr = projectToSegment(lat, lon, P[i][0], P[i][1], P[i + 1][0], P[i + 1][1]);
        if(!best || pr.dist < best.off){
          best = { idx: i, off: pr.dist, along: route.cum[i] + pr.t * (route.cum[i + 1] - route.cum[i]) };
        }
      }
      if(best && best.off > 200 && hint != null){ best = null; from = 0; to = P.length - 1; }  // lost — search all
    }
    return best || { idx: 0, off: Infinity, along: 0 };
  };

  Router.haversine = haversine;
  global.OffroadRouter = Router;
})(typeof window !== 'undefined' ? window : globalThis);
