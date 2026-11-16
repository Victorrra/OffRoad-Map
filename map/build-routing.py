#!/usr/bin/env python3
"""
Builds bulgaria-routing.bin — a compact routing graph for ATVs — from an OSM .pbf extract.

Only junctions and dead ends become graph vertices; the points in between are kept as
edge geometry so the route can be drawn exactly. Ways closed to ATVs are left out.

Usage:  python3 map/build-routing.py data/sources/bulgaria.osm.pbf bulgaria-routing.bin

Binary format (little endian), version 1 — read by router.js:
  header  32 bytes : "ORRG", u32 version, u32 nodes, u32 edges, u32 geomPoints, 12 reserved
  i32[2*nodes]     : node lat, lon in micro-degrees
  u32[edges]       : edge start node
  u32[edges]       : edge end node
  f32[edges]       : edge length in metres
  u32[edges+1]     : start of each edge's inner geometry (in points)
  i32[2*geomPoints]: inner geometry lat, lon in micro-degrees (start → end)
  u8[edges]        : road class (see CLASS below)
  u8[edges]        : flags (bit0 = one-way start→end, bit1 = ford)
"""
import math
import struct
import sys
from array import array

import osmium

# Road classes — must match router.js
CLASS = {
    'grade1': 1, 'grade2': 2, 'grade3': 3, 'grade4': 4, 'grade5': 5,
    'track': 6,           # track without tracktype
    'path': 7,            # path/bridleway explicitly open to ATVs
    'service': 8, 'residential': 9, 'unclassified': 10,
    'tertiary': 11, 'secondary': 12, 'primary': 13,
}
ROAD_CLASS = {
    'service': 'service', 'residential': 'residential', 'living_street': 'residential',
    'unclassified': 'unclassified', 'road': 'unclassified',
    'tertiary': 'tertiary', 'tertiary_link': 'tertiary',
    'secondary': 'secondary', 'secondary_link': 'secondary',
    'primary': 'primary', 'primary_link': 'primary',
}
# motorway / trunk are left out on purpose: no ATVs there.
OPEN = {'yes', 'designated', 'permissive', 'destination'}
CLOSED = {'no', 'private'}


def atv_access(tags):
    """True / False / None (no explicit rule) — explicit atv=* wins, like the map style."""
    atv = tags.get('atv')
    if atv in OPEN:
        return True
    if atv in CLOSED:
        return False
    for key in ('motor_vehicle', 'motorcycle', 'vehicle', 'access'):
        v = tags.get(key)
        if v in OPEN:
            return True
        if v in CLOSED:
            return False
    return None


def classify(tags):
    hw = tags.get('highway')
    if not hw or tags.get('area') == 'yes':
        return None
    access = atv_access(tags)
    if access is False:
        return None
    if hw == 'track':
        return CLASS.get(tags.get('tracktype', ''), CLASS['track'])
    if hw in ('path', 'bridleway'):
        return CLASS['path'] if access is True else None   # only when explicitly allowed
    if hw in ROAD_CLASS:
        return CLASS[ROAD_CLASS[hw]]
    return None


def oneway(tags):
    v = tags.get('oneway')
    if v in ('yes', '1', 'true'):
        return 1
    if v == '-1':
        return -1
    if tags.get('junction') in ('roundabout', 'circular') and v != 'no':
        return 1
    return 0


class WayCollector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.ways = []            # (class, oneway, ford, node ids array, lat array, lon array)
        self.use = {}             # node id → number of times used (junction detection)

    def way(self, w):
        tags = w.tags
        cls = classify(tags)
        if cls is None or len(w.nodes) < 2:
            return
        ids, lats, lons = array('q'), array('i'), array('i')
        for n in w.nodes:
            if not n.location.valid():
                continue
            ids.append(n.ref)
            lats.append(round(n.location.lat * 1e6))
            lons.append(round(n.location.lon * 1e6))
        if len(ids) < 2:
            return
        ow = oneway(tags)
        if ow == -1:              # store reversed so one-way is always start → end
            ids.reverse(); lats.reverse(); lons.reverse()
        ford = 1 if tags.get('ford') == 'yes' else 0
        self.ways.append((cls, 1 if ow else 0, ford, ids, lats, lons))
        use = self.use
        for i, nid in enumerate(ids):
            # endpoints always count twice so they become vertices
            use[nid] = use.get(nid, 0) + (2 if i == 0 or i == len(ids) - 1 else 1)


def seg_len(lat1, lon1, lat2, lon2):
    """Haversine distance in metres between micro-degree coordinates."""
    p1, p2 = math.radians(lat1 / 1e6), math.radians(lat2 / 1e6)
    dp, dl = p2 - p1, math.radians((lon2 - lon1) / 1e6)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * 6371008.8 * math.asin(math.sqrt(a))


def build(pbf_path, out_path):
    h = WayCollector()
    h.apply_file(pbf_path, locations=True, idx='flex_mem')
    print(f'ways kept: {len(h.ways):,}')

    vertex = {}                   # node id → vertex index
    node_lat, node_lon = array('i'), array('i')
    e_a, e_b, e_len, e_geo = array('I'), array('I'), array('f'), array('I')
    e_cls, e_flags = array('B'), array('B')
    g_lat, g_lon = array('i'), array('i')

    def vid(nid, lat, lon):
        v = vertex.get(nid)
        if v is None:
            v = len(node_lat)
            vertex[nid] = v
            node_lat.append(lat); node_lon.append(lon)
        return v

    use = h.use
    for cls, ow, ford, ids, lats, lons in h.ways:
        start = 0
        length = 0.0
        for i in range(1, len(ids)):
            length += seg_len(lats[i - 1], lons[i - 1], lats[i], lons[i])
            if use[ids[i]] >= 2 or i == len(ids) - 1:
                a = vid(ids[start], lats[start], lons[start])
                b = vid(ids[i], lats[i], lons[i])
                if a != b or i - start > 1:          # skip degenerate zero-length loops
                    e_a.append(a); e_b.append(b); e_len.append(length)
                    e_geo.append(len(g_lat))
                    g_lat.extend(lats[start + 1:i]); g_lon.extend(lons[start + 1:i])
                    e_cls.append(cls); e_flags.append(ow | (ford << 1))
                start, length = i, 0.0
    e_geo.append(len(g_lat))

    n, m, g = len(node_lat), len(e_a), len(g_lat)
    nodes = array('i', [0]) * (2 * n)
    nodes[0::2] = node_lat; nodes[1::2] = node_lon
    geom = array('i', [0]) * (2 * g)
    geom[0::2] = g_lat; geom[1::2] = g_lon

    for arr in (nodes, e_a, e_b, e_len, e_geo, geom):
        if sys.byteorder != 'little':
            arr.byteswap()
    with open(out_path, 'wb') as f:
        f.write(b'ORRG' + struct.pack('<IIII', 1, n, m, g) + b'\0' * 12)
        for arr in (nodes, e_a, e_b, e_len, e_geo, geom, e_cls, e_flags):
            arr.tofile(f)
    total_km = sum(e_len) / 1000
    print(f'vertices: {n:,}  edges: {m:,}  inner points: {g:,}  network: {total_km:,.0f} km')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    build(sys.argv[1], sys.argv[2])
