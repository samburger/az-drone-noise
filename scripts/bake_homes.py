# /// script
# requires-python = ">=3.11"
# dependencies = ["osmium>=3.7", "pyshp>=2.3", "shapely>=2.0"]
# ///
"""Bake residential building centroids (with 2020 Census population weights) near the Amazon
BDL3 drone pad into data/homes.js, and the OSM address index into data/addresses.js.

Usage: uv run scripts/bake_homes.py [--refresh]
"""

import datetime
import io
import json
import math
import re
import sys
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

import osmium
import shapefile
from shapely import STRtree, points
from shapely.geometry import shape

PAD = (41.4140625, -72.8474375)
RADIUS_M = 12070.0  # 7.5 mi
PBF_URL = "https://download.geofabrik.de/north-america/us/connecticut-latest.osm.pbf"
CACHE = Path.home() / ".cache" / "amazon-drones" / "connecticut-latest.osm.pbf"
OUT = Path(__file__).resolve().parent.parent / "data" / "homes.js"
CENSUS_URL = "https://www2.census.gov/geo/tiger/TIGER2020/TABBLOCK20/tl_2020_09_tabblock20.zip"
CENSUS_CACHE = CACHE.parent / "tl_2020_09_tabblock20.zip"
ADDR_OUT = OUT.parent / "addresses.js"

R_EARTH = 6371008.8
BUILDING_TYPES = {
    "yes", "house", "detached", "residential",
    "semidetached_house", "terrace", "bungalow", "apartments",
}
EXCLUDE_KEYS = ("name", "shop", "amenity", "office", "brand", "craft", "tourism", "leisure")

LAT0, LON0 = PAD
COS_LAT0 = math.cos(math.radians(LAT0))
DEG = math.pi / 180.0


def download(url: str, dest: Path, refresh: bool) -> None:
    if dest.exists() and not refresh:
        print(f"Using cached {dest} ({dest.stat().st_size / 1e6:.1f} MB)")
        return
    dest.parent.mkdir(parents=True, exist_ok=True)
    part = dest.with_name(dest.name + ".part")
    req = urllib.request.Request(url, headers={"User-Agent": "amazon-drones-noise-sim/1.0"})
    print(f"Downloading {url}")
    try:
        with urllib.request.urlopen(req) as resp, open(part, "wb") as f:
            done = 0
            next_report = 10 * 1024 * 1024
            while chunk := resp.read(1024 * 1024):
                f.write(chunk)
                done += len(chunk)
                if done >= next_report:
                    print(f"  {done / (1024 * 1024):.0f} MB")
                    next_report += 10 * 1024 * 1024
    except urllib.error.HTTPError as e:
        print(f"HTTP error: {e.code} {e.reason}")
        sys.exit(1)
    part.replace(dest)
    print(f"Saved {dest} ({dest.stat().st_size / 1e6:.1f} MB)")


def natkey(num: str) -> tuple:
    m = re.match(r"(\d+)(.*)", num)
    return (int(m[1]), m[2]) if m else (10**9, num)


class HomeHandler(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.homes = []  # (way_id, lat, lon, floor)
        self.addrs = []  # (city, street, housenumber, lat, lon)

    def _add_address(self, tags, lat, lon):
        x = R_EARTH * COS_LAT0 * (lon - LON0) * DEG
        y = R_EARTH * (lat - LAT0) * DEG
        if math.hypot(x, y) > RADIUS_M:
            return
        self.addrs.append((tags.get("addr:city", ""), tags["addr:street"],
                           tags["addr:housenumber"], lat, lon))

    def node(self, n):
        tags = n.tags
        if "addr:housenumber" not in tags or "addr:street" not in tags:
            return
        loc = n.location
        if not loc.valid():
            return
        self._add_address(tags, loc.lat, loc.lon)

    def _capture_address(self, w):
        tags = w.tags
        if "addr:housenumber" not in tags or "addr:street" not in tags:
            return
        locs = []
        try:
            for n in w.nodes:
                loc = n.location
                if not loc.valid():
                    return
                locs.append((loc.lat, loc.lon))
        except osmium.InvalidLocationError:
            return
        if not locs:
            return
        if w.is_closed():
            locs = locs[:-1]
        if not locs:
            return
        lat = sum(p[0] for p in locs) / len(locs)
        lon = sum(p[1] for p in locs) / len(locs)
        self._add_address(tags, lat, lon)

    def way(self, w):
        self._capture_address(w)
        tags = w.tags
        btype = tags.get("building")
        if btype not in BUILDING_TYPES:
            return
        if any(k in tags for k in EXCLUDE_KEYS):
            return
        if not w.is_closed() or len(w.nodes) < 4:
            return
        pts = []
        try:
            for n in w.nodes:
                loc = n.location
                if not loc.valid():
                    return
                pts.append((R_EARTH * COS_LAT0 * (loc.lon - LON0) * DEG,
                            R_EARTH * (loc.lat - LAT0) * DEG))
        except osmium.InvalidLocationError:
            return
        uniq = pts[:-1]
        cx = sum(p[0] for p in uniq) / len(uniq)
        cy = sum(p[1] for p in uniq) / len(uniq)
        area = 0.0
        for i in range(len(uniq)):
            x1, y1 = uniq[i]
            x2, y2 = uniq[(i + 1) % len(uniq)]
            area += x1 * y2 - x2 * y1
        area = abs(area) / 2.0
        max_area = 3000.0 if btype == "apartments" else 800.0
        if area < 40.0 or area > max_area:
            return
        if math.hypot(cx, cy) > RADIUS_M:
            return
        lat = LAT0 + cy / R_EARTH / DEG
        lon = LON0 + cx / (R_EARTH * COS_LAT0) / DEG
        levels = None
        try:
            levels = float(tags.get("building:levels"))
        except (TypeError, ValueError):
            pass
        if levels is None or not 1.0 <= levels <= 40.0:
            levels = 3.0 if btype == "apartments" else 1.0
        self.homes.append((w.id, lat, lon, area * levels))


def assign_population(homes: list) -> tuple[list, dict]:
    """Split each Census block's POP20 among its homes by floor area.

    Returns (rows sorted by way id as (id, lat, lon, weight), stats). Homes outside any
    populated block are dropped.
    """
    dlat = (RADIUS_M + 500.0) / (R_EARTH * DEG)
    dlon = (RADIUS_M + 500.0) / (R_EARTH * COS_LAT0 * DEG)
    with zipfile.ZipFile(CENSUS_CACHE) as z:
        members = {}
        for name in z.namelist():
            for ext in ("shp", "shx", "dbf"):
                if name.lower().endswith("." + ext):
                    members[ext] = io.BytesIO(z.read(name))
        reader = shapefile.Reader(shp=members["shp"], shx=members["shx"], dbf=members["dbf"])

    geoms = []
    pops = []
    block_pop = 0
    with reader:
        for sr in reader.iterShapeRecords():
            rec = sr.record.as_dict()
            pop = int(rec["POP20"])
            if pop <= 0:
                continue
            minx, miny, maxx, maxy = sr.shape.bbox
            if maxx < LON0 - dlon or minx > LON0 + dlon or maxy < LAT0 - dlat or miny > LAT0 + dlat:
                continue
            geoms.append(shape(sr.shape.__geo_interface__))
            pops.append(pop)
            ilat = float(rec["INTPTLAT20"])
            ilon = float(rec["INTPTLON20"])
            if math.hypot(R_EARTH * COS_LAT0 * (ilon - LON0) * DEG,
                          R_EARTH * (ilat - LAT0) * DEG) <= RADIUS_M:
                block_pop += pop

    block_of = {}
    if geoms and homes:
        tree = STRtree(geoms)
        home_idx, block_idx = tree.query(
            points([h[2] for h in homes], [h[1] for h in homes]), predicate="within")
        for hi, bi in zip(home_idx.tolist(), block_idx.tolist()):
            block_of.setdefault(hi, bi)

    floor_sum = {}
    for hi, bi in block_of.items():
        floor_sum[bi] = floor_sum.get(bi, 0.0) + homes[hi][3]

    rows = []
    assigned = 0.0
    for hi, bi in block_of.items():
        wid, lat, lon, floor = homes[hi]
        weight = pops[bi] * floor / floor_sum[bi]
        assigned += weight
        rows.append((wid, lat, lon, weight))
    rows.sort(key=lambda r: r[0])
    stats = {"blocks": len(geoms), "blockPop": block_pop, "assigned": assigned,
             "dropped": len(homes) - len(rows)}
    return rows, stats


def main() -> None:
    refresh = "--refresh" in sys.argv[1:]
    download(PBF_URL, CACHE, refresh)
    download(CENSUS_URL, CENSUS_CACHE, refresh)
    print("Parsing PBF ...")
    h = HomeHandler()
    h.apply_file(str(CACHE), locations=True, idx="flex_mem")
    h.homes.sort(key=lambda t: t[0])

    print("Assigning Census population ...")
    rows, stats = assign_population(h.homes)

    d = []
    w = []
    for _, lat, lon, weight in rows:
        d.append(round((lat - LAT0) * 1e5))
        d.append(round((lon - LON0) * 1e5))
        w.append(max(1, round(weight * 10)))
    payload = {"origin": [LAT0, LON0], "scale": 100000, "radiusM": 12070,
               "count": len(rows), "d": d, "w": w, "wScale": 10,
               "population": round(stats["assigned"])}
    stamp = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    header = (
        f"// Generated by scripts/bake_homes.py on {stamp}. Do not edit.\n"
        "// Data © OpenStreetMap contributors, ODbL 1.0 (https://www.openstreetmap.org/copyright), "
        "via Geofabrik connecticut-latest.osm.pbf.\n"
    )
    text = (
        header
        + "// Population: U.S. Census Bureau 2020 Census blocks (TIGER/Line TABBLOCK20 POP20).\n"
        + f"window.HOMES={json.dumps(payload, separators=(',', ':'))};\n"
    )
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(text, encoding="utf-8")

    count = len(rows)
    print(f"Homes: {count} (dropped {stats['dropped']} outside populated blocks)")
    print(f"Residents assigned: {round(stats['assigned'])} of {stats['blockPop']} "
          "block population in radius")
    print(f"Wrote {OUT} ({OUT.stat().st_size / 1e6:.2f} MB)")
    if not 50000 <= count <= 80000:
        print("WARNING: unexpected home count")
    if not 150000 <= stats["assigned"] <= 450000:
        print("WARNING: unexpected population")

    addrs = sorted(h.addrs, key=lambda a: (a[0], a[1], natkey(a[2])))
    streets = {}
    count_addr = 0
    prev = None
    for city, street, num, lat, lon in addrs:
        key = (city, street, num)
        if key == prev:
            continue
        prev = key
        entry = streets.setdefault((street, city), [street, city, [], []])
        entry[2].append(num)
        entry[3].append(round((lat - LAT0) * 1e5))
        entry[3].append(round((lon - LON0) * 1e5))
        count_addr += 1
    addr_payload = {"origin": [LAT0, LON0], "scale": 100000, "count": count_addr,
                    "s": list(streets.values())}
    ADDR_OUT.write_text(
        header + f"window.ADDRESSES={json.dumps(addr_payload, separators=(',', ':'))};\n",
        encoding="utf-8")
    print(f"Addresses: {count_addr} on {len(streets)} streets")
    print(f"Wrote {ADDR_OUT} ({ADDR_OUT.stat().st_size / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
