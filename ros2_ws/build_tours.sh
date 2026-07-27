#!/usr/bin/env bash
# Erzeugt pro Radius ein Tour-Config-File neben building.json.
#
#   ./build_tours.sh <radius>
#
# Liest config/building.json (footprints + Startpunkt pro Raum), berechnet mit VisLib
# (Rational-Pfad, exakt) je Raum eine Sightseeing-Tour zum gegebenen Radius und schreibt
# config/tours_r<radius>.json. Der Radius steht im Dateinamen und als Key in der Datei;
# die Raeume sind unter "rooms" mit dem Raumnamen als Key abfragbar.
#
# Der gesnappte Waypoint (locations[i]) liegt auf/knapp neben dem Rand. Ein Startpunkt
# exakt auf oder ausserhalb der Kante laesst die Sichtbarkeitsberechnung degenerieren,
# deshalb wird der Start auf den Rand projiziert und ein Stueck Richtung Zentroid nach
# innen gezogen (robustes Regime). Falls das nicht klappt, werden weitere Kandidaten
# probiert; jeder Kandidat laeuft in einem eigenen Prozess mit Timeout.
set -euo pipefail

RADIUS="${1:?usage: ./build_tours.sh <radius>}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILDING="$HERE/config/building.json"
TOOL_DIR="$HERE/tools/tour_gen"
TOOL="$TOOL_DIR/build/tour_gen"
OUT="$HERE/config/tours_r${RADIUS}.json"

if [ ! -f "$BUILDING" ]; then
    echo "building.json nicht gefunden: $BUILDING (erst ./build_snapshot.sh)" >&2
    exit 1
fi

echo "Building tour_gen (Rational, NDEBUG)..."
cmake -S "$TOOL_DIR" -B "$TOOL_DIR/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$TOOL_DIR/build" -j >/dev/null

echo "Computing tours (radius=$RADIUS) ..."
python3 - "$BUILDING" "$TOOL" "$RADIUS" "$OUT" <<'PY'
import json, subprocess, sys, time

building_path, tool, radius, out_path = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
b = json.load(open(building_path))
names = b["names"]
fps   = b.get("footprints", [])
locs  = b["locations"]

TIMEOUT = 45


def centroid(verts):
    n = len(verts)
    return (sum(x for x, _ in verts) / n, sum(y for _, y in verts) / n)


def project_to_boundary(px, py, verts):
    best = (px, py)
    bd = float("inf")
    n = len(verts)
    for i in range(n):
        ax, ay = verts[i]
        bx, by = verts[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        seg2 = dx * dx + dy * dy
        t = 0.0 if seg2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg2))
        qx, qy = ax + t * dx, ay + t * dy
        d = ((px - qx) ** 2 + (py - qy) ** 2) ** 0.5
        if d < bd:
            bd = d
            best = (qx, qy)
    return best


def start_candidates(sx, sy, verts):
    cx, cy = centroid(verts)
    bx, by = project_to_boundary(sx, sy, verts)
    for alpha in (0.10, 0.25):
        yield (bx + (cx - bx) * alpha, by + (cy - by) * alpha)
    yield (sx, sy)


def run_tour(verts, ux, uy):
    poly = " ".join(f"{x} {y}" for x, y in verts)
    try:
        r = subprocess.run([tool, str(radius)], input=f"{ux} {uy}\n{poly}\n",
                           capture_output=True, text=True, timeout=TIMEOUT)
    except subprocess.TimeoutExpired:
        return ("timeout", None)
    if r.returncode != 0:
        return (f"crash(exit {r.returncode})", None)
    lines = r.stdout.splitlines()
    for idx, ln in enumerate(lines):
        t = ln.split()
        if not t:
            continue
        if t[0] == "OK":
            steps = int(t[1])
            dist = float(t[2])
            path = [[float(a), float(bb)] for a, bb in (p.split() for p in lines[idx + 1:idx + 1 + steps])]
            return ("OK", {"steps": steps, "distance": dist, "path": path})
        if t[0] in ("DEGENERATE", "THROW"):
            return (t[0].lower(), None)
    return ("noout", None)


rooms = {}
n_ok = n_fail = 0
for i, name in enumerate(names):
    fp = fps[i] if i < len(fps) else None
    if not isinstance(fp, list) or len(fp) < 3:
        continue
    verts = [(p[0], p[1]) for p in fp]
    data = None
    used = None
    reason = "no-candidate"
    for ux, uy in start_candidates(locs[i]["x"], locs[i]["y"], verts):
        status, result = run_tour(verts, ux, uy)
        if status == "OK":
            data = result
            used = [ux, uy]
            break
        reason = status
    if data is not None:
        rooms[name] = {"ok": True, "steps": data["steps"], "distance": data["distance"],
                       "start": used, "path": data["path"]}
        n_ok += 1
    else:
        rooms[name] = {"ok": False, "reason": reason}
        n_fail += 1

out = {
    "radius": radius,
    "generated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    "rooms": rooms,
}
with open(out_path, "w") as f:
    json.dump(out, f, indent=2)
print(f"tours: {n_ok} ok, {n_fail} failed  ->  {out_path}")
PY
