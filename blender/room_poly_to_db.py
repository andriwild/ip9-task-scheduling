import bpy
import bmesh
import re
from sqlalchemy import create_engine, text

# --- CONFIGURATION ---
POLYGON_COLLECTION = "RoomPolygons"
WAYPOINT_COLLECTION = "Waypoints"
DATABASE_URL = "postgresql://wsr_user:wsr_password@localhost:5432/wsr"
# ---------------------

print("-" * 30)


def get_waypoints():
    """Read waypoint positions from the Blender collection."""
    waypoints = {}
    if WAYPOINT_COLLECTION not in bpy.data.collections:
        print(f"ERROR: Collection '{WAYPOINT_COLLECTION}' not found!")
        return waypoints

    for obj in bpy.data.collections[WAYPOINT_COLLECTION].objects:
        name = obj.name.replace(" ", "_")
        x = round(obj.location.x, 2)
        y = round(obj.location.y, 2)
        waypoints[name] = (x, y)
    return waypoints


def point_in_polygon(px, py, verts):
    """Ray-casting point-in-polygon test. Works for arbitrary (incl. concave) polygons."""
    inside = False
    n = len(verts)
    j = n - 1
    for i in range(n):
        xi, yi = verts[i]
        xj, yj = verts[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def classify_room(name):
    """Classify room type based on name (matches C++ parseRoomName logic)."""
    if "Toilet" in name:
        return "TOILET"
    if "Kitchen" in name:
        return "KITCHEN"
    if re.match(r"5\.2[A-Z]\d+", name):
        return "WORKPLACE"
    return "OTHER"


def _signed_area_2d(pts):
    n = len(pts)
    a = 0.0
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        a += x1 * y2 - x2 * y1
    return a / 2.0


def get_polygon_vertices(obj):
    """Extract ordered 2D outer-boundary vertices from a mesh object.
    Collects all boundary loops (handling holes / disjoint islands / multi-face meshes)
    and returns the one with the largest absolute area (= outer contour)."""
    mesh = obj.data
    matrix = obj.matrix_world

    if not mesh.polygons:
        return None

    bm = bmesh.new()
    try:
        bm.from_mesh(mesh)
        bm.transform(matrix)  # bake world transform into vertex coords

        boundary_edges = set(e for e in bm.edges if len(e.link_faces) == 1)
        if not boundary_edges:
            return None

        loops = []
        unused = set(boundary_edges)
        while unused:
            seed = next(iter(unused))
            loop_verts = []
            current_edge = seed
            current_vert = current_edge.verts[0]
            while True:
                unused.discard(current_edge)
                loop_verts.append(current_vert)
                next_vert = current_edge.other_vert(current_vert)
                next_edge = None
                for e in next_vert.link_edges:
                    if e in boundary_edges and e is not current_edge and e in unused:
                        next_edge = e
                        break
                if next_edge is None:
                    break
                current_edge = next_edge
                current_vert = next_vert
            if len(loop_verts) >= 3:
                loops.append(loop_verts)

        if not loops:
            return None

        # Pick the outer boundary: loop with the largest absolute area
        def loop_area(loop):
            return abs(_signed_area_2d([(v.co.x, v.co.y) for v in loop]))

        outer = max(loops, key=loop_area)
        return [(round(v.co.x, 2), round(v.co.y, 2)) for v in outer]
    finally:
        bm.free()


def verts_to_wkt(verts):
    """Convert vertex list to a closed WKT POLYGON string."""
    closed = verts + [verts[0]]
    coords = ", ".join(f"{x} {y}" for x, y in closed)
    return f"POLYGON(({coords}))"


# --- MAIN ---

waypoints = get_waypoints()
print(f"Found: {len(waypoints)} waypoints")

if POLYGON_COLLECTION not in bpy.data.collections:
    print(f"ERROR: Collection '{POLYGON_COLLECTION}' not found!")
else:
    engine = create_engine(DATABASE_URL)

    with engine.connect() as conn:
        trans = conn.begin()
        try:
            # Load poi_id mapping from DB
            rows = conn.execute(text("SELECT id, name FROM points_of_interest")).fetchall()
            poi_ids = {row[1]: row[0] for row in rows}
            print(f"Found: {len(poi_ids)} points of interest in DB")

            # Clear existing search zones for clean re-import
            conn.execute(text("DELETE FROM search_zones"))

            polygon_objects = sorted(
                bpy.data.collections[POLYGON_COLLECTION].objects, key=lambda o: o.name
            )

            matched = 0
            unmatched = []

            for obj in polygon_objects:
                if obj.type != "MESH":
                    continue

                verts = get_polygon_vertices(obj)
                if not verts or len(verts) < 3:
                    print(f"  SKIP: {obj.name} (not enough vertices)")
                    continue

                # Find waypoint that lies inside this polygon
                matched_name = None
                for wp_name, (wx, wy) in waypoints.items():
                    if point_in_polygon(wx, wy, verts):
                        matched_name = wp_name
                        break

                if not matched_name:
                    unmatched.append(obj.name)
                    print(f"  WARNING: {obj.name} contains no waypoint!")
                    # Diagnostics: polygon bbox + 5 nearest waypoints w/ inside-flag
                    xs = [x for x, _ in verts]
                    ys = [y for _, y in verts]
                    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
                    print(f"    bbox: x=[{min(xs):.2f},{max(xs):.2f}] y=[{min(ys):.2f},{max(ys):.2f}]  centroid=({cx:.2f},{cy:.2f})  vertices={len(verts)}")
                    nearby = sorted(
                        waypoints.items(),
                        key=lambda kv: (kv[1][0] - cx) ** 2 + (kv[1][1] - cy) ** 2,
                    )[:5]
                    for wp_name, (wx, wy) in nearby:
                        dist = ((wx - cx) ** 2 + (wy - cy) ** 2) ** 0.5
                        inside = point_in_polygon(wx, wy, verts)
                        print(f"    nearest wp: {wp_name:20} at ({wx:.2f},{wy:.2f})  dist={dist:.2f}  inside={inside}")
                    continue

                if matched_name not in poi_ids:
                    print(f"  WARNING: {matched_name} not in points_of_interest table!")
                    continue

                room_type = classify_room(matched_name)
                wkt = verts_to_wkt(verts)
                poi_id = poi_ids[matched_name]

                conn.execute(
                    text("INSERT INTO search_zones (name, type, poi_id, polygon) "
                         "VALUES (:name, :type, :poi_id, ST_GeomFromText(:wkt))"),
                    {"name": matched_name, "type": room_type, "poi_id": poi_id, "wkt": wkt},
                )

                # Rename Blender object to matched waypoint name
                obj.name = matched_name
                matched += 1
                print(f"  OK: {matched_name:20} | Type: {room_type:10} | poi_id: {poi_id} | Vertices: {len(verts)}")

            trans.commit()
            print(f"\n{'=' * 30}")
            print(f"Saved: {matched} search zones")
            if unmatched:
                print(f"No waypoint match: {unmatched}")

        except Exception as e:
            trans.rollback()
            print(f"Error: {e}")
            raise
