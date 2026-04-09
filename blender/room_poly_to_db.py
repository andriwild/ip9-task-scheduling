import bpy
import re
from mathutils import Vector
from mathutils.geometry import intersect_point_tri_2d
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
    """Check if point (px, py) lies inside a polygon using triangle fan decomposition."""
    n = len(verts)
    for i in range(1, n - 1):
        if intersect_point_tri_2d(
            Vector((px, py)),
            Vector((verts[0][0], verts[0][1])),
            Vector((verts[i][0], verts[i][1])),
            Vector((verts[i + 1][0], verts[i + 1][1])),
        ):
            return True
    return False


def classify_room(name):
    """Classify room type based on name (matches C++ parseRoomName logic)."""
    if "Toilet" in name:
        return "TOILET"
    if "Kitchen" in name:
        return "KITCHEN"
    if re.match(r"5\.2[A-Z]\d+", name):
        return "WORKPLACE"
    return "OTHER"


def get_polygon_vertices(obj):
    """Extract ordered 2D vertices from a mesh object's first polygon face."""
    mesh = obj.data
    matrix = obj.matrix_world

    if not mesh.polygons:
        return None

    verts = []
    for i in mesh.polygons[0].vertices:
        co = matrix @ mesh.vertices[i].co
        verts.append((round(co.x, 2), round(co.y, 2)))
    return verts


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
