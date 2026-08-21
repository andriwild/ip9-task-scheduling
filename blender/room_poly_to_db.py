# Generated with Claude Code (Anthropic), then reviewed and adapted by the author. See the index of auxiliary tools.

import bpy
import bmesh
from sqlalchemy import create_engine, text

WAYPOINT_COLLECTION = "Waypoints"
DATABASE_URL = "postgresql://wsr_user:wsr_password@localhost:5432/wsr"
TYPE_COLLECTIONS = {
    "Office": "OFFICE",
    "Classrooms": "CLASSROOM",
    "Meeting": "MEETING",
    "Kitchen": "KITCHEN",
    "Toilets": "TOILET",
    "Access": "ACCESS",
    "Spaces": "SPACE",
    "Misc": "MISC",
}

INSERT_ZONE = text(
    "INSERT INTO search_zones (type, poi_id, polygon) "
    "VALUES (:type, :poi_id, ST_GeomFromText(:wkt))"
)
SELECT_INVALID = text(
    "SELECT p.name, ST_IsValidReason(sz.polygon) FROM search_zones sz "
    "JOIN points_of_interest p ON p.id = sz.poi_id WHERE NOT ST_IsValid(sz.polygon)"
)


def read_waypoints():
    collection = bpy.data.collections[WAYPOINT_COLLECTION]
    return {
        obj.name.replace(" ", "_"): (round(obj.location.x, 2), round(obj.location.y, 2))
        for obj in collection.objects
    }


def read_rooms():
    rooms = [
        (obj, room_type)
        for collection_name, room_type in TYPE_COLLECTIONS.items()
        for obj in bpy.data.collections[collection_name].all_objects
    ]
    return sorted(rooms, key=lambda room: room[0].name)


def boundary_polygon(obj):
    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        bm.transform(obj.matrix_world)

        unused = {e for e in bm.edges if len(e.link_faces) == 1}
        edge = next(iter(unused))
        vert = edge.verts[0]
        polygon = []
        while edge is not None:
            unused.discard(edge)
            polygon.append((round(vert.co.x, 2), round(vert.co.y, 2)))
            vert = edge.other_vert(vert)
            edge = next((e for e in vert.link_edges if e in unused), None)

        if unused:
            raise ValueError(f"{obj.name}: mesh has more than one boundary loop")
        if len(polygon) < 3:
            raise ValueError(f"{obj.name}: boundary loop has only {len(polygon)} vertices")
        return polygon
    finally:
        bm.free()


def contains(polygon, px, py):
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def waypoint_of(obj, polygon, waypoints):
    inside = [name for name, (x, y) in waypoints.items() if contains(polygon, x, y)]
    if len(inside) != 1:
        raise ValueError(f"{obj.name}: expected exactly one waypoint inside, got {inside}")
    return inside[0]


def to_wkt(polygon):
    coords = ", ".join(f"{x} {y}" for x, y in polygon + [polygon[0]])
    return f"POLYGON(({coords}))"


print("-" * 30)

waypoints = read_waypoints()
rooms = read_rooms()
print(f"Found: {len(waypoints)} waypoints, {len(rooms)} rooms")

with create_engine(DATABASE_URL).begin() as conn:
    poi_ids = {name: poi_id for poi_id, name in conn.execute(text("SELECT id, name FROM points_of_interest"))}
    print(f"Found: {len(poi_ids)} points of interest in DB")

    conn.execute(text("DELETE FROM search_zones"))

    for obj, room_type in rooms:
        polygon = boundary_polygon(obj)
        name = waypoint_of(obj, polygon, waypoints)
        poi_id = poi_ids[name]

        conn.execute(INSERT_ZONE, {"type": room_type, "poi_id": poi_id, "wkt": to_wkt(polygon)})
        obj.name = name
        print(f"  OK: {name:20} | Type: {room_type:10} | poi_id: {poi_id} | Vertices: {len(polygon)}")

    invalid = conn.execute(SELECT_INVALID).fetchall()
    if invalid:
        raise ValueError("Invalid polygons: " + "; ".join(f"{name}: {reason}" for name, reason in invalid))

print("=" * 30)
print(f"Saved: {len(rooms)} search zones")
