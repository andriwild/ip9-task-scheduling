import bpy
import math
from mathutils import Vector
from sqlalchemy import create_engine, MetaData, Table, Column, Integer, String, Float
from sqlalchemy.dialects.postgresql import DOUBLE_PRECISION
from geoalchemy2 import Geometry
# --- KONFIGURATION ---
COLLECTION_NAME = "Waypoints"
# Passe den Connection String an deine Umgebung an
DATABASE_URL = "postgresql://wsr_user:wsr_password@localhost:5432/wsr"
# ---------------------
print("-" * 30)
def get_db_engine():
    return create_engine(DATABASE_URL)
def upsert_waypoint(conn, points_of_interest, name, x, y, yaw):
    # Zuerst prüfen, ob der Punkt existiert
    select_stmt = points_of_interest.select().where(points_of_interest.c.name == name)
    result = conn.execute(select_stmt).fetchone()
    
    point_wkt = f"POINT({x} {y})"
    
    if result:
        # Update
        print(f"Updating {name}...")
        update_stmt = points_of_interest.update().where(points_of_interest.c.name == name).values(
            coordinate=point_wkt,
            yaw=yaw
        )
        conn.execute(update_stmt)
    else:
        # Insert
        print(f"Inserting {name}...")
        insert_stmt = points_of_interest.insert().values(
            name=name,
            coordinate=point_wkt,
            yaw=yaw
        )
        conn.execute(insert_stmt)
if COLLECTION_NAME in bpy.data.collections:
    target_collection = bpy.data.collections[COLLECTION_NAME]
    objects_to_process = sorted(target_collection.objects, key=lambda obj: obj.name)
    
    # DB Verbindung aufbauen
    engine = get_db_engine()
    metadata = MetaData()
    
    # Tabellendefinition
    points_of_interest = Table(
        'points_of_interest', metadata,
        Column('id', Integer, primary_key=True),
        Column('name', String, nullable=False),
        Column('coordinate', Geometry("POINT", srid=0), nullable=False),
        Column('yaw', Float, nullable=True),
    )
    
    with engine.connect() as conn:
        # Transaktion starten
        trans = conn.begin()
        try:
            for obj in objects_to_process:
                # KORREKTUR: Kein .lower() mehr
                name = obj.name.replace(" ", "_")
                
                x = round(obj.location.x, 2)
                y = round(obj.location.y, 2)
                
                local_up = Vector((0, 0, 1))
                world_direction = obj.matrix_world.to_3x3() @ local_up
                
                yaw = math.atan2(world_direction.y, world_direction.x)
                yaw = round(yaw, 3)
                
                degree = round(math.degrees(yaw), 1)
                print(f"Processing: {name:15} | X: {x:6} | Y: {y:6} | Yaw: {yaw} ({degree}°)")
                
                upsert_waypoint(conn, points_of_interest, name, x, y, yaw)
            
            trans.commit()
            print("Successfully saved to database.")
            
        except Exception as e:
            trans.rollback()
            print(f"Error: {e}")
            
else:
    print(f"FEHLER: Collection '{COLLECTION_NAME}' nicht gefunden!")
