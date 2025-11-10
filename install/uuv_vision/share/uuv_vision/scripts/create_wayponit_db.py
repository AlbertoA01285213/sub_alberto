#!/usr/bin/env python3
import sqlite3
import os

DB_FILE = "waypoints.db"

# ¡Aquí puedes añadir todos los puntos que quieras!
example_points = [
    (5.0, 5.0),
    (-3.0, 4.0),
    (2.0, -4.0),
    (-5.0, -1.0),
    (6.0, 0.0),
    (7.0, 8.0),
    (-5.0, 2.0),
    (10.0, -3.0)
]

def create_database():
    # Eliminar la base de datos si ya existe, para un inicio limpio
    if os.path.exists(DB_FILE):
        os.remove(DB_FILE)
        print(f"Base de datos antigua '{DB_FILE}' eliminada.")

    conn = None
    try:
        # Conectarse (esto creará el archivo si no existe)
        conn = sqlite3.connect(DB_FILE)
        cursor = conn.cursor()
        
        # 1. Crear la tabla 'waypoints'
        cursor.execute('''
        CREATE TABLE waypoints (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            x REAL NOT NULL,
            y REAL NOT NULL
        )
        ''')
        
        print("Tabla 'waypoints' creada.")
        
        # 2. Insertar los puntos de ejemplo
        cursor.executemany("INSERT INTO waypoints (x, y) VALUES (?, ?)", example_points)
        
        # 3. Guardar los cambios (commit)
        conn.commit()
        print(f"{len(example_points)} waypoints insertados en la base de datos '{DB_FILE}'.")
        
    except sqlite3.Error as e:
        print(f"Error al crear la base de datos SQLite: {e}")
    finally:
        if conn:
            conn.close()

if __name__ == '__main__':
    create_database()