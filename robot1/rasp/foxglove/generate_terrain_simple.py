#!/usr/bin/env python3
"""
Génère un GLB très simple pour le terrain — juste un quad gris.
Sans texture → beaucoup plus léger et rapide à charger.
La texture peut être appliquée via projection plane en parallèle si besoin.
"""

import struct
import json
from pathlib import Path

try:
    from pygltflib import (
        GLTF2, Scene, Node, Mesh, Primitive, Accessor,
        BufferView, Buffer, Material, Asset
    )
except ImportError:
    print("❌ pygltflib non installé.")
    print("Installez avec : pip install pygltflib")
    exit(1)

# ──────────────────────────────────────────────────────────────────
# Paramètres du terrain
# ──────────────────────────────────────────────────────────────────

TABLE_LENGTH_M = 3.0  # X
TABLE_WIDTH_M  = 2.0  # Y
OUTPUT_PATH    = Path(__file__).parent / "terrain_simple.glb"

# ──────────────────────────────────────────────────────────────────
# Construction du glTF2 ULTRA-SIMPLE
# ──────────────────────────────────────────────────────────────────

gltf = GLTF2()
gltf.asset = Asset(version="2.0")

# 4 vertices du quad terrain (3m x 2m centré en (0,0))
half_x = TABLE_LENGTH_M / 2.0  # 1.5
half_y = TABLE_WIDTH_M / 2.0   # 1.0

vertices = [
    -half_x, -half_y, 0.0,
    +half_x, -half_y, 0.0,
    +half_x, +half_y, 0.0,
    -half_x, +half_y, 0.0,
]

indices = [0, 1, 2, 0, 2, 3]

# Sérialisation en binaire
v_bytes = struct.pack(f"{len(vertices)}f", *vertices)
i_bytes = struct.pack(f"{len(indices)}H", *indices)

buf_data = v_bytes + i_bytes

# Buffer unique
gltf.buffers = [
    Buffer(
        byteLength=len(buf_data),
        uri="data:application/octet-stream;base64," + 
            __import__('base64').b64encode(buf_data).decode()
    )
]

# BufferViews
gltf.bufferViews = [
    BufferView(
        buffer=0,
        byteOffset=0,
        byteLength=len(v_bytes)
    ),
    BufferView(
        buffer=0,
        byteOffset=len(v_bytes),
        byteLength=len(i_bytes)
    ),
]

# Accessors
gltf.accessors = [
    Accessor(
        bufferView=0,
        componentType=5126,  # FLOAT
        count=4,
        type="VEC3",
        min=[-half_x, -half_y, 0.0],
        max=[+half_x, +half_y, 0.0]
    ),
    Accessor(
        bufferView=1,
        componentType=5123,  # UNSIGNED_SHORT
        count=6,
        type="SCALAR"
    ),
]

# Material : gris uni (pas de texture)
gltf.materials = [
    Material(
        pbrMetallicRoughness={
            "baseColorFactor": [0.5, 0.5, 0.5, 1.0],  # Gris
            "metallicFactor": 0.0,
            "roughnessFactor": 0.8
        },
        doubleSided=True
    )
]

# Mesh + Primitive
gltf.meshes = [
    Mesh(
        primitives=[
            Primitive(
                attributes={"POSITION": 0},
                indices=1,
                material=0
            )
        ]
    )
]

# Node
gltf.nodes = [Node(mesh=0)]

# Scene
gltf.scenes = [Scene(nodes=[0])]
gltf.scene = 0

# ──────────────────────────────────────────────────────────────────
# Sauvegarde
# ──────────────────────────────────────────────────────────────────

print(f"💾 Génération : {OUTPUT_PATH}")
gltf.save(str(OUTPUT_PATH))

file_size = OUTPUT_PATH.stat().st_size
print(f"✅ Succès ! Taille : {file_size / 1024:.1f} KB (ULTRA-LÉGER)")
print(f"\nCe GLB est prêt pour Foxglove sans problème de taille.")
print(f"Pour une texture visuelle, utilise robot/map_image en parallèle.")
