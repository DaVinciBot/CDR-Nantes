#!/usr/bin/env python3
"""
Génère un fichier terrain.glb avec la texture intégrée.
À exécuter une seule fois pour créer le modèle 3D du terrain.

Usage:
    python generate_terrain_glb.py
"""

import base64
import struct
import json
from pathlib import Path

# Tentative d'import de pygltflib
try:
    from pygltflib import (
        GLTF2, Scene, Node, Mesh, Primitive, Accessor,
        BufferView, Buffer, Material, Image, Texture, TextureInfo, Asset
    )
except ImportError:
    print("❌ pygltflib non installé.")
    print("Installez avec : pip install pygltflib")
    exit(1)

# ──────────────────────────────────────────────────────────────────
# Paramètres du terrain
# ──────────────────────────────────────────────────────────────────

# Terrain CDR 2026 : 3m x 2m
TABLE_LENGTH_M = 3.0  # X
TABLE_WIDTH_M  = 2.0  # Y

# Texture - compresser en JPEG de plus basse qualité si trop volumineux
TEXTURE_PATH = Path(__file__).parent / "map_assets" / "eurobot2026" / "textures" / "playmat_2026.jpg"
OUTPUT_PATH  = Path(__file__).parent / "terrain.glb"

# Optionnel: créer une version compressée de la texture avant GLB
COMPRESS_TEXTURE = False  # Mettre à True si le GLB est trop gros

# ──────────────────────────────────────────────────────────────────
# Vérifications
# ──────────────────────────────────────────────────────────────────

if not TEXTURE_PATH.exists():
    print(f"❌ Texture introuvable : {TEXTURE_PATH}")
    exit(1)

print(f"📦 Chargement texture : {TEXTURE_PATH}")
img_data = TEXTURE_PATH.read_bytes()
print(f"   Taille : {len(img_data) / 1024:.1f} KB")

# ──────────────────────────────────────────────────────────────────
# Construction du glTF2
# ──────────────────────────────────────────────────────────────────

gltf = GLTF2()
gltf.asset = Asset(version="2.0")

# 4 vertices du quad terrain (3m x 2m centré en (0,0))
# Position Z = 0 (au sol) pour que z=-0.001 dans Foxglove laisse place aux objets dessus
half_x = TABLE_LENGTH_M / 2.0  # 1.5
half_y = TABLE_WIDTH_M / 2.0   # 1.0

vertices = [
    -half_x, -half_y, 0.0,
    +half_x, -half_y, 0.0,
    +half_x, +half_y, 0.0,
    -half_x, +half_y, 0.0,
]

uvs = [
    0.0, 1.0,  # coin bas-gauche
    1.0, 1.0,  # coin bas-droite
    1.0, 0.0,  # coin haut-droite
    0.0, 0.0,  # coin haut-gauche
]

indices = [0, 1, 2, 0, 2, 3]

# Sérialisation en binaire
v_bytes  = struct.pack(f"{len(vertices)}f", *vertices)
uv_bytes = struct.pack(f"{len(uvs)}f", *uvs)
i_bytes  = struct.pack(f"{len(indices)}H", *indices)

buf_data = v_bytes + uv_bytes + i_bytes

# Buffer unique contenant vertices + UVs + indices
gltf.buffers = [
    Buffer(
        byteLength=len(buf_data),
        uri="data:application/octet-stream;base64," + base64.b64encode(buf_data).decode()
    )
]

# BufferViews : découpage du buffer
gltf.bufferViews = [
    BufferView(
        buffer=0,
        byteOffset=0,
        byteLength=len(v_bytes)
    ),
    BufferView(
        buffer=0,
        byteOffset=len(v_bytes),
        byteLength=len(uv_bytes)
    ),
    BufferView(
        buffer=0,
        byteOffset=len(v_bytes) + len(uv_bytes),
        byteLength=len(i_bytes)
    ),
]

# Accessors : interprétation des données
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
        componentType=5126,  # FLOAT
        count=4,
        type="VEC2"
    ),
    Accessor(
        bufferView=2,
        componentType=5123,  # UNSIGNED_SHORT
        count=6,
        type="SCALAR"
    ),
]

# Image (texture embarquée en base64)
gltf.images = [
    Image(
        uri="data:image/jpeg;base64," + base64.b64encode(img_data).decode()
    )
]

# Texture
gltf.textures = [Texture(source=0)]

# Material : PBR simple
gltf.materials = [
    Material(
        pbrMetallicRoughness={
            "baseColorTexture": {"index": 0},
            "metallicFactor": 0.0,
            "roughnessFactor": 1.0
        },
        doubleSided=True
    )
]

# Mesh + Primitive
gltf.meshes = [
    Mesh(
        primitives=[
            Primitive(
                attributes={"POSITION": 0, "TEXCOORD_0": 1},
                indices=2,
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

print(f"\n💾 Génération : {OUTPUT_PATH}")
gltf.save(str(OUTPUT_PATH))

file_size = OUTPUT_PATH.stat().st_size
print(f"✅ Succès ! Taille : {file_size / 1024:.1f} KB")
print(f"\nPour utiliser dans Foxglove :")
print(f"  - Le fichier sera chargé depuis : foxglove/terrain.glb")
print(f"  - Assurez-vous que foxglove_3d_bridge.py le découvre")
