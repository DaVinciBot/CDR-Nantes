#!/usr/bin/env python3
"""
Diagnostic du GLB — Vérifier que tout est bien chargé et encodé.
"""

import sys
from pathlib import Path
import base64
import json

sys.path.insert(0, str(Path(__file__).parent))

print("=" * 60)
print("DIAGNOSTIC TERRAIN GLB")
print("=" * 60)
print(f"   Existe: {glb_path.exists()}")
if glb_path.exists():
    size_bytes = glb_path.stat().st_size
    size_mb = size_bytes / (1024 * 1024)
    print(f"   Taille: {size_bytes:,} bytes ({size_mb:.2f} MB)")
    
    # Vérifier la magie
    magic = glb_path.read_bytes()[:4]
    print(f"   Magic: {magic} ({'✅ glTF' if magic == b'glTF' else '❌ INVALIDE'})")
else:
    print("   ❌ FILE MISSING!")
    sys.exit(1)

# 2. Tester le helper
print(f"\n2️⃣  Charger avec terrain_glb_loader:")
try:
    from terrain_glb_loader import load_terrain_glb
    glb_model = load_terrain_glb(Path(__file__).parent)
    
    if glb_model:
        print("   ✅ load_terrain_glb() retourné")
        
        data = glb_model.get("data", "")
        print(f"   - data length: {len(data)} chars")
        print(f"   - media_type: {glb_model.get('media_type')}")
        print(f"   - pose z: {glb_model['pose']['position']['z']}")
        
        if len(data) == 0:
            print("   ❌ ERREUR: data vide!")
        else:
            # Vérifier si c'est une URI ou du base64
            if data.startswith("file://"):
                print(f"   ✅ Mode URI file://: {data[:50]}...")
            else:
                # Essayer de décoder comme base64
                try:
                    decoded = base64.b64decode(data)
                    print(f"   ✅ Mode BASE64 décodé: {len(decoded):,} bytes")
                    if decoded[:4] == b'glTF':
                        print(f"   ✅ Magic vérifié après décodage")
                    else:
                        print(f"   ❌ Magic INVALIDE après décodage!")
                except Exception as e:
                    print(f"   ⚠️  Pas du base64 (ok si c'est une URI): {e}")
    else:
        print("   ❌ load_terrain_glb() retourné None")
except ImportError as e:
    print(f"   ❌ Import erreur: {e}")

# 3. Tester l'intégration dans build_map_scene_msg()
print(f"\n3️⃣  Tester integration foxglove_3d_bridge:")
try:
    from foxglove_3d_bridge import build_map_scene_msg
    msg = build_map_scene_msg()
    msg_size_kb = len(msg) / 1024
    print(f"   ✅ Message généré: {msg_size_kb:.1f} KB")
    
    # Parser et chercher le GLB
    msg_json = json.loads(msg)
    entities = msg_json.get("entities", [])
    print(f"   - Entités: {len(entities)}")
    
    glb_found = False
    for e in entities:
        if "models" in e and len(e.get("models", [])) > 0:
            for model in e["models"]:
                model_data_len = len(model.get("data", ""))
                print(f"   ✅ GLB trouvé dans entité '{e['id']}': {model_data_len} chars")
                glb_found = True
    
    if not glb_found:
        print("   ⚠️  Pas de GLB trouvé dans le message!")
        
except Exception as e:
    print(f"   ❌ Erreur: {e}")
    import traceback
    traceback.print_exc()

print("\n" + "=" * 60)
print("RECOMMANDATIONS:")
print("=" * 60)
print("""
✅ Si tous les tests passent:
   → Lance: python foxglove_3d_bridge.py
   
❌ Si des tests échouent:
   → Régénère le GLB: python generate_terrain_glb.py
   → Puis relance ce diagnostic
""")
