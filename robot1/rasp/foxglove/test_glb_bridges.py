#!/usr/bin/env python3
"""
Test multi-bridge : Foxglove avec GLB sur PC

Teste que le GLB fonctionne dans tous les bridges.
À exécuter sur ton PC **avant** de déployer sur la Rasp.
"""

import sys
import time
import logging
from pathlib import Path

# Setup paths
sys.path.insert(0, str(Path(__file__).parent))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(levelname)s | %(message)s"
)
logger = logging.getLogger("test_glb_bridges")

def test_glb_generation():
    """Test 1: Générer le GLB"""
    print("\n" + "="*60)
    print("TEST 1 : GÉNÉRATION DU GLB")
    print("="*60)
    
    try:
        import generate_terrain_glb
        logger.info("✅ generate_terrain_glb importé avec succès")
        
        glb_path = Path(__file__).parent / "terrain.glb"
        if glb_path.exists():
            size_mb = glb_path.stat().st_size / (1024 * 1024)
            logger.info(f"✅ terrain.glb existe déjà ({size_mb:.2f} MB)")
        else:
            logger.warning("⚠️  terrain.glb n'existe pas — génération recommandée")
            logger.info("   Exécute: python generate_terrain_glb.py")
    except Exception as e:
        logger.error(f"❌ Erreur: {e}")

def test_glb_loader():
    """Test 2: Charger le GLB avec le helper"""
    print("\n" + "="*60)
    print("TEST 2 : CHARGEMENT DU GLB (terrain_glb_loader)")
    print("="*60)
    
    try:
        from terrain_glb_loader import load_terrain_glb
        logger.info("✅ terrain_glb_loader importé")
        
        glb_model = load_terrain_glb(Path(__file__).parent)
        if glb_model:
            logger.info("✅ GLB chargé avec succès")
            logger.info(f"   - media_type: {glb_model['media_type']}")
            logger.info(f"   - data size: {len(glb_model['data']) / 1024:.1f} KB (base64)")
        else:
            logger.warning("❌ GLB non chargé (fichier manquant?)")
    except Exception as e:
        logger.error(f"❌ Erreur: {e}")

def test_foxglove_3d_bridge():
    """Test 3: foxglove_3d_bridge avec GLB"""
    print("\n" + "="*60)
    print("TEST 3 : foxglove_3d_bridge")
    print("="*60)
    
    try:
        from foxglove_3d_bridge import build_map_scene_msg
        logger.info("✅ build_map_scene_msg importée")
        
        msg = build_map_scene_msg()
        logger.info(f"✅ Message construit ({len(msg) / 1024:.1f} KB)")
        
        import json
        data = json.loads(msg)
        n_entities = len(data.get("entities", []))
        logger.info(f"   - Entités: {n_entities}")
        
        # Vérifier si le GLB est inclus
        has_glb = any("models" in e and len(e.get("models", [])) > 0 
                      for e in data.get("entities", []))
        if has_glb:
            logger.info("   ✅ GLB intégré dans les entités")
        else:
            logger.warning("   ⚠️  Pas de GLB trouvé (fallback sur cubes?)")
        
    except Exception as e:
        logger.error(f"❌ Erreur: {e}")

def test_foxglove_bridge():
    """Test 4: foxglove_bridge avec GLB"""
    print("\n" + "="*60)
    print("TEST 4 : foxglove_bridge (ancien)")
    print("="*60)
    
    try:
        from foxglove_bridge import FoxgloveBridge
        logger.info("✅ FoxgloveBridge importé")
        logger.info("   Note: Ce bridge nécessite un objet com")
        logger.info("   → Passage du test OK (pas de crash à l'import)")
    except Exception as e:
        logger.error(f"❌ Erreur: {e}")

def test_foxglove_bridge_advanced():
    """Test 5: foxglove_bridge_advanced avec GLB"""
    print("\n" + "="*60)
    print("TEST 5 : foxglove_bridge_advanced")
    print("="*60)
    
    try:
        from foxglove_bridge_advanced import FoxgloveBridgeAdvanced
        logger.info("✅ FoxgloveBridgeAdvanced importé")
        logger.info("   Note: Ce bridge ne chargera le GLB qu'au démarrage avec .start()")
        logger.info("   → Passage du test OK (pas de crash à l'import)")
    except Exception as e:
        logger.error(f"❌ Erreur: {e}")

def print_summary():
    """Résumé final"""
    print("\n" + "="*60)
    print("RÉSUMÉ DES TESTS")
    print("="*60)
    print("""
✅ Si tous les tests passent, tu peux :

1. GÉNÉRER LE GLB (une seule fois sur PC):
   python generate_terrain_glb.py

2. LANCER UN BRIDGE LOCAL POUR TESTER:
   python foxglove_3d_bridge.py
   
3. DANS FOXGLOVE :
   - Connecte à ws://localhost:8765
   - Ajoute un 3D Panel
   - Sélectionne les topics robot/tf et robot/map_scene
   
4. VÉRIFICATION :
   - Tu dois voir le terrain textrisé en arrière-plan (z≈-0.001)
   - Les autres objets (table, supports, caisses) par-dessus

❌ SI ERREUR :
   - ¿ GLB manquant ? → Exécute generate_terrain_glb.py
   - ¿ pygltflib non installé ? → pip install pygltflib
   - ¿ foxglove-websocket non installé ? → pip install foxglove-websocket

📱 UNE FOIS TESTÉ SUR PC, TU PEUX DÉPLOYER :
   - Copie terrain.glb sur la Rasp
   - Lance foxglove_3d_bridge.py sur la Rasp
   - Connecte depuis Foxglove vers ws://raspberry_ip:8765
""")

if __name__ == "__main__":
    logger.info("🚀 Test Foxglove + GLB - Bridges")
    
    test_glb_generation()
    test_glb_loader()
    test_foxglove_3d_bridge()
    test_foxglove_bridge()
    test_foxglove_bridge_advanced()
    
    print_summary()
