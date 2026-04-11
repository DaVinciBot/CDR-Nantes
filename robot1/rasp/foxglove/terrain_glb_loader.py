"""
Utilitaire partagé pour charger le GLB du terrain.
À utiliser dans tous les bridges Foxglove.

Support de deux modes:
1. Data inline (base64) - pour petits fichiers
2. URI file:// - pour fichiers volumineux (recommandé)
"""

import base64
import logging
from pathlib import Path
from typing import Optional
from urllib.request import pathname2url

logger = logging.getLogger("terrain_loader")


def load_terrain_glb(bridge_dir: Path, use_file_uri: bool = True, simple: bool = True) -> Optional[dict]:
    """
    Charge le fichier terrain.glb depuis le répertoire foxglove.
    
    Parameters
    ----------
    bridge_dir : Path
        Répertoire du bridge (contient terrain.glb)
    use_file_uri : bool
        Si True: utilise file:// URI (recommandé pour gros fichiers)
        Si False: utilise data inline base64 (pour petits fichiers)
    simple : bool
        Si True: utilise terrain_simple.glb (quad gris, ultra-léger)
        Si False: utilise terrain.glb (avec texture, plus gros)
    
    Returns
    -------
    dict : ModelPrimitive JSON prêt à utiliser dans foxglove.SceneUpdate
    None : Si le fichier n'existe pas ou erreur de chargement
    
    Exemple
    -------
    glb_model = load_terrain_glb(Path(__file__).parent)
    if glb_model:
        terrain_entity["models"] = [glb_model]
    """
    # Choisir quel fichier GLB utiliser
    if simple:
        glb_path = bridge_dir / "terrain_simple.glb"
        glb_name = "terrain_simple.glb (ultra-léger)"
    else:
        glb_path = bridge_dir / "terrain.glb"
        glb_name = "terrain.glb (avec texture)"
    
    if not glb_path.exists():
        logger.warning("❌ %s introuvable: %s", glb_name, glb_path)
        # Essayer le fallback
        if simple:
            logger.info("   Essai du fallback: terrain.glb")
            return load_terrain_glb(bridge_dir, use_file_uri=use_file_uri, simple=False)
        return None
    
    try:
        file_size_kb = glb_path.stat().st_size / 1024
        
        # Vérifier la magie glTF2 (signature fichier)
        glb_bytes = glb_path.read_bytes()
        if len(glb_bytes) == 0:
            logger.error("❌ Fichier GLB vide (0 bytes)")
            return None
        
        if not glb_bytes.startswith(b'glTF'):
            logger.error("❌ Fichier GLB invalide (magie glTF manquante)")
            return None
        
        # Choisir le mode de distribution
        if use_file_uri:
            # URI mode: beaucoup plus léger!
            # Convertir le chemin en URI file://
            file_uri = glb_path.as_uri()
            logger.info("✅ GLB chargé (URI mode, %s): %.1f KB", glb_name, file_size_kb)
            
            return {
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": -0.001},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
                "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
                "override_color": False,
                "media_type": "model/gltf-binary",
                "data": file_uri,  # ← URI au lieu de base64!
            }
        else:
            # Data mode: base64 inline (pour petits fichiers < 1 MB)
            glb_data_b64 = base64.b64encode(glb_bytes).decode('utf-8')
            
            if not glb_data_b64 or len(glb_data_b64) == 0:
                logger.error("❌ Encodage base64 échoué (résultat vide)")
                return None
            
            b64_size_kb = len(glb_data_b64) / 1024
            logger.info("✅ GLB chargé (DATA mode, %s): %.1f KB binaire → %.1f KB base64", 
                       glb_name, file_size_kb, b64_size_kb)
            
            return {
                "pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": -0.001},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
                "color": {"r": 1.0, "g": 1.0, "b": 1.0, "a": 1.0},
                "override_color": False,
                "media_type": "model/gltf-binary",
                "data": glb_data_b64,  # ← Base64 inline
            }
            
    except Exception as e:
        logger.error("❌ Erreur chargement GLB: %s", e, exc_info=True)
        return None
