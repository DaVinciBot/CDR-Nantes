# Configuration du Terrain GLB pour Foxglove

## Vue d'ensemble

Le terrain du jeu Eurobot 2026 est maintenant rendu en 3D avec un modèle GLB (binary glTF) pour une meilleure performance et une visualisation native dans Foxglove.

### Avantages
- ✅ **Performance** : Un seul fichier au lieu de streaming d'images 2D
- ✅ **Vision 3D native** : Rotation/zoom directement dans Foxglove
- ✅ **Latence zéro** : Entité statique (pas d'updates continues)
- ✅ **Texture embarquée** : Tout inclus dans le fichier `.glb`

## Étapes d'installation

### 1. Installer les dépendances

```bash
pip install -r ../../../requirements.txt
```

Cela installe :
- `foxglove-websocket` : Bridge Foxglove
- `pygltflib` : Génération du fichier GLB

### 2. Générer le fichier `terrain.glb`

Ce script doit être exécuté **une seule fois** sur votre PC (ou sur le Raspberry Pi) :

```bash
cd robot1/rasp/foxglove/
python generate_terrain_glb.py
```

**Résultat attendu** :
```
📦 Chargement texture : ...map_assets/eurobot2026/textures/playmat_2026.jpg
   Taille : 2500.5 KB
💾 Génération : .../terrain.glb
✅ Succès ! Taille : 3400.2 KB
```

Le fichier `terrain.glb` sera créé dans `robot1/rasp/foxglove/`.

### 3. Lancer le bridge

```bash
python foxglove_3d_bridge.py
```

Le bridge se connecte à Foxglove et publie automatiquement les entités 3D :
- Robot (cylindre bleu + flèche direction)
- Terrain (GLB importé)
- Table, murs, supports, caisses, balises

## Validation

1. Connectez-vous à Foxglove WebSocket : `ws://localhost:8765`
2. Ajoutez un **3D Panel** à votre layout
3. Sélectionnez les topics :
   - **robot/tf** → pour les transforms
   - **robot/scene** → pour le robot et sa direction
   - **robot/map_scene** → pour la carte statique (terrain + objets)

### Vérification visuelle
- La carte apparaît au sol (z ≈ -0.001) avec la texture du terrain
- Les caisses/balises s'affichent par-dessus
- Le robot se déplace en temps réel

## Dépannage

### ❌ Le GLB n'est pas trouvé
```
⚠️  Fichier terrain.glb introuvable (attendu à : ...)
```
**Solution** : Exécutez `python generate_terrain_glb.py` pour générer le fichier.

### ❌ pygltflib non installé
```
❌ pygltflib non installé.
Installez avec : pip install pygltflib
```
**Solution** :
```bash
pip install pygltflib
```

### ❌ Texture non trouvée au moment de la génération
```
❌ Texture introuvable : ....playmat_2026.jpg
```
**Solution** : Vérifiez que le fichier existe dans :  
`map_assets/eurobot2026/textures/playmat_2026.jpg`

### ❌ Erreur à la lecture du GLB dans le bridge
```
Erreur chargement GLB terrain : [cause]
```
**Vérifications** :
1. Que le fichier `terrain.glb` n'est pas corrompu (>1 MB généralement)
2. Que les permissions de lecture existent
3. Relancez `python generate_terrain_glb.py` pour régénérer

## Architecture

### Flux de données
```
playmat_2026.jpg (texture)
    ↓
generate_terrain_glb.py
    ↓
terrain.glb (3.5 MB environ)
    ↓
foxglove_3d_bridge.py (charge + encode base64)
    ↓
ws://localhost:8765
    ↓
Foxglove (3D Panel MonitoringCore)
    ↓
Visualisation interactive
```

### Paramètres du terrain
- **Dimensions** : 3.0m × 2.0m (table CDR)
- **Position** : Centrée en (0, 0) du repère `world`
- **Z** : -0.001 (couche de base, laisse place aux objets)
- **Fichier** : `terrain.glb` (environs 3-4 MB)
- **Format** : glTF2.0 binaire avec texture JPEG embarquée

## Maintenance

### Mettre à jour la texture du terrain
1. Remplacez `map_assets/eurobot2026/textures/playmat_2026.jpg`
2. Régénérez : `python generate_terrain_glb.py`
3. Redémarrez le bridge

### Modifier les dimensions
Éditez `generate_terrain_glb.py` :
```python
TABLE_LENGTH_M = 3.0  # Longueur en mètres
TABLE_WIDTH_M  = 2.0  # Largeur en mètres
```
Puis régénérez comme ci-dessus.

## Notes de performance

- **Taille du fichier GLB** : ~3-4 MB (une seule fois au chargement)
- **Bande passante Foxglove** : Minimale (pas d'updates continues)
- **Latence** : Zéro après chargement
- **CPU** : Impact négligeable

## Désactiver le GLB (fallback classique)

Si vous rencontrez des problèmes, vous pouvez temporairement désactiver le GLB en commentant dans `foxglove_3d_bridge.py` :

```python
def build_map_scene_msg() -> bytes:
    # ... setup ...
    
    # À commenter pour fallback :
    # if glb_path.exists(): ...
    
    # Les cubes/cylinders classiques restent actifs
```

---

📝 Dernière mise à jour : Avril 2026  
🤖 Pour CDR Nantes — Eurobot 2026
