# Clarification: Foxglove n'effectue PAS la fusion

## Architecture à savoir

```
CAPTEURS BRUTS
    ↓
    ├─ Teensy: UPDATE_ROLLING_BASIS (x_odom, y_odom, θ_brut)
    ├─ Lidar: LIDAR_POSITION (x_lidar, y_lidar, confidence)
    └─ IMU: IMU_ANGLE (θ_imu) ← SOURCE UNIQUE POUR L'ANGLE
    ↓
═════════════════════════════════════════════════════
                PATH FINDING (VOS LOGIQUES)
═════════════════════════════════════════════════════
    Fusion réelle:
    • x_fused, y_fused = fusion(odom, lidar)
    • θ_fused = imu_theta
    ↓
    Décisions:
    • Navigation vers target
    • Détection collisions
    • Évitement obstacles
    ↓
    Commandes moteurs
═════════════════════════════════════════════════════
    ↓ (optionnel)
FOXGLOVE (monitoring)
    • Affiche 4 marqueurs
    • Courbes temps réel
    • Debug/visualisation
    • NE fait PAS la fusion!
```

---

## Différence clé

### ❌ AVANT: Fusion dans Foxglove (mauvais)
```
Capteurs → Foxglove (fusion ici) → Affichage
              ↓
          Les données finalisées ne vont nulle part sauf affichage
          Path Finding doit refaire la fusion lui-même 😞
```

### ✅ APRÈS: Fusion dans Path Finding (correct)
```
Capteurs → Path Finding (fusion ici) → Commandes Teensy
              ↓ (data finalisées)
              Foxglove (monitoring seulement)
```

---

## Code: Où faire la fusion

### Dans votre Navigator/Path Finding:

```python
class Navigator:
    """Logique de navigation (c'est ici qu'on fusionne)."""
    
    def __init__(self, bridge=None):
        self.odom = None           # Reçoit: (x, y) du Teensy
        self.lidar = None          # Reçoit: (x, y) du Lidar
        self.imu_theta = 0.0       # Reçoit: θ de l'IMU
        self.confidence = 0.0      # Reçoit: confiance du Lidar
        self.target = (0, 0)       # Cible de navigation
        self.bridge = bridge       # Référence à Foxglove (optionnel)
    
    def fuse_position(self):
        """LA FUSION SE FAIT ICI! (pas dans Foxglove)."""
        
        # Stratégie simple: si Lidar valide, faire confiance à 60%
        if self.confidence > 0.2:
            x_fused = 0.6 * self.lidar[0] + 0.4 * self.odom[0]
            y_fused = 0.6 * self.lidar[1] + 0.4 * self.odom[1]
        else:
            # Lidar absent: utiliser odométrie seule
            x_fused = self.odom[0]
            y_fused = self.odom[1]
        
        # L'angle vient TOUJOURS de l'IMU (plus fiable)
        return (x_fused, y_fused, self.imu_theta)
    
    def navigate(self):
        """Logique de navigation (utilise position fusionnée)."""
        
        # Récupérer position fusionnée
        x_fused, y_fused, theta = self.fuse_position()
        
        # Calculs de navigation
        dx = self.target[0] - x_fused
        dy = self.target[1] - y_fused
        distance_to_target = hypot(dx, dy)
        
        # Détection collision
        if distance_to_target < 100:  # mm
            logger.info("⚠️ Proche de la cible!")
        
        # ... envoyer commandes au Teensy ...
        # (basées sur position fusionnée)
        
        # Optionnel: publier état pour monitoring Foxglove
        if self.bridge:
            self.bridge.publish_data(
                target_x=self.target[0],
                target_y=self.target[1],
                odom_x=self.odom[0],
                odom_y=self.odom[1],
                lidar_x=self.lidar[0],
                lidar_y=self.lidar[1],
                lidar_confidence=self.confidence,
                imu_theta=self.imu_theta
            )

# Utilisation
navigator = Navigator(bridge=bridge)

# Boucle principale
while True:
    # 1. Capteurs mettent à jour l'état (callbacks)
    # (faits automatiquement par les callbacks)
    
    # 2. Navigation (où la fusion se fait!)
    navigator.navigate()
    
    time.sleep(0.05)  # 20 Hz
```

---

## Responsabilités

| Composant | Responsabilité | Détails |
|-----------|----------------|---------|
| **Teensy** | Envoyer odométrie | UPDATE_ROLLING_BASIS (x, y, θ_brut) |
| **Lidar** | Envoyer balises détectées | LIDAR_POSITION (x, y, confidence) |
| **IMU** | Envoyer angle fiable | IMU_ANGLE (θ) |
| **Path Finding** | **Fusion & Navigation** | **Fusion=60% lidar+40% odom. Angle=IMU toujours.** |
| **Foxglove** | Monitoring visualisation | Affiche 4 marqueurs. NE fusionne PAS. |

---

## Pourquoi cette architecture?

### ✅ Avantages
1. **Séparation des responsabilités**: Chaque module a un rôle clair
2. **Réutilisabilité**: La fusion du Path Finding peut servir ailleurs
3. **Testabilité**: Tester la fusion sans Foxglove
4. **Performance**: Foxglove n'impacte pas la logique de navigation
5. **Clarté**: Code lisible et maintenable

### ❌ Éviter cette erreur
```python
# MAUVAIS: Foxglove qui fusionne
def _fuse_in_foxglove(self):
    x = 0.6*lidar + 0.4*odom  # ← Mauvais endroit!
    publish_to_server(x)
```
→ Path Finding ne sait pas où chercher la position fusionnée

---

## Checklist: Formation finale

Avant de considérer terminé:

- ✅ Path Finding a une méthode `fuse_position()` 
- ✅ L'angle vient TOUJOURS de l'IMU (pas de drift)
- ✅ Fusion = moyenne pondérée ou Kalman (selon complexité)
- ✅ Foxglove affiche les 4 positions (brutes + fusion)
- ✅ Foxglove NE modifie NI data NI logique
- ✅ Callbacks mettent à jour Navigator automatiquement
- ✅ Logs affichent position fusionnée actuelle

---

## Prochaines étapes

1. ✅ Créer Navigator avec `fuse_position()`
2. ✅ Ajouter les 3 messages manquants (LIDAR_POSITION, IMU_ANGLE, TARGET_POSITION)
3. ✅ Enregistrer les callbacks
4. ✅ Tester avec données synthétiques d'abord
5. ✅ Intégrer avec hardware réel
6. ⏳ Ajouter Kalman filter si dérive observée
7. ⏳ Ajouter détection avancée obstacles

---

## Questions courantes

**Q: Pourquoi ne pas faire la fusion dans Foxglove?**  
R: Parce que Foxglove n'est qu'un affichage. Path Finding a besoin de la position fusionnée continuellement pour naviguer.

**Q: Et si je n'ai pas Path Finding?**  
R: Créez une classe Navigator/Fusion. C'est le cœur de votre robot!

**Q: Foxglove est optionnel?**  
R: Oui. Path Finding fonctionne sans. Foxglove = monitoring en temps réel, pas obligatoire.

**Q: Quand utiliser Kalman filter?**  
R: Si vous observez une dérive même après moyenne pondérée. Sinon, la stratégie simple suffira.

---

**Date**: 2026-04-08  
**Statut**: Clarification architecture (Foxglove = viewer seul)
