"""
lidar.py
Connexion USB au RPLIDAR A2M12 et conversion mathématique des points.
"""
import math

try:
    from rplidar import RPLidar
except ImportError:
    print("ATTENTION : La librairie rplidar n'est pas installée. Tapez: pip install rplidar-roboticia")

# CORRECTIF : Seuils adaptés au terrain de compétition (3000 x 2000 mm)
# La diagonale max du terrain est ~3606 mm, on garde une marge de sécurité.
LIDAR_DIST_MIN_MM = 150
LIDAR_DIST_MAX_MM = 3500
LIDAR_QUALITE_MIN = 10

def calc_absolute_pos(robot_x, robot_y, robot_theta_rad, dist_mm, angle_rel_deg):
    """Convertit la distance et l'angle du RPLIDAR en coordonnées (X, Y) absolues."""
    angle_rel_rad = math.radians(angle_rel_deg)
    angle_global = robot_theta_rad + angle_rel_rad
    
    abs_x = robot_x + dist_mm * math.cos(angle_global)
    abs_y = robot_y + dist_mm * math.sin(angle_global)
    
    return abs_x, abs_y

class GestionnaireLidar:
    def __init__(self, port_usb='/dev/ttyUSB0',
                 dist_min=LIDAR_DIST_MIN_MM,
                 dist_max=LIDAR_DIST_MAX_MM,
                 qualite_min=LIDAR_QUALITE_MIN):
        """
        Initialise la connexion USB avec le vrai RPLIDAR.
        
        Args:
            port_usb   : Port série du LiDAR (ex: '/dev/ttyUSB0')
            dist_min   : Distance minimale de détection en mm (ignore les points trop proches)
            dist_max   : Distance maximale de détection en mm — à adapter à votre terrain
            qualite_min: Seuil de qualité minimum d'un point LiDAR (0-15)
        """
        # CORRECTIF : Seuils configurables à la construction pour s'adapter au terrain
        self.dist_min = dist_min
        self.dist_max = dist_max
        self.qualite_min = qualite_min

        try:
            self.lidar = RPLidar(port_usb)
            self.iterator = self.lidar.iter_scans()
            print(f"RPLIDAR connecté avec succès sur {port_usb}")
        except Exception as e:
            print(f"Impossible de se connecter au RPLIDAR sur {port_usb}. Erreur: {e}")
            self.lidar = None

    def obtenir_obstacles_absolus(self, robot_x, robot_y, robot_theta_rad):
        """Lit un tour complet du LiDAR et renvoie les obstacles en coordonnées absolues."""
        obstacles_absolus = []
        
        if not self.lidar:
            return obstacles_absolus

        try:
            scan = next(self.iterator)
            
            for point in scan:
                qualite = point[0]
                angle_deg = point[1]
                dist_mm = point[2]
                
                # CORRECTIF : Utilise les seuils configurables au lieu de valeurs figées
                if qualite > self.qualite_min and self.dist_min < dist_mm < self.dist_max:
                    obs_x, obs_y = calc_absolute_pos(
                        robot_x, robot_y, robot_theta_rad, dist_mm, angle_deg
                    )
                    obstacles_absolus.append((obs_x, obs_y))
                    
        except StopIteration:
            # Si le buffer est vide, on relance l'itérateur pour éviter le crash
            self.iterator = self.lidar.iter_scans()
        except Exception as e:
            print(f"Erreur de lecture LiDAR : {e}")
            
        return obstacles_absolus

    def arreter(self):
        """Coupe proprement le moteur du LiDAR."""
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            print("RPLIDAR déconnecté proprement.")