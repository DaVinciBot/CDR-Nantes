#!/usr/bin/env python3
"""
Gestionnaire LIDAR pour robot CDR-Nantes
Récupère les 360 points du LIDAR (compatibilité RPLIDAR A2M8)

Usage:
    from lidar_manager import LidarManager
    
    def on_scan(lidar_data):
        # lidar_data = {'angles': [0-359], 'distances': [dist en mm], 'timestamp': ms}
        for angle, distance in zip(lidar_data['angles'], lidar_data['distances']):
            print(f"Angle {angle}°: {distance}mm")
    
    lidar = LidarManager(com, on_scan_callback=on_scan)
"""

import struct


class LidarManager:
    """
    Gestionnaire LIDAR - Réception des 360 points en 4 parties
    
    Le LIDAR envoie 360 points divisés en 4 messages (90 points chacun):
    - Part 1: Angles 0-89°
    - Part 2: Angles 90-179°
    - Part 3: Angles 180-269°
    - Part 4: Angles 270-359° + timestamp
    """
    
    def __init__(self, com, on_scan_callback=None):
        """
        Initialise le gestionnaire LIDAR
        
        Args:
            com: Instance de Com pour la communication
            on_scan_callback: (optionnel) Fonction appelée à chaque scan complet
        """
        self.com = com
        self.on_scan_callback = on_scan_callback
        
        # Buffer pour les 4 parties
        self._buffer = {
            'part1': None,  # 0-89°
            'part2': None,  # 90-179°
            'part3': None,  # 180-269°
            'part4': None,  # 270-359° + timestamp
        }
        
        # Stockage du dernier scan reçu
        self.last_scan = None
        self.points = []  # Liste de tuples (angle, distance)
        self.angles = []  # Liste des 360 angles
        self.distances = []  # Liste des 360 distances
        
        # Statistiques
        self.scans_received = 0
        self.last_timestamp = 0
        
        # Enregistrement des callbacks (IDs 130-133)
        self._register_callbacks()
    
    def _register_callbacks(self):
        """Enregistre les callbacks pour les 4 parties du LIDAR"""
        # Import local pour éviter dépendance circulaire
        import sys
        from pathlib import Path
        WORKSPACE = Path(__file__).resolve().parent.parent.parent.parent
        sys.path.insert(0, str(WORKSPACE / "common" / "usb_com" / "python"))
        from loader import loader
        Messages = loader.load_class('usb_com', 'Messages')
        
        self.com.add_callback(self._on_part1, Messages.LIDAR_SCAN_PART1.value)
        self.com.add_callback(self._on_part2, Messages.LIDAR_SCAN_PART2.value)
        self.com.add_callback(self._on_part3, Messages.LIDAR_SCAN_PART3.value)
        self.com.add_callback(self._on_part4, Messages.LIDAR_SCAN_PART4.value)
    
    def _parse_part(self, payload, part_num):
        """
        Décode une partie du LIDAR (90 points en uint16)
        
        Args:
            payload: bytes reçus
            part_num: 1, 2, 3 ou 4
            
        Returns:
            list[int] pour parts 1-3
            dict{'distances': list[int], 'timestamp': int} pour part 4
            None si erreur
        """
        if part_num < 4:
            # Parts 1-3: 90 × uint16 = 180 bytes
            if len(payload) != 180:
                return None
            return list(struct.unpack('<90H', payload))
        else:
            # Part 4: 90 × uint16 + 1 × uint32 = 184 bytes
            if len(payload) != 184:
                return None
            data = struct.unpack('<90HI', payload)
            return {
                'distances': list(data[0:90]),
                'timestamp': data[90]
            }
    
    def _on_part1(self, payload):
        """Callback pour partie 1 (angles 0-89°)"""
        result = self._parse_part(payload, 1)
        if result:
            self._buffer['part1'] = result
            self._process_scan()
    
    def _on_part2(self, payload):
        """Callback pour partie 2 (angles 90-179°)"""
        result = self._parse_part(payload, 2)
        if result:
            self._buffer['part2'] = result
            self._process_scan()
    
    def _on_part3(self, payload):
        """Callback pour partie 3 (angles 180-269°)"""
        result = self._parse_part(payload, 3)
        if result:
            self._buffer['part3'] = result
            self._process_scan()
    
    def _on_part4(self, payload):
        """Callback pour partie 4 (angles 270-359° + timestamp)"""
        result = self._parse_part(payload, 4)
        if result:
            self._buffer['part4'] = result
            self._process_scan()
    
    def _process_scan(self):
        """Traite le scan complet si toutes les parties sont reçues"""
        # Vérifier que toutes les parties sont présentes
        if None in self._buffer.values():
            return
        
        # Reconstituer les 360 points
        distances = (
            self._buffer['part1'] +
            self._buffer['part2'] +
            self._buffer['part3'] +
            self._buffer['part4']['distances']
        )
        
        timestamp = self._buffer['part4']['timestamp']
        
        # Réinitialiser le buffer
        self._buffer = {k: None for k in self._buffer.keys()}
        
        # Créer la structure de données LIDAR
        lidar_data = {
            'angles': list(range(360)),  # 0-359°
            'distances': distances,       # mm (0 = invalide)
            'timestamp': timestamp        # ms
        }
        
        # Stocker dans l'instance (accès direct comme vrai LIDAR)
        self.last_scan = lidar_data
        self.angles = lidar_data['angles']
        self.distances = lidar_data['distances']
        self.points = list(zip(self.angles, self.distances))
        
        # Mise à jour statistiques
        self.scans_received += 1
        self.last_timestamp = timestamp
        
        # Appeler le callback utilisateur si défini
        if self.on_scan_callback:
            self.on_scan_callback(lidar_data)
    
    def get_stats(self):
        """Retourne les statistiques du LIDAR"""
        return {
            'scans_received': self.scans_received,
            'last_timestamp': self.last_timestamp
        }
