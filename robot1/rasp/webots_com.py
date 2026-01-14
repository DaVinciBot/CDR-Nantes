#!/usr/bin/env python3
"""Wrapper simplifié de Com pour la simulation Webots avec ports COM virtuels."""

import struct
import threading
import time
import crc8
from serial import Serial

from loader import loader

# Charger Messages via le loader
Messages = loader.load_class('usb_com', 'Messages')

# Charger END_BYTES_SIGNATURE directement depuis le module configuré par le loader
import importlib
messages_module = importlib.import_module('messages')
END_BYTES_SIGNATURE = messages_module.END_BYTES_SIGNATURE

NACK_ID = 127

class WebotsComBridge:
    """Classe simplifiée pour communication COM virtuelle avec Webots.
    
    Compatible avec l'interface de la classe Com mais fonctionne avec des ports COM virtuels.
    """

    def __init__(self, port: str, baudrate: int = 115200, enable_crc: bool = True, logger=None):
        """Initialise la connexion COM virtuelle.
        
        Args:
            port: Port COM à utiliser (ex: 'COM1')
            baudrate: Vitesse de communication (défaut: 115200)
            enable_crc: Active la vérification CRC8 (défaut: True)
            logger: Logger pour les messages (optionnel)
        """
        self.port = port
        self.baudrate = baudrate
        self.enable_crc = enable_crc
        self.logger = logger
        
        # Connexion série
        self._device = Serial(port, baudrate=baudrate, timeout=0.1)
        self._crc8 = crc8.crc8()
        
        # Callbacks
        self.message_id_callback = {}
        self.last_message = None
        
        # Thread de réception
        self._receiver_thread = threading.Thread(target=self._receiver, name="WebotsComReceiver", daemon=True)
        self._receiver_thread.start()
        
        if self.logger:
            self.logger.info(f"✅ Connexion {port} établie à {baudrate} bauds")

    def _receiver(self):
        """Thread de réception des messages (format compatible avec Com C++)."""
        buffer = bytearray()
        
        while True:
            try:
                # Lire les données disponibles
                if self._device.in_waiting > 0:
                    data = self._device.read(self._device.in_waiting)
                    buffer.extend(data)
                    
                    # Chercher la signature de fin
                    while END_BYTES_SIGNATURE in buffer:
                        end_index = buffer.index(END_BYTES_SIGNATURE)
                        
                        # Le format C++ est : message + taille + CRC + signature
                        # On doit trouver la taille juste avant le CRC
                        if end_index >= 2:  # Au moins 1 byte message + taille + CRC
                            # La taille est à end_index - 2 (avant le CRC)
                            # Le CRC est à end_index - 1
                            if end_index >= 2:
                                message_size = buffer[end_index - 2]
                                received_crc = buffer[end_index - 1]
                                
                                # Vérifier qu'on a assez de données pour le message
                                message_start = end_index - 2 - message_size
                                if message_start >= 0:
                                    # Extraire le message
                                    message_data = bytes(buffer[message_start:message_start + message_size])
                                    
                                    # Vérifier le CRC si activé (calculé sur message + taille)
                                    if self.enable_crc:
                                        crc_data = message_data + bytes([message_size])
                                        self._crc8.reset()
                                        self._crc8.update(crc_data)
                                        calculated_crc = self._crc8.digest()[0]
                                        
                                        if calculated_crc != received_crc:
                                            if self.logger:
                                                self.logger.warning(f"⚠️  CRC invalide: reçu={received_crc:02x}, calculé={calculated_crc:02x}")
                                            # Retirer jusqu'à la signature et continuer
                                            buffer = buffer[end_index + len(END_BYTES_SIGNATURE):]
                                            continue
                                    
                                    # Message valide, extraire l'ID et les données
                                    if len(message_data) > 0:
                                        message_id = message_data[0]
                                        payload = message_data[1:]
                                        
                                        # Appeler le callback si enregistré
                                        if message_id in self.message_id_callback:
                                            self.message_id_callback[message_id](payload)
                                    
                                    # Retirer tout jusqu'après la signature
                                    buffer = buffer[end_index + len(END_BYTES_SIGNATURE):]
                                else:
                                    # Pas assez de données, attendre
                                    break
                            else:
                                # Données incomplètes
                                buffer = buffer[end_index + len(END_BYTES_SIGNATURE):]
                        else:
                            # Message trop court
                            buffer = buffer[end_index + len(END_BYTES_SIGNATURE):]
                
                time.sleep(0.001)  # Petite pause pour ne pas surcharger le CPU
                
            except Exception as e:
                if self.logger:
                    self.logger.error(f"❌ Erreur réception: {e}")
                time.sleep(0.1)

    def send_bytes(self, message: bytes):
        """Envoie un message avec CRC et signature (format compatible avec Com C++).
        
        Args:
            message: Message à envoyer (ID + données)
        """
        size = len(message)
        
        # Créer le paquet pour le CRC : message + taille
        crc_data = message + bytes([size])
        
        # Calculer le CRC sur (message + taille)
        self._crc8.reset()
        self._crc8.update(crc_data)
        crc_value = self._crc8.digest()[0]
        
        # Construire le paquet final : message + taille + CRC + signature
        packet = message + bytes([size, crc_value]) + END_BYTES_SIGNATURE
        
        # Envoyer
        self._device.write(packet)
        self.last_message = message

    def add_callback(self, callback, message_id: int):
        """Enregistre un callback pour un type de message.
        
        Args:
            callback: Fonction à appeler lors de la réception
            message_id: ID du message à écouter
        """
        self.message_id_callback[message_id] = callback

    def close(self):
        """Ferme la connexion série."""
        if self._device:
            self._device.close()
        if self.logger:
            self.logger.info(f"🔌 Connexion {self.port} fermée")


# Alias pour compatibilité avec le code existant
Com = WebotsComBridge
