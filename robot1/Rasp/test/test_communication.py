#!/usr/bin/env python3
"""Test de communication Rasp ↔ Teensy SANS moteurs."""

import struct
import time
from loader import loader
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')

# Compteurs pour les statistiques
messages_sent = 0
messages_received = 0

def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position."""
    global messages_received
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        messages_received += 1
        logger.info(f"📥 Position reçue #{messages_received}: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        logger.warning(f"⚠️  Message trop court: {len(data)} bytes")

def main():
    serial_config = loader.get_config('serial_config')
    
    logger.info("🔌 Connexion à la Teensy...")
    com = Com(
        logger=logger,
        serial_number=serial_config['serial_number'],
        vid=serial_config['vid'],
        pid=serial_config['pid'],
        baudrate=serial_config['baudrate'],
        enable_crc=serial_config['enable_crc'],
        enable_dummy=serial_config['enable_dummy']
    )
    
    # Enregistrer le callback
    com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)
    
    logger.info("✅ Connexion établie!")
    logger.info("=" * 70)
    
    # Test 1 : Réinitialiser l'odométrie
    logger.info("📍 Test 1 : Réinitialisation de l'odométrie à (0, 0, 0)")
    msg = Messages.SET_ODOMETRIE.to_bytes()
    msg += struct.pack('<ddd', 0.0, 0.0, 0.0)
    com.send_bytes(msg)
    messages_sent += 1
    logger.info("📤 Message envoyé!")
    time.sleep(2)
    
    # Test 2 : Envoyer quelques positions cibles
    positions = [
        (100.0, 0.0, 0.0, "Avancer 100mm"),
        (100.0, 100.0, 0.0, "Diagonale"),
        (0.0, 100.0, 0.0, "Gauche"),
        (0.0, 0.0, 0.0, "Retour origine"),
    ]
    
    for x, y, theta, description in positions:
        logger.info(f"📍 Test : {description}")
        msg = Messages.SET_TARGET_POSITION.to_bytes()
        msg += struct.pack('<ddd', x, y, theta)
        com.send_bytes(msg)
        messages_sent += 1
        logger.info(f"📤 Commande envoyée: X={x}mm, Y={y}mm, θ={theta}rad")
        time.sleep(3)
    
    # Attendre pour recevoir les derniers messages
    logger.info("\n⏳ Attente de 5 secondes pour les messages restants...")
    time.sleep(5)
    
    # Statistiques finales
    logger.info("\n" + "=" * 70)
    logger.info("📊 STATISTIQUES DE COMMUNICATION")
    logger.info("=" * 70)
    logger.info(f"📤 Messages envoyés   : {messages_sent}")
    logger.info(f"📥 Messages reçus     : {messages_received}")
    
    if messages_received > 0:
        logger.info("\n✅ SUCCÈS : La communication Rasp ↔ Teensy fonctionne !")
        logger.info("   • Les messages sont envoyés correctement")
        logger.info("   • Les réponses sont reçues correctement")
        logger.info("   • Le protocole USB fonctionne")
        logger.info("\n🎯 Prochaine étape : Connecter les moteurs pas-à-pas")
    else:
        logger.warning("\n⚠️  ATTENTION : Aucun message reçu de la Teensy")
        logger.warning("   Vérifications à faire :")
        logger.warning("   • Le firmware est-il bien flashé sur la Teensy ?")
        logger.warning("   • Le baudrate est-il correct (115200) ?")
        logger.warning("   • La Teensy envoie-t-elle des messages UPDATE_ROLLING_BASIS ?")
    
    logger.info("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n\n⏹️  Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"\n❌ ERREUR : {e}", exc_info=True)
