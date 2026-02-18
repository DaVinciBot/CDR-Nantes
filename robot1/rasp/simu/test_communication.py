#!/usr/bin/env python3
"""Test de communication Python ↔ Webots SANS moteurs."""

import struct
import time
import logging
from robot1.rasp.simu.loader import loader
from robot1.rasp.webots_com import WebotsComBridge

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

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
    global messages_sent, messages_received
    
    serial_config = loader.get_config('serial_config')
    
    logger.info("=" * 70)
    logger.info("       Test de communication Python ↔ Webots via COM1/COM2")
    logger.info("=" * 70)
    logger.info("⚠️  Assurez-vous que:")
    logger.info("   • Webots est lancé avec teensy_controller")
    logger.info("   • Virtual Serial Port Tools: COM1 ↔ COM2 (local bridge) actif")
    logger.info("   • Console Webots affiche '[Webots] ✅ COM2 connecté !'")
    logger.info("=" * 70)
    logger.info("")
    
    logger.info("🔌 Connexion à COM1 (Webots sur COM2)...")
    
    # Utiliser WebotsComBridge pour les ports COM virtuels
    com = WebotsComBridge(
        port=serial_config['port'],
        baudrate=serial_config['baudrate'],
        enable_crc=serial_config['enable_crc'],
        logger=logger
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
        logger.info("\n✅ SUCCÈS : La communication Python ↔ Webots fonctionne !")
        logger.info("   • Les messages sont envoyés correctement via COM1")
        logger.info("   • Les réponses sont reçues depuis COM2")
        logger.info("   • Le protocole USB virtuel fonctionne")
        logger.info("\n🎯 Prochaine étape : Tester les mouvements du robot dans Webots")
    else:
        logger.warning("\n⚠️  ATTENTION : Aucun message reçu de Webots")
        logger.warning("   Vérifications à faire :")
        logger.warning("   • Webots est-il bien lancé ?")
        logger.warning("   • teensy_controller.exe tourne-t-il ?")
        logger.warning("   • Console Webots affiche '[Webots] ✅ COM2 connecté !' ?")
        logger.warning("   • Virtual Serial Port Tools: COM1 ↔ COM2 actif ?")
        logger.warning("   • Le baudrate est-il correct (115200) ?")
    
    logger.info("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n\n⏹️  Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"\n❌ ERREUR : {e}", exc_info=True)
