#!/usr/bin/env python3
"""Test de communication Python ↔ Webots via COM1/COM2."""

import struct
import time
import logging
from robot1.rasp.simu.loader import loader

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')
logger = logging.getLogger(__name__)

# Compteurs pour les statistiques
messages_sent = 0
messages_received = 0

com = loader.load_class('usb_com', 'Com')
messages = loader.load_class('usb_com', 'Messages')

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
    logger.info("=" * 70)
    logger.info("Test de communication Python ↔ Webots")
    logger.info("=" * 70)
    logger.info("⚠️  Assurez-vous que Webots est lancé avec teensy_controller !")
    logger.info("⚠️  Virtual Serial Port Tools: COM1 ↔ COM2 (local bridge)\n")
    
    logger.info("🔌 Connexion à COM1 (Webots sur COM2)...")
    
    # Récupérer la configuration depuis config.json
    serial_config = loader.get_config('serial_config')
    
    # Configuration pour port COM virtuel (simulation Webots)
    com = Com(
        logger=logger,
        serial_number=serial_config['serial_number'],
        vid=serial_config['vid'],
        pid=serial_config['pid'],
        baudrate=serial_config['baudrate'],
        enable_crc=serial_config['enable_crc'],
        enable_dummy=serial_config['enable_dummy'],
        port=serial_config['port']  # Force COM1
    )
    
    # Enregistrer le callback
    com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)
    
    logger.info("✅ Connexion COM1 établie!")
    logger.info("=" * 70)
    
    # Test 1 : Réinitialiser l'odométrie
    logger.info("📍 Test 1 : Réinitialisation de l'odométrie à (0, 0, 0)")
    msg = Messages.SET_ODOMETRIE.to_bytes()
    msg += struct.pack('<ddd', 0.0, 0.0, 0.0)
    com.send_bytes(msg)
    messages_sent += 1
    logger.info("📤 Message envoyé!")
    
    # Écouter pendant 2 secondes
    start = time.time()
    while time.time() - start < 2:
        com.handle()
        time.sleep(0.01)
    
    # Test 2 : Envoyer quelques positions cibles
    positions = [
        (100.0, 0.0, 0.0, "Avancer 100mm"),
        (100.0, 100.0, 0.0, "Diagonale"),
        (0.0, 100.0, 0.0, "Gauche"),
        (0.0, 0.0, 0.0, "Retour origine"),
    ]
    
    for x, y, theta, description in positions:
        logger.info(f"\n📍 Test : {description}")
        msg = Messages.SET_TARGET_POSITION.to_bytes()
        msg += struct.pack('<ddd', x, y, theta)
        com.send_bytes(msg)
        messages_sent += 1
        logger.info(f"📤 Commande envoyée: X={x}mm, Y={y}mm, θ={theta}rad")
        
        # Écouter pendant 3 secondes
        start = time.time()
        while time.time() - start < 3:
            com.handle()
            time.sleep(0.01)
    
    # Attendre pour recevoir les derniers messages
    logger.info("\n⏳ Attente de 5 secondes pour les messages restants...")
    start = time.time()
    while time.time() - start < 5:
        com.handle()
        time.sleep(0.01)
    
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
        logger.info("\n🎯 La simulation Webots est prête pour les tests de navigation !")
    else:
        logger.warning("\n⚠️  ATTENTION : Aucun message reçu de Webots")
        logger.warning("   Vérifications à faire :")
        logger.warning("   • Webots est-il lancé ?")
        logger.warning("   • teensy_controller.exe tourne-t-il ?")
        logger.warning("   • Console Webots affiche '[Webots] ✅ COM2 connecté !' ?")
        logger.warning("   • Virtual Serial Port Tools : COM1 ↔ COM2 actif ?")
    
    logger.info("=" * 70)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n\n⚠️  Arrêt par l'utilisateur")
    except Exception as e:
        logger.error(f"\n❌ Erreur: {e}", exc_info=True)
