#!/usr/bin/env python3
"""Test simple pour faire tourner UN seul moteur pas-à-pas."""

import struct
import time
from loader import loader
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')

def handle_position(data: bytes) -> None:
    """Affiche la position reçue de la Teensy."""
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"📍 Position: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")

def main():
    serial_config = loader.get_config('serial_config')
    
    logger.info("=" * 70)
    logger.info("🔧 TEST D'UN SEUL MOTEUR PAS-À-PAS")
    logger.info("=" * 70)
    
    # Connexion
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
    
    com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)
    logger.info("✅ Connexion établie!\n")
    
    # Réinitialiser l'odométrie à (0, 0, 0)
    logger.info("📍 Réinitialisation de l'odométrie à (0, 0, 0)")
    msg = Messages.SET_ODOMETRIE.to_bytes()
    msg += struct.pack('<ddd', 0.0, 0.0, 0.0)
    com.send_bytes(msg)
    time.sleep(1)
    
    logger.info("\n" + "=" * 70)
    logger.info("🎯 TEST 1 : Mouvement en X (avancer)")
    logger.info("=" * 70)
    logger.info("Commande : Avancer de 50mm en ligne droite")
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', 50.0, 0.0, 0.0)  # 50mm en X, 0 en Y, 0 rotation
    com.send_bytes(msg)
    logger.info("⏳ Attente de 5 secondes...\n")
    time.sleep(5)
    
    logger.info("\n" + "=" * 70)
    logger.info("🎯 TEST 2 : Mouvement en Y (gauche)")
    logger.info("=" * 70)
    logger.info("Commande : Se déplacer de 50mm vers la gauche")
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', 50.0, 50.0, 0.0)  # Position (50, 50, 0)
    com.send_bytes(msg)
    logger.info("⏳ Attente de 5 secondes...\n")
    time.sleep(5)
    
    logger.info("\n" + "=" * 70)
    logger.info("🎯 TEST 3 : Rotation sur place")
    logger.info("=" * 70)
    logger.info("Commande : Tourner de 90° (π/2 rad)")
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', 50.0, 50.0, 1.5708)  # π/2 ≈ 1.5708 rad
    com.send_bytes(msg)
    logger.info("⏳ Attente de 5 secondes...\n")
    time.sleep(5)
    
    logger.info("\n" + "=" * 70)
    logger.info("🎯 TEST 4 : Retour à l'origine")
    logger.info("=" * 70)
    logger.info("Commande : Retour à (0, 0, 0)")
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', 0.0, 0.0, 0.0)
    com.send_bytes(msg)
    logger.info("⏳ Attente de 5 secondes...\n")
    time.sleep(5)
    
    logger.info("\n" + "=" * 70)
    logger.info("✅ TEST TERMINÉ")
    logger.info("=" * 70)
    logger.info("\n📝 OBSERVATIONS À FAIRE :")
    logger.info("  1. Les moteurs ont-ils fait du bruit ?")
    logger.info("  2. Les roues ont-elles tourné ?")
    logger.info("  3. Les positions affichées changent-elles ?")
    logger.info("  4. Y a-t-il des erreurs dans la console ?")
    logger.info("\n💡 SANS MOTEURS CONNECTÉS :")
    logger.info("  - Aucun mouvement physique (normal)")
    logger.info("  - Position reste à (0, 0, 0) (normal)")
    logger.info("  - Pas d'erreur = communication OK ✅")
    logger.info("=" * 70)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n\n⏹️  Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"\n❌ ERREUR: {e}", exc_info=True)
