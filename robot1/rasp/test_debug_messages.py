#!/usr/bin/env python3
"""Test avec logs détaillés pour debugger les messages."""

import struct
import time
from loader import loader
import logging

# Configuration du logging très détaillé
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')

def handle_position(data: bytes) -> None:
    """Callback détaillé pour analyser les messages."""
    logger.info(f"📥 Message reçu: {len(data)} bytes")
    logger.info(f"   Hex: {data.hex(' ')}")
    
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"   ✅ Décodé: X={x:.6f}mm, Y={y:.6f}mm, θ={theta:.6f}rad")
        
        if x == 0.0 and y == 0.0 and theta == 0.0:
            logger.warning("   ⚠️  Toutes les valeurs sont à zéro !")
    else:
        logger.error(f"   ❌ Message trop court: attendu 24 bytes, reçu {len(data)}")

def main():
    serial_config = loader.get_config('serial_config')
    
    logger.info("=" * 70)
    logger.info("Configuration:")
    logger.info(f"  Serial: {serial_config['serial_number']}")
    logger.info(f"  VID: {serial_config['vid']}")
    logger.info(f"  PID: {serial_config['pid']}")
    logger.info(f"  Baudrate: {serial_config['baudrate']}")
    logger.info(f"  CRC: {serial_config['enable_crc']}")
    logger.info(f"  Dummy: {serial_config['enable_dummy']}")
    logger.info("=" * 70)
    
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
    logger.info("✅ Callback enregistré pour UPDATE_ROLLING_BASIS (ID=128)")
    
    logger.info("✅ Connexion établie!")
    logger.info("\n⏳ Écoute des messages pendant 15 secondes...\n")
    
    time.sleep(15)
    
    logger.info("\n✅ Test terminé!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n⏹️  Test interrompu")
    except Exception as e:
        logger.error(f"\n❌ ERREUR: {e}", exc_info=True)
