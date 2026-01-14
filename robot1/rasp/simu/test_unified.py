#!/usr/bin/env python3
"""
Test de communication unifié : fonctionne automatiquement en simulation et sur hardware.

Ce script détecte automatiquement le contexte (simulation Webots ou robot réel)
et utilise la configuration appropriée.
"""

import struct
import time
import logging
from pathlib import Path
import sys

# Ajouter le chemin du dossier parent pour importer robot_context
sys.path.insert(0, str(Path(__file__).parent))

from robot1.rasp.robot_context import is_simulation, create_com
from robot1.rasp.simu.loader import loader

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
    
    # Détection automatique du mode
    mode = "SIMULATION" if is_simulation() else "HARDWARE"
    
    logger.info("=" * 70)
    logger.info(f"       Test de communication - Mode {mode}")
    logger.info("=" * 70)
    
    if is_simulation():
        logger.info("⚠️  Assurez-vous que:")
        logger.info("   • Webots est lancé avec teensy_controller")
        logger.info("   • Virtual Serial Port Tools: COM1 ↔ COM2 (local bridge) actif")
        logger.info("   • Console Webots affiche '[Webots] ✅ COM2 connecté !'")
    else:
        logger.info("⚠️  Assurez-vous que:")
        logger.info("   • La Teensy est connectée en USB")
        logger.info("   • Le firmware teensy_moteur est flashé")
    
    logger.info("=" * 70)
    logger.info("")
    
    logger.info(f"🔌 Connexion en mode {mode}...")
    
    # Création automatique de l'instance Com appropriée
    com = create_com(logger=logger)
    
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
        logger.info(f"\n📍 Test : {description}")
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
        logger.info(f"\n✅ SUCCÈS : La communication en mode {mode} fonctionne !")
        logger.info("   • Les messages sont envoyés correctement")
        logger.info("   • Les réponses sont reçues correctement")
        logger.info("   • Le protocole de communication fonctionne")
        
        if is_simulation():
            logger.info("\n🎯 La simulation Webots est prête pour les tests de navigation !")
        else:
            logger.info("\n🎯 Le robot réel est prêt pour les tests de navigation !")
    else:
        logger.warning(f"\n⚠️  ATTENTION : Aucun message reçu en mode {mode}")
        
        if is_simulation():
            logger.warning("   Vérifications simulation :")
            logger.warning("   • Webots est-il bien lancé ?")
            logger.warning("   • teensy_controller.exe tourne-t-il ?")
            logger.warning("   • Console Webots affiche '[Webots] ✅ COM2 connecté !' ?")
            logger.warning("   • Virtual Serial Port Tools: COM1 ↔ COM2 actif ?")
        else:
            logger.warning("   Vérifications hardware :")
            logger.warning("   • Le firmware est-il bien flashé sur la Teensy ?")
            logger.warning("   • La Teensy est-elle bien reconnue en USB ?")
            logger.warning("   • La Teensy envoie-t-elle des messages UPDATE_ROLLING_BASIS ?")
        
        logger.warning("   • Le baudrate est-il correct (115200) ?")
    
    logger.info("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("\n\n⏹️  Test interrompu par l'utilisateur")
    except Exception as e:
        logger.error(f"\n❌ ERREUR : {e}", exc_info=True)
