import sys
import os
import struct
import time
from robot1.rasp.simu.loader import loader

# Ajouter le chemin vers common pour importer Com
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'common', 'usb_com', 'python'))


def handle_rolling_basis_update(data: bytes) -> None:
    """Callback pour recevoir la position du robot depuis Webots."""
    if len(data) >= 24:  # 3 doubles = 24 bytes
        x, y, theta = struct.unpack('<ddd', data[:24])
        print(f"📍 Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        print(f"⚠️  Message UPDATE_ROLLING_BASIS trop court: {len(data)} bytes")

def send_target_position(com: Com, x: float, y: float, theta: float) -> None:
    """Envoie une position cible à Webots."""
    msg = Messages.SET_TARGET_POSITION.to_bytes() 
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    print(f"📤 Envoi cible: X={x}mm, Y={y}mm, θ={theta:.2f}rad")

def main():
    print("=" * 60)
    print("Test de contrôle Webots via COM1")
    print("=" * 60)
    print("⚠️  Assurez-vous que Webots est lancé avec teensy_controller !")
    print()
    
    # Configuration pour port COM virtuel (pas de dummy, pas de détection USB)
    com = Com(
        logger=None,
        serial_number=None,
        vid=None,
        pid=None,
        baudrate=115200,
        enable_crc=True,
        enable_dummy=False,
        port='COM1'  # Forcer COM1
    )
    
    # Enregistrer le callback
    com.add_callback(handle_rolling_basis_update, Messages.UPDATE_ROLLING_BASIS.value)
    
    print("✅ Connexion COM1 établie avec Webots!\n")
    
    try:
        # Test 1: Translation X
        print("\n[Test 1] Translation X: (100, 0, 0)")
        send_target_position(com, 100.0, 0.0, 0.0)
        
        # Écouter pendant 5 secondes
        print("📥 Écoute des messages pendant 5 secondes...")
        start = time.time()
        while time.time() - start < 5:
            com.handle()
            time.sleep(0.01)
        
        # Test 2: Translation Y
        print("\n[Test 2] Translation Y: (0, 100, 0)")
        send_target_position(com, 0.0, 100.0, 0.0)
        
        # Écouter pendant 5 secondes
        print("📥 Écoute des messages pendant 5 secondes...")
        start = time.time()
        while time.time() - start < 5:
            com.handle()
            time.sleep(0.01)
        
        # Test 3: Rotation
        print("\n[Test 3] Rotation: (0, 0, π/2)")
        send_target_position(com, 0.0, 0.0, 1.57)
        
        # Écouter pendant 5 secondes
        print("📥 Écoute des messages pendant 5 secondes...")
        start = time.time()
        while time.time() - start < 5:
            com.handle()
            time.sleep(0.01)
        
        print("\n✅ Tests terminés!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Arrêt par l'utilisateur")
    
    finally:
        print("🔌 Fermeture de la connexion")

if __name__ == '__main__':
    main()
