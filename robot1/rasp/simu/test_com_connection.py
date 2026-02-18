import serial
import time

print("Test de connexion COM1...")
print("⚠️  Assurez-vous que Webots est lancé et que teensy_controller tourne !")
print()

try:
    ser = serial.Serial('COM1', 115200, timeout=1)
    print(f"✅ COM1 ouvert : {ser.name}")
    
    print("\n📤 Test d'envoi vers Webots (COM1 → COM2)...")
    ser.write(b'HELLO_WEBOTS')
    print("✅ Données envoyées")
    
    print("\n📥 Écoute pendant 3 secondes...")
    for i in range(6):
        time.sleep(0.5)
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"✅ Réception : {data.hex()} ({len(data)} octets)")
        else:
            print(f"   [{i+1}/6] En attente...")
    
    if ser.in_waiting == 0:
        print("\n⚠️  Aucune donnée reçue de Webots")
        print("Vérifiez :")
        print("  1. Webots est lancé")
        print("  2. teensy_controller est recompilé (bouton Revert ⟲)")
        print("  3. Console Webots affiche '[Webots] ✅ COM2 connecté !'")
    
    ser.close()
    print("\n✅ Test terminé")
    
except serial.SerialException as e:
    print(f"❌ ERREUR : {e}")
    print("Vérifiez que com0com est bien installé et que la paire COM1↔COM2 existe")
