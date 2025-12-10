#!/usr/bin/env python3
"""Test bas niveau pour voir EXACTEMENT ce que la Teensy envoie."""

import serial
import time

PORT = "/dev/ttyACM0"  # Changez si nécessaire
BAUDRATE = 115200
END_SIGNATURE = bytes([0xBA, 0xDD, 0x1C, 0xC5])

print("=" * 70)
print("TEST BAS NIVEAU - MESSAGES BRUTS")
print("=" * 70)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"✅ Connecté à {PORT} @ {BAUDRATE} baud\n")
    
    print("📡 Écoute des messages (Ctrl+C pour arrêter)...\n")
    
    message_count = 0
    buffer = bytearray()
    
    while True:
        if ser.in_waiting > 0:
            # Lire les données disponibles
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
            
            # Chercher la signature de fin
            while END_SIGNATURE in buffer:
                # Trouver la position de la signature
                sig_pos = buffer.find(END_SIGNATURE)
                
                # Extraire le message complet
                full_msg = buffer[:sig_pos + 4]
                
                # Supprimer ce message du buffer
                buffer = buffer[sig_pos + 4:]
                
                message_count += 1
                
                print(f"\n{'='*70}")
                print(f"📨 MESSAGE #{message_count}")
                print(f"{'='*70}")
                print(f"Longueur totale: {len(full_msg)} bytes")
                print(f"Hex complet: {full_msg.hex(' ')}")
                
                if len(full_msg) >= 6:
                    # Format attendu: [data...] [length] [crc] [signature(4 bytes)]
                    signature = full_msg[-4:]
                    crc_byte = full_msg[-5:-4]
                    length_byte = full_msg[-6:-5]
                    message_data = full_msg[:-6]
                    
                    print(f"\n📊 Décomposition:")
                    print(f"  Signature    : {signature.hex(' ')} {'✅' if signature == END_SIGNATURE else '❌'}")
                    print(f"  CRC          : {crc_byte.hex()}")
                    print(f"  Length décl. : {length_byte[0] if length_byte else 'N/A'}")
                    print(f"  Data length  : {len(message_data)}")
                    print(f"  Data (hex)   : {message_data.hex(' ')}")
                    
                    if message_data:
                        msg_id = message_data[0]
                        print(f"\n  Message ID   : {msg_id} (0x{msg_id:02x})")
                        
                        if msg_id == 128:  # UPDATE_ROLLING_BASIS
                            print(f"  Type         : UPDATE_ROLLING_BASIS")
                            payload = message_data[1:]
                            print(f"  Payload      : {len(payload)} bytes")
                            
                            if len(payload) >= 24:
                                import struct
                                x, y, theta = struct.unpack('<ddd', payload[:24])
                                print(f"  ✅ X={x:.6f}, Y={y:.6f}, θ={theta:.6f}")
                            else:
                                print(f"  ⚠️  Payload trop court pour 3 doubles")
                    
                    # Vérifier la cohérence
                    if length_byte:
                        declared_len = length_byte[0]
                        actual_len = len(full_msg) - 6  # Sans length, crc, signature
                        if declared_len != actual_len:
                            print(f"\n  ⚠️  INCOHÉRENCE: Length déclarée={declared_len}, réelle={actual_len}")
                        else:
                            print(f"\n  ✅ Length cohérente: {declared_len} bytes")
                
                print(f"{'='*70}\n")
        
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\n⏹️  Test arrêté")
except Exception as e:
    print(f"\n❌ ERREUR: {e}")
    import traceback
    traceback.print_exc()
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("✅ Port fermé")
