#!/usr/bin/env python3
"""Test bas niveau pour voir EXACTEMENT ce que la Teensy envoie."""

import os
import sys

import serial
import time
from serial.tools.list_ports import comports

from usb_com import FRAME_DELIMITER, Messages, payload_size, unpack_message
from usb_com.python.com.framing import cobs_decode, compute_crc8

from robot1.rasp.comm import get_com_config


def find_teensy_port(config):
    """Locate the Teensy port, using the same rule as usb_com.Com.

    Matching on vid + pid + serial_number, never on a hardcoded device path:
    /dev/ttyACM0 was written here and moves with the USB enumeration order.
    TEENSY_PORT forces a device when the automatic match is not wanted.
    """
    forced = os.environ.get("TEENSY_PORT")
    if forced:
        return forced

    for port in comports():
        if (port.vid == config["vid"]
                and port.pid == config["pid"]
                and port.serial_number is not None
                and port.serial_number == str(config["serial_number"])):
            return port.device
    return None


config = get_com_config()
BAUDRATE = config.get("baudrate", 115200)

print("=" * 70)
print("TEST BAS NIVEAU - MESSAGES BRUTS")
print("=" * 70)

PORT = find_teensy_port(config)
if PORT is None:
    print("\n[FAIL] Aucune Teensy trouvee "
          f"(vid={config['vid']}, pid={config['pid']}, "
          f"serial={config['serial_number']}).")
    print("  Lancer 'python -m tools.bringup.test_usb_detection' pour lister les ports,")
    print("  ou forcer un device : TEENSY_PORT=/dev/ttyACM0 python -m tools.bringup.test_length_messages")
    sys.exit(1)

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"[OK] Connecté à {PORT} @ {BAUDRATE} baud\n")

    print(" Écoute des messages (Ctrl+C pour arrêter)...\n")

    message_count = 0
    buffer = bytearray()

    while True:
        if ser.in_waiting > 0:
            # Lire les données disponibles
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

            # Each frame ends with the delimiter: COBS([ID][data][CRC8]) 0x00
            while FRAME_DELIMITER in buffer:
                end = buffer.find(FRAME_DELIMITER)
                encoded = bytes(buffer[:end])
                buffer = buffer[end + 1:]
                if not encoded:
                    continue  # two delimiters in a row

                message_count += 1

                print(f"\n{'='*70}")
                print(f" MESSAGE #{message_count}")
                print(f"{'='*70}")
                print(f"Longueur encodée : {len(encoded)} bytes (+ délimiteur)")
                print(f"Hex encodé       : {encoded.hex(' ')}")

                body = cobs_decode(encoded)
                if body is None:
                    print("  [FAIL] COBS invalide")
                    print(f"{'='*70}\n")
                    continue
                if len(body) < 2:
                    print(f"  [FAIL] Trame trop courte : {len(body)} byte(s) décodé(s)")
                    print(f"{'='*70}\n")
                    continue

                message_data, crc_byte = body[:-1], body[-1]
                crc_ok = compute_crc8(message_data) == crc_byte

                print(f"\n Décomposition:")
                print(f"  CRC          : {crc_byte:02x} {'[OK]' if crc_ok else '[FAIL]'}")
                print(f"  Data length  : {len(message_data)}")
                print(f"  Data (hex)   : {message_data.hex(' ')}")

                msg_id = message_data[0]
                print(f"\n  Message ID   : {msg_id} (0x{msg_id:02x})")

                if msg_id == Messages.ODOMETRY.value:
                    print(f"  Type         : ODOMETRY")
                    payload = message_data[1:]
                    expected = payload_size(Messages.ODOMETRY)
                    print(f"  Payload      : {len(payload)} bytes (attendu {expected})")

                    if len(payload) == expected:
                        x, y, theta, vx, vy, omega, t_us = unpack_message(Messages.ODOMETRY, payload)
                        print(f"  [OK] X={x:.3f}, Y={y:.3f}, theta={theta:.5f}")
                        print(f"       vx={vx:.1f}, vy={vy:.1f}, omega={omega:.4f}, t={t_us} us")
                    else:
                        print(f"  [WARN]  Taille de payload inattendue")

                print(f"{'='*70}\n")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\n\n  Test arrêté")
except Exception as e:
    print(f"\n[FAIL] ERREUR: {e}")
    import traceback
    traceback.print_exc()
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("[OK] Port fermé")
