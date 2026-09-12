#!/usr/bin/env python3
"""Test de détection USB de la Teensy."""

import serial.tools.list_ports
import sys

print("=" * 70)
print("TEST DE DÉTECTION USB - TEENSY")
print("=" * 70)

# Lister tous les ports
ports = serial.tools.list_ports.comports()
print(f"\nNombre total de ports détectés : {len(ports)}\n")

if len(ports) == 0:
    print("[FAIL] AUCUN PORT SÉRIE DÉTECTÉ !")
    print("\nVérifications à faire :")
    print("  1. La Teensy est-elle branchée via USB ?")
    print("  2. Le câble USB transmet-il les données (pas seulement l'alimentation) ?")
    print("  3. La LED de la Teensy clignote-t-elle ?")
    sys.exit(1)

teensy_found = False

for i, port in enumerate(ports, 1):
    print(f" Port #{i}")
    print(f"  Device      : {port.device}")
    print(f"  Description : {port.description}")
    print(f"  VID         : {port.vid} (0x{port.vid:04x})" if port.vid else "  VID         : N/A")
    print(f"  PID         : {port.pid} (0x{port.pid:04x})" if port.pid else "  PID         : N/A")
    print(f"  Serial #    : {port.serial_number}")
    print(f"  Manufacturer: {port.manufacturer}")
    print(f"  Product     : {port.product}")

    # Détection Teensy (VID standard = 0x16c0 = 5824)
    if port.vid == 0x16c0 or (port.manufacturer and "teensy" in port.manufacturer.lower()):
        print("\n   [OK] *** TEENSY DÉTECTÉE ! ***")
        teensy_found = True

        print("\n    Configuration à utiliser dans config.json :")
        print(f'   "serial_number": {port.serial_number},')
        print(f'   "vid": {port.vid},')
        print(f'   "pid": {port.pid},')

    print("-" * 70)

print("\n" + "=" * 70)
if teensy_found:
    print("[OK] RÉSULTAT : Teensy trouvée et accessible")
else:
    print("[FAIL] RÉSULTAT : Aucune Teensy détectée")
    print("\n[WARN]  Points à vérifier :")
    print("   • La Teensy est peut-être en mode bootloader")
    print("   • Le firmware n'est peut-être pas flashé")
    print("   • Le câble USB ne transmet peut-être que l'alimentation")
    print("   • Essayez un autre port USB")

print("=" * 70)
