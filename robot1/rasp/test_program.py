import struct
import math
from loader import loader
import logging

# Créer un logger simple
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Charger les classes via le loader
Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')

SET_TARGET_POSITION = 0x01

def send_target_coordonates(target_x_pos: float, target_y_pos: float, target_theta: float, com: Com) -> None:
    """Envoie les coordonnées cibles (x, y, theta) à la Teensy via USB."""
    msg = Messages.SET_TARGET_POSITION.to_bytes() 
    msg += struct.pack('<ddd', target_x_pos, target_y_pos, target_theta)
    com.send_bytes(msg)
    print(f"Coordonnées envoyées : X={target_x_pos}mm, Y={target_y_pos}mm, θ={target_theta:.2f}rad")

def main():
    # Récupérer la configuration série
    serial_config = loader.get_config('serial_config')
    
    # Initialiser la communication avec TOUS les paramètres requis
    com = Com(
        logger=logger,
        serial_number=serial_config['serial_number'],
        vid=serial_config['vid'],
        pid=serial_config['pid'],
        baudrate=serial_config['baudrate'],
        enable_crc=serial_config['enable_crc'],
        enable_dummy=serial_config['enable_dummy']
    )
    
    # Ligne droite horizontale
    send_target_coordonates(200, 0, 0, com) 

    # Ligne droite verticale 
    send_target_coordonates(0, 200, 0, com)

    # Carré
    send_target_coordonates(200, 0, 0, com)
    send_target_coordonates(200, -200, 0, com)
    send_target_coordonates(0, -200, 0, com)
    send_target_coordonates(0, 0, 0, com)

    # Triangle rectangle
    send_target_coordonates(200, 0, 0, com)
    send_target_coordonates(200, 200, 0, com)
    send_target_coordonates(0, 0, 0, com)

    # Cercle
    nb_pas = 1000
    rayon = 100

    for i in range(nb_pas + 1):
        angle = i * 2 * math.pi / nb_pas
        x = rayon * math.cos(angle)
        y = rayon * math.sin(angle)
        theta = angle + math.pi / 2
        send_target_coordonates(x, y, theta, com)

if __name__ == "__main__":
    main()
