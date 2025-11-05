import struct
from common.usb_com.python.com.com import Com
from usb_com.python.messages import Messages
import math

SET_TARGET_POSITION = 0x01  # ID du message, doit correspondre à Teensy

def send_target_coordonates(target_x_pos: float, target_y_pos: float, target_theta: float, com: Com) -> None:
    """
    Envoie les coordonnées cibles (x, y, theta) à la Teensy via USB.

    Args:
        target_x_pos (float): coordonnée X
        target_y_pos (float): coordonnée Y
        target_theta (float): orientation theta en radians
        com (Com): instance de la classe Com déjà initialisée
    """
    
    """Envoyer une position cible au Teensy"""
    msg = Messages.SET_TARGET_POSITION.to_bytes() 
    msg += struct.pack('<ddd', target_x_pos, target_y_pos, target_theta)
    com.send_bytes(msg)

    # Affichage clair avec unités et arrondi
    print(f"Coordonnées envoyées : X={target_x_pos}mm, Y={target_y_pos}mm, θ={target_theta:.2f}rad")

#ligne droite horizontal
send_target_coordonates(200,0,0) 

#ligne droite verticale 
send_target_coordonates(0,200,0)

#carré
send_target_coordonates(200,0,0)
send_target_coordonates(200,-200,0)
send_target_coordonates(0,-200,0)
send_target_coordonates(0,0,0)

#triangle rectangle
send_target_coordonates(200,0,0)
send_target_coordonates(200,200,0)
send_target_coordonates(0,0,0)

# Cercle - coordonnées absolues
nb_pas = 1000
rayon = 100

for i in range(nb_pas + 1):
    angle = i * 2 * math.pi / nb_pas  # Angle sur le cercle
    
    # Coordonnées absolues du point sur le cercle
    x = rayon * math.cos(angle)
    y = rayon * math.sin(angle)
    
    # Theta = tangente au cercle (direction du mouvement)
    theta = angle + math.pi / 2
    
    send_target_coordonates(x, y, theta)
