
LARGEUR_TABLE = 3000
couleur_equipe = "JAUNE" # ou "BLEU"

def adapter_coordonnees(x_bleu, y_bleu, angle_bleu):
    """
    Transforme les coordonnées 'Bleu' en coordonnées réelles selon la couleur de l'équipe.
    """
    if couleur_equipe == "BLEU":
        # On ne change rien, le code a été pensé pour ce côté
        return x_bleu, y_bleu, angle_bleu
        
    elif couleur_equipe == "JAUNE":
        # On applique la symétrie !
        x_jaune = LARGEUR_TABLE - x_bleu
        y_jaune = y_bleu # Y reste identique

        # La formule pour inverser l'angle sur un axe vertical est : (180 - angle) modulo 360
        angle_jaune = (180 - angle_bleu) % 360
        
        return x_jaune, y_jaune, angle_jaune

