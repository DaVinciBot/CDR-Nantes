
LARGEUR_TABLE = 3000   # X (mm)
HAUTEUR_TABLE = 2000   # Y (mm)
couleur_equipe = "JAUNE" # ou "BLEU"

def adapter_coordonnees(x_bleu, y_bleu, angle_bleu):
    """
    Transforme les coordonnées 'Bleu' en coordonnées réelles selon la couleur de l'équipe.
    
    Zones de départ CDR 2026:
    - BLEU: (1775, 2700)  — haut-gauche
    - JAUNE: (1775, 300)  — bas-gauche
    
    Symétrie Y uniquement (miroir haut ↔ bas), PAS de symétrie X.
    """
    if couleur_equipe == "BLEU":
        # On ne change rien, le code a été pensé pour ce côté
        return x_bleu, y_bleu, angle_bleu
        
    elif couleur_equipe == "JAUNE":
        # Symétrie Y uniquement (haut ↔ bas)
        x_jaune = x_bleu  # X ne change pas
        y_jaune = HAUTEUR_TABLE - y_bleu  # Y est inversé

        # L'angle reste identique (pas d'inversion)
        angle_jaune = angle_bleu
        
        return x_jaune, y_jaune, angle_jaune

