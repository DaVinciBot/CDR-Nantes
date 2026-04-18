"""
strat_manager.py
La machine à états qui dicte les missions du robot.
"""
import logging
from .actions import Action, TypeAction

class StratManager:
    def __init__(self, couleur_equipe: str):
        self.logger = logging.getLogger("STRAT")
        self.couleur_equipe = couleur_equipe
        self.etape_actuelle = 0 
        self.liste_actions = self._generer_strategie()
        
        # UTILISATION D'UNE SENTINELLE 'None' (Au lieu de 0.0)
        self.chrono_action = None

    def _generer_strategie(self):
        """Génère la liste des actions à effectuer pendant les 90s."""
        actions = []
        
        # Exemple de match :
        # 1. Sortir de la zone de départ
        actions.append(Action(TypeAction.DEPLACEMENT, cible_x=500, cible_y=500))
        
        # 2. Aller devant un pot
        actions.append(Action(TypeAction.DEPLACEMENT, cible_x=1200, cible_y=800))
        
        # 3. Baisser le bras
        actions.append(Action(TypeAction.ACTIONNEUR, nom_actionneur="BAISSER_BRAS"))
        
        # 4. Attendre 1 seconde que le bras descende
        actions.append(Action(TypeAction.ATTENTE, temps_attente=1.0))
        
        # 5. Retourner à la base
        actions.append(Action(TypeAction.DEPLACEMENT, cible_x=200, cible_y=200))
        
        return actions

    def get_action_actuelle(self):
        """Renvoie la mission que le robot doit faire maintenant."""
        if self.etape_actuelle < len(self.liste_actions):
            return self.liste_actions[self.etape_actuelle]
        else:
            return None # Plus aucune action à faire, le match est fini !

    def demarrer_chrono_si_necessaire(self):
        """Lance le chrono si l'action vient de débuter."""
        if self.chrono_action is None:
            self.chrono_action = time.time()
            return True
        return False

    def chrono_ecoule(self, duree):
        """Vérifie si le temps imparti est dépassé."""
        if self.chrono_action is None:
            return False
        return (time.time() - self.chrono_action) >= duree

    def valider_action_terminee(self):
        """Passe à la mission suivante et réinitialise le chrono."""
        if self.etape_actuelle < len(self.liste_actions):
            action_finie = self.liste_actions[self.etape_actuelle]
            self.logger.info(f"✅ Action terminée : {action_finie.type.name}")
            
            self.etape_actuelle += 1
            # ON RÉINITIALISE LE CHRONO ICI
            self.chrono_action = None