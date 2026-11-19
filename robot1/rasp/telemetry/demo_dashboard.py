"""Lance le tableau de bord avec une trajectoire robot synthetique."""

import math
import time

from robot1.rasp.telemetry.dashboard import RobotDashboard


def main() -> None:
    """Affiche un mouvement circulaire sans initialiser le robot reel."""
    interface = RobotDashboard(title="Démo supervision robot — CDR Nantes")
    debut = time.monotonic()
    duree_tour = 24.0
    rayon_x = 900.0
    rayon_y = 600.0

    def fournir_pose() -> dict[str, object]:
        temps = time.monotonic() - debut
        angle = 2.0 * math.pi * temps / duree_tour
        x = 1500.0 + rayon_x * math.cos(angle)
        y = 1000.0 + rayon_y * math.sin(angle)
        theta = math.atan2(
            rayon_y * math.cos(angle),
            -rayon_x * math.sin(angle),
        )
        return {
            "x": x,
            "y": y,
            "theta": theta,
            "team": "BLUE",
            "mode": "DÉMO",
        }

    interface.set_data_provider(fournir_pose)
    interface.set_status("Simulation en cours")

    def actualiser_chronometre() -> None:
        temps = time.monotonic() - debut
        interface.set_match_time(temps, max(0.0, 100.0 - temps))
        interface.root.after(100, actualiser_chronometre)

    interface.root.after(0, actualiser_chronometre)
    interface.root.mainloop()


if __name__ == "__main__":
    main()
