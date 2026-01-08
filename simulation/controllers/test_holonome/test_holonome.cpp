#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <iostream>

// Configuration
#define TIME_STEP 64
#define MAX_SPEED 10.0

using namespace webots;

int main(int argc, char **argv) {
  // 1. Création de l'instance du Robot
  Robot *robot = new Robot();

  // 2. Récupération des moteurs
  // (Les noms doivent correspondre EXACTEMENT à votre fichier .wbt)
  Motor *motor1 = robot->getMotor("motor1"); // Avant Droite
  Motor *motor2 = robot->getMotor("motor2"); // Avant Gauche
  Motor *motor3 = robot->getMotor("motor3"); // Arrière

  // 3. Initialisation des moteurs (Mode Vitesse)
  motor1->setPosition(INFINITY);
  motor2->setPosition(INFINITY);
  motor3->setPosition(INFINITY);

  motor1->setVelocity(0.0);
  motor2->setVelocity(0.0);
  motor3->setVelocity(0.0);

  std::cout << "--- DEMARRAGE DU TEST OMNIDIRECTIONNEL ---" << std::endl;

  int compteur = 0;

  // 4. Boucle principale
  while (robot->step(TIME_STEP) != -1) {
    double v1 = 0;
    double v2 = 0;
    double v3 = 0;

    // --- SEQUENCE DE TEST (Toutes les 200 itérations ~= 6 secondes) ---
    
    // ETAPE A : Tourner sur place (Rotation)
    // Utile pour vérifier que toutes les roues tournent dans le même sens
    if (compteur < 100) {
        if(compteur == 0) std::cout << "Test: Rotation sur place" << std::endl;
        v1 = 5.0;
        v2 = 5.0;
        v3 = 5.0;
    }
    // ETAPE B : Avancer tout droit (X)
    // La roue arrière (3) ne doit pas tourner (ou très peu)
    else if (compteur < 200) {
        if(compteur == 100) std::cout << "Test: Avancer (Axe X)" << std::endl;
        v1 = -5.0; 
        v2 = 5.0;
        v3 = 0.0; 
    }
    // ETAPE C : Glisser sur le côté (Y)
    // Déplacement en crabe
    else if (compteur < 300) {
        if(compteur == 200) std::cout << "Test: Glisser (Axe Y)" << std::endl;
        v1 = -3.0;
        v2 = -3.0;
        v3 = 6.0;
    }
    // ETAPE D : Stop
    else {
        if(compteur == 300) std::cout << "Test: Fini." << std::endl;
        v1 = 0; v2 = 0; v3 = 0;
    }

    // Envoi des commandes
    motor1->setVelocity(v1);
    motor2->setVelocity(v2);
    motor3->setVelocity(v3);

    compteur++;
  }

  delete robot;
  return 0;
}