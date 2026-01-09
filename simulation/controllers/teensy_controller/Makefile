
# Si un membre de l'équipe a un chemin différent, il le met dedans.
# Le "-" devant include signifie "ne plante pas si le fichier n'existe pas".
-include webots_path.mk

# 2. Définition des chemins par défaut (si pas définis dans le fichier local)
ifndef WEBOTS_HOME
  # Détection automatique de l'OS
  ifeq ($(OS),Windows_NT)
    # Chemin standard Windows
    WEBOTS_HOME = C:/Program Files/Webots
  else
    # Chemin standard Linux / macOS
    WEBOTS_HOME = /usr/local/webots
  endif
endif

# 3. Vérification de sécurité (Est-ce que le chemin est valide ?)
# On gère les espaces dans "Program Files" pour le test
null :=
space := $(null) $(null)
WEBOTS_HOME_ESCAPED = $(subst $(space),\ ,$(WEBOTS_HOME))

ifeq ($(wildcard $(WEBOTS_HOME_ESCAPED)/resources/Makefile.include),)
  $(error [ERREUR CRITIQUE] Webots introuvable dans "$(WEBOTS_HOME)". \
  SOLUTION: Créez un fichier 'webots_path.mk' dans ce dossier contenant: \
  WEBOTS_HOME = D:/Mon/Chemin/Vers/Webots)
endif

# -------------------------------------------------------------
# Configuration des sources du projet
# ------------------------------------------------------------

# 1. Sources locales (nos fichiers fakes)
CXX_SOURCES = teensy_controller.cpp fake_stepper.cpp

# 2. Chemin vers la racine du repo (A adapter si besoin !)
# On remonte 3 fois : controller -> controllers -> simulation -> CDR-Nantes
ROOT_DIR = ../../../

# 3. Sources de VOTRE projet (Le vrai code du robot)
# Vérifiez que ces chemins existent bien sur votre disque
CXX_SOURCES += $(ROOT_DIR)/common/usb_com/cpp/src/com.cpp \
               $(ROOT_DIR)/common/usb_com/cpp/src/crc.cpp \
               $(ROOT_DIR)/robot1/teensy_moteur/lib/holonomic_basis/src/holonomic_basis.cpp \
               $(ROOT_DIR)/robot1/teensy_moteur/lib/pid/src/pid.cpp

# 4. Dossiers d'Includes (.h)
INCLUDE = -I"$(ROOT_DIR)/common/usb_com/cpp/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/holonomic_basis/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/KaribouMotion" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/pid/include" \
          -I"."

# Inclusion de ArduinoFake partout pour que "Serial" soit connu
CFLAGS += -include "ArduinoFake.h"

# Inclusion du Makefile standard de Webots
include $(WEBOTS_HOME)/resources/Makefile.include