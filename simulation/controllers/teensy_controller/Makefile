# -------------------------------------------------------------
# Makefile Universel (Corrigé pour éviter les conflits)
# -------------------------------------------------------------

# 1. Inclusion de la config locale (ex: E:/Webots)
-include webots_path.mk

# 2. Définition des chemins par défaut
ifndef WEBOTS_HOME
  ifeq ($(OS),Windows_NT)
    WEBOTS_HOME = C:/Program Files/Webots
  else
    WEBOTS_HOME = /usr/local/webots
  endif
endif

# 3. Vérification de sécurité
ifeq ($(wildcard $(WEBOTS_HOME)/resources/Makefile.include),)
  $(error [ERREUR] Webots introuvable dans "$(WEBOTS_HOME)". Verifiez webots_path.mk)
endif

# -------------------------------------------------------------
# CONFIGURATION DES SOURCES
# -------------------------------------------------------------

# 1. Sources locales (Simulation) - tous les fichiers à la racine
CXX_SOURCES = teensy_controller.cpp fake_stepper.cpp ArduinoFake.cpp

# 2. Chemin vers la racine du repo
ROOT_DIR = ../../..

# 3. Sources du vrai code robot (Librairies partagées)
CXX_SOURCES += $(ROOT_DIR)/common/usb_com/cpp/src/com.cpp \
               $(ROOT_DIR)/common/usb_com/cpp/src/crc.cpp \
               $(ROOT_DIR)/robot1/teensy_moteur/lib/holonomic_basis/src/holonomic_basis.cpp \
               $(ROOT_DIR)/robot1/teensy_moteur/lib/pid/src/pid.cpp

# 4. Dossiers d'Includes (.h)
INCLUDE = -I"." \
          -I"./sensors" \
          -I"$(WEBOTS_HOME)/include/controller/c" \
          -I"$(WEBOTS_HOME)/include/controller/cpp" \
          -I"$(ROOT_DIR)/common/usb_com/cpp/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/holonomic_basis/include" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/KaribouMotion" \
          -I"$(ROOT_DIR)/robot1/teensy_moteur/lib/pid/include" \
          -I"$(ROOT_DIR)/simulation/controllers/teensy_controller/sensors"
# -------------------------------------------------------------
# FLAGS DE COMPILATION (Nettoyage des conflits)
# -------------------------------------------------------------

# On retire les définitions Windows qui gênent (KP_X, KP_Y)
CFLAGS += -UKP_X -UKP_Y -DWEBOTS_SIMULATION

# IMPORTANT : Force l'inclusion de ArduinoFake.h pour les fichiers spécifiques
build/release/teensy_controller.o: CFLAGS += -include "ArduinoFake.h"
build/release/fake_stepper.o: CFLAGS += -include "ArduinoFake.h"

# Lancement de la compilation standard Webots
include $(WEBOTS_HOME)/resources/Makefile.include