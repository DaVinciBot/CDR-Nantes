// Mock_MKS_SERVO57B.h - Émulation des encodeurs intégrés MKS SERVO57B
// Ce fichier fournit les positions des moteurs via Webots
#pragma once
#include <cstdio>

#ifdef WEBOTS_SIMULATION
#include <webots/robot.h>
#include <webots/position_sensor.h>
#else
#include <Arduino.h>
#endif

class MKS_SERVO57B {
private:
#ifdef WEBOTS_SIMULATION
    WbDeviceTag encoder;
    int motor_id;
#else
    uint8_t uart_tx_pin;
    uint8_t uart_rx_pin;
    int32_t position_steps = 0;
#endif

public:
    // Constructeur compatible simulation et hardware
    MKS_SERVO57B(int motor_id, uint8_t tx_pin = 0, uint8_t rx_pin = 0) {
#ifdef WEBOTS_SIMULATION
        this->motor_id = motor_id;
        char name[32];
        sprintf(name, "encoder%d", motor_id);
        encoder = wb_robot_get_device(name);
        
        if (encoder) {
            wb_position_sensor_enable(encoder, 16); // Lecture toutes les 16ms
            printf("✅ Mock MKS SERVO57B: Encodeur motor%d activé\n", motor_id);
        } else {
            printf("❌ Mock MKS SERVO57B: Encodeur motor%d introuvable\n", motor_id);
        }
#else
        this->uart_tx_pin = tx_pin;
        this->uart_rx_pin = rx_pin;
        // Initialisation UART pour communication avec le driver MKS
        pinMode(tx_pin, OUTPUT);
        pinMode(rx_pin, INPUT);
#endif
    }

    // Lecture de la position en steps (compatible avec le vrai driver)
    int32_t getPosition() {
#ifdef WEBOTS_SIMULATION
        if (!encoder) return 0;
        
        // Webots retourne la position en radians
        double rad = wb_position_sensor_get_value(encoder);
        
        // Conversion radians -> steps
        // 1 tour = 200 steps * 32 microsteps = 6400 steps = 2*PI radians
        const double STEPS_PER_REV = 200.0 * 32.0;
        const double RAD_TO_STEPS = STEPS_PER_REV / (2.0 * M_PI);
        
        return (int32_t)(rad * RAD_TO_STEPS);
#else
        // VRAI MATÉRIEL : Lecture via UART du driver MKS
        // Commande série pour demander la position
        return position_steps;
#endif
    }

    // Commande de vitesse (déjà géré par fake_stepper.cpp via setMaxSpeed)
    void setSpeed(float rpm) {
#ifndef WEBOTS_SIMULATION
        // VRAI MATÉRIEL : Envoyer commande UART au driver MKS
#endif
    }

    // Reset position
    void resetPosition() {
#ifdef WEBOTS_SIMULATION
        // Webots ne permet pas de reset l'encodeur, mais on peut ignorer
        // Car l'odométrie se base sur les deltas
#else
        position_steps = 0;
#endif
    }

    // Commande de pas (utilisé par le contrôle direct)
    void move(int32_t steps, int speed_rpm = 600) {
#ifndef WEBOTS_SIMULATION
        // VRAI MATÉRIEL : Commande UART pour déplacer de X steps
#endif
    }
};