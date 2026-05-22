#pragma once
#include <Arduino.h>

#define TIRETTE_PIN 34

inline void tirette_init() {
    pinMode(TIRETTE_PIN, INPUT_PULLDOWN);
}

// Retourne true si le cable est en place (pin HIGH = 3.3V)
inline bool tirette_is_inserted() {
    return digitalRead(TIRETTE_PIN) == HIGH;
}

// Bloque jusqu'au retrait de la tirette.
// Attend d'abord que la tirette soit en place, puis attend son retrait.
inline void tirette_wait_for_start() {
    // Phase 1 : attendre que la tirette soit insérée
    if (!tirette_is_inserted()) {
        Serial.println("[TIRETTE] En attente d'insertion...");
        while (!tirette_is_inserted()) {
            delay(50);
        }
        Serial.println("[TIRETTE] Tirette inseree. Robot pret.");
        delay(200); // anti-rebond
    }

    // Phase 2 : attendre le retrait pour lancer la sequence
    Serial.println("[TIRETTE] En attente du retrait pour demarrage...");
    while (tirette_is_inserted()) {
        delay(10);
    }
    delay(50); // anti-rebond
    Serial.println("[TIRETTE] Tirette retiree ! Lancement de la sequence.");
}
