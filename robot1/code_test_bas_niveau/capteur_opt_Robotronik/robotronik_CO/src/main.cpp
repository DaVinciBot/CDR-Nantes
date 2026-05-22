#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

QwiicOTOS otos;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Wire.setSDA(18);
    Wire.setSCL(19);
    Wire.begin();
    delay(200);

    // Scan I2C sur Wire (SDA=18, SCL=19)
    Serial.println("Scan I2C Wire (SDA=18 SCL=19):");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  -> 0x%02X\n", addr);
            found++;
        }
    }
    if (!found) Serial.println("  Aucun device trouve");

    // Tentative begin OTOS
    Serial.println("begin OTOS...");
    if (otos.begin(Wire)) {
        Serial.println("OK");
    } else {
        Serial.println("ECHEC");
    }
}

void loop() {}
