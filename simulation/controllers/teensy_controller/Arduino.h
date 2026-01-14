// Arduino.h - Interface minimale Arduino SANS dépendances Webots
// ⚠️ Ce fichier est utilisé par TOUS les fichiers du projet (com.cpp, crc.cpp, pid.cpp, holonomic_basis.cpp)
// ⚠️ Ne PAS inclure webots/robot.h ici ! Seul ArduinoFake.h (pour teensy_controller.cpp) a Webots.
#ifndef ARDUINO_FAKE_H  // Protection : ne charge pas si ArduinoFake.h déjà inclus
#define ARDUINO_MINIMAL_H
#pragma once

#include <stdint.h>
#include <cmath>

// TYPES ARDUINO
typedef uint8_t byte;
typedef bool boolean;

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define PI 3.14159265358979323846
#define M_PI 3.14159265358979323846

// Fonctions Arduino stub (pour les fichiers du projet qui n'ont pas Webots)
inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int value) {}
inline int digitalRead(int pin) { return 0; }
inline void analogWrite(int pin, int value) {}
inline int analogRead(int pin) { return 0; }
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long ms) {}
inline void delayMicroseconds(unsigned int us) {}
inline void interrupts() {}
inline void noInterrupts() {}
inline int abs(int x) { return (x)>0?(x):-(x); }
inline int constrain(int x, int a, int b) { if(x<a) return a; if(x>b) return b; return x; }

// 6. CLASSES STREAM / SERIAL (Pour com.h)
class Stream {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t c) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;
    virtual void flush() {}
};

class usb_serial_class : public Stream {
public:
    virtual void begin(long baud) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(uint8_t c) override { return 0; }
    size_t write(const uint8_t *buffer, size_t size) override { return 0; }
    void flush() override {}
};

class HardwareSerial : public Stream {
public:
    virtual void begin(long baud) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(uint8_t c) override { return 0; }
    size_t write(const uint8_t *buffer, size_t size) override { return 0; }
    void flush() override {}
};

// SerialDummy pour les fichiers du projet qui n'utilisent pas vraiment Serial
class SerialDummy : public Stream {
public:
    void begin(long baud) {}
    int available() override { return 0; }
    int read() override { return -1; }
    size_t write(uint8_t c) override { return 0; }
    size_t write(const uint8_t *buffer, size_t size) override { return 0; }
    void flush() override {}
    void print(const char* s) {}
    void println(const char* s) {}
};

extern SerialDummy Serial;

#endif // ARDUINO_FAKE_H