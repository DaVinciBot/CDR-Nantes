// ArduinoFake.h - Implémentation Webots avec SerialMock
// Ce fichier est UNIQUEMENT pour teensy_controller.cpp (via -include dans Makefile)
#ifndef ARDUINO_FAKE_H
#define ARDUINO_FAKE_H
#pragma once

// IMPORTANT : Empêcher Arduino.h de se charger si ArduinoFake.h est inclus
#ifndef ARDUINO_H
#define ARDUINO_H  // On bloque Arduino.h
#endif

#include <stdint.h>
#include <iostream>
#include <cmath>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

// Inclure Windows en dernier pour pouvoir nettoyer les conflits
#include <windows.h>

// Nettoyer les conflits Windows APRÈS l'inclusion
#undef min
#undef max
#undef KP_X       // wincrypt.h définit KP_X=14
#undef KP_Y       // wincrypt.h définit KP_Y=15
#undef boolean    // rpcndr.h définit boolean comme unsigned char

typedef uint8_t byte;

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define PI 3.14159265358979323846
#define M_PI 3.14159265358979323846

// Fonctions Arduino de base
inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int value) {}
inline int digitalRead(int pin) { return 0; }
inline void analogWrite(int pin, int value) {}
inline int analogRead(int pin) { return 0; }

// Temps Webots
inline unsigned long millis() { return (unsigned long)(wb_robot_get_time() * 1000.0); }
inline unsigned long micros() { return (unsigned long)(wb_robot_get_time() * 1000000.0); }
inline void delay(unsigned long ms) {}
inline void delayMicroseconds(unsigned int us) {}
inline void interrupts() {}
inline void noInterrupts() {}
inline int abs(int x) { return (x)>0?(x):-(x); }
inline int constrain(int x, int a, int b) { if(x<a) return a; if(x>b) return b; return x; }

// Classes Stream pour com.h
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

// SerialMock pour Webots avec vrai port COM Windows
class SerialMock : public usb_serial_class {
    HANDLE hSerial;
    bool connected;
public:
    SerialMock() : hSerial(INVALID_HANDLE_VALUE), connected(false) {}

    void begin(long baud) override {
        hSerial = CreateFile("\\\\.\\COM2", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        if (hSerial == INVALID_HANDLE_VALUE) {
             printf("[Webots] ⚠️  COM2 non disponible. Mode simulation hors ligne.\n"); 
             connected = false;
        } else {
             printf("[Webots] ✅ COM2 connecté !\n"); 
             connected = true;
             DCB dcbSerialParams = {0};
             dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
             GetCommState(hSerial, &dcbSerialParams);
             dcbSerialParams.BaudRate = baud;
             dcbSerialParams.ByteSize = 8;
             dcbSerialParams.StopBits = ONESTOPBIT;
             dcbSerialParams.Parity = NOPARITY;
             SetCommState(hSerial, &dcbSerialParams);
             COMMTIMEOUTS timeouts = {MAXDWORD, 0, 0, 0, 0};
             SetCommTimeouts(hSerial, &timeouts);
        }
    }
    
    int available() override {
        if (!connected) return 0;
        COMSTAT stat; DWORD errors;
        ClearCommError(hSerial, &errors, &stat);
        return stat.cbInQue;
    }
    
    int read() override {
        if (!connected) return -1;
        char buffer; DWORD bytesRead;
        if (ReadFile(hSerial, &buffer, 1, &bytesRead, NULL) && bytesRead > 0) return (unsigned char)buffer;
        return -1;
    }
    
    size_t write(const uint8_t *buffer, size_t size) override {
        if (!connected) return 0;
        DWORD bytesWritten;
        WriteFile(hSerial, buffer, size, &bytesWritten, NULL);
        return bytesWritten;
    }
    
    size_t write(uint8_t c) override { return write(&c, 1); }
    void flush() override { if (connected) FlushFileBuffers(hSerial); }
    void print(const char* s) { printf("%s", s); }
    void println(const char* s) { printf("%s\n", s); }
};

extern SerialMock Serial;

#endif // ARDUINO_FAKE_H