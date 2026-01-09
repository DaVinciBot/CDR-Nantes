#pragma once
#include <windows.h>
#include <iostream>
#include <stdint.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h> // Ajout nécessaire pour les encodeurs

typedef uint8_t byte;
typedef bool boolean;

#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0
#define PI 3.14159265358979323846

// Redirection du temps vers le temps de simulation Webots
inline unsigned long millis() { return (unsigned long)(wb_robot_get_time() * 1000.0); }
inline unsigned long micros() { return (unsigned long)(wb_robot_get_time() * 1000000.0); }
inline void delay(unsigned long ms) { } // Ne rien faire, pour ne pas bloquer Webots
inline void interrupts() {}
inline void noInterrupts() {}
inline int abs(int x) { return (x)>0?(x):-(x); }
inline int constrain(int x, int a, int b) { if(x<a) return a; if(x>b) return b; return x; }

// Classe pour simuler Serial via le port COM Windows
class SerialMock {
    HANDLE hSerial;
    bool connected;
public:
    void begin(long baud) {
        // ATTENTION : Vérifiez que "COM2" est bien votre port virtuel côté Webots !
        hSerial = CreateFile("\\\\.\\COM2", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        if (hSerial == INVALID_HANDLE_VALUE) {
             printf("ERREUR CRITIQUE: Impossible d'ouvrir COM2. Verifiez vos ports virtuels.\n"); 
             connected = false;
        } else {
             printf("SUCCES: Webots connecte sur COM2\n"); 
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
    
    int available() {
        if (!connected) return 0;
        COMSTAT stat; DWORD errors;
        ClearCommError(hSerial, &errors, &stat);
        return stat.cbInQue;
    }
    
    int read() {
        if (!connected) return -1;
        char buffer; DWORD bytesRead;
        if (ReadFile(hSerial, &buffer, 1, &bytesRead, NULL) && bytesRead > 0) return (unsigned char)buffer;
        return -1;
    }
    
    size_t write(const uint8_t *buffer, size_t size) {
        if (!connected) return 0;
        DWORD bytesWritten;
        WriteFile(hSerial, buffer, size, &bytesWritten, NULL);
        return bytesWritten;
    }
    
    size_t write(uint8_t c) { return write(&c, 1); }
    void print(const char* s) { printf("%s", s); } // Affiche dans la console Webots pour le debug
    void println(const char* s) { printf("%s\n", s); }
};

extern SerialMock Serial;