#pragma once
#include <Arduino.h>

#define LIDAR_MOTOCTRL_PIN  2       // pin PWM → MOTOCTRL du RPLIDAR
#define LIDAR_PWM_FREQ      25000   // 25 kHz
#define LIDAR_PWM_MAX       1023    // résolution 10-bit
#define LIDAR_SPEED_LOW     300     // ~30% duty cycle (faible)
#define LIDAR_SPEED_NORMAL  700     // ~68% duty cycle (normal)

inline void lidar_pwm_init() {
    analogWriteResolution(10);
    analogWriteFrequency(LIDAR_MOTOCTRL_PIN, LIDAR_PWM_FREQ);
    analogWrite(LIDAR_MOTOCTRL_PIN, 0);
}

inline void lidar_set_speed(int speed) {
    analogWrite(LIDAR_MOTOCTRL_PIN, constrain(speed, 0, LIDAR_PWM_MAX));
}

inline void lidar_stop()         { lidar_set_speed(0); }
inline void lidar_speed_low()    { lidar_set_speed(LIDAR_SPEED_LOW); }
inline void lidar_speed_normal() { lidar_set_speed(LIDAR_SPEED_NORMAL); }
