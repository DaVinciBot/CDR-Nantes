#pragma once
#include <Arduino.h>
#include <config.h>
#include "../../MKSServo/mks_servo.h"
#include "../../MKSServo/mks_group.h"

#include <pid.h>
#include "structures.h"
#include <com.h>

class Holonomic_Basis {
   public:
   //Odométrie 
   double X = 0.0;
    double Y = 0.0;
    double THETA = 0.0;

    bool use_encoders = true;
    bool use_pid_control = false;

    // PID controllers (3 for X, Y, THETA)
    PID x_pid;
    PID y_pid;
    PID theta_pid;

    // Robot geometry parameters
    inline double wheel_circumference() { return this->wheel_diameter * PI; };

    MKSServo* wheel1;  // Front-right wheel (120°)
    MKSServo* wheel2;  // Front-left wheel (240°)
    MKSServo* wheel3;  // Back wheel (0°)
    MKSGroup* mksGroup;

    // Robot parameters
    double robot_radius;      // Distance from center to wheels (mm)
    double wheel_diameter;    // Wheel diameter (mm)
    double max_speed_rpm;     // Maximum wheel speed (RPM)

    // Variables pour stocker les vitesses calculées par le PID
    double last_wheel1_rpm = 0.0;
    double last_wheel2_rpm = 0.0;
    double last_wheel3_rpm = 0.0;

    // Variables pour le filtre passe-bas (lissage des commandes)
    double filtered_wheel1_rpm = 0.0;
    double filtered_wheel2_rpm = 0.0;
    double filtered_wheel3_rpm = 0.0;
    double speed_filter_alpha = 0.3;  // Coefficient de filtrage (0 = pas de filtre, 1 = pas de lissage)

    // Constructor
    Holonomic_Basis(double robot_radius,
                    double wheel_diameter,
                    double max_speed_rpm,
                    const PID& x_pid,
                    const PID& y_pid,
                    const PID& theta_pid);
    ~Holonomic_Basis();

    // Initialization functions
    void define_wheel1(HardwareSerial& serial, uint8_t addr);
    void define_wheel2(HardwareSerial& serial, uint8_t addr);
    void define_wheel3(HardwareSerial& serial, uint8_t addr);

    void init_motors();
    void init_holonomic_basis(double x, double y, double theta);
    void enable_motors();
    void disable_motors();

    // Control PID
    //Odométrie
    void update_odometry();
    //Boucle principale
    void handle(Point target_position, Com* com);

    // Mouvement
    void run_motors();        // Obsolète (gardé pour compatibilité)
    void execute_movement();
    Point get_current_position();
    void set_position(double x, double y, double theta); // Update position from sensors
    void update_from_sensor_deltas(double dx_mm, double dy_mm, double dtheta); // Queue sensor deltas for fusion in update_odometry
    void emergency_stop();

    // ===== ARCHITECTURE NON-BLOQUANTE : RS485 HORS ISR =====
    // Ces méthodes doivent être appelées depuis loop(), pas depuis ISR
    
    // Lecture encodeurs (RS485 bloquant 50ms) - À APPELER DEPUIS loop()
    bool read_encoders_nonblocking();
    
    // Envoi vitesses vers MKS (RS485 bloquant 10ms x3) - À APPELER DEPUIS loop()
    bool send_movement_commands_nonblocking();
    
    // Flag pour que loop() sache quand il faut lire/envoyer (volatile: accès depuis ISR)
    volatile bool need_read_encoders = false;
    volatile bool need_send_movement = false;

   private:
    struct OdometryData {
        // Dernières positions des encodeurs
        int64_t last_enc1 = 0;
        int64_t last_enc2 = 0;
        int64_t last_enc3 = 0;
        
        // ===== BUFFERS NON-BLOQUANTS =====
        // Ces valeurs sont mises à jour par read_encoders_nonblocking() depuis loop()
        // update_odometry() (ISR) lit ces buffers au lieu d'appeler RS485
        volatile int64_t buffered_enc1 = 0;
        volatile int64_t buffered_enc2 = 0;
        volatile int64_t buffered_enc3 = 0;
        volatile uint32_t buffer_timestamp = 0;  // Pour tracking validité du buffer

        // Deltas reçus depuis teensy_capteur (mis à jour dans loop, consommés en ISR)
        volatile double pending_sensor_dx_mm = 0.0;
        volatile double pending_sensor_dy_mm = 0.0;
        volatile double pending_sensor_dtheta = 0.0;
        volatile uint16_t pending_sensor_packets = 0;
        volatile uint32_t last_sensor_packet_ms = 0;
        
        // Statistiques internes (debug possible)
        double encoder_x_acc = 0.0;
        double encoder_y_acc = 0.0;
    } odo_data;
};
