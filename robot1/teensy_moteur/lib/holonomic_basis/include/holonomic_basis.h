#pragma once
#include <Arduino.h>
#include <config.h>
#include "../../MKSServo/mks_servo.h"
#include "../../MKSServo/mks_group.h"

#include <pid.h>
#include "structures.h"
#include <com.h>

#ifdef WEBOTS_SIMULATION
    // Mode simulation : Mocks
    #include "Mock_PAA5100.h"
    #include "Mock_BNO085.h"
    // Les encodeurs MKS sont gérés par fake_stepper.cpp
#else
    // Mode robot réel : Vraies librairies
    #include <Bitcraze_PMW3901.h>      // Capteur optique
    #include <Adafruit_BNO08x.h>      // IMU
    #include <Adafruit_Sensor.h> 
#endif

class Holonomic_Basis {
   public:
   //Odométrie 
   double X = 0.0;
    double Y = 0.0;
    double THETA = 0.0;

    bool use_encoders = true;       
    bool use_optical_flow = false; 
    bool use_imu = false;            
    bool use_pid_control = false;    


    const double OPTICAL_SCALE = 1.0; // 1.0 Pour webots sinon 0.0423 ( a redefinir) pour réel
    const double OPTICAL_OFFSET_X = 0.0; // mm
    const double OPTICAL_OFFSET_Y = 0.0; // mm
    const double OPTICAL_MOUNT_ANGLE = 0.0; // rad

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
    // Capteurs
    #ifdef WEBOT    S_SIMULATION
        PAA5100* pmw3901 = nullptr; // On garde le Mock en simu
        Adafruit_BNO08x* bno085 = nullptr;
    #else
        // Objet réel Bitcraze
        Bitcraze_PMW3901* pmw3901 = nullptr; 
        Adafruit_BNO08x* bno085 = nullptr;
    #endif

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
    //Capteur
    void init_sensors();
    //Odométrie
    void update_odometry();
    //Boucle principale
    void handle(Point target_position, Com* com);

    // Mouvement
    void run_motors();        // Obsolète (gardé pour compatibilité)
    void execute_movement();
    Point get_current_position();
    void emergency_stop();

    // ===== ARCHITECTURE NON-BLOQUANTE : RS485 HORS ISR =====
    // Ces méthodes doivent être appelées depuis loop(), pas depuis ISR
    
    // Lecture encodeurs (RS485 bloquant 50ms) - À APPELER DEPUIS loop()
    bool read_encoders_nonblocking();
    
    // Envoi vitesses vers MKS (RS485 bloquant 10ms x3) - À APPELER DEPUIS loop()
    bool send_movement_commands_nonblocking();
    
    // Flag pour que loop() sache quand il faut lire/envoyer
    bool need_read_encoders = false;
    bool need_send_movement = false;

   private:
    struct OdometryData {
        // Dernières positions des encodeurs
        int64_t last_enc1 = 0;
        int64_t last_enc2 = 0;
        int64_t last_enc3 = 0;
        
        // ===== BUFFERS NON-BLOQUANTS =====
        // Ces valeurs sont mises à jour par read_encoders_nonblocking() depuis loop()
        // update_odometry() (ISR) lit ces buffers au lieu d'appeler RS485
        int64_t buffered_enc1 = 0;
        int64_t buffered_enc2 = 0;
        int64_t buffered_enc3 = 0;
        uint32_t buffer_timestamp = 0;  // Pour tracking validité du buffer
        
        // Calibration IMU
        double imu_yaw_offset = 0.0;
        bool imu_calibrated = false;

        // Capteur optique PAA5100
        double optical_x_acc = 0.0;
        double optical_y_acc = 0.0;
        
        // Fusion complémentaire (encodeurs + optique)
        double encoder_x_acc = 0.0;      // Accumulation position encodeurs
        double encoder_y_acc = 0.0;
        uint32_t optical_outlier_count = 0; // Nombre de rejets outliers
        uint32_t optical_valid_count = 0;   // Nombre de lectures valides
        
        uint32_t debug_counter = 0;
    } odo_data;

    void update_optical_odometry(double dtheta_robot);
};
