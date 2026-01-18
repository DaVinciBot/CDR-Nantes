#pragma once
#include <Arduino.h>
#include "../../KaribouMotion/stepper.h" //Librairie Maison KaribouMotion

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
    #include <PAA5100.h>              // Capteur optique
    #include <Adafruit_BNO085.h>      // IMU
    #include <Adafruit_Sensor.h>
    #include <sh2.h>
    #include <sh2_SensorValue.h>
#endif

class Holonomic_Basis {
   public:
   //Odométrie 
   double X = 0.0;
    double Y = 0.0;
    double THETA = 0.0;

    // PID controllers (3 for X, Y, THETA)
    PID x_pid;
    PID y_pid;
    PID theta_pid;
    bool use_pid_control = false;
    // Robot geometry parameters
    inline double wheel_circumference() { return this->wheel_diameter * PI; };

    // Stepper motors (KaribouMotion)
    // On utilise nos propres classes Stepper maintenant
    Stepper* wheel1;  // Front wheel (0°)
    Stepper* wheel2;  // Back-left wheel (120°)
    Stepper* wheel3;  // Back-right wheel (240°)
    
    // Le groupe de synchronisation KaribouMotion
    StepperGroup* stepperGroup;
    // Capteurs
    PAA5100* paa5100 = nullptr; //capteur Optique
    Adafruit_BNO085* bno085 = nullptr; //IMU

    // Robot parameters
    double robot_radius;      // Distance from center to wheels (mm)
    double wheel_diameter;    // Wheel diameter (mm)
    double max_speed;         // Maximum speed (steps/sec)
    double max_acceleration;  // Maximum acceleration (steps/sec²)
    unsigned short steps_per_revolution;
    unsigned short microsteps;

    // Variables pour stocker les vitesses calculées par le PID
    double last_wheel1_speed = 0.0;
    double last_wheel2_speed = 0.0;
    double last_wheel3_speed = 0.0;

    // Constructor
    Holonomic_Basis(double robot_radius,
                    double wheel_diameter,
                    double max_speed,
                    double max_acceleration,
                    unsigned short steps_per_revolution,
                    unsigned short microsteps,
                    const PID& x_pid,
                    const PID& y_pid,
                    const PID& theta_pid);

    ~Holonomic_Basis();

    // Initialization functions
    void define_wheel1(byte step_pin, byte dir_pin, byte enable_pin);
    void define_wheel2(byte step_pin, byte dir_pin, byte enable_pin);
    void define_wheel3(byte step_pin, byte dir_pin, byte enable_pin);

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
    void execute_movement();  // Convertit vitesse -> pas relatifs
    
    void compute_steppers();  // Calcul des profils (Timer Lent)
    void step_steppers();     // Génération des pas (Timer Rapide)

    Point get_current_position();
    void emergency_stop();

   private:
    byte wheel1_enable_pin;
    byte wheel2_enable_pin;
    byte wheel3_enable_pin;

    struct OdometryData {
        // Dernières positions des encodeurs
        int32_t last_pos1 = 0;
        int32_t last_pos2 = 0;
        int32_t last_pos3 = 0;
        
        // Calibration IMU
        double imu_yaw_offset = 0.0;
        bool imu_calibrated = false;
        
        // Compteurs debug
        uint32_t fusion_counter = 0;
        uint32_t debug_counter = 0;
    } odo_data;
};
