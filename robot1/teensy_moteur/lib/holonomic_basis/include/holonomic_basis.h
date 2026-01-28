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

    bool use_encoders = true;       // ÉTAPE 1
    bool use_optical_flow =true ;  //ÉTAPE 2
    bool use_imu = true;           // ÉTAPE 3
    bool use_pid_control = true;


    const double OPTICAL_SCALE = 1.0; // 1.0 Pour webots sinon 0.0423 pour réel
    const double OPTICAL_OFFSET_X = 0.0; // mm
    const double OPTICAL_OFFSET_Y = 0.0; // mm
    const double OPTICAL_MOUNT_ANGLE = 0.0; // rad

    // PID controllers (3 for X, Y, THETA)
    PID x_pid;
    PID y_pid;
    PID theta_pid;

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
    #ifdef WEBOTS_SIMULATION
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

        //Capteur optique PAA5100
        double optical_x_acc = 0.0;
        double optical_y_acc = 0.0;
        
        uint32_t debug_counter = 0;
    } odo_data;

    void update_optical_odometry(double dtheta_robot);
};
