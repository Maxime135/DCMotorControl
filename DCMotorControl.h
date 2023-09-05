/*
    Arduino Library "DCMotorControl"
        Aiming to control DC motor (speed or position) with an encoder.
    Maxime Boulanger, 09/05/2023
*/



#ifndef DCMotorControl_h
#define DCMotorControl_h


#include <Arduino.h>
#include <SimpleTimer.h>

class DCMotorControl {
    public:

        // spinning direction
        int sens = 0; //0 or 1
        // int speed = 0; // between 0 and 255
        int cmd = 0; // between 0 and 255
        volatile float previous_cmd = 0 ; // between 0 and 255

        // desired & actual frequencies
        int freq_echant = 50 ; // Hz
        int freq_effectif = freq_echant; // Hz
        float time = 0; // s
        float previous_time = 0; // s

        // desired output rpm
        int consigne_redcuteur_rpm = 0 ; // rpm

        // motor properties
        float rapport_reduction = 1/297.92 ; //gear ratio
        float beta = 0.4175; // coefficient beta
        float J = 0.0321; // inertia kg.m^2

        // encoder property
        int ticks_par_tour = 12 ; // resolution
        volatile int encoderCount = 0;
        volatile int lastEncoded = 0;
        volatile boolean encoderASet = false;
        volatile boolean encoderBSet = false;

        int encoderPinA; //= 2;  // Interrupt pin for encoder signal A
        int encoderPinB; //= 3;  // Encoder signal B

        // control variables
        float gamma = 10; // gain
        float vitesse_reducteur = 0; //measured speed
        volatile float error = 0;
        volatile float previous_error = 0;
        volatile float last_error = 0;
        volatile float sum_error = 0;
        
        // Constructor (used to create an instance of the class)
        DCMotorControl(
            char PWM_ENABLE,
            char DO_MOTOR_1,
            char DO_MOTOR_2,
            const int encoderPinA,
            const int encoderPinB
        ); 

        // Setup function
        void begin();

        // Loop function
        // void loop();

        // Read the encoder function
        void updateEncoder();

        // Function for updating the asservissement of speed for one iteration
        void getSpeedCommand();

        // Function for updating the asservissement of position for one iteration
        // void asservissementPosition();

        // Function to control the DC Motor with voltage
        void operateMotor(int sens, int voltageInBits);

        // Function to plot signal in the Serial Plotter
        void plotSignals();

        // Function to print signal in the Serial Monitor
        // void printSignals();




    private:
        // DC motor pins
        int _PWM_ENABLE; //= 6; //Pin 1 of the L293D controler
        int _DO_MOTOR_1; //= 5; //Pin 2 of the L293D controler
        int _DO_MOTOR_2; //= 4; //Pin 7 of the L293D controler

        // Encoder pins
        // int _encoderPinA; //= 2;  // Interrupt pin for encoder signal A
        // int _encoderPinB; //= 3;  // Encoder signal B


};

#endif