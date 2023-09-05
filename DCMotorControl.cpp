/*
    Arduino Library "DCMotorControl"
        Aiming to control DC motor (speed or position) with an encoder.
    Maxime Boulanger, 09/05/2023
*/



#include <Arduino.h>
#include <DCMotorControl.h>



// Constructor

DCMotorControl::DCMotorControl(char PWM_ENABLE, char DO_MOTOR_1, char DO_MOTOR_2, int PinA, int PinB) {
    _PWM_ENABLE = PWM_ENABLE;
    _DO_MOTOR_1 = DO_MOTOR_1;
    _DO_MOTOR_2 = DO_MOTOR_2;
    encoderPinA = PinA;
    encoderPinB = PinB;
}

// Setup function
void DCMotorControl::begin() {
    // Motor pins declaration
    pinMode(_DO_MOTOR_1, OUTPUT);
    pinMode(_DO_MOTOR_2, OUTPUT);
    pinMode(_PWM_ENABLE, OUTPUT);

    // Initialisation of the motor : stopped
    digitalWrite(_DO_MOTOR_1, LOW);
    digitalWrite(_DO_MOTOR_2, LOW);
    analogWrite(_PWM_ENABLE, 0);

    // Encoder pin declaration
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    // SimpleTimer timer;
    // timer.setInterval(1000/freq_echant);
    
    // attachInterrupt(digitalPinToInterrupt(_encoderPinA), updateEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(_encoderPinB), updateEncoder, CHANGE);

    // attachInterrupt(digitalPinToInterrupt(_encoderPinA), [this]() {
    //     updateEncoder();
    // }, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(_encoderPinB), [this]() {
    //     updateEncoder();
    // }, CHANGE);

    Serial.begin(9600);
}

// void DCMotorControl::loop() {

// }

// Function for updating the encoder counter
void DCMotorControl::updateEncoder() {
    int MSB = digitalRead(encoderPinA);  // Read the state of encoder pin A
    int LSB = digitalRead(encoderPinB);  // Read the state of encoder pin B

    int encoded = (MSB << 1) | LSB;  // Convert the two signals into a single value
    int sum = (lastEncoded << 2) | encoded;  // Combine with the previous value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoderCount++;
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoderCount--;
    }

    lastEncoded = encoded;
}


// Function for updating the asservissement of speed for one iteration
void DCMotorControl::getSpeedCommand(){
    // Time
    previous_time = time;
    time = millis()*0.001;
    freq_effectif = 1/(time-previous_time);

    // Calcul of speed
    vitesse_reducteur = ((float)encoderCount / (float)ticks_par_tour) * (float)freq_effectif * 60.0 * rapport_reduction ; //rpm
    
    // Calcul of error
    // last_error = error;

    if (consigne_redcuteur_rpm < 0) {
        sens = 1;
        error = -(float)consigne_redcuteur_rpm + vitesse_reducteur  ; //rpm
    }
    else {
        sens = 0;
        error = (float)consigne_redcuteur_rpm - vitesse_reducteur  ; //rpm
    }

    

    // Reinitializartion of tick number of the encoder
    encoderCount = 0;

    // Calcul de la commande
    // sum_error += error;
    // cmd = Kp * error + Ki *sum_error + Kd * (error - last_error)*(float)freq_echant ;


    cmd = previous_cmd + gamma_rpm*(error - exp(-beta/((float)freq_echant*J)) * previous_error);
    previous_error = error;
    previous_cmd = cmd;
    //cmd = 255;

    // Normalisation de la commande
    if(cmd<0) cmd=0;
    else if (cmd>255) cmd=255;
    // // speed = cmd;

    // if (cmd > 255) cmd=255;
    // else if (cmd < -255) {
    //     cmd = 255;
    //     sens = 1;
    // }

    // // sens
    // if (cmd >= 0) {
    //     sens = 0 ;
    // } 
    // else {
    //     sens = 1 ;
    // }

    // // cmd floor
    // if (cmd>255) {
    //     cmd=255;
    // } 
    // else if (cmd< -255) {
    //     cmd = 255; 
    // } 
}


// Function for updating the asservissement of position for one iteration
void DCMotorControl::getPositionCommand(){
    // Time
    previous_time = time;
    time = millis()*0.001;
    freq_effectif = 1/(time-previous_time);

    // Calcul of angular position reducteur

    angularPosition_reducteur = (float)encoderCount/(float)ticks_par_tour * rapport_reduction * 360  ; // in °

    // Calcul of error
    // last_error = error;
    error = (float)consigne_reducteur_degre - angularPosition_reducteur  ; // in °

    if (error < 0) {
        sens = 1;
        error = -error;
    }
    else {
        sens = 0;
        
    }


    // Reinitialization of tick number of the encoder
    // encoderCount = 0;

    // Calcul de la commande
    cmd = gamma_degre*(error - exp(-beta/((float)freq_echant*J)) * previous_error) ;
    previous_error = error;
    previous_cmd = cmd;

    // Normalisation de la commande
    if(cmd<0) cmd=0;
    else if (cmd>255) cmd=255;
}

// Function to control the DC Motor with voltage input
void DCMotorControl::operateMotor(int sens, int voltageInBits) {
    if (sens == 0){
        // Clockwizse rotation
        digitalWrite(_DO_MOTOR_1, LOW);
        digitalWrite(_DO_MOTOR_2, HIGH);
        //Serial.print("+");
    }
    else{
        // Anti-clockwize rotation
        digitalWrite(_DO_MOTOR_1, HIGH);
        digitalWrite(_DO_MOTOR_2, LOW);
        //Serial.print("-");
    }
    analogWrite(_PWM_ENABLE, voltageInBits);
}


// Function to plot signal in the Serial Plotter
void DCMotorControl::plotSignals() {
    Serial.print("Position_reducteur:");
    Serial.println(angularPosition_reducteur);
    Serial.print(",");
    Serial.print("Vitesse_reducteur:");
    Serial.println(vitesse_reducteur);
    Serial.print(",");
    Serial.print("Error:");
    Serial.println(error);
    Serial.print(",");
    Serial.print("Cmd:");
    Serial.println(cmd);
    Serial.print(",");
    Serial.print("Desired position:");
    Serial.println(consigne_reducteur_degre);
    Serial.print(",");
    Serial.print("Desired speed:");
    Serial.println(consigne_redcuteur_rpm);
    Serial.print(",");
    // Serial.print("dT (s):");
    // Serial.println((time-previous_time));
    // Serial.print(",");
    Serial.print("freq:");
    Serial.println(freq_echant);
}



