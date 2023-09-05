#include <DCMotorControl.h>

DCMotorControl motorObject(6, 5, 4, 2, 3);

void setup() {
  //Motor object initialisation
  motorObject.begin(); 

  // Some motor parameters tunning
  motorObject.rapport_reduction = 1/297.92;
  motorObject.J = 0.0321;
  motorObject.beta = 0.4175; 

  // Encoder resolution definition
  motorObject.ticks_par_tour = 12;

  // Control parameter
  motorObject.gamma = 10;

  // attachInterrupt have to be defined outside the library for the encoder...
  attachInterrupt(digitalPinToInterrupt(motorObject.encoderPinA), motorObject_updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorObject.encoderPinB), motorObject_updateEncoder, CHANGE);
}

void loop() {

  // Run the speed control function related to motorObject
  motorObject.getSpeedCommand();
  
  // Define the desired motor speed
  motorObject.consigne_redcuteur_rpm = 25*sin(2*3.14*0.3*motorObject.time);

  // Give the voltage and spinning direction inputs provied by the getSpeedCommand function as the actual inputs for motorObject
  motorObject.operateMotor(motorObject.sens, motorObject.cmd);

  // Plot several useful signals in the Serial monitor and plotter
  motorObject.plotSignals();
}


// Has to be included manually in your script
void motorObject_updateEncoder() {                  //needs to be separated from the library definition because of pointers in C++
  int MSB = digitalRead(motorObject.encoderPinA);  // Read the state of encoder pin A
  int LSB = digitalRead(motorObject.encoderPinB);  // Read the state of encoder pin B

  int encoded = (MSB << 1) | LSB;                      // Convert the two signals into a single value
  int sum = (motorObject.lastEncoded << 2) | encoded;  // Combine with the previous value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    motorObject.encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    motorObject.encoderCount--;
  }

  motorObject.lastEncoded = encoded;
}
