

// Thomas Klinedinst    EENG 350    3 10 2021 
// This program is designed to make the robot move forwards a set number of inches
// based on the number of rotary encoder counts and the circumference of the 
// wheel. 

<<<<<<< Updated upstream
// In this case, the desired distance is hard-coded in the #define goalDistance statement
=======
<<<<<<< HEAD
// In this case, the desired distance is hard-coded from the serial monitor. 
=======
// In this case, the desired distance is hard-coded in the #define goalDistance statement
>>>>>>> 1b9cfe5fb3c7e724bf10ba58e142dd72e0b6bcad
>>>>>>> Stashed changes

#include <Encoder.h>
#include <DualMC33926MotorShield.h>

#define diameter          0.5 // Diameter of the wheel, in ft
#define countsPerRotation 3200 
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4  
#define goalDistance      5     // Linear goal distance in feet (CHANGE PER RUN)

Encoder rightWheel(2,5);  // 2 is an interrupt pin (A/ Yellow Right)    ***DON'T USE PIN 1***
Encoder leftWheel(3,6);   // 3 is an interrupt pin (A/ Yellow Left)   

float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL;   // Built-in variable for the speed of the motor (Left), Between -400, 400
float motorVoltageR;    
float motorVoltageL;

static float uR; // Output gain of the PI controller for the Right wheel 
static float uL; // Output gain of the PI controller for the Left wheel 

DualMC33926MotorShield md;

void setup() {   
  
  circumference = (float) PI * diameter; // Calculates wheel circumference in ft
  md.init(); // Configures the motor shield pin outputs based on pins shown in "DualMC33926MotorShield.cpp"
    
  Serial.begin(9600); 
  Serial.println ("Straight line test");
  Serial.println ("Press r to reset the angular position"); 
  Serial.print ("The robot will drive forwards ");  
  Serial.print(goalDistance);
  Serial.println(" ft");

  // Determining the scaling to convert position to angular position in radians
  // 80 counts per rotation = 80 counts / 2pi rad
  radPerCount = (float) 2 * (PI) / (float) countsPerRotation; 
  
}

long oldPositionL  = -999;   // Included in the provided Basic encoder example code
long oldPositionR  = -999;    

void loop() {
  // put your main code here, to run repeatedly:

  //***************************************************************
  //***     Reading Encoders and Calculating Angular Position   *** 
  //***************************************************************

  long newPositionL = leftWheel.read(); 
  long newPositionR = rightWheel.read(); 
//  Serial.print(leftWheel.read());
//  Serial.print("  ");
//  Serial.println(rightWheel.read());
  if (newPositionL != oldPositionL) {   
    if (oldPositionL != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionL = angularPositionL + (newPositionL - oldPositionL) * radPerCount; // Calculated based on position diff. 
    }                                                                                // to allow for easy reset w/o resetting
    oldPositionL = newPositionL;                                                       // position variables. 
    //Serial.println(newPositionL);  
    //Serial.print(angularPositionL);
    //Serial.print("  ");
  }   

  if (newPositionR != oldPositionR) {   
    if (oldPositionR != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionR = angularPositionR + (newPositionR - oldPositionR) * radPerCount; // Calculated based on position diff. 
    }                                                                                    // to allow for easy reset w/o resetting
    oldPositionR = newPositionR;                                                         // position variables. 
    //Serial.println(newPositionR);  
    //Serial.println(angularPositionR);
  } 

   // Mechanism for resetting the positions to zero
  if (Serial.read() == 'r') {   
    angularPositionR = 0; 
    angularPositionL = 0;
    Serial.println("Reset angular positions");
  } 

  // Calculating linear position based on angular positions:
  linearPositionL = -( angularPositionL * circumference / (float) (2*PI) );   // Negative just to account for the proper direction
  linearPositionR = ( angularPositionR * circumference / (float) (2*PI) );  

  motorVoltageL = speedL * ( (float) 7 / (float) 400 ); 
  motorVoltageR = speedR * ( (float) 7 / (float) 400 );

  Serial.print("LinearPositionL: ");
  Serial.print(linearPositionL);  
  Serial.print("   motorVoltageL: ");
  Serial.print(motorVoltageL);

  Serial.print(" LinearPositionR: ");
  Serial.print(linearPositionR);
  Serial.print("   motorVoltageR: ");
  Serial.println(motorVoltageR);

//  Serial.print(" AngularPositionL: ");
//  Serial.print(angularPositionL);
//  
//  Serial.print(" AngularPositionR: ");
//  Serial.print(angularPositionR);
//  

  //***************************************************************
  //***     Calculating & Driving Motors Based on Control Sys.  *** 
  //***************************************************************

  if (linearPositionL < 5) { 
    speedL = 100;
  } 

  else if (linearPositionL >= 5) { 
    speedL = 0;
  }

  if (linearPositionR < 5) { 
    speedR = -100;
  } 

  else if (linearPositionR >= 5) { 
    speedR = 0;
  } 

  md.setM1Speed(speedL);
  md.setM2Speed(speedR);

  
  

}
