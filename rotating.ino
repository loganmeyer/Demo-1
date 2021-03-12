
// Thomas Klinedinst    EENG 350    3 10 2021 
// This program is designed to make the robot rotate a set number of degrees
// from a reference image, based on the rotation of each individual motor. 

// In this case, the desired angle is hard coded

#include <Encoder.h>
#include <DualMC33926MotorShield.h>

#define diameter          0.5 // Diameter of the wheel, in ft 
#define wheelRadius       0.25  
#define wheelbase         1.167  // Wheelbase, in ft (Logan measured 1.166666)
#define turnRadius        0.584  // Half of the wheelbase (VERIFY)
#define countsPerRotation 3200 
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4  
#define goalAngle         90     // Desired angle of rotation, in degrees (CHANGE PER RUN) *** POSITIVE INDICATES ROBOT NEETS TO ROTATE CCW ***  

float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL;   // Built-in variable for the speed of the motor (Left), Between -400, 400
float endGoalRads; // Final angle the robot must reach
float travelAngleRads; // Angle the robot must travel per iteration (Updated by control system) 
float leftWheelNeededDistance;
float rightWheelNeededDistance;

static float uR; // Output gain of the PI controller for the Right wheel 
static float uL; // Output gain of the PI controller for the Left wheel 

Encoder rightWheel(2,5);  // 2 is an interrupt pin (A/ Yellow Right)    ***DON'T USE PIN 1***
Encoder leftWheel(3,6);   // 3 is an interrupt pin (A/ Yellow Left) 
DualMC33926MotorShield md;

void setup() {   
  
  circumference = (float) PI * diameter; // Calculates wheel circumference in ft
  md.init(); // Configures the motor shield pin outputs based on pins shown in "DualMC33926MotorShield.cpp"
    
  Serial.begin(9600); 
  Serial.println ("Rotating Test");
  Serial.println ("Press r to reset the angular position"); 
  Serial.print ("The robot will turn now:  ");  
  Serial.print(goalAngle);
  Serial.println(" degrees");

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
//  linearPositionL = -( angularPositionL * circumference / (float) (2*PI) );   // Negative just to account for the proper direction
//  linearPositionR = ( angularPositionR * circumference / (float) (2*PI) );  

//  motorVoltageL = speedL * ( (float) 7 / (float) 400 ); 
//  motorVoltageR = speedR * ( (float) 7 / (float) 400 );

  Serial.print(" AngularPositionL: ");
  Serial.print(angularPositionL);
  
  Serial.print(" AngularPositionR: ");
  Serial.print(angularPositionR);  

  // ******************************************************** 
  // *** Determining goal angle and rotating robot ********** 
  // ******************************************************** 

  endGoalRads = PI*(float) goalAngle / (float) 180;

  //**************************************************************
  // Control system here which outputs the needed angle left to go (in rads)
  //**************************************************************

  // Converting the needed angle to go into needed distance per wheel based on s = r*theta 
   if (goalAngle > 0) { 
    leftWheelNeededDistance = - ( travelAngleRads * (float) (turnRadius) ); 
    rightWheelNeededDistance = -leftWheelNeededDistance;
   } 

   else if (goalAngle < 0) { 
    leftWheelNeededDistance = travelAngleRads * (float) (turnRadius);
    rightWheelNeededDistance = -leftWheelNeededDistance;
   } 

  //**************************************************************
  // Same control system to convert needed distance per wheel to speed per wheel
  //**************************************************************

  md.setM1Speed(speedL);
  md.setM2Speed(speedR); 

  //***************************************************************** 
  //*** Move forwards 1 ft ********* 
  //***************************************************************** 

  //**************************************************************
  // Same control system to convert needed distance per wheel to speed per wheel
  //**************************************************************

  md.setM1Speed(speedL);
  md.setM2Speed(speedR); 
}
  
