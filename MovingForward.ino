

// Thomas Klinedinst    EENG 350    3 5 2021 
// This program is designed to make the robot move forwards a set number of inches
// based on the number of rotary encoder counts and the circumference of the 
// wheel. 

// In this case, the desired distance is input from the serial monitor. 

#include <Encoder.h>
#include <DualMC33926MotorShield.h>

#define diameter 0.5 // Diameter of the wheel, in ft
#define countsPerRotation 3200 

Encoder leftWheel(2,1); // 2 is an interrupt pin (A Right)
Encoder rightWheel(3,6); // 3 is an interrupt pin (A Left)

#define motorLPWM         9
#define motorRPWM         10 
#define voltageLDir       7   // Direction for Left motor
#define voltageRDir       8   // Direction for Right motor
#define pinD2             4 

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

int aWriteL;
int aWriteR;

static float uR; // Output gain of the PI controller for the Right wheel 
static float uL; // Output gain of the PI controller for the Left wheel 

void setup() {   
  
  circumference = PI * diameter; // Calculates wheel circumference in ft

  DualMC33926MotorShield(); // Configures the motor shield pin outputs based on 
                            // pins shown in "DualMC33926MotorShield.cpp"

  Serial.begin(9600); 
  Serial.println ("Straight line test");
  Serial.println ("Press r to reset the angular position"); 

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
  
  if (newPositionL != oldPositionR) {   
    if (oldPositionL != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionL = angularPositionL + (newPositionL - oldPositionL) * radPerCount; // Calculated based on position diff. 
    }                                                                                // to allow for easy reset w/o resetting
    oldPositionL = newPositionL;                                                       // position variables. 
    //Serial.println(newPosition);  
    //Serial.print(angularPositionL);
    //Serial.print("  ");
  }   

  if (newPositionR != oldPositionR) {   
    if (oldPositionR != -999) {    // Makes sure angularPosition isn't calculated using the initial value for oldPosition
      angularPositionR = angularPositionR + (newPositionR - oldPositionR) * radPerCount; // Calculated based on position diff. 
    }                                                                                    // to allow for easy reset w/o resetting
    oldPositionL = newPositionR;                                                         // position variables. 
    //Serial.println(newPosition);  
    //Serial.println(angularPositionR);
  } 

   // Mechanism for resetting the positions to zero
  if (Serial.read() == 'r') {   
    angularPositionR = 0; 
    angularPositionL = 0;
    Serial.println("Reset angular positions");
  } 

  // Calculating linear position based on angular positions:
  linearPositionL = angularPositionL * circumference / (float) (2*PI);
  linearPositionR = angularPositionR * circumference / (float) (2*PI);  

  motorVoltageL = speedL * ( (float) 7 / (float) 400 ); 
  motorVoltageR = speedR * ( (float) 7 / (float) 400 );

  
  Serial.println(linearPositionL); 
  Serial.print("     ");
  Serial.print(motorVoltageL);

  Serial.println(linearPositionR);
  Serial.print("     ");
  Serial.print(motorVoltageR);

  //***************************************************************
  //***     Calculating & Driving Motors Based on Control Sys.  *** 
  //***************************************************************

}
