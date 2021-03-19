// Thomas Klinedinst Logan Meyer   EENG 350    3 18 2021 
// This program is designed to make the robot move in a straight line 
// a set number of feet and rotate in place a set number of degrees. 
// This uses a PI controller to properly accomplish this. 

#include <Encoder.h>
#include <DualMC33926MotorShield.h>


#define countsPerRotation 3200 
#define motorRPWM         10
#define motorLPWM         9 
#define voltageRDir       7   // Direction for Left motor
#define voltageLDir       8   // Direction for Right motor
#define pinD2             4  
float  ifspin = 1;      //This is set to 0 or one depeding on ifspin ifspintherobot (CHANGE THIS IF TURN TEST OR LINE TEST)

float  goalAngle = 3.14159;   // Desired angle of rotation, in radians (CHANGE PER RUN)  
float  goalDistance = 1;            // Desired linear distance traveled, in ft (CHANGE PER RUN)
float directiongoing = 1; ///  1 if counter clockwise    ////-1 if backwards or clockwise

int counter = 0;


#define diameter 0.5 // Diameter of the wheel, in ft 
#define diametermod 0.5 // More accurate diameter based on testing
#define diametermodspin 0.498     // Different to account for minor slippage on rotation (wheel following curved path)
#define wheelRadius  0.25
#define wheelbase  1.167  // Wheelbase, in ft (Logan measured 1.166666)
#define turnRadius  0.584  // Half of the wheelbase 

float angularPositionL;
float linearPositionL;
float angularPositionR; 
float linearPositionR;
float radPerCount;
float circumference; 
float speedR = 20;//20;   // Built-in variable for the speed of the motor (Right), Between -400, 400
float speedL = 20;//20;   // Built-in variable for the speed of the motor (Left), Between -400, 400
/////control sytem variables here for rotational control


float KpLs = 0.34; // 0.64  Proportional gain of left motor in spin test (unitless)
float KiLs = 0.00019; //0.0001;    0.00019 for pi/2 and under 0.00016 for pi and 0.00012 for 2*pi   Integrator gain of left motor in spin test (v/encoder rads)
float KpRs = 0.34; // 0.64;
float KiRs = 0.00019; //0.0001;

/////control sytem variables here for linear control
float KpL = 0.25; // 0.64;    Proportional gain of left motor in linear test
float KiL = 0.0002; //0.0001;    0.0002 for 5 feet 0.0001 for 10  Integrator gain of left motor in linear test (v/encoder rads)
float KpR = 0.24; // 0.64;
float KiR = 0.0002; //0.0001; 

float delayValue = 120;//40;      // Makes sure the Control system doesn't run twice with the same data (ensures no div/0)
float timetogo = 5000;
float triggertime = 0;
//converted values for goal distance and truning angle
float goalDistanceAngle = (float) 1*goalDistance* (float) 2/ (float) diametermod;
float goalAngleAngle =  goalAngle*(float) wheelbase/ (float) diametermodspin; //wheelbas*wheelbase/r

////////
  ///static control variables
  float rL, rR, yL, yR, eL, eR;
  float IL = 0;
  float IR = 0;
  float Ts = 0;
  float Tc = 0;
  


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

  radPerCount = (float) 2 * (PI) / (float) countsPerRotation;

  KiLs = KiLs-((goalAngle/(2*(PI)))* (float) 0.00007);  //0.00007
  KiRs = KiRs-((goalAngle/(2*(PI)))* (float) 0.00007);

  KiL = KiL - ((goalDistance/((float) 10))* (float) 0.0001);      // Ki is proportional to the requested distance to prevent abrupt stopping
  KiR = KiR - ((goalDistance/((float) 10))* (float) 0.0001);      // Longer distance --> lower Ki since there is available time to spool up/down slowly
  //goalAngleAngle = goalAngleAngle


  if(goalAngle > 3.2){
    goalAngleAngle = goalAngleAngle* (float) 0.985; // Limits overshoot on longer spins. When it spins for a while, it gains momentum and drifts and slides past desired point
    
  }
}

long oldPositionL  = -999;   // Included in the provided Basic encoder example code
long oldPositionR  = -999;    

void loop() {
    
  static float uR = speedR;
  static float uL = speedL;
  
  if((triggertime > 0) && (millis() > (triggertime+timetogo))){
    if(counter == 0){
      
      goalDistanceAngle = (float) 2/ (float) diametermod;
      ifspin = 0;
      angularPositionR = 0;
      angularPositionL = 0;
      
      IL = 0;
      IR = 0;
      Ts = 0;
      //Tc = 0;
      uR = speedR;
      uL = speedL;
      counter  = counter + 1;
      Serial.println("loop");
    }
    //Serial.println("loop");
  }


  
  long newPositionL = leftWheel.read(); 
  long newPositionR = rightWheel.read(); 
  if (newPositionL != oldPositionL) {   
    if (oldPositionL != -999) {    
      angularPositionL = angularPositionL + (newPositionL - oldPositionL) * radPerCount;  
    }                                                                                
    oldPositionL = newPositionL;                                                     

  }   

  if (newPositionR != oldPositionR) {   
    if (oldPositionR != -999) {    
      angularPositionR = angularPositionR + (newPositionR - oldPositionR) * radPerCount;  
    }                                                                                    
    oldPositionR = newPositionR;                                                        

  } 
/*
  if (Serial.read() == 'r') {   
    angularPositionR = 0; 
    angularPositionL = 0;
    Serial.println("Reset angular positions");
  } 
*/
/*
  Serial.print(" L: ");
  Serial.print(angularPositionL);
  
  Serial.print("          ||         R: ");
  Serial.print(angularPositionR); 
  Serial.print("\n"); 

*/

  if(ifspin == 1){
    rL = goalAngleAngle;
    rR = goalAngleAngle;
    yL = angularPositionL*directiongoing;
    yR = angularPositionR*directiongoing;

    eL = rL-yL;
    eR = rR-yR;
    
    IL = IL + Ts*eL;
    IR = IR + Ts*eR;

    uL = KpLs*eL + KiLs*IL;
    uR = KpRs*eR + KiRs*IR;

    if(directiongoing > 0){
      if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
        uL = uL - KiLs*IL;
        if(triggertime == 0){
          triggertime = millis();
        }
    }
      if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
        uR = uR - KiRs*IR;
        if(triggertime == 0){
          triggertime = millis();
        }
    }
   }
   
   if(directiongoing < 0){
      if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
        uL = uL - KiLs*IL;
        if(triggertime == 0){
          triggertime = millis();
        }
    }
      if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
        uR = uR - KiRs*IR;
        if(triggertime == 0){
          triggertime = millis();
        }
    }
   }


  uL = uL*directiongoing;
  uR = uR*directiongoing;


  }
  if(ifspin == 0){
    rL = goalDistanceAngle;
    rR = goalDistanceAngle;
    yL = angularPositionL*directiongoing*((float) -1);  // y is actual angular position
    yR = angularPositionR*directiongoing;

    eL = rL-yL;
    eR = rR-yR;

 
    IL = IL + Ts*eL;
    IR = IR + Ts*eR;

    uL = KpL*eL + KiL*IL;
    uR = KpR*eR + KiR*IR;

    if((((rL-yL))<0.1) || ((yL-rL)>0.1)){
   
      uL = uL - KiL*IL;
    }
    if((((rR-yR))<0.1) || ((yR-rR)>0.1)){
      uR = uR - KiR*IR;
    }

    uL = uL*directiongoing*((float) -1);
    uR = uR*directiongoing;

  }

  Serial.print(" L: ");
  Serial.print(uL);
  
  Serial.print("          ||         R: ");
  Serial.print(uR); 
  Serial.print("\n"); 
 


  Ts = millis()-Tc;
  Tc = millis();
  delay(delayValue - Ts);
  
  md.setM1Speed(speedL*uL* (float) -1);
  md.setM2Speed(speedR*uR* (float) -1);
  


 
}
