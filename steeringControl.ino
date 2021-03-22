/* Controller
   Function:
   Implement a PI controller for wheel subsystem
   Setting up hardware:
   attach motor driver shield to arduino
   attach motor encoder to motor driver shield
      Green to ground
      Blue to 5V
      Yellow to digital i/o 2
      White to digital i/o 3
   Motor driver shield powered by battery
   Wire RasberryPi to Arduino via I2C
   MODIFY for TWO wheels
*/
 
#include <Wire.h>
#include <Encoder.h>
 
////////////////////MOTOR DRIVER PINS/////////////////////////
//Left Wheel Encoder
#define L_CHAN_A 3              //encoder channel a
#define L_CHAN_B 6              //encoder channel b
//Right Wheel Encoder
#define R_CHAN_A 2              //encoder channel a
#define R_CHAN_B 5              //encoder channel b
//Enable Pin
#define ENABLE 4 
//Right Control
#define R_DRIVE 9 
#define R_DIR 7
//Left Control          
#define L_DRIVE 10
#define L_DIR 8
//Encoders, See Encoder.h for ref
Encoder leftEncoder(L_CHAN_A, L_CHAN_B);
Encoder rightEncoder(R_CHAN_A, R_CHAN_B);
 
////////////////////Physical Constants////////////////////
#define COUNTS_PER_REV 3200;
const double WHEEL_RADIUS = .164; //radius in feet
 
//////////////////Speed and Control Globals////////////////////
const double BASE_SPEED = 150;
const double MIN_SPEED = 70;
uint8_t LDutyCycle;                 //8-bit PWM duty   
uint8_t RDutyCycle;                 //8-bit PWM duty   
double currentPositionR = 0; //position in feet
double currentPositionL = 0;
double finalPosition = 0; //destination in feet
double delta = .01; //acceptable variation in arrival point
double gamma = .5; //slow down point 6in from target
int beta = 50; //catchup adjustment threshold
bool movementEnabled = false;
int turn;
long int angle;
long int turningAngle;
//////////////////I2C and External Comm//////////////////////////
int input;                          //I2C command received from raspi
 
 
 
bool turnDone = false;
 
void setup() {
   Serial.begin(115200);
  //////////////////Pin setup//////////////////
  pinMode(ENABLE, OUTPUT);                  //MUST BE SET HIGH to enable driver board
  pinMode(L_DRIVE, OUTPUT);          
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DRIVE, OUTPUT);          
  pinMode(R_DIR, OUTPUT);
  
  //////////////////Initial Control/Encoder conditions//////////////////
    turn = 2;
    angle = 0;
    turningAngle = 190*10.85;
    Serial.println(turningAngle);
  if(turn==0 || turn == 1){
    //finalPosition = .01;
 }
  switch(turn){
    case 0: 
      digitalWrite(L_DIR, HIGH); //L-H is forward
      digitalWrite(R_DIR, HIGH);
      break;
    case 1:
      digitalWrite(L_DIR, LOW); //L-H is forward
      digitalWrite(R_DIR, LOW);
      break;
    case 2:
      digitalWrite(L_DIR, LOW); //L-H is forward
      digitalWrite(R_DIR, HIGH);
      break;
    
      
  }
  digitalWrite(ENABLE, HIGH); //Enable driver board
  long posL = 0;
  long posR = 0;
  leftEncoder.write(posL);
  rightEncoder.write(posR);
 
 
  /////
  finalPosition = 5;
  delay(1000); //delay 10sec to let you place the robot on track
  Serial.println("Starting");
  leftEncoder.write(0); //reset the encoders
  rightEncoder.write(0);
  movementEnabled = true;
 
 
 
 
  ////////
}
 
 
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  
 
//Main code 
void loop() {
    switch(turn){
    case 0: 
      digitalWrite(L_DIR, HIGH); //L-H is forward
      digitalWrite(R_DIR, HIGH);
      break;
    case 1:
      digitalWrite(L_DIR, LOW); //L-H is forward
      digitalWrite(R_DIR, LOW);
      break;
    case 2:
      digitalWrite(L_DIR, LOW); //L-H is forward
      digitalWrite(R_DIR, HIGH);
      break;
    
      
  }
  //receive user input, we use this to set up the path and start a movement execution
  if(Serial.available()){
    char command = Serial.read();
    switch(command){
      //set destination
      case 'd':
        Serial.println("Please ender a destination position");
        serialFlush(); //clean out any junk
        while(!Serial.available());
        finalPosition = Serial.parseInt();
        Serial.print("Destination Received:  ");
        Serial.println(finalPosition);
        break;
      //start moving
      case 's':
        if(abs(finalPosition-currentPositionR)<delta){
          Serial.println("Please set a destination first!");
          break;
        }else{
          Serial.println("Ok, Robot is starting, please place on track and wait 10sec");
          delay(5000); //delay 10sec to let you place the robot on track
          Serial.println("Starting");
          leftEncoder.write(0); //reset the encoders
          rightEncoder.write(0);
          movementEnabled = true;
        }
        break;
      default:
        break;
    }
  }
  //control loop and movement
  if(movementEnabled && turn==2){
    //set the new current position, based on Right wheel positioning!
    currentPositionR = (rightEncoder.read()/3200.0)*2*3.14*WHEEL_RADIUS*1.51;
    currentPositionL = (leftEncoder.read()/3215.0)*2*3.14*WHEEL_RADIUS*1.51;
    
    Serial.print(currentPositionL);
    Serial.print("   ");
    Serial.print(currentPositionR);
    
    Serial.print("   ");
    Serial.print(LDutyCycle);
    Serial.print("   ");
    Serial.println(RDutyCycle);
    
    //set a speed for both wheels without taking into acount catchup adjustments!
    //check if we are close enough to destination
     if(abs(finalPosition-currentPositionR)<delta){
        LDutyCycle = 0; //stop motor
        RDutyCycle = 0;
        movementEnabled = false; //stop this loop
        turnDone = true;
     //check if within gamma of end point
     }else if(abs(finalPosition-currentPositionR)<gamma){
        LDutyCycle = max((uint8_t)BASE_SPEED*(finalPosition-currentPositionR)*2.0, MIN_SPEED);
        RDutyCycle = max((uint8_t)BASE_SPEED*(finalPosition-currentPositionR)*2.0, MIN_SPEED);
     //check if within gamma of start point
     }else if(currentPositionR < gamma){
        LDutyCycle = min((uint8_t)BASE_SPEED*currentPositionR*2.0+MIN_SPEED, BASE_SPEED); //gamma goes from 0 to .5 so multiply by 2 for smooth transition to BASE_SPEED
        RDutyCycle = min((uint8_t)BASE_SPEED*currentPositionR*2.0+MIN_SPEED, BASE_SPEED);
     //if we are in middle of travel segment go at BASE_SPEED
     }else{
     
      if(abs(finalPosition-currentPositionR)<delta){
          LDutyCycle = 0; //stop motor
          RDutyCycle = 0;
          movementEnabled = false; //stop this loop
          turnDone = true;
      }else{
          LDutyCycle = BASE_SPEED+2;
          RDutyCycle = BASE_SPEED;
          //catchup adjustments (when one wheel is further along the path than the other)
         double L = currentPositionL;
         double R = currentPositionR;
          if(abs(R-L) > .005){
            //Serial.println("adjusting!");
            //Serial.println(R-L);
            LDutyCycle += (R>L)*10;
            RDutyCycle += (L>R)*10;
          }
        }
        
      }
     
     
     analogWrite(L_DRIVE, LDutyCycle); //actually write the motors
     analogWrite(R_DRIVE, RDutyCycle);
  }else if(!turnDone){
    while(angle < turningAngle){
     //Serial.println("Turning");
     analogWrite(L_DRIVE, 90);
     analogWrite(R_DRIVE, 90);
     angle=abs(rightEncoder.read());
     Serial.println(angle);
    }
    //Serial.println(turningAngle);
    //Serial.println("Done Turning");
    analogWrite(L_DRIVE, 0);
    analogWrite(R_DRIVE, 0);
    turn = 2;
    movementEnabled = true;
    rightEncoder.write(0);
    leftEncoder.write(0);
    currentPositionR = 0;
    currentPositionL = 0;
    delay(500);
  }
  /*Serial.println("starting timing sequence");
  leftEncoder.write(0);
  rightEncoder.write(0);
  analogWrite(L_DRIVE, BASE_SPEED);
  analogWrite(R_DRIVE, BASE_SPEED);
  delay(1000);
  analogWrite(L_DRIVE, 0);
  analogWrite(R_DRIVE, 0);
  Serial.println(leftEncoder.read());
  Serial.println(rightEncoder.read());
  delay(1000);*/
}
