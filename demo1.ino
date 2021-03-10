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

//variable declarations
unsigned long timeNow;
//Left Wheel
const int LchanA = 2;              //encoder channel a
const int LchanB = 5;              //encoder channel b
//Right Wheel
const int RchanA = 3;              //encoder channel a
const int RchanB = 6;              //encoder channel b
const int enable = 4;             //Hardware set up
int samplingTime;                 //rate between 5-10 ms
int LvoltageCommand = 10;          //pwm pin controlling motor
int LvoltageDirection = 8;         //Controls Voltage Direction
int RvoltageCommand = 9;          //pwm pin controlling motor
int RvoltageDirection = 7;         //Controls Voltage Direction
double LdutyCycle;                 //Controls speed of motor   
double RdutyCycle;                 //Controls speed of motor    
int input;                        //Value comes from I2C connection
double LposA, RposA, posD;                //in radians/sec
int Lcount, Rcount;                        //counts to determine velocity
double countsPerRevolution;       //constant with motor
double Lerror, Rerror;                     //Error between actual and desired position
double LKp, LKi, RKp, RKi;                    //PI control constants
double wheelRadius;

//interrupt associated with L encoder
void LwheelTurn() {
  //determine if clockwise or counterclockwise and increment accordingly
  if (digitalRead(LchanA) != digitalRead(LchanB)) {
    Lcount++;
  }
  else {
    Lcount--;
  }
}

//interrupt associated with R encoder
void RwheelTurn() {
  //determine if clockwise or counterclockwise and increment accordingly
  if (digitalRead(RchanA) != digitalRead(RchanB)) {
    Rcount++;
  }
  else {
    Rcount--;
  }
}

void setup() {
  pinMode(LchanA, INPUT_PULLUP);             
  pinMode(LchanB, INPUT_PULLUP);
  pinMode(RchanA, INPUT_PULLUP);             
  pinMode(RchanB, INPUT_PULLUP);
  pinMode(enable, OUTPUT);                  //MUST BE SET HIGH
  pinMode(LvoltageCommand, OUTPUT);          //PWM
  pinMode(LvoltageDirection, OUTPUT);
  pinMode(RvoltageCommand, OUTPUT);          //PWM
  pinMode(RvoltageDirection, OUTPUT);
  digitalWrite(LvoltageDirection,HIGH);
  digitalWrite(RvoltageDirection,HIGH);
  digitalWrite(enable,HIGH);
  Serial.begin(115200);                     //Must change baud rate in serial monitor as well
  samplingTime = 10;                        //sampling time in ms
  countsPerRevolution = 3200;               //counts per rev on spec sheet (specific to motor)
  input = 10;
  posD = input * 3.14/2.0;  
  LposA = 0;   
  RposA = 0;                    
  Lcount = 0;  
  Rcount = 0;    
  Lerror = 0;
  Rerror = 0;                                 
  LdutyCycle = 200;                          //will modify angular velocity of motor
  RdutyCycle = 200;
  RKp = 6;
  LKp = 6;
  RKi = 0.08;
  RKi = 0.08;
  wheelRadius = .05;                         //radius in meters


  //enable interrupt
  attachInterrupt(digitalPinToInterrupt(2), LwheelTurn, CHANGE);    //Left wheel interrupt
  attachInterrupt(digitalPinToInterrupt(3), RwheelTurn, CHANGE);    //Right wheel interrupt
  
}

//Main code, responsible for writing controller
void loop() {

  posD = (double)input / (3.14 * wheelRadius);  
  
  //stores current time to determine when to stop printing data and to determine if main exceeds sampling time
  timeNow = millis();
  Lerror = posD - LposA;                //units: radians
  Rerror = posD - RposA; 
  
  if(Lerror<0){
    digitalWrite(LvoltageDirection, LOW);
  }
  else{
    digitalWrite(LvoltageDirection, HIGH);
  }

  if(Rerror<0){
    digitalWrite(RvoltageDirection, LOW);
  }
  else{
    digitalWrite(RvoltageDirection, HIGH);
  }
  
  if(abs(Lerror)<0.05){
    LdutyCycle = max(0,min((LKp+LKi*0.001)*abs(Lerror)/5.0*255.0,255));
  }
  else{
    LdutyCycle = max(0,min((LKp)*abs(Lerror)/5.0*255.0,255));
  }
  
  if(abs(Rerror)<0.05){
    RdutyCycle = max(0,min((RKp+RKi*0.001)*abs(Rerror)/5.0*255.0,255));
  }
  else{
    RdutyCycle = max(0,min((RKp)*abs(Rerror)/5.0*255.0,255));
  }
  

  
  //write voltageCommand 
  analogWrite(LvoltageCommand,(int)LdutyCycle);
  analogWrite(RvoltageCommand,(int)RdutyCycle);

  //calculate angular velocity (radians/sec)
  LposA = (1.0*Lcount)/countsPerRevolution*2*3.14;
  RposA = (1.0*Rcount)/countsPerRevolution*2*3.14;
  
  
  //only print data between 1 and 2 seconds
  //if ((timeNow > 1000) && (timeNow < 2000)) {
    Serial.print(millis() / (double)1000);
    Serial.print("\t");
    Serial.print(LdutyCycle);
    Serial.print("\t");
    Serial.print(RdutyCycle);
    Serial.print("\t");
    Serial.print(LposA);
    Serial.print("\t");
    Serial.print(RposA);
    Serial.print("\t");
    Serial.print(Lerror);
    Serial.print("\t");
    Serial.print(Rerror);
    Serial.println();
  //}

  //check to see if main running exceeded the sampling time
  if ((millis() - timeNow) > samplingTime) {
    Serial.println("Error: main exceeds sampling time");
  }

  //wait until sampling time to re run loop
  while ((millis() - timeNow) < samplingTime);
}
