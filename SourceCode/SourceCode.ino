#include <Servo.h>
#define TBD 0 

//Connections
//Ultrasonic sensor
int usTriggerPin = TBD;
int usReadPin = TBD;

//Servo
int servoPwmPin = 9;

//DC motor (through H-bridge)
int dcPwmPin = TBD;

//Line sensor
int leftLinePin = A5;
int midLinePin = A4;
int rightLinePin = A3;



Servo servo;

int* lineSensor = NULL;

enum linePos
{
 unknown,
 left,
 middle,
 right
};


void setup() { 
  lineSensor = malloc(3*sizeof(int));
  lineSensor[0] = 0;
  lineSensor[1] = 0;
  lineSensor[2] = 0;
  
  Serial.begin(9600);
  setupDcMotor();
  setupServo();
  
  setupUsSensor();
  setupLineSensor();

}

void loop() {
  /*Serial.println(analogRead(leftLinePin));
  Serial.println(analogRead(midLinePin));
  Serial.println(analogRead(rightLinePin));
  Serial.println("");*/
  readLineSensor(lineSensor,lineSensor);
  /*Serial.println(lineSensor[0]);
  Serial.println(lineSensor[1]);
  Serial.println(lineSensor[2]);*/
  Serial.println(linePosition());
  Serial.println();
  delay(500);
}









//SETUP FUNCTIONS
void setupDcMotor() {
      //pinMode(buttonPin, INPUT); 
//      pinMode(inaPin, OUTPUT);
//      pinMode(inbPin, OUTPUT);
      pinMode(dcPwmPin, OUTPUT);
//      pinMode(diagaPin, INPUT);
//      pinMode(diagbPin, INPUT);
//      pinMode(trimPin, INPUT);
      }

void setupUsSensor(){
pinMode(usTriggerPin, OUTPUT);
pinMode(usReadPin, INPUT);
}

void setupLineSensor()
{
   readLineSensor(lineSensor,lineSensor);
  delay(0.1);
 readLineSensor(lineSensor,lineSensor);
  delay(0.1);
  readLineSensor(lineSensor,lineSensor);
}
void setupServo()
{
  servo.attach(servoPwmPin);
  servo.write(90);
}



//CONTROL FUNCTIONS
int readUsSensor(){
      long duration, distance;
      digitalWrite(usTriggerPin, LOW);  // Added this line
      delayMicroseconds(2); // Added this line
      digitalWrite(usTriggerPin, HIGH);
      delayMicroseconds(10); // Added this line
      digitalWrite(usTriggerPin, LOW);
      duration = pulseIn(usReadPin, HIGH);
      distance = (duration/2) / 29.1;
      return distance;
}


void setServoPos(int currentAngle,int newAngle)
{
  float nbTurnsPerSPositive = 0.272;//how many turn per second the servo does at rotationSpeed
  float nbTurnsPerSNegative = 0.37;
  float deltaAngle = currentAngle - newAngle;
  
  if(deltaAngle > 0)
  {
    servo.write(80);
    delay(float(1000*abs(deltaAngle))/360/nbTurnsPerSPositive);
  }
  else if (deltaAngle < 0)
  {
    servo.write(100);
    delay(float(1000*abs(deltaAngle))/360/nbTurnsPerSNegative);
    Serial.println((float(1000*abs(deltaAngle))/360/nbTurnsPerSNegative));
  }
    
  servo.write(90);//Stop
}


void setDcSpeed(int dutyCycle)
{
if(dutyCycle > 100)
  dutyCycle = 100;
if(dutyCycle < 0)
  dutyCycle = 0;
analogWrite(dcPwmPin, map(dutyCycle, 0, 100, 0, 255));
}  

void readLineSensor(int* newStates, int* previousStates)
{
  
  /*sensorStates[0] = analogRead(leftLinePin);
  sensorStates[1] = analogRead(middleLinePin);
  sensorStates[2] = analogRead(rightLinePin);*/

  //Low pass filtering the measurements
  int lowPassAlpha = 0.5;
  newStates[0] = analogRead(leftLinePin);// previousStates[0] + lowPassAlpha * (newStates[0] - analogRead(leftLinePin));
  newStates[1] = analogRead(midLinePin);//previousStates[1] + lowPassAlpha * (newStates[1] - analogRead(midLinePin));
  newStates[2] = analogRead(rightLinePin);//previousStates[2] + lowPassAlpha * (newStates[2] - analogRead(rightLinePin));
  //If measurements are not stable enough, add another block like this with a small delay
  
  //return sensorStates;
}


linePos linePosition()
{
    readLineSensor(lineSensor,lineSensor);

    if((lineSensor[0] > 1.1*lineSensor[1]) && (lineSensor[2] > 1.1*lineSensor[1]))
     {
      return middle;//Line is on the middle sensor
     }
    else if((lineSensor[0] > 1.1*lineSensor[2]) && (lineSensor[1] > 1.1*lineSensor[2]))
     {
      return right; //line is on the right sensor
     }
    else if((lineSensor[1] > 1.1*lineSensor[0]) && (lineSensor[2] > 1.1*lineSensor[0]))
     {
      return left; //line is on the left sensor
     }
    else
    {
      return unknown;//Line lost
    }

}

















//Function to control the DC Motor
//ONLY TEST CODE in function. Change accordingly when assembling
//void motorCtrl(){
  
    //const int inaPin = 13;
    //const int inbPin = 9;
    //const int pwmPin = 11;
    //const int diagaPin = 10;
    //const int diagbPin = 12;
    //const int buttonPin = 2;
    //const int trimPin = A0;
    //
    //int i = 0;
    //
    //void setup() {
    //  pinMode(buttonPin, INPUT); 
    //  pinMode(inaPin, OUTPUT);
    //  pinMode(inbPin, OUTPUT);
    //  pinMode(pwmPin, OUTPUT);
    //  pinMode(diagaPin, INPUT);
    //  pinMode(diagbPin, INPUT);
    //  pinMode(trimPin, INPUT);
    // 
    //}
    //
    //void loop() {
    //  
    //  // CW
    //  digitalWrite(inaPin, HIGH);
    //  digitalWrite(inbPin, LOW);
    //  while(HIGH == HIGH){
    //    i = 1023;
    //    analogWrite(pwmPin, map(i, 0, 1023, 0, 255));
    //
    //  }
    //
    //  
    //}
  
//}

//Function to controll the Ultrasonic sensor
//ONLY TEST CODE in function. Change accordingly when assembling
//void ultraSonCtrl(){

    //#define trigPin 13
    //#define echoPin 12
    //#define led 11
    //#define led2 10
    //
    //void setup() {
    //  Serial.begin (9600);
    //  pinMode(trigPin, OUTPUT);
    //  pinMode(echoPin, INPUT);
    //  pinMode(led, OUTPUT);
    //  pinMode(led2, OUTPUT);
    //}
    //
    //void loop() {
    //  long duration, distance;
    //  digitalWrite(trigPin, LOW);  // Added this line
    //  delayMicroseconds(2); // Added this line
    //  digitalWrite(trigPin, HIGH);
    ////  delayMicroseconds(1000); - Removed this line
    //  delayMicroseconds(10); // Added this line
    //  digitalWrite(trigPin, LOW);
    //  duration = pulseIn(echoPin, HIGH);
    //  distance = (duration/2) / 29.1;
    //  if (distance < 4) {  // This is where the LED On/Off happens
    //    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
    //  digitalWrite(led2,LOW);
    //}
    //  else {
    //    digitalWrite(led,LOW);
    //    digitalWrite(led2,HIGH);
    //  }
    //  if (distance >= 200 || distance <= 0){
    //    Serial.println("Out of range");
    //  }
    //  else {
    //    Serial.print(distance);
    //    Serial.println(" cm");
    //  }
    //  delay(500);
    //}

//}


//SERVO control example
//#include <Servo.h>
//
//Servo myservo;  // create servo object to control a servo
//// twelve servo objects can be created on most boards
//
//int pos = 0;    // variable to store the servo position
//
//void setup() {
//  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
//}
//
//void loop() {
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//}
//

