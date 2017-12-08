void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


//Function to controll the DC Motor
//ONLY TEST CODE in function. Change accordingly when assembling
void motorCtrl(){
  
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
  
}

//Function to controll the Ultrasonic sensor
//ONLY TEST CODE in function. Change accordingly when assembling
void ultraSonCtrl(){

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

}

