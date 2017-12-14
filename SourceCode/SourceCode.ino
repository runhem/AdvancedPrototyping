  #include <Servo.h>
  
  //---Defining connections ---
  
  //Servo
  int servoPwmPin = 9;
  
  //Ultrasonic sensor
  int usTriggerPin = 10;
  int usReadPin = 12;
  
  
  //DC motor (through H-bridge)
  int dcPwmPin = 11; //Pin 6 on H-bridge
  int dcInAPin = 13; //Pin 4 on H-bridge
  int dcInBPin = 8; // Pin 5 on H-bridge
  
  //Line sensor
  int leftLinePin = A5;
  int midLinePin = A4;
  int rightLinePin = A3;
  
  //---END Defining connections ---
  
  Servo servo;
  
  int* lineSensor = NULL;
  int count = 0;
  
  enum linePos
  {
   unknown,
   left,
   middle,
   right
  };
  
  enum distanceToCollision
  {
      tooClose,
      farEnough,  //Unnecesary since we never use the last three
      far,
      veryFar
  };

  //---Set Up----
  void setup() { 
    lineSensor = malloc(3*sizeof(int));
    lineSensor[0] = 0;
    lineSensor[1] = 0;
    lineSensor[2] = 0;
    digitalWrite(dcInAPin, LOW);
    digitalWrite(dcInBPin, HIGH);
    
    for(int speed = 0 ; speed <= 150; speed +=1){
    analogWrite(dcPwmPin, speed); 
    delay(20);      
    }
    
    //analogWrite(dcPwmPin, 150);
    
    Serial.begin(9600);

    //Calling on setUp functions for components
    setupDcMotor();
    setupServo();
    setupUsSensor();
    setupLineSensor();

  }

  //---END OF Set Up----

  //---LOOP----

  void loop() 
  {
    //Check distance to obstacle
    int distanceToCollision = readUsSensor();

    //If distance to obstacle is too close, run obstacle avoidance protocol
    if (distanceToCollision == tooClose)
    { 
      Serial.println("Too close");
      digitalWrite(dcInAPin, HIGH);
      digitalWrite(dcInBPin, LOW);
      analogWrite(dcPwmPin, 70);
      setServoPos(0,30);
      delay(1000);
      setServoPos(30,0);

      do{
      delay(700);
      distanceToCollision = readUsSensor();
      }
      while(distanceToCollision == tooClose);
      digitalWrite(dcInAPin, LOW);
      digitalWrite(dcInBPin, HIGH);
      analogWrite(dcPwmPin, 150);
    }
    //If distance to obstacle is far enough, follow the line
    else if (distanceToCollision != tooClose){
      Serial.println("Far enough");
      followLineProtocol();
    }
    delay(500);
    
    /*Serial.println(analogRead(leftLinePin));
    Serial.println(analogRead(midLinePin));
    Serial.println(analogRead(rightLinePin));
    Serial.println("");*/
    //readLineSensor(lineSensor,lineSensor);
    /*Serial.println(lineSensor[0]);
    Serial.println(lineSensor[1]);
    Serial.println(lineSensor[2]);*/  
  
  }

//---END OF Loop----

//---SETUP FUNCTIONS---
  void setupDcMotor() 
  {
        //pinMode(buttonPin, INPUT); 
        pinMode(dcInAPin, OUTPUT);
        pinMode(dcInBPin, OUTPUT);
        pinMode(dcPwmPin, OUTPUT);
  }

  void setupUsSensor()
  {
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

//---END OF SET UP FUNCTIONS---


//---CONTROL FUNCTIONS---
  int readUsSensor()
  {
    long duration, distance;
    digitalWrite(usTriggerPin, LOW);  
    delayMicroseconds(2); 
    digitalWrite(usTriggerPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(usTriggerPin, LOW);
    duration = pulseIn(usReadPin, HIGH);
    distance = (duration/2) / 29.1;

    if (distance <15){
      return tooClose;
    }
    
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
    }
    
    servo.write(90);//Stop
  }


  void setDcSpeed(int dutyCycle)
  {
    if(dutyCycle > 100)
    {
      dutyCycle = 100;
    }
    if(dutyCycle < 0)
    {
      dutyCycle = 0;
    }

  analogWrite(dcPwmPin, map(dutyCycle, 0, 100, 0, 255));
  }  

  void readLineSensor(int* newStates, int* previousStates)
  {

  //Low pass filtering the measurements
  int lowPassAlpha = 0.5;
  newStates[0] = analogRead(leftLinePin);// previousStates[0] + lowPassAlpha * (newStates[0] - analogRead(leftLinePin));
  newStates[1] = analogRead(midLinePin);//previousStates[1] + lowPassAlpha * (newStates[1] - analogRead(midLinePin));
  newStates[2] = analogRead(rightLinePin);//previousStates[2] + lowPassAlpha * (newStates[2] - analogRead(rightLinePin));
  //If measurements are not stable enough, add another block like this with a small delay
  //return sensorStates;
  }


  void followLineProtocol(){
    int linePos = linePosition(); 
    
    //Line following if no obstacle
    if(linePos == left)
    {
      Serial.println(linePos);
      Serial.println(left);
      Serial.println("Line to the left");
      setServoPos(0,20);
      delay(1000);
      setServoPos(20,0);
      linePos = linePosition();
      
      while(linePos != middle)
        {
          delay(500);
          linePos = linePosition();
        }
          
      setServoPos(0,-20);//return in initial direction
      delay(1000);
      setServoPos(-20,0);
    }
    
    else if(linePos == right)
    {
      Serial.println(linePos);
      Serial.println(right);
      Serial.println("Line to the right");
      setServoPos(0,-20);
      delay(1000);
      setServoPos(-20,0);
      linePos = linePosition();
      
      while(linePos != middle)
      {
        delay(500);
        linePos = linePosition();
      }
      
      setServoPos(0,20);//return in initial direction
      delay(1000);
      setServoPos(20,0);
    }
    else if(linePos == unknown){
      if(count = 0){
        obstacleAvoidance();
        count = 1;
      }
      
      else {
       analogWrite(dcPwmPin, 0);
      }
    
    }
  }
  
  
  void obstacleAvoidance(){
    
    setServoPos(0,40);
    delay(3000);
    setServoPos(-40,0);
    delay(3000);
    }

    
    


  int linePosition()
  {
    readLineSensor(lineSensor,lineSensor);
    int leftMeasure = lineSensor[0];
    int midMeasure = lineSensor[1];
    int rightMeasure = lineSensor[2];

    
    if(isWhiter(leftMeasure,midMeasure) && isWhiter(rightMeasure,midMeasure))
     {
      Serial.println("middle");
      return middle;//Line is on the middle sensor
      
     }
    else if(isWhiter(leftMeasure,rightMeasure))
     {
      Serial.println("right");
      return right; //line is on the right sensor
      
     }
    else if(isWhiter(rightMeasure, leftMeasure))
     {
      Serial.println("left");
      return left; //line is on the left sensor
      
     }
    else
    {
      Serial.println("dont nooo");
      return unknown;//Line lost
      
    }
  }


  bool isWhiter(int a, int b)//if a is whiter than b returns true
  {
   if(a > 1.1*b)
    return true;
   else 
    return false;  
  }





