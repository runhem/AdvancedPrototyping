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
      farEnough,
      far,
      veryFar
  };

  //---Set Up----
  void setup() { 
    lineSensor = malloc(3*sizeof(int));
    lineSensor[0] = 0;
    lineSensor[1] = 0;
    lineSensor[2] = 0;
    digitalWrite(dcInAPin, HIGH);
    digitalWrite(dcInBPin, LOW);
    analogWrite(dcPwmPin, 150);
    
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
      Serial.println("too close");
      obstacleAvoidance();
    }
    //If distance to obstacle is far enough, follow the line
    else if (distanceToCollision == farEnough){
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
  //    pinMode(diagaPin, INPUT);
  //    pinMode(diagbPin, INPUT);
  //    pinMode(trimPin, INPUT);
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
    digitalWrite(usTriggerPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(usTriggerPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(usTriggerPin, LOW);
    duration = pulseIn(usReadPin, HIGH);
    distance = (duration/2) / 29.1;

    if (distance <10){
      return tooClose;
    }

    else if (distance > 10 && distance < 20){
        return farEnough;
    }

    else if (distance > 20 && distance < 40){
        return far;
    }

    else{
      return veryFar;
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


  void followLineProtocol(){
    Serial.println("In FollowLineProtocol");
    int linePos = linePosition(); 
    Serial.println(linePos); 
    
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
  }
  
  int count = 0; //Used to check if it's the first or second time we call obstacle avoidance (); Maybe have to move!
  
  void obstacleAvoidance(){
    Serial.println("Count = ");
    Serial.println(count);
    if(count = 0){
      //Make hard coded track
      count = 1;
      Serial.println("Count = ");
      Serial.println(count);
    }

    else{ //Second time we lose the line
      //STOP. Probably at the finish line
    }
    
    
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
      Serial.println("do nooo");
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
/*int linePosition()
{
    readLineSensor(lineSensor,lineSensor);

    if((lineSensor[0] > 1.1*lineSensor[1]) && (lineSensor[2] > 1.1*lineSensor[1]))
     {
      return middle;//Line is on the middle sensor
     }
    else if(((lineSensor[0] > 1.1*lineSensor[2]) && (lineSensor[1] > 1.1*lineSensor[2])) || ((lineSensor[0] > 1.1*lineSensor[1]) && !(lineSensor[1] > 1.1*lineSensor[2])))
     {
      return right; //line is on the right sensor
     }
    else if((lineSensor[1] > 1.1*lineSensor[0]) && (lineSensor[2] > 1.1*lineSensor[0]) || ((lineSensor[1] > 1.1*lineSensor[0]) && !(lineSensor[2] > 1.1*lineSensor[0])))
     {
      return left; //line is on the left sensor
     }
    else
    {
      return unknown;//Line lost
    }

}*/





