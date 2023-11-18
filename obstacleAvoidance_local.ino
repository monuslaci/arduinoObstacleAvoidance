#include <Servo.h>

//define servo variables
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0; 

//define ultrasonic sensor variables
long a = 0;

//define motor driver variables
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board

//define global variables
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousDistance = 0;
unsigned long currentDistance = 0;
int distance = 13;
int maxDistance = 790;
int speed = 100;
const long interval = 3000;  // Interval in milliseconds (3 seconds)
int backCount = 0;
bool isStuck = false;

void setup() {
// put your setup code here, to run once:
//setup motor driver
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM) 
  
//setup servo
  myservo.attach(A2);  // attaches the servo on pin 9 to the servo object

//setup ultrasonic
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
  delay(1000);
}

void loop() 
{
    currentMillis = millis(); //the millis() function in Arduino returns the number of milliseconds since the Arduino began running the current program. It is often used for timing in non-blocking code to perform tasks at specific intervals without using delay(), which would block the program. It overflows approximately every 50 days,
    currentDistance = checkDistanceForDegree(90);
  
    goForward();
  //   if(checkDistanceForDegree(90)  < distance || checkDistanceForDegree(90)  > maxDistance)
  //   {
         assessSituation();
  //   }


  //Check distance every 3 seconds
  if (currentMillis - previousMillis >= interval) 
  {
    // Save the current time
    previousMillis = currentMillis;

    // Code to execute every 5 seconds
    Serial.print("Current distance: ");
        Serial.print(currentDistance);
    Serial.print("Previous distance: ");
     Serial.print(+previousDistance);

    if (currentDistance == previousDistance )
    {  
       Serial.print("Robot is stuck");
        
      isStuck = true;
      assessSituation();
    }
    // Save the current distance
    previousDistance = currentDistance;
  }
}


//complex motor controlling functions
// void advance()
// {

// }

void assessSituation()
{
    long rightDist = checkDistanceForDegree(0);
    delay(200); 
        long forwDist = checkDistanceForDegree(90);
    delay(200); 
    long leftDist = checkDistanceForDegree(180);
     delay(200); 

        Serial.print("Right distance: ");
        Serial.print(rightDist);
Serial.print("\n\n");

        Serial.print("Left distance: ");
        Serial.print(leftDist);
        Serial.print("\n\n");

    // if (backCount >= 3)
    // {
    //   backCount = 0;
    //   if(checkDistanceForDegree(0) > distance )
    //     turnRight();
    //   if(checkDistanceForDegree(180) > distance )
    //     turnLeft();
    //     break;
    // }
    if(backCount == 0 && isStuck == false && forwDist > distance && forwDist < maxDistance )
    {
      goForward();
      return;
    }

    if(rightDist > leftDist && rightDist > distance && rightDist < maxDistance)
    {
      Serial.print("turning right");
        turnRight();
        backCount = 0;
        isStuck = false;
        return;
        //advance();
    }
    if( leftDist > rightDist && leftDist > distance && leftDist < maxDistance)
    {
      Serial.print("turning left");
        turnLeft();
        backCount = 0;
        isStuck = false;
        return;
        //advance();
    }
    if ( ((leftDist < distance && leftDist < distance) || (leftDist > maxDistance && leftDist > maxDistance) || isStuck)  && backCount < 3)
    {
      Serial.print("going back");
       delay(100); 
        back();
        Set_Speed(speed);
        delay(1000);
        stopp();
        Set_Speed(0);
        delay(1000);
        
        backCount++;  
        isStuck = false;
        assessSituation();
    }   
}

void goForward()
{
  myservo.write(90); //set the servo to look in front
  delay(100); 
  long startTime = millis();

    Serial.print("goForward distance: ");
     Serial.print(checkDistanceForDegree(90));
      Serial.print("\n\n");
 delay(100); 

  if (backCount == 0)
  {
    while (checkDistanceForDegree(90) > distance && checkDistanceForDegree(90) < maxDistance && millis() <= startTime + 2000) //check if we are not too close to the wall, and runs for 1 sec
    {
      Serial.print("goForward distance in while: ");
      Serial.print(checkDistanceForDegree(90));
      Serial.print("\n\n");

      forward();
      Set_Speed(speed);
      delay(20);
      
    }
    stopp();
    Set_Speed(0);
    delay(1000);

  }
}

//basic motor controlling functions
bool checkSide(int degree)
{
  myservo.write(degree); //turn the servo 
  delay(200); 
  return checkdistance() > distance;
}

long checkDistanceForDegree(int degree)
{
  myservo.write(degree); //turn the servo 
  delay(200); 
  return checkdistance();
}

void turnRight()
{
    turnR(); 
    Set_Speed(200);
    delay(430);
    Set_Speed(0);
    delay(1000);
}

void turnLeft()
{
    turnL(); 
    Set_Speed(200);
    delay(450);
    Set_Speed(0);
    delay(1000);
}

void turnBack()
{
    turnR(); 
    Set_Speed(speed);
    turnR(); 
    Set_Speed(speed);
    delay(1000);
}
//motor driver related functions
void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(Lpwm_pin,pwm);
  analogWrite(Rpwm_pin,pwm);
}

void back()    //  going forward
    {
    digitalWrite(pinRB,LOW);  // making motor move towards right rear
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,LOW);  // making motor move towards left rear
    digitalWrite(pinLF,HIGH); 
   
    }
void turnR()        //turning right(dual wheel)
    {
    digitalWrite(pinRB,LOW);  //making motor move towards right rear
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);  //making motor move towards left front

    }
void turnL()         //turning left(dual wheel)
    {
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW );   //making motor move towards right front
    digitalWrite(pinLB,LOW);   //making motor move towards left rear
    digitalWrite(pinLF,HIGH);
    
    }    
void stopp()        //stop
    {
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,HIGH);
    
    }
void forward()         //back up
    {
    digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);  //making motor move towards left rear
    digitalWrite(pinLF,LOW);     
    }
    
//ultrasonic sensor related functions
float checkdistance() 
{
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);

   Serial.print(distance);
   Serial.println(" cm");

  return distance;
}