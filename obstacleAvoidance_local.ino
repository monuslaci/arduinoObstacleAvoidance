#include <Servo.h>

//define servo variables
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int pos = 0; 

//define ultrasonic sensor variables
long a = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long previousDistance = 0;
const long interval = 5000;  // Interval in milliseconds (5 seconds)

//define motor driver variables
int speed = 120;
int distance = 15;
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board

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
  previousDistance = checkdistance();
  advance();
  // Check distance every 5 seconds
  if (currentMillis - previousMillis >= interval)
  {
    // Save the current time
    previousMillis = currentMillis;

    // Code to execute every 5 seconds
    float currentDistance = checkdistance();
    Serial.print("Current distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    if (currentDistance == previousDistance && (currentDistance < 3 || currentDistance > 390))
    {
    previousDistance = currentDistance;
    back();
    Set_Speed(speed);
    delay(2000);
    turnLeft();
    advance();
    }

  }
}

void advance()
{
  goForward();

  if (checkSide(180))
  {
    turnRight();
    advance();
  } 
  else if (checkSide(0)) //if right is free
  {
    turnLeft();
    advance();
  } 
  else if (checkSide(180))
  {
    turnLeft();
    advance();
  } 
  else
  {
    turnBack();
    advance();
  }
  
  
}

int assessSituation()
{
  var rightDist = checkSide(0);
  var leftDist = checkSide(0);
  if (rightDist < 15 && leftDist < 15 || rightDist > 390 && leftDist > 390)
  {
    back();
    Set_Speed(speed);
    delay(1000);
    assessSituation();
  }

}

void goForward()
{
  myservo.write(90); //set the servo to look in front
  delay(1000); 
  while (checkdistance() > distance)
  {
    forward();
    Set_Speed(speed);
    delay(500);
  }
  stopp();
  Set_Speed(0);
  delay(1000);
}

bool checkSide(int degree)
{
  myservo.write(degree); //and turn the servo in 180 degrees
  delay(200); 
  return checkdistance() > distance;
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