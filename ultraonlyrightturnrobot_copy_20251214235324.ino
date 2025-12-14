//includes the servo library in the program
#include <Servo.h>

//defining constants for pin numbers and the scan loop counter
#define rightSpeedPin 3        //pwm output for right motors for speed
#define rightDirectionPin1 12  //pin for forward direction
#define rightDirectionPin2 11  //pin for backward direction
#define leftSpeedPin 6         //pwm output for right motors for speed
#define leftDirectionPin1 7    //pin for forward direction
#define leftDirectionPin2 8    //pin for backward direction
#define LPT 1
#define servoPin 9         //pin for signal to servo
#define echoPin 2          //pin for signal from ultrasonic sensor
#define triggerPin 10      //pin for signal to ultrasonic to send a sound wave out
#define fastSpeed 200      //value for fast motor speed
#define speed 120          //value for slower motor speed was 100
#define turnSpeed 180      //value for motor speed when turning
#define highBackSpeed 200  //value for high motor speed
#define lowBackSpeed 70    //value for low motor speed
#define lineSensor0 A0
#define lineSensor1 A1
#define lineSensor2 A2
#define lineSensor3 A3
#define lineSensor4 A4

//defining variables for different positions in front of the robot as well as the
//parameters for the detection of objects
int centerScan;
int distance;
const int turnTime = 90;  //in miliseconds
int thereis;

//
boolean flag = false;

//creating an object of the servo class called head
Servo head;

//functions for controlling movement of the robot
//forward() function - moves the  robot forward
void forward(void) {
  digitalWrite(rightDirectionPin1, HIGH);
  digitalWrite(rightDirectionPin2, LOW);
  digitalWrite(leftDirectionPin1, HIGH);
  digitalWrite(leftDirectionPin2, LOW);
}
//left() function - turns the robot left
void left() {
  digitalWrite(rightDirectionPin1, HIGH);
  digitalWrite(rightDirectionPin2, LOW);
  digitalWrite(leftDirectionPin1, LOW);
  digitalWrite(leftDirectionPin2, HIGH);
}
//right() function - turns the robot right
void right() {
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2, HIGH);
  digitalWrite(leftDirectionPin1, HIGH);
  digitalWrite(leftDirectionPin2, LOW);
}
//backward() function - moves the robot backward
void backward() {
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2, HIGH);
  digitalWrite(leftDirectionPin1, LOW);
  digitalWrite(leftDirectionPin2, HIGH);
}
//stop() function - stops the robot
void stop() {
  digitalWrite(rightDirectionPin1, LOW);
  digitalWrite(rightDirectionPin2, LOW);
  digitalWrite(leftDirectionPin1, LOW);
  digitalWrite(leftDirectionPin2, LOW);
  setSpeed(0, 0);
}
//setSpeed() function - sets the speed of the motors on left and right sides of the robot seperately
void setSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(leftSpeedPin, leftSpeed);
  analogWrite(rightSpeedPin, rightSpeed);
}

//watch() function - returns the calculated distance from the ultrasonic distance sensor
long watch() {
  long echoDistance;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(5);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(triggerPin, LOW);
  echoDistance = pulseIn(echoPin, HIGH);
  echoDistance = echoDistance * 0.01657;  //object distance in cm
  return echoDistance;
}

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 10;      //distance limit for obstacles in front
const int ddistancelimit = 14;     //distance limit for obstacles diagonal ( approx 11.313   but add some tolerance)
const int sidedistancelimit = 12;  //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int numcycles = 0;

String watchsurrounding() {
  /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */

  int obstacle_status = B100000;
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    stop();

    obstacle_status = obstacle_status | B100;
  }
  head.write(135);
  delay(100);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < ddistancelimit) {
    stop();

    obstacle_status = obstacle_status | B1000;
  }
  head.write(180);
  delay(300);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit) {
    stop();

    obstacle_status = obstacle_status | B10000;
  }

  head.write(90);  //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if (centerscanval < distancelimit) {
    stop();

    obstacle_status = obstacle_status | B100;
  }
  head.write(45);
  delay(100);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < ddistancelimit) {
    stop();

    obstacle_status = obstacle_status | B10;
  }
  head.write(0);
  delay(100);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit) {
    stop();

    obstacle_status = obstacle_status | 1;
  }
  head.write(90);  //Finish looking around (look forward again)
  delay(300);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6);

  return obstacle_str;  //return 5-character string standing for 5 direction obstacle status
}

//wallFollow() function - takes a wide turn around an object until reaching a line
void wallFollow() {
  //String obstacle_sign = watchsurrounding();
  head.write(180); //turn left assuming 180 is left
  stop();
  setSpeed(speed, speed);
  backward();
  delay(200);
  right();
  delay(800);
  forward();
  delay(200);
  //++numcycles;
  //if (numcycles >= LPT) {  //Watch if something is around every LPT loops while moving forward
  //if using numcycles add numcycles = 0; into different if statements
    stop();
    String obstacle_sign = watchsurrounding();  // 5 digits of obstacle_sign binary value means the 5 direction obstacle status //assumes 180 means left and 0 means right
    if (obstacle_sign == "00000") {//no obstacle
      left();//start going left in order to get back to the line
      delay(500);
      forward();
      delay(200);
    }else if (obstacle_sign == "10000") {
      forward();
      delay(200);
    } else if (obstacle_sign == "11000" || obstacle_sign == "01000"){
      right();
      delay(200); //(a little more right)
      forward();
      delay(200);
    }
    else if (obstacle_sign == "11100"|| obstacle_sign == "01100"|| obstacle_sign == "10100"){
      right();
      delay(800); // a lot right
      forward();
      delay(200);
    }

  //  else if  {//(obstacle_sign == "10101"|| obstacle_sign == "01001"|| obstacle_sign == "10111" || obstacle_sign == "10001" 
   // || obstacle_sign == "10010" || obstacle_sign == "11001" || obstacle_sign == "10011" ){ // local minimum? robot won't fit likely
      //complete reverse and become a left turning robot
  //  }
    else {
      backward();
      delay(200);
      leff();
      delay(1600);
      forward();
      delay(400); //it has no become left turning add code for left turning opperation, give up if it hits the same issue
    }
  }
//}

//creates an empty character array consisting of 5 items
char sensor[5];

//getSensorValues() function - obtains the digital output from the sensors and
//then puts them into a string that is then returned where 1 stands for black and
//0 stands for white
String getSensorValues() {
  int sensorValue = 32;
  sensor[0] = !digitalRead(lineSensor0);
  sensor[1] = !digitalRead(lineSensor1);
  sensor[2] = !digitalRead(lineSensor2);
  sensor[3] = !digitalRead(lineSensor3);
  sensor[4] = !digitalRead(lineSensor4);
  sensorValue += sensor[0] * 16 + sensor[1] * 8 + sensor[2] * 4 + sensor[3] * 2 + sensor[4];
  String senseStr = String(sensorValue, BIN);
  senseStr = senseStr.substring(1, 6);
  return senseStr;
}

//lineTracking() function - moves the robot to remain on the line based on the
//output from the getSensorValues() function
void lineTracking() {
  String sensorVal = getSensorValues();
  Serial.println(sensorVal);
  if (sensorVal == "10000") {
    //turn left as black line is on the left of the robot
    left();
    setSpeed(fastSpeed, fastSpeed);
    delay(turnTime);
  }
  if (sensorVal == "10100" || sensorVal == "01000" || sensorVal == "11100" || sensorVal == "10010" || sensorVal == "11010" || sensorVal == "11000") {
    forward();
    setSpeed(0, fastSpeed);  //slight left
    delay(turnTime);
  }
  if (sensorVal == "00001") {
    //turn right as the black line is on the right of the car
    right();
    setSpeed(fastSpeed, fastSpeed);
    delay(turnTime);
  }
  if (sensorVal == "00011" || sensorVal == "00010" || sensorVal == "00101" || sensorVal == "00111" || sensorVal == "01101" || sensorVal == "01111" || sensorVal == "01011" || sensorVal == "01001") {
    forward();
    setSpeed(fastSpeed, 0);  //slight right
    delay(turnTime);
  }
  if (sensorVal == "11111" || sensorVal == "00000" || sensorVal == "01110") {
    //turn right as the black line is on the right of the car
    right();
    setSpeed(fastSpeed, fastSpeed);
    delay(turnTime);
  }
  if (sensorVal == "00100" || sensorVal == "01100" || sensorVal == "00110") {
    forward();
    setSpeed(speed, speed);
    delay(turnTime);
  }
  stop();
  setSpeed(0, 0);
}

//setup() function - assigns the mode for each of the pins and starts serial communications
void setup() {
  pinMode(rightDirectionPin1, OUTPUT);
  pinMode(rightDirectionPin2, OUTPUT);
  pinMode(leftSpeedPin, OUTPUT);
  pinMode(leftDirectionPin1, OUTPUT);
  pinMode(leftDirectionPin2, OUTPUT);
  pinMode(rightSpeedPin, OUTPUT);
  stop();  //stop move
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  head.attach(servoPin);
  head.write(180);
  delay(2000);
  Serial.begin(9600);
}
//loop() function - continuously runs goalSequence() function until reaching the goal
bool goalReached = false;
bool wallHit = false;
bool lineHit = false;
String lastHit = "";
void loop() {
  if (!goalReached) {
    goalSequence();
  } else {
    stop();
    setSpeed(0, 0);
  }
}
//
void goalSequence() {
  head.write(90);
  if (!wallHit) {
    String sensorVal = getSensorValues();
    if ((sensorVal == "11111" ) && (lastHit == "line")) { //would like to add sensorVal == "00000" to imply that if it goes from line following
    // to no line it has reached it's goal
      stop();
      setSpeed(0, 0);  //stopping the car as the goal has been reached
      goalReached = true;
    }
    //head.write(90);
    else if (watch() <= 10) {
      head.write(90);
      stop();
      setSpeed(0, 0);
      setSpeed(fastSpeed, speed);
      right();
      delay(turnTime * 3 / 4);
      stop();
      setSpeed(0, 0);
      lastHit = "wall";
      wallHit = true;
      lineHit = false;
    } else {
      //head.write(90);
      lineTracking();
    }
  } else if (!lineHit) {
    String sensorVal = getSensorValues();
    if (sensorVal.indexOf("1") >= 0) { //checking for any sign of a line after leaving the line- could be falsly triggered resulting 
    //in early exit of wall following messing up navigation
      head.write(90);
      stop();
      setSpeed(0, 0);
      right();
      setSpeed(fastSpeed, speed);
      delay(turnTime);
      stop();
      lastHit = "line";
      wallHit = false;
      lineHit = true;
    } else {
      head.write(180);
      wallFollow();
   
    }
  }
}
