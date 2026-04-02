/********************************************************
  M103Lab4LineSensorCheck (v1.4)

  Original by H. Fernando, 25/04/2021
  
  This code is to help calibrate the line sensor for your setup.
  Place the robot on the a white surface or black tape and run 
  the program. Obtain the analog values for black tape and white 
  surface using the readings provided in the serial monitor.
*********************************************************/

 #include <Servo.h>
 #include <algorithm>
 Servo leftWheel;
 Servo rightWheel;
 Servo myServoA;  // Makes a servo object to control servo A
 Servo myServoB;
 
// Pin Assignments
const int RED = 10;           //red LED Pin
const int GRN = 9;           //green LED Pin
const int YLW = 5;           //yellow LED Pin
const int BUTTON = 7;        //pushbutton Pin
int MOTOR_R = 3;
int MOTOR_L = 4;
int SHARP = A3;

const int LSENSOR = A1; // Left Sensor on Analog Pin 1
const int RSENSOR = A2; // Right Sensor on Analog Pin 2

//global variables
int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
const int threshold = 1800;
const int stopPulse = 147;
const int delta = 9;
const float offset = 1.2;
int value = 0;
int mv_value = 0;
int turns = 0;
int tim;

//PID
float kp = 1.8;
float ki = 0.02;
float kd = 0.7;
float integral;
int last_error = 0;
int last_time = 0;
const int white_l = 750;
const int black_l = 2850;

//Bucket
const int servoPinA = 11;     // Bucket servomotor #1 pin
const int servoPinB = 12;     // Bucket servomotor #2 pin
const int myAngleA1 = 160;    // initial angle, bucket lifts off ground if too high
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
const int myAngleA2 = 90;     // highest angle (lift), puts almost straight, set to 110
const int myAngleB1 = 80;
const int myAngleB2 = 150;

bool seen = false;

//********** Functions (subroutines) ******************

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

void allLEDsoff(){
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
}

//normalize line following outputs
float normalize(int raw){
  float clamped = std::max( white_l, std::min(raw, black_l));
  return (clamped - white_l)/(black_l - white_l);
}

//calc err (thats short for calculate error chat)
float errorCalc(int left_raw, int right_raw){
  float left = normalize(left_raw);
  float right = normalize(right_raw);

  //if -, right sees more, turn left
  return (left - right)*100;
}

float update_pid(int left_raw, int right_raw){
  float error = errorCalc(left_raw, right_raw);
  //P is pretty simple, it corrects current error
  float p = kp * error;

  //I is the integral, which corrects for consistent drift
  integral += error * ((millis() - last_time)/1000);
  integral = max(min(integral, 100), -100); // apparently prevents windup?
  float i = ki * integral;

  //D is the derivative, which dampens oscillations and somehow predicts future error/curves?
  //I don't really get this one but if it works it works?
  float d = kd * (error - last_error)/((millis() - last_time)/1000.0);  
  Serial.print("D: ");
  Serial.println(d);
  
  last_error = error;
  last_time = millis();
  return (p + i);
}

// run robot wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL)*10;    //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR)*10; 
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR); 
}

//Toggle an LED on/off
void toggleLED(int colour){
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250); 
}

float distMeasure(){
  int sum = 0;
  for (int i = 0; i<20; i++){
    value = analogRead(SHARP);
    mv_value = map(value,0,1023,0,3300); //convert AtoD count to millivolts
    sum +=mv_value;
  }
 return sum/20;
}

float distTarget(int dist){
  return (-0.0171*dist*dist*dist + 2.9443*dist*dist - 164.14*dist + 3555.3);
}

void lineDetect(){
  //read the sensor value
  lvalue = analogRead(LSENSOR);
  rvalue = analogRead(RSENSOR);
      
  //map the values into millivolts (assuming 3000 mV reference voltage)
  lvalue = map(lvalue,0,1023,0,3000);
  rvalue = map(rvalue,0,1023,0,3000); 
}

void lineRecenter(int length){
  for (int i = 0; i<length; i++){
    lineDetect();
    float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
    runMotors(delta/2+left_correct,delta/2-left_correct);
    delay(1);
  }
}

void drop(){
  for (int posB = myAngleB1; posB <= myAngleB2; posB++){
    myServoB.write(posB);
    delay(20);
  }
}
  void curl(){
  for (int posB = myAngleB2; posB >= myAngleB1; posB--){
    myServoB.write(posB);
    delay(20);
  }
}

void lift(){
  for (posA = myAngleA1; posA >= myAngleA2; posA--) { // Lift action
    myServoA.write(posA);
    delay(20);
  }
}

void down(){
  for (posA = myAngleA2; posA <= myAngleA1; posA++) {  // Drop action
    myServoA.write(posA);
    delay(20);
  }
}

void turn_180(){
  // stops at distance
  runMotors(0,0); 
  delay(100);
  // goes backwards for 500 ms
  runMotors(-(delta-offset),-(delta)); 
  delay(500);
  runMotors(0,0);
  delay(100);
  // left wheel reverse for 300 ms
  runMotors(-(delta),0);
  delay(2100); 
  runMotors(0,0);
  delay(100);
  // backwards for some period, removed so it dosen't fall off the table.
  //runMotors(-(delta-offset),-(delta)); 
  //delay(600); 
  //runMotors(0,0);
  //delay(100);
  //forward until it sees the line again
  runMotors(delta-offset,delta);
  while ((lvalue<threshold)||(rvalue<threshold)) {
    delay(1);
    lineDetect();
  }
  delay(50);
  runMotors(0,0);
  delay(400);
  //single turn
  runMotors(-delta,delta);
  delay(900);
  runMotors(0,0);
  delay(400);
}

void turn_180_weighted(){
  // stops at distance
  runMotors(0,0); 
  delay(100);
  // goes backwards for 500 ms
  runMotors(-(delta-offset),-(delta)); 
  delay(500);
  runMotors(0,0);
  delay(100);
  // left wheel reverse for 300 ms
  runMotors(-(delta),0);
  delay(2600); 
  runMotors(0,0);
  delay(100);
  // backwards for some period, removed so it dosen't fall off the table, then added back so it dosen't miss the line.
  runMotors(-delta,-delta-2); 
  delay(600); 
  runMotors(0,0);
  delay(100);
  //forward until it sees the line again
  runMotors(delta-offset,delta);
  while ((lvalue<threshold)||(rvalue<threshold)) {
    delay(1);
    lineDetect();
  }
  delay(50);
  runMotors(0,0);
  delay(400);
  //single turn
  runMotors(-delta,delta);
  delay(1200);
  runMotors(0,0);
  delay(400);
}

void doubleblack_turn(){
  if (lvalue>threshold&&rvalue>threshold&&turns<5){ // standard turn into straight-away
        if (turns == 4)turns++; 
          delay(100);
          runMotors(0,0);
          delay(400);
          runMotors(-delta,delta);
          //single turn
          delay(900);
          runMotors(0,0);
          delay(400);
          for(int i = 0; i<1000; i++){
            lineDetect();
        
            float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
            runMotors(delta/2+left_correct,delta/2);
            delay(1);
          }
        }
        if (turns == 5){ // stops after the fifth turn
          lineDetect();
        
          float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
          runMotors(delta-offset+left_correct,delta);
          
          
          if (millis()>tim+30000){
            runMotors(0,0);
            delay(1000000);
          }  
        }
}

// Set-up Routine
void setup() {
                
// Initialize led pins as outputs.
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);
  
// Initialize button pins as inputs
  pinMode(BUTTON, INPUT);
  pinMode(SHARP, INPUT);

// Initialize line following sensor pins as inputs
  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);

// Initialize motor control pins as servo pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

 
  
// Initialize serial and monitor
  Serial.begin(9600); 
  runMotors(0,0);    
  //digitalWrite(YLW, HIGH);
  //while(!Serial);  // wait for serial monitor to open (or reopen)
  //digitalWrite(YLW, LOW);
  //Serial.println(" "); /// line feed
  //Serial.println("Program ready.") 

  // Set-up servo motors
  myServoA.write(posA);         // Servo A starting position
  myServoA.attach(servoPinA);   // Attaches the servo to the servo object
  myServoB.write(0);
  myServoB.attach(servoPinB);
  do{
    delay(50);
    
  }while(digitalRead(BUTTON) == LOW);
  lift();
  curl();
}

// Main Routine
void loop() { 
      allLEDsoff();
      turnOnLED(RED);
      lineDetect();
      distMeasure();
      
      float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
      runMotors(delta-offset+left_correct,delta);

      doubleblack_turn();

      while (distMeasure() > distTarget(17) && (turns % 2 == 0) ){ // bucket pickup
        allLEDsoff();
        turnOnLED(GRN);
        turns++;
        turn_180();
        lineRecenter(3000);
        runMotors(0,0);
        seen = true;
        down(); // drops bucket down
        delay(100);
        while (millis()<tim+800&&seen){
          lineDetect();
          float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
          runMotors(-delta,-delta+offset-1); // rams into payload for pickup
        }
        runMotors(0,0);
        delay(100);
        runMotors(delta - offset, delta);
        delay(100);
        runMotors(0,0);
        lift();
        delay(100);
        lineRecenter(2000);
        for(int i = 0; i<2000; i++){ // slow exit
          lineDetect();
      
          float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
          runMotors(delta-offset/2+left_correct,delta/2);
          delay(1);
        }
         tim = millis();
      }

      while (distMeasure() > distTarget(17) && (turns % 2 == 1) ){ // bucket dropoff
        allLEDsoff();
        turnOnLED(YLW);
        turns++;
        runMotors(0,0); // stops at distance
        delay(100);
        turn_180_weighted();
        lineRecenter(3000);
        runMotors(0,0);
        delay(100);
       runMotors(-delta,-delta-3);
        delay(3500);
       runMotors(0,0);
        delay(100);
        runMotors(delta-offset,delta);
        delay(150);
        runMotors(0,0);
        // add slow backup

        drop();
        
        for(int i = 0; i<2000; i++){
          lineDetect();
      
          float left_correct = map(update_pid(lvalue,rvalue), -300, 300, -(delta), delta);
          runMotors(delta-offset/2+left_correct,delta/2);
          delay(1);
        }
         tim = millis();
      }
}

    
      