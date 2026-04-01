#include <Servo.h>
#include <algorithm>
Servo leftWheel;
Servo rightWheel;

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
const int threshold = 1600;
const int stopPulse = 147;
const int delta = 7;
const int offset = -1;
int value = 0;
int mv_value = 0;

//PID
float kp = 2;
float ki = 0.01;
float kd = 1;
float integral;
int last_error = 0;
int last_time = 0;
const int white_l = 750;
const int black_l = 2850;

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
  //Serial.begin(9600);
  //digitalWrite(YLW, HIGH);
  //while(!Serial);  // wait for serial monitor to open (or reopen)
  //digitalWrite(YLW, LOW);
  //Serial.println(" "); /// line feed
  //Serial.println("Program ready.");
}

// Main Routine
void loop() {
  //read the sensor value
  lvalue = analogRead(LSENSOR);
  rvalue = analogRead(RSENSOR);

  //map the values into millivolts (assuming 3000 mV reference voltage)
  lvalue = map(lvalue, 0, 1023, 0, 3000);
  rvalue = map(rvalue, 0, 1023, 0, 3000);

  float left_correct = map(update_pid(lvalue, rvalue), -300, 300, -(delta), delta);
  runMotors(delta - offset + left_correct, delta);

  while (distMeasure() > distTarget(25)) {
    runMotors(0, 0);
    delay(1);
  }
  if (lvalue > threshold && rvalue > threshold) {
    runMotors(0, 0);
    allLEDsON();
    delay(200);
    allLEDsOFF();
  }
  delay(1);
}

//********** Functions (subroutines) ******************

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

//normalize line following outputs
float normalize(int raw) {
  float clamped = std::max( white_l, std::min(raw, black_l));
  return (clamped - white_l) / (black_l - white_l);
}

//calc err (thats short for calculate error chat)
float errorCalc(int left_raw, int right_raw) {
  float left = normalize(left_raw);
  float right = normalize(right_raw);

  //if -, right sees more, turn left
  return (left - right) * 100;
}

float update_pid(int left_raw, int right_raw) {
  float error = errorCalc(left_raw, right_raw);
  //P is pretty simple, it corrects current error
  float p = kp * error;

  //I is the integral, which corrects for consistent drift
  integral += error * ((millis() - last_time) / 1000);
  integral = max(min(integral, 100), -100); // apparently prevents windup?
  float i = ki * integral;

  //D is the derivative, which dampens oscillations and somehow predicts future error/curves?
  //I don't really get this one but if it works it works?
  //float d = kd * (error - last_error)/((millis() - last_time)/1000.0);
  //Serial.print("D: ");
  //Serial.println(d);

  last_error = error;
  last_time = millis();
  return (p + i);
}

// run robot wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL) * 10;  //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR) * 10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}

//Toggle an LED on/off
void toggleLED(int colour) {
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250);
}
void allLEDsON(){
  digitalWrite(GRN, HIGH);
  digitalWrite(YLW, HIGH);
  digitalWrite(RED, HIGH);
}
void allLEDsOFF() {
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
}

float distMeasure() {
  int sum = 0;
  for (int i = 0; i < 20; i++) {
    value = analogRead(SHARP);
    mv_value = map(value, 0, 1023, 0, 3300); //convert AtoD count to millivolts
    sum += mv_value;
  }
  return sum / 20;
}

float distTarget(int dist) {
  return (-0.0171 * dist * dist * dist + 2.9443 * dist * dist - 164.14 * dist + 3555.3);
}
