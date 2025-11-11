#include <Servo.h>
#include <IRremote.hpp>

// ====== Servo Setup ======
Servo myservo;

// ====== Pin setup ======
int Echo_Pin = A0; // Ultrasonic ECHO
int Trig_Pin = A1; // Ultrasonic TRIG
#define Lpwm_pin 5 // ENA (Left speed)
#define Rpwm_pin 6 // ENB (Right speed)
int pinLB = 2; // IN1
int pinLF = 4; // IN2
int pinRB = 7; // IN3
int pinRF = 8; // IN4
#define LED_PIN 12 // Power indicator LED

// ====== IR remote setup ======
#define IR_PIN 9 // IR receiver signal pin
#define ONOFF_BUTTON 0xAD52FF00 // Remote’s ON/OFF hex code
bool carOn = false; // Toggle state

// ====== Distance variables ======
volatile int Front_Distance, Left_Distance, Right_Distance,Wall_Distance;

// ====== Function declarations ======
long distance;
long checkdistance();
void Car_To_Wall_Position(); // Checks for obstacle, and if its not detected it'll follow the wall
void Obstacle_Avoiding();
void Gather_Distances(); // Gathers wall and front distance interchanging
void get_close(); // This overrides the wall following when it detects an obstacle 50cm away, it'll stop checking wall distance and make sure it wont run into the obstacle
void static_friction();  // This is to avoid the car getting stuck
void Ultrasonic_Front_obstacle_avoidance();  // initialize the car and where it is, if its close to an obstacle it'll avoid it first and vice versa
void Obstacle_Avoidance_Main();
void go_forward(unsigned char speed_val);
void go_backward(unsigned char speed_val);
void rotate_left(unsigned char speed_val);
void rotate_right(unsigned char speed_val);
void stopp();

// ===================================================
// ========== SETUP ==================================

void setup() {

myservo.attach(A2);
IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
Serial.begin(9600);


Front_Distance = 0;
Left_Distance = 0;
Right_Distance = 0;

myservo.write(90); // Center servo at 20°

pinMode(Echo_Pin, INPUT);
pinMode(Trig_Pin, OUTPUT);
pinMode(pinLB, OUTPUT);
pinMode(pinLF, OUTPUT);
pinMode(pinRB, OUTPUT);
pinMode(pinRF, OUTPUT);
pinMode(Lpwm_pin, OUTPUT);
pinMode(Rpwm_pin, OUTPUT);
pinMode(LED_PIN, OUTPUT);

digitalWrite(LED_PIN, LOW); // Ensure LED starts OFF

}

// ========== LOOP ===================================

void loop() {

// Check for IR remote input

if (IrReceiver.decode()) {

unsigned long value = IrReceiver.decodedIRData.decodedRawData;


if (value == ONOFF_BUTTON) {

carOn = !carOn; // Toggle ON/OFF
digitalWrite(LED_PIN, carOn ? HIGH : LOW); // LED follows car state

  if (carOn) go_forward(0); // optional, car starts stopped
  else stopp();

  }
IrReceiver.resume(); // Ready for next signal

}

// Run car logic only if ON
if (carOn) {
Obstacle_Avoidance_Main();
} 
else {
stopp();

}
}



// ===================================================

// ========== ULTRASONIC FUNCTIONS ===================

long checkdistance() {
  long duration;
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  duration = pulseIn(Echo_Pin, HIGH);
  distance = duration / 58;
  delay(50);
  return distance;
}

// Main loop
// Checks front and wall distances before moving and chosing to follow the wall or avoid obstacles
void Ultrasonic_Front_obstacle_avoidance() {
stopp();
Gather_Distances();

if (Front_Distance < Wall_Distance) {
  Obstacle_Avoiding();
}

else {
  Car_To_Wall_Position();
}
}


// Fixing the car position to the wall while still checking for front obstacles
void Car_To_Wall_Position() {

while (Front_Distance >= 20) {

Front_Distance = checkdistance();

Gather_Distances();

  if (Front_Distance <= 50) {  // checking for obstacle
    obstacle_detected()
  }

  else { 
     // if no obstacle is detected then it'll keep following the wall and correcting its position
    if (Wall_Distance >= 20) {
      Serial.println("Obstacle Not Detected");
      rotate_left(150);
      delay(500);
      go_forward(100);
      delay(500);
    }
    else if (Wall_Distance <= 20) {
      rotate_right(150);
      delay(200);
      go_forward(100);
      delay(500);
    }
  }
}
}



void Gather_Distances() {
Serial.println("Checking Distance: Gather_Distances()");
go_forward(80);
delay(300);
myservo.write(90);
delay(500);
Front_Distance = checkdistance();
Serial.println(Front_Distance);
if (Front_Distance > 250) {
  go_backward(100);
  delay(300);
  myservo.write(90);
  delay(300);
  Front_Distance = checkdistance();
  delay(300);
  stopp();
}
delay(300);
myservo.write(180);
delay(500);
Wall_Distance = checkdistance();
if (Wall_Distance > 250) {
  go_backward(100);
  delay(300);
  myservo.write(180);
  delay(300);
  Wall_Distance = checkdistance();
  delay(300);
  stopp();
}
Serial.println(Wall_Distance);
delay(300);
static_friction();
}



void obstacle_detected() {
  myservo.write(90);
  Serial.println("Obstacle Detected");
  stopp();
  Serial.println("Step1");
  get_close();
  stopp();
  Obstacle_Avoiding();
}


void get_close() {
Front_Distance = checkdistance();
  while (Front_Distance >= 20) {
  go_forward(80);
  Front_Distance = checkdistance();
  Serial.println("getting closer: get_close()");
}

}



void Obstacle_Avoiding() {
// Obstacle detected
stopp();
delay(500);

Front_Distance = checkdistance();
if (Front_Distance <= 5){
go_backward(150);
delay(500);
stopp(); 
delay(500);
}

// Look left
myservo.write(190);
delay(500);
Left_Distance = checkdistance();
delay(500);

// Look right
myservo.write(0);
delay(500);
Right_Distance = checkdistance();
delay(500);

// Decide direction
if (Left_Distance > Right_Distance) {
rotate_left(200);
delay(500);
} 
else {
rotate_right(200);
delay(500);
}

delay(400);
myservo.write(90); // Recenter after turning
Serial.println("Obstacle avoided successfully");
}

// This is to overcome the wheels getting stuck
void static_friction() {
go_forward(150);
delay(200);
go_forward(75);
delay(300);
}


void Obstacle_Avoidance_Main() {
Ultrasonic_Front_obstacle_avoidance();
}


// ===================================================
// ========== MOTOR CONTROL ==========================
void go_forward(unsigned char speed_val) {
digitalWrite(pinRB, LOW);  // Front wheels on and back wheels off
digitalWrite(pinRF, HIGH);
digitalWrite(pinLB, LOW);
digitalWrite(pinLF, HIGH);
analogWrite(Lpwm_pin, speed_val);
analogWrite(Rpwm_pin, speed_val);
}


void go_backward(unsigned char speed_val) {
digitalWrite(pinRB, HIGH);  //Front wheels off and back wheels on
digitalWrite(pinRF, LOW);
digitalWrite(pinLB, HIGH);
digitalWrite(pinLF, LOW);
analogWrite(Lpwm_pin, speed_val);
analogWrite(Rpwm_pin, speed_val);
}


void rotate_left(unsigned char speed_val) {
digitalWrite(pinRB, LOW);  // Right front wheel and back left wheel on, other 2 are off
digitalWrite(pinRF, HIGH);
digitalWrite(pinLB, HIGH);
digitalWrite(pinLF, LOW);
analogWrite(Lpwm_pin, speed_val);
analogWrite(Rpwm_pin, speed_val);
}


void rotate_right(unsigned char speed_val) {
digitalWrite(pinRB, HIGH);  // Left front wheel and back right wheel on, other 2 are off
digitalWrite(pinRF, LOW);
digitalWrite(pinLB, LOW);
digitalWrite(pinLF, HIGH);
analogWrite(Lpwm_pin, speed_val);
analogWrite(Rpwm_pin, speed_val);
}


void stopp() {
digitalWrite(pinRB, HIGH);
digitalWrite(pinRF, HIGH);
digitalWrite(pinLB, HIGH);
digitalWrite(pinLF, HIGH);
}



