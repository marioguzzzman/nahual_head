
/* -----BIENAL LATINOAMERICANA DE INTELIGENCIA ARTIFICIAL---
  Mario Alberto Guzman Cerdio
  mariouzzzman@gmail.com  */

//si hay alguien, prende motores,
//si no hay nadie apaga motores.

boolean hay_alguien;
boolean ver_distancia = true;
boolean manual = true;

int distanciaPersona = 15;

int count;
int sendPin = 0;

//------------------------------------  I2C Master Demo

//// Include Arduino Wire library for I2C
//#include <Wire.h>
//
//// Define Slave I2C Address
//#define SLAVE_ADDR 9
//
//// Define Slave answer size
//#define ANSWERSIZE 5

//-------------------------------------MOTORES
#include <AFMotor.h>

//Defining the DC motor you are using.
AF_DCMotor motor(1);

int distanceXspeed;
int minSpeed = 230;
int minSpeedMap = 200;

int x;


//-------------------------------------POTENTIOMETER
int motorSpeedPot = A3;


//-------------------------------------ULTRASOUND
//#define echoPin A0 // attach pin D2 Arduino to pin Echo of HC-SR04
//#define trigPin A1 //attach pin D3 Arduino to pin Trig of HC-SR04

const int trigPin = A1; // Ultrasound signal pin
const int echoPin = A0;

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//--------------------------------------------------SETUP
//--------------------------------------------------SETUP
void setup()
{

  pinMode(sendPin, OUTPUT);

 // Setup serial monitor
  Serial.begin(9600);


  //-----------------------------------------MOTOR
  //Set initial speed of the motor & stop
  motor.setSpeed(minSpeed);
  //motor.run(RELEASE);

  //---SENSOR DISTANCIA --

  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);

  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}

//--------------------------------------------------LOOP
//--------------------------------------------------LOOP

void loop()
{

  if (count % 10 == 0) {
    //Serial.println('a');
//    digitalWrite(2, HIGH);
        digitalWrite(2, HIGH);
        //Serial.println("mando data");
        //Serial.println(digitalWrite(sendPin);
  } else {
    digitalWrite(sendPin, LOW);
  }
  count++;

  delay(100);

  //  motor.setSpeed(200);

  //   //Set controlled speed of the motor & stop
  //    motor.setSpeed(minSpeed);       //-------------------------------------MOTOR ALGUIEN
  //    // Turn on motor
  //  motor.run(BACKWARD);

  sensordistancia();

  if (distance < distanciaPersona) {
    hay_alguien = true;
    //enciendo motores

  } else if (distance > distanciaPersona) {
    hay_alguien = false;
    //apago motores

  }

  //----------------------------------------------- MANUAL
  //if (manual == true) {

  // Serial.print("Manual");
  // Serial.print("    ");
  //delay(1000);

  //-------------------------------------POTENTIOMETER
  //Read potentiometer
  // read the input on analog pin 5:
  motorSpeedPot = analogRead(A3);

  /* Map an analog value to 8 bits (0 to 255) */
  //motorSpeedPot = map(motorSpeedPot, 0, 1023, 0, 255);
  motorSpeedPot = map(motorSpeedPot, 0, 1023, 0, 255);

  // print out the value you read:
  // Serial.print("Motor: ");
  //Serial.println(motorSpeedPot);
  //delay(1);        // delay in between reads for stability

  //} else
  if (hay_alguien == true) {  //------------------------------ALGUIEN
    Serial.print("ALGUIEN ");
    Serial.print("    ");
    delay(1);

    //change variables here once I know distance
    //map(value, fromLow, fromHigh, toLow, toHigh)
    //map(distance, desde0, donde potencialmente esta la cabeza, speed Min, speed Max)

    distanceXspeed = map(distance, 0, distanciaPersona, minSpeedMap, 255);

    //    //Set controlled speed of the motor & stop
    motor.setSpeed(distanceXspeed);       //-------------------------------------MOTOR ALGUIEN
    // Turn on motor
    motor.run(BACKWARD);

    // print out the value you read:
    Serial.print("Motor: ");
    Serial.println(distanceXspeed);
    delay(1);        // delay in between reads for stability


  } else { //------------------------ NADIE
    Serial.print("................NADIE ");
    delay(1);        // delay in between reads for stability

    //Set controlled speed of the motor & stop
    motor.setSpeed(0);      //-------------------------------------MOTOR NADIE
    // Turn on motor
    motor.run(BACKWARD);

  }

}

void sensordistancia () {
  // boolean vuelve = false;
  long duration;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // distance = (duration/2) / 29.1;
  distance = duration / 58.2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
