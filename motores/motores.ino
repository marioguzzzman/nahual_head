
/* -----BIENAL LATINOAMERICANA DE INTELIGENCIA ARTIFICIAL---
  Mario Alberto Guzman Cerdio
  mariouzzzman@gmail.com  */

//si hay alguien, prende motores,
//si no hay nadie apaga motores.

boolean hay_alguien = false;
boolean ver_distancia = true;
boolean manual = false;

//------------------------------------  I2C Master Demo

// Include Arduino Wire library for I2C
#include <Wire.h>

// Define Slave I2C Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define ANSWERSIZE 5

//-------------------------------------MOTORES
#include <AFMotor.h>
#include <Servo.h>

//Defining the DC motor you are using.
AF_DCMotor motor(1);

Servo myservo;

int distanceXspeed;
int minSpeed = 20;


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

void setup()
{

  //------------------ Initialize I2C communications as Master
  //Wire.begin();
  
  // Setup serial monitor
  Serial.begin(9600);
  Serial.println("I2C Master Demonstration");


  //-----------------------------------------MOTOR
  //Set initial speed of the motor & stop
  motor.setSpeed(0);
  motor.run(RELEASE);

  myservo.attach(9);   //Determine the pin connecting to Servo.(pin 9 for sevo #1 and pin 10 for servo #2)


  //---SENSOR DISTANCIA --

  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);

  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  Serial.println("with Arduino UNO R3");
}

void loop()
{

//------------------------------------  I2C Master Demo

//   delay(50);
//  Serial.println("Write data to slave");
//  
//  // Write a charatre to the Slave
//  Wire.beginTransmission(SLAVE_ADDR);
//  Wire.write(0);
//  Wire.endTransmission();
//    
//  Serial.println("Receive data");
//  
//  // Read response from Slave
//  // Read back 5 characters
//  Wire.requestFrom(SLAVE_ADDR,ANSWERSIZE);
//  
//  // Add characters to string
//  String response = "";
//  while (Wire.available()) {
//      char b = Wire.read();
//      response += b;
//  } 
//  
//  // Print to Serial Monitor
//  Serial.println(response);

//-----------------------------------------------
  if (manual == true) {

    Serial.print("Manual");

    //-------------------------------------POTENTIOMETER
    //Read potentiometer
    // read the input on analog pin 5:
    motorSpeedPot = analogRead(A3);

    /* Map an analog value to 8 bits (0 to 255) */
    //motorSpeedPot = map(motorSpeedPot, 0, 1023, 0, 255);
    motorSpeedPot = map(motorSpeedPot, 0, 1023, 0, 360);

    // print out the value you read:
    Serial.print("Motor: ");
    Serial.println(motorSpeedPot);
    delay(1);        // delay in between reads for stability
  }


  sensordistancia();

  if (distance < 20) {
    hay_alguien = true;
    //enciendo motores

  } else if (distance > 30) {
    hay_alguien = false;
    //apago motores

  }

  //------------------------------ALGUIEN
  if (hay_alguien == true) {

    Serial.print("ALGUIEN ");


    //change variables here once I know distance
    //map(value, fromLow, fromHigh, toLow, toHigh)
    //map(distance, desde0, donde potencialmente esta la cabeza, speed Min, speed Max)

    distanceXspeed = map(distance, 0, 20, 25, 360);

    //Set controlled speed of the motor & stop
    motor.setSpeed(distanceXspeed);

    // print out the value you read:
    Serial.print("Motor: ");
    Serial.println(distanceXspeed);
    delay(1);        // delay in between reads for stability


    //------------------------------ NADIE


  } else if (hay_alguien == false) {

    Serial.print("................NADIE ");

    //Set controlled speed of the motor & stop
    motor.setSpeed(minSpeed);

    // print out the value you read:
    Serial.print("Motor: ");
    Serial.println(minSpeed);
    delay(1);        // delay in between reads for stability

  }



  // print out the value you read:
  Serial.print("Motor: ");
  Serial.println(distanceXspeed);
  delay(1);        // delay in between reads for stability

  //-------------------------------------MOTOR

  //Set controlled speed of the motor & stop
  motor.setSpeed(motorSpeedPot);

  // Turn on motor
  motor.run(FORWARD);

  //Servo
  //Determine the amount of motor rotation. Between 0 to 360 or 0 to 180 according to motor type.

  //myservo.write(motorSpeedPot);
  //delay(15);

  // }



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
