
/* -----BIENAL LATINOAMERICANA DE INTELIGENCIA ARTIFICIAL---
  Mario Alberto Guzman Cerdio
  mariouzzzman@gmail.com  */

//si hay alguien, prende motores,
//si no hay nadie apaga motores.

boolean hay_alguien = false;
boolean ver_distancia = true;

//-------------------------------------MOTORES
#include <AFMotor.h>
//Defining the DC motor you are using.
AF_DCMotor motor(1);

//-------------------------------------POTENTIOMETER
int motorSpeedPot = A5;


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
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

  //Set initial speed of the motor & stop
  motor.setSpeed(0);
  motor.run(RELEASE);

  //---SENSOR DISTANCIA --

    pinMode (trigPin, OUTPUT);
    pinMode (echoPin, INPUT);
  
    Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
    Serial.println("with Arduino UNO R3");
}

void loop()
{

if (hay_alguien == false) {

//-------------------------------------POTENTIOMETER
    //Read potentiometer
  // read the input on analog pin 5:
  motorSpeedPot = analogRead(A5);

  /* Map an analog value to 8 bits (0 to 255) */
  motorSpeedPot = map(motorSpeedPot, 0, 1023, 0, 255);

  // print out the value you read:
  Serial.println(motorSpeedPot);
  delay(1);        // delay in between reads for stability

//-------------------------------------MOTOR

   //Set controlled speed of the motor & stop
  motor.setSpeed(motorSpeedPot);

  // Turn on motor
  motor.run(FORWARD);

    }



  //  // Clears the trigPin condition
  //  analogWrite(trigPin, LOW);
  //  delayMicroseconds(2);
  //  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  //  analogWrite(trigPin, HIGH);
  //  delayMicroseconds(10);
  //  analogWrite(trigPin, LOW);
  //  // Reads the echoPin, returns the sound wave travel time in microseconds
  //  duration = pulseIn(echoPin, HIGH);
  //  // Calculating the distance
  //  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  //  // Displays the distance on the Serial Monitor
  //  Serial.print("Distance: ");
  //  Serial.print(distance);
  //  Serial.println(" cm");

  uint8_t i;


}

//void sensordistancia () {
//  // boolean vuelve = false;
//  long duration;
//  digitalWrite(trigPin, LOW);  // Added this line
//  delayMicroseconds(2); // Added this line
//  digitalWrite(trigPin, HIGH);
//  //  delayMicroseconds(1000); - Removed this line
//  delayMicroseconds(10); // Added this line
//  digitalWrite(trigPin, LOW);
//  duration = pulseIn(echoPin, HIGH);
//  // distance = (duration/2) / 29.1;
//  distance = duration / 58.2;
//}
