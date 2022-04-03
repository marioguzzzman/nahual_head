
/* -----BIENAL LATINOAMERICANA DE INTELIGENCIA ARTIFICIAL---
  Mario Alberto Guzman Cerdio
  mariouzzzman@gmail.com  */

//si hay alguien, prende motores,
//si no hay nadie apaga motores.

boolean hay_alguien;
boolean hay_musica;

boolean ver_distancia = true;
boolean manual = true;

int distanciaPersona = 15;

// send data to Touchboard
int count;
int dataSend = 2;
int dataIn = 3;


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

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);


  //send data
  pinMode(dataSend, OUTPUT);
  //Receive data
  pinMode(dataIn, INPUT);


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
}

//--------------------------------------------------LOOP
//--------------------------------------------------LOOP

void loop()
{

  checkdata();

  //testSendData();

  sensordistancia();

  //if (myVal > lowerBound && myVal < upperBound) {
  if (distance > 2 && distance < distanciaPersona) {
    hay_alguien = true;
    //enciendo motores
    digitalWrite(dataSend, HIGH); //Envio info a touchboard desde digitalpin 2

  } else {
    hay_alguien = false;
    //apago motores

    digitalWrite(dataSend, LOW); //Envio info a touchboard desde digitalpin 2

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


  } else if (hay_alguien == false || hay_musica == true) { //------------------------ NADIE

    //if nadie or music is paying
    Serial.print("................NADIE ");
    delay(1);        // delay in between reads for stability

    //Set controlled speed of the motor & stop
    motor.setSpeed(0);      //-------------------------------------MOTOR NADIE
    // Turn on motor
    motor.run(BACKWARD);

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(250);                       // wait for a second


  } else if (hay_musica == false) {
    // sigue tocando
    Serial.print("................SILENCIO ");
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(250);

  }
}

//---------------------------------------
//------------------ END LOOP -----------
//---------------------------------------

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

void testSendData() {
  if (count % 10 == 0) {
    //Serial.println('a');
    //    digitalWrite(, HIGH);
    digitalWrite(dataSend, HIGH);
    Serial.println("mando data");

    //delay(1000);

    //delay(250);
  } else {
    // digitalWrite(sendPin, LOW);
    digitalWrite(dataSend, LOW);
    //delay(2000);
  }
  count++;
  delay(100);
   //   Serial.println(digitalWrite(dataSend));

}

//----------------------------CHECK DATA
void checkdata() {

  if (digitalRead(dataIn) == 1) {
    hay_musica = true;
    Serial.println("------HAY MUSICA");

//    rolita_alguien();

  } else if (digitalRead(dataIn) == 0) {
    hay_musica = false;
    Serial.println("------SILENCIO");

  }

  Serial.println(digitalRead(dataIn));

}
