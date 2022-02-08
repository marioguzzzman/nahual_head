 //si hay alguien, prende motores,
//si no hay nadie apaga motores.

#include <AFMotor.h>
AF_DCMotor motor(1);

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
  //motor.setSpeed(255);
  //motor.run(RELEASE);

  //---SENSOR DISTANCIA --

//  pinMode (trigPin, OUTPUT);
//  pinMode (echoPin, INPUT);
//  
//  Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
//  Serial.println("with Arduino UNO R3");
}

void loop()
{

//Read sensor
 // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability

  
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

  // Turn on motor
  motor.run(FORWARD);
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
