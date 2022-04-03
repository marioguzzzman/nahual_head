
/*******************************************************************************
  -----BIENAL LATINOAMERICANA DE INTELIGENCIA ARTIFICIAL---
  Mario Alberto Guzman Cerdio
  mariouzzzman@gmail.com

  You need twelve MP3 named TRACK000.mp3 to TRACK011.mp3 in the root of
  microSD card.

  When you touch electrode E0, TRACK000.mp3 will play. When you touch electrode
  E1, TRACK001.mp3 will play

  based on: https://github.com/BareConductive/touch_mp3_with_leds
  https://github.com/BareConductive/prox-volume/blob/public/prox_volume/prox_volume.ino

*******************************************************************************/
//Hay alguien
boolean hay_alguien;

//Send data
int count;
int inPin = 11;
int outPin = 10;

// TIMER
unsigned long myTime;
long rolita_random;
long tiempo_rolita_random = 10;
long vol_random;

// compiler error handling
#include "Compiler_Errors.h"

// touch includes
#include <MPR121.h>
#include <Wire.h>
#define MPR121_ADDR 0x5C
#define MPR121_INT 4

// mp3 includes
#include <SPI.h>
#include <SdFat.h>
#include <FreeStack.h>
#include <SFEMP3Shield.h>

// mp3 variables
SFEMP3Shield MP3player;
byte result;
int lastPlayed = 0;
uint8_t volume = 0;

// mp3 behaviour defines
#define REPLAY_MODE FALSE  // By default, touching an electrode repeatedly will 
// play the track again from the start each time.
//
// If you set this to FALSE, repeatedly touching an
// electrode will stop the track if it is already
// playing, or play it from the start if it is not.

// touch behaviour definitions
#define firstPin 0
#define lastPin 11 // 11

// sd card instantiation
SdFat sd;

// LED pins
// maps electrode 0 to digital 0, electrode 2 to digital 1, electrode 3 to digital 10 and so on...
// A0..A5 are the analogue input pins, used as digital outputs in this example
//const int ledPins[12] = {0, 1, 10, 11, 12, 13, A0, A1, A2, A3, A4, A5};

const int ledPins[3] = {10, 11, 13};
//const int ledPins[6] = {5, 6, 9, 10, 11, 13};

// mapping and filter definitions
#define LOW_DIFF 0
#define HIGH_DIFF 50
#define filterWeight 0.5f // 0.0f to 1.0f - higher value = more smoothing
float lastProx = 0;

// the electrode to monitor
//#define ELECTRODE 0
//#define ELECTRODE 0
//#define ELECTRODE 1

int ELECTRODE_NOW = 0;

//LEDS
//int ledPins[0] = 10;
//int ledPins[1] = 11;
// PWM pins = 13, 11, 10, 9, 6, 5
//#ifndef LED_BUILTIN
//#define LED_BUILTIN 13
//#endif

void setup() {

  //recieve data
  pinMode(inPin, INPUT);

  //Send data
  pinMode(outPin, OUTPUT);

  //pinMode (13, OUTPUT);//Connect LED to pin 13
  //  Wire.begin(9);//9 here is the address(Mentioned even in the master board code)
  //  Wire.onReceive(receiveEvent);

  //-------------------------------------

  Serial.begin(9600);
  // Serial.begin(57600);

  //while (!Serial) ; {} //uncomment when using the serial monitor
  // Serial.println("Bare Conductive Touch MP3 player");

  if (!sd.begin(SD_SEL, SPI_HALF_SPEED)) sd.initErrorHalt();

  if (!MPR121.begin(MPR121_ADDR)) Serial.println("error setting up MPR121");
  MPR121.setInterruptPin(MPR121_INT);

  MPR121.setTouchThreshold(40);
  MPR121.setReleaseThreshold(20);

  result = MP3player.begin();
  MP3player.setVolume(volume, volume);

  if (result != 0) {
    Serial.print("Error code: ");
    Serial.print(result);
    Serial.println(" when trying to start MP3 player");
  }

  //pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledPins[0], OUTPUT); // initialize the pin
  pinMode(ledPins[1], OUTPUT); // initialize the pin
  pinMode(ledPins[2], OUTPUT); // initialize the pin

  //This supplies 5 volts to the LED anode,the positive leg
  //(pin,pwm value)
  //analogWrite(LED_BUILTIN, 0);
  analogWrite(ledPins[0], 0);
  analogWrite(ledPins[1], 0);
  analogWrite(ledPins[2], 0);

  // slow down some of the MPR121 baseline filtering to avoid
  // filtering out slow hand movements
  MPR121.setRegister(MPR121_NHDF, 0x01); //noise half delta (falling)
  MPR121.setRegister(MPR121_FDLF, 0x3F); //filter delay limit (falling)

}

void loop() {

  checkdata();
  testSendData();

  //------------------------TIMER

  timer_rolita();

  //--------------------TOUCH

  readTouchInputs();
  //  checkTrackFinished();

  // update all of the data from the MPR121
  MPR121.updateAll();

  Serial.println("on loop");

  //******THIS IS THE SAME AS LED
  // read the difference between the measured baseline and the measured continuous data
  int reading = MPR121.getBaselineData(ELECTRODE_NOW) - MPR121.getFilteredData(ELECTRODE_NOW);

  // print out the reading value for debug
 // Serial.println(reading);

  // constrain the reading between our low and high mapping values
  unsigned int prox = constrain(reading, LOW_DIFF, HIGH_DIFF);

  // implement a simple (IIR lowpass) smoothing filter
  lastProx = (filterWeight * lastProx) + ((1 - filterWeight) * (float)prox);

  // map the LOW_DIFF..HIGH_DIFF range to 0..255 (8-bit resolution for analogWrite)
  uint8_t thisOutput = (uint8_t)map(lastProx, LOW_DIFF, HIGH_DIFF, 0, 255);
  uint8_t thisVolume = (uint8_t)map(lastProx, LOW_DIFF, HIGH_DIFF, 0, 254);

  //analogWrite(ELECTRODE_NOW, thisOutput);
  //analogWrite(ledPins[1], thisOutput);
  for (int i = 0; i < 3; i++) {
    if (i == ELECTRODE_NOW) {
      analogWrite(ledPins[ELECTRODE_NOW], thisOutput);
    } else {
      analogWrite(ledPins[i], 0);
    }
  }
  // if((uint8_t)lastProx!=prox){ // only update volume if the value has changed
  MP3player.setVolume(thisVolume, thisVolume);
}

//---------------------------------------
//------------------ END LOOP -----------
//---------------------------------------


//----------------------------CHECK DATA
void checkdata() {

  if (digitalRead(inPin) == 1) {
    digitalWrite(13, HIGH);
    hay_alguien = true;
    Serial.println("------HAY ALGUIEN");

    rolita_alguien();

  } else if (digitalRead(inPin) == 0) {
    digitalWrite(13, LOW);
    hay_alguien = false;

    Serial.println("------NADIE");
    //rolita_alguien();

  }

  //Serial.println(digitalRead(inPin));

}


void readTouchInputs() {
  if (MPR121.touchStatusChanged()) {

    MPR121.updateTouchData();

    // only make an action if we have one or fewer pins touched
    // ignore multiple touches

    if (MPR121.getNumTouches() <= 1) {

     // Serial.print("volume: ");
      //Serial.println (volume);


      for (int i = 0; i < 12; i++) { // Check which electrodes were pressed
        if (MPR121.isNewTouch(i)) {

          //pin i was just touched
          Serial.print("pin ");
          Serial.print(i);
          Serial.println(" was just touched");

          if (i <= lastPin && i >= firstPin) {
            if (MP3player.isPlaying()) {
              if (lastPlayed == i && !REPLAY_MODE) {
                // if we're already playing the requested track, stop it
                // (but only if we're in REPLAY_MODE)
                MP3player.stopTrack();
                Serial.print("stopping track ");
                Serial.println(i - firstPin);

                digitalWrite(outPin, LOW); // SEND DATA, MUSIC STORED

              } else {
                ELECTRODE_NOW = i;
                // if we're already playing a different track (or we're in
                // REPLAY_MODE), stop and play the newly requested one
                MP3player.stopTrack();
                MP3player.playTrack(i - firstPin);
                Serial.print("playing track ");
                Serial.println(i - firstPin);

                // switch off the relevant LED output
                //digitalWrite(ledPins[lastPlayed], LOW);
                //analogWrite(ledPins[lastPlayed], LOW);

                // don't forget to update lastPlayed - without it we don't
                // have a history
                lastPlayed = i;

                digitalWrite(outPin, HIGH);// SEND DATA, MUSIC IS PLAYIN

              }
            } else {

              //ELECTRODE_NOW = i;
              // if we're playing nothing, play the requested track
              // and update lastplayed
              MP3player.playTrack(i - firstPin);
              Serial.print("playing track ");
              //Serial.println(i - firstPin);

              lastPlayed = i;

            }
          }
        } else {
          if (MPR121.isNewRelease(i)) {
            Serial.print("pin ");
            Serial.print(i);
            Serial.println(" is no longer being touched");
          }
        }
      }
    }
  }
}

void checkTrackFinished() {
  if (!MP3player.isPlaying()) {
    //digitalWrite(ledPins[lastPlayed], LOW);
    // digitalWrite(ledPins[1], LOW);
    analogWrite(ledPins[lastPlayed], 0);

  }
}

void timer_rolita() {
  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  Serial.print("Time: ");
  myTime = millis();
  Serial.println(myTime); // prints time since program started
  delay(1000);          // wait a second so as not to send massive amounts of data

  if (myTime % tiempo_rolita_random == 0) {

    tiempo_rolita_random = random(500, 800);
    Serial.print("tiempo random: ");
    Serial.println (tiempo_rolita_random);

    rolita_random = random(0, 11);
    Serial.print("TOCANDO ROLITA: ");
    Serial.println (rolita_random);

    vol_random = random(7, 20);
    Serial.print("volume: ");
    Serial.println (vol_random);

    MP3player.stopTrack();
    MP3player.playTrack(rolita_random);
    MP3player.setVolume(vol_random, vol_random);


  }
}


void rolita_alguien() {
  if (count % 15 == 0) {
    //digitalWrite(dataSend, HIGH);

    // if analog input pin 0 is unconnected, random analog
    // noise will cause the call to randomSeed() to generate
    // different seed numbers each time the sketch runs.
    // randomSeed() will then shuffle the random function.
    randomSeed(analogRead(0));

    rolita_random = random(0, 11);

    Serial.print(" ROLITA ALGUIEN: ");
    Serial.println (rolita_random);

    Serial.print("volume: ");
    Serial.println (vol_random);

    MP3player.stopTrack();
    MP3player.playTrack(rolita_random);
    MP3player.setVolume(20, 20);

  } else {
    //digitalWrite(dataSend, LOW);
  }
  count++;
  delay(1000);

}


void testSendData() {
  if (count % 10 == 0) {
    //Serial.println('a');
    //    digitalWrite(, HIGH);
    digitalWrite(outPin, HIGH);
    Serial.println("mando data");

    //delay(1000);

    //delay(250);
  } else {
    // digitalWrite(sendPin, LOW);
    digitalWrite(outPin, LOW);
    //delay(2000);
  }
  count++;
  delay(100);
  //   Serial.println(digitalWrite(dataSend));

}
