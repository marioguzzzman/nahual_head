# La conciencia es el precocimiento de la mente universal / Consciousness Is the Foreknowledge of the Universal Mind (2022)

*Ancestral AI / experimental lutherie / techno-chamanism / sound sculpture*

[![Video documentation of the piece](https://img.youtube.com/vi/ASrmGYQaXNI/maxresdefault.jpg)](https://www.youtube.com/watch?v=ASrmGYQaXNI)

**▶ [Watch the piece running](https://www.youtube.com/watch?v=ASrmGYQaXNI)**

Shown at the **Bienal Latinoamericana de Inteligencia Artificial** (México, 2022) and at **AAAh! School of Machines** (2024).

## About the piece

**EN:**
The piece addresses the emergence of consciousness by considering bodily perception, sound, and voice in the field of artificial intelligence, integrating Resonance Theory, experimental lutherie, ceramics, electronics, and neural networks trained on texts by Jacobo Grinberg and the history of shamanism in Mexico.

What is the relationship between mind and matter? Why are some things conscious and others apparently not? Are animals conscious? Stones? Artificial intelligence? Is consciousness a tool? The work addresses the mind-body problem by juxtaposing bodily entities that self-complete in their mutual perception. The ceramic skull is a simple subject that proposes an experience responding to the human skull as a complex subject, opening a space for frequency synchronization whose texture enables mutual reflection.

**ES:**
La pieza aborda la emergencia de la conciencia considerando la percepción corporal, el sonido y la voz en el campo de la inteligencia artificial, integrando la Teoría de la Resonancia, la luthería experimental, la cerámica, la electrónica y redes neuronales entrenadas con textos de Jacobo Grinberg y la historia del chamanismo en México.

¿Cuál es la relación entre la mente y la materia? ¿Por qué algunas cosas son conscientes y otras aparentemente no? ¿Son conscientes los animales? ¿Las piedras? ¿La inteligencia artificial? ¿Es la conciencia una herramienta? La obra aborda el problema "mente-cuerpo" yuxtaponiendo entidades corporales que se auto-completan en su percepción mutua. La cerámica-cráneo es un sujeto simple que propone una experiencia que responde a la presencia del cráneo-humano como sujeto complejo, y habilita un espacio para la sincronización de frecuencias que, en su textura, abre una reflexión mutua.

## How it works

This repository holds the **physical-computing layer** of the piece — sensing, movement, and audio playback. The neural-network voice (trained on Grinberg's texts) is present here only as pre-rendered MP3 tracks on the Touch Board's microSD card; the training and generation code is not in this repo.

Two boards talk to each other over two digital lines:

```
 visitor ──> HC-SR04 ultrasound ──> Arduino UNO + Adafruit Motor Shield
                                      │  presence signal (pin 2 → Touch Board pin 11)
                                      │  DC motor speed mapped to distance
                                      ▼
                            Bare Conductive Touch Board (MPR121 + VS1053 MP3)
                                      │  12 capacitive electrodes in the ceramic
                                      │  touch → plays TRACK000–011 from microSD
                                      │  proximity → modulates volume + LED brightness
                                      │  presence → random track / random volume
                                      └─ "music playing" signal back (pin 10 → Arduino pin 3)
```

- **`motores/motores.ino`** (Arduino + AFMotor): reads the HC-SR04; a person within ~15 cm sets `hay_alguien`, drives the DC motor at a speed mapped to distance, and raises the presence line to the Touch Board. A potentiometer on A3 is read for manual speed control (mapping is written but its output is not applied to the motor in the current code).
- **`touch_mp3_with_leds_V2/`** (Touch Board): the main performance sketch. Touching an electrode plays the corresponding track; the MPR121 baseline-vs-filtered reading of the active electrode modulates volume and LED brightness (IIR-smoothed). When the presence line is high, `rolita_alguien()` interrupts with a randomly chosen track; a millis-based timer (`timer_rolita()`) also fires random tracks at random volumes. Signals "music playing" back to the Arduino.
- **`random_touch_v2/`, `random_touch_v1/`**: development iterations of the touch/proximity-volume sketch, kept as work history.
- **`DataStream_nahual/`**: stock Bare Conductive datastream sketch — streams raw MPR121 capacitance data over serial, used for calibrating the electrodes.
- **`Touch Board Communication_TEST.maxpat`** + **`Max-MSP-Touch-Board-communication-public/`**: Max/MSP patch (Bare Conductive's public patch plus a test version) for visualizing the electrode datastream during calibration.
- **`installers/`**: stock Bare Conductive setup bundle (Arduino board definitions, libraries, example sketches) — vendor files kept for convenience, not part of the artwork's code.

## Running it

Requires the physical hardware — a Bare Conductive Touch Board, an Arduino UNO with Adafruit Motor Shield (v1), an HC-SR04, a DC motor, and the wired ceramic — so none of this is testable without the sculpture. Verified against the code:

1. Install Arduino IDE support for the Touch Board using `installers/` (or Bare Conductive's current setup guide).
2. Libraries: `MPR121` + `MPR121_Datastream` (Bare Conductive), `SFEMP3Shield`, `SdFat`, and `AFMotor` (Adafruit Motor Shield v1 library) for the Arduino sketch.
3. Put twelve MP3s named `TRACK000.mp3`–`TRACK011.mp3` in the root of the Touch Board's microSD card — these are the piece's voice.
4. Flash `touch_mp3_with_leds_V2` to the Touch Board and `motores` to the Arduino. Wire the two signal lines (Arduino pin 2 → Touch Board pin 11; Touch Board pin 10 → Arduino pin 3) and the HC-SR04 on A0/A1.
5. Calibrate electrodes with `DataStream_nahual` + the Max patch if touch response drifts (new enclosure, humidity, paint).
