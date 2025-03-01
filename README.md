# StreetNoiseSensor

Arduino library for background RMS measurement of street noise using a microphone (e.g. Max9814) on SAMD21 (Arduino Zero). 

## Features

- No large buffer required. 
- Timer interrupt on TC4 for continuous ADC sampling.
- On-the-fly RMS calculation from `startMeasurement()` until `stopMeasurement()`.

## Installation

1. Place the `StreetNoiseSensor` folder into your Arduino `libraries` folder.
2. Restart the Arduino IDE.

## Usage

See the example `StreetNoiseTest.ino`.
