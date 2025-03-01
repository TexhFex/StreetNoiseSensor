#ifndef STREET_NOISE_SENSOR_H
#define STREET_NOISE_SENSOR_H

#include <Arduino.h>

class StreetNoiseSensor {
public:
    // Konstruktor
    //  micPin:     ADC-Pin (z.B. A0 für Max9814)
    //  fs:         Abtastrate in Hz (z.B. 8000)
    //  adcOffset:  Offset (z.B. 2048 für 12-Bit)
    //
    //  Beispiel: StreetNoiseSensor sensor(A0, 8000, 2048);
    StreetNoiseSensor(uint8_t micPin, uint32_t fs, uint16_t adcOffset = 2048);

    // Muss in setup() aufgerufen werden, um Timer/ADC einzurichten
    void begin();

    // Startet das Sammeln von Samples (sumOfSquares und sampleCount werden genullt).
    void startMeasurement();

    // Stoppt die Messung, berechnet den RMS-Wert und speichert ihn intern.
    void stopMeasurement();

    // Gibt den zuletzt berechneten RMS-Wert zurück
    float getRMS() const;

private:
    // Settings
    uint8_t  _micPin;
    uint32_t _sampleRate;
    uint16_t _adcOffset;

    // Laufende Summenbildung
    // Hier speichern wir Sum of Squares (double, um Overflow zu reduzieren).
    volatile double _sumOfSquares;
    volatile uint32_t _sampleCount;

    // Interner RMS-Wert nach stopMeasurement()
    volatile float _lastRMS;

    // Timer konfigurieren
    void configureTimer();
    static void startTimer();
    static void stopTimer();

    // Für den ISR-Zugriff
    static StreetNoiseSensor* _instance;
    static void TC4_Handler_Wrapper();
};

#endif
