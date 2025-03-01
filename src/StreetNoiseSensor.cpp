#include "StreetNoiseSensor.h"

// Statischer Zeiger auf die aktuell genutzte Instanz (für ISR)
StreetNoiseSensor* StreetNoiseSensor::_instance = nullptr;

// Konstruktor
StreetNoiseSensor::StreetNoiseSensor(uint8_t micPin, uint32_t fs, uint16_t adcOffset)
: _micPin(micPin)
, _sampleRate(fs)
, _adcOffset(adcOffset)
, _sumOfSquares(0.0)
, _sampleCount(0)
, _lastRMS(0.0f)
{
}

// Initialisierung
void StreetNoiseSensor::begin() {
    _instance = this;  // Setze globale Instanz für die ISR

    analogReadResolution(12); // 12-Bit
    // optional: analogReference(AR_DEFAULT) o.Ä.

    configureTimer();
}

// startMeasurement(): Summen und Zähler zurücksetzen, Timer an
void StreetNoiseSensor::startMeasurement() {
    noInterrupts();
    _sumOfSquares = 0.0;
    _sampleCount = 0;
    _lastRMS = 0.0f;
    interrupts();

    startTimer();
}

// stopMeasurement(): Timer aus, RMS berechnen
void StreetNoiseSensor::stopMeasurement() {
    stopTimer();

    noInterrupts();
    // RMS = sqrt( sumOfSquares / sampleCount )
    if (_sampleCount == 0) {
        // Keine Samples -> RMS = 0
        _lastRMS = 0.0f;
    } else {
        double meanSquare = _sumOfSquares / (double)_sampleCount;
        _lastRMS = (float)sqrt(meanSquare);
    }
    interrupts();
}

// getRMS(): Liefert den letzten berechneten RMS
float StreetNoiseSensor::getRMS() const {
    return _lastRMS;
}

// Timer konfigurieren (TC4, Arduino Zero - 48 MHz)
void StreetNoiseSensor::configureTimer() {
    // APB für TC4 aktivieren
    PM->APBCMASK.reg |= PM_APBCMASK_TC4;

    // GCLK für TC4
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | 
                                   GCLK_CLKCTRL_GEN_GCLK0 | 
                                   GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Timer stoppen, bevor wir ihn konfigurieren
    TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    // 16-Bit Mode
    TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16;

    // Prescaler /1024
    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
    
    // Basistakt: 48MHz / 1024 = 46875 Hz
    if (_sampleRate < 10) {
        _sampleRate = 10; // Minimale Sicherung
    }
    uint32_t baseFreq = 48000000UL / 1024UL; // 46875
    uint32_t compareValue = baseFreq / _sampleRate; 
    if (compareValue < 2) {
        compareValue = 2; 
    }

    // Waveform: MFRQ (Match Frequency)
    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    TC4->COUNT16.CC[0].reg = (uint16_t)compareValue;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    // Compare0-Interrupt aktivieren
    TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
    NVIC_SetPriority(TC4_IRQn, 1);
    NVIC_EnableIRQ(TC4_IRQn);
}

// Timer starten
void StreetNoiseSensor::startTimer() {
    TC4->COUNT16.COUNT.reg = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

    TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
}

// Timer stoppen
void StreetNoiseSensor::stopTimer() {
    TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
}

// ISR-Wrapper
void StreetNoiseSensor::TC4_Handler_Wrapper() {
    // Check Compare0 Flag
    if (TC4->COUNT16.INTFLAG.bit.MC0) {
        // Flag löschen
        TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

        // ADC auslesen + Summenbildung
        // Safety-Check, ob _instance existiert
        if (_instance) {
            uint16_t raw = analogRead(_instance->_micPin);
            int centered = (int)raw - (int)_instance->_adcOffset;

            // Quadratsumme inkrementell
            // (double für mehr "headroom")
            _instance->_sumOfSquares += (double)centered * (double)centered;
            _instance->_sampleCount++;
        }
    }
}

// Tatsächliche ISR-Funktion
#ifdef __cplusplus
extern "C" {
#endif
void TC4_Handler() {
    StreetNoiseSensor::TC4_Handler_Wrapper();
}
#ifdef __cplusplus
}
#endif
