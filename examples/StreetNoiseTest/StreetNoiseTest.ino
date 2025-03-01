#include <StreetNoiseSensor.h>

// Beispiel: 8 kHz, A0-Pin, Offset = 2048 (12-Bit ADC)
StreetNoiseSensor sensor(A0, 8000, 2048);

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("StreetNoiseTest: setup()");
  sensor.begin();
}

void loop() {
  Serial.println("Start measurement (10s) ...");
  sensor.startMeasurement();

  // Hier z.B. 10s blockierend warten oder andere Sensoren (SCD41/SPS30) abfragen
  delay(10000);

  sensor.stopMeasurement();
  float rmsVal = sensor.getRMS();

  Serial.print("RMS: ");
  Serial.println(rmsVal, 2);

  // Pause von 5 Sekunden
  delay(5000);
}
