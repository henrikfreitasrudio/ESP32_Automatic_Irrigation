#include <Arduino.h>

#define sensor 32

float voltage = 0.0;

void setup() {
  pinMode(sensor, INPUT);
  Serial.begin(115200);
}

void loop() {
  int leitura = analogRead(sensor);
  voltage = (3.3 / 4095) * leitura;

  Serial.print("PotValue: ");
  Serial.print(leitura);
  Serial.print(" ");
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println("V");

  delay(50);
}