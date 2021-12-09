#include <Arduino.h>

#define ldr1 12
#define ldr2 13

int ldr1_read = 0;
int ldr2_read = 0;

void setup() {
  Serial.begin(115200); 
  
  pinMode(ldr1, INPUT);
  pinMode(ldr2, INPUT);
}

void loop() {
  ldr1_read = analogRead(ldr1);
  ldr2_read = analogRead(ldr2);

  Serial.print("LDR 1: ");
  Serial.print(ldr1_read);
  Serial.print(" | LDR 2: ");
  Serial.println(ldr2_read);

  delay(500);
}