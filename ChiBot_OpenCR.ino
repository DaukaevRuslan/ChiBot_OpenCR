#include "ChiMotor.h"

ChiMotor chiMotor;
String strIn;
double in;

long timer;

void fuTest() {
  chiMotor.interruptListener();
}

void setup() {
  chiMotor.init(1, 2, 0, 3);
  chiMotor.wheelMode();
  attachInterrupt(digitalPinToInterrupt(2), fuTest, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(3);

  timer = millis();
  
}

void loop() {
  chiMotor.tick();
  if (millis() - timer > 50) {
    Serial.print(chiMotor.getRealRadianVelocity());
    Serial.print(' '); 
    Serial.print(chiMotor.getGoalRadianVelocity());
    Serial.println(' ');
    timer = millis();
  }
  
  

  if (Serial.available() > 0) {
    strIn = Serial.readString();
    in = strIn.toFloat();
    chiMotor.setGoalVelocity(in);
  }
}
