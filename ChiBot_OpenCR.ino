#include "ChiMotor.h"
#include "ChiRobot.h"

ChiMotor chiMotor1;
ChiMotor chiMotor2;

String strIn;
double in;

long timer;

void fuTest1() {
  chiMotor1.interruptListener();
}
void fuTest2() {
  chiMotor2.interruptListener();
}

void setup() {
  chiMotor1.init(1, 2, 0, 3);
  chiMotor2.init(5, 4, 7, 6);
  chiMotor1.wheelMode();
  chiMotor2.wheelMode();
  attachInterrupt(digitalPinToInterrupt(2), fuTest1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), fuTest2, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(3);

  timer = millis();
  ChiBot.tick();
}

void loop() {
  chiMotor1.tick();
  chiMotor2.tick();
  if (millis() - timer > 50) {
    Serial.print(chiMotor1.getRealRadianVelocity());
    Serial.print(' '); 
    Serial.print(chiMotor2.getGoalRadianVelocity());
    Serial.println(' ');
    timer = millis();
  }
  
  

  if (Serial.available() > 0) {
    strIn = Serial.readString();
    in = strIn.toFloat();
    chiMotor1.setGoalVelocity(in);
    chiMotor2.setGoalVelocity(in);
  }
}
