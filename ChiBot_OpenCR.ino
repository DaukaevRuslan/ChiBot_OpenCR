#include "ChiMotor.h"
#include "ChiRobot.h"

char in;
long timer;

int x = 0;
int y = 0;
int z = 0;

void fuTest1() {
  ChiBot.ChiMotorFL.interruptListener();
}
void fuTest2() {
  ChiBot.ChiMotorBL.interruptListener();
}
void fuTest3() {
  ChiBot.ChiMotorBR.interruptListener();
}
void fuTest4() {
  ChiBot.ChiMotorFR.interruptListener();
}

void setup() {
  ChiBot.init(10, 4, 11, 5,
              12, 7,  1, 6,
              14, 8, 15, 9,
               0, 2, 61, 3);
  attachInterrupt(digitalPinToInterrupt(4), fuTest1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), fuTest2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), fuTest3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), fuTest4, CHANGE);

  Serial.begin(115200);
  Serial.setTimeout(3);

  timer = millis();
  
}

void loop() {
  ChiBot.tick();
  if (millis() - timer > 50) {

    timer = millis();
  }
  
    if (Serial.available() > 0){
    in = Serial.read();

    switch(in) {
      case 'w':
      x += 1;
      break;
      case 'x':
      x -= 1;
      break;
      case 's':
      x = 0;
      y = 0;
      z = 0;
      break;
      case 'a':
      y -= 1;
      break;
      case 'd':
      y += 1;
      break;
      case 'q':
      z -= 6;
      break;
      case 'e':
      z += 6;
      break;
    }

    ChiBot.setGoalVelocity(x, y, z);
    }
}
