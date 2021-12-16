#include "ChiMotor.h"
#include "ChiRobot.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

char in;
long timer;

double x;
double y;
double z;

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

void cmdGetter(const geometry_msgs::Twist &twist){
  x = 6 * twist.linear.x;
  y = 6 * twist.linear.y;
  z = 10 * twist.angular.z;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> mySub("cmd_vel", cmdGetter);
nav_msgs::Odometry odom;
ros::Publisher myPub("call_back", &odom);

void setup() {

  nh.initNode();
  nh.getHardware()->setBaud(1000000);
  nh.subscribe(mySub);
  nh.advertise(myPub);
  
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
  if (millis() - timer > 20) {

    timer = millis();
    nh.spinOnce();
  }
  
//    if (Serial.available() > 0){
//    in = Serial.read();
//
//    switch(in) {
//      case 'w':
//      x += 1;
//      break;
//      case 'x':
//      x -= 1;
//      break;
//      case 's':
//      x = 0;
//      y = 0;
//      z = 0;
//      break;
//      case 'a':
//      y -= 1;
//      break;
//      case 'd':
//      y += 1;
//      break;
//      case 'q':
//      z -= 6;
//      break;
//      case 'e':
//      z += 6;
//      break;
//    }
//
//    ChiBot.setGoalVelocity(x, y, z);
//    }

    if (nh.connected()){
      ChiBot.setGoalVelocity(x, y, z);  
    }
    else {
      ChiBot.setGoalVelocity(0, 0, 0);
    }
}
