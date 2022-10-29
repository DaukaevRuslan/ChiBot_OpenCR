#include "ChiMotor.h"
#include "ChiRobot.h"

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

long timerVoltage;
long timerConnect;
long timerVelocity;
bool flagConnect = false;

float goalVelocityX = 0.0;
float goalVelocityY = 0.0;
float goalVelocityZ = 0.0;

double realVelocityX = 0.0;
double realVelocityY = 0.0;
double realVelocityZ = 0.0;

double X = 0.0;
double Y = 0.0;
double A = 0.0;

void interruptListenerFL() {
  ChiBot.ChiMotorFL.interruptListener();
}
void interruptListenerBL() {
  ChiBot.ChiMotorBL.interruptListener();
}
void interruptListenerBR() {
  ChiBot.ChiMotorBR.interruptListener();
}
void interruptListenerFR() {
  ChiBot.ChiMotorFR.interruptListener();
}

float checkVoltage() {
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);
  float vol_value = map(adc_value, 0, 1023, 0, 1630);  //330*57/10
  vol_value = vol_value / 100;
  return vol_value;
}

typedef ros::NodeHandle_<ArduinoHardware, 11, 11, 16384, 16384> NodeHandle;
ros::NodeHandle nh;

void cmdGetter(const geometry_msgs::Twist &twist) {

  goalVelocityX = twist.linear.x;
  goalVelocityY = -twist.linear.y;
  goalVelocityZ = -twist.angular.z;
}

void coonectListener(const std_msgs::String &connectString) {
  String conn = connectString.data;
  if (conn.equals("Connect")) flagConnect = true;
  else flagConnect = false;
}

ros::Subscriber<geometry_msgs::Twist> mySub("cmd_vel", cmdGetter);
ros::Subscriber<std_msgs::String> subConnect("connection", coonectListener);

std_msgs::Float32 voltageMsg;
ros::Publisher pubVoltage("voltage", &voltageMsg);
std_msgs::Float64MultiArray velocityMsg;
ros::Publisher pubVelocity("velocity_data", &velocityMsg);

void setup() {

  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(mySub);
  nh.subscribe(subConnect);
  nh.advertise(pubVoltage);
  nh.advertise(pubVelocity);

  velocityMsg.data = (float*)malloc(sizeof(float) * 4);
  velocityMsg.data_length = 4;

  ChiBot.init(10, 4, 11, 5,
              12, 7,  1, 6,
              14, 8, 15, 9,
              0, 2, 61, 3);

  attachInterrupt(digitalPinToInterrupt(4), interruptListenerFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), interruptListenerBL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), interruptListenerBR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), interruptListenerFR, CHANGE);

  timerVoltage = millis();
  timerConnect = millis();
  timerVelocity = millis();

  pinMode(A4, OUTPUT); // Индикация включения
  pinMode(A5, OUTPUT); // Индикация начала общения с верхним уровнем

}

void loop() {

  ChiBot.tick();

  if (millis() - timerVelocity > 100) {
    timerVelocity = millis();

    velocityMsg.data[0] = ChiBot.ChiMotorFL.getRealRadianVelocity();
    velocityMsg.data[1] = ChiBot.ChiMotorBL.getRealRadianVelocity();
    velocityMsg.data[2] = ChiBot.ChiMotorBR.getRealRadianVelocity();
    velocityMsg.data[3] = ChiBot.ChiMotorFR.getRealRadianVelocity();

    pubVelocity.publish(&velocityMsg);

    if (goalVelocityX > -0.01 and goalVelocityX < 0.01) goalVelocityX = 0;
    if (goalVelocityY > -0.01 and goalVelocityY < 0.01) goalVelocityY = 0;
    if (goalVelocityZ > -0.01 and goalVelocityZ < 0.01) goalVelocityZ = 0;

    if (nh.connected()) {
      ChiBot.setGoalVelocity(goalVelocityX, goalVelocityY, goalVelocityZ);
    }
    else {
      ChiBot.setGoalVelocity(0, 0, 0);
    }
  }

  if (millis() - timerVoltage > 1000) {
    timerVoltage = millis();
    voltageMsg.data = checkVoltage();
    pubVoltage.publish(&voltageMsg);
  }

  if (millis() - timerConnect > 3000) {
    timerConnect = millis();
    flagConnect = false;
  }

  digitalWrite(A4, HIGH);


  if (flagConnect) digitalWrite(A5, HIGH);
  else digitalWrite(A5, LOW);

  nh.spinOnce();
}
