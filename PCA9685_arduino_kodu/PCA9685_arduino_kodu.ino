#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define J1_PIN 0
#define J2_PIN 1
#define J3_PIN 2
#define J4_PIN 3
#define J5_PIN 4
#define SERVOMIN  150
#define SERVOMAX  600

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

String inputBuffer = "";

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  setServoAngle(J1_PIN, 90);
  setServoAngle(J2_PIN, 90);
  setServoAngle(J3_PIN, 90);
  setServoAngle(J4_PIN, 90);
  setServoAngle(J5_PIN, 90);

  Serial.println("READY");
}

void setServoAngle(uint8_t servonum, uint16_t angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servonum, 0, pulse);
}

void parseAndApply(String command) {
  int angles[5] = {-1, -1, -1, -1, -1};

  int startIdx = 0;
  while (startIdx < command.length()) {
    int commaIdx = command.indexOf(',', startIdx);
    if (commaIdx == -1) commaIdx = command.length();

    String part = command.substring(startIdx, commaIdx);
    int colonIdx = part.indexOf(':');

    if (colonIdx != -1) {
      String label = part.substring(0, colonIdx);
      int value = part.substring(colonIdx + 1).toInt();
      value = constrain(value, MIN_ANGLE, MAX_ANGLE);

      if (label == "J1") angles[0] = value;
      else if (label == "J2") angles[1] = value;
      else if (label == "J3") angles[2] = value;
      else if (label == "J4") angles[3] = value;
      else if (label == "J5") angles[4] = value;
    }

    startIdx = commaIdx + 1;
  }
  if (angles[0] != -1) setServoAngle(J1_PIN, angles[0]);
  if (angles[1] != -1) setServoAngle(J2_PIN, angles[1]);
  if (angles[2] != -1) setServoAngle(J3_PIN, angles[2]);
  if (angles[3] != -1) setServoAngle(J4_PIN, angles[3]);
  if (angles[4] != -1) setServoAngle(J5_PIN, angles[4]);

  Serial.println("OK");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        parseAndApply(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}
