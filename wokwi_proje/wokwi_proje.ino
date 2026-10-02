#include <Servo.h>

Servo J1;
Servo J2;
Servo J3;
Servo J4;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

String inputBuffer = "";

void setup() {
  Serial.begin(9600);

  J1.attach(9);
  J2.attach(6);
  J3.attach(5);
  J4.attach(3);

  J1.write(90);
  J2.write(90);
  J3.write(90);
  J4.write(90);

  Serial.println("READY");
}

void parseAndApply(String command) {
  int angles[4] = {-1, -1, -1, -1};

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
    }

    startIdx = commaIdx + 1;
  }

  if (angles[0] != -1) J1.write(angles[0]);
  if (angles[1] != -1) J2.write(angles[1]);
  if (angles[2] != -1) J3.write(angles[2]);
  if (angles[3] != -1) J4.write(angles[3]);

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