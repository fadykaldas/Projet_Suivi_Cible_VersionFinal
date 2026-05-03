#include <Servo.h>

Servo servoX;  // Horizontal servo (left/right)
Servo servoY;  // Vertical servo (up/down)

#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10

int angleX = 90;
int angleY = 90;

void setup() {
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Center servos initially
  servoX.write(angleX);
  servoY.write(angleY);
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      return;
    }

    int commaIndex = line.indexOf(',');
    if (commaIndex < 0) {
      return;
    }

    int targetX = line.substring(0, commaIndex).toInt();
    int targetY = line.substring(commaIndex + 1).toInt();

    targetX = constrain(targetX, 0, 180);
    targetY = constrain(targetY, 0, 180);

    if (targetX != angleX) {
      angleX = targetX;
      servoX.write(angleX);
    }
    if (targetY != angleY) {
      angleY = targetY;
      servoY.write(angleY);
    }
  }
}
