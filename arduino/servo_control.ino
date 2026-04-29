#include <Servo.h>

Servo servoX;  // Horizontal servo (left/right)
Servo servoY;  // Vertical servo (up/down)

#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10

void setup() {
  Serial.begin(115200);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  // Center servos initially
  servoX.write(90);
  servoY.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Parse "angleX,angleY"
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      String xStr = data.substring(0, commaIndex);
      String yStr = data.substring(commaIndex + 1);

      int angleX = xStr.toInt();
      int angleY = yStr.toInt();

      // Constrain angles to 0-180
      angleX = constrain(angleX, 0, 180);
      angleY = constrain(angleY, 0, 180);

      servoX.write(angleX);
      servoY.write(angleY);

      // Optional: echo back for confirmation
      // Serial.print("Set to: ");
      // Serial.print(angleX);
      // Serial.print(",");
      // Serial.println(angleY);
    }
  }
}