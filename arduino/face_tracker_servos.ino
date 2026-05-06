/*
 * face_tracker_servos.ino
 * -------------------------------------------------------
 * Arduino pour le Face Tracking avec 2 servomoteurs.
 *
 * BRANCHEMENTS :
 *   Servo X (gauche/droite) → signal sur D9
 *   Servo Y (haut/bas)      → signal sur D10
 *   GND Arduino             → GND rail breadboard (-) ET GND alim externe
 *   Alim externe +5V        → rail rouge (+) breadboard (NE PAS utiliser le 5V Arduino)
 *
 * PROTOCOLE SERIE :
 *   Baud : 115200
 *   Format recu : "angleX,angleY\n"   ex: "95,85\n"
 *   Angles : 0 a 180 degres, centres a 90
 *
 * MOUVEMENT SMOOTH :
 *   Le servo n'est PAS teleporte directement a la cible.
 *   Il avance de STEP_DEG degres par iteration, ce qui evite les tremblements.
 *   Reduire STEP_DEG pour plus de douceur (mais plus lent).
 *   Augmenter STEP_DEG pour plus de vitesse (mais moins doux).
 * -------------------------------------------------------
 */

#include <Servo.h>

// --- Pins ---
#define PIN_SERVO_X   9    // Servo horizontal (gauche / droite)
#define PIN_SERVO_Y  10    // Servo vertical   (haut  / bas)

// --- Parametres ---
#define STEP_DEG      2    // Pas max par cycle de loop() — toucher pour ajuster la douceur
#define LOOP_DELAY_MS 12   // Delai du cycle loop en ms (~83 Hz)

// --- Objets servo ---
Servo servoX;
Servo servoY;

// --- Etat courant (position reelle) ---
float currentX = 90.0;
float currentY = 90.0;

// --- Angles cibles (recus depuis Python) ---
int targetX = 90;
int targetY = 90;

// -------------------------------------------------------
void setup() {
  Serial.begin(115200);

  servoX.attach(PIN_SERVO_X);
  servoY.attach(PIN_SERVO_Y);

  // Centrer les deux servos au demarrage
  servoX.write((int)currentX);
  servoY.write((int)currentY);

  Serial.println("FaceTracker ready");
}

// -------------------------------------------------------
void loop() {
  // === 1. Lecture des commandes serie ===
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    int commaIdx = line.indexOf(',');
    if (commaIdx > 0) {
      int rx = line.substring(0, commaIdx).toInt();
      int ry = line.substring(commaIdx + 1).toInt();

      // Contraindre entre 0° et 180° pour proteger les servos
      targetX = constrain(rx, 0, 180);
      targetY = constrain(ry, 0, 180);
    }
  }

  // === 2. Mouvement smooth vers la cible ===
  // Servo X
  if (abs(currentX - targetX) > 0.5) {
    if (currentX < targetX)
      currentX = min(currentX + STEP_DEG, (float)targetX);
    else
      currentX = max(currentX - STEP_DEG, (float)targetX);

    servoX.write((int)round(currentX));
  }

  // Servo Y
  if (abs(currentY - targetY) > 0.5) {
    if (currentY < targetY)
      currentY = min(currentY + STEP_DEG, (float)targetY);
    else
      currentY = max(currentY - STEP_DEG, (float)targetY);

    servoY.write((int)round(currentY));
  }

  // Petit delai pour ne pas saturer le microcontroleur
  delay(LOOP_DELAY_MS);
}
