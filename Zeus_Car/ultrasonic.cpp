#include "ultrasonic.h"
#include <Arduino.h>

// Get extern vars for smart pulsing
extern bool carIsMoving;
extern int16_t currentAngle;

// Zeus_car's velocity in cm/sec (approximately cuz idk)
const float carVelocity = 12.0; //TRY TO FIND THIS

// Smart pulse memory
float lastUsDistance = -1;
unsigned long lastUsCheckTime = 0;
int16_t lastUsCheckAngle = 0;

float ultrasonicRead() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // noInterrupts(); // Pause all interruptions to avoid affecting data
                     // However，turning off interrupts affects the functionality of softpwm, such as motors

  float duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
  // If no echo after 30ms, return
  if (duration == 0) {
    return -1;
  }

  float distance = duration  * 0.017; // S = vt = 340m/s * (t/2)us= (340 * 100 cm/s) * 0.5 * (t / 10^6)s = 0.017 * t
  
  // interrupts(); // resume interruptions

  if (distance > MAX_DISTANCE) {
    return -1;
  }
  return distance;
}

float getUsSmart() {
  unsigned long now = millis();

  int16_t deltaAngle = abs(currentAngle - lastUsCheckAngle);
  if (deltaAngle > 180) deltaAngle = 360 - deltaAngle;

  if (!carIsMoving) {
    // Not moving, check every 1sec or if last read was obstacle
    if (now - lastUsCheckTime > 1000 || lastUsDistance < ULTRASONIC_AVOIDANCE_THRESHOLD) {
      lastUsDistance = ultrasonicRead();
      lastUsCheckTime = now;
      lastUsCheckAngle = currentAngle;
    }
    return lastUsDistance;
  } else {
    // Moving, check when nearing boundary of last read distance
    float timePassed = (now - lastUsCheckTime) / 1000.0;
    float estTravel = timePassed * carVelocity;
    float bufferZone = 8;
    if (lastUsDistance < 0 || estTravel + bufferZone > lastUsDistance || now - lastUsCheckTime > 300 || deltaAngle > 15) {
      lastUsDistance = ultrasonicRead();
      lastUsCheckTime = now;
      lastUsCheckAngle = currentAngle;
    }
    return lastUsDistance;
  }
}

bool ultrasonicIsObstacle() {
  return getUsSmart() < ULTRASONIC_AVOIDANCE_THRESHOLD;
}

bool ultrasonicIsClear() {
  float distance = getUsSmart();
  if (distance > ULTRASONIC_AVOIDANCE_THRESHOLD || distance < 0) {
    return true;
  } else {
    return false;
  }
}