#pragma once
#include <stdint.h>

enum ServoType {
  BASE=0,
  SHOULDER=1,
  ELBOW=2,
  GRIPPER=3
};

typedef struct {
  uint16_t minAngle;
  uint16_t maxAngle;
  uint16_t initAngle;
  uint16_t homeAngle;
} ServoMetadata;

typedef struct {
  volatile uint8_t *ddr;
  volatile uint8_t *port;
  uint8_t pin;
  ServoMetadata metadata;
} Servo;

extern const int NUM_SERVOS;
extern const Servo Servos[];

void changeServoAngle(int targetAngle, ServoType servoType);
void changeTargetPulseWidth(int newPulseWidth, ServoType servoType);
void updatePulseWidth(ServoType servoType);
void homeAll();
void initPulse();
void startPulse();
void updateAllServos();

const char* servoTypeToString(ServoType type);