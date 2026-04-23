#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>

const uint16_t MIN_PULSE_TICKS = 1000; //set to 2000 for simulation
const uint16_t MAX_PULSE_TICKS = 5000; //set to 4000 for simulation
const uint16_t TOTAL_TICKS = 40000;

const int NUM_SERVOS = 4;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

const int STEP_SIZE = 16;

volatile uint16_t currTicks = 0;
int msPerDeg = 10; //initial speed


const Servo Servos[NUM_SERVOS] = {
  [BASE] = {.ddr=&DDRK, .port=&PORTK, .pin=PK4, .metadata={.minAngle=0, .maxAngle=180, .initAngle=90, .homeAngle=90}},
  [SHOULDER] = {.ddr=&DDRK, .port=&PORTK, .pin=PK5, .metadata={.minAngle=0, .maxAngle=180, .initAngle=80, .homeAngle=80}},
  [ELBOW] = {.ddr=&DDRK, .port=&PORTK, .pin=PK6, .metadata={.minAngle=0, .maxAngle=180, .initAngle=90, .homeAngle=90}},
  [GRIPPER] = {.ddr=&DDRK, .port=&PORTK, .pin=PK7, .metadata={.minAngle=70, .maxAngle=80, .initAngle=80, .homeAngle=80}}
};

volatile uint16_t pulseWidths[NUM_SERVOS];
volatile uint16_t targetPulseWidths[NUM_SERVOS];
static volatile uint16_t isrPulseWidths[NUM_SERVOS]; // ISR-private shadow copy


const char* servoTypeToString(ServoType type) {
  switch (type) {
    case BASE: return "b";
    case SHOULDER: return "s";
    case ELBOW: return "e";
    case GRIPPER: return "g";
    default: return "u";
  }
}


int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

void changeServoAngle(int targetAngle, ServoType servoType) {
  targetAngle = constrain(targetAngle, Servos[servoType].metadata.minAngle, Servos[servoType].metadata.maxAngle);
  int target = angleToPulseWidth(targetAngle);
  changeTargetPulseWidth(target, servoType);
}

//gripper methods
void closeGripper() {
  changeServoAngle(Servos[GRIPPER].metadata.maxAngle, GRIPPER);
}

void openGripper() {
  changeServoAngle(Servos[GRIPPER].metadata.minAngle, GRIPPER);
}


int angleToPulseWidth(int angle) {
  return (int)(MIN_PULSE_TICKS + ((int32_t)(angle - MIN_ANGLE) * (MAX_PULSE_TICKS - MIN_PULSE_TICKS)) / (MAX_ANGLE - MIN_ANGLE));
}

void homeAll() {
  for (int i=0;i<NUM_SERVOS;i++) {
    cli();
    targetPulseWidths[i] = angleToPulseWidth(Servos[i].metadata.homeAngle);
    sei();
  }
}

void homeNoGripper() {
  cli();
  targetPulseWidths[BASE] = angleToPulseWidth(Servos[BASE].metadata.homeAngle);
  targetPulseWidths[SHOULDER] = angleToPulseWidth(Servos[SHOULDER].metadata.homeAngle);
  targetPulseWidths[ELBOW] = angleToPulseWidth(Servos[ELBOW].metadata.homeAngle);
  sei();
}

void changeTargetPulseWidth(int newPulseWidth, ServoType servoType) {
  cli();
  targetPulseWidths[servoType] = newPulseWidth;
  sei();
}

void updatePulseWidth(ServoType servoType) {
  cli();
  int32_t target = targetPulseWidths[servoType];
  int32_t curr = pulseWidths[servoType];
  sei();
  int32_t next;
  int dir;
  if (target == curr) {
    return;
  } else {
    dir = (target > curr) ? 1 : -1;
    if (abs(target - curr) < STEP_SIZE) {
      next = target;
    } else {
      next = ((int32_t)curr) + dir * STEP_SIZE;
    }
  }
  int32_t minTicks = angleToPulseWidth(Servos[servoType].metadata.minAngle);
  int32_t maxTicks = angleToPulseWidth(Servos[servoType].metadata.maxAngle);

  if (next < minTicks) next = minTicks;
  if (next > maxTicks) next = maxTicks;

  cli();
  pulseWidths[servoType] = (uint16_t) next;
  sei();
}

//methods to set pulse
void setSinglePulseLow(Servo sv) {
  *sv.port &= ~(1 << sv.pin);
}

void setSinglePulseLow(int i) {
  Servo sv = Servos[i];
  setSinglePulseLow(sv);
}

void setSinglePulseLow(ServoType servoType) {
  Servo sv = Servos[servoType];
  setSinglePulseLow(sv);
}

void setAllPulseLow() {
  for (Servo sv : Servos) {
    setSinglePulseLow(sv);
  }
}

void setSinglePulseHigh(Servo sv) {
  *sv.port |= (1 << sv.pin);
}

void setSinglePulseHigh(int i) {
  Servo sv = Servos[i];
  setSinglePulseHigh(sv);
}

void setSinglePulseHigh(ServoType servoType) {
  Servo sv = Servos[servoType];
  setSinglePulseHigh(sv);
}

void setAllPulseHigh() {
  for (Servo sv : Servos) {
    setSinglePulseHigh(sv);
  }
}

//ISR
ISR(TIMER5_COMPA_vect) {
  uint16_t currOcr = OCR5A;
  currTicks += currOcr;
  uint16_t nextAbs = TOTAL_TICKS;
  uint16_t pulseWidth;
  if (currTicks >= TOTAL_TICKS) {
    setAllPulseLow(); //safety
    currTicks = 0;


    for (int i = 0; i < NUM_SERVOS; i++) {
      isrPulseWidths[i] = pulseWidths[i];
    }

    setAllPulseHigh();
    for (int i=0;i<NUM_SERVOS;i++) {
      pulseWidth = isrPulseWidths[i]; // FIX 2: use shadow copy
      if (pulseWidth < nextAbs) {
        nextAbs = pulseWidth;
      }
    }
  } else {
    for (int i=0;i<NUM_SERVOS;i++) {
      pulseWidth = isrPulseWidths[i]; // FIX 2: use shadow copy

      if (pulseWidth <= currTicks) {
        setSinglePulseLow(i);
      } else if (pulseWidth < nextAbs) {
        nextAbs = pulseWidth;
      }
    }
  }
  int delta = nextAbs - currTicks;
  if (delta == 0) {
    delta = 1;
  }
  OCR5A = delta;
}

//initialisation methods
void initPulseArrays() {
  for (int i=0;i<NUM_SERVOS;i++) {
    pulseWidths[i] = angleToPulseWidth(Servos[i].metadata.initAngle);
    targetPulseWidths[i] = angleToPulseWidth(Servos[i].metadata.initAngle);
    isrPulseWidths[i] = pulseWidths[i]; // initialise shadow copy too
  }
}

int initOcr1aVal() {
  int val = TOTAL_TICKS;
  for (int pulseWidth : pulseWidths) {
    if (pulseWidth < val) {
      val = pulseWidth;
    }
  }
  return val;
}

void initPulse() {
  for (int i=0;i<NUM_SERVOS;i++) {
    *(Servos[i].ddr) |= (1 << Servos[i].pin);
  }
  TCNT5 = 0;
  OCR5A = initOcr1aVal();
  TCCR5A = 0b0;
  TIMSK5 = (1 << OCIE5A);
}

void startPulse() {
  setAllPulseHigh();
  TCCR5B = 0b00001010;
}

//method that moves servos
void updateAllServos() {
  for (int i=0;i<NUM_SERVOS;i++) {
    updatePulseWidth(static_cast<ServoType>(i));
  }
  delay(msPerDeg);
}
