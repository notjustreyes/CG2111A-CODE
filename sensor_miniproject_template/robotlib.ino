#ifndef ROBOTLIB_H
#define ROBOTLIB_H

#include <AFMotor.h>


typedef enum dir {
  STOP,
  GO,
  BACK,
  CW,
  CCW
} dir;

// Motor control
#define BACK_RIGHT 2   // M2 on the driver shield 2
#define FRONT_RIGHT 3  // M3 on the driver shield 3
#define BACK_LEFT 1    // M1 on the driver shield 1
#define FRONT_LEFT 4   // M4 on the driver shield 4

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);



void move(int speed, int direction) {


  switch (direction) {
    case BACK:
      motorFL.run(FORWARD);
      motorFR.run(FORWARD);
      motorBL.run(FORWARD);
      motorBR.run(FORWARD);

      motorFL.setSpeed(speed);
      motorFR.setSpeed(speed);
      motorBL.setSpeed(speed);
      motorBR.setSpeed(speed);
      break;
    case GO:
      motorFL.run(BACKWARD);
      motorFR.run(BACKWARD);
      motorBL.run(BACKWARD);
      motorBR.run(BACKWARD);

      motorFL.setSpeed(speed);
      motorFR.setSpeed(speed);
      motorBL.setSpeed(speed);
      motorBR.setSpeed(speed);
      break;
    case CW:
      motorFL.run(BACKWARD);
      motorFR.run(FORWARD);
      motorBL.run(BACKWARD);
      motorBR.run(FORWARD);

      motorFL.setSpeed(speed);
      motorFR.setSpeed(speed);
      motorBL.setSpeed(speed);
      motorBR.setSpeed(0);
      break;
    case CCW:
      motorFL.run(FORWARD);
      motorFR.run(BACKWARD);
      motorBL.run(FORWARD);
      motorBR.run(BACKWARD);

      motorFL.setSpeed(speed);
      motorFR.setSpeed(speed);
      motorBL.setSpeed(0);
      motorBR.setSpeed(speed);
      break;
    case STOP:
    default:
      motorFL.run(RELEASE);
      motorFR.run(RELEASE);
      motorBL.run(RELEASE);
      motorBR.run(RELEASE);
  }
}

void forward(int speed) {
  move(speed, GO);
}

void backward(int speed) {
  move(speed, BACK);
}
void ccw(int speed) {
  move(speed, CCW);
}

void cw(int speed) {
  move(speed, CW);
}

void stop() {
  move(0, STOP);
}
#endif
// // =========================
// // Encoder
// // Mega: pin 18 = INT5, pin 19 = INT4
// // =========================
// #define LEFT_ENCODER_PIN 18
// #define RIGHT_ENCODER_PIN 19

// volatile long leftEncoderTicks = 0;
// volatile long rightEncoderTicks = 0;

// void leftEncoderISR() {
//   leftEncoderTicks++;
// }

// void rightEncoderISR() {
//   rightEncoderTicks++;
// }

// void setupEncoders() {
//   pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
//   pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);
// }

// void resetEncoders() {
//   noInterrupts();
//   leftEncoderTicks = 0;
//   rightEncoderTicks = 0;
//   interrupts();
// }

// long getLeftTicks() {
//   noInterrupts();
//   long ticks = leftEncoderTicks;
//   interrupts();
//   return ticks;
// }

// long getRightTicks() {
//   noInterrupts();
//   long ticks = rightEncoderTicks;
//   interrupts();
//   return ticks;
// }

// long getAverageTicks() {
//   noInterrupts();
//   long avg = (leftEncoderTicks + rightEncoderTicks) / 2;
//   interrupts();
//   return avg;
// }

// void turnRight(int speed, float angle) {
//   long target_ticks = (long)(ticks_per_degree * angle);

//   resetEncoders();
//   cw(speed);

//   while (getLeftTicks() < target_ticks) {
//     // wait
//   }

//   stop();
// }

// void turnLeft(int speed, float angle) {
//   long target_ticks = (long)(ticks_per_degree * angle);

//   resetEncoders();
//   ccw(speed);

//   while (getLeftTicks() < target_ticks) {
//     // wait
//   }

//   stop();
// }

// #endif
