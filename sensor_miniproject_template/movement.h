#pragma once

float TICKS_PER_DEGREE = 1; //encoder

typedef enum {
    MOVE_STOP=0,
    MOVE_FRONT=1,
    MOVE_BACK=2,
    MOVE_CW=3,
    MOVE_CCW=4
} MoveState;

typedef enum {
    INCREASE_SPEED=0,
    DECREASE_SPEED=1,
    HOLD=2
} ChangeSpeedState;

