#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>
#define SENSOR_OUT PL2
volatile int move_state = MOVE_STOP;
volatile int change_speed_state = HOLD;
volatile int robot_speed = 100;
volatile int robot_turn_speed = 255;
int change_speed = 25;
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;


void move(MoveState move_state) {
    switch (move_state) {
        case MOVE_FRONT: {
            forward(robot_speed);
            break;
        }
        case MOVE_BACK: {
            backward(robot_speed);
            break;
        }
        case MOVE_CW: {
            cw(robot_turn_speed);
            break;
        }
        case MOVE_CCW: {
            ccw(robot_turn_speed);
            break;
        }
        case MOVE_STOP: {
            stop();
            break;
            
        }
    }
}

int constrain_speed(int curr_speed) {
    if (curr_speed > 255) {
        return 255;
    } else if (curr_speed < 0) {
        return 0;
    } else {
        return curr_speed;
    }
}

ChangeSpeedState change_speed_wheels(ChangeSpeedState change_speed_state) {
    robot_speed = get_new_speed(change_speed_state);
    return HOLD;
}

void set_robot_speed(int new_speed) {
    if (new_speed > 255) {
        robot_speed = 255;
    } else if (new_speed < 0) {
        robot_speed = 0;
    } else {
        robot_speed = new_speed;
    }
    
}

void set_robot_turn_speed(int new_turn_speed) {
    if (new_turn_speed > 255) {
        robot_turn_speed = 255;
    } else if (new_turn_speed < 0) {
        robot_turn_speed = 0;
    } else {
        robot_turn_speed = new_turn_speed;
    }
}

int get_new_speed(ChangeSpeedState change_speed_state) {
    int new_speed = robot_speed;
    switch (change_speed_state) {
        case INCREASE_SPEED: {
            new_speed = robot_speed + 10;
            if (new_speed > MAX_SPEED) {
                new_speed = MAX_SPEED;
            } 
            break;
        }
        case DECREASE_SPEED: {
            new_speed = robot_speed - 10;
            if (new_speed < MIN_SPEED) {
                new_speed = MIN_SPEED;
            } 
            break;
        }
    }
    return new_speed;
}
