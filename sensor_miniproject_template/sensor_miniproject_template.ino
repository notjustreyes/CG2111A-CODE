/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */
#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>
#define SENSOR_OUT PL2

#include "shared_state.h"

//volatile TState buttonState = STATE_RUNNING;
//volatile bool stateChanged = false;

//volatile int move_state = MOVE_STOP;
//volatile int change_speed_state = HOLD;
//volatile int speed = 150;



void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    cli();
    _init_estopISR();
    setupColourSensor();

    initPulseArrays();
    initPulse();
    startPulse();

    sei();

}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }

    //movement
    move(move_state);
    change_speed_state = change_speed_wheels(change_speed_state);

    //arm
    updateAllServos();


}

