#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>
#define SENSOR_OUT PL2
volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;
#define estopPIN (1<<2) //INT2 ISR

void _init_estopISR()
 {
  DDRD &=~ estopPIN; //set as input, pin19  on the arduino
  PORTD |= estopPIN; //set to LOW
  EICRA = 0b00010000; //set to changing
  EIMSK |= 0b00000100; //activate ext INT2
 }

 
ISR(INT2_vect){

 
  static unsigned long lastTime = 0;
  unsigned long startTime = millis();
  const int THRESHOLD = 20;
  static int lastState;
  static int flag = 0; // keep track of button interactions
  
  if (startTime - lastTime > THRESHOLD){
    if (lastState != buttonState) {
      if (lastState == STATE_RUNNING) {
        flag = 1;
      }
      else {
        flag = 0;
      }

    }
  

      if(flag == 2)                             // After Pressing button twice and releasing
      {
        buttonState = STATE_RUNNING;
        lastState = buttonState;
        stateChanged = true;
        flag = 0;
      }
      else if(buttonState == STATE_RUNNING)   // Pressing for first time.
      {
        buttonState = STATE_STOPPED;
        lastState = buttonState;
        stateChanged = true;
      }
      else if (buttonState == STATE_STOPPED && flag == 0) // releasing first time
      {
        lastState = buttonState;
        flag = 1;
      }

      else if (buttonState == STATE_STOPPED && flag == 1) // Pressing for second time
      {
        lastState = buttonState;
        flag = 2;
      }
    lastTime = startTime;
  }
}
 
