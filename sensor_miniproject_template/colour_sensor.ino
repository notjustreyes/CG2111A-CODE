#include "arm.h"
#include "movement.h"
#include "shared_state.h"
#include "packets.h"
#include "serial_driver.h"
#include <stdio.h>

// //for colour sensor
#define S0 (1<<0)  //PIN22 PA0
#define S1 (1<<1)  //PIN23 PA1
#define S2 (1<<2)  //PIN24 PA2
#define S3 (1<<3)  //PIN25 PA3


#include <avr/io.h>
#include <avr/interrupt.h>


volatile uint32_t pulseCount = 0;

// ── INT3 ISR: fires on every rising edge from TCS3200 OUT ──────
ISR(INT3_vect) {
    pulseCount++;
}


static void setupINT3(void) {
  
    DDRD  &= ~(1 << PD3);

    // Rising edge on INT3: ISC31=1, ISC30=1
    EICRA |= 0b11000000;

  
    EIMSK |= (1 << INT3);
}


static uint32_t measureChannel(uint8_t s2, uint8_t s3) {
    // Set S2
    if (s2 == HIGH) PORTA |=  S2;
    else            PORTA &= ~S2;

    // Set S3
    if (s3 == HIGH) PORTA |=  S3;
    else            PORTA &= ~S3;

    // Allow output-scaling counter inside TCS3200 to reset.
    // begins one pulse later. 10 ms settle is more than enough.

    cli();    // Reset pulse counter atomically
    pulseCount = 0;
    sei();

    unsigned long start = millis();   // Count for exactly 100 ms
    while ((millis() - start) < 100) {}

    // Atomic 32-bit read
    cli();
    uint32_t count = pulseCount;
    sei();

    // count = edges in 100 ms
    // frequency (Hz) = count / 0.1 s = count * 10
    return count * 10;
}


void readColourChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    *r = measureChannel(LOW,  LOW );   // Red:   S2=L, S3=L
    *b = measureChannel(LOW,  HIGH);   // Blue:  S2=L, S3=H
    *g = measureChannel(HIGH, HIGH);   // Green: S2=H, S3=H
}


void setupColourSensor(void) {
    // S0–S3 as outputs
    DDRA |= S0 | S1 | S2 | S3;

    PORTA |=  S0;
    PORTA &= ~S1;

    setupINT3();

  
    sei();
}
